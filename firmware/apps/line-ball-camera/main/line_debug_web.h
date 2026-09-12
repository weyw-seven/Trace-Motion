#ifndef LINE_DEBUG_WEB_H
#define LINE_DEBUG_WEB_H

/*
 * line_debug_web.h
 *
 * Read-only dual-vision browser observer for the ESP32-S3 line car.
 *
 * Data sources:
 *   line_tracker      -> 50 Hz control telemetry
 *   line_vision       -> line metadata + optional 120x80 algorithm snapshot
 *   colorball_vision  -> ball metadata + optional 60x40 algorithm snapshot
 *   uvc_module        -> original 480x320 MJPEG bytes for browser video
 *
 * The algorithm path and browser-video path are deliberately separate:
 *   - JPEG is decoded once by vision_shared_decode for line/ball algorithms.
 *   - The browser receives the camera's original JPEG bytes without re-encoding.
 *
 * The MJPEG stream runs on a second HTTP server so a slow video client cannot
 * block the WebSocket telemetry server. UVC frames are copied and immediately
 * released before any network send, preserving latest-wins camera behavior.
 *
 * Protocol v3 adds lightweight ball metadata while keeping algorithm image
 * messages available as an optional compile-time diagnostic mode.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LINE_DEBUG_WEB_PORT
#define LINE_DEBUG_WEB_PORT              80U
#endif

#ifndef LINE_DEBUG_WEB_MJPEG_PORT
#define LINE_DEBUG_WEB_MJPEG_PORT        81
#endif

#ifndef LINE_DEBUG_WEB_MJPEG_CTRL_PORT
#define LINE_DEBUG_WEB_MJPEG_CTRL_PORT   32769U
#endif

/* 0 = raw MJPEG + metadata only (recommended); 1 = also send algorithm images. */
#ifndef LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE
#define LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE 0
#endif

#ifndef LINE_DEBUG_WEB_TASK_STACK_SIZE
#define LINE_DEBUG_WEB_TASK_STACK_SIZE   5120U
#endif

#ifndef LINE_DEBUG_WEB_TASK_PRIORITY
#define LINE_DEBUG_WEB_TASK_PRIORITY     2U
#endif

#ifndef LINE_DEBUG_WEB_PERIOD_MS
#define LINE_DEBUG_WEB_PERIOD_MS         10U
#endif

#ifndef LINE_DEBUG_WEB_STATS_PERIOD_MS
#define LINE_DEBUG_WEB_STATS_PERIOD_MS   1000U
#endif

typedef struct
{
    bool initialized;
    bool running;
    bool client_connected;

    uint32_t telemetry_sent;
    uint32_t telemetry_dropped;

    /* Aggregate image counters retained for source compatibility. */
    uint32_t image_sent;
    uint32_t image_dropped;

    uint32_t line_image_sent;
    uint32_t line_image_dropped;
    uint32_t colorball_image_sent;
    uint32_t colorball_image_dropped;

    uint32_t stats_sent;
    uint32_t stats_dropped;
    uint32_t ws_send_errors;

    size_t max_packet_bytes;

    /* Raw camera MJPEG stream counters appended after the v2 layout. */
    uint32_t mjpeg_frames_sent;
    uint32_t mjpeg_frames_dropped;
    uint32_t mjpeg_send_errors;
    bool mjpeg_client_connected;
    size_t mjpeg_max_frame_bytes;

} line_debug_web_stats_t;

esp_err_t line_debug_web_init(void);
esp_err_t line_debug_web_start(void);
void line_debug_web_stop(void);
bool line_debug_web_is_running(void);
bool line_debug_web_client_connected(void);
void line_debug_web_get_stats(line_debug_web_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#ifdef LINE_DEBUG_WEB_IMPLEMENTATION

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "line_vision.h"
#include "colorball_vision.h"
#include "line_tracker.h"
#include "uvc_module.h"

#if !defined(CONFIG_HTTPD_WS_SUPPORT) || !CONFIG_HTTPD_WS_SUPPORT
#error "line_debug_web.h requires CONFIG_HTTPD_WS_SUPPORT=y"
#endif

static const char *LINE_DEBUG_WEB_TAG = "DEBUG_WEB";

#define LINE_DEBUG_MAGIC                    0x444CU
#define LINE_DEBUG_PROTOCOL_VERSION         3U
#define LINE_DEBUG_HEADER_BYTES             20U

#define LINE_DEBUG_LINE_CONFIG_BYTES        48U
#define LINE_DEBUG_BALL_CONFIG_BYTES        48U
#define LINE_DEBUG_TELEMETRY_BYTES          72U
#define LINE_DEBUG_STATS_BYTES              76U

#define LINE_DEBUG_LINE_IMAGE_META_BYTES    20U
#define LINE_DEBUG_BALL_IMAGE_META_BYTES    96U
#define LINE_DEBUG_BALL_META_BYTES          96U

#define LINE_DEBUG_LINE_IMAGE_BYTES \
    ((size_t)LINE_VISION_IMAGE_WIDTH * (size_t)LINE_VISION_IMAGE_HEIGHT)

#define LINE_DEBUG_BALL_RGB_BYTES \
    ((size_t)COLORBALL_VISION_IMAGE_WIDTH * \
     (size_t)COLORBALL_VISION_IMAGE_HEIGHT * 2U)

#define LINE_DEBUG_BALL_MASK_BYTES \
    ((size_t)COLORBALL_VISION_IMAGE_WIDTH * \
     (size_t)COLORBALL_VISION_IMAGE_HEIGHT)

#define LINE_DEBUG_LINE_PACKET_BYTES \
    (LINE_DEBUG_HEADER_BYTES + \
     LINE_DEBUG_LINE_IMAGE_META_BYTES + \
     LINE_DEBUG_LINE_IMAGE_BYTES)

#define LINE_DEBUG_BALL_PACKET_BYTES \
    (LINE_DEBUG_HEADER_BYTES + \
     LINE_DEBUG_BALL_IMAGE_META_BYTES + \
     LINE_DEBUG_BALL_RGB_BYTES + \
     LINE_DEBUG_BALL_MASK_BYTES)

#if LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE
#define LINE_DEBUG_MAX_PACKET_BYTES \
    ((LINE_DEBUG_LINE_PACKET_BYTES > LINE_DEBUG_BALL_PACKET_BYTES) ? \
        LINE_DEBUG_LINE_PACKET_BYTES : LINE_DEBUG_BALL_PACKET_BYTES)
#else
#define LINE_DEBUG_MAX_PACKET_BYTES \
    (LINE_DEBUG_HEADER_BYTES + LINE_DEBUG_BALL_META_BYTES)
#endif

/* 增加 HTTP 请求头缓冲区大小，解决 WebSocket 431 错误 */
#ifndef HTTPD_MAX_REQ_HDR_LEN
#define HTTPD_MAX_REQ_HDR_LEN 2048
#endif

#define LINE_DEBUG_MSG_LINE_CONFIG          1U
#define LINE_DEBUG_MSG_TELEMETRY            2U
#define LINE_DEBUG_MSG_LINE_GRAY8           3U
#define LINE_DEBUG_MSG_STATS                4U
#define LINE_DEBUG_MSG_BALL_CONFIG          5U
#define LINE_DEBUG_MSG_BALL_FRAME           6U
#define LINE_DEBUG_MSG_BALL_META            7U

#define LINE_DEBUG_FLAG_VISION_VALID        (1U << 0)
#define LINE_DEBUG_FLAG_LINE_FOUND          (1U << 1)
#define LINE_DEBUG_FLAG_AVOID_DONE          (1U << 2)
#define LINE_DEBUG_FLAG_NEW_MEAS            (1U << 3)

#define LINE_DEBUG_BALL_FLAG_VALID          (1U << 0)
#define LINE_DEBUG_BALL_FLAG_DETECTED       (1U << 1)

#define LINE_DEBUG_STR_INNER(x)              #x
#define LINE_DEBUG_STR(x)                    LINE_DEBUG_STR_INNER(x)
#define LINE_DEBUG_MJPEG_BUFFER_BYTES        ((size_t)UVC_MODULE_BUFFER_SIZE)

static bool s_line_debug_web_initialized = false;
static bool s_line_debug_web_running = false;
static httpd_handle_t s_line_debug_httpd = NULL;
static httpd_handle_t s_line_debug_mjpeg_httpd = NULL;
static TaskHandle_t s_line_debug_task_handle = NULL;

static int s_line_debug_ws_fd = -1;
static bool s_line_debug_ws_handshake_pending = false;
static bool s_line_debug_line_config_pending = false;
static bool s_line_debug_ball_config_pending = false;

static uint8_t *s_line_debug_tx_buffer = NULL;
static size_t s_line_debug_tx_len = 0U;
static uint8_t s_line_debug_tx_type = 0U;
static bool s_line_debug_tx_busy = false;

static uint32_t s_line_debug_message_sequence = 0U;
static uint32_t s_line_debug_last_line_image_sequence = UINT32_MAX;
static uint32_t s_line_debug_last_ball_image_sequence = UINT32_MAX;
static uint32_t s_line_debug_last_ball_meta_sequence = UINT32_MAX;
static uint32_t s_line_debug_last_telemetry_sequence = UINT32_MAX;
static bool s_line_debug_prefer_ball = false;

static line_debug_web_stats_t s_line_debug_stats = {0};
static portMUX_TYPE s_line_debug_lock = portMUX_INITIALIZER_UNLOCKED;

static const char s_line_debug_html[] =
    "<!doctype html>\n"
    "<html lang=\"en\">\n"
    "<head>\n"
    "<meta charset=\"utf-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
    "<title>ESP32 Dual Vision Debug</title>\n"
    "<style>\n"
    ":root{color-scheme:dark;--bg:#101216;--panel:#181c22;--muted:#8e99a8;--text:#edf2f7;--line:#2b3440;--ok:#57d38c;--warn:#f1c75b;--bad:#f26b6b;--accent:#69a7ff;--cyan:#65d9e8}\n"
    "*{box-sizing:border-box}\n"
    "body{margin:0;background:var(--bg);color:var(--text);font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}\n"
    "header{padding:14px 18px;border-bottom:1px solid var(--line);display:flex;gap:12px;align-items:center;flex-wrap:wrap;position:sticky;top:0;background:rgba(16,18,22,.96);z-index:5}\n"
    ".brand{font-weight:800;letter-spacing:.06em}.pill{padding:4px 8px;border:1px solid var(--line);border-radius:999px;font-size:12px}.online{color:var(--ok)}.offline{color:var(--bad)}\n"
    "main{padding:14px;display:grid;grid-template-columns:1fr 1fr;gap:14px}\n"
    ".panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;min-width:0}\n"
    ".title{font-size:12px;color:var(--muted);letter-spacing:.08em;margin-bottom:9px}\n"
    ".camera{position:relative;aspect-ratio:3/2;background:#000;overflow:hidden;border-radius:7px}\n"
    ".camera canvas,.camera img{position:absolute;inset:0;width:100%;height:100%}\n"
    ".camera img{object-fit:fill;background:#000}\n"
    ".camera canvas{image-rendering:pixelated}\n"
    ".camera .overlay{pointer-events:none}\n"
    ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}\n"
    ".metric{border-bottom:1px solid var(--line);padding:7px 0}.metric .k{color:var(--muted);font-size:11px}.metric .v{font-size:17px;margin-top:3px}\n"
    ".small{font-size:12px;color:var(--muted);line-height:1.45}\n"
    ".wide{grid-column:1/-1}.graph{height:250px}.graph canvas{width:100%;height:100%;display:block}\n"
    ".stats{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px}.stat{padding:8px;background:#11151a;border-radius:6px}.stat b{display:block;font-size:16px}.stat span{font-size:10px;color:var(--muted)}\n"
    "@media(max-width:950px){main{grid-template-columns:1fr}.wide{grid-column:auto}.stats{grid-template-columns:repeat(2,1fr)}}\n"
    "</style>\n"
    "</head>\n"
    "<body>\n"
    "<header>\n"
    "  <div class=\"brand\">ESP32 SINGLE-DECODE VISION DEBUG</div>\n"
    "  <div id=\"conn\" class=\"pill offline\">OFFLINE</div>\n"
    "  <div class=\"pill\">MJPEG <span id=\"mjpegState\">CONNECTING</span></div>\n"
    "  <div class=\"pill\">LINE <span id=\"lineHz\">0.0</span> Hz</div>\n"
    "  <div class=\"pill\">BALL <span id=\"ballHz\">0.0</span> Hz</div>\n"
    "  <div class=\"pill\">TELEM <span id=\"teleHz\">0.0</span> Hz</div>\n"
    "  <div class=\"pill\">RENDER <span id=\"renderHz\">0.0</span> Hz</div>\n"
    "</header>\n"
    "<main>\n"
    "  <section class=\"panel wide\">\n"
    "    <div class=\"title\">RAW UVC MJPEG - ORIGINAL 480x320 CAMERA JPEG + BROWSER OVERLAY</div>\n"
    "    <div class=\"camera\">\n"
    "      <img id=\"mjpeg\" alt=\"MJPEG stream\"><canvas id=\"mjpegOverlay\" class=\"overlay\"></canvas>\n"
    "    </div>\n"
    "    <div class=\"small\" style=\"margin-top:8px\">No JPEG re-encode on ESP32. Green = ROIs; red = line/ball detections; cyan = ball center/radius; yellow = commanded angular velocity.</div>\n"
    "  </section>\n"
    "\n"
    "  <section class=\"panel algoPanel\">\n"
    "    <div class=\"title\">LINE VISION - EXACT 120x80 GRAYSCALE USED BY TRACKER</div>\n"
    "    <div class=\"camera\">\n"
    "      <canvas id=\"lineImage\"></canvas><canvas id=\"lineOverlay\" class=\"overlay\"></canvas>\n"
    "    </div>\n"
    "    <div class=\"small\" style=\"margin-top:8px\">Green = line ROI; blue = ROI center; red = line centroid; yellow arrow = commanded angular velocity.</div>\n"
    "  </section>\n"
    "\n"
    "  <section class=\"panel algoPanel\">\n"
    "    <div class=\"title\">COLOR BALL VISION - 60x40 BOX-FILTERED RGB565 + TARGET MASK</div>\n"
    "    <div class=\"camera\">\n"
    "      <canvas id=\"ballImage\"></canvas><canvas id=\"ballOverlay\" class=\"overlay\"></canvas>\n"
    "    </div>\n"
    "    <div class=\"small\" style=\"margin-top:8px\">Yellow tint = HSV mask; green = detector ROI; red = selected component box; cyan = center/radius.</div>\n"
    "  </section>\n"
    "\n"
    "  <section class=\"panel\">\n"
    "    <div class=\"title\">LINE TRACKER / CONTROL</div>\n"
    "    <div class=\"grid\">\n"
    "      <div class=\"metric\"><div class=\"k\">STATE</div><div class=\"v\" id=\"state\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">LINE VISION</div><div class=\"v\" id=\"valid\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">ERROR</div><div class=\"v\" id=\"error\">+0.000</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">D FILTERED</div><div class=\"v\" id=\"deriv\">+0.000</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">VX</div><div class=\"v\" id=\"vx\">+0.000</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">W</div><div class=\"v\" id=\"w\">+0.000</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">LINE CENTER X</div><div class=\"v\" id=\"lineCenter\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">BLACK RATIO</div><div class=\"v\" id=\"black\">0.0%</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">LINE SEQ</div><div class=\"v\" id=\"lineSeq\">0</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">LINE AGE</div><div class=\"v\" id=\"lineAge\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">VISION DT</div><div class=\"v\" id=\"vdt\">0 ms</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">NEW FRAME</div><div class=\"v\" id=\"newFrame\">NO</div></div>\n"
    "    </div>\n"
    "  </section>\n"
    "\n"
    "  <section class=\"panel\">\n"
    "    <div class=\"title\">COLOR BALL DETECTOR</div>\n"
    "    <div class=\"grid\">\n"
    "      <div class=\"metric\"><div class=\"k\">STATUS</div><div class=\"v\" id=\"ballStatus\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">CONFIDENCE</div><div class=\"v\" id=\"ballConfidence\">0.00</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">CENTER X / Y</div><div class=\"v\" id=\"ballCenter\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">NORMALIZED X / Y</div><div class=\"v\" id=\"ballNorm\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">RADIUS</div><div class=\"v\" id=\"ballRadius\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">FILL RATIO</div><div class=\"v\" id=\"ballFill\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">MEAN HUE</div><div class=\"v\" id=\"ballHue\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">BALL PIXELS</div><div class=\"v\" id=\"ballPixels\">0</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">BALL SEQ</div><div class=\"v\" id=\"ballSeq\">0</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">BALL AGE</div><div class=\"v\" id=\"ballAge\">-</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">DECODE</div><div class=\"v\" id=\"ballDecode\">0 us</div></div>\n"
    "      <div class=\"metric\"><div class=\"k\">PROCESS</div><div class=\"v\" id=\"ballProcess\">0 us</div></div>\n"
    "    </div>\n"
    "  </section>\n"
    "\n"
    "  <section class=\"panel wide graph\">\n"
    "    <div class=\"title\">10 SECOND HISTORY - LINE ERROR / W / VX</div>\n"
    "    <canvas id=\"graph\"></canvas>\n"
    "  </section>\n"
    "\n"
    "  <section class=\"panel wide\">\n"
    "    <div class=\"title\">PIPELINE / DROP COUNTERS</div>\n"
    "    <div class=\"stats\">\n"
    "      <div class=\"stat\"><b id=\"uvcAccepted\">0</b><span>UVC accepted</span></div>\n"
    "      <div class=\"stat\"><b id=\"uvcBusy\">0</b><span>UVC busy drop</span></div>\n"
    "      <div class=\"stat\"><b id=\"uvcSlot\">0</b><span>UVC no-slot drop</span></div>\n"
    "      <div class=\"stat\"><b id=\"uvcOver\">0</b><span>UVC oversize</span></div>\n"
    "      <div class=\"stat\"><b id=\"webTeleSent\">0</b><span>telemetry sent</span></div>\n"
    "      <div class=\"stat\"><b id=\"webTeleDrop\">0</b><span>telemetry drop</span></div>\n"
    "      <div class=\"stat\"><b id=\"lineImgSent\">0</b><span>line image sent</span></div>\n"
    "      <div class=\"stat\"><b id=\"lineImgDrop\">0</b><span>line image drop</span></div>\n"
    "      <div class=\"stat\"><b id=\"ballImgSent\">0</b><span>ball frame sent</span></div>\n"
    "      <div class=\"stat\"><b id=\"ballImgDrop\">0</b><span>ball frame drop</span></div>\n"
    "      <div class=\"stat\"><b id=\"mjpegSent\">0</b><span>MJPEG frames sent</span></div>\n"
    "      <div class=\"stat\"><b id=\"mjpegDrop\">0</b><span>MJPEG frame drop</span></div>\n"
    "      <div class=\"stat\"><b id=\"mjpegErr\">0</b><span>MJPEG send error</span></div>\n"
    "    </div>\n"
    "  </section>\n"
    "</main>\n"
    "<script>\n"
    "'use strict';\n"
    "const $=id=>document.getElementById(id);\n"
    "const MAGIC=0x444c, VERSION=3, HEADER=20;\n"
    "const TYPE_LINE_CFG=1, TYPE_TELEM=2, TYPE_LINE_GRAY=3, TYPE_STATS=4, TYPE_BALL_CFG=5, TYPE_BALL_FRAME=6, TYPE_BALL_META=7;\n"
    "const MJPEG_PORT=parseInt('" LINE_DEBUG_STR(LINE_DEBUG_WEB_MJPEG_PORT) "',10);\n"
    "const SHOW_ALGO=(parseInt('" LINE_DEBUG_STR(LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE) "',10)!==0);\n"
    "const FLAG_VALID=1<<0, FLAG_FOUND=1<<1, FLAG_AVOID=1<<2, FLAG_NEW=1<<3;\n"
    "const BALL_VALID=1<<0, BALL_FOUND=1<<1;\n"
    "const states=['SEARCH','FOLLOW','LOST','FINISH_CHECK','FINISHED','STOPPED'];\n"
    "\n"
    "const mjpeg=$('mjpeg'), mjpegOverlay=$('mjpegOverlay'), mjpegOverlayCtx=mjpegOverlay.getContext('2d');\n"
    "const lineImage=$('lineImage'), lineCtx=lineImage.getContext('2d',{alpha:false});\n"
    "const lineOverlay=$('lineOverlay'), lineOverlayCtx=lineOverlay.getContext('2d');\n"
    "const ballImage=$('ballImage'), ballCtx=ballImage.getContext('2d',{alpha:false});\n"
    "const ballOverlay=$('ballOverlay'), ballOverlayCtx=ballOverlay.getContext('2d');\n"
    "const graph=$('graph'), graphCtx=graph.getContext('2d');\n"
    "\n"
    "const lineOff=document.createElement('canvas'), lineOffCtx=lineOff.getContext('2d',{alpha:false});\n"
    "const ballOff=document.createElement('canvas'), ballOffCtx=ballOff.getContext('2d',{alpha:false});\n"
    "const maskOff=document.createElement('canvas'), maskOffCtx=maskOff.getContext('2d');\n"
    "\n"
    "let lineCfg=null, ballCfg=null, tele=null, ball=null, stats=null, ws=null;\n"
    "let history=[];\n"
    "let teleCount=0,lineCount=0,ballCount=0,renderCount=0;\n"
    "let lastLineRateSeq=-1,lastBallRateSeq=-1;\n"
    "let latestEspTimeUs=0,lastRateTime=performance.now();\n"
    "\n"
    "function u64(v,o){return v.getUint32(o,true)+v.getUint32(o+4,true)*4294967296;}\n"
    "function f(v,o){return v.getFloat32(o,true);}\n"
    "function signed(x,n=3){return (x>=0?'+':'')+x.toFixed(n);}\n"
    "function resizeCanvas(c){const r=c.getBoundingClientRect(),d=devicePixelRatio||1,w=Math.max(1,Math.round(r.width*d)),h=Math.max(1,Math.round(r.height*d));if(c.width!==w||c.height!==h){c.width=w;c.height=h;return true}return false}\n"
    "\n"
    "function drawLineImage(){if(!lineOff.width)return;resizeCanvas(lineImage);lineCtx.imageSmoothingEnabled=false;lineCtx.drawImage(lineOff,0,0,lineImage.width,lineImage.height)}\n"
    "function drawBallImage(){if(!ballOff.width)return;resizeCanvas(ballImage);ballCtx.imageSmoothingEnabled=false;ballCtx.drawImage(ballOff,0,0,ballImage.width,ballImage.height)}\n"
    "\n"
    "function parseLineConfig(v,o){\n"
    "  lineCfg={cameraW:v.getUint16(o,true),cameraH:v.getUint16(o+2,true),w:v.getUint16(o+4,true),h:v.getUint16(o+6,true),\n"
    "  r0:v.getUint16(o+8,true),r1:v.getUint16(o+10,true),c0:v.getUint16(o+12,true),c1:v.getUint16(o+14,true),\n"
    "  threshold:v.getUint8(o+16),cameraFps:v.getUint8(o+17),controlMs:v.getUint16(o+18,true),kp:f(v,o+20),kd:f(v,o+24),\n"
    "  alpha:f(v,o+28),wMax:f(v,o+32),vMax:f(v,o+36),vMin:f(v,o+40),mirror:v.getUint8(o+44)!==0,staleMs:v.getUint16(o+46,true)};\n"
    "  mjpeg.style.transform=lineCfg.mirror?'scaleX(-1)':'none';\n"
    "}\n"
    "function parseBallConfig(v,o){\n"
    "  ballCfg={cameraW:v.getUint16(o,true),cameraH:v.getUint16(o+2,true),w:v.getUint16(o+4,true),h:v.getUint16(o+6,true),\n"
    "  r0:v.getUint16(o+8,true),r1:v.getUint16(o+10,true),c0:v.getUint16(o+12,true),c1:v.getUint16(o+14,true),\n"
    "  hue:v.getUint16(o+16,true),hueTol:v.getUint16(o+18,true),sat:v.getUint8(o+20),val:v.getUint8(o+21),minPixels:v.getUint16(o+22,true),\n"
    "  minAspect:f(v,o+24),maxAspect:f(v,o+28),minFill:f(v,o+32),minRadius:f(v,o+36),maxRadius:f(v,o+40),\n"
    "  staleMs:v.getUint16(o+44,true),mirror:v.getUint8(o+46)!==0,cameraFps:v.getUint8(o+47)};\n"
    "}\n"
    "function parseTele(v,o,t){\n"
    "  const flags=v.getUint8(o+9);\n"
    "  tele={t,controlSeq:v.getUint32(o,true),visionSeq:v.getUint32(o+4,true),state:v.getUint8(o+8),\n"
    "  valid:(flags&FLAG_VALID)!==0,found:(flags&FLAG_FOUND)!==0,avoid:(flags&FLAG_AVOID)!==0,isNew:(flags&FLAG_NEW)!==0,\n"
    "  error:f(v,o+12),d:f(v,o+16),vx:f(v,o+20),w:f(v,o+24),cx:f(v,o+28),roiCx:f(v,o+32),roiHalf:f(v,o+36),\n"
    "  black:f(v,o+40),vdt:f(v,o+44),blackPx:v.getUint32(o+48,true),roiPx:v.getUint32(o+52,true),\n"
    "  frameTs:u64(v,o+56),updateTs:u64(v,o+64)};\n"
    "  if(tele.visionSeq!==lastLineRateSeq){lastLineRateSeq=tele.visionSeq;lineCount++;}\n"
    "  history.push({t,error:tele.error,w:tele.w,vx:tele.vx});\n"
    "  const cutoff=t-10e6;while(history.length&&history[0].t<cutoff)history.shift();teleCount++;\n"
    "}\n"
    "function parseLineGray(buf,v,o){\n"
    "  const w=v.getUint16(o,true),h=v.getUint16(o+2,true),stride=v.getUint16(o+4,true),format=v.getUint8(o+6);\n"
    "  if(format!==1)return;\n"
    "  const pixelOffset=o+20;if(buf.byteLength<pixelOffset+stride*h)return;\n"
    "  lineOff.width=w;lineOff.height=h;\n"
    "  const out=lineOffCtx.createImageData(w,h),src=new Uint8Array(buf,pixelOffset,stride*h);\n"
    "  for(let y=0;y<h;y++)for(let x=0;x<w;x++){const g=src[y*stride+x],j=(y*w+x)*4;out.data[j]=g;out.data[j+1]=g;out.data[j+2]=g;out.data[j+3]=255}\n"
    "  lineOffCtx.putImageData(out,0,0);drawLineImage();\n"
    "}\n"
    "function parseBallMeta(v,o){\n"
    "  const flags=v.getUint8(o+7);\n"
    "  ball={valid:(flags&BALL_VALID)!==0,found:(flags&BALL_FOUND)!==0,seq:v.getUint32(o+8,true),frameTs:u64(v,o+12),updateTs:u64(v,o+20),\n"
    "    cx:f(v,o+28),cy:f(v,o+32),nx:f(v,o+36),ny:f(v,o+40),radius:f(v,o+44),fill:f(v,o+48),confidence:f(v,o+52),\n"
    "    hue:f(v,o+56),sat:f(v,o+60),val:f(v,o+64),x0:v.getUint16(o+68,true),y0:v.getUint16(o+70,true),x1:v.getUint16(o+72,true),y1:v.getUint16(o+74,true),\n"
    "    pixels:v.getUint32(o+76,true),thresholdPixels:v.getUint32(o+80,true),roiPixels:v.getUint32(o+84,true),\n"
    "    decodeUs:v.getUint32(o+88,true),processUs:v.getUint32(o+92,true)};\n"
    "  if(ball.seq!==lastBallRateSeq){lastBallRateSeq=ball.seq;ballCount++;}\n"
    "}\n"
    "function parseBallFrame(buf,v,o){\n"
    "  const w=v.getUint16(o,true),h=v.getUint16(o+2,true),stride=v.getUint16(o+4,true),format=v.getUint8(o+6);\n"
    "  if(format!==2)return;parseBallMeta(v,o);ball.w=w;ball.h=h;\n"
    "  const rgbOffset=o+96,rgbBytes=stride*h,maskOffset=rgbOffset+rgbBytes,maskBytes=w*h;\n"
    "  if(buf.byteLength<maskOffset+maskBytes)return;\n"
    "  const src=new Uint8Array(buf,rgbOffset,rgbBytes);\n"
    "  ballOff.width=w;ballOff.height=h;\n"
    "  const out=ballOffCtx.createImageData(w,h);\n"
    "  for(let y=0;y<h;y++)for(let x=0;x<w;x++){\n"
    "    const k=y*stride+x*2,p=src[k]|(src[k+1]<<8),j=(y*w+x)*4;\n"
    "    out.data[j]=Math.round(((p>>11)&31)*255/31);out.data[j+1]=Math.round(((p>>5)&63)*255/63);out.data[j+2]=Math.round((p&31)*255/31);out.data[j+3]=255;\n"
    "  }\n"
    "  ballOffCtx.putImageData(out,0,0);\n"
    "  const msrc=new Uint8Array(buf,maskOffset,maskBytes);maskOff.width=w;maskOff.height=h;\n"
    "  const mout=maskOffCtx.createImageData(w,h);\n"
    "  for(let i=0;i<maskBytes;i++){const j=i*4,a=msrc[i]?100:0;mout.data[j]=255;mout.data[j+1]=210;mout.data[j+2]=40;mout.data[j+3]=a}\n"
    "  maskOffCtx.putImageData(mout,0,0);drawBallImage();\n"
    "}\n"
    "function parseStats(v,o){\n"
    "  stats={accepted:v.getUint32(o,true),busy:v.getUint32(o+4,true),over:v.getUint32(o+8,true),non:v.getUint32(o+12,true),\n"
    "  slot:v.getUint32(o+16,true),max:v.getUint32(o+20,true),teleSent:v.getUint32(o+24,true),teleDrop:v.getUint32(o+28,true),\n"
    "  lineSent:v.getUint32(o+32,true),lineDrop:v.getUint32(o+36,true),ballSent:v.getUint32(o+40,true),ballDrop:v.getUint32(o+44,true),\n"
    "  statsSent:v.getUint32(o+48,true),statsDrop:v.getUint32(o+52,true),wsErr:v.getUint32(o+56,true),\n"
    "  mjpegSent:v.getUint32(o+60,true),mjpegDrop:v.getUint32(o+64,true),mjpegErr:v.getUint32(o+68,true),mjpegActive:v.getUint32(o+72,true)!==0};\n"
    "}\n"
    "function onPacket(buf){\n"
    "  const v=new DataView(buf);if(v.byteLength<HEADER||v.getUint16(0,true)!==MAGIC||v.getUint8(2)!==VERSION)return;\n"
    "  const type=v.getUint8(3),len=v.getUint16(4,true);if(v.byteLength<HEADER+len)return;latestEspTimeUs=u64(v,12);\n"
    "  if(type===TYPE_LINE_CFG)parseLineConfig(v,HEADER);else if(type===TYPE_TELEM)parseTele(v,HEADER,latestEspTimeUs);\n"
    "  else if(type===TYPE_LINE_GRAY)parseLineGray(buf,v,HEADER);else if(type===TYPE_STATS)parseStats(v,HEADER);\n"
    "  else if(type===TYPE_BALL_CFG)parseBallConfig(v,HEADER);else if(type===TYPE_BALL_FRAME)parseBallFrame(buf,v,HEADER);\n"
    "  else if(type===TYPE_BALL_META)parseBallMeta(v,HEADER);\n"
    "}\n"
    "function connect(){\n"
    "  $('conn').textContent='CONNECTING';$('conn').className='pill offline';\n"
    "  const proto=location.protocol==='https:'?'wss':'ws';ws=new WebSocket(`${proto}://${location.host}/ws`);ws.binaryType='arraybuffer';\n"
    "  ws.onopen=()=>{$('conn').textContent='ONLINE';$('conn').className='pill online'};\n"
    "  ws.onmessage=e=>onPacket(e.data);\n"
    "  ws.onclose=e=>{$('conn').textContent=`OFFLINE ${e.code}`;$('conn').className='pill offline';setTimeout(connect,1000)};\n"
    "  ws.onerror=()=>ws.close();\n"
    "}\n"
    "function updateCards(){\n"
    "  if(tele){\n"
    "    $('state').textContent=states[tele.state]||String(tele.state);\n"
    "    $('valid').textContent=tele.valid?(tele.found?'FOUND':'VALID / NO LINE'):'STALE';\n"
    "    $('error').textContent=signed(tele.error);$('deriv').textContent=signed(tele.d);$('vx').textContent=signed(tele.vx);$('w').textContent=signed(tele.w);\n"
    "    $('lineCenter').textContent=tele.found?tele.cx.toFixed(2):'-';$('black').textContent=(tele.black*100).toFixed(1)+'%';\n"
    "    $('lineSeq').textContent=tele.visionSeq;$('newFrame').textContent=tele.isNew?'YES':'NO';$('vdt').textContent=(tele.vdt*1000).toFixed(1)+' ms';\n"
    "    $('lineAge').textContent=tele.updateTs>0?Math.max(0,(latestEspTimeUs-tele.updateTs)/1000).toFixed(0)+' ms':'-';\n"
    "  }\n"
    "  if(ball){\n"
    "    $('ballStatus').textContent=ball.valid?(ball.found?'FOUND':'VALID / NO BALL'):'STALE';\n"
    "    $('ballConfidence').textContent=ball.confidence.toFixed(2);\n"
    "    $('ballCenter').textContent=ball.found?`${ball.cx.toFixed(1)} / ${ball.cy.toFixed(1)}`:'-';\n"
    "    $('ballNorm').textContent=ball.found?`${signed(ball.nx,2)} / ${signed(ball.ny,2)}`:'-';\n"
    "    $('ballRadius').textContent=ball.found?ball.radius.toFixed(2)+' px':'-';$('ballFill').textContent=ball.found?(ball.fill*100).toFixed(1)+'%':'-';\n"
    "    $('ballHue').textContent=ball.found?ball.hue.toFixed(0)+' deg':'-';$('ballPixels').textContent=ball.pixels;$('ballSeq').textContent=ball.seq;\n"
    "    $('ballAge').textContent=ball.updateTs>0?Math.max(0,(latestEspTimeUs-ball.updateTs)/1000).toFixed(0)+' ms':'-';\n"
    "    $('ballDecode').textContent=ball.decodeUs+' us';$('ballProcess').textContent=ball.processUs+' us';\n"
    "  }\n"
    "  if(stats){\n"
    "    $('uvcAccepted').textContent=stats.accepted;$('uvcBusy').textContent=stats.busy;$('uvcSlot').textContent=stats.slot;$('uvcOver').textContent=stats.over;\n"
    "    $('webTeleSent').textContent=stats.teleSent;$('webTeleDrop').textContent=stats.teleDrop;\n"
    "    $('lineImgSent').textContent=stats.lineSent;$('lineImgDrop').textContent=stats.lineDrop;\n"
    "    $('ballImgSent').textContent=stats.ballSent;$('ballImgDrop').textContent=stats.ballDrop;\n"
    "    $('mjpegSent').textContent=stats.mjpegSent;$('mjpegDrop').textContent=stats.mjpegDrop;$('mjpegErr').textContent=stats.mjpegErr;\n"
    "    $('mjpegState').textContent=stats.mjpegActive?'STREAMING':'READY';\n"
    "  }\n"
    "}\n"
    "setInterval(updateCards,20);\n"
    "\n"
    "function drawMjpegOverlay(){\n"
    "  resizeCanvas(mjpegOverlay);const c=mjpegOverlayCtx,w=mjpegOverlay.width,h=mjpegOverlay.height;c.clearRect(0,0,w,h);c.lineWidth=Math.max(2,devicePixelRatio||1);\n"
    "  if(lineCfg){const sx=w/lineCfg.w,sy=h/lineCfg.h;c.strokeStyle='#57d38c';c.strokeRect(lineCfg.c0*sx,lineCfg.r0*sy,(lineCfg.c1-lineCfg.c0+1)*sx,(lineCfg.r1-lineCfg.r0+1)*sy);\n"
    "    const rcx=(lineCfg.c0+lineCfg.c1)*.5;c.strokeStyle='#69a7ff';c.beginPath();c.moveTo(rcx*sx,lineCfg.r0*sy);c.lineTo(rcx*sx,(lineCfg.r1+1)*sy);c.stroke();\n"
    "    if(tele&&tele.found){c.strokeStyle='#f26b6b';c.beginPath();c.moveTo(tele.cx*sx,lineCfg.r0*sy);c.lineTo(tele.cx*sx,(lineCfg.r1+1)*sy);c.stroke();}}\n"
    "  if(ballCfg){const sx=w/ballCfg.w,sy=h/ballCfg.h;c.strokeStyle='#57d38c';c.strokeRect(ballCfg.c0*sx,ballCfg.r0*sy,(ballCfg.c1-ballCfg.c0+1)*sx,(ballCfg.r1-ballCfg.r0+1)*sy);\n"
    "    if(ball&&ball.found){c.strokeStyle='#f26b6b';c.strokeRect(ball.x0*sx,ball.y0*sy,(ball.x1-ball.x0+1)*sx,(ball.y1-ball.y0+1)*sy);\n"
    "      const cx=ball.cx*sx,cy=ball.cy*sy,r=ball.radius*(sx+sy)*.5;c.strokeStyle='#65d9e8';c.beginPath();c.arc(cx,cy,Math.max(2,r),0,Math.PI*2);c.stroke();c.beginPath();c.moveTo(cx-8,cy);c.lineTo(cx+8,cy);c.moveTo(cx,cy-8);c.lineTo(cx,cy+8);c.stroke();}}\n"
    "  if(tele){const x=w*.5,y=h*.86,len=Math.min(w,h)*.18,ang=tele.w*Math.PI*.35;c.strokeStyle='#f1c75b';c.fillStyle='#f1c75b';c.lineWidth=3*(devicePixelRatio||1);\n"
    "    const x2=x+Math.sin(ang)*len,y2=y-Math.cos(ang)*len;c.beginPath();c.moveTo(x,y);c.lineTo(x2,y2);c.stroke();const a=Math.atan2(y2-y,x2-x);c.beginPath();c.moveTo(x2,y2);c.lineTo(x2-12*Math.cos(a-.45),y2-12*Math.sin(a-.45));c.lineTo(x2-12*Math.cos(a+.45),y2-12*Math.sin(a+.45));c.closePath();c.fill();}\n"
    "}\n"
    "function drawLineOverlay(){\n"
    "  resizeCanvas(lineOverlay);const c=lineOverlayCtx,w=lineOverlay.width,h=lineOverlay.height;c.clearRect(0,0,w,h);if(!lineCfg)return;\n"
    "  const sx=w/lineCfg.w,sy=h/lineCfg.h;c.lineWidth=Math.max(2,devicePixelRatio||1);\n"
    "  c.strokeStyle='#57d38c';c.strokeRect(lineCfg.c0*sx,lineCfg.r0*sy,(lineCfg.c1-lineCfg.c0+1)*sx,(lineCfg.r1-lineCfg.r0+1)*sy);\n"
    "  const roiCx=(lineCfg.c0+lineCfg.c1)*.5;c.strokeStyle='#69a7ff';c.beginPath();c.moveTo(roiCx*sx,lineCfg.r0*sy);c.lineTo(roiCx*sx,(lineCfg.r1+1)*sy);c.stroke();\n"
    "  if(tele&&tele.found){c.strokeStyle='#f26b6b';c.beginPath();c.moveTo(tele.cx*sx,lineCfg.r0*sy);c.lineTo(tele.cx*sx,(lineCfg.r1+1)*sy);c.stroke()}\n"
    "  if(tele){const x=w*.5,y=h*.86,len=Math.min(w,h)*.18,ang=tele.w*Math.PI*.35;c.strokeStyle='#f1c75b';c.fillStyle='#f1c75b';c.lineWidth=3*(devicePixelRatio||1);\n"
    "    const x2=x+Math.sin(ang)*len,y2=y-Math.cos(ang)*len;c.beginPath();c.moveTo(x,y);c.lineTo(x2,y2);c.stroke();\n"
    "    const a=Math.atan2(y2-y,x2-x);c.beginPath();c.moveTo(x2,y2);c.lineTo(x2-12*Math.cos(a-.45),y2-12*Math.sin(a-.45));c.lineTo(x2-12*Math.cos(a+.45),y2-12*Math.sin(a+.45));c.closePath();c.fill()}\n"
    "}\n"
    "function drawBallOverlay(){\n"
    "  resizeCanvas(ballOverlay);const c=ballOverlayCtx,w=ballOverlay.width,h=ballOverlay.height;c.clearRect(0,0,w,h);if(!ballCfg)return;\n"
    "  c.imageSmoothingEnabled=false;if(maskOff.width)c.drawImage(maskOff,0,0,w,h);\n"
    "  const sx=w/ballCfg.w,sy=h/ballCfg.h;c.lineWidth=Math.max(2,devicePixelRatio||1);\n"
    "  c.strokeStyle='#57d38c';c.strokeRect(ballCfg.c0*sx,ballCfg.r0*sy,(ballCfg.c1-ballCfg.c0+1)*sx,(ballCfg.r1-ballCfg.r0+1)*sy);\n"
    "  if(ball&&ball.found){\n"
    "    c.strokeStyle='#f26b6b';c.strokeRect(ball.x0*sx,ball.y0*sy,(ball.x1-ball.x0+1)*sx,(ball.y1-ball.y0+1)*sy);\n"
    "    const cx=ball.cx*sx,cy=ball.cy*sy,r=ball.radius*(sx+sy)*.5;c.strokeStyle='#65d9e8';c.beginPath();c.arc(cx,cy,Math.max(2,r),0,Math.PI*2);c.stroke();\n"
    "    c.beginPath();c.moveTo(cx-8,cy);c.lineTo(cx+8,cy);c.moveTo(cx,cy-8);c.lineTo(cx,cy+8);c.stroke();\n"
    "  }\n"
    "}\n"
    "function drawGraph(){\n"
    "  resizeCanvas(graph);const c=graphCtx,w=graph.width,h=graph.height;c.clearRect(0,0,w,h);c.fillStyle='#181c22';c.fillRect(0,0,w,h);\n"
    "  c.strokeStyle='#2b3440';c.lineWidth=1;c.beginPath();c.moveTo(0,h/2);c.lineTo(w,h/2);c.stroke();if(!history.length)return;\n"
    "  const newest=history[history.length-1].t,oldest=newest-10e6;\n"
    "  const line=(key,color)=>{c.strokeStyle=color;c.lineWidth=2*(devicePixelRatio||1);c.beginPath();let first=true;for(const p of history){const x=(p.t-oldest)/1e7*w,y=h/2-p[key]*(h*.42);if(first){c.moveTo(x,y);first=false}else c.lineTo(x,y)}c.stroke()};\n"
    "  line('error','#69a7ff');line('w','#f26b6b');line('vx','#57d38c');\n"
    "  c.fillStyle='#8e99a8';c.font=`${11*(devicePixelRatio||1)}px ui-monospace`;c.fillText('error',10,18*(devicePixelRatio||1));c.fillText('w',70*(devicePixelRatio||1),18*(devicePixelRatio||1));c.fillText('vx',95*(devicePixelRatio||1),18*(devicePixelRatio||1));\n"
    "}\n"
    "function render(){renderCount++;drawMjpegOverlay();if(SHOW_ALGO){drawLineOverlay();drawBallOverlay();}drawGraph();requestAnimationFrame(render)}\n"
    "requestAnimationFrame(render);\n"
    "setInterval(()=>{const now=performance.now(),dt=(now-lastRateTime)/1000;\n"
    "  $('lineHz').textContent=(lineCount/dt).toFixed(1);$('ballHz').textContent=(ballCount/dt).toFixed(1);$('teleHz').textContent=(teleCount/dt).toFixed(1);$('renderHz').textContent=(renderCount/dt).toFixed(1);\n"
    "  lineCount=ballCount=teleCount=renderCount=0;lastRateTime=now},1000);\n"
    "addEventListener('resize',()=>{if(SHOW_ALGO){drawLineImage();drawBallImage();}});\n"
    "if(!SHOW_ALGO)document.querySelectorAll('.algoPanel').forEach(e=>e.style.display='none');\n"
    "function startMjpeg(){mjpeg.src=`http://${location.hostname}:${MJPEG_PORT}/stream?ts=${Date.now()}`;$('mjpegState').textContent='CONNECTING';mjpeg.onerror=()=>{$('mjpegState').textContent='RETRY';setTimeout(()=>{mjpeg.src='';startMjpeg()},1000)};mjpeg.onload=()=>{$('mjpegState').textContent='STREAMING'};}\n"
    "startMjpeg();connect();\n"
    "</script>\n"
    "</body>\n"
    "</html>\n"
    ;

static void line_debug_put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void line_debug_put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void line_debug_put_u64(uint8_t *p, uint64_t value)
{
    for (unsigned i = 0U; i < 8U; ++i)
    {
        p[i] = (uint8_t)(value >> (8U * i));
    }
}

static void line_debug_put_float(uint8_t *p, float value)
{
    uint32_t raw = 0U;
    memcpy(&raw, &value, sizeof(raw));
    line_debug_put_u32(p, raw);
}

static void *line_debug_alloc_packet_buffer(size_t size)
{
    void *p = NULL;

#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif

    if (p == NULL)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }

    return p;
}

static bool line_debug_client_is_live(void)
{
    bool live = false;
    int fd = -1;
    httpd_handle_t httpd = NULL;

    taskENTER_CRITICAL(&s_line_debug_lock);
    fd = s_line_debug_ws_fd;
    httpd = s_line_debug_httpd;
    taskEXIT_CRITICAL(&s_line_debug_lock);

    if ((httpd != NULL) && (fd >= 0))
    {
        const httpd_ws_client_info_t info =
            httpd_ws_get_fd_info(httpd, fd);

        if (info == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            live = true;

            taskENTER_CRITICAL(&s_line_debug_lock);
            s_line_debug_ws_handshake_pending = false;
            taskEXIT_CRITICAL(&s_line_debug_lock);
        }
        else if (info == HTTPD_WS_CLIENT_HTTP)
        {
            bool pending;
            taskENTER_CRITICAL(&s_line_debug_lock);
            pending = s_line_debug_ws_handshake_pending;
            taskEXIT_CRITICAL(&s_line_debug_lock);

            if (pending)
            {
                return false;
            }
        }
    }

    if (!live)
    {
        taskENTER_CRITICAL(&s_line_debug_lock);
        if (s_line_debug_ws_fd == fd)
        {
            s_line_debug_ws_fd = -1;
            s_line_debug_ws_handshake_pending = false;
            s_line_debug_line_config_pending = false;
            s_line_debug_ball_config_pending = false;
        }
        taskEXIT_CRITICAL(&s_line_debug_lock);
    }

    return live;
}

static bool line_debug_try_begin_tx(void)
{
    bool acquired = false;

    taskENTER_CRITICAL(&s_line_debug_lock);
    if (!s_line_debug_tx_busy &&
        (s_line_debug_ws_fd >= 0) &&
        s_line_debug_web_running)
    {
        s_line_debug_tx_busy = true;
        acquired = true;
    }
    taskEXIT_CRITICAL(&s_line_debug_lock);

    return acquired;
}

static void line_debug_release_tx(void)
{
    taskENTER_CRITICAL(&s_line_debug_lock);
    s_line_debug_tx_busy = false;
    taskEXIT_CRITICAL(&s_line_debug_lock);
}

static void line_debug_count_drop(uint8_t type)
{
    taskENTER_CRITICAL(&s_line_debug_lock);

    if (type == LINE_DEBUG_MSG_TELEMETRY)
    {
        ++s_line_debug_stats.telemetry_dropped;
    }
    else if (type == LINE_DEBUG_MSG_LINE_GRAY8)
    {
        ++s_line_debug_stats.image_dropped;
        ++s_line_debug_stats.line_image_dropped;
    }
    else if (type == LINE_DEBUG_MSG_BALL_FRAME)
    {
        ++s_line_debug_stats.image_dropped;
        ++s_line_debug_stats.colorball_image_dropped;
    }
    else if (type == LINE_DEBUG_MSG_STATS)
    {
        ++s_line_debug_stats.stats_dropped;
    }

    taskEXIT_CRITICAL(&s_line_debug_lock);
}

static void line_debug_finish_header(
    uint8_t type,
    uint16_t payload_bytes,
    uint16_t flags)
{
    ++s_line_debug_message_sequence;

    line_debug_put_u16(s_line_debug_tx_buffer + 0U, LINE_DEBUG_MAGIC);
    s_line_debug_tx_buffer[2] = LINE_DEBUG_PROTOCOL_VERSION;
    s_line_debug_tx_buffer[3] = type;
    line_debug_put_u16(s_line_debug_tx_buffer + 4U, payload_bytes);
    line_debug_put_u16(s_line_debug_tx_buffer + 6U, flags);
    line_debug_put_u32(s_line_debug_tx_buffer + 8U, s_line_debug_message_sequence);
    line_debug_put_u64(
        s_line_debug_tx_buffer + 12U,
        (uint64_t)esp_timer_get_time());

    s_line_debug_tx_type = type;
    s_line_debug_tx_len = LINE_DEBUG_HEADER_BYTES + (size_t)payload_bytes;
}

static void line_debug_httpd_send_work(void *arg)
{
    (void)arg;

    int fd;
    httpd_handle_t httpd;
    uint8_t type;

    taskENTER_CRITICAL(&s_line_debug_lock);
    fd = s_line_debug_ws_fd;
    httpd = s_line_debug_httpd;
    type = s_line_debug_tx_type;
    taskEXIT_CRITICAL(&s_line_debug_lock);

    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if ((httpd != NULL) &&
        (fd >= 0) &&
        (httpd_ws_get_fd_info(httpd, fd) == HTTPD_WS_CLIENT_WEBSOCKET))
    {
        httpd_ws_frame_t frame = {0};
        frame.final = true;
        frame.fragmented = false;
        frame.type = HTTPD_WS_TYPE_BINARY;
        frame.payload = s_line_debug_tx_buffer;
        frame.len = s_line_debug_tx_len;

        ret = httpd_ws_send_frame_async(httpd, fd, &frame);
    }

    taskENTER_CRITICAL(&s_line_debug_lock);

    if (ret == ESP_OK)
    {
        if (type == LINE_DEBUG_MSG_TELEMETRY)
        {
            ++s_line_debug_stats.telemetry_sent;
        }
        else if (type == LINE_DEBUG_MSG_LINE_GRAY8)
        {
            ++s_line_debug_stats.image_sent;
            ++s_line_debug_stats.line_image_sent;
        }
        else if (type == LINE_DEBUG_MSG_BALL_FRAME)
        {
            ++s_line_debug_stats.image_sent;
            ++s_line_debug_stats.colorball_image_sent;
        }
        else if (type == LINE_DEBUG_MSG_STATS)
        {
            ++s_line_debug_stats.stats_sent;
        }
    }
    else
    {
        ++s_line_debug_stats.ws_send_errors;

        if (s_line_debug_ws_fd == fd)
        {
            s_line_debug_ws_fd = -1;
            s_line_debug_ws_handshake_pending = false;
            s_line_debug_line_config_pending = false;
            s_line_debug_ball_config_pending = false;
        }
    }

    s_line_debug_tx_busy = false;
    taskEXIT_CRITICAL(&s_line_debug_lock);
}

static bool line_debug_queue_current_packet(void)
{
    if (s_line_debug_httpd == NULL)
    {
        line_debug_release_tx();
        return false;
    }

    const esp_err_t ret =
        httpd_queue_work(
            s_line_debug_httpd,
            line_debug_httpd_send_work,
            NULL);

    if (ret != ESP_OK)
    {
        const uint8_t type = s_line_debug_tx_type;
        line_debug_release_tx();
        line_debug_count_drop(type);
        return false;
    }

    return true;
}

static bool line_debug_send_line_config(void)
{
    if (!line_debug_try_begin_tx())
    {
        return false;
    }

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    size_t o = 0U;

    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_CAMERA_WIDTH); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_CAMERA_HEIGHT); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_IMAGE_WIDTH); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_IMAGE_HEIGHT); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_SCAN_START_ROW); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_SCAN_END_ROW); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_SCAN_START_COL); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_SCAN_END_COL); o += 2U;
    p[o++] = (uint8_t)LINE_VISION_THRESHOLD;
    p[o++] = (uint8_t)LINE_VISION_CAMERA_FPS;
    line_debug_put_u16(p + o, (uint16_t)LINE_CONTROL_PERIOD_MS); o += 2U;
    line_debug_put_float(p + o, LINE_KP); o += 4U;
    line_debug_put_float(p + o, LINE_KD); o += 4U;
    line_debug_put_float(p + o, LINE_D_ALPHA); o += 4U;
    line_debug_put_float(p + o, LINE_W_MAX); o += 4U;
    line_debug_put_float(p + o, LINE_V_MAX); o += 4U;
    line_debug_put_float(p + o, LINE_V_MIN); o += 4U;
    p[o++] = IMAGE_DECODE_MIRROR_X ? 1U : 0U;
    p[o++] = 0U;
    line_debug_put_u16(p + o, (uint16_t)LINE_VISION_STALE_MS); o += 2U;

    if (o != LINE_DEBUG_LINE_CONFIG_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_LINE_CONFIG,
        (uint16_t)o,
        0U);

    return line_debug_queue_current_packet();
}

static bool line_debug_send_ball_config(void)
{
    if (!line_debug_try_begin_tx())
    {
        return false;
    }

    colorball_vision_config_t cfg;
    colorball_vision_get_config(&cfg);

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    size_t o = 0U;

    line_debug_put_u16(p + o, (uint16_t)COLORBALL_VISION_CAMERA_WIDTH); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)COLORBALL_VISION_CAMERA_HEIGHT); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)COLORBALL_VISION_IMAGE_WIDTH); o += 2U;
    line_debug_put_u16(p + o, (uint16_t)COLORBALL_VISION_IMAGE_HEIGHT); o += 2U;

    line_debug_put_u16(p + o, cfg.start_row); o += 2U;
    line_debug_put_u16(p + o, cfg.end_row); o += 2U;
    line_debug_put_u16(p + o, cfg.start_col); o += 2U;
    line_debug_put_u16(p + o, cfg.end_col); o += 2U;

    line_debug_put_u16(p + o, cfg.hue_center_deg); o += 2U;
    line_debug_put_u16(p + o, cfg.hue_tolerance_deg); o += 2U;
    p[o++] = cfg.saturation_min;
    p[o++] = cfg.value_min;
    line_debug_put_u16(p + o, cfg.min_pixels); o += 2U;

    line_debug_put_float(p + o, cfg.min_aspect); o += 4U;
    line_debug_put_float(p + o, cfg.max_aspect); o += 4U;
    line_debug_put_float(p + o, cfg.min_fill_ratio); o += 4U;
    line_debug_put_float(p + o, cfg.min_radius_px); o += 4U;
    line_debug_put_float(p + o, cfg.max_radius_px); o += 4U;

    line_debug_put_u16(p + o, (uint16_t)COLORBALL_VISION_STALE_MS); o += 2U;
    p[o++] = IMAGE_DECODE_MIRROR_X ? 1U : 0U;
    p[o++] = (uint8_t)COLORBALL_VISION_CAMERA_FPS;

    if (o != LINE_DEBUG_BALL_CONFIG_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_BALL_CONFIG,
        (uint16_t)o,
        0U);

    return line_debug_queue_current_packet();
}

static bool line_debug_send_telemetry(const line_tracker_status_t *s)
{
    if ((s == NULL) || !line_debug_try_begin_tx())
    {
        return false;
    }

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    size_t o = 0U;

    line_debug_put_u32(p + o, s->control_sequence); o += 4U;
    line_debug_put_u32(p + o, s->vision_sequence); o += 4U;
    p[o++] = (uint8_t)s->state;

    uint8_t flags = 0U;
    if (s->vision_valid) flags |= LINE_DEBUG_FLAG_VISION_VALID;
    if (s->line_found) flags |= LINE_DEBUG_FLAG_LINE_FOUND;
    if (s->avoid_done) flags |= LINE_DEBUG_FLAG_AVOID_DONE;
    if (s->measurement_new) flags |= LINE_DEBUG_FLAG_NEW_MEAS;

    p[o++] = flags;
    p[o++] = 0U;
    p[o++] = 0U;

    line_debug_put_float(p + o, s->error); o += 4U;
    line_debug_put_float(p + o, s->d_filtered); o += 4U;
    line_debug_put_float(p + o, s->target_vx); o += 4U;
    line_debug_put_float(p + o, s->target_w); o += 4U;
    line_debug_put_float(p + o, s->vision_center_x); o += 4U;
    line_debug_put_float(p + o, s->vision_roi_center_x); o += 4U;
    line_debug_put_float(p + o, s->vision_roi_half_width); o += 4U;
    line_debug_put_float(p + o, s->vision_black_ratio); o += 4U;
    line_debug_put_float(p + o, s->vision_dt_s); o += 4U;
    line_debug_put_u32(p + o, s->vision_black_pixels); o += 4U;
    line_debug_put_u32(p + o, s->vision_roi_pixels); o += 4U;
    line_debug_put_u64(p + o, (uint64_t)s->vision_frame_timestamp_us); o += 8U;
    line_debug_put_u64(p + o, (uint64_t)s->vision_update_timestamp_us); o += 8U;

    if (o != LINE_DEBUG_TELEMETRY_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_TELEMETRY,
        (uint16_t)o,
        0U);

    return line_debug_queue_current_packet();
}

#if LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE
static bool line_debug_send_line_image(void)
{
    if (!line_debug_try_begin_tx())
    {
        return false;
    }

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    uint8_t *pixels = p + LINE_DEBUG_LINE_IMAGE_META_BYTES;

    grayscale_image_t image = {
        .data = pixels,
        .capacity_bytes = LINE_DEBUG_LINE_IMAGE_BYTES,
        .width = (uint16_t)LINE_VISION_IMAGE_WIDTH,
        .height = (uint16_t)LINE_VISION_IMAGE_HEIGHT,
    };

    line_vision_data_t vision;

    if (!line_vision_try_get_grayscale_snapshot(&image, &vision))
    {
        line_debug_release_tx();
        line_debug_count_drop(LINE_DEBUG_MSG_LINE_GRAY8);
        return false;
    }

    if (vision.sequence == s_line_debug_last_line_image_sequence)
    {
        line_debug_release_tx();
        return false;
    }

    size_t o = 0U;
    line_debug_put_u16(p + o, image.width); o += 2U;
    line_debug_put_u16(p + o, image.height); o += 2U;
    line_debug_put_u16(p + o, image.width); o += 2U;
    p[o++] = 1U; /* GRAY8 */
    p[o++] = 0U;
    line_debug_put_u32(p + o, vision.sequence); o += 4U;
    line_debug_put_u64(p + o, (uint64_t)vision.frame_timestamp_us); o += 8U;

    if (o != LINE_DEBUG_LINE_IMAGE_META_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    const size_t payload_bytes =
        LINE_DEBUG_LINE_IMAGE_META_BYTES +
        LINE_DEBUG_LINE_IMAGE_BYTES;

    if (payload_bytes > UINT16_MAX)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_LINE_GRAY8,
        (uint16_t)payload_bytes,
        0U);

    if (!line_debug_queue_current_packet())
    {
        return false;
    }

    s_line_debug_last_line_image_sequence = vision.sequence;
    return true;
}

#endif /* LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE */

static size_t line_debug_pack_ball_metadata(
    uint8_t *p,
    uint16_t width,
    uint16_t height,
    uint16_t stride_bytes,
    uint8_t format,
    const colorball_vision_data_t *vision)
{
    if ((p == NULL) || (vision == NULL))
    {
        return 0U;
    }

    size_t o = 0U;
    line_debug_put_u16(p + o, width); o += 2U;
    line_debug_put_u16(p + o, height); o += 2U;
    line_debug_put_u16(p + o, stride_bytes); o += 2U;
    p[o++] = format;

    uint8_t flags = 0U;
    if (vision->valid) flags |= LINE_DEBUG_BALL_FLAG_VALID;
    if (vision->ball_detected) flags |= LINE_DEBUG_BALL_FLAG_DETECTED;
    p[o++] = flags;

    line_debug_put_u32(p + o, vision->sequence); o += 4U;
    line_debug_put_u64(p + o, (uint64_t)vision->frame_timestamp_us); o += 8U;
    line_debug_put_u64(p + o, (uint64_t)vision->update_timestamp_us); o += 8U;

    line_debug_put_float(p + o, vision->center_x); o += 4U;
    line_debug_put_float(p + o, vision->center_y); o += 4U;
    line_debug_put_float(p + o, vision->normalized_x); o += 4U;
    line_debug_put_float(p + o, vision->normalized_y); o += 4U;
    line_debug_put_float(p + o, vision->radius_px); o += 4U;
    line_debug_put_float(p + o, vision->fill_ratio); o += 4U;
    line_debug_put_float(p + o, vision->confidence); o += 4U;
    line_debug_put_float(p + o, vision->mean_hue_deg); o += 4U;
    line_debug_put_float(p + o, vision->mean_saturation); o += 4U;
    line_debug_put_float(p + o, vision->mean_value); o += 4U;

    line_debug_put_u16(p + o, vision->bbox_start_x); o += 2U;
    line_debug_put_u16(p + o, vision->bbox_start_y); o += 2U;
    line_debug_put_u16(p + o, vision->bbox_end_x); o += 2U;
    line_debug_put_u16(p + o, vision->bbox_end_y); o += 2U;

    line_debug_put_u32(p + o, vision->ball_pixels); o += 4U;
    line_debug_put_u32(p + o, vision->threshold_pixels); o += 4U;
    line_debug_put_u32(p + o, vision->roi_pixels); o += 4U;
    line_debug_put_u32(p + o, vision->decode_time_us); o += 4U;
    line_debug_put_u32(p + o, vision->process_time_us); o += 4U;

    return o;
}

static bool line_debug_send_ball_meta(void)
{
    colorball_vision_data_t vision;
    if (!colorball_vision_try_get_data(&vision) ||
        (vision.sequence == s_line_debug_last_ball_meta_sequence))
    {
        return false;
    }

    if (!line_debug_try_begin_tx())
    {
        return false;
    }

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    const size_t o = line_debug_pack_ball_metadata(
        p,
        (uint16_t)COLORBALL_VISION_IMAGE_WIDTH,
        (uint16_t)COLORBALL_VISION_IMAGE_HEIGHT,
        0U,
        0U,
        &vision);

    if (o != LINE_DEBUG_BALL_META_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_BALL_META,
        (uint16_t)o,
        0U);

    if (!line_debug_queue_current_packet())
    {
        return false;
    }

    s_line_debug_last_ball_meta_sequence = vision.sequence;
    return true;
}

#if LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE
static bool line_debug_send_ball_frame(void)
{
    if (!line_debug_try_begin_tx())
    {
        return false;
    }

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    uint8_t *rgb = p + LINE_DEBUG_BALL_IMAGE_META_BYTES;
    uint8_t *mask = rgb + LINE_DEBUG_BALL_RGB_BYTES;

    colorball_rgb565_image_t image = {
        .data = rgb,
        .capacity_bytes = LINE_DEBUG_BALL_RGB_BYTES,
        .width = (uint16_t)COLORBALL_VISION_IMAGE_WIDTH,
        .height = (uint16_t)COLORBALL_VISION_IMAGE_HEIGHT,
        .stride_bytes = (uint16_t)(COLORBALL_VISION_IMAGE_WIDTH * 2U),
    };

    colorball_vision_data_t vision;

    if (!colorball_vision_try_get_debug_snapshot(
            &image,
            mask,
            LINE_DEBUG_BALL_MASK_BYTES,
            &vision))
    {
        line_debug_release_tx();
        line_debug_count_drop(LINE_DEBUG_MSG_BALL_FRAME);
        return false;
    }

    if (vision.sequence == s_line_debug_last_ball_image_sequence)
    {
        line_debug_release_tx();
        return false;
    }

    const size_t o = line_debug_pack_ball_metadata(
        p,
        image.width,
        image.height,
        image.stride_bytes,
        2U, /* RGB565 little-endian */
        &vision);

    if (o != LINE_DEBUG_BALL_IMAGE_META_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    const size_t payload_bytes =
        LINE_DEBUG_BALL_IMAGE_META_BYTES +
        LINE_DEBUG_BALL_RGB_BYTES +
        LINE_DEBUG_BALL_MASK_BYTES;

    if (payload_bytes > UINT16_MAX)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_BALL_FRAME,
        (uint16_t)payload_bytes,
        0U);

    if (!line_debug_queue_current_packet())
    {
        return false;
    }

    s_line_debug_last_ball_image_sequence = vision.sequence;
    return true;
}

#endif /* LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE */

static bool line_debug_send_stats(void)
{
    if (!line_debug_try_begin_tx())
    {
        return false;
    }

    uvc_module_stats_t uvc = {0};
    uvc_module_get_stats(&uvc);

    line_debug_web_stats_t web_stats;
    line_debug_web_get_stats(&web_stats);

    uint8_t *p = s_line_debug_tx_buffer + LINE_DEBUG_HEADER_BYTES;
    size_t o = 0U;

    line_debug_put_u32(p + o, uvc.accepted_frames); o += 4U;
    line_debug_put_u32(p + o, uvc.dropped_busy); o += 4U;
    line_debug_put_u32(p + o, uvc.dropped_oversize); o += 4U;
    line_debug_put_u32(p + o, uvc.dropped_non_mjpeg); o += 4U;
    line_debug_put_u32(p + o, uvc.dropped_no_free_slot); o += 4U;
    line_debug_put_u32(p + o, (uint32_t)uvc.max_frame_len); o += 4U;

    line_debug_put_u32(p + o, web_stats.telemetry_sent); o += 4U;
    line_debug_put_u32(p + o, web_stats.telemetry_dropped); o += 4U;
    line_debug_put_u32(p + o, web_stats.line_image_sent); o += 4U;
    line_debug_put_u32(p + o, web_stats.line_image_dropped); o += 4U;
    line_debug_put_u32(p + o, web_stats.colorball_image_sent); o += 4U;
    line_debug_put_u32(p + o, web_stats.colorball_image_dropped); o += 4U;
    line_debug_put_u32(p + o, web_stats.stats_sent); o += 4U;
    line_debug_put_u32(p + o, web_stats.stats_dropped); o += 4U;
    line_debug_put_u32(p + o, web_stats.ws_send_errors); o += 4U;
    line_debug_put_u32(p + o, web_stats.mjpeg_frames_sent); o += 4U;
    line_debug_put_u32(p + o, web_stats.mjpeg_frames_dropped); o += 4U;
    line_debug_put_u32(p + o, web_stats.mjpeg_send_errors); o += 4U;
    line_debug_put_u32(p + o, web_stats.mjpeg_client_connected ? 1U : 0U); o += 4U;

    if (o != LINE_DEBUG_STATS_BYTES)
    {
        line_debug_release_tx();
        return false;
    }

    line_debug_finish_header(
        LINE_DEBUG_MSG_STATS,
        (uint16_t)o,
        0U);

    return line_debug_queue_current_packet();
}

static void line_debug_set_mjpeg_client(bool connected)
{
    taskENTER_CRITICAL(&s_line_debug_lock);
    s_line_debug_stats.mjpeg_client_connected = connected;
    taskEXIT_CRITICAL(&s_line_debug_lock);
}

static esp_err_t line_debug_mjpeg_handler(httpd_req_t *req)
{
    if (req == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Allocate the JPEG copy buffer only while a browser is actually viewing. */
    uint8_t *mjpeg_buffer =
        (uint8_t *)line_debug_alloc_packet_buffer(LINE_DEBUG_MJPEG_BUFFER_BYTES);

    if (mjpeg_buffer == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(
        req,
        "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    uint32_t last_sequence = UINT32_MAX;
    esp_err_t result = ESP_OK;
    line_debug_set_mjpeg_client(true);

    ESP_LOGI(LINE_DEBUG_WEB_TAG, "MJPEG client connected");

    while (s_line_debug_web_running)
    {
        uvc_module_frame_t frame = {0};
        const esp_err_t frame_ret =
            uvc_module_acquire_latest_frame(last_sequence, &frame);

        if (frame_ret == ESP_OK)
        {
            const uint32_t sequence = frame.sequence;
            const size_t frame_len = frame.len;

            if (frame_len > LINE_DEBUG_MJPEG_BUFFER_BYTES)
            {
                uvc_module_release_frame(&frame);
                last_sequence = sequence;

                taskENTER_CRITICAL(&s_line_debug_lock);
                ++s_line_debug_stats.mjpeg_frames_dropped;
                taskEXIT_CRITICAL(&s_line_debug_lock);
                continue;
            }

            /* Never keep a camera slot pinned while TCP/Wi-Fi may block. */
            memcpy(mjpeg_buffer, frame.data, frame_len);
            uvc_module_release_frame(&frame);
            last_sequence = sequence;

            char part_header[128];
            const int header_len = snprintf(
                part_header,
                sizeof(part_header),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nX-Sequence: %u\r\n\r\n",
                (unsigned)frame_len,
                (unsigned)sequence);

            if ((header_len <= 0) ||
                ((size_t)header_len >= sizeof(part_header)))
            {
                result = ESP_FAIL;
                break;
            }

            result = httpd_resp_send_chunk(
                req,
                part_header,
                (ssize_t)header_len);

            if (result == ESP_OK)
            {
                result = httpd_resp_send_chunk(
                    req,
                    (const char *)mjpeg_buffer,
                    (ssize_t)frame_len);
            }

            if (result == ESP_OK)
            {
                result = httpd_resp_send_chunk(req, "\r\n", 2);
            }

            if (result != ESP_OK)
            {
                taskENTER_CRITICAL(&s_line_debug_lock);
                ++s_line_debug_stats.mjpeg_send_errors;
                taskEXIT_CRITICAL(&s_line_debug_lock);
                break;
            }

            taskENTER_CRITICAL(&s_line_debug_lock);
            ++s_line_debug_stats.mjpeg_frames_sent;
            if (frame_len > s_line_debug_stats.mjpeg_max_frame_bytes)
            {
                s_line_debug_stats.mjpeg_max_frame_bytes = frame_len;
            }
            taskEXIT_CRITICAL(&s_line_debug_lock);
        }
        else if ((frame_ret == ESP_ERR_NOT_FOUND) ||
                 (frame_ret == ESP_ERR_INVALID_STATE) ||
                 (frame_ret == ESP_ERR_TIMEOUT))
        {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        else
        {
            taskENTER_CRITICAL(&s_line_debug_lock);
            ++s_line_debug_stats.mjpeg_frames_dropped;
            taskEXIT_CRITICAL(&s_line_debug_lock);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    line_debug_set_mjpeg_client(false);
    free(mjpeg_buffer);
    ESP_LOGI(LINE_DEBUG_WEB_TAG, "MJPEG client disconnected");
    return result;
}

static esp_err_t line_debug_root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, s_line_debug_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t line_debug_ws_handler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET)
    {
        taskENTER_CRITICAL(&s_line_debug_lock);
        s_line_debug_ws_fd = fd;
        s_line_debug_ws_handshake_pending = true;
        s_line_debug_line_config_pending = true;
        s_line_debug_ball_config_pending = true;
        s_line_debug_last_line_image_sequence = UINT32_MAX;
        s_line_debug_last_ball_image_sequence = UINT32_MAX;
        s_line_debug_last_ball_meta_sequence = UINT32_MAX;
        s_line_debug_last_telemetry_sequence = UINT32_MAX;
        s_line_debug_prefer_ball = false;
        taskEXIT_CRITICAL(&s_line_debug_lock);

        ESP_LOGI(LINE_DEBUG_WEB_TAG, "WebSocket client connected fd=%d", fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0U);

    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t small_payload[32];

    if (frame.len > 0U)
    {
        if (frame.len > sizeof(small_payload))
        {
            return ESP_OK;
        }

        frame.payload = small_payload;
        ret = httpd_ws_recv_frame(req, &frame, sizeof(small_payload));

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE)
    {
        taskENTER_CRITICAL(&s_line_debug_lock);

        if (s_line_debug_ws_fd == fd)
        {
            s_line_debug_ws_fd = -1;
            s_line_debug_ws_handshake_pending = false;
            s_line_debug_line_config_pending = false;
            s_line_debug_ball_config_pending = false;
        }

        taskEXIT_CRITICAL(&s_line_debug_lock);
        ESP_LOGI(LINE_DEBUG_WEB_TAG, "WebSocket client closed fd=%d", fd);
    }

    return ESP_OK;
}

#if LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE
static bool line_debug_try_send_pending_images(
    const line_tracker_status_t *tracker)
{
    const bool line_pending =
        (tracker != NULL) &&
        (tracker->vision_update_timestamp_us > 0) &&
        (tracker->vision_sequence != s_line_debug_last_line_image_sequence);

    colorball_vision_data_t ball_meta;
    const bool have_ball_meta =
        colorball_vision_try_get_data(&ball_meta);

    const bool ball_pending =
        have_ball_meta &&
        (ball_meta.sequence != s_line_debug_last_ball_image_sequence);

    if (!line_pending && !ball_pending)
    {
        return false;
    }

    if (line_pending && ball_pending)
    {
        bool sent = false;

        if (s_line_debug_prefer_ball)
        {
            sent = line_debug_send_ball_frame();
            if (!sent)
            {
                sent = line_debug_send_line_image();
            }
        }
        else
        {
            sent = line_debug_send_line_image();
            if (!sent)
            {
                sent = line_debug_send_ball_frame();
            }
        }

        if (sent)
        {
            s_line_debug_prefer_ball = !s_line_debug_prefer_ball;
        }

        return sent;
    }

    if (ball_pending)
    {
        const bool sent = line_debug_send_ball_frame();
        if (sent) s_line_debug_prefer_ball = false;
        return sent;
    }

    const bool sent = line_debug_send_line_image();
    if (sent) s_line_debug_prefer_ball = true;
    return sent;
}

#endif /* LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE */

static void line_debug_web_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LINE_DEBUG_WEB_PERIOD_MS);
    uint32_t stats_elapsed_ms = 0U;

    while (s_line_debug_web_running)
    {
        if (line_debug_client_is_live())
        {
            bool line_config_pending;
            bool ball_config_pending;
            bool tx_busy;

            taskENTER_CRITICAL(&s_line_debug_lock);
            line_config_pending = s_line_debug_line_config_pending;
            ball_config_pending = s_line_debug_ball_config_pending;
            tx_busy = s_line_debug_tx_busy;
            taskEXIT_CRITICAL(&s_line_debug_lock);

            if (!tx_busy)
            {
                if (line_config_pending)
                {
                    if (line_debug_send_line_config())
                    {
                        taskENTER_CRITICAL(&s_line_debug_lock);
                        s_line_debug_line_config_pending = false;
                        taskEXIT_CRITICAL(&s_line_debug_lock);
                    }
                }
                else if (ball_config_pending)
                {
                    if (line_debug_send_ball_config())
                    {
                        taskENTER_CRITICAL(&s_line_debug_lock);
                        s_line_debug_ball_config_pending = false;
                        taskEXIT_CRITICAL(&s_line_debug_lock);
                    }
                }
                else
                {
                    line_tracker_status_t tracker;
                    line_tracker_get_status(&tracker);

                    /*
                     * Control telemetry has priority over debug images. A slow
                     * browser may drop pictures, but it must not make the 50 Hz
                     * control stream look artificially stale.
                     */
                    if (tracker.control_sequence !=
                        s_line_debug_last_telemetry_sequence)
                    {
                        if (line_debug_send_telemetry(&tracker))
                        {
                            s_line_debug_last_telemetry_sequence =
                                tracker.control_sequence;
                        }
                    }
                    else if (line_debug_send_ball_meta())
                    {
                        /* Lightweight observer metadata; no debug-image copy. */
                    }
#if LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE
                    else if (line_debug_try_send_pending_images(&tracker))
                    {
                        /* Optional one-image-per-pass algorithm diagnostic mode. */
                    }
#endif
                    else if (stats_elapsed_ms >= LINE_DEBUG_WEB_STATS_PERIOD_MS)
                    {
                        if (line_debug_send_stats())
                        {
                            stats_elapsed_ms = 0U;
                        }
                    }
                }
            }
        }

        stats_elapsed_ms += LINE_DEBUG_WEB_PERIOD_MS;
        vTaskDelayUntil(&last_wake, period);
    }

    s_line_debug_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t line_debug_web_init(void)
{
    if (s_line_debug_web_initialized)
    {
        return ESP_OK;
    }

    if ((LINE_DEBUG_LINE_PACKET_BYTES > UINT16_MAX + LINE_DEBUG_HEADER_BYTES) ||
        (LINE_DEBUG_BALL_PACKET_BYTES > UINT16_MAX + LINE_DEBUG_HEADER_BYTES))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    s_line_debug_tx_buffer =
        (uint8_t *)line_debug_alloc_packet_buffer(LINE_DEBUG_MAX_PACKET_BYTES);

    if (s_line_debug_tx_buffer == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_line_debug_stats, 0, sizeof(s_line_debug_stats));
    s_line_debug_stats.initialized = true;
    s_line_debug_stats.max_packet_bytes = LINE_DEBUG_MAX_PACKET_BYTES;

    s_line_debug_web_initialized = true;

    ESP_LOGI(
        LINE_DEBUG_WEB_TAG,
        "Initialized raw-MJPEG + metadata debug: ws_packet=%u lazy_mjpeg_buf=%u period=%ums algo_images=%u",
        (unsigned)LINE_DEBUG_MAX_PACKET_BYTES,
        (unsigned)LINE_DEBUG_MJPEG_BUFFER_BYTES,
        (unsigned)LINE_DEBUG_WEB_PERIOD_MS,
        (unsigned)LINE_DEBUG_WEB_ALGORITHM_IMAGES_ENABLE);

    return ESP_OK;
}

esp_err_t line_debug_web_start(void)
{
    if (!s_line_debug_web_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_line_debug_web_running)
    {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = (uint16_t)LINE_DEBUG_WEB_PORT;
    config.max_uri_handlers = 4U;
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&s_line_debug_httpd, &config);

    if (ret != ESP_OK)
    {
        s_line_debug_httpd = NULL;
        return ret;
    }

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = line_debug_root_handler,
        .user_ctx = NULL,
    };

    const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = line_debug_ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };

    ret = httpd_register_uri_handler(s_line_debug_httpd, &root_uri);

    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_line_debug_httpd, &ws_uri);
    }

    if (ret != ESP_OK)
    {
        httpd_stop(s_line_debug_httpd);
        s_line_debug_httpd = NULL;
        return ret;
    }

    httpd_config_t mjpeg_config = HTTPD_DEFAULT_CONFIG();
    mjpeg_config.server_port = (uint16_t)LINE_DEBUG_WEB_MJPEG_PORT;
    mjpeg_config.ctrl_port = (uint16_t)LINE_DEBUG_WEB_MJPEG_CTRL_PORT;
    mjpeg_config.max_uri_handlers = 2U;
    mjpeg_config.max_open_sockets = 2U;
    mjpeg_config.lru_purge_enable = true;
    mjpeg_config.send_wait_timeout = 2U;

    ret = httpd_start(&s_line_debug_mjpeg_httpd, &mjpeg_config);
    if (ret == ESP_OK)
    {
        const httpd_uri_t stream_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = line_debug_mjpeg_handler,
            .user_ctx = NULL,
        };
        ret = httpd_register_uri_handler(
            s_line_debug_mjpeg_httpd,
            &stream_uri);
    }

    if (ret != ESP_OK)
    {
        if (s_line_debug_mjpeg_httpd != NULL)
        {
            httpd_stop(s_line_debug_mjpeg_httpd);
            s_line_debug_mjpeg_httpd = NULL;
        }
        httpd_stop(s_line_debug_httpd);
        s_line_debug_httpd = NULL;
        return ret;
    }

    s_line_debug_web_running = true;

    BaseType_t task_ret =
        xTaskCreate(
            line_debug_web_task,
            "line_debug_web",
            LINE_DEBUG_WEB_TASK_STACK_SIZE,
            NULL,
            LINE_DEBUG_WEB_TASK_PRIORITY,
            &s_line_debug_task_handle);

    if (task_ret != pdPASS)
    {
        s_line_debug_web_running = false;
        if (s_line_debug_mjpeg_httpd != NULL)
        {
            httpd_stop(s_line_debug_mjpeg_httpd);
            s_line_debug_mjpeg_httpd = NULL;
        }
        httpd_stop(s_line_debug_httpd);
        s_line_debug_httpd = NULL;
        return ESP_ERR_NO_MEM;
    }

    taskENTER_CRITICAL(&s_line_debug_lock);
    s_line_debug_stats.running = true;
    taskEXIT_CRITICAL(&s_line_debug_lock);

    ESP_LOGI(
        LINE_DEBUG_WEB_TAG,
        "HTTP/WebSocket started on %u; raw MJPEG /stream started on %u",
        (unsigned)LINE_DEBUG_WEB_PORT,
        (unsigned)LINE_DEBUG_WEB_MJPEG_PORT);

    return ESP_OK;
}

void line_debug_web_stop(void)
{
    if (!s_line_debug_web_running)
    {
        return;
    }

    s_line_debug_web_running = false;

    if (s_line_debug_task_handle != NULL)
    {
        vTaskDelete(s_line_debug_task_handle);
        s_line_debug_task_handle = NULL;
    }

    taskENTER_CRITICAL(&s_line_debug_lock);
    s_line_debug_ws_fd = -1;
    s_line_debug_ws_handshake_pending = false;
    s_line_debug_line_config_pending = false;
    s_line_debug_ball_config_pending = false;
    s_line_debug_tx_busy = false;
    s_line_debug_stats.running = false;
    s_line_debug_stats.client_connected = false;
    s_line_debug_stats.mjpeg_client_connected = false;
    taskEXIT_CRITICAL(&s_line_debug_lock);

    if (s_line_debug_mjpeg_httpd != NULL)
    {
        httpd_stop(s_line_debug_mjpeg_httpd);
        s_line_debug_mjpeg_httpd = NULL;
    }

    if (s_line_debug_httpd != NULL)
    {
        httpd_stop(s_line_debug_httpd);
        s_line_debug_httpd = NULL;
    }
}

bool line_debug_web_is_running(void)
{
    return s_line_debug_web_running;
}

bool line_debug_web_client_connected(void)
{
    return line_debug_client_is_live();
}

void line_debug_web_get_stats(line_debug_web_stats_t *out_stats)
{
    if (out_stats == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&s_line_debug_lock);
    *out_stats = s_line_debug_stats;
    out_stats->initialized = s_line_debug_web_initialized;
    out_stats->running = s_line_debug_web_running;
    out_stats->client_connected =
        (s_line_debug_ws_fd >= 0) &&
        (!s_line_debug_ws_handshake_pending);
    taskEXIT_CRITICAL(&s_line_debug_lock);
}

#endif /* LINE_DEBUG_WEB_IMPLEMENTATION */
#endif /* LINE_DEBUG_WEB_H */
