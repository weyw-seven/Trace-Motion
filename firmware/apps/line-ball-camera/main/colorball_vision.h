#ifndef COLORBALL_VISION_H
#define COLORBALL_VISION_H

/*
 * colorball_vision.h
 *
 * Low-cost colored-ball detector running in parallel with line_vision.
 * It consumes the RGB565 plane from the shared single-JPEG decoder, owns only
 * its downsample/detector buffers, task, detector state, and debug snapshots.
 * It never replaces line_vision and is not part of the line_tracker control path.
 *
 * Design:
 *   - One shared TJpgDec path produces 120x80 RGB565 + GRAY8 once per MJPEG.
 *   - Ball path box-downsamples shared RGB565 2x2 -> 60x40 RGB565.
 *   - HSV threshold + largest connected component.
 *   - Publishes RGB565 + binary target mask + matching metadata.
 *   - Debug snapshot uses try-lock and never blocks the vision task.
 *
 * Single-header usage:
 *   In exactly ONE .c file:
 *       #define COLORBALL_VISION_IMPLEMENTATION
 *       #include "colorball_vision.h"
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "image_decode.h"
#include "vision_shared_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. Camera / decode                                                         */
/* ========================================================================== */

#ifndef COLORBALL_VISION_CAMERA_WIDTH
#define COLORBALL_VISION_CAMERA_WIDTH          VISION_SHARED_CAMERA_WIDTH
#endif

#ifndef COLORBALL_VISION_CAMERA_HEIGHT
#define COLORBALL_VISION_CAMERA_HEIGHT         VISION_SHARED_CAMERA_HEIGHT
#endif

#ifndef COLORBALL_VISION_CAMERA_FPS
#define COLORBALL_VISION_CAMERA_FPS            VISION_SHARED_CAMERA_FPS
#endif

/*
 * Logical output remains equivalent to 1/8 input geometry (60x40), but there
 * is no second JPEG decode. It is produced by a 2x2 box filter from shared
 * 120x80 RGB565, which reduces aliasing/noise compared with point sampling.
 */
#ifndef COLORBALL_VISION_JPEG_SCALE
#define COLORBALL_VISION_JPEG_SCALE            3U
#endif

#ifndef COLORBALL_VISION_IMAGE_WIDTH
#define COLORBALL_VISION_IMAGE_WIDTH           60U
#endif

#ifndef COLORBALL_VISION_IMAGE_HEIGHT
#define COLORBALL_VISION_IMAGE_HEIGHT          40U
#endif

#ifndef COLORBALL_VISION_POLL_MS
#define COLORBALL_VISION_POLL_MS               20U
#endif

#ifndef COLORBALL_VISION_STALE_MS
#define COLORBALL_VISION_STALE_MS              600U
#endif

#ifndef COLORBALL_VISION_TASK_STACK_SIZE
#define COLORBALL_VISION_TASK_STACK_SIZE       4096U
#endif

#ifndef COLORBALL_VISION_TASK_PRIORITY
#define COLORBALL_VISION_TASK_PRIORITY         4U
#endif

#ifndef COLORBALL_VISION_TASK_CORE
#define COLORBALL_VISION_TASK_CORE             1
#endif

/* 1 = analyze every shared decoded frame; 2 = every second frame, etc. */
#ifndef COLORBALL_VISION_FRAME_DIVIDER
#define COLORBALL_VISION_FRAME_DIVIDER         1U
#endif

#if (COLORBALL_VISION_FRAME_DIVIDER < 1)
#error "COLORBALL_VISION_FRAME_DIVIDER must be >= 1"
#endif

/* ========================================================================== */
/* 2. Default detector parameters                                             */
/* ========================================================================== */

/*
 * Default target is red.  Runtime config can change the hue without rebuild.
 * Hue is degrees [0,359].  Tolerance is circular, so red correctly wraps 359/0.
 */
#ifndef COLORBALL_VISION_HUE_CENTER_DEG
#define COLORBALL_VISION_HUE_CENTER_DEG        0U
#endif

#ifndef COLORBALL_VISION_HUE_TOLERANCE_DEG
#define COLORBALL_VISION_HUE_TOLERANCE_DEG     15U
#endif

#ifndef COLORBALL_VISION_SATURATION_MIN
#define COLORBALL_VISION_SATURATION_MIN        140U
#endif

#ifndef COLORBALL_VISION_VALUE_MIN
#define COLORBALL_VISION_VALUE_MIN             80U
#endif

#ifndef COLORBALL_VISION_MIN_PIXELS
#define COLORBALL_VISION_MIN_PIXELS            10U
#endif

#ifndef COLORBALL_VISION_MIN_ASPECT
#define COLORBALL_VISION_MIN_ASPECT            0.45f
#endif

#ifndef COLORBALL_VISION_MAX_ASPECT
#define COLORBALL_VISION_MAX_ASPECT            2.20f
#endif

#ifndef COLORBALL_VISION_MIN_FILL_RATIO
#define COLORBALL_VISION_MIN_FILL_RATIO        0.30f
#endif

#ifndef COLORBALL_VISION_MIN_RADIUS_PX
#define COLORBALL_VISION_MIN_RADIUS_PX         2.0f
#endif

#ifndef COLORBALL_VISION_MAX_RADIUS_PX
#define COLORBALL_VISION_MAX_RADIUS_PX         30.0f
#endif

#ifndef COLORBALL_VISION_SCAN_START_ROW
#define COLORBALL_VISION_SCAN_START_ROW        0U
#endif

#ifndef COLORBALL_VISION_SCAN_END_ROW
#define COLORBALL_VISION_SCAN_END_ROW          (COLORBALL_VISION_IMAGE_HEIGHT - 1U)
#endif

#ifndef COLORBALL_VISION_SCAN_START_COL
#define COLORBALL_VISION_SCAN_START_COL        0U
#endif

#ifndef COLORBALL_VISION_SCAN_END_COL
#define COLORBALL_VISION_SCAN_END_COL          (COLORBALL_VISION_IMAGE_WIDTH - 1U)
#endif

/* ========================================================================== */
/* 3. Public data types                                                       */
/* ========================================================================== */

typedef struct
{
    uint16_t hue_center_deg;       /* 0..359 */
    uint16_t hue_tolerance_deg;    /* 0..180 */
    uint8_t saturation_min;        /* HSV S: 0..255 */
    uint8_t value_min;             /* HSV V: 0..255 */

    uint16_t min_pixels;

    float min_aspect;
    float max_aspect;
    float min_fill_ratio;
    float min_radius_px;
    float max_radius_px;

    uint16_t start_row;
    uint16_t end_row;
    uint16_t start_col;
    uint16_t end_col;

} colorball_vision_config_t;

/*
 * RGB565 little-endian image.  stride_bytes is normally width*2.
 * Caller owns data.
 */
typedef struct
{
    uint8_t *data;
    size_t capacity_bytes;
    uint16_t width;
    uint16_t height;
    uint16_t stride_bytes;

} colorball_rgb565_image_t;

typedef struct
{
    bool ready;
    bool valid;
    bool ball_detected;

    float center_x;
    float center_y;

    /* Center normalized to image center: -1 left/up, +1 right/down. */
    float normalized_x;
    float normalized_y;

    float radius_px;
    float fill_ratio;
    float confidence;

    float mean_hue_deg;
    float mean_saturation;         /* 0..1 */
    float mean_value;              /* 0..1 */

    uint16_t bbox_start_x;
    uint16_t bbox_start_y;
    uint16_t bbox_end_x;
    uint16_t bbox_end_y;

    uint32_t ball_pixels;          /* Pixels in selected component. */
    uint32_t threshold_pixels;     /* All HSV-matching pixels in active ROI. */
    uint32_t roi_pixels;

    uint32_t sequence;
    int64_t frame_timestamp_us;
    int64_t update_timestamp_us;

    uint32_t decode_time_us;
    uint32_t process_time_us;

} colorball_vision_data_t;

/* ========================================================================== */
/* 4. Public API                                                              */
/* ========================================================================== */

esp_err_t colorball_vision_init(void);
bool colorball_vision_is_initialized(void);
bool colorball_vision_is_ready(void);

void colorball_vision_get_data(colorball_vision_data_t *out_data);

/* Non-blocking metadata peek for observers/schedulers. No image is copied. */
bool colorball_vision_try_get_data(colorball_vision_data_t *out_data);
void colorball_vision_get_config(colorball_vision_config_t *out_config);
esp_err_t colorball_vision_set_config(const colorball_vision_config_t *config);

/*
 * Non-blocking debug snapshot.
 *
 * rgb565, mask and out_data are copied under the same mutex, so all three
 * describe the same UVC sequence. mask is width*height bytes: 255=target,
 * 0=non-target.  Either image pointer may be NULL if that plane is not needed.
 */
bool colorball_vision_try_get_debug_snapshot(
    colorball_rgb565_image_t *rgb565,
    uint8_t *mask,
    size_t mask_capacity_bytes,
    colorball_vision_data_t *out_data);

#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/* IMPLEMENTATION                                                             */
/* ========================================================================== */

#ifdef COLORBALL_VISION_IMPLEMENTATION

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


#define COLORBALL_RGB_BYTES \
    ((size_t)COLORBALL_VISION_IMAGE_WIDTH * \
     (size_t)COLORBALL_VISION_IMAGE_HEIGHT * 2U)

#define COLORBALL_MASK_BYTES \
    ((size_t)COLORBALL_VISION_IMAGE_WIDTH * \
     (size_t)COLORBALL_VISION_IMAGE_HEIGHT)

#define COLORBALL_QUEUE_ITEMS \
    ((size_t)COLORBALL_VISION_IMAGE_WIDTH * \
     (size_t)COLORBALL_VISION_IMAGE_HEIGHT)

static const char *COLORBALL_VISION_TAG = "COLORBALL_VISION";

static bool s_colorball_initialized = false;
static TaskHandle_t s_colorball_task_handle = NULL;
static SemaphoreHandle_t s_colorball_mutex = NULL;

static uint8_t *s_colorball_work_rgb565 = NULL;
static uint8_t *s_colorball_public_rgb565 = NULL;
static uint8_t *s_colorball_work_mask = NULL;
static uint8_t *s_colorball_public_mask = NULL;
static uint16_t *s_colorball_queue = NULL;

static colorball_vision_data_t s_colorball_latest = {0};

static colorball_vision_config_t s_colorball_config =
{
    .hue_center_deg = (uint16_t)COLORBALL_VISION_HUE_CENTER_DEG,
    .hue_tolerance_deg = (uint16_t)COLORBALL_VISION_HUE_TOLERANCE_DEG,
    .saturation_min = (uint8_t)COLORBALL_VISION_SATURATION_MIN,
    .value_min = (uint8_t)COLORBALL_VISION_VALUE_MIN,
    .min_pixels = (uint16_t)COLORBALL_VISION_MIN_PIXELS,
    .min_aspect = COLORBALL_VISION_MIN_ASPECT,
    .max_aspect = COLORBALL_VISION_MAX_ASPECT,
    .min_fill_ratio = COLORBALL_VISION_MIN_FILL_RATIO,
    .min_radius_px = COLORBALL_VISION_MIN_RADIUS_PX,
    .max_radius_px = COLORBALL_VISION_MAX_RADIUS_PX,
    .start_row = (uint16_t)COLORBALL_VISION_SCAN_START_ROW,
    .end_row = (uint16_t)COLORBALL_VISION_SCAN_END_ROW,
    .start_col = (uint16_t)COLORBALL_VISION_SCAN_START_COL,
    .end_col = (uint16_t)COLORBALL_VISION_SCAN_END_COL,
};

static portMUX_TYPE s_colorball_config_lock = portMUX_INITIALIZER_UNLOCKED;

static inline float colorball_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void *colorball_alloc(size_t size, bool prefer_internal)
{
    void *p = NULL;

    if (prefer_internal)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    if (p == NULL)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
#endif

    if (p == NULL)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }

    return p;
}

static void colorball_free_resources(void)
{
    free(s_colorball_work_rgb565);
    free(s_colorball_public_rgb565);
    free(s_colorball_work_mask);
    free(s_colorball_public_mask);
    free(s_colorball_queue);

    s_colorball_work_rgb565 = NULL;
    s_colorball_public_rgb565 = NULL;
    s_colorball_work_mask = NULL;
    s_colorball_public_mask = NULL;
    s_colorball_queue = NULL;

    if (s_colorball_mutex != NULL)
    {
        vSemaphoreDelete(s_colorball_mutex);
        s_colorball_mutex = NULL;
    }

    memset(&s_colorball_latest, 0, sizeof(s_colorball_latest));
}

static bool colorball_config_valid(const colorball_vision_config_t *c)
{
    if (c == NULL) return false;

    if (c->hue_center_deg > 359U) return false;
    if (c->hue_tolerance_deg > 180U) return false;
    if (c->min_pixels == 0U) return false;

    if ((c->min_aspect <= 0.0f) ||
        (c->max_aspect < c->min_aspect) ||
        (c->min_fill_ratio < 0.0f) ||
        (c->min_fill_ratio > 1.0f) ||
        (c->min_radius_px < 0.0f) ||
        (c->max_radius_px < c->min_radius_px))
    {
        return false;
    }

    if ((c->start_row > c->end_row) ||
        (c->start_col > c->end_col) ||
        (c->end_row >= COLORBALL_VISION_IMAGE_HEIGHT) ||
        (c->end_col >= COLORBALL_VISION_IMAGE_WIDTH))
    {
        return false;
    }

    return true;
}

static bool colorball_downsample_shared_rgb565(
    const vision_shared_frame_t *frame)
{
    if ((frame == NULL) ||
        (frame->rgb565 == NULL) ||
        (s_colorball_work_rgb565 == NULL) ||
        (frame->width != VISION_SHARED_IMAGE_WIDTH) ||
        (frame->height != VISION_SHARED_IMAGE_HEIGHT) ||
        (VISION_SHARED_IMAGE_WIDTH != (COLORBALL_VISION_IMAGE_WIDTH * 2U)) ||
        (VISION_SHARED_IMAGE_HEIGHT != (COLORBALL_VISION_IMAGE_HEIGHT * 2U)))
    {
        return false;
    }

    const uint16_t src_w = frame->width;
    const uint16_t dst_w = (uint16_t)COLORBALL_VISION_IMAGE_WIDTH;
    const uint16_t dst_h = (uint16_t)COLORBALL_VISION_IMAGE_HEIGHT;

    /*
     * Average in native 5/6/5 space. Four source samples cost very little at
     * 60x40 and suppress single-pixel JPEG/color noise before HSV thresholding.
     */
    for (uint16_t y = 0U; y < dst_h; ++y)
    {
        const uint16_t sy = (uint16_t)(y * 2U);

        for (uint16_t x = 0U; x < dst_w; ++x)
        {
            const uint16_t sx = (uint16_t)(x * 2U);

            uint16_t sum_r5 = 0U;
            uint16_t sum_g6 = 0U;
            uint16_t sum_b5 = 0U;

            for (uint16_t dy = 0U; dy < 2U; ++dy)
            {
                for (uint16_t dx = 0U; dx < 2U; ++dx)
                {
                    const size_t si =
                        (size_t)(sy + dy) * (size_t)src_w +
                        (size_t)(sx + dx);

                    const uint8_t *sp = frame->rgb565 + si * 2U;
                    const uint16_t v =
                        (uint16_t)sp[0] | ((uint16_t)sp[1] << 8);

                    sum_r5 += (uint16_t)((v >> 11) & 0x1FU);
                    sum_g6 += (uint16_t)((v >> 5) & 0x3FU);
                    sum_b5 += (uint16_t)(v & 0x1FU);
                }
            }

            const uint16_t r5 = (uint16_t)((sum_r5 + 2U) >> 2);
            const uint16_t g6 = (uint16_t)((sum_g6 + 2U) >> 2);
            const uint16_t b5 = (uint16_t)((sum_b5 + 2U) >> 2);

            const uint16_t v =
                (uint16_t)((r5 << 11) | (g6 << 5) | b5);

            const size_t di =
                (size_t)y * (size_t)dst_w + (size_t)x;

            s_colorball_work_rgb565[di * 2U + 0U] = (uint8_t)v;
            s_colorball_work_rgb565[di * 2U + 1U] = (uint8_t)(v >> 8);
        }
    }

    return true;
}

static inline void colorball_rgb565_load(
    const uint8_t *p,
    uint8_t *r,
    uint8_t *g,
    uint8_t *b)
{
    const uint16_t v =
        (uint16_t)p[0] |
        ((uint16_t)p[1] << 8);

    const uint8_t r5 = (uint8_t)((v >> 11) & 0x1FU);
    const uint8_t g6 = (uint8_t)((v >> 5) & 0x3FU);
    const uint8_t b5 = (uint8_t)(v & 0x1FU);

    *r = (uint8_t)((r5 * 255U + 15U) / 31U);
    *g = (uint8_t)((g6 * 255U + 31U) / 63U);
    *b = (uint8_t)((b5 * 255U + 15U) / 31U);
}

static void colorball_rgb_to_hsv_u8(
    uint8_t r,
    uint8_t g,
    uint8_t b,
    uint16_t *hue_deg,
    uint8_t *sat,
    uint8_t *val)
{
    const uint8_t maxv = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    const uint8_t minv = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    const uint8_t delta = (uint8_t)(maxv - minv);

    *val = maxv;

    if (maxv == 0U)
    {
        *sat = 0U;
        *hue_deg = 0U;
        return;
    }

    *sat = (uint8_t)(((uint16_t)delta * 255U) / maxv);

    if (delta == 0U)
    {
        *hue_deg = 0U;
        return;
    }

    int32_t h;

    if (maxv == r)
    {
        h = (60 * ((int32_t)g - (int32_t)b)) / (int32_t)delta;
    }
    else if (maxv == g)
    {
        h = 120 + (60 * ((int32_t)b - (int32_t)r)) / (int32_t)delta;
    }
    else
    {
        h = 240 + (60 * ((int32_t)r - (int32_t)g)) / (int32_t)delta;
    }

    while (h < 0) h += 360;
    while (h >= 360) h -= 360;

    *hue_deg = (uint16_t)h;
}

static inline bool colorball_hue_matches(
    uint16_t hue,
    uint16_t center,
    uint16_t tolerance)
{
    uint16_t d = (hue > center) ? (hue - center) : (center - hue);
    if (d > 180U) d = (uint16_t)(360U - d);
    return d <= tolerance;
}

typedef struct
{
    uint32_t pixels;
    uint64_t sum_x;
    uint64_t sum_y;
    uint64_t sum_r;
    uint64_t sum_g;
    uint64_t sum_b;
    uint16_t min_x;
    uint16_t min_y;
    uint16_t max_x;
    uint16_t max_y;

} colorball_component_t;

static void colorball_analyze(
    uint32_t sequence,
    int64_t frame_timestamp_us,
    const colorball_vision_config_t *cfg,
    colorball_vision_data_t *out)
{
    memset(out, 0, sizeof(*out));

    out->ready = true;
    out->valid = true;
    out->sequence = sequence;
    out->frame_timestamp_us = frame_timestamp_us;

    const uint16_t w = (uint16_t)COLORBALL_VISION_IMAGE_WIDTH;
    const uint16_t h = (uint16_t)COLORBALL_VISION_IMAGE_HEIGHT;

    memset(s_colorball_work_mask, 0, COLORBALL_MASK_BYTES);

    uint32_t threshold_pixels = 0U;
    uint32_t roi_pixels = 0U;

    for (uint16_t y = cfg->start_row; y <= cfg->end_row; ++y)
    {
        for (uint16_t x = cfg->start_col; x <= cfg->end_col; ++x)
        {
            ++roi_pixels;

            const size_t index = (size_t)y * (size_t)w + (size_t)x;
            const uint8_t *p = s_colorball_work_rgb565 + index * 2U;

            uint8_t r, g, b;
            uint16_t hue;
            uint8_t sat, val;

            colorball_rgb565_load(p, &r, &g, &b);
            colorball_rgb_to_hsv_u8(r, g, b, &hue, &sat, &val);

            if ((sat >= cfg->saturation_min) &&
                (val >= cfg->value_min) &&
                colorball_hue_matches(
                    hue,
                    cfg->hue_center_deg,
                    cfg->hue_tolerance_deg))
            {
                s_colorball_work_mask[index] = 1U;
                ++threshold_pixels;
            }
        }
    }

    out->threshold_pixels = threshold_pixels;
    out->roi_pixels = roi_pixels;

    colorball_component_t best = {0};
    float best_fill = 0.0f;
    float best_radius = 0.0f;
    float best_aspect = 0.0f;

    for (uint16_t sy = cfg->start_row; sy <= cfg->end_row; ++sy)
    {
        for (uint16_t sx = cfg->start_col; sx <= cfg->end_col; ++sx)
        {
            const uint16_t start_index =
                (uint16_t)((uint32_t)sy * (uint32_t)w + (uint32_t)sx);

            if (s_colorball_work_mask[start_index] != 1U)
            {
                continue;
            }

            colorball_component_t comp =
            {
                .pixels = 0U,
                .sum_x = 0U,
                .sum_y = 0U,
                .sum_r = 0U,
                .sum_g = 0U,
                .sum_b = 0U,
                .min_x = sx,
                .min_y = sy,
                .max_x = sx,
                .max_y = sy,
            };

            size_t head = 0U;
            size_t tail = 0U;

            s_colorball_work_mask[start_index] = 2U;
            s_colorball_queue[tail++] = start_index;

            while (head < tail)
            {
                const uint16_t index = s_colorball_queue[head++];
                const uint16_t y = (uint16_t)(index / w);
                const uint16_t x = (uint16_t)(index - (uint16_t)(y * w));

                ++comp.pixels;
                comp.sum_x += x;
                comp.sum_y += y;

                if (x < comp.min_x) comp.min_x = x;
                if (x > comp.max_x) comp.max_x = x;
                if (y < comp.min_y) comp.min_y = y;
                if (y > comp.max_y) comp.max_y = y;

                uint8_t r, g, b;
                colorball_rgb565_load(
                    s_colorball_work_rgb565 + (size_t)index * 2U,
                    &r, &g, &b);

                comp.sum_r += r;
                comp.sum_g += g;
                comp.sum_b += b;

#define COLORBALL_TRY_PUSH(nx_, ny_) do { \
    const uint16_t _x = (uint16_t)(nx_); \
    const uint16_t _y = (uint16_t)(ny_); \
    if ((_x >= cfg->start_col) && (_x <= cfg->end_col) && \
        (_y >= cfg->start_row) && (_y <= cfg->end_row)) { \
        const uint16_t _i = (uint16_t)((uint32_t)_y * (uint32_t)w + (uint32_t)_x); \
        if (s_colorball_work_mask[_i] == 1U) { \
            s_colorball_work_mask[_i] = 2U; \
            if (tail < COLORBALL_QUEUE_ITEMS) s_colorball_queue[tail++] = _i; \
        } \
    } \
} while (0)

                if (x > cfg->start_col) COLORBALL_TRY_PUSH(x - 1U, y);
                if (x < cfg->end_col)   COLORBALL_TRY_PUSH(x + 1U, y);
                if (y > cfg->start_row) COLORBALL_TRY_PUSH(x, y - 1U);
                if (y < cfg->end_row)   COLORBALL_TRY_PUSH(x, y + 1U);

#undef COLORBALL_TRY_PUSH
            }

            const uint32_t bbox_w =
                (uint32_t)comp.max_x - (uint32_t)comp.min_x + 1U;

            const uint32_t bbox_h =
                (uint32_t)comp.max_y - (uint32_t)comp.min_y + 1U;

            const uint32_t bbox_area = bbox_w * bbox_h;
            const float aspect = (float)bbox_w / (float)bbox_h;
            const float fill =
                (bbox_area > 0U)
                ? ((float)comp.pixels / (float)bbox_area)
                : 0.0f;

            const float radius =
                sqrtf((float)comp.pixels / 3.14159265f);

            const bool candidate =
                (comp.pixels >= cfg->min_pixels) &&
                (aspect >= cfg->min_aspect) &&
                (aspect <= cfg->max_aspect) &&
                (fill >= cfg->min_fill_ratio) &&
                (radius >= cfg->min_radius_px) &&
                (radius <= cfg->max_radius_px);

            if (candidate && (comp.pixels > best.pixels))
            {
                best = comp;
                best_fill = fill;
                best_radius = radius;
                best_aspect = aspect;
            }
        }
    }

    /* Convert visited target pixels into browser-friendly 0/255 mask. */
    for (size_t i = 0U; i < COLORBALL_MASK_BYTES; ++i)
    {
        s_colorball_work_mask[i] =
            (s_colorball_work_mask[i] != 0U) ? 255U : 0U;
    }

    if (best.pixels == 0U)
    {
        return;
    }

    out->ball_detected = true;
    out->ball_pixels = best.pixels;

    out->center_x = (float)best.sum_x / (float)best.pixels;
    out->center_y = (float)best.sum_y / (float)best.pixels;

    const float half_w = 0.5f * (float)(w - 1U);
    const float half_h = 0.5f * (float)(h - 1U);

    out->normalized_x =
        (half_w > 0.0f)
        ? colorball_clampf((out->center_x - half_w) / half_w, -1.0f, 1.0f)
        : 0.0f;

    out->normalized_y =
        (half_h > 0.0f)
        ? colorball_clampf((out->center_y - half_h) / half_h, -1.0f, 1.0f)
        : 0.0f;

    out->radius_px = best_radius;
    out->fill_ratio = best_fill;

    out->bbox_start_x = best.min_x;
    out->bbox_start_y = best.min_y;
    out->bbox_end_x = best.max_x;
    out->bbox_end_y = best.max_y;

    const uint8_t mean_r = (uint8_t)(best.sum_r / best.pixels);
    const uint8_t mean_g = (uint8_t)(best.sum_g / best.pixels);
    const uint8_t mean_b = (uint8_t)(best.sum_b / best.pixels);

    uint16_t mean_hue = 0U;
    uint8_t mean_sat = 0U;
    uint8_t mean_val = 0U;

    colorball_rgb_to_hsv_u8(
        mean_r, mean_g, mean_b,
        &mean_hue, &mean_sat, &mean_val);

    out->mean_hue_deg = (float)mean_hue;
    out->mean_saturation = (float)mean_sat / 255.0f;
    out->mean_value = (float)mean_val / 255.0f;

    const float size_score =
        colorball_clampf(
            (float)best.pixels / ((float)cfg->min_pixels * 4.0f),
            0.0f, 1.0f);

    const float round_score =
        (best_aspect >= 1.0f)
        ? (1.0f / best_aspect)
        : best_aspect;

    const float fill_den = 0.78f - cfg->min_fill_ratio;
    const float fill_score =
        (fill_den > 0.01f)
        ? colorball_clampf(
              (best_fill - cfg->min_fill_ratio) / fill_den,
              0.0f, 1.0f)
        : 1.0f;

    out->confidence =
        colorball_clampf(
            0.45f * size_score +
            0.30f * round_score +
            0.25f * fill_score,
            0.0f, 1.0f);
}

static void colorball_task(void *arg)
{
    (void)arg;

    uint32_t last_sequence = UINT32_MAX;
    uint32_t frame_counter = 0U;

    ESP_LOGI(COLORBALL_VISION_TAG, "Vision consumer started (shared RGB565)");

    while (1)
    {
        vision_shared_frame_t frame = {0};

        const esp_err_t frame_ret =
            vision_shared_decode_acquire_latest(
                last_sequence,
                &frame);

        if (frame_ret == ESP_OK)
        {
            const uint32_t sequence = frame.sequence;
            const int64_t frame_timestamp_us = frame.frame_timestamp_us;
            const uint32_t shared_decode_time_us = frame.decode_time_us;

            last_sequence = sequence;

            const bool process_this =
                ((frame_counter++ % COLORBALL_VISION_FRAME_DIVIDER) == 0U);

            if (!process_this)
            {
                vision_shared_decode_release(&frame);
                vTaskDelay(pdMS_TO_TICKS(COLORBALL_VISION_POLL_MS));
                continue;
            }

            const int64_t process_start = esp_timer_get_time();

            const bool downsample_ok =
                colorball_downsample_shared_rgb565(&frame);

            /* Release shared slot immediately after the small 2x2 box copy. */
            vision_shared_decode_release(&frame);

            if (downsample_ok)
            {
                colorball_vision_config_t cfg;
                taskENTER_CRITICAL(&s_colorball_config_lock);
                cfg = s_colorball_config;
                taskEXIT_CRITICAL(&s_colorball_config_lock);

                colorball_vision_data_t observation;
                colorball_analyze(
                    sequence,
                    frame_timestamp_us,
                    &cfg,
                    &observation);

                const int64_t process_end = esp_timer_get_time();

                /* Shared decoder time: there is no second color JPEG decode. */
                observation.decode_time_us = shared_decode_time_us;
                observation.process_time_us =
                    (uint32_t)((process_end >= process_start)
                        ? (process_end - process_start)
                        : 0);
                observation.update_timestamp_us = process_end;

                if (xSemaphoreTake(
                        s_colorball_mutex,
                        pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    memcpy(
                        s_colorball_public_rgb565,
                        s_colorball_work_rgb565,
                        COLORBALL_RGB_BYTES);

                    memcpy(
                        s_colorball_public_mask,
                        s_colorball_work_mask,
                        COLORBALL_MASK_BYTES);

                    s_colorball_latest = observation;
                    xSemaphoreGive(s_colorball_mutex);
                }
            }
            else
            {
                ESP_LOGW(
                    COLORBALL_VISION_TAG,
                    "Shared RGB geometry mismatch at seq=%u",
                    (unsigned)sequence);
            }
        }
        else if ((frame_ret != ESP_ERR_NOT_FOUND) &&
                 (frame_ret != ESP_ERR_INVALID_STATE) &&
                 (frame_ret != ESP_ERR_TIMEOUT))
        {
            ESP_LOGW(
                COLORBALL_VISION_TAG,
                "Acquire shared frame failed: %s",
                esp_err_to_name(frame_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(COLORBALL_VISION_POLL_MS));
    }
}

esp_err_t colorball_vision_init(void)
{
    if (s_colorball_initialized)
    {
        return ESP_OK;
    }

    if (!colorball_config_valid(&s_colorball_config))
    {
        ESP_LOGE(COLORBALL_VISION_TAG, "Invalid default detector configuration");
        return ESP_ERR_INVALID_ARG;
    }

    if ((VISION_SHARED_IMAGE_WIDTH != (COLORBALL_VISION_IMAGE_WIDTH * 2U)) ||
        (VISION_SHARED_IMAGE_HEIGHT != (COLORBALL_VISION_IMAGE_HEIGHT * 2U)))
    {
        ESP_LOGE(COLORBALL_VISION_TAG, "shared/ball geometry must be exactly 2:1");
        return ESP_ERR_INVALID_SIZE;
    }

    s_colorball_mutex = xSemaphoreCreateMutex();
    if (s_colorball_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_colorball_work_rgb565 =
        (uint8_t *)colorball_alloc(COLORBALL_RGB_BYTES, false);

    s_colorball_public_rgb565 =
        (uint8_t *)colorball_alloc(COLORBALL_RGB_BYTES, false);

    s_colorball_work_mask =
        (uint8_t *)colorball_alloc(COLORBALL_MASK_BYTES, false);

    s_colorball_public_mask =
        (uint8_t *)colorball_alloc(COLORBALL_MASK_BYTES, false);

    s_colorball_queue =
        (uint16_t *)colorball_alloc(
            COLORBALL_QUEUE_ITEMS * sizeof(uint16_t),
            false);

    if ((s_colorball_work_rgb565 == NULL) ||
        (s_colorball_public_rgb565 == NULL) ||
        (s_colorball_work_mask == NULL) ||
        (s_colorball_public_mask == NULL) ||
        (s_colorball_queue == NULL))
    {
        colorball_free_resources();
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t ret = vision_shared_decode_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            COLORBALL_VISION_TAG,
            "vision_shared_decode_init failed: %s",
            esp_err_to_name(ret));
        colorball_free_resources();
        return ret;
    }

    const BaseType_t task_result =
        xTaskCreatePinnedToCore(
            colorball_task,
            "colorball_vision",
            COLORBALL_VISION_TASK_STACK_SIZE,
            NULL,
            COLORBALL_VISION_TASK_PRIORITY,
            &s_colorball_task_handle,
            COLORBALL_VISION_TASK_CORE);

    if (task_result != pdPASS)
    {
        s_colorball_task_handle = NULL;
        colorball_free_resources();
        return ESP_ERR_NO_MEM;
    }

    s_colorball_initialized = true;

    ESP_LOGI(
        COLORBALL_VISION_TAG,
        "Initialized consumer: shared=%ux%u@%u -> box2 RGB565=%ux%u every=%u target H=%u+-%u S>=%u V>=%u",
        (unsigned)VISION_SHARED_IMAGE_WIDTH,
        (unsigned)VISION_SHARED_IMAGE_HEIGHT,
        (unsigned)VISION_SHARED_CAMERA_FPS,
        (unsigned)COLORBALL_VISION_IMAGE_WIDTH,
        (unsigned)COLORBALL_VISION_IMAGE_HEIGHT,
        (unsigned)COLORBALL_VISION_FRAME_DIVIDER,
        (unsigned)s_colorball_config.hue_center_deg,
        (unsigned)s_colorball_config.hue_tolerance_deg,
        (unsigned)s_colorball_config.saturation_min,
        (unsigned)s_colorball_config.value_min);

    return ESP_OK;
}

bool colorball_vision_is_initialized(void)
{
    return s_colorball_initialized;
}

void colorball_vision_get_data(colorball_vision_data_t *out_data)
{
    if (out_data == NULL)
    {
        return;
    }

    memset(out_data, 0, sizeof(*out_data));

    if ((!s_colorball_initialized) ||
        (s_colorball_mutex == NULL))
    {
        return;
    }

    if (xSemaphoreTake(
            s_colorball_mutex,
            pdMS_TO_TICKS(20)) != pdTRUE)
    {
        return;
    }

    *out_data = s_colorball_latest;

    xSemaphoreGive(s_colorball_mutex);

    if (!out_data->ready ||
        (out_data->update_timestamp_us <= 0))
    {
        out_data->valid = false;
        return;
    }

    const int64_t age_us =
        esp_timer_get_time() - out_data->update_timestamp_us;

    out_data->valid =
        (age_us >= 0) &&
        (age_us <= ((int64_t)COLORBALL_VISION_STALE_MS * 1000LL));
}


bool colorball_vision_try_get_data(colorball_vision_data_t *out_data)
{
    if (out_data == NULL)
    {
        return false;
    }

    memset(out_data, 0, sizeof(*out_data));

    if ((!s_colorball_initialized) ||
        (s_colorball_mutex == NULL))
    {
        return false;
    }

    /* Observers must never wait for the detector. */
    if (xSemaphoreTake(s_colorball_mutex, 0) != pdTRUE)
    {
        return false;
    }

    *out_data = s_colorball_latest;

    xSemaphoreGive(s_colorball_mutex);

    if (!out_data->ready)
    {
        return false;
    }

    if (out_data->update_timestamp_us > 0)
    {
        const int64_t age_us =
            esp_timer_get_time() - out_data->update_timestamp_us;

        out_data->valid =
            (age_us >= 0) &&
            (age_us <= ((int64_t)COLORBALL_VISION_STALE_MS * 1000LL));
    }
    else
    {
        out_data->valid = false;
    }

    return true;
}

bool colorball_vision_is_ready(void)
{
    colorball_vision_data_t data;
    colorball_vision_get_data(&data);
    return data.valid;
}

void colorball_vision_get_config(colorball_vision_config_t *out_config)
{
    if (out_config == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&s_colorball_config_lock);
    *out_config = s_colorball_config;
    taskEXIT_CRITICAL(&s_colorball_config_lock);
}

esp_err_t colorball_vision_set_config(const colorball_vision_config_t *config)
{
    if (!colorball_config_valid(config))
    {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_colorball_config_lock);
    s_colorball_config = *config;
    taskEXIT_CRITICAL(&s_colorball_config_lock);

    return ESP_OK;
}

bool colorball_vision_try_get_debug_snapshot(
    colorball_rgb565_image_t *rgb565,
    uint8_t *mask,
    size_t mask_capacity_bytes,
    colorball_vision_data_t *out_data)
{
    if (out_data == NULL)
    {
        return false;
    }

    const bool want_rgb = (rgb565 != NULL);

    if (want_rgb)
    {
        if ((rgb565->data == NULL) ||
            (rgb565->capacity_bytes < COLORBALL_RGB_BYTES))
        {
            return false;
        }
    }

    if ((mask != NULL) &&
        (mask_capacity_bytes < COLORBALL_MASK_BYTES))
    {
        return false;
    }

    memset(out_data, 0, sizeof(*out_data));

    if ((!s_colorball_initialized) ||
        (s_colorball_mutex == NULL) ||
        (s_colorball_public_rgb565 == NULL) ||
        (s_colorball_public_mask == NULL))
    {
        return false;
    }

    /* Debug must never wait for the detector. */
    if (xSemaphoreTake(s_colorball_mutex, 0) != pdTRUE)
    {
        return false;
    }

    colorball_vision_data_t data = s_colorball_latest;

    if (!data.ready)
    {
        xSemaphoreGive(s_colorball_mutex);
        return false;
    }

    if (want_rgb)
    {
        memcpy(
            rgb565->data,
            s_colorball_public_rgb565,
            COLORBALL_RGB_BYTES);
    }

    if (mask != NULL)
    {
        memcpy(
            mask,
            s_colorball_public_mask,
            COLORBALL_MASK_BYTES);
    }

    xSemaphoreGive(s_colorball_mutex);

    if (data.update_timestamp_us > 0)
    {
        const int64_t age_us =
            esp_timer_get_time() - data.update_timestamp_us;

        data.valid =
            (age_us >= 0) &&
            (age_us <= ((int64_t)COLORBALL_VISION_STALE_MS * 1000LL));
    }
    else
    {
        data.valid = false;
    }

    *out_data = data;

    if (want_rgb)
    {
        rgb565->width = (uint16_t)COLORBALL_VISION_IMAGE_WIDTH;
        rgb565->height = (uint16_t)COLORBALL_VISION_IMAGE_HEIGHT;
        rgb565->stride_bytes =
            (uint16_t)(COLORBALL_VISION_IMAGE_WIDTH * 2U);
    }

    return true;
}

#endif /* COLORBALL_VISION_IMPLEMENTATION */
#endif /* COLORBALL_VISION_H */
