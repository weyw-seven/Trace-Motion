/*
 * main.c
 *
 * 合并版本：
 *   1. 先执行 line_tracker 巡线避障（包含超声波、避障模块）
 *   2. 检测到 line_tracker 完成（到达终点）后停车，等待避障结束
 *   3. 调用 line_tracker_stop() 和 line_tracker_enable(false)
 *   4. 等待 2 秒
 *   5. 启动 ball_kicker 任务，寻找并推动两个球
 *
 * 整合自：
 *   - ball_kicker 版本（找球传球）
 *   - debug_test 版本（巡线避障 + 调试）
 */

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spiffs.h"
#include "usb_stream.h"


/* ============================================================================
 * 单头文件实现宏 – 全部放在这里，避免重复定义
 * ========================================================================== */

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"

#define CHASSIS_KINEMATICS_IMPLEMENTATION
#include "chassis_kinematics.h"

#define CHASSIS_ODOMETRY_IMPLEMENTATION
#include "chassis_odometry.h"

#define CHASSIS_MOTION_IMPLEMENTATION
#include "chassis_motion.h"

#define VOICE_DEMO_IMPLEMENTATION
#include "voice_demo.h"

#define IMAGE_DECODE_IMPLEMENTATION
#include "image_decode.h"

#define UVC_MODULE_IMPLEMENTATION
#include "uvc_module.h"

#define LINE_VISION_IMPLEMENTATION
#include "line_vision.h"

#define COLORBALL_VISION_IMPLEMENTATION
#include "colorball_vision.h"

#define LINE_TRACKER_IMPLEMENTATION
#include "line_tracker.h"

#define ULTRASONIC_SENSOR_IMPLEMENTATION
#include "ultrasonic_sensor.h"

#define OBSTACLE_AVOID_IMPLEMENTATION
#include "obstacle_avoid.h"

#define OLED_DISPLAY_IMPLEMENTATION
#include "oled_display.h"

#define BALL_TARGET_IMPLEMENTATION
#include "ball_target.h"

#define BALL_APPROACH_IMPLEMENTATION
#include "ball_approach.h"

#define BALL_KICKER_IMPLEMENTATION
#include "ball_kicker.h"

#define LINE_DEBUG_WIFI_IMPLEMENTATION
#include "line_debug_wifi.h"

#define LINE_DEBUG_WEB_IMPLEMENTATION
#include "line_debug_web.h"

/* ============================================================================
 * 应用配置
 * ========================================================================== */

#ifndef APP_CAMERA_FPS
#define APP_CAMERA_FPS                         15U
#endif

#ifndef VISION_SHARED_CAMERA_FPS
#define VISION_SHARED_CAMERA_FPS               APP_CAMERA_FPS
#endif

#ifndef APP_STATUS_PERIOD_MS
#define APP_STATUS_PERIOD_MS                   2000U
#endif

/* 巡线完成后等待时间（ms） */
#ifndef APP_WAIT_BEFORE_BALL_MS
#define APP_WAIT_BEFORE_BALL_MS                2000U
#endif

/* 语音 Demo 总开关：0 关闭，1 开启 */
#ifndef APP_VOICE_DEMO_ENABLE
#define APP_VOICE_DEMO_ENABLE                  1
#endif

/* 软 AP 配置 */
#ifndef APP_WIFI_SSID
#define APP_WIFI_SSID                          "TraceMotion-Setup"
#endif

#ifndef APP_WIFI_PASSWORD
/* Development placeholder: provide a private password before enabling Wi-Fi. */
#define APP_WIFI_PASSWORD                      "change-me-before-flashing"
#endif

#ifndef APP_WIFI_CHANNEL
#define APP_WIFI_CHANNEL                       6
#endif

#ifndef APP_WIFI_MAX_CLIENTS
#define APP_WIFI_MAX_CLIENTS                   3
#endif

#ifndef APP_WIFI_IP_STRING
#define APP_WIFI_IP_STRING                     "192.168.4.1"
#endif

#ifndef APP_WEB_ALGORITHM_IMAGES_ENABLE
#define APP_WEB_ALGORITHM_IMAGES_ENABLE        0
#endif

/* ============================================================================
 * 全局变量
 * ========================================================================== */

static const char *TAG = "LINE_CAR_APP";

static TaskHandle_t s_ball_kicker_task_handle = NULL;

/* ============================================================================
 * 辅助函数：检查 line_tracker 是否完成
 * ========================================================================== */

static bool app_is_line_tracker_finished(void)
{
    line_tracker_status_t status;
    line_tracker_get_status(&status);
    return (status.state == LINE_STATE_FINISHED);
}

/* ============================================================================
 * Ball-kicker 后台任务（与单球版本相同）
 * ========================================================================== */

static void ball_kicker_task_func(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz

    ESP_LOGI(TAG, "Ball-kicker task started");

    while (1) {
        (void)ball_kicker_update();
        vTaskDelay(period);
    }
}

/* ============================================================================
 * 状态日志（扩展版，包含 line_tracker 和 ball_kicker 信息）
 * ========================================================================== */

static void app_log_status(
    uint32_t *prev_uvc_accepted,
    uint32_t *prev_decoded,
    int64_t *prev_time_us)
{
    line_tracker_status_t line = {0};
    colorball_vision_data_t ball = {0};
    line_debug_web_stats_t web = {0};
    uvc_module_stats_t uvc = {0};
    vision_shared_decode_stats_t shared = {0};
    ball_kicker_status_t kicker_status = {0};

    line_tracker_get_status(&line);
    colorball_vision_get_data(&ball);
    line_debug_web_get_stats(&web);
    uvc_module_get_stats(&uvc);
    vision_shared_decode_get_stats(&shared);
    (void)ball_kicker_get_status(&kicker_status);

    const int64_t now_us = esp_timer_get_time();
    const int64_t dt_us = now_us - *prev_time_us;

    const uint32_t uvc_delta = uvc.accepted_frames - *prev_uvc_accepted;
    const uint32_t decoded_delta = shared.decoded_frames - *prev_decoded;

    const float uvc_fps = (dt_us > 0) ? ((float)uvc_delta * 1000000.0f / (float)dt_us) : 0.0f;
    const float decoded_fps = (dt_us > 0) ? ((float)decoded_delta * 1000000.0f / (float)dt_us) : 0.0f;

    const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    const size_t psram_free = 0U;
#endif

    ESP_LOGI(TAG,
        "fps uvc=%.1f decoded=%.1f req=%u | "
        "line state=%d found=%d | "
        "ball seq=%" PRIu32 " valid=%d found=%d proc=%.1fms | "
        "kicker state=%d running=%d finished=%d error=%d",
        uvc_fps,
        decoded_fps,
        (unsigned)APP_CAMERA_FPS,
        (int)line.state,
        line.line_found,
        ball.sequence,
        ball.valid,
        ball.ball_detected,
        (float)ball.process_time_us / 1000.0f,
        (int)kicker_status.state,
        kicker_status.running,
        kicker_status.finished,
        kicker_status.error);

    ESP_LOGI(TAG,
        "shared ok=%" PRIu32 " dec_err=%" PRIu32
        " pub_drop=%" PRIu32 " last=%.1fms max=%.1fms | "
        "uvc accepted=%" PRIu32 " busy=%" PRIu32
        " slot=%" PRIu32 " over=%" PRIu32 " max_jpeg=%u B",
        shared.decoded_frames,
        shared.decode_errors,
        shared.dropped_no_publish_slot,
        (float)shared.last_decode_time_us / 1000.0f,
        (float)shared.max_decode_time_us / 1000.0f,
        uvc.accepted_frames,
        uvc.dropped_busy,
        uvc.dropped_no_free_slot,
        uvc.dropped_oversize,
        (unsigned)uvc.max_frame_len);

    ESP_LOGI(TAG,
        "web ws=%d mjpeg=%d tel=%" PRIu32 " mjpeg_sent=%" PRIu32
        " mjpeg_drop=%" PRIu32 " send_err=%" PRIu32
        " | heap internal=%u KB psram=%u KB",
        web.client_connected,
        web.mjpeg_client_connected,
        web.telemetry_sent,
        web.mjpeg_frames_sent,
        web.mjpeg_frames_dropped,
        web.mjpeg_send_errors,
        (unsigned)(internal_free / 1024U),
        (unsigned)(psram_free / 1024U));

    *prev_uvc_accepted = uvc.accepted_frames;
    *prev_decoded = shared.decoded_frames;
    *prev_time_us = now_us;
}

// 语音demo 挂载spiffs
static esp_err_t app_mount_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = false,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* ============================================================================
 * 主函数
 * ========================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "ESP32-S3 完整任务：巡线避障 → 找球推球");
    ESP_LOGI(TAG, "================================================");

    /* ----------------------------------------------------------------------
     * 1. 初始化所有硬件及算法模块
     * ------------------------------------------------------------------ */

    /* 电机 */
    ESP_ERROR_CHECK(motor_control_init());

    /* IMU */
    ESP_ERROR_CHECK(mpu6050_init());

    /* 里程计 */
    ESP_ERROR_CHECK(chassis_odometry_init());
    ESP_ERROR_CHECK(chassis_odometry_reset(0.0f, 0.0f, 0.0f));

    /* 运动规划器 */
    ESP_ERROR_CHECK(chassis_motion_init());

    /* 视觉：共享解码 + 线视觉（会启动 UVC） */
    ESP_ERROR_CHECK(line_vision_init());

    /* 彩色球视觉（依赖同一解码器） */
    ESP_ERROR_CHECK(colorball_vision_init());

    /* 巡线状态机（初始化但不启动） */
    ESP_ERROR_CHECK(line_tracker_init());
    line_tracker_set_avoid_done(false);

    /* 超声波 */
    ESP_ERROR_CHECK(ultrasonic_sensor_init());
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(ultrasonic_sensor_start());

    /* 避障模块（依赖 line_tracker, 超声波, chassis_motion） */
    ESP_ERROR_CHECK(obstacle_avoid_init());
    ESP_ERROR_CHECK(obstacle_avoid_start());

    /* OLED 显示（可选） */
    ESP_ERROR_CHECK(oled_display_init());
    ESP_ERROR_CHECK(oled_display_start());

    /* 球相关模块（初始化，但不启动任务） */
    ESP_ERROR_CHECK(ball_kicker_init());   // 内部调用 ball_target_init & ball_approach_init

    /* Wi-Fi SoftAP 和 Web 调试服务 */
    /*
    ESP_ERROR_CHECK(line_debug_wifi_init());
    ESP_ERROR_CHECK(line_debug_web_init());
    ESP_ERROR_CHECK(line_debug_web_start());

    ESP_LOGI(TAG, "Wi-Fi SSID: %s", APP_WIFI_SSID);
    ESP_LOGI(TAG, "密码: %s", APP_WIFI_PASSWORD);
    ESP_LOGI(TAG, "仪表板: http://%s/", APP_WIFI_IP_STRING);
    ESP_LOGI(TAG, "MJPEG 流: http://%s:%u/stream",
             APP_WIFI_IP_STRING, (unsigned)LINE_DEBUG_WEB_MJPEG_PORT);
    */

    /* 语音 Demo（独立模块，不依赖 UVC，只要 I2S 外设可用） */
    if (APP_VOICE_DEMO_ENABLE) {
        (void)app_mount_spiffs();          /* 如果 PCM 在 SPIFFS */
    }
    ESP_ERROR_CHECK(voice_demo_init(APP_VOICE_DEMO_ENABLE));

    /* ----------------------------------------------------------------------
     * 2. 启动巡线避障阶段
     * ------------------------------------------------------------------ */

    ESP_LOGI(TAG, "启动巡线避障...");
    line_tracker_start();

    /* ----------------------------------------------------------------------
     * 3. 等待巡线完成或避障正常完成
     * ------------------------------------------------------------------ */

    ESP_LOGI(TAG, "等待巡线完成...");
    while (!app_is_line_tracker_finished() && !obstacle_avoid_is_finished()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* 阶段结束，无论是巡线终点还是避障完成，都停止巡线并禁用输出 */
    line_tracker_stop();
    line_tracker_enable(false);
    chassis_motion_stop();
    Set_motor(0.0f, 0.0f, 0.0f);

    /* 等待 2 秒（停车等待） */
    ESP_LOGI(TAG, "停车等待 %u ms 后开始找球...", (unsigned)APP_WAIT_BEFORE_BALL_MS);
    vTaskDelay(pdMS_TO_TICKS(APP_WAIT_BEFORE_BALL_MS));

    /* ----------------------------------------------------------------------
     * 6. 启动 ball_kicker 任务
     * ------------------------------------------------------------------ */

    BaseType_t ret = xTaskCreate(
        ball_kicker_task_func,
        "ball_kicker",
        4096,
        NULL,
        6,              /* 优先级与 odometry 一致或稍低 */
        &s_ball_kicker_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 ball_kicker 任务失败");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    esp_err_t start_ret = ball_kicker_start();
    if (start_ret != ESP_OK) {
        ESP_LOGE(TAG, "ball_kicker_start 失败: %s", esp_err_to_name(start_ret));
    } else {
        ESP_LOGI(TAG, "ball_kicker 已启动");
    }

    /* ----------------------------------------------------------------------
     * 7. 主循环 – 持续打印状态
     * ------------------------------------------------------------------ */

    uint32_t prev_uvc_accepted = 0U;
    uint32_t prev_decoded = 0U;
    int64_t prev_time_us = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(APP_STATUS_PERIOD_MS));
        app_log_status(&prev_uvc_accepted, &prev_decoded, &prev_time_us);
    }
}
