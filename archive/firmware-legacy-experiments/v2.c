/*
 * esp32_runner_f2_2_test_stack_safe.c
 *
 * Real ESP32 / ESP-IDF F2.2 Runner test, revised for stack safety.
 *
 * Why this version exists:
 * ------------------------
 * The previous test allocated trajectory_runner_t, configuration and status
 * objects as locals inside helper functions, then synchronously called
 * trajectory_runner_run_file(). The Runner may in turn initialize/calibrate
 * the MPU6050. All of those stack frames coexist and can overflow the ESP-IDF
 * main task stack.
 *
 * This version:
 *   1. keeps large Runner/config/status objects in static storage;
 *   2. runs the blocking Runner test in a dedicated FreeRTOS task;
 *   3. gives that task an explicit 12 KiB stack;
 *   4. initializes hardware once, then uses auto_init_hardware=false;
 *   5. logs stack high-water mark in ESP-IDF bytes.
 *
 * F2.2 PEN events are still logical only; no real pen hardware is called.
 */

#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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


#define TRAJECTORY_DECODER_IMPLEMENTATION
#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#define TRAJECTORY_TRACKER_IMPLEMENTATION
#define TRAJECTORY_RUNNER_IMPLEMENTATION

#include "trajectory_decoder.h"
#include "trajectory_executor.h"
#include "trajectory_runner.h"
#include "trajectory_tracker.h"


static const char *TAG = "runner_f2_2_test";


#ifndef F2_2_SPIFFS_PARTITION_LABEL
#define F2_2_SPIFFS_PARTITION_LABEL "storage"
#endif


#ifndef F2_2_RUN_MOTION_TESTS
#define F2_2_RUN_MOTION_TESTS 0
#endif


#ifndef F2_2_RUN_UNSUPPORTED_CUBIC_TEST
#define F2_2_RUN_UNSUPPORTED_CUBIC_TEST 0
#endif


/*
 * ESP-IDF FreeRTOS task stack size is specified in bytes.
 * 12 KiB is intentionally generous for the current blocking Runner +
 * MPU6050 calibration integration test.
 */
#ifndef F2_2_TEST_TASK_STACK_BYTES
#define F2_2_TEST_TASK_STACK_BYTES (12U * 1024U)
#endif


#ifndef F2_2_TEST_TASK_PRIORITY
#define F2_2_TEST_TASK_PRIORITY 5
#endif


/*
 * Large runtime objects live in static storage, not on the task stack.
 */
static trajectory_runner_config_t s_runner_config;
static trajectory_runner_t s_runner;
static trajectory_runner_status_t s_runner_status;


static void log_stack_hwm(
    const char *where)
{
    /*
     * ESP-IDF reports uxTaskGetStackHighWaterMark() in bytes.
     */
    const UBaseType_t free_bytes =
        uxTaskGetStackHighWaterMark(NULL);

    ESP_LOGI(
        TAG,
        "Stack HWM @ %s: %u bytes minimum free",
        where,
        (unsigned)free_bytes);
}


static esp_err_t mount_spiffs(void)
{
    const esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = F2_2_SPIFFS_PARTITION_LABEL,
        .max_files = 8,
        .format_if_mount_failed = false,
    };

    esp_err_t ret =
        esp_vfs_spiffs_register(&conf);

    if ((ret != ESP_OK) &&
        (ret != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(
            TAG,
            "SPIFFS mount failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    size_t total = 0;
    size_t used = 0;

    ret =
        esp_spiffs_info(
            F2_2_SPIFFS_PARTITION_LABEL,
            &total,
            &used);

    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS total=%u used=%u",
            (unsigned)total,
            (unsigned)used);
    }

    return ESP_OK;
}


static bool file_exists(
    const char *path)
{
    FILE *f =
        fopen(path, "rb");

    if (f == NULL)
    {
        ESP_LOGE(
            TAG,
            "Missing fixture: %s",
            path);

        return false;
    }

    fseek(
        f,
        0,
        SEEK_END);

    const long size =
        ftell(f);

    fclose(f);

    ESP_LOGI(
        TAG,
        "Fixture %s size=%ld",
        path,
        size);

    return true;
}


static bool prepare_hardware_once(void)
{
    trajectory_runner_get_default_config(
        &s_runner_config);

    /*
     * First initialize one Runner instance only so system_init() can use its
     * configuration. Large instance lives in static storage.
     */
    esp_err_t ret =
        trajectory_runner_init(
            &s_runner,
            &s_runner_config);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Initial runner_init failed: %s",
            esp_err_to_name(ret));

        return false;
    }

    log_stack_hwm(
        "before system_init / MPU calibration");

    ret =
        trajectory_runner_system_init(
            &s_runner);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_runner_system_init failed: %s",
            esp_err_to_name(ret));

        return false;
    }

    log_stack_hwm(
        "after system_init / MPU calibration");

    /*
     * Hardware is now initialized once.
     * Future trajectory_runner_run_file() calls only verify readiness and do
     * not enter MPU calibration again.
     */
    s_runner_config.auto_init_hardware =
        false;

    s_runner_config.debug_log_enable =
        true;

    s_runner_config.debug_log_period_ms =
        100U;

    return true;
}


static bool run_expect_success(
    const char *path)
{
    /*
     * Reinitialize the static Runner runtime, keeping hardware already ready.
     */
    esp_err_t ret =
        trajectory_runner_init(
            &s_runner,
            &s_runner_config);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "runner_init failed: %s",
            esp_err_to_name(ret));

        return false;
    }

    ESP_LOGI(
        TAG,
        "RUN: %s",
        path);

    log_stack_hwm(
        "before trajectory_runner_run_file");

    ret =
        trajectory_runner_run_file(
            &s_runner,
            path);

    log_stack_hwm(
        "after trajectory_runner_run_file");

    s_runner_status =
        (trajectory_runner_status_t){0};

    trajectory_runner_get_status(
        &s_runner,
        &s_runner_status);

    ESP_LOGI(
        TAG,
        "Result: ret=%s state=%s error=%s pen=%s cycles=%lu",
        esp_err_to_name(ret),
        trajectory_runner_state_name(
            s_runner_status.state),
        trajectory_runner_error_name(
            s_runner_status.error),
        trajectory_runner_pen_state_name(
            s_runner_status.pen_state),
        (unsigned long)
            s_runner_status.control_cycles);

    return
        (ret == ESP_OK) &&
        (s_runner_status.state ==
         TRAJECTORY_RUNNER_FINISHED) &&
        (s_runner_status.error ==
         TRAJECTORY_RUNNER_ERROR_NONE) &&
        (s_runner_status.pen_state ==
         TRAJECTORY_RUNNER_PEN_UP);
}


static bool run_expect_cubic_unsupported(
    const char *path)
{
    esp_err_t ret =
        trajectory_runner_init(
            &s_runner,
            &s_runner_config);

    if (ret != ESP_OK)
    {
        return false;
    }

    ret =
        trajectory_runner_run_file(
            &s_runner,
            path);

    s_runner_status =
        (trajectory_runner_status_t){0};

    trajectory_runner_get_status(
        &s_runner,
        &s_runner_status);

    ESP_LOGI(
        TAG,
        "CUBIC expected error: ret=%s state=%s error=%s",
        esp_err_to_name(ret),
        trajectory_runner_state_name(
            s_runner_status.state),
        trajectory_runner_error_name(
            s_runner_status.error));

    return
        (ret == ESP_ERR_NOT_SUPPORTED) &&
        (s_runner_status.state ==
         TRAJECTORY_RUNNER_ERROR) &&
        (s_runner_status.error ==
         TRAJECTORY_RUNNER_ERROR_RECORD_TYPE) &&
        (s_runner_status.pen_state ==
         TRAJECTORY_RUNNER_PEN_UP);
}


static void runner_test_task(
    void *arg)
{
    (void)arg;

    ESP_LOGI(
        TAG,
        "========================================");

    ESP_LOGI(
        TAG,
        "F2.2 stack-safe Runner ESP32 test START");

    ESP_LOGI(
        TAG,
        "Task stack = %u bytes",
        (unsigned)F2_2_TEST_TASK_STACK_BYTES);

    ESP_LOGI(
        TAG,
        "Logical PEN only; no pen hardware is called");

    ESP_LOGI(
        TAG,
        "========================================");

    log_stack_hwm(
        "task entry");

    if (mount_spiffs() != ESP_OK)
    {
        vTaskDelete(NULL);
        return;
    }

    if (!prepare_hardware_once())
    {
        ESP_LOGE(
            TAG,
            "FINAL RESULT: FAIL (hardware init)");

        vTaskDelete(NULL);
        return;
    }

    unsigned pass = 0U;
    unsigned fail = 0U;

    const char *event_only =
        "/spiffs/trj2_event_only.traj";

    if (file_exists(event_only) &&
        run_expect_success(event_only))
    {
        ESP_LOGI(
            TAG,
            "PASS: event-only + non-blocking WAIT");

        pass++;
    }
    else
    {
        ESP_LOGE(
            TAG,
            "FAIL: event-only + WAIT");

        fail++;
    }


#if F2_2_RUN_MOTION_TESTS

    ESP_LOGW(
        TAG,
        "MOTION TESTS ENABLED - ROBOT MAY MOVE");

    const char *smooth =
        "/spiffs/trj2_smooth_small.traj";

    if (file_exists(smooth) &&
        run_expect_success(smooth))
    {
        ESP_LOGI(
            TAG,
            "PASS: small smooth LINE/CIRCLE/LINE");

        pass++;
    }
    else
    {
        ESP_LOGE(
            TAG,
            "FAIL: small smooth path");

        fail++;
    }

    const char *motion_event =
        "/spiffs/trj2_motion_event_small.traj";

    if (file_exists(motion_event) &&
        run_expect_success(motion_event))
    {
        ESP_LOGI(
            TAG,
            "PASS: small Motion/Event barriers");

        pass++;
    }
    else
    {
        ESP_LOGE(
            TAG,
            "FAIL: Motion/Event path");

        fail++;
    }

#endif


#if F2_2_RUN_UNSUPPORTED_CUBIC_TEST

    ESP_LOGW(
        TAG,
        "CUBIC unsupported test ENABLED - first LINE moves 20 mm");

    const char *cubic =
        "/spiffs/trj2_cubic_after_line.traj";

    if (file_exists(cubic) &&
        run_expect_cubic_unsupported(cubic))
    {
        ESP_LOGI(
            TAG,
            "PASS: CUBIC rejected after safe stop");

        pass++;
    }
    else
    {
        ESP_LOGE(
            TAG,
            "FAIL: CUBIC rejection behavior");

        fail++;
    }

#endif

    log_stack_hwm(
        "test end");

    ESP_LOGI(
        TAG,
        "========================================");

    ESP_LOGI(
        TAG,
        "F2.2 summary: PASS=%u FAIL=%u",
        pass,
        fail);

    if (fail == 0U)
    {
        ESP_LOGI(
            TAG,
            "FINAL RESULT: PASS");
    }
    else
    {
        ESP_LOGE(
            TAG,
            "FINAL RESULT: FAIL");
    }

    ESP_LOGI(
        TAG,
        "========================================");

    vTaskDelete(NULL);
}


void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Creating dedicated F2.2 Runner test task");

    const BaseType_t created =
        xTaskCreate(
            runner_test_task,
            "f2_2_runner_test",
            F2_2_TEST_TASK_STACK_BYTES,
            NULL,
            F2_2_TEST_TASK_PRIORITY,
            NULL);

    if (created != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create Runner test task");
    }
}
