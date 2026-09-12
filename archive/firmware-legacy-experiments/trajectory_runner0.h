#ifndef TRAJECTORY_RUNNER_H
#define TRAJECTORY_RUNNER_H

/*
 * trajectory_runner.h
 *
 * Blocking orchestration layer for the trajectory system:
 *
 *   .traj file
 *      -> trajectory_decoder
 *      -> trajectory_executor
 *      -> trajectory_tracker
 *      -> motor_control
 *      -> chassis_odometry feedback
 *
 * The runner deliberately does NOT mount SPIFFS/SD itself. The caller only
 * needs to make the path readable by stdio/fopen() before run_file().
 *
 * Single-header usage:
 *
 *   In exactly ONE .c/.cpp file:
 *
 *     #define TRAJECTORY_RUNNER_IMPLEMENTATION
 *     #include "trajectory_runner.h"
 *
 *   In all other files:
 *
 *     #include "trajectory_runner.h"
 *
 * This header does NOT define the IMPLEMENTATION macros of decoder/executor/
 * tracker/motor/odometry/MPU. Those must still be defined exactly once in
 * the project, as required by each module.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "trajectory_decoder.h"
#include "trajectory_executor.h"
#include "trajectory_tracker.h"
#include "chassis_odometry.h"
#include "motor_control.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. Defaults
 * ============================================================ */

#ifndef TRAJECTORY_RUNNER_DEFAULT_PERIOD_MS
#define TRAJECTORY_RUNNER_DEFAULT_PERIOD_MS             10U
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_READY_TIMEOUT_MS
#define TRAJECTORY_RUNNER_DEFAULT_READY_TIMEOUT_MS       5000U
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_DEBUG_LOG_PERIOD_MS
#define TRAJECTORY_RUNNER_DEFAULT_DEBUG_LOG_PERIOD_MS    100U
#endif

/* 0 disables runner-level total runtime timeout. */
#ifndef TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS
#define TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS         0U
#endif

/* ============================================================
 * 2. State / error
 * ============================================================ */

typedef enum
{
    TRAJECTORY_RUNNER_IDLE = 0,
    TRAJECTORY_RUNNER_RUNNING,
    TRAJECTORY_RUNNER_FINISHED,
    TRAJECTORY_RUNNER_CANCELLED,
    TRAJECTORY_RUNNER_ERROR

} trajectory_runner_state_t;


typedef enum
{
    TRAJECTORY_RUNNER_ERROR_NONE = 0,
    TRAJECTORY_RUNNER_ERROR_INVALID_ARGUMENT,
    TRAJECTORY_RUNNER_ERROR_HARDWARE_INIT,
    TRAJECTORY_RUNNER_ERROR_HARDWARE_NOT_READY,
    TRAJECTORY_RUNNER_ERROR_DECODER_OPEN,
    TRAJECTORY_RUNNER_ERROR_HEADER,
    TRAJECTORY_RUNNER_ERROR_EXECUTOR_INIT,
    TRAJECTORY_RUNNER_ERROR_EXECUTOR_START,
    TRAJECTORY_RUNNER_ERROR_ODOMETRY_RESET,
    TRAJECTORY_RUNNER_ERROR_TRACKER_INIT,
    TRAJECTORY_RUNNER_ERROR_TRACKER_START,
    TRAJECTORY_RUNNER_ERROR_EXECUTOR_UPDATE,
    TRAJECTORY_RUNNER_ERROR_TRACKER_UPDATE,
    TRAJECTORY_RUNNER_ERROR_TIMEOUT,
    TRAJECTORY_RUNNER_ERROR_CANCELLED

} trajectory_runner_error_t;

/* ============================================================
 * 3. Configuration
 * ============================================================ */

typedef struct
{
    /* Tracker parameters used for every run. */
    trajectory_tracker_config_t tracker;

    /* Nominal control period. Recommended: 10 ms. */
    uint32_t control_period_ms;

    /* Time allowed after init while waiting for subsystem readiness. */
    uint32_t ready_timeout_ms;

    /* Optional whole-run timeout. 0 = disabled. */
    uint32_t run_timeout_ms;

    /*
     * true:
     *   run_file() automatically initializes motor -> MPU -> odometry.
     * false:
     *   application owns hardware initialization; runner only verifies ready.
     */
    bool auto_init_hardware;

    /*
     * true:
     *   current physical pose is defined as the .traj header start pose before
     *   tracker_start(). This matches the current drawing/trajectory workflow.
     * false:
     *   existing odometry WORLD pose is preserved and tracker_start() verifies
     *   it against the trajectory header.
     */
    bool reset_odometry_to_trajectory_start;

    /* Development log. Keep false in normal operation. */
    bool debug_log_enable;
    uint32_t debug_log_period_ms;

} trajectory_runner_config_t;

/* ============================================================
 * 4. Runtime status
 * ============================================================ */

typedef struct
{
    trajectory_runner_state_t state;
    trajectory_runner_error_t error;
    esp_err_t last_esp_err;

    uint32_t control_cycles;
    float elapsed_s;

    bool stop_requested;
    bool executor_finished;
    bool tracker_finished;

    trajectory_file_header_t header;
    trajectory_tracker_status_t tracker;
    chassis_odometry_state_t odometry;
    motor_status_t motor;

} trajectory_runner_status_t;

/* ============================================================
 * 5. Runner instance
 * ============================================================ */

typedef struct
{
    bool initialized;
    bool hardware_initialized_by_runner;
    bool decoder_opened;
    bool executor_initialized;
    bool tracker_initialized;

    volatile bool stop_requested;

    trajectory_runner_config_t config;

    trajectory_decoder_t decoder;
    trajectory_executor_t executor;
    trajectory_tracker_t tracker;
    trajectory_file_header_t header;

    trajectory_runner_state_t state;
    trajectory_runner_error_t error;
    esp_err_t last_esp_err;

    uint32_t control_cycles;
    int64_t run_start_us;

} trajectory_runner_t;

/* ============================================================
 * 6. Public API
 * ============================================================ */

/** Fill a configuration with conservative defaults. */
void trajectory_runner_get_default_config(
    trajectory_runner_config_t *config);

/** Initialize/reset one runner instance. Does not move the chassis. */
esp_err_t trajectory_runner_init(
    trajectory_runner_t *runner,
    const trajectory_runner_config_t *config);

/**
 * Initialize motor_control -> MPU6050 -> chassis_odometry and wait until ready.
 * Safe to call again with the current modules; their init functions are
 * idempotent.
 */
esp_err_t trajectory_runner_system_init(
    trajectory_runner_t *runner);

/**
 * Run one .traj file synchronously in the calling task.
 *
 * The file system must already be mounted and path must be readable by fopen().
 * The function returns only after FINISHED / CANCELLED / ERROR.
 */
esp_err_t trajectory_runner_run_file(
    trajectory_runner_t *runner,
    const char *path);

/** Request a blocking run_file() call to stop from another task/context. */
void trajectory_runner_request_stop(
    trajectory_runner_t *runner);

bool trajectory_runner_is_running(
    const trajectory_runner_t *runner);

void trajectory_runner_get_status(
    trajectory_runner_t *runner,
    trajectory_runner_status_t *status);

const char *trajectory_runner_state_name(
    trajectory_runner_state_t state);

const char *trajectory_runner_error_name(
    trajectory_runner_error_t error);

/*
 * Convenience singleton API.
 *
 * First call lazily creates a default runner with auto_init_hardware=true.
 * This is the short-form API intended for a small app_main():
 *
 *     trajectory_run_file("/spiffs/test.traj");
 */
esp_err_t trajectory_run_file(
    const char *path);

trajectory_runner_t *trajectory_runner_default_instance(void);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */

#ifdef TRAJECTORY_RUNNER_IMPLEMENTATION

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRAJECTORY_RUNNER_TAG "traj_runner"

static trajectory_runner_t g_trajectory_runner_default = {0};


static bool trajectory_runner_config_valid(
    const trajectory_runner_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if (config->control_period_ms == 0U)
    {
        return false;
    }

    if (config->ready_timeout_ms == 0U)
    {
        return false;
    }

    if (config->debug_log_enable &&
        (config->debug_log_period_ms == 0U))
    {
        return false;
    }

    return true;
}


static bool trajectory_runner_base_hardware_ready(void)
{
    if (!motor_control_is_ready())
    {
        return false;
    }

    mpu6050_status_t imu = {0};
    mpu6050_get_status(&imu);

    return imu.initialized && imu.sampling;
}


static bool trajectory_runner_all_hardware_ready(void)
{
    return
        trajectory_runner_base_hardware_ready() &&
        chassis_odometry_is_ready();
}


static esp_err_t trajectory_runner_wait_ready(
    bool require_odometry,
    uint32_t timeout_ms)
{
    const int64_t start_us = esp_timer_get_time();
    const int64_t timeout_us =
        (int64_t)timeout_ms * 1000LL;

    for (;;)
    {
        const bool ready =
            require_odometry
                ? trajectory_runner_all_hardware_ready()
                : trajectory_runner_base_hardware_ready();

        if (ready)
        {
            return ESP_OK;
        }

        if ((esp_timer_get_time() - start_us) >= timeout_us)
        {
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}


static void trajectory_runner_stop_outputs(
    trajectory_runner_t *runner,
    bool stop_tracker)
{
    motor_stop();

    if (runner == NULL)
    {
        return;
    }

    if (stop_tracker && runner->tracker_initialized)
    {
        trajectory_tracker_stop(&runner->tracker);
    }

    if (runner->executor_initialized)
    {
        trajectory_executor_stop(&runner->executor);
    }

    if (runner->decoder_opened)
    {
        trajectory_decoder_close(&runner->decoder);
    }

    runner->decoder_opened = false;
    runner->executor_initialized = false;
    runner->tracker_initialized = false;
}


static esp_err_t trajectory_runner_fail(
    trajectory_runner_t *runner,
    trajectory_runner_error_t error,
    esp_err_t ret,
    const char *message)
{
    if (runner != NULL)
    {
        runner->state = TRAJECTORY_RUNNER_ERROR;
        runner->error = error;
        runner->last_esp_err = ret;

        trajectory_runner_stop_outputs(
            runner,
            true);
    }
    else
    {
        motor_stop();
    }

    if (message != NULL)
    {
        ESP_LOGE(
            TRAJECTORY_RUNNER_TAG,
            "%s: %s",
            message,
            esp_err_to_name(ret));
    }

    return ret;
}


static void trajectory_runner_debug_log(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) ||
        !runner->config.debug_log_enable)
    {
        return;
    }

    trajectory_tracker_status_t ts = {0};
    motor_status_t motor = {0};

    trajectory_tracker_get_status(
        &runner->tracker,
        &ts);

    motor_get_status(&motor);

    ESP_LOGI(
        TRAJECTORY_RUNNER_TAG,
        "t=%6.2f seg=%lu ref=(%7.1f,%7.1f) "
        "odom=(%7.1f,%7.1f,%6.1fdeg) err=%6.1f "
        "vref=%6.1f cmd=(%6.1f,%6.1f,%6.2f) "
        "MOTOR A=(%.1f/%.1f/%.1f) "
        "B=(%.1f/%.1f/%.1f) D=(%.1f/%.1f/%.1f)",
        (double)((float)(esp_timer_get_time() - runner->run_start_us) / 1000000.0f),
        (unsigned long)ts.segment_index,
        (double)ts.reference_x_mm,
        (double)ts.reference_y_mm,
        (double)ts.odom_x_mm,
        (double)ts.odom_y_mm,
        (double)ts.odom_yaw_deg,
        (double)ts.position_error_mm,
        (double)ts.reference_speed_mm_s,
        (double)ts.command_vx_body_mm_s,
        (double)ts.command_vy_body_mm_s,
        (double)ts.command_w_rad_s,
        (double)motor.A.target_speed_mm_s,
        (double)motor.A.actual_speed_mm_s,
        (double)motor.A.output_pwm,
        (double)motor.B.target_speed_mm_s,
        (double)motor.B.actual_speed_mm_s,
        (double)motor.B.output_pwm,
        (double)motor.D.target_speed_mm_s,
        (double)motor.D.actual_speed_mm_s,
        (double)motor.D.output_pwm);
}


void trajectory_runner_get_default_config(
    trajectory_runner_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    *config = (trajectory_runner_config_t){0};

    config->tracker =
        (trajectory_tracker_config_t)
        TRAJECTORY_TRACKER_CONFIG_DEFAULT();

    config->control_period_ms =
        TRAJECTORY_RUNNER_DEFAULT_PERIOD_MS;

    config->ready_timeout_ms =
        TRAJECTORY_RUNNER_DEFAULT_READY_TIMEOUT_MS;

    config->run_timeout_ms =
        TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS;

    config->auto_init_hardware = true;
    config->reset_odometry_to_trajectory_start = true;
    config->debug_log_enable = false;

    config->debug_log_period_ms =
        TRAJECTORY_RUNNER_DEFAULT_DEBUG_LOG_PERIOD_MS;
}


esp_err_t trajectory_runner_init(
    trajectory_runner_t *runner,
    const trajectory_runner_config_t *config)
{
    if ((runner == NULL) ||
        !trajectory_runner_config_valid(config))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *runner = (trajectory_runner_t){0};

    runner->config = *config;
    runner->decoder =
        (trajectory_decoder_t)
        TRAJECTORY_DECODER_INITIALIZER;

    runner->state = TRAJECTORY_RUNNER_IDLE;
    runner->error = TRAJECTORY_RUNNER_ERROR_NONE;
    runner->last_esp_err = ESP_OK;
    runner->initialized = true;

    return ESP_OK;
}


esp_err_t trajectory_runner_system_init(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) ||
        !runner->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = motor_control_init();

    if (ret != ESP_OK)
    {
        runner->error =
            TRAJECTORY_RUNNER_ERROR_HARDWARE_INIT;
        runner->last_esp_err = ret;
        return ret;
    }

    ret = mpu6050_init();

    if (ret != ESP_OK)
    {
        runner->error =
            TRAJECTORY_RUNNER_ERROR_HARDWARE_INIT;
        runner->last_esp_err = ret;
        return ret;
    }

    ret = trajectory_runner_wait_ready(
        false,
        runner->config.ready_timeout_ms);

    if (ret != ESP_OK)
    {
        runner->error =
            TRAJECTORY_RUNNER_ERROR_HARDWARE_NOT_READY;
        runner->last_esp_err = ret;
        return ret;
    }

    ret = chassis_odometry_init();

    if (ret != ESP_OK)
    {
        runner->error =
            TRAJECTORY_RUNNER_ERROR_HARDWARE_INIT;
        runner->last_esp_err = ret;
        return ret;
    }

    ret = trajectory_runner_wait_ready(
        true,
        runner->config.ready_timeout_ms);

    if (ret != ESP_OK)
    {
        runner->error =
            TRAJECTORY_RUNNER_ERROR_HARDWARE_NOT_READY;
        runner->last_esp_err = ret;
        return ret;
    }

    motor_stop();
    runner->hardware_initialized_by_runner = true;
    runner->error = TRAJECTORY_RUNNER_ERROR_NONE;
    runner->last_esp_err = ESP_OK;

    return ESP_OK;
}


esp_err_t trajectory_runner_run_file(
    trajectory_runner_t *runner,
    const char *path)
{
    if ((runner == NULL) ||
        !runner->initialized ||
        (path == NULL) ||
        (path[0] == '\0'))
    {
        if (runner != NULL)
        {
            runner->state = TRAJECTORY_RUNNER_ERROR;
            runner->error =
                TRAJECTORY_RUNNER_ERROR_INVALID_ARGUMENT;
            runner->last_esp_err = ESP_ERR_INVALID_ARG;
        }

        return ESP_ERR_INVALID_ARG;
    }

    if (runner->state == TRAJECTORY_RUNNER_RUNNING)
    {
        return ESP_ERR_INVALID_STATE;
    }

    runner->stop_requested = false;
    runner->control_cycles = 0U;
    runner->run_start_us = 0;
    runner->header = (trajectory_file_header_t){0};
    runner->error = TRAJECTORY_RUNNER_ERROR_NONE;
    runner->last_esp_err = ESP_OK;
    runner->state = TRAJECTORY_RUNNER_IDLE;

    if (runner->decoder_opened)
    {
        trajectory_decoder_close(&runner->decoder);
    }

    runner->decoder_opened = false;
    runner->executor_initialized = false;
    runner->tracker_initialized = false;

    runner->decoder =
        (trajectory_decoder_t)
        TRAJECTORY_DECODER_INITIALIZER;
    runner->executor = (trajectory_executor_t){0};
    runner->tracker = (trajectory_tracker_t){0};

    esp_err_t ret;

    if (runner->config.auto_init_hardware)
    {
        ret = trajectory_runner_system_init(runner);

        if (ret != ESP_OK)
        {
            return trajectory_runner_fail(
                runner,
                runner->error,
                ret,
                "Hardware initialization failed");
        }
    }
    else if (!trajectory_runner_all_hardware_ready())
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_HARDWARE_NOT_READY,
            ESP_ERR_INVALID_STATE,
            "Hardware is not ready");
    }

    motor_stop();

    ret = trajectory_decoder_open(
        &runner->decoder,
        path);

    if (ret != ESP_OK)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_DECODER_OPEN,
            ret,
            "trajectory_decoder_open failed");
    }

    runner->decoder_opened = true;

    ret = trajectory_decoder_get_header(
        &runner->decoder,
        &runner->header);

    if (ret != ESP_OK)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_HEADER,
            ret,
            "trajectory_decoder_get_header failed");
    }

    ret = trajectory_executor_init(
        &runner->executor,
        &runner->decoder);

    if (ret != ESP_OK)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_EXECUTOR_INIT,
            ret,
            "trajectory_executor_init failed");
    }

    runner->executor_initialized = true;

    ret = trajectory_tracker_init(
        &runner->tracker,
        &runner->config.tracker);

    if (ret != ESP_OK)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_TRACKER_INIT,
            ret,
            "trajectory_tracker_init failed");
    }

    runner->tracker_initialized = true;

    if (runner->config.reset_odometry_to_trajectory_start)
    {
        ret = chassis_odometry_reset(
            runner->header.start_x_mm,
            runner->header.start_y_mm,
            runner->header.start_yaw_deg);

        if (ret != ESP_OK)
        {
            return trajectory_runner_fail(
                runner,
                TRAJECTORY_RUNNER_ERROR_ODOMETRY_RESET,
                ret,
                "chassis_odometry_reset failed");
        }
    }

    ret = trajectory_executor_start(
        &runner->executor);

    if (ret != ESP_OK)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_EXECUTOR_START,
            ret,
            "trajectory_executor_start failed");
    }

    ret = trajectory_tracker_start(
        &runner->tracker,
        &runner->header);

    if (ret != ESP_OK)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_TRACKER_START,
            ret,
            "trajectory_tracker_start failed");
    }

    runner->state = TRAJECTORY_RUNNER_RUNNING;
    runner->run_start_us = esp_timer_get_time();

    ESP_LOGI(
        TRAJECTORY_RUNNER_TAG,
        "Running %s: start=(%.1f, %.1f, %.1f deg), segments=%lu",
        path,
        (double)runner->header.start_x_mm,
        (double)runner->header.start_y_mm,
        (double)runner->header.start_yaw_deg,
        (unsigned long)runner->header.segment_count);

    const float dt_s =
        (float)runner->config.control_period_ms / 1000.0f;

    TickType_t period_ticks =
        pdMS_TO_TICKS(runner->config.control_period_ms);

    if (period_ticks == 0)
    {
        period_ticks = 1;
    }

    TickType_t last_wake = xTaskGetTickCount();

    const uint32_t log_divider =
        runner->config.debug_log_enable
            ? ((runner->config.debug_log_period_ms +
                runner->config.control_period_ms - 1U) /
               runner->config.control_period_ms)
            : 0U;

    for (;;)
    {
        if (runner->stop_requested)
        {
            runner->state = TRAJECTORY_RUNNER_CANCELLED;
            runner->error =
                TRAJECTORY_RUNNER_ERROR_CANCELLED;
            runner->last_esp_err = ESP_ERR_INVALID_STATE;

            trajectory_runner_stop_outputs(
                runner,
                true);

            ESP_LOGW(
                TRAJECTORY_RUNNER_TAG,
                "Trajectory cancelled");

            return ESP_ERR_INVALID_STATE;
        }

        if ((runner->config.run_timeout_ms > 0U) &&
            ((esp_timer_get_time() - runner->run_start_us) >=
             ((int64_t)runner->config.run_timeout_ms * 1000LL)))
        {
            return trajectory_runner_fail(
                runner,
                TRAJECTORY_RUNNER_ERROR_TIMEOUT,
                ESP_ERR_TIMEOUT,
                "Trajectory runner timeout");
        }

        trajectory_reference_t ref = {0};

        ret = trajectory_executor_update(
            &runner->executor,
            dt_s,
            &ref);

        if (ret != ESP_OK)
        {
            return trajectory_runner_fail(
                runner,
                TRAJECTORY_RUNNER_ERROR_EXECUTOR_UPDATE,
                ret,
                "trajectory_executor_update failed");
        }

        ret = trajectory_tracker_update(
            &runner->tracker,
            &ref,
            dt_s);

        if (ret != ESP_OK)
        {
            return trajectory_runner_fail(
                runner,
                TRAJECTORY_RUNNER_ERROR_TRACKER_UPDATE,
                ret,
                "trajectory_tracker_update failed");
        }

        runner->control_cycles++;

        if ((log_divider > 0U) &&
            ((runner->control_cycles % log_divider) == 0U))
        {
            trajectory_runner_debug_log(runner);
        }

        if (trajectory_tracker_is_finished(
                &runner->tracker))
        {
            motor_stop();

            if (runner->executor_initialized)
            {
                trajectory_executor_stop(&runner->executor);
                runner->executor_initialized = false;
            }

            if (runner->decoder_opened)
            {
                trajectory_decoder_close(&runner->decoder);
                runner->decoder_opened = false;
            }

            runner->state = TRAJECTORY_RUNNER_FINISHED;
            runner->error = TRAJECTORY_RUNNER_ERROR_NONE;
            runner->last_esp_err = ESP_OK;

            ESP_LOGI(
                TRAJECTORY_RUNNER_TAG,
                "Trajectory finished in %.2f s (%lu cycles)",
                (double)((float)(esp_timer_get_time() - runner->run_start_us) /
                         1000000.0f),
                (unsigned long)runner->control_cycles);

            return ESP_OK;
        }

        /*
         * Important: do NOT stop merely because executor is finished.
         * Continue calling executor_update() so it keeps supplying the final
         * stationary reference while tracker performs SETTLING.
         */
        vTaskDelayUntil(
            &last_wake,
            period_ticks);
    }
}


void trajectory_runner_request_stop(
    trajectory_runner_t *runner)
{
    if (runner == NULL)
    {
        return;
    }

    runner->stop_requested = true;
}


bool trajectory_runner_is_running(
    const trajectory_runner_t *runner)
{
    return
        (runner != NULL) &&
        runner->initialized &&
        (runner->state == TRAJECTORY_RUNNER_RUNNING);
}


void trajectory_runner_get_status(
    trajectory_runner_t *runner,
    trajectory_runner_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    *status = (trajectory_runner_status_t){0};

    if (runner == NULL)
    {
        status->state = TRAJECTORY_RUNNER_ERROR;
        status->error =
            TRAJECTORY_RUNNER_ERROR_INVALID_ARGUMENT;
        status->last_esp_err = ESP_ERR_INVALID_ARG;
        return;
    }

    status->state = runner->state;
    status->error = runner->error;
    status->last_esp_err = runner->last_esp_err;
    status->control_cycles = runner->control_cycles;
    status->stop_requested = runner->stop_requested;
    status->header = runner->header;

    if (runner->run_start_us > 0)
    {
        status->elapsed_s =
            (float)(esp_timer_get_time() - runner->run_start_us) /
            1000000.0f;
    }

    status->executor_finished =
        (runner->state == TRAJECTORY_RUNNER_FINISHED) ||
        (runner->executor_initialized &&
         trajectory_executor_is_finished(&runner->executor));

    status->tracker_finished =
        (runner->state == TRAJECTORY_RUNNER_FINISHED) ||
        (runner->tracker_initialized &&
         trajectory_tracker_is_finished(&runner->tracker));

    if (runner->tracker_initialized)
    {
        trajectory_tracker_get_status(
            &runner->tracker,
            &status->tracker);
    }

    chassis_odometry_get_state(
        &status->odometry);

    motor_get_status(
        &status->motor);
}


const char *trajectory_runner_state_name(
    trajectory_runner_state_t state)
{
    switch (state)
    {
        case TRAJECTORY_RUNNER_IDLE:
            return "IDLE";

        case TRAJECTORY_RUNNER_RUNNING:
            return "RUNNING";

        case TRAJECTORY_RUNNER_FINISHED:
            return "FINISHED";

        case TRAJECTORY_RUNNER_CANCELLED:
            return "CANCELLED";

        case TRAJECTORY_RUNNER_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}


const char *trajectory_runner_error_name(
    trajectory_runner_error_t error)
{
    switch (error)
    {
        case TRAJECTORY_RUNNER_ERROR_NONE:
            return "NONE";
        case TRAJECTORY_RUNNER_ERROR_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";
        case TRAJECTORY_RUNNER_ERROR_HARDWARE_INIT:
            return "HARDWARE_INIT";
        case TRAJECTORY_RUNNER_ERROR_HARDWARE_NOT_READY:
            return "HARDWARE_NOT_READY";
        case TRAJECTORY_RUNNER_ERROR_DECODER_OPEN:
            return "DECODER_OPEN";
        case TRAJECTORY_RUNNER_ERROR_HEADER:
            return "HEADER";
        case TRAJECTORY_RUNNER_ERROR_EXECUTOR_INIT:
            return "EXECUTOR_INIT";
        case TRAJECTORY_RUNNER_ERROR_EXECUTOR_START:
            return "EXECUTOR_START";
        case TRAJECTORY_RUNNER_ERROR_ODOMETRY_RESET:
            return "ODOMETRY_RESET";
        case TRAJECTORY_RUNNER_ERROR_TRACKER_INIT:
            return "TRACKER_INIT";
        case TRAJECTORY_RUNNER_ERROR_TRACKER_START:
            return "TRACKER_START";
        case TRAJECTORY_RUNNER_ERROR_EXECUTOR_UPDATE:
            return "EXECUTOR_UPDATE";
        case TRAJECTORY_RUNNER_ERROR_TRACKER_UPDATE:
            return "TRACKER_UPDATE";
        case TRAJECTORY_RUNNER_ERROR_TIMEOUT:
            return "TIMEOUT";
        case TRAJECTORY_RUNNER_ERROR_CANCELLED:
            return "CANCELLED";
        default:
            return "UNKNOWN";
    }
}


trajectory_runner_t *trajectory_runner_default_instance(void)
{
    return &g_trajectory_runner_default;
}


esp_err_t trajectory_run_file(
    const char *path)
{
    if (!g_trajectory_runner_default.initialized)
    {
        trajectory_runner_config_t config;
        trajectory_runner_get_default_config(&config);

        esp_err_t ret = trajectory_runner_init(
            &g_trajectory_runner_default,
            &config);

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    return trajectory_runner_run_file(
        &g_trajectory_runner_default,
        path);
}

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_RUNNER_IMPLEMENTATION */
#endif /* TRAJECTORY_RUNNER_H */
