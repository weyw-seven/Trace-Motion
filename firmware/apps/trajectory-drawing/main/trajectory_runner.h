#ifndef TRAJECTORY_RUNNER_H
#define TRAJECTORY_RUNNER_H

/*
 * trajectory_runner.h
 *
 * Blocking orchestration / TRJ2 record-dispatch layer:
 *
 *   .traj file
 *      -> trajectory_decoder
 *      -> trajectory_runner record dispatcher
 *           |-> Motion -> trajectory_executor
 *           |-> Event  -> logical event state machine
 *      -> trajectory_tracker
 *      -> motor_control
 *      -> chassis_odometry feedback
 *
 * TRJ1 keeps the validated legacy decoder->executor path.
 * TRJ2 gives Decoder cursor ownership to Runner. LINE/CIRCLE are injected
 * into the EXTERNAL executor; PEN_UP/PEN_DOWN/WAIT stay in Runner.
 *
 * Physical PEN events are optional and are enabled explicitly through the
 * runner configuration. The default configuration remains logical-only.
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

#include "pen_control.h"

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


#ifndef TRAJECTORY_RUNNER_DEFAULT_EVENT_POSITION_TOLERANCE_MM
#define TRAJECTORY_RUNNER_DEFAULT_EVENT_POSITION_TOLERANCE_MM        5.0f
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_EVENT_YAW_TOLERANCE_DEG
#define TRAJECTORY_RUNNER_DEFAULT_EVENT_YAW_TOLERANCE_DEG            2.0f
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_EVENT_MAX_TRANSLATION_COMMAND_MM_S
#define TRAJECTORY_RUNNER_DEFAULT_EVENT_MAX_TRANSLATION_COMMAND_MM_S 15.0f
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_EVENT_MAX_ANGULAR_COMMAND_RAD_S
#define TRAJECTORY_RUNNER_DEFAULT_EVENT_MAX_ANGULAR_COMMAND_RAD_S    0.10f
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_EVENT_SETTLE_TIME_MS
#define TRAJECTORY_RUNNER_DEFAULT_EVENT_SETTLE_TIME_MS               100U
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_EVENT_BARRIER_TIMEOUT_MS
#define TRAJECTORY_RUNNER_DEFAULT_EVENT_BARRIER_TIMEOUT_MS           3000U
#endif

/* 0 disables runner-level total runtime timeout. */
#ifndef TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS
#define TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS         0U
#endif

#ifndef TRAJECTORY_RUNNER_DEFAULT_PEN_ACTION_TIMEOUT_MS
#define TRAJECTORY_RUNNER_DEFAULT_PEN_ACTION_TIMEOUT_MS  1000U
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
    TRAJECTORY_RUNNER_PHASE_NONE = 0,
    TRAJECTORY_RUNNER_PHASE_DISPATCH,
    TRAJECTORY_RUNNER_PHASE_MOTION,
    TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING,
    TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING,
    TRAJECTORY_RUNNER_PHASE_WAIT,
    TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING

} trajectory_runner_phase_t;


typedef enum
{
    TRAJECTORY_RUNNER_PEN_UP = 0,
    TRAJECTORY_RUNNER_PEN_DOWN

} trajectory_runner_pen_state_t;


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
    TRAJECTORY_RUNNER_ERROR_CANCELLED,

    TRAJECTORY_RUNNER_ERROR_RECORD_READ,
    TRAJECTORY_RUNNER_ERROR_RECORD_TYPE,
    TRAJECTORY_RUNNER_ERROR_MOTION_CONVERSION,
    TRAJECTORY_RUNNER_ERROR_EXECUTOR_BEGIN,
    TRAJECTORY_RUNNER_ERROR_EXECUTOR_ADVANCE,
    TRAJECTORY_RUNNER_ERROR_EXECUTOR_FINISH,
    TRAJECTORY_RUNNER_ERROR_EVENT,
    TRAJECTORY_RUNNER_ERROR_BARRIER_TIMEOUT,
    TRAJECTORY_RUNNER_ERROR_PEN_INIT,
    TRAJECTORY_RUNNER_ERROR_PEN_NOT_READY,
    TRAJECTORY_RUNNER_ERROR_PEN_COMMAND,
    TRAJECTORY_RUNNER_ERROR_PEN_UPDATE,
    TRAJECTORY_RUNNER_ERROR_PEN_TIMEOUT,
    TRAJECTORY_RUNNER_ERROR_MOTOR_STALL

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

    /*
     * Physical pen actuator. Disabled by default so a generic Runner config
     * never drives a GPIO accidentally. The application enables it and fills
     * the calibrated pen_control_config_t explicitly.
     */
    bool pen_enabled;
    pen_control_config_t pen;
    uint32_t pen_action_timeout_ms;

    /*
     * Event barrier: nominal Executor speed=0 is not by itself proof that
     * physical tracking correction has settled.
     */
    float event_position_tolerance_mm;
    float event_yaw_tolerance_deg;
    float event_max_translation_command_mm_s;
    float event_max_angular_command_rad_s;
    uint32_t event_settle_time_ms;

    /* 0 disables Event barrier timeout. */
    uint32_t event_barrier_timeout_ms;

    /* Development log. Keep false in normal operation. */
    bool debug_log_enable;
    uint32_t debug_log_period_ms;

} trajectory_runner_config_t;

typedef struct
{
    bool valid;
    uint32_t index;
    trajectory_record_t record;

} trajectory_runner_record_slot_t;


/* ============================================================
 * 4. Runtime status
 * ============================================================ */

typedef struct
{
    trajectory_runner_state_t state;
    trajectory_runner_phase_t phase;
    trajectory_runner_error_t error;
    esp_err_t last_esp_err;

    uint32_t control_cycles;
    float elapsed_s;

    bool stop_requested;
    bool executor_finished;
    bool tracker_finished;

    trajectory_file_header_t header;

    bool current_record_valid;
    uint32_t current_record_index;
    trajectory_record_type_t current_record_type;

    bool next_record_valid;
    uint32_t next_record_index;
    trajectory_record_type_t next_record_type;

    trajectory_runner_pen_state_t pen_state;
    trajectory_runner_pen_state_t pen_target_state;
    bool pen_enabled;
    bool pen_initialized;
    bool pen_action_active;
    uint32_t pen_action_elapsed_ms;
    uint32_t pen_action_remaining_ms;

    bool barrier_settled;
    uint32_t barrier_settle_elapsed_ms;

    bool wait_active;
    float wait_remaining_s;

    trajectory_tracker_status_t tracker;
    chassis_odometry_state_t odometry;
    motor_status_t motor;
    pen_control_status_t pen;

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
    /*
     * stop_outputs() tears down the active tracker after a terminal state.
     * Keep its final snapshot so N3 can report the real root cause rather
     * than collapsing every tracker fault into TRACKER_UPDATE.
     */
    trajectory_tracker_status_t last_tracker_status;
    pen_control_t pen;
    trajectory_file_header_t header;

    trajectory_runner_state_t state;
    trajectory_runner_phase_t phase;
    trajectory_runner_error_t error;
    esp_err_t last_esp_err;

    trajectory_runner_record_slot_t current_record;
    trajectory_runner_record_slot_t next_record;

    trajectory_runner_pen_state_t pen_state;
    trajectory_runner_pen_state_t pen_target_state;
    bool pen_initialized;
    bool pen_action_active;
    int64_t pen_action_start_us;
    int64_t pen_action_deadline_us;

    bool barrier_settled;
    uint32_t barrier_settle_elapsed_ms;
    int64_t barrier_enter_us;

    bool wait_active;
    int64_t wait_deadline_us;

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

const char *trajectory_runner_phase_name(
    trajectory_runner_phase_t phase);

const char *trajectory_runner_pen_state_name(
    trajectory_runner_pen_state_t state);

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
#include <limits.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRAJECTORY_RUNNER_TAG "traj_runner"

static trajectory_runner_t g_trajectory_runner_default = {0};


static bool trajectory_runner_pen_target_reached(
    const trajectory_runner_t *runner)
{
    if ((runner == NULL) || !runner->pen_action_active)
    {
        return false;
    }

    return
        (runner->pen_target_state == TRAJECTORY_RUNNER_PEN_UP)
            ? pen_control_is_up(&runner->pen)
            : pen_control_is_down(&runner->pen);
}


static esp_err_t trajectory_runner_pen_initialize_and_home(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) || !runner->config.pen_enabled)
    {
        return ESP_OK;
    }

    if (runner->config.pen_action_timeout_ms == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!runner->config.pen.positions_calibrated)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!runner->pen_initialized)
    {
        const esp_err_t ret =
            pen_control_init(
                &runner->pen,
                &runner->config.pen);

        if (ret != ESP_OK)
        {
            return ret;
        }

        runner->pen_initialized = true;
    }

    const int64_t start_us =
        esp_timer_get_time();

    const int64_t deadline_us =
        start_us +
        ((int64_t)runner->config.pen_action_timeout_ms * 1000LL);

    for (;;)
    {
        esp_err_t ret =
            pen_control_update(&runner->pen);

        if (ret != ESP_OK)
        {
            return ret;
        }

        if (pen_control_is_up(&runner->pen))
        {
            runner->pen_state =
                TRAJECTORY_RUNNER_PEN_UP;
            runner->pen_target_state =
                TRAJECTORY_RUNNER_PEN_UP;
            runner->pen_action_active = false;
            return ESP_OK;
        }

        if (esp_timer_get_time() >= deadline_us)
        {
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(
            pdMS_TO_TICKS(runner->config.control_period_ms));
    }
}


static esp_err_t trajectory_runner_pen_start_action(
    trajectory_runner_t *runner,
    trajectory_runner_pen_state_t target)
{
    if ((runner == NULL) ||
        !runner->config.pen_enabled ||
        !runner->pen_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t ret =
        (target == TRAJECTORY_RUNNER_PEN_UP)
            ? pen_control_request_up(&runner->pen)
            : pen_control_request_down(&runner->pen);

    if (ret != ESP_OK)
    {
        return ret;
    }

    const int64_t now_us =
        esp_timer_get_time();

    runner->pen_target_state = target;
    runner->pen_action_active = true;
    runner->pen_action_start_us = now_us;
    runner->pen_action_deadline_us =
        now_us +
        ((int64_t)runner->config.pen_action_timeout_ms * 1000LL);
    runner->phase =
        TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING;

    ESP_LOGI(
        TRAJECTORY_RUNNER_TAG,
        "PEN %s start",
        (target == TRAJECTORY_RUNNER_PEN_UP)
            ? "UP"
            : "DOWN");

    return ESP_OK;
}


static bool trajectory_runner_config_valid(
    const trajectory_runner_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if ((config->control_period_ms == 0U) ||
        (config->ready_timeout_ms == 0U))
    {
        return false;
    }

    if (config->pen_enabled &&
        (config->pen_action_timeout_ms == 0U))
    {
        return false;
    }

    if (!isfinite(config->event_position_tolerance_mm) ||
        !isfinite(config->event_yaw_tolerance_deg) ||
        !isfinite(config->event_max_translation_command_mm_s) ||
        !isfinite(config->event_max_angular_command_rad_s) ||
        (config->event_position_tolerance_mm < 0.0f) ||
        (config->event_yaw_tolerance_deg < 0.0f) ||
        (config->event_max_translation_command_mm_s < 0.0f) ||
        (config->event_max_angular_command_rad_s < 0.0f) ||
        (config->event_settle_time_ms == 0U))
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


static void trajectory_runner_record_slot_clear(
    trajectory_runner_record_slot_t *slot)
{
    if (slot != NULL)
    {
        *slot = (trajectory_runner_record_slot_t){0};
    }
}


static bool trajectory_runner_record_is_supported_motion(
    const trajectory_record_t *record)
{
    return
        (record != NULL) &&
        ((record->type == TRAJECTORY_RECORD_LINE) ||
         (record->type == TRAJECTORY_RECORD_CIRCLE));
}


static bool trajectory_runner_record_is_event(
    const trajectory_record_t *record)
{
    return
        (record != NULL) &&
        ((record->type == TRAJECTORY_RECORD_PEN_UP) ||
         (record->type == TRAJECTORY_RECORD_PEN_DOWN) ||
         (record->type == TRAJECTORY_RECORD_WAIT));
}


static esp_err_t trajectory_runner_record_to_motion(
    const trajectory_record_t *record,
    trajectory_segment_t *motion)
{
    if ((record == NULL) || (motion == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *motion = (trajectory_segment_t){0};
    motion->flags = record->flags;
    motion->speed_mm_s = record->speed_mm_s;
    motion->acceleration_mm_s2 = record->acceleration_mm_s2;

    switch (record->type)
    {
        case TRAJECTORY_RECORD_LINE:
            motion->type = TRAJECTORY_SEGMENT_LINE;
            motion->geometry.line = record->payload.line;
            return ESP_OK;

        case TRAJECTORY_RECORD_CIRCLE:
            motion->type = TRAJECTORY_SEGMENT_CIRCLE;
            motion->geometry.circle = record->payload.circle;
            return ESP_OK;

        case TRAJECTORY_RECORD_CUBIC_BEZIER:
            return ESP_ERR_NOT_SUPPORTED;

        default:
            return ESP_ERR_INVALID_ARG;
    }
}


static esp_err_t trajectory_runner_read_record_slot(
    trajectory_runner_t *runner,
    trajectory_runner_record_slot_t *slot)
{
    if ((runner == NULL) || (slot == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    trajectory_runner_record_slot_clear(slot);

    if (!trajectory_decoder_has_next(&runner->decoder))
    {
        return ESP_OK;
    }

    slot->index =
        trajectory_decoder_get_next_record_index(&runner->decoder);

    const esp_err_t ret =
        trajectory_decoder_read_next_record(
            &runner->decoder,
            &slot->record);

    if (ret != ESP_OK)
    {
        trajectory_runner_record_slot_clear(slot);
        return ret;
    }

    slot->valid = true;
    return ESP_OK;
}


static esp_err_t trajectory_runner_prefill_v2_window(
    trajectory_runner_t *runner)
{
    trajectory_runner_record_slot_clear(&runner->current_record);
    trajectory_runner_record_slot_clear(&runner->next_record);

    esp_err_t ret =
        trajectory_runner_read_record_slot(
            runner,
            &runner->current_record);

    if (ret != ESP_OK)
    {
        return ret;
    }

    return trajectory_runner_read_record_slot(
        runner,
        &runner->next_record);
}


static esp_err_t trajectory_runner_shift_v2_window(
    trajectory_runner_t *runner)
{
    if (runner == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    runner->current_record = runner->next_record;
    trajectory_runner_record_slot_clear(&runner->next_record);

    return trajectory_runner_read_record_slot(
        runner,
        &runner->next_record);
}


static void trajectory_runner_reset_event_barrier(
    trajectory_runner_t *runner)
{
    runner->barrier_settled = false;
    runner->barrier_settle_elapsed_ms = 0U;
    runner->barrier_enter_us = esp_timer_get_time();
}


static void trajectory_runner_force_logical_pen_up(
    trajectory_runner_t *runner)
{
    if (runner != NULL)
    {
        runner->pen_state = TRAJECTORY_RUNNER_PEN_UP;
        runner->pen_target_state = TRAJECTORY_RUNNER_PEN_UP;
    }
}


static esp_err_t trajectory_runner_v2_begin_current_motion(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) || !runner->current_record.valid)
    {
        return ESP_ERR_INVALID_STATE;
    }

    trajectory_segment_t current = {0};

    esp_err_t ret =
        trajectory_runner_record_to_motion(
            &runner->current_record.record,
            &current);

    if (ret != ESP_OK)
    {
        return ret;
    }

    trajectory_segment_t lookahead = {0};
    const trajectory_segment_t *lookahead_ptr = NULL;
    uint32_t lookahead_index = 0U;

    if (runner->next_record.valid &&
        trajectory_runner_record_is_supported_motion(
            &runner->next_record.record))
    {
        ret =
            trajectory_runner_record_to_motion(
                &runner->next_record.record,
                &lookahead);

        if (ret != ESP_OK)
        {
            return ret;
        }

        lookahead_ptr = &lookahead;
        lookahead_index = runner->next_record.index;
    }

    ret =
        trajectory_executor_begin_motion(
            &runner->executor,
            &current,
            runner->current_record.index,
            lookahead_ptr,
            lookahead_index);

    if (ret == ESP_OK)
    {
        runner->phase = TRAJECTORY_RUNNER_PHASE_MOTION;
        runner->wait_active = false;
        trajectory_runner_reset_event_barrier(runner);
    }

    return ret;
}


static esp_err_t trajectory_runner_v2_advance_motion(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) ||
        !trajectory_executor_needs_advance(&runner->executor))
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret =
        trajectory_runner_shift_v2_window(runner);

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!runner->current_record.valid ||
        !trajectory_runner_record_is_supported_motion(
            &runner->current_record.record))
    {
        return ESP_ERR_INVALID_STATE;
    }

    trajectory_segment_t lookahead = {0};
    const trajectory_segment_t *lookahead_ptr = NULL;
    uint32_t lookahead_index = 0U;

    if (runner->next_record.valid &&
        trajectory_runner_record_is_supported_motion(
            &runner->next_record.record))
    {
        ret =
            trajectory_runner_record_to_motion(
                &runner->next_record.record,
                &lookahead);

        if (ret != ESP_OK)
        {
            return ret;
        }

        lookahead_ptr = &lookahead;
        lookahead_index = runner->next_record.index;
    }

    ret =
        trajectory_executor_advance_motion(
            &runner->executor,
            lookahead_ptr,
            lookahead_index);

    if (ret == ESP_OK)
    {
        runner->phase = TRAJECTORY_RUNNER_PHASE_MOTION;
        trajectory_runner_reset_event_barrier(runner);
    }

    return ret;
}


static bool trajectory_runner_barrier_status_stable(
    const trajectory_runner_t *runner,
    const trajectory_tracker_status_t *status)
{
    if ((runner == NULL) || (status == NULL))
    {
        return false;
    }

    const float translation_command =
        hypotf(
            status->command_vx_body_mm_s,
            status->command_vy_body_mm_s);

    return
        status->reference_valid &&
        status->hold_active &&
        !status->trajectory_end &&
        (status->position_error_mm <=
         runner->config.event_position_tolerance_mm) &&
        (fabsf(status->yaw_error_deg) <=
         runner->config.event_yaw_tolerance_deg) &&
        (translation_command <=
         runner->config.event_max_translation_command_mm_s) &&
        (fabsf(status->command_w_rad_s) <=
         runner->config.event_max_angular_command_rad_s);
}


static esp_err_t trajectory_runner_v2_update_barrier_after_tracker(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) ||
        (runner->phase != TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING))
    {
        return ESP_OK;
    }

    trajectory_tracker_status_t status = {0};
    trajectory_tracker_get_status(&runner->tracker, &status);

    if (trajectory_runner_barrier_status_stable(runner, &status))
    {
        const uint32_t remaining =
            UINT32_MAX - runner->barrier_settle_elapsed_ms;

        const uint32_t add_ms =
            (runner->config.control_period_ms <= remaining)
                ? runner->config.control_period_ms
                : remaining;

        runner->barrier_settle_elapsed_ms += add_ms;
    }
    else
    {
        runner->barrier_settle_elapsed_ms = 0U;
    }

    if (runner->barrier_settle_elapsed_ms >=
        runner->config.event_settle_time_ms)
    {
        runner->barrier_settled = true;
        runner->phase = TRAJECTORY_RUNNER_PHASE_DISPATCH;

        ESP_LOGI(
            TRAJECTORY_RUNNER_TAG,
            "Event barrier settled");

        return ESP_OK;
    }

    if ((runner->config.event_barrier_timeout_ms > 0U) &&
        ((esp_timer_get_time() - runner->barrier_enter_us) >=
         ((int64_t)runner->config.event_barrier_timeout_ms * 1000LL)))
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


static esp_err_t trajectory_runner_v2_start_wait(
    trajectory_runner_t *runner,
    float duration_s)
{
    if ((runner == NULL) ||
        !isfinite(duration_s) ||
        !(duration_s > 0.0f))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const double duration_us_d =
        (double)duration_s * 1000000.0;

    if (duration_us_d > (double)INT64_MAX)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    const int64_t duration_us =
        (int64_t)(duration_us_d + 0.5);

    const int64_t now = esp_timer_get_time();

    if ((duration_us > 0) &&
        (now > (INT64_MAX - duration_us)))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    runner->wait_active = true;
    runner->wait_deadline_us = now + duration_us;
    runner->phase = TRAJECTORY_RUNNER_PHASE_WAIT;

    ESP_LOGI(
        TRAJECTORY_RUNNER_TAG,
        "WAIT %.3f s",
        (double)duration_s);

    return ESP_OK;
}


static esp_err_t trajectory_runner_v2_dispatch_pre_cycle(
    trajectory_runner_t *runner)
{
    if (runner == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (runner->phase ==
        TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING)
    {
        if (trajectory_runner_pen_target_reached(runner))
        {
            runner->pen_state =
                runner->pen_target_state;
            runner->pen_action_active = false;

            ESP_LOGI(
                TRAJECTORY_RUNNER_TAG,
                "PEN %s complete",
                (runner->pen_state == TRAJECTORY_RUNNER_PEN_UP)
                    ? "UP"
                    : "DOWN");

            const esp_err_t ret =
                trajectory_runner_shift_v2_window(runner);

            if (ret == ESP_OK)
            {
                runner->phase =
                    TRAJECTORY_RUNNER_PHASE_DISPATCH;
                runner->barrier_settled = true;
            }

            return ret;
        }

        if (esp_timer_get_time() >=
            runner->pen_action_deadline_us)
        {
            return ESP_ERR_TIMEOUT;
        }

        return ESP_OK;
    }

    if ((runner->phase == TRAJECTORY_RUNNER_PHASE_MOTION) &&
        trajectory_executor_needs_advance(&runner->executor))
    {
        return trajectory_runner_v2_advance_motion(runner);
    }

    if (runner->phase == TRAJECTORY_RUNNER_PHASE_WAIT)
    {
        if (esp_timer_get_time() < runner->wait_deadline_us)
        {
            return ESP_OK;
        }

        runner->wait_active = false;

        esp_err_t ret =
            trajectory_runner_shift_v2_window(runner);

        if (ret != ESP_OK)
        {
            return ret;
        }

        runner->phase = TRAJECTORY_RUNNER_PHASE_DISPATCH;
        runner->barrier_settled = true;
        return ESP_OK;
    }

    if ((runner->phase == TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING) ||
        (runner->phase == TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING) ||
        (runner->phase == TRAJECTORY_RUNNER_PHASE_MOTION))
    {
        return ESP_OK;
    }

    if (runner->phase != TRAJECTORY_RUNNER_PHASE_DISPATCH)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!runner->current_record.valid)
    {
        const esp_err_t ret =
            trajectory_executor_finish(&runner->executor);

        if (ret == ESP_OK)
        {
            runner->phase =
                TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING;
        }

        return ret;
    }

    const trajectory_record_t *record =
        &runner->current_record.record;

    if (trajectory_runner_record_is_supported_motion(record))
    {
        return trajectory_runner_v2_begin_current_motion(runner);
    }

    if (record->type == TRAJECTORY_RECORD_CUBIC_BEZIER)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!trajectory_runner_record_is_event(record))
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!runner->barrier_settled)
    {
        runner->phase =
            TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING;

        trajectory_runner_reset_event_barrier(runner);
        return ESP_OK;
    }

    switch (record->type)
    {
        case TRAJECTORY_RECORD_PEN_UP:
            if (runner->config.pen_enabled)
            {
                return trajectory_runner_pen_start_action(
                    runner,
                    TRAJECTORY_RUNNER_PEN_UP);
            }

            runner->pen_state =
                TRAJECTORY_RUNNER_PEN_UP;
            ESP_LOGI(
                TRAJECTORY_RUNNER_TAG,
                "EVENT PEN_UP record=%lu (logical)",
                (unsigned long)runner->current_record.index);
            break;

        case TRAJECTORY_RECORD_PEN_DOWN:
            if (runner->config.pen_enabled)
            {
                return trajectory_runner_pen_start_action(
                    runner,
                    TRAJECTORY_RUNNER_PEN_DOWN);
            }

            runner->pen_state =
                TRAJECTORY_RUNNER_PEN_DOWN;
            ESP_LOGI(
                TRAJECTORY_RUNNER_TAG,
                "EVENT PEN_DOWN record=%lu (logical)",
                (unsigned long)runner->current_record.index);
            break;

        case TRAJECTORY_RECORD_WAIT:
            return trajectory_runner_v2_start_wait(
                runner,
                record->payload.wait.duration_s);

        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    const esp_err_t ret =
        trajectory_runner_shift_v2_window(runner);

    if (ret == ESP_OK)
    {
        runner->phase = TRAJECTORY_RUNNER_PHASE_DISPATCH;
        runner->barrier_settled = true;
    }

    return ret;
}


static esp_err_t trajectory_runner_v2_observe_post_cycle(
    trajectory_runner_t *runner)
{
    if (runner == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((runner->phase == TRAJECTORY_RUNNER_PHASE_MOTION) &&
        trajectory_executor_is_holding(&runner->executor))
    {
        if (!runner->next_record.valid)
        {
            const esp_err_t ret =
                trajectory_executor_finish(&runner->executor);

            if (ret == ESP_OK)
            {
                runner->phase =
                    TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING;
            }

            return ret;
        }

        esp_err_t ret =
            trajectory_runner_shift_v2_window(runner);

        if (ret != ESP_OK)
        {
            return ret;
        }

        if (runner->current_record.valid &&
            trajectory_runner_record_is_event(
                &runner->current_record.record))
        {
            runner->phase =
                TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING;

            trajectory_runner_reset_event_barrier(runner);
        }
        else
        {
            /*
             * Unsupported CUBIC is now current, but preceding Motion is already
             * at zero-speed HOLDING. Dispatch next cycle and fail safely.
             */
            runner->phase =
                TRAJECTORY_RUNNER_PHASE_DISPATCH;
        }
    }

    return trajectory_runner_v2_update_barrier_after_tracker(runner);
}


static void trajectory_runner_emergency_pen_up_and_wait(
    trajectory_runner_t *runner)
{
    if ((runner == NULL) ||
        !runner->config.pen_enabled ||
        !runner->pen_initialized)
    {
        return;
    }

    const esp_err_t command_ret =
        pen_control_emergency_up(&runner->pen);
    if (command_ret != ESP_OK)
    {
        ESP_LOGE(
            TRAJECTORY_RUNNER_TAG,
            "Emergency PEN_UP failed: %s",
            esp_err_to_name(command_ret));
        return;
    }

    const uint32_t timeout_ms =
        (runner->config.pen_action_timeout_ms > 0U)
            ? runner->config.pen_action_timeout_ms
            : TRAJECTORY_RUNNER_DEFAULT_PEN_ACTION_TIMEOUT_MS;
    const int64_t deadline_us =
        esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);

    while (!pen_control_is_up(&runner->pen))
    {
        const esp_err_t update_ret =
            pen_control_update(&runner->pen);
        if (update_ret != ESP_OK)
        {
            ESP_LOGE(
                TRAJECTORY_RUNNER_TAG,
                "Emergency PEN_UP update failed: %s",
                esp_err_to_name(update_ret));
            return;
        }

        if (esp_timer_get_time() >= deadline_us)
        {
            ESP_LOGE(
                TRAJECTORY_RUNNER_TAG,
                "Emergency PEN_UP timed out after %lu ms",
                (unsigned long)timeout_ms);
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(runner->config.control_period_ms));
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

    trajectory_runner_emergency_pen_up_and_wait(runner);
    runner->pen_action_active = false;
    runner->pen_target_state = TRAJECTORY_RUNNER_PEN_UP;

    trajectory_runner_force_logical_pen_up(runner);
    runner->wait_active = false;
    runner->barrier_settled = false;
    runner->phase = TRAJECTORY_RUNNER_PHASE_NONE;

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
        if (runner->tracker_initialized)
        {
            trajectory_tracker_get_status(
                &runner->tracker,
                &runner->last_tracker_status);
        }

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

    trajectory_tracker_get_status(&runner->tracker, &ts);
    motor_get_status(&motor);

    ESP_LOGI(
        TRAJECTORY_RUNNER_TAG,
        "t=%6.2f phase=%s trk=%s hold=%d rec=%s[%lu] pen=%s->%s%s "
        "pulse=%u/%u "
        "ref=(%7.1f,%7.1f) odom=(%7.1f,%7.1f,%6.1fdeg) "
        "err=%6.1f vref=%6.1f cmd=(%6.1f,%6.1f,%6.2f) "
        "MOTOR A=(%.1f/%.1f/%.1f) "
        "B=(%.1f/%.1f/%.1f) D=(%.1f/%.1f/%.1f) "
        "ENC A=(%ld/%d/%d/%d/%lu) "
        "B=(%ld/%d/%d/%d/%lu) D=(%ld/%d/%d/%d/%lu)",
        (double)((float)(esp_timer_get_time() - runner->run_start_us) /
                 1000000.0f),
        trajectory_runner_phase_name(runner->phase),
        trajectory_tracker_state_name(ts.state),
        (int)ts.hold_active,
        runner->current_record.valid
            ? trajectory_decoder_record_type_name(
                  runner->current_record.record.type)
            : "EOF",
        runner->current_record.valid
            ? (unsigned long)runner->current_record.index
            : 0UL,
        trajectory_runner_pen_state_name(runner->pen_state),
        trajectory_runner_pen_state_name(runner->pen_target_state),
        runner->pen_action_active ? "*" : "",
        runner->pen_initialized
            ? (unsigned)runner->pen.current_pulse_us
            : 0U,
        runner->pen_initialized
            ? (unsigned)runner->pen.target_pulse_us
            : 0U,
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
        (double)motor.D.output_pwm,
        (long)motor.A.encoder_count,
        (int)motor.A.encoder_ready,
        (int)motor.A.startup_active,
        (int)motor.A.stall_suspected,
        (unsigned long)motor.A.stall_elapsed_ms,
        (long)motor.B.encoder_count,
        (int)motor.B.encoder_ready,
        (int)motor.B.startup_active,
        (int)motor.B.stall_suspected,
        (unsigned long)motor.B.stall_elapsed_ms,
        (long)motor.D.encoder_count,
        (int)motor.D.encoder_ready,
        (int)motor.D.startup_active,
        (int)motor.D.stall_suspected,
        (unsigned long)motor.D.stall_elapsed_ms);
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

    config->event_position_tolerance_mm =
        TRAJECTORY_RUNNER_DEFAULT_EVENT_POSITION_TOLERANCE_MM;

    config->event_yaw_tolerance_deg =
        TRAJECTORY_RUNNER_DEFAULT_EVENT_YAW_TOLERANCE_DEG;

    config->event_max_translation_command_mm_s =
        TRAJECTORY_RUNNER_DEFAULT_EVENT_MAX_TRANSLATION_COMMAND_MM_S;

    config->event_max_angular_command_rad_s =
        TRAJECTORY_RUNNER_DEFAULT_EVENT_MAX_ANGULAR_COMMAND_RAD_S;

    config->event_settle_time_ms =
        TRAJECTORY_RUNNER_DEFAULT_EVENT_SETTLE_TIME_MS;

    config->event_barrier_timeout_ms =
        TRAJECTORY_RUNNER_DEFAULT_EVENT_BARRIER_TIMEOUT_MS;

    /* Keep Tracker's intermediate hold criteria aligned with Runner's event
     * barrier criteria. Applications may still override these explicitly. */
    config->tracker.hold_position_tolerance_mm =
        config->event_position_tolerance_mm;
    config->tracker.hold_yaw_tolerance_deg =
        config->event_yaw_tolerance_deg;
    config->tracker.hold_settle_time_ms =
        config->event_settle_time_ms;

    config->auto_init_hardware = true;
    config->reset_odometry_to_trajectory_start = true;
    config->pen_enabled = false;
    config->pen = (pen_control_config_t){0};
    config->pen_action_timeout_ms =
        TRAJECTORY_RUNNER_DEFAULT_PEN_ACTION_TIMEOUT_MS;
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
    runner->phase = TRAJECTORY_RUNNER_PHASE_NONE;
    runner->pen_state = TRAJECTORY_RUNNER_PEN_UP;
    runner->pen_target_state = TRAJECTORY_RUNNER_PEN_UP;
    runner->pen = (pen_control_t)PEN_CONTROL_INITIALIZER;
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
            runner->error = TRAJECTORY_RUNNER_ERROR_INVALID_ARGUMENT;
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
    runner->last_tracker_status = (trajectory_tracker_status_t){0};
    runner->error = TRAJECTORY_RUNNER_ERROR_NONE;
    runner->last_esp_err = ESP_OK;
    runner->state = TRAJECTORY_RUNNER_IDLE;
    runner->phase = TRAJECTORY_RUNNER_PHASE_NONE;
    runner->pen_state = TRAJECTORY_RUNNER_PEN_UP;
    runner->pen_target_state = TRAJECTORY_RUNNER_PEN_UP;
    runner->pen_action_active = false;
    runner->pen_action_start_us = 0;
    runner->pen_action_deadline_us = 0;
    runner->wait_active = false;
    runner->wait_deadline_us = 0;
    runner->barrier_settled = false;
    runner->barrier_settle_elapsed_ms = 0U;
    runner->barrier_enter_us = 0;

    trajectory_runner_record_slot_clear(&runner->current_record);
    trajectory_runner_record_slot_clear(&runner->next_record);

    if (runner->decoder_opened)
    {
        trajectory_decoder_close(&runner->decoder);
    }

    runner->decoder_opened = false;
    runner->executor_initialized = false;
    runner->tracker_initialized = false;

    runner->decoder =
        (trajectory_decoder_t)TRAJECTORY_DECODER_INITIALIZER;

    runner->executor =
        (trajectory_executor_t)TRAJECTORY_EXECUTOR_INITIALIZER;

    runner->tracker =
        (trajectory_tracker_t)TRAJECTORY_TRACKER_INITIALIZER;

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

    if (runner->config.pen_enabled)
    {
        ret =
            trajectory_runner_pen_initialize_and_home(
                runner);

        if (ret != ESP_OK)
        {
            trajectory_runner_error_t pen_error =
                TRAJECTORY_RUNNER_ERROR_PEN_INIT;

            if (ret == ESP_ERR_INVALID_STATE)
            {
                pen_error =
                    TRAJECTORY_RUNNER_ERROR_PEN_NOT_READY;
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                pen_error =
                    TRAJECTORY_RUNNER_ERROR_PEN_TIMEOUT;
            }

            return trajectory_runner_fail(
                runner,
                pen_error,
                ret,
                "Pen initialization/home failed");
        }
    }

    motor_stop();

    ret = trajectory_decoder_open(&runner->decoder, path);

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

    const bool is_v1 =
        runner->header.version == TRAJECTORY_FILE_VERSION_V1;

    const bool is_v2 =
        runner->header.version == TRAJECTORY_FILE_VERSION_V2;

    if (!is_v1 && !is_v2)
    {
        return trajectory_runner_fail(
            runner,
            TRAJECTORY_RUNNER_ERROR_HEADER,
            ESP_ERR_NOT_SUPPORTED,
            "Unsupported trajectory version");
    }

    if (is_v1)
    {
        ret = trajectory_executor_init(
            &runner->executor,
            &runner->decoder);
    }
    else
    {
        ret = trajectory_executor_init_external(
            &runner->executor,
            &runner->header);
    }

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

    ret = trajectory_executor_start(&runner->executor);

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

    if (is_v2)
    {
        ret = trajectory_runner_prefill_v2_window(runner);

        if (ret != ESP_OK)
        {
            return trajectory_runner_fail(
                runner,
                TRAJECTORY_RUNNER_ERROR_RECORD_READ,
                ret,
                "Failed to prefill TRJ2 record window");
        }

        runner->phase = TRAJECTORY_RUNNER_PHASE_DISPATCH;
        trajectory_runner_reset_event_barrier(runner);
    }
    else
    {
        runner->phase =
            trajectory_executor_is_finished(&runner->executor)
                ? TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING
                : TRAJECTORY_RUNNER_PHASE_MOTION;
    }

    runner->state = TRAJECTORY_RUNNER_RUNNING;
    runner->run_start_us = esp_timer_get_time();

    ESP_LOGI(
        TRAJECTORY_RUNNER_TAG,
        "Running %s: version=%u start=(%.1f, %.1f, %.1f deg), "
        "records=%lu segments=%lu",
        path,
        (unsigned)runner->header.version,
        (double)runner->header.start_x_mm,
        (double)runner->header.start_y_mm,
        (double)runner->header.start_yaw_deg,
        (unsigned long)runner->header.record_count,
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
            runner->error = TRAJECTORY_RUNNER_ERROR_CANCELLED;
            runner->last_esp_err = ESP_ERR_INVALID_STATE;

            trajectory_runner_stop_outputs(runner, true);

            ESP_LOGW(TRAJECTORY_RUNNER_TAG, "Trajectory cancelled");
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

        if (runner->config.pen_enabled &&
            runner->pen_initialized)
        {
            ret =
                pen_control_update(&runner->pen);

            if (ret != ESP_OK)
            {
                return trajectory_runner_fail(
                    runner,
                    TRAJECTORY_RUNNER_ERROR_PEN_UPDATE,
                    ret,
                    "pen_control_update failed");
            }
        }

        if (is_v2)
        {
            ret = trajectory_runner_v2_dispatch_pre_cycle(runner);

            if (ret != ESP_OK)
            {
                trajectory_runner_error_t error =
                    TRAJECTORY_RUNNER_ERROR_EVENT;

                if (ret == ESP_ERR_NOT_SUPPORTED)
                {
                    error = TRAJECTORY_RUNNER_ERROR_RECORD_TYPE;
                }
                else if (ret == ESP_ERR_TIMEOUT &&
                         runner->phase ==
                             TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING)
                {
                    error =
                        TRAJECTORY_RUNNER_ERROR_PEN_TIMEOUT;
                }
                else if (runner->phase ==
                         TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING)
                {
                    error =
                        TRAJECTORY_RUNNER_ERROR_PEN_COMMAND;
                }
                else if (trajectory_executor_get_error(
                             &runner->executor) ==
                         TRAJECTORY_EXECUTOR_ERROR_SOURCE)
                {
                    error = TRAJECTORY_RUNNER_ERROR_EXECUTOR_ADVANCE;
                }
                else if (runner->phase == TRAJECTORY_RUNNER_PHASE_MOTION)
                {
                    error = TRAJECTORY_RUNNER_ERROR_EXECUTOR_BEGIN;
                }

                return trajectory_runner_fail(
                    runner,
                    error,
                    ret,
                    "TRJ2 pre-cycle dispatch failed");
            }
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

        const bool tracker_hold_requested =
            (runner->phase ==
             TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING) ||
            (runner->phase ==
             TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING) ||
            (runner->phase ==
             TRAJECTORY_RUNNER_PHASE_WAIT);

        ret = trajectory_tracker_set_hold_mode(
            &runner->tracker,
            tracker_hold_requested);

        if (ret != ESP_OK)
        {
            return trajectory_runner_fail(
                runner,
                TRAJECTORY_RUNNER_ERROR_TRACKER_UPDATE,
                ret,
                "trajectory_tracker_set_hold_mode failed");
        }

        ret = trajectory_tracker_update(
            &runner->tracker,
            &ref,
            dt_s);

        if (ret != ESP_OK)
        {
            const trajectory_runner_error_t tracker_error =
                (trajectory_tracker_get_error(&runner->tracker) ==
                 TRAJECTORY_TRACKER_ERROR_MOTOR_STALL)
                    ? TRAJECTORY_RUNNER_ERROR_MOTOR_STALL
                    : TRAJECTORY_RUNNER_ERROR_TRACKER_UPDATE;

            return trajectory_runner_fail(
                runner,
                tracker_error,
                ret,
                "trajectory_tracker_update failed");
        }

        runner->control_cycles++;

        if (is_v2)
        {
            ret = trajectory_runner_v2_observe_post_cycle(runner);

            if (ret != ESP_OK)
            {
                const trajectory_runner_error_t error =
                    (ret == ESP_ERR_TIMEOUT)
                        ? TRAJECTORY_RUNNER_ERROR_BARRIER_TIMEOUT
                        : TRAJECTORY_RUNNER_ERROR_RECORD_READ;

                return trajectory_runner_fail(
                    runner,
                    error,
                    ret,
                    "TRJ2 post-cycle dispatch failed");
            }
        }

        if ((log_divider > 0U) &&
            ((runner->control_cycles % log_divider) == 0U))
        {
            trajectory_runner_debug_log(runner);
        }

        if (trajectory_tracker_is_finished(&runner->tracker))
        {
            motor_stop();

            if (runner->config.pen_enabled &&
                runner->pen_initialized)
            {
                if (runner->pen_action_active)
                {
                    if (trajectory_runner_pen_target_reached(runner))
                    {
                        runner->pen_state =
                            TRAJECTORY_RUNNER_PEN_UP;
                        runner->pen_target_state =
                            TRAJECTORY_RUNNER_PEN_UP;
                        runner->pen_action_active = false;
                    }
                    else if (esp_timer_get_time() >=
                             runner->pen_action_deadline_us)
                    {
                        return trajectory_runner_fail(
                            runner,
                            TRAJECTORY_RUNNER_ERROR_PEN_TIMEOUT,
                            ESP_ERR_TIMEOUT,
                            "Final PEN_UP timed out");
                    }
                    else
                    {
                        vTaskDelayUntil(
                            &last_wake,
                            period_ticks);
                        continue;
                    }
                }

                if (!pen_control_is_up(&runner->pen))
                {
                    ret =
                        trajectory_runner_pen_start_action(
                            runner,
                            TRAJECTORY_RUNNER_PEN_UP);

                    if (ret != ESP_OK)
                    {
                        return trajectory_runner_fail(
                            runner,
                            TRAJECTORY_RUNNER_ERROR_PEN_COMMAND,
                            ret,
                            "Final PEN_UP command failed");
                    }

                    /* Final safety UP is handled here, not as a record. */
                    runner->phase =
                        TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING;

                    vTaskDelayUntil(
                        &last_wake,
                        period_ticks);
                    continue;
                }
            }

            trajectory_runner_force_logical_pen_up(runner);
            runner->wait_active = false;

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
            runner->phase = TRAJECTORY_RUNNER_PHASE_NONE;
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

        vTaskDelayUntil(&last_wake, period_ticks);
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
        status->error = TRAJECTORY_RUNNER_ERROR_INVALID_ARGUMENT;
        status->last_esp_err = ESP_ERR_INVALID_ARG;
        return;
    }

    status->state = runner->state;
    status->phase = runner->phase;
    status->error = runner->error;
    status->last_esp_err = runner->last_esp_err;
    status->control_cycles = runner->control_cycles;
    status->stop_requested = runner->stop_requested;
    status->header = runner->header;

    status->current_record_valid = runner->current_record.valid;
    status->current_record_index = runner->current_record.index;
    status->current_record_type =
        runner->current_record.valid
            ? runner->current_record.record.type
            : TRAJECTORY_RECORD_NONE;

    status->next_record_valid = runner->next_record.valid;
    status->next_record_index = runner->next_record.index;
    status->next_record_type =
        runner->next_record.valid
            ? runner->next_record.record.type
            : TRAJECTORY_RECORD_NONE;

    status->pen_state = runner->pen_state;
    status->pen_target_state = runner->pen_target_state;
    status->pen_enabled = runner->config.pen_enabled;
    status->pen_initialized = runner->pen_initialized;
    status->pen_action_active = runner->pen_action_active;

    if (runner->pen_action_active)
    {
        const int64_t now_us =
            esp_timer_get_time();

        const int64_t elapsed_us =
            now_us - runner->pen_action_start_us;

        const int64_t remaining_us =
            runner->pen_action_deadline_us - now_us;

        status->pen_action_elapsed_ms =
            (elapsed_us > 0)
                ? (uint32_t)(elapsed_us / 1000LL)
                : 0U;

        status->pen_action_remaining_ms =
            (remaining_us > 0)
                ? (uint32_t)(remaining_us / 1000LL)
                : 0U;
    }

    status->barrier_settled = runner->barrier_settled;
    status->barrier_settle_elapsed_ms =
        runner->barrier_settle_elapsed_ms;
    status->wait_active = runner->wait_active;

    if (runner->wait_active)
    {
        const int64_t remaining =
            runner->wait_deadline_us - esp_timer_get_time();

        status->wait_remaining_s =
            (remaining > 0)
                ? (float)remaining / 1000000.0f
                : 0.0f;
    }

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
    else
    {
        status->tracker = runner->last_tracker_status;
    }

    /* Report the underlying actuator state even when initialization failed.
     * Otherwise N3 can only expose the generic PEN_INIT runner error. */
    if (runner->config.pen_enabled)
    {
        pen_control_get_status(
            &runner->pen,
            &status->pen);
    }

    chassis_odometry_get_state(&status->odometry);
    motor_get_status(&status->motor);
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


const char *trajectory_runner_phase_name(
    trajectory_runner_phase_t phase)
{
    switch (phase)
    {
        case TRAJECTORY_RUNNER_PHASE_NONE:
            return "NONE";
        case TRAJECTORY_RUNNER_PHASE_DISPATCH:
            return "DISPATCH";
        case TRAJECTORY_RUNNER_PHASE_MOTION:
            return "MOTION";
        case TRAJECTORY_RUNNER_PHASE_BARRIER_SETTLING:
            return "BARRIER_SETTLING";
        case TRAJECTORY_RUNNER_PHASE_PEN_ACTUATING:
            return "PEN_ACTUATING";
        case TRAJECTORY_RUNNER_PHASE_WAIT:
            return "WAIT";
        case TRAJECTORY_RUNNER_PHASE_FINAL_SETTLING:
            return "FINAL_SETTLING";
        default:
            return "UNKNOWN";
    }
}


const char *trajectory_runner_pen_state_name(
    trajectory_runner_pen_state_t state)
{
    switch (state)
    {
        case TRAJECTORY_RUNNER_PEN_UP:
            return "UP";
        case TRAJECTORY_RUNNER_PEN_DOWN:
            return "DOWN";
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
        case TRAJECTORY_RUNNER_ERROR_RECORD_READ:
            return "RECORD_READ";
        case TRAJECTORY_RUNNER_ERROR_RECORD_TYPE:
            return "RECORD_TYPE";
        case TRAJECTORY_RUNNER_ERROR_MOTION_CONVERSION:
            return "MOTION_CONVERSION";
        case TRAJECTORY_RUNNER_ERROR_EXECUTOR_BEGIN:
            return "EXECUTOR_BEGIN";
        case TRAJECTORY_RUNNER_ERROR_EXECUTOR_ADVANCE:
            return "EXECUTOR_ADVANCE";
        case TRAJECTORY_RUNNER_ERROR_EXECUTOR_FINISH:
            return "EXECUTOR_FINISH";
        case TRAJECTORY_RUNNER_ERROR_EVENT:
            return "EVENT";
        case TRAJECTORY_RUNNER_ERROR_BARRIER_TIMEOUT:
            return "BARRIER_TIMEOUT";
        case TRAJECTORY_RUNNER_ERROR_PEN_INIT:
            return "PEN_INIT";
        case TRAJECTORY_RUNNER_ERROR_PEN_NOT_READY:
            return "PEN_NOT_READY";
        case TRAJECTORY_RUNNER_ERROR_PEN_COMMAND:
            return "PEN_COMMAND";
        case TRAJECTORY_RUNNER_ERROR_PEN_UPDATE:
            return "PEN_UPDATE";
        case TRAJECTORY_RUNNER_ERROR_PEN_TIMEOUT:
            return "PEN_TIMEOUT";
        case TRAJECTORY_RUNNER_ERROR_MOTOR_STALL:
            return "MOTOR_STALL";
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
