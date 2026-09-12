#include "n3_runner_bridge.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "trajectory_decoder.h"
#include "trajectory_executor.h"
#include "n3_build_config.h"
#include "n3_hardware.h"

#if N3_ENABLE_MOTION
#include "trajectory_runner.h"
#endif


#ifndef N3_RUNNER_TASK_STACK_SIZE
#define N3_RUNNER_TASK_STACK_SIZE (12U * 1024U)
#endif

#ifndef N3_RUNNER_TASK_PRIORITY
#define N3_RUNNER_TASK_PRIORITY 5
#endif

#ifndef N3_EVENT_WAIT_SLICE_MS
#define N3_EVENT_WAIT_SLICE_MS 50U
#endif

#ifndef N3_EVENT_MAX_WAIT_MS
#define N3_EVENT_MAX_WAIT_MS (5U * 60U * 1000U)
#endif

#if N3_ENABLE_CIRCLE
#define N3_MOTION_MAX_LINE_DISTANCE_MM 800.0f
#define N3_MOTION_MAX_TOTAL_DISTANCE_MM 30000.0f
#define N3_MOTION_MAX_RECORDS 1024U
#define N3_MOTION_RUN_TIMEOUT_MS (20U * 60U * 1000U)
#define N3_MOTION_MAX_SPEED_MM_S 250.0f
#else
#define N3_MOTION_MAX_LINE_DISTANCE_MM 500.0f
#define N3_MOTION_MAX_TOTAL_DISTANCE_MM 1000.0f
#define N3_MOTION_MAX_RECORDS 8U
#define N3_MOTION_RUN_TIMEOUT_MS 30000U
#define N3_MOTION_MAX_SPEED_MM_S 60.0f
#endif

#ifndef N3_CIRCLE_MIN_RADIUS_MM
#define N3_CIRCLE_MIN_RADIUS_MM 30.0f
#endif

#ifndef N3_CIRCLE_MAX_RADIUS_MM
#define N3_CIRCLE_MAX_RADIUS_MM 5000.0f
#endif

#ifndef N3_CIRCLE_MAX_SWEEP_DEG
#define N3_CIRCLE_MAX_SWEEP_DEG 360.0f
#endif

#ifndef N3_CIRCLE_MAX_ARC_LENGTH_MM
#define N3_CIRCLE_MAX_ARC_LENGTH_MM 3000.0f
#endif

#ifndef N3_MOTION_PATH_CONTINUITY_TOLERANCE_MM
#define N3_MOTION_PATH_CONTINUITY_TOLERANCE_MM 5.0f
#endif

#ifndef N3_DEG_TO_RAD
#define N3_DEG_TO_RAD 0.01745329251994329577f
#endif

#ifndef N3_MOTION_MIN_SPEED_MM_S
#define N3_MOTION_MIN_SPEED_MM_S 30.0f
#endif

#ifndef N3_LINE_MAX_ACCELERATION_MM_S2
#define N3_LINE_MAX_ACCELERATION_MM_S2 1000.0f
#endif

#ifndef N3_LINE_LINK_TIMEOUT_MS
/*
 * The host sends a PING once per second.  Leave enough headroom for a
 * temporarily delayed serial STATUS burst and for the host's ACK wait, while
 * still stopping promptly if the USB link is actually lost.  The former
 * 2.5-second interval could expire during normal pen settle / final-trim
 * traffic even with a healthy link.
 */
#define N3_LINE_LINK_TIMEOUT_MS 6000U
#endif

#ifndef N3_LINE_WATCHDOG_PERIOD_MS
#define N3_LINE_WATCHDOG_PERIOD_MS 50U
#endif

#ifndef N3_SIMULATION_PERIOD_MS
#define N3_SIMULATION_PERIOD_MS 10U
#endif


typedef struct
{
    SemaphoreHandle_t mutex;
    TaskHandle_t task;
    n3_runner_state_t state;
    bool stop_requested;
    bool emergency_stop;
    bool pen_down;
    uint32_t record_index;
    uint32_t record_count;
    bool pose_valid;
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vx_world_mm_s;
    float vy_world_mm_s;
    float w_rad_s;
    char path[128];
    char error[64];

#if N3_ENABLE_MOTION
    trajectory_runner_t motion_runner;
    bool motion_runner_initialized;
    int64_t last_transport_us;
#endif

} n3_runner_context_t;


static n3_runner_context_t s_runner;

static void copy_text(
    char *destination,
    size_t destination_size,
    const char *source);


#if N3_ENABLE_MOTION
static esp_err_t preflight_motion_path_file(
    const char *path,
    uint32_t *record_count,
    const char **error_code,
    const char **error_message)
{
    trajectory_decoder_t decoder = TRAJECTORY_DECODER_INITIALIZER;
    trajectory_file_header_t header = {0};
    float previous_x_mm = 0.0f;
    float previous_y_mm = 0.0f;
    float total_distance_mm = 0.0f;
    esp_err_t ret = trajectory_decoder_open(&decoder, path);

    if (ret != ESP_OK)
    {
        *error_code = "TRJ_INVALID";
        *error_message = "motion runner could not open the trajectory";
        return ret;
    }

    ret = trajectory_decoder_get_header(&decoder, &header);
    if ((ret != ESP_OK) ||
        (header.version != TRAJECTORY_FILE_VERSION_V2) ||
        (header.record_count == 0U))
    {
        trajectory_decoder_close(&decoder);
        *error_code = "TRJ_INVALID";
        *error_message = (ret == ESP_OK)
            ? "motion profile requires at least one TRJ2 record"
            : "motion runner could not decode the TRJ2 header";
        return (ret == ESP_OK) ? ESP_ERR_INVALID_SIZE : ret;
    }

    if (header.record_count > N3_MOTION_MAX_RECORDS)
    {
        trajectory_decoder_close(&decoder);
        *error_code = "TOO_MANY_SEGMENTS";
        *error_message = "motion profile accepts too many records";
        return ESP_ERR_NOT_SUPPORTED;
    }

    previous_x_mm = header.start_x_mm;
    previous_y_mm = header.start_y_mm;
    for (uint32_t index = 0U; index < header.record_count; ++index)
    {
        trajectory_record_t record = {0};
        if (!trajectory_decoder_has_next(&decoder) ||
            (trajectory_decoder_read_next_record(&decoder, &record) != ESP_OK))
        {
            trajectory_decoder_close(&decoder);
            *error_code = "TRJ_INVALID";
            *error_message = "motion runner could not decode a path record";
            return ESP_FAIL;
        }

        float segment_distance_mm = 0.0f;

        const bool is_event_record =
            (record.type == TRAJECTORY_RECORD_PEN_UP) ||
            (record.type == TRAJECTORY_RECORD_PEN_DOWN) ||
            (record.type == TRAJECTORY_RECORD_WAIT);

        if (record.type == TRAJECTORY_RECORD_LINE)
        {
            const float dx = record.payload.line.end_x_mm - previous_x_mm;
            const float dy = record.payload.line.end_y_mm - previous_y_mm;
            segment_distance_mm = sqrtf((dx * dx) + (dy * dy));
            if (!isfinite(segment_distance_mm) ||
                !(segment_distance_mm > 0.0f) ||
                (segment_distance_mm > N3_MOTION_MAX_LINE_DISTANCE_MM))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "LINE_TOO_LONG";
                *error_message = "each LINE length must be greater than zero and within the profile limit";
                return ESP_ERR_INVALID_SIZE;
            }

            previous_x_mm = record.payload.line.end_x_mm;
            previous_y_mm = record.payload.line.end_y_mm;
        }
        else if (record.type == TRAJECTORY_RECORD_CIRCLE)
        {
#if N3_ENABLE_CIRCLE
            const trajectory_circle_t *circle = &record.payload.circle;
            const float radius_mm = circle->radius_mm;
            const float sweep_deg = circle->sweep_deg;
            if (!isfinite(circle->center_x_mm) ||
                !isfinite(circle->center_y_mm) ||
                !isfinite(radius_mm) ||
                !isfinite(circle->start_angle_deg) ||
                !isfinite(sweep_deg) ||
                !(radius_mm > N3_CIRCLE_MIN_RADIUS_MM) ||
                (radius_mm > N3_CIRCLE_MAX_RADIUS_MM))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "CIRCLE_RADIUS_LIMIT";
                *error_message = "CIRCLE radius must be greater than 30 mm and within the profile limit";
                return ESP_ERR_INVALID_ARG;
            }

            if (!(fabsf(sweep_deg) > 0.0f) ||
                (fabsf(sweep_deg) > N3_CIRCLE_MAX_SWEEP_DEG))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "CIRCLE_SWEEP_LIMIT";
                *error_message = "CIRCLE sweep must be non-zero and at most 360 degrees";
                return ESP_ERR_INVALID_ARG;
            }

            const float start_rad = circle->start_angle_deg * N3_DEG_TO_RAD;
            const float start_x_mm = circle->center_x_mm + radius_mm * cosf(start_rad);
            const float start_y_mm = circle->center_y_mm + radius_mm * sinf(start_rad);
            const float continuity_mm = sqrtf(
                ((start_x_mm - previous_x_mm) * (start_x_mm - previous_x_mm)) +
                ((start_y_mm - previous_y_mm) * (start_y_mm - previous_y_mm)));
            if (!isfinite(continuity_mm) ||
                (continuity_mm > N3_MOTION_PATH_CONTINUITY_TOLERANCE_MM))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "PATH_DISCONTINUITY";
                *error_message = "CIRCLE start must match the preceding path endpoint";
                return ESP_ERR_INVALID_ARG;
            }

            segment_distance_mm = radius_mm * fabsf(sweep_deg) * N3_DEG_TO_RAD;
            if (!isfinite(segment_distance_mm) ||
                (segment_distance_mm > N3_CIRCLE_MAX_ARC_LENGTH_MM))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "CIRCLE_TOO_LONG";
                *error_message = "CIRCLE arc length exceeds the profile limit";
                return ESP_ERR_INVALID_SIZE;
            }

            const float end_rad = (circle->start_angle_deg + sweep_deg) * N3_DEG_TO_RAD;
            previous_x_mm = circle->center_x_mm + radius_mm * cosf(end_rad);
            previous_y_mm = circle->center_y_mm + radius_mm * sinf(end_rad);
#else
            trajectory_decoder_close(&decoder);
            *error_code = "UNSUPPORTED_RECORD";
            *error_message = "motion-line profile accepts only LINE records";
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }
        else if (is_event_record)
        {
#if N3_ENABLE_PEN
            if ((record.type == TRAJECTORY_RECORD_WAIT) &&
                (!isfinite(record.payload.wait.duration_s) ||
                 (record.payload.wait.duration_s >
                  ((float)N3_EVENT_MAX_WAIT_MS / 1000.0f))))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "WAIT_LIMIT";
                *error_message = "WAIT duration exceeds the active profile limit";
                return ESP_ERR_INVALID_ARG;
            }
#else
            trajectory_decoder_close(&decoder);
            *error_code = "PEN_DISABLED";
            *error_message = "PEN_UP, PEN_DOWN, and WAIT require the pen profile";
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }
        else
        {
            trajectory_decoder_close(&decoder);
            *error_code = "UNSUPPORTED_RECORD";
            *error_message = "motion profile supports LINE, guarded CIRCLE, and pen events only";
            return ESP_ERR_NOT_SUPPORTED;
        }

        if (!is_event_record)
        {
            total_distance_mm += segment_distance_mm;
            if (!isfinite(total_distance_mm) ||
                (total_distance_mm > N3_MOTION_MAX_TOTAL_DISTANCE_MM))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "TRAJECTORY_TOO_LONG";
                *error_message = "total motion distance exceeds the profile limit";
                return ESP_ERR_INVALID_SIZE;
            }

            if (!isfinite(record.speed_mm_s) ||
                (record.speed_mm_s < N3_MOTION_MIN_SPEED_MM_S) ||
                (record.speed_mm_s > N3_MOTION_MAX_SPEED_MM_S))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "SPEED_LIMIT";
                *error_message = "motion speed is outside the active profile limit";
                return ESP_ERR_INVALID_ARG;
            }

            if (!isfinite(record.acceleration_mm_s2) ||
                (record.acceleration_mm_s2 <= 0.0f) ||
                (record.acceleration_mm_s2 > N3_LINE_MAX_ACCELERATION_MM_S2))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "ACCEL_LIMIT";
                *error_message = "motion acceleration must be within (0, 1000] mm/s2";
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    trajectory_decoder_close(&decoder);

    *record_count = header.record_count;
    return ESP_OK;
}


static void configure_motion_runner(trajectory_runner_config_t *config)
{
    trajectory_runner_get_default_config(config);
    config->control_period_ms = 10U;
    config->ready_timeout_ms = 5000U;
    config->run_timeout_ms = N3_MOTION_RUN_TIMEOUT_MS;
    config->auto_init_hardware = false;
    config->reset_odometry_to_trajectory_start = false;
    config->pen_enabled = false;
#if N3_ENABLE_PEN
    /* Provisional production wiring.  GPIO45 is the dedicated servo signal;
     * the servo must use an external 5 V supply with common ground. */
    config->pen_enabled = true;
    pen_control_get_default_config(&config->pen);
    config->pen.gpio_num = GPIO_NUM_45;
    config->pen.timer_num = LEDC_TIMER_3;
    config->pen.channel = LEDC_CHANNEL_7;
    config->pen.positions_calibrated = true;
    config->pen.up_pulse_us = 1540U;
    config->pen.down_pulse_us = 1100U;
    config->pen.raise_time_ms = 250U;
    config->pen.lower_time_ms = 300U;
    config->pen.settle_time_ms = 120U;
    config->pen_action_timeout_ms = 2000U;
#endif
    config->debug_log_enable = true;
    config->debug_log_period_ms = 200U;

    config->tracker.position_kp = 1.20f;
    config->tracker.yaw_kp = 2.00f;
    config->tracker.max_feedback_speed_mm_s = 250.0f;
    config->tracker.max_translation_speed_mm_s = 250.0f;
    config->tracker.max_angular_speed_rad_s = 0.60f;
    config->tracker.max_tracking_error_mm = 100.0f;
    /* I2C/encoder updates can occasionally be delayed under real motor load.
     * 400 ms still fails safe, while avoiding false faults from a single
     * scheduling or bus-latency spike. */
    config->tracker.odometry_timeout_ms = 400U;
    config->tracker.start_position_tolerance_mm = 15.0f;
    config->tracker.start_yaw_tolerance_deg = 5.0f;
    config->tracker.position_tolerance_mm = 8.0f;
    config->tracker.yaw_tolerance_deg = 4.0f;
    config->tracker.settling_min_speed_mm_s = 30.0f;
    config->tracker.settling_max_speed_mm_s = 35.0f;
    config->tracker.settle_time_ms = 300U;
    config->tracker.hold_position_tolerance_mm = 8.0f;
    config->tracker.hold_yaw_tolerance_deg = 4.0f;
    config->tracker.hold_min_speed_mm_s = 30.0f;
    config->tracker.hold_max_speed_mm_s = 35.0f;
    config->tracker.hold_settle_time_ms = 100U;
    config->tracker.motor_stall_fault_enable = true;

    /*
     * The runner's Motion -> Event barrier and the tracker's hold controller
     * must use the same completion criteria.  If the tracker stops correcting
     * at 8 mm while the runner still waits for its 5 mm default, PEN/WAIT can
     * deadlock until BARRIER_TIMEOUT.
     */
    config->event_position_tolerance_mm =
        config->tracker.hold_position_tolerance_mm;
    config->event_yaw_tolerance_deg =
        config->tracker.hold_yaw_tolerance_deg;
    config->event_settle_time_ms =
        config->tracker.hold_settle_time_ms;
    config->event_barrier_timeout_ms = 8000U;
}


static void motion_runner_task(void *argument)
{
    (void)argument;
    char path[sizeof(s_runner.path)] = {0};

    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        copy_text(path, sizeof(path), s_runner.path);
        s_runner.state = N3_RUNNER_STATE_RUNNING;
        xSemaphoreGive(s_runner.mutex);
    }

    const esp_err_t ret = trajectory_runner_run_file(
        &s_runner.motion_runner,
        path);
    trajectory_runner_status_t runner_status = {0};
    trajectory_runner_get_status(&s_runner.motion_runner, &runner_status);

    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_runner.emergency_stop)
        {
            s_runner.state = N3_RUNNER_STATE_ESTOPPED;
        }
        else if (s_runner.stop_requested)
        {
            s_runner.state = N3_RUNNER_STATE_STOPPED;
        }
        else if ((ret == ESP_OK) &&
                 (runner_status.state == TRAJECTORY_RUNNER_FINISHED))
        {
            s_runner.state = N3_RUNNER_STATE_FINISHED;
        }
        else
        {
            s_runner.state = N3_RUNNER_STATE_ERROR;
            copy_text(
                s_runner.error,
                sizeof(s_runner.error),
                trajectory_runner_error_name(runner_status.error));
        }

        s_runner.stop_requested = false;
        s_runner.vx_world_mm_s = 0.0f;
        s_runner.vy_world_mm_s = 0.0f;
        s_runner.w_rad_s = 0.0f;
        s_runner.task = NULL;
        xSemaphoreGive(s_runner.mutex);
    }

    vTaskDelete(NULL);
}


static void motion_watchdog_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        bool active = false;
        bool emergency_stop = false;
        int64_t last_transport_us = 0;

        if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
        {
            active = (s_runner.task != NULL) &&
                     ((s_runner.state == N3_RUNNER_STATE_STARTING) ||
                      (s_runner.state == N3_RUNNER_STATE_RUNNING));
            emergency_stop = s_runner.emergency_stop;
            last_transport_us = s_runner.last_transport_us;
            xSemaphoreGive(s_runner.mutex);
        }

        if (active && !emergency_stop &&
            ((esp_timer_get_time() - last_transport_us) >
             ((int64_t)N3_LINE_LINK_TIMEOUT_MS * 1000LL)))
        {
            ESP_LOGE(
                "n3_motion_watchdog",
                "transport heartbeat timeout; emergency stopping motion");
            n3_hardware_stop(true);
            trajectory_runner_request_stop(&s_runner.motion_runner);

            if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
            {
                s_runner.emergency_stop = true;
                s_runner.stop_requested = true;
                s_runner.state = N3_RUNNER_STATE_ESTOPPED;
                copy_text(
                    s_runner.error,
                    sizeof(s_runner.error),
                    "transport heartbeat timeout");
                xSemaphoreGive(s_runner.mutex);
            }
        }

        if (active && !emergency_stop)
        {
            n3_hardware_status_t hardware = {0};
            n3_hardware_get_status(&hardware);
            if (!hardware.motor_ready || !hardware.odom_ready || hardware.estop)
            {
                ESP_LOGE(
                    "n3_motion_watchdog",
                    "hardware became unsafe; emergency stopping motion");
                n3_hardware_stop(true);
                trajectory_runner_request_stop(&s_runner.motion_runner);

                if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
                {
                    s_runner.emergency_stop = true;
                    s_runner.stop_requested = true;
                    s_runner.state = N3_RUNNER_STATE_ESTOPPED;
                    copy_text(
                        s_runner.error,
                        sizeof(s_runner.error),
                        "hardware became unavailable");
                    xSemaphoreGive(s_runner.mutex);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(N3_LINE_WATCHDOG_PERIOD_MS));
    }
}
#endif /* N3_ENABLE_MOTION */


static void copy_text(
    char *destination,
    size_t destination_size,
    const char *source)
{
    if ((destination == NULL) || (destination_size == 0U))
    {
        return;
    }

    if (source == NULL)
    {
        destination[0] = '\0';
        return;
    }

    (void)snprintf(destination, destination_size, "%s", source);
}


static void set_error_locked(
    const char *message)
{
    copy_text(s_runner.error, sizeof(s_runner.error), message);
    s_runner.state = N3_RUNNER_STATE_ERROR;
    s_runner.pen_down = false;
}


static bool is_motion_record(
    trajectory_record_type_t type)
{
    return (type == TRAJECTORY_RECORD_LINE) ||
           (type == TRAJECTORY_RECORD_CIRCLE) ||
           (type == TRAJECTORY_RECORD_CUBIC_BEZIER);
}


#if N3_ENABLE_SIMULATION
static bool is_simulation_motion_record(
    trajectory_record_type_t type)
{
    return (type == TRAJECTORY_RECORD_LINE) ||
           (type == TRAJECTORY_RECORD_CIRCLE);
}
#endif


static bool is_event_record(
    trajectory_record_type_t type)
{
    return (type == TRAJECTORY_RECORD_PEN_UP) ||
           (type == TRAJECTORY_RECORD_PEN_DOWN) ||
           (type == TRAJECTORY_RECORD_WAIT);
}


static esp_err_t preflight_event_file(
    const char *path,
    uint32_t *record_count,
    const char **error_code,
    const char **error_message)
{
    trajectory_decoder_t decoder = TRAJECTORY_DECODER_INITIALIZER;
    trajectory_file_header_t header = {0};
    uint64_t wait_ms = 0U;
    esp_err_t ret = trajectory_decoder_open(&decoder, path);

    if (ret != ESP_OK)
    {
        *error_code = "TRJ_INVALID";
        *error_message = "event runner could not open the validated trajectory";
        return ret;
    }

    ret = trajectory_decoder_get_header(&decoder, &header);
    if ((ret != ESP_OK) || (header.version != TRAJECTORY_FILE_VERSION_V2) ||
        (header.record_count == 0U))
    {
        trajectory_decoder_close(&decoder);
        *error_code = "TRJ_INVALID";
        *error_message = "event runner requires a non-empty TRJ2 file";
        return (ret == ESP_OK) ? ESP_ERR_NOT_SUPPORTED : ret;
    }

    while (trajectory_decoder_has_next(&decoder))
    {
        trajectory_record_t record = {0};
        ret = trajectory_decoder_read_next_record(&decoder, &record);
        if (ret != ESP_OK)
        {
            trajectory_decoder_close(&decoder);
            *error_code = "TRJ_INVALID";
            *error_message = "event runner could not decode a TRJ2 record";
            return ret;
        }

        if (is_motion_record(record.type))
        {
#if N3_ENABLE_SIMULATION
            if (!is_simulation_motion_record(record.type))
            {
                trajectory_decoder_close(&decoder);
                *error_code = "UNSUPPORTED_RECORD";
                *error_message = "simulation profile does not support CUBIC_BEZIER";
                return ESP_ERR_NOT_SUPPORTED;
            }
            /* LINE/CIRCLE are accepted motion records in simulation; do not
             * fall through to the event-only validation below. */
            continue;
#else
            trajectory_decoder_close(&decoder);
            *error_code = "MOTION_DISABLED";
            *error_message = "safe firmware accepts only event records";
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }

        if (!is_event_record(record.type))
        {
            trajectory_decoder_close(&decoder);
            *error_code = "TRJ_INVALID";
            *error_message = "TRJ2 contains an unsupported event record";
            return ESP_ERR_NOT_SUPPORTED;
        }

        if (record.type == TRAJECTORY_RECORD_WAIT)
        {
            if (!isfinite(record.payload.wait.duration_s) ||
                record.payload.wait.duration_s < 0.0f)
            {
                trajectory_decoder_close(&decoder);
                *error_code = "TRJ_INVALID";
                *error_message = "WAIT duration is invalid";
                return ESP_ERR_INVALID_RESPONSE;
            }

            wait_ms += (uint64_t)(record.payload.wait.duration_s * 1000.0f);
            if (wait_ms > N3_EVENT_MAX_WAIT_MS)
            {
                trajectory_decoder_close(&decoder);
                *error_code = "RUN_TIMEOUT";
                *error_message = "event-only WAIT duration exceeds the safety limit";
                return ESP_ERR_TIMEOUT;
            }
        }
    }

    trajectory_decoder_close(&decoder);
    *record_count = header.record_count;
    return ESP_OK;
}


static bool stop_requested(
    bool *emergency)
{
    bool stop = false;

    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        stop = s_runner.stop_requested;
        if (emergency != NULL)
        {
            *emergency = s_runner.emergency_stop;
        }
        xSemaphoreGive(s_runner.mutex);
    }

    return stop;
}


static bool wait_with_stop_checks(
    uint32_t duration_ms)
{
    uint32_t remaining = duration_ms;

    while (remaining > 0U)
    {
        if (stop_requested(NULL))
        {
            return false;
        }

        const uint32_t slice =
            (remaining < N3_EVENT_WAIT_SLICE_MS)
                ? remaining
                : N3_EVENT_WAIT_SLICE_MS;
        vTaskDelay(pdMS_TO_TICKS(slice));
        remaining -= slice;
    }

    return true;
}


static void finish_task(
    n3_runner_state_t terminal_state,
    const char *error)
{
    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_runner.emergency_stop)
        {
            s_runner.state = N3_RUNNER_STATE_ESTOPPED;
        }
        else if (error != NULL)
        {
            set_error_locked(error);
        }
        else
        {
            s_runner.state = terminal_state;
            s_runner.pen_down = false;
        }

        /* The task has now consumed the stop request and fully exited. */
        s_runner.stop_requested = false;
        s_runner.vx_world_mm_s = 0.0f;
        s_runner.vy_world_mm_s = 0.0f;
        s_runner.w_rad_s = 0.0f;
        s_runner.task = NULL;
        xSemaphoreGive(s_runner.mutex);
    }
}


#if !N3_ENABLE_SIMULATION
static void event_runner_task(
    void *argument)
{
    (void)argument;

    char path[sizeof(s_runner.path)] = {0};
    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        copy_text(path, sizeof(path), s_runner.path);
        s_runner.state = N3_RUNNER_STATE_RUNNING;
        xSemaphoreGive(s_runner.mutex);
    }

    trajectory_decoder_t decoder = TRAJECTORY_DECODER_INITIALIZER;
    esp_err_t ret = trajectory_decoder_open(&decoder, path);
    if (ret != ESP_OK)
    {
        finish_task(N3_RUNNER_STATE_ERROR, "event runner failed to open trajectory");
        vTaskDelete(NULL);
        return;
    }

    while (trajectory_decoder_has_next(&decoder))
    {
        bool emergency = false;
        if (stop_requested(&emergency))
        {
            trajectory_decoder_close(&decoder);
            finish_task(
                emergency ? N3_RUNNER_STATE_ESTOPPED : N3_RUNNER_STATE_STOPPED,
                NULL);
            vTaskDelete(NULL);
            return;
        }

        trajectory_record_t record = {0};
        ret = trajectory_decoder_read_next_record(&decoder, &record);
        if (ret != ESP_OK)
        {
            trajectory_decoder_close(&decoder);
            finish_task(N3_RUNNER_STATE_ERROR, "event runner record decode failed");
            vTaskDelete(NULL);
            return;
        }

        if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
        {
            s_runner.record_index = decoder.next_segment_index - 1U;
            if (record.type == TRAJECTORY_RECORD_PEN_DOWN)
            {
                s_runner.pen_down = true;
            }
            else if (record.type == TRAJECTORY_RECORD_PEN_UP)
            {
                s_runner.pen_down = false;
            }
            xSemaphoreGive(s_runner.mutex);
        }

        if (record.type == TRAJECTORY_RECORD_WAIT)
        {
            const uint32_t wait_ms =
                (uint32_t)(record.payload.wait.duration_s * 1000.0f);
            if (!wait_with_stop_checks(wait_ms))
            {
                bool emergency = false;
                (void)stop_requested(&emergency);
                trajectory_decoder_close(&decoder);
                finish_task(
                    emergency ? N3_RUNNER_STATE_ESTOPPED : N3_RUNNER_STATE_STOPPED,
                    NULL);
                vTaskDelete(NULL);
                return;
            }
        }
        else
        {
            /* Yield so STOP/ESTOP can be processed between zero-time events. */
            vTaskDelay(1);
        }
    }

    trajectory_decoder_close(&decoder);
    finish_task(N3_RUNNER_STATE_FINISHED, NULL);
    vTaskDelete(NULL);
}
#endif /* !N3_ENABLE_SIMULATION */


#if N3_ENABLE_SIMULATION

typedef struct
{
    bool valid;
    uint32_t index;
    trajectory_record_t record;
} simulation_record_slot_t;


static void simulation_slot_clear(
    simulation_record_slot_t *slot)
{
    if (slot != NULL)
    {
        memset(slot, 0, sizeof(*slot));
    }
}


static esp_err_t simulation_read_slot(
    trajectory_decoder_t *decoder,
    simulation_record_slot_t *slot)
{
    if ((decoder == NULL) || (slot == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    simulation_slot_clear(slot);
    if (!trajectory_decoder_has_next(decoder))
    {
        return ESP_OK;
    }

    slot->index = trajectory_decoder_get_next_record_index(decoder);
    const esp_err_t ret =
        trajectory_decoder_read_next_record(decoder, &slot->record);
    if (ret != ESP_OK)
    {
        simulation_slot_clear(slot);
        return ret;
    }

    slot->valid = true;
    return ESP_OK;
}


static bool simulation_slot_is_motion(
    const simulation_record_slot_t *slot)
{
    return (slot != NULL) && slot->valid &&
           is_simulation_motion_record(slot->record.type);
}


static esp_err_t simulation_record_to_motion(
    const trajectory_record_t *record,
    trajectory_segment_t *motion)
{
    if ((record == NULL) || (motion == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(motion, 0, sizeof(*motion));
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

        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}


static esp_err_t simulation_shift_window(
    trajectory_decoder_t *decoder,
    simulation_record_slot_t *current,
    simulation_record_slot_t *next)
{
    if ((decoder == NULL) || (current == NULL) || (next == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *current = *next;
    return simulation_read_slot(decoder, next);
}


static esp_err_t simulation_begin_current_motion(
    trajectory_executor_t *executor,
    const simulation_record_slot_t *current,
    const simulation_record_slot_t *next)
{
    if (!simulation_slot_is_motion(current))
    {
        return ESP_ERR_INVALID_STATE;
    }

    trajectory_segment_t current_motion = {0};
    esp_err_t ret = simulation_record_to_motion(
        &current->record,
        &current_motion);
    if (ret != ESP_OK)
    {
        return ret;
    }

    trajectory_segment_t lookahead_motion = {0};
    const trajectory_segment_t *lookahead = NULL;
    uint32_t lookahead_index = 0U;
    if (simulation_slot_is_motion(next))
    {
        ret = simulation_record_to_motion(
            &next->record,
            &lookahead_motion);
        if (ret != ESP_OK)
        {
            return ret;
        }
        lookahead = &lookahead_motion;
        lookahead_index = next->index;
    }

    return trajectory_executor_begin_motion(
        executor,
        &current_motion,
        current->index,
        lookahead,
        lookahead_index);
}


static void simulation_update_pose(
    const trajectory_reference_t *reference,
    float yaw_deg)
{
    if ((reference == NULL) ||
        (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) != pdTRUE))
    {
        return;
    }

    s_runner.pose_valid = reference->valid;
    s_runner.x_mm = reference->x_mm;
    s_runner.y_mm = reference->y_mm;
    s_runner.yaw_deg = yaw_deg;
    s_runner.vx_world_mm_s = reference->tangent_x * reference->speed_mm_s;
    s_runner.vy_world_mm_s = reference->tangent_y * reference->speed_mm_s;
    s_runner.w_rad_s = 0.0f;
    xSemaphoreGive(s_runner.mutex);
}


static void simulation_runner_task(
    void *argument)
{
    (void)argument;

    char path[sizeof(s_runner.path)] = {0};
    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        copy_text(path, sizeof(path), s_runner.path);
        s_runner.state = N3_RUNNER_STATE_RUNNING;
        xSemaphoreGive(s_runner.mutex);
    }

    trajectory_decoder_t decoder = TRAJECTORY_DECODER_INITIALIZER;
    trajectory_executor_t executor = TRAJECTORY_EXECUTOR_INITIALIZER;
    trajectory_file_header_t header = {0};
    simulation_record_slot_t current = {0};
    simulation_record_slot_t next = {0};

    esp_err_t ret = trajectory_decoder_open(&decoder, path);
    if (ret != ESP_OK)
    {
        finish_task(N3_RUNNER_STATE_ERROR, "simulation runner failed to open trajectory");
        vTaskDelete(NULL);
        return;
    }

    ret = trajectory_decoder_get_header(&decoder, &header);
    if (ret != ESP_OK ||
        trajectory_executor_init_external(&executor, &header) != ESP_OK ||
        trajectory_executor_start(&executor) != ESP_OK)
    {
        trajectory_decoder_close(&decoder);
        finish_task(N3_RUNNER_STATE_ERROR, "simulation executor initialization failed");
        vTaskDelete(NULL);
        return;
    }

    ret = simulation_read_slot(&decoder, &current);
    if (ret == ESP_OK)
    {
        ret = simulation_read_slot(&decoder, &next);
    }
    if (ret != ESP_OK)
    {
        trajectory_decoder_close(&decoder);
        finish_task(N3_RUNNER_STATE_ERROR, "simulation runner could not read records");
        vTaskDelete(NULL);
        return;
    }

    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
    {
        s_runner.pose_valid = true;
        s_runner.x_mm = header.start_x_mm;
        s_runner.y_mm = header.start_y_mm;
        s_runner.yaw_deg = header.start_yaw_deg;
        s_runner.vx_world_mm_s = 0.0f;
        s_runner.vy_world_mm_s = 0.0f;
        s_runner.w_rad_s = 0.0f;
        xSemaphoreGive(s_runner.mutex);
    }

    for (;;)
    {
        bool emergency = false;
        if (stop_requested(&emergency))
        {
            trajectory_decoder_close(&decoder);
            finish_task(
                emergency ? N3_RUNNER_STATE_ESTOPPED : N3_RUNNER_STATE_STOPPED,
                NULL);
            vTaskDelete(NULL);
            return;
        }

        const trajectory_executor_state_t executor_state =
            trajectory_executor_get_state(&executor);

        if (executor_state == TRAJECTORY_EXECUTOR_RUNNING)
        {
            trajectory_reference_t reference = {0};
            ret = trajectory_executor_update(
                &executor,
                (float)N3_SIMULATION_PERIOD_MS / 1000.0f,
                &reference);
            if (ret != ESP_OK)
            {
                trajectory_decoder_close(&decoder);
                finish_task(N3_RUNNER_STATE_ERROR, "simulation executor update failed");
                vTaskDelete(NULL);
                return;
            }

            if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
            {
                s_runner.record_index = current.index;
                xSemaphoreGive(s_runner.mutex);
            }
            simulation_update_pose(&reference, header.start_yaw_deg);

            if (trajectory_executor_needs_advance(&executor))
            {
                ret = simulation_shift_window(&decoder, &current, &next);
                if (ret == ESP_OK && simulation_slot_is_motion(&current))
                {
                    trajectory_segment_t lookahead_motion = {0};
                    const trajectory_segment_t *lookahead = NULL;
                    uint32_t lookahead_index = 0U;
                    if (simulation_slot_is_motion(&next))
                    {
                        ret = simulation_record_to_motion(
                            &next.record,
                            &lookahead_motion);
                        lookahead = &lookahead_motion;
                        lookahead_index = next.index;
                    }
                    if (ret == ESP_OK)
                    {
                        ret = trajectory_executor_advance_motion(
                            &executor,
                            lookahead,
                            lookahead_index);
                    }
                }
                if (ret != ESP_OK)
                {
                    trajectory_decoder_close(&decoder);
                    finish_task(N3_RUNNER_STATE_ERROR, "simulation motion transition failed");
                    vTaskDelete(NULL);
                    return;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(N3_SIMULATION_PERIOD_MS));
            continue;
        }

        if (executor_state == TRAJECTORY_EXECUTOR_HOLDING)
        {
            if (!current.valid)
            {
                ret = trajectory_executor_finish(&executor);
                trajectory_decoder_close(&decoder);
                if (ret == ESP_OK)
                {
                    finish_task(N3_RUNNER_STATE_FINISHED, NULL);
                }
                else
                {
                    finish_task(N3_RUNNER_STATE_ERROR, "simulation executor could not finish");
                }
                vTaskDelete(NULL);
                return;
            }

            if (simulation_slot_is_motion(&current))
            {
                /*
                 * A motion may follow an event record.  In that case the
                 * executor is still valid and HOLDING, but its current index
                 * belongs to the previous motion.  Only advance the executor
                 * when the slot already represents that same cached motion.
                 */
                if (!executor.current_valid ||
                    executor.current_index != current.index)
                {
                    ret = simulation_begin_current_motion(
                        &executor,
                        &current,
                        &next);
                }
                else
                {
                    /* The previous motion reached HOLDING before an event/EOF. */
                    ret = simulation_shift_window(&decoder, &current, &next);
                }
                if (ret != ESP_OK)
                {
                    trajectory_decoder_close(&decoder);
                    finish_task(N3_RUNNER_STATE_ERROR, "simulation motion start failed");
                    vTaskDelete(NULL);
                    return;
                }
                continue;
            }

            if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
            {
                s_runner.record_index = current.index;
                if (current.record.type == TRAJECTORY_RECORD_PEN_UP)
                {
                    s_runner.pen_down = false;
                }
                else if (current.record.type == TRAJECTORY_RECORD_PEN_DOWN)
                {
                    s_runner.pen_down = true;
                }
                xSemaphoreGive(s_runner.mutex);
            }

            if (current.record.type == TRAJECTORY_RECORD_WAIT)
            {
                const uint32_t wait_ms =
                    (uint32_t)(current.record.payload.wait.duration_s * 1000.0f);
                if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
                {
                    s_runner.state = N3_RUNNER_STATE_WAITING;
                    xSemaphoreGive(s_runner.mutex);
                }
                if (!wait_with_stop_checks(wait_ms))
                {
                    (void)stop_requested(&emergency);
                    trajectory_decoder_close(&decoder);
                    finish_task(
                        emergency ? N3_RUNNER_STATE_ESTOPPED : N3_RUNNER_STATE_STOPPED,
                        NULL);
                    vTaskDelete(NULL);
                    return;
                }
                if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE)
                {
                    s_runner.state = N3_RUNNER_STATE_RUNNING;
                    xSemaphoreGive(s_runner.mutex);
                }
            }

            ret = simulation_shift_window(&decoder, &current, &next);
            if (ret != ESP_OK)
            {
                trajectory_decoder_close(&decoder);
                finish_task(N3_RUNNER_STATE_ERROR, "simulation runner could not advance records");
                vTaskDelete(NULL);
                return;
            }
            continue;
        }

        trajectory_decoder_close(&decoder);
        finish_task(N3_RUNNER_STATE_ERROR, "simulation executor entered an invalid state");
        vTaskDelete(NULL);
        return;
    }
}

#endif /* N3_ENABLE_SIMULATION */


esp_err_t n3_runner_bridge_init(void)
{
    if (s_runner.mutex != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_runner, 0, sizeof(s_runner));
    s_runner.mutex = xSemaphoreCreateMutex();
    if (s_runner.mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_runner.state = N3_RUNNER_STATE_IDLE;
    s_runner.pose_valid = (N3_ENABLE_SIMULATION != 0);
    s_runner.yaw_deg = 0.0f;

#if N3_ENABLE_MOTION
    trajectory_runner_config_t config = {0};
    configure_motion_runner(&config);
    const esp_err_t runner_ret = trajectory_runner_init(
        &s_runner.motion_runner,
        &config);
    if (runner_ret != ESP_OK)
    {
        vSemaphoreDelete(s_runner.mutex);
        s_runner.mutex = NULL;
        return runner_ret;
    }
    s_runner.motion_runner_initialized = true;
    s_runner.last_transport_us = esp_timer_get_time();
    if (xTaskCreate(
            motion_watchdog_task,
            "n3_motion_watchdog",
            3072U,
            NULL,
            6U,
            NULL) != pdPASS)
    {
        vSemaphoreDelete(s_runner.mutex);
        s_runner.mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
#endif
    return ESP_OK;
}


void n3_runner_bridge_note_transport_activity(void)
{
#if N3_ENABLE_MOTION
    if ((s_runner.mutex != NULL) &&
        (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE))
    {
        s_runner.last_transport_us = esp_timer_get_time();
        xSemaphoreGive(s_runner.mutex);
    }
#endif
}


esp_err_t n3_runner_bridge_start_event_only(
    const char *path,
    const char **error_code,
    const char **error_message)
{
    static const char *const INTERNAL_CODE = "INTERNAL";
    static const char *const INTERNAL_MESSAGE = "runner bridge is not initialized";

    if ((error_code == NULL) || (error_message == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *error_code = INTERNAL_CODE;
    *error_message = INTERNAL_MESSAGE;

    if ((path == NULL) || (path[0] == '\0') || (s_runner.mutex == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t record_count = 0U;
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;
#if N3_ENABLE_MOTION
    ret = preflight_motion_path_file(
        path,
        &record_count,
        error_code,
        error_message);
#else
    ret = preflight_event_file(
        path,
        &record_count,
        error_code,
        error_message);
#endif
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) != pdTRUE)
    {
        *error_code = INTERNAL_CODE;
        *error_message = "event runner mutex unavailable";
        return ESP_ERR_TIMEOUT;
    }

    if ((s_runner.state == N3_RUNNER_STATE_STARTING) ||
        (s_runner.state == N3_RUNNER_STATE_RUNNING) ||
        (s_runner.task != NULL))
    {
        xSemaphoreGive(s_runner.mutex);
        *error_code = "RUNNER_BUSY";
        *error_message = "another trajectory is already running";
        return ESP_ERR_INVALID_STATE;
    }

    if (s_runner.state == N3_RUNNER_STATE_ESTOPPED)
    {
        xSemaphoreGive(s_runner.mutex);
        *error_code = "ESTOP_ACTIVE";
        *error_message = "clear emergency stop before running";
        return ESP_ERR_INVALID_STATE;
    }

    copy_text(s_runner.path, sizeof(s_runner.path), path);
    s_runner.state = N3_RUNNER_STATE_STARTING;
    s_runner.stop_requested = false;
    s_runner.emergency_stop = false;
    s_runner.pen_down = false;
    s_runner.record_index = 0U;
    s_runner.record_count = record_count;
    s_runner.error[0] = '\0';

#if N3_ENABLE_MOTION
    const TaskFunction_t runner_task = motion_runner_task;
    const char *runner_task_name =
#if N3_ENABLE_CIRCLE
        "n3_path_runner";
#else
        "n3_line_runner";
#endif
#elif N3_ENABLE_SIMULATION
    const TaskFunction_t runner_task = simulation_runner_task;
    const char *runner_task_name = "n3_sim_runner";
#else
    const TaskFunction_t runner_task = event_runner_task;
    const char *runner_task_name = "n3_event_runner";
#endif

    const BaseType_t created = xTaskCreate(
        runner_task,
        runner_task_name,
        N3_RUNNER_TASK_STACK_SIZE,
        NULL,
        N3_RUNNER_TASK_PRIORITY,
        &s_runner.task);

    if (created != pdPASS)
    {
        s_runner.task = NULL;
        set_error_locked("could not create event runner task");
        xSemaphoreGive(s_runner.mutex);
        *error_code = INTERNAL_CODE;
        *error_message = "could not create event runner task";
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_runner.mutex);
    return ESP_OK;
}


void n3_runner_bridge_request_stop(
    bool emergency)
{
    if (s_runner.mutex == NULL ||
        xSemaphoreTake(s_runner.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    const bool task_active = (s_runner.task != NULL);
    s_runner.stop_requested = true;
    if (emergency)
    {
        s_runner.emergency_stop = true;
        s_runner.state = N3_RUNNER_STATE_ESTOPPED;
        s_runner.pen_down = false;
    }
    else if ((s_runner.state == N3_RUNNER_STATE_STARTING) ||
             (s_runner.state == N3_RUNNER_STATE_RUNNING))
    {
        s_runner.state = N3_RUNNER_STATE_STOPPED;
        s_runner.pen_down = false;
    }
    else if (s_runner.task == NULL)
    {
        /* STOP after FINISHED/ERROR is a completed no-op, not a latched stop. */
        s_runner.stop_requested = false;
        if (s_runner.state != N3_RUNNER_STATE_ESTOPPED)
        {
            s_runner.state = N3_RUNNER_STATE_STOPPED;
            s_runner.pen_down = false;
        }
    }

    xSemaphoreGive(s_runner.mutex);

#if N3_ENABLE_MOTION
    if (task_active && s_runner.motion_runner_initialized)
    {
        trajectory_runner_request_stop(&s_runner.motion_runner);
    }
#endif
}


void n3_runner_bridge_clear_estop(void)
{
    if (s_runner.mutex == NULL ||
        xSemaphoreTake(s_runner.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    if ((s_runner.task == NULL) &&
        (s_runner.state != N3_RUNNER_STATE_STARTING) &&
        (s_runner.state != N3_RUNNER_STATE_RUNNING) &&
        (s_runner.state != N3_RUNNER_STATE_WAITING))
    {
        /* CLEAR_ESTOP is also the explicit software re-arm operation.  It
         * releases a terminal ERROR/STOPPED/FINISHED state after the motor
         * driver has been re-enabled by n3_hardware_clear_estop(), allowing
         * the next uploaded job without a board reset. */
        s_runner.state = N3_RUNNER_STATE_IDLE;
        s_runner.stop_requested = false;
        s_runner.emergency_stop = false;
        s_runner.pen_down = false;
        s_runner.error[0] = '\0';
    }

    xSemaphoreGive(s_runner.mutex);
}


void n3_runner_bridge_reset_pose(
    float x_mm,
    float y_mm,
    float yaw_deg)
{
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(yaw_deg) ||
        s_runner.mutex == NULL ||
        xSemaphoreTake(s_runner.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    if ((s_runner.state != N3_RUNNER_STATE_STARTING) &&
        (s_runner.state != N3_RUNNER_STATE_RUNNING) &&
        (s_runner.state != N3_RUNNER_STATE_WAITING) &&
        (s_runner.task == NULL))
    {
        s_runner.pose_valid = true;
        s_runner.x_mm = x_mm;
        s_runner.y_mm = y_mm;
        s_runner.yaw_deg = yaw_deg;
        s_runner.vx_world_mm_s = 0.0f;
        s_runner.vy_world_mm_s = 0.0f;
        s_runner.w_rad_s = 0.0f;
    }

    xSemaphoreGive(s_runner.mutex);
}


bool n3_runner_bridge_is_active(void)
{
    bool active = false;
    if ((s_runner.mutex != NULL) &&
        (xSemaphoreTake(s_runner.mutex, portMAX_DELAY) == pdTRUE))
    {
        active = (s_runner.state == N3_RUNNER_STATE_STARTING) ||
                 (s_runner.state == N3_RUNNER_STATE_RUNNING) ||
                 (s_runner.task != NULL);
        xSemaphoreGive(s_runner.mutex);
    }
    return active;
}


void n3_runner_bridge_get_status(
    n3_runner_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    memset(status, 0, sizeof(*status));
#if N3_ENABLE_MOTION
    trajectory_runner_status_t motion_status = {0};
#endif
    status->valid = (s_runner.mutex != NULL);
    if (!status->valid ||
        xSemaphoreTake(s_runner.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    status->state = s_runner.state;
    status->stop_requested = s_runner.stop_requested;
    status->emergency_stop = s_runner.emergency_stop;
    status->pen_down = s_runner.pen_down;
    status->pen_enabled = (N3_ENABLE_PEN != 0);
    copy_text(status->pen_state, sizeof(status->pen_state), status->pen_down ? "DOWN" : "UP");
    copy_text(status->pen_target, sizeof(status->pen_target), status->pen_down ? "DOWN" : "UP");
    copy_text(status->pen_error, sizeof(status->pen_error), "NONE");
    status->record_index = s_runner.record_index;
    status->record_count = s_runner.record_count;
    status->pose_valid = s_runner.pose_valid;
    status->x_mm = s_runner.x_mm;
    status->y_mm = s_runner.y_mm;
    status->yaw_deg = s_runner.yaw_deg;
    status->vx_world_mm_s = s_runner.vx_world_mm_s;
    status->vy_world_mm_s = s_runner.vy_world_mm_s;
    status->w_rad_s = s_runner.w_rad_s;
#if N3_ENABLE_MOTION
    if (s_runner.motion_runner_initialized)
    {
        trajectory_runner_get_status(
            &s_runner.motion_runner,
            &motion_status);
        status->record_index = motion_status.current_record_valid
            ? motion_status.current_record_index
            : 0U;
        status->record_count = motion_status.header.record_count;
        copy_text(
            status->record_type,
            sizeof(status->record_type),
            motion_status.current_record_valid
                ? trajectory_decoder_record_type_name(
                    motion_status.current_record_type)
                : "NONE");
        status->tracking_error_mm = motion_status.tracker.position_error_mm;
        status->tracking_yaw_error_deg = motion_status.tracker.yaw_error_deg;
        status->reference_x_mm = motion_status.tracker.reference_x_mm;
        status->reference_y_mm = motion_status.tracker.reference_y_mm;
        status->command_vx_body_mm_s = motion_status.tracker.command_vx_body_mm_s;
        status->command_vy_body_mm_s = motion_status.tracker.command_vy_body_mm_s;
        status->command_w_rad_s = motion_status.tracker.command_w_rad_s;
        status->settle_elapsed_ms = motion_status.tracker.settle_elapsed_ms;
        status->wheel_a_target_mm_s = motion_status.motor.A.target_speed_mm_s;
        status->wheel_a_actual_mm_s = motion_status.motor.A.actual_speed_mm_s;
        status->wheel_a_pwm = motion_status.motor.A.output_pwm;
        status->wheel_a_stall_suspected = motion_status.motor.A.stall_suspected;
        status->wheel_a_stall_elapsed_ms = motion_status.motor.A.stall_elapsed_ms;
        status->wheel_b_target_mm_s = motion_status.motor.B.target_speed_mm_s;
        status->wheel_b_actual_mm_s = motion_status.motor.B.actual_speed_mm_s;
        status->wheel_b_pwm = motion_status.motor.B.output_pwm;
        status->wheel_b_stall_suspected = motion_status.motor.B.stall_suspected;
        status->wheel_b_stall_elapsed_ms = motion_status.motor.B.stall_elapsed_ms;
        status->wheel_d_target_mm_s = motion_status.motor.D.target_speed_mm_s;
        status->wheel_d_actual_mm_s = motion_status.motor.D.actual_speed_mm_s;
        status->wheel_d_pwm = motion_status.motor.D.output_pwm;
        status->wheel_d_stall_suspected = motion_status.motor.D.stall_suspected;
        status->wheel_d_stall_elapsed_ms = motion_status.motor.D.stall_elapsed_ms;
        status->pen_enabled = motion_status.pen_enabled;
        status->pen_busy = motion_status.pen_action_active || motion_status.pen.busy;
        status->pen_settling = motion_status.pen.settling;
        status->pen_current_pulse_us = motion_status.pen.current_pulse_us;
        status->pen_target_pulse_us = motion_status.pen.target_pulse_us;
        status->pen_down =
            (motion_status.pen_state == TRAJECTORY_RUNNER_PEN_DOWN);
        copy_text(status->pen, sizeof(status->pen), status->pen_down ? "DOWN" : "UP");
        copy_text(
            status->pen_state,
            sizeof(status->pen_state),
            motion_status.pen_enabled
                ? pen_control_state_name(motion_status.pen.state)
                : trajectory_runner_pen_state_name(motion_status.pen_state));
        copy_text(
            status->pen_target,
            sizeof(status->pen_target),
            motion_status.pen_initialized
                ? trajectory_runner_pen_state_name(motion_status.pen_target_state)
                : trajectory_runner_pen_state_name(motion_status.pen_target_state));
        copy_text(
            status->pen_error,
            sizeof(status->pen_error),
            motion_status.pen_enabled
                ? pen_control_error_name(motion_status.pen.error)
                : "NONE");
        copy_text(
            status->phase,
            sizeof(status->phase),
            trajectory_runner_phase_name(motion_status.phase));
        copy_text(
            status->error_code,
            sizeof(status->error_code),
            trajectory_runner_error_name(motion_status.error));
        copy_text(
            status->tracker_error,
            sizeof(status->tracker_error),
            trajectory_tracker_error_name(motion_status.tracker.error));
    }
#endif
    copy_text(
        status->tracker,
        sizeof(status->tracker),
        (s_runner.state == N3_RUNNER_STATE_IDLE)
            ? "IDLE"
#if N3_ENABLE_MOTION
#if N3_ENABLE_CIRCLE
            : "PATH"
#else
            : "LINE"
#endif
#elif N3_ENABLE_SIMULATION
            : "SIMULATOR"
#elif N3_ENABLE_ROTATE_REL
            : "ROTATE_REL"
#elif N3_ENABLE_HARDWARE
            : "HARDWARE_CHECK"
#else
            : "EVENT_ONLY"
#endif
    );
#if N3_ENABLE_MOTION
    if (s_runner.motion_runner_initialized)
    {
        copy_text(
            status->tracker,
            sizeof(status->tracker),
            trajectory_tracker_state_name(motion_status.tracker.state));
    }
#endif
    copy_text(
        status->execution_mode,
        sizeof(status->execution_mode),
#if N3_ENABLE_SIMULATION
        "SIMULATED"
#elif N3_ENABLE_MOTION
#if N3_ENABLE_PEN
        "HARDWARE_DRAW"
#elif N3_ENABLE_CIRCLE
        "HARDWARE_PATH"
#else
        "HARDWARE_LINE"
#endif
#elif N3_ENABLE_ROTATE_REL
        "HARDWARE_ROTATE"
#elif N3_ENABLE_HARDWARE
        "HARDWARE_CHECK"
#else
        "EVENT_ONLY"
#endif
    );
#if !N3_ENABLE_MOTION
    copy_text(
        status->pen,
        sizeof(status->pen),
        s_runner.pen_down ? "DOWN" : "UP");
#endif
    copy_text(status->error, sizeof(status->error), s_runner.error);

    if (status->record_type[0] == '\0')
    {
        copy_text(status->record_type, sizeof(status->record_type), "NONE");
    }

    switch (s_runner.state)
    {
        case N3_RUNNER_STATE_STARTING:
            copy_text(status->runner, sizeof(status->runner), "STARTING");
            break;
        case N3_RUNNER_STATE_RUNNING:
            copy_text(status->runner, sizeof(status->runner), "RUNNING");
            break;
        case N3_RUNNER_STATE_WAITING:
            copy_text(status->runner, sizeof(status->runner), "WAITING");
            break;
        case N3_RUNNER_STATE_FINISHED:
            copy_text(status->runner, sizeof(status->runner), "FINISHED");
            break;
        case N3_RUNNER_STATE_STOPPED:
            copy_text(status->runner, sizeof(status->runner), "STOPPED");
            break;
        case N3_RUNNER_STATE_ESTOPPED:
            copy_text(status->runner, sizeof(status->runner), "ESTOPPED");
            break;
        case N3_RUNNER_STATE_ERROR:
            copy_text(status->runner, sizeof(status->runner), "ERROR");
            break;
        case N3_RUNNER_STATE_IDLE:
        default:
            copy_text(status->runner, sizeof(status->runner), "IDLE");
            break;
    }

    xSemaphoreGive(s_runner.mutex);
}
