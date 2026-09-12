#ifndef TRAJECTORY_TRACKER_H
#define TRAJECTORY_TRACKER_H

/*
 * trajectory_tracker.h
 *
 * V1 closed-loop trajectory tracker for the 3-wheel omni chassis.
 *
 * Responsibilities:
 *   trajectory_reference_t (WORLD frame)
 *      + chassis_odometry_state_t feedback
 *      -> trajectory feedforward
 *      -> 2-D WORLD-frame position P feedback
 *      -> fixed heading hold
 *      -> WORLD-to-BODY velocity conversion
 *      -> trajectory-level speed limiting
 *      -> safety/watchdog checks
 *      -> final settling
 *      -> motor_set_velocity(vx_body, vy_body, w)
 *
 * This module deliberately does NOT:
 *   - decode .traj files
 *   - know LINE/CIRCLE geometry
 *   - advance trajectory time or path distance
 *   - reset odometry/world coordinates
 *   - perform chassis inverse kinematics
 *   - perform wheel FF/PID/PWM control
 *   - control trajectory_executor state
 *
 * Coordinate convention:
 *   WORLD and BODY use the same handedness:
 *     +X / vx : forward at yaw = 0
 *     +Y / vy : left
 *     +Yaw / w: counter-clockwise
 *
 * Units:
 *   position              : mm
 *   linear velocity       : mm/s
 *   yaw angle             : degree
 *   angular velocity      : rad/s
 *   gains position_kp/yaw_kp : 1/s when used with mm and rad errors
 *
 * Startup contract:
 *   Before trajectory_tracker_start(), the application must align odometry
 *   WORLD coordinates with the trajectory header, typically:
 *
 *     trajectory_executor_start(&executor);
 *     trajectory_executor_get_header(&executor, &header);
 *     chassis_odometry_reset(
 *         header.start_x_mm,
 *         header.start_y_mm,
 *         header.start_yaw_deg);
 *     trajectory_tracker_start(&tracker, &header);
 *
 *   The tracker verifies the alignment but never calls
 *   chassis_odometry_reset() itself.
 *
 * Typical loop:
 *
 *     trajectory_reference_t ref;
 *
 *     trajectory_executor_update(&executor, dt_s, &ref);
 *     trajectory_tracker_update(&tracker, &ref, dt_s);
 *
 * Single-header usage:
 *
 *   In exactly ONE .c/.cpp file:
 *
 *     #define TRAJECTORY_TRACKER_IMPLEMENTATION
 *     #include "trajectory_tracker.h"
 *
 *   In all other .c/.cpp files:
 *
 *     #include "trajectory_tracker.h"
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "trajectory_executor.h"
#include "chassis_odometry.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================
 * 1. Configuration defaults
 * ============================================================ */

/*
 * These are conservative first-run values, not final tuning values.
 * Override them before including this header, or pass an explicit config.
 */

#ifndef TRAJECTORY_TRACKER_DEFAULT_POSITION_KP
#define TRAJECTORY_TRACKER_DEFAULT_POSITION_KP                 1.50f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_YAW_KP
#define TRAJECTORY_TRACKER_DEFAULT_YAW_KP                      2.50f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_MAX_FEEDBACK_SPEED_MM_S
#define TRAJECTORY_TRACKER_DEFAULT_MAX_FEEDBACK_SPEED_MM_S     600.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_MAX_TRANSLATION_SPEED_MM_S
#define TRAJECTORY_TRACKER_DEFAULT_MAX_TRANSLATION_SPEED_MM_S  1000.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_MAX_ANGULAR_SPEED_RAD_S
#define TRAJECTORY_TRACKER_DEFAULT_MAX_ANGULAR_SPEED_RAD_S     1.50f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_MAX_TRACKING_ERROR_MM
#define TRAJECTORY_TRACKER_DEFAULT_MAX_TRACKING_ERROR_MM       300.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_ODOMETRY_TIMEOUT_MS
#define TRAJECTORY_TRACKER_DEFAULT_ODOMETRY_TIMEOUT_MS         100U
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_START_POSITION_TOLERANCE_MM
#define TRAJECTORY_TRACKER_DEFAULT_START_POSITION_TOLERANCE_MM 30.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_START_YAW_TOLERANCE_DEG
#define TRAJECTORY_TRACKER_DEFAULT_START_YAW_TOLERANCE_DEG     5.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_POSITION_TOLERANCE_MM
#define TRAJECTORY_TRACKER_DEFAULT_POSITION_TOLERANCE_MM       10.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_YAW_TOLERANCE_DEG
#define TRAJECTORY_TRACKER_DEFAULT_YAW_TOLERANCE_DEG           3.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_SETTLE_TIME_MS
#define TRAJECTORY_TRACKER_DEFAULT_SETTLE_TIME_MS              300U
#endif

/* Tracker update dt safety guard. */
#ifndef TRAJECTORY_TRACKER_UPDATE_DT_MAX_S
#define TRAJECTORY_TRACKER_UPDATE_DT_MAX_S                     1.0f
#endif

/* Treat smaller speeds as stationary for reference validation/settling. */
#ifndef TRAJECTORY_TRACKER_SPEED_EPSILON_MM_S
#define TRAJECTORY_TRACKER_SPEED_EPSILON_MM_S                  1.0e-3f
#endif

/* Executor promises unit tangents while moving. Allow small numeric error. */
#ifndef TRAJECTORY_TRACKER_TANGENT_NORM_MIN
#define TRAJECTORY_TRACKER_TANGENT_NORM_MIN                    0.90f
#endif

#ifndef TRAJECTORY_TRACKER_TANGENT_NORM_MAX
#define TRAJECTORY_TRACKER_TANGENT_NORM_MAX                    1.10f
#endif

#define TRAJECTORY_TRACKER_PI                                  3.14159265358979323846f
#define TRAJECTORY_TRACKER_DEG_TO_RAD                          0.01745329251994329577f


#ifndef TRAJECTORY_TRACKER_DEFAULT_SETTLING_MIN_SPEED_MM_S
#define TRAJECTORY_TRACKER_DEFAULT_SETTLING_MIN_SPEED_MM_S      15.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_SETTLING_MAX_SPEED_MM_S
#define TRAJECTORY_TRACKER_DEFAULT_SETTLING_MAX_SPEED_MM_S      40.0f
#endif

/* Intermediate Motion -> Event holds need the same deadband-aware behavior as
 * final settling, but they must not mark the whole trajectory finished. */
#ifndef TRAJECTORY_TRACKER_DEFAULT_HOLD_POSITION_TOLERANCE_MM
#define TRAJECTORY_TRACKER_DEFAULT_HOLD_POSITION_TOLERANCE_MM    8.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_HOLD_YAW_TOLERANCE_DEG
#define TRAJECTORY_TRACKER_DEFAULT_HOLD_YAW_TOLERANCE_DEG        3.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_HOLD_MIN_SPEED_MM_S
#define TRAJECTORY_TRACKER_DEFAULT_HOLD_MIN_SPEED_MM_S            20.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_HOLD_MAX_SPEED_MM_S
#define TRAJECTORY_TRACKER_DEFAULT_HOLD_MAX_SPEED_MM_S            50.0f
#endif

#ifndef TRAJECTORY_TRACKER_DEFAULT_HOLD_SETTLE_TIME_MS
#define TRAJECTORY_TRACKER_DEFAULT_HOLD_SETTLE_TIME_MS            100U
#endif


/* ============================================================
 * 2. Public state / error types
 * ============================================================ */

typedef enum
{
    TRAJECTORY_TRACKER_IDLE = 0,
    TRAJECTORY_TRACKER_RUNNING,
    TRAJECTORY_TRACKER_HOLD_SETTLING,
    TRAJECTORY_TRACKER_SETTLING,
    TRAJECTORY_TRACKER_FINISHED,
    TRAJECTORY_TRACKER_ERROR

} trajectory_tracker_state_t;


typedef enum
{
    TRAJECTORY_TRACKER_ERROR_NONE = 0,
    TRAJECTORY_TRACKER_ERROR_NOT_INITIALIZED,
    TRAJECTORY_TRACKER_ERROR_INVALID_ARGUMENT,
    TRAJECTORY_TRACKER_ERROR_INVALID_CONFIG,
    TRAJECTORY_TRACKER_ERROR_MOTOR_NOT_READY,
    TRAJECTORY_TRACKER_ERROR_ODOMETRY_NOT_READY,
    TRAJECTORY_TRACKER_ERROR_ODOMETRY_STALE,
    TRAJECTORY_TRACKER_ERROR_START_POSE_MISMATCH,
    TRAJECTORY_TRACKER_ERROR_INVALID_REFERENCE,
    TRAJECTORY_TRACKER_ERROR_INVALID_DT,
    TRAJECTORY_TRACKER_ERROR_TRACKING_ERROR_EXCEEDED,
    TRAJECTORY_TRACKER_ERROR_MOTOR_STALL,
    TRAJECTORY_TRACKER_ERROR_NUMERIC

} trajectory_tracker_error_t;


/* ============================================================
 * 3. Tracker configuration
 * ============================================================ */

typedef struct
{
    /* WORLD-frame position P controller: v_fb = position_kp * position_error. */
    float position_kp;

    /* Fixed-heading P controller: w = yaw_kp * yaw_error_rad. */
    float yaw_kp;

    /* Limit feedback alone before adding trajectory feedforward. */
    float max_feedback_speed_mm_s;

    /* Limit final WORLD translation vector magnitude. */
    float max_translation_speed_mm_s;

    /* Limit absolute BODY angular command. */
    float max_angular_speed_rad_s;

    /* Runtime safety limit against loss of trajectory tracking. */
    float max_tracking_error_mm;

    /* Maximum accepted age of odometry snapshot. */
    uint32_t odometry_timeout_ms;

    /* Required tracker-start alignment with trajectory header. */
    float start_position_tolerance_mm;
    float start_yaw_tolerance_deg;

    /* Final-target settling tolerances. */
    float position_tolerance_mm;
    float yaw_tolerance_deg;

    /*
     * Terminal translation speed while SETTLING.
     *
     * If position error is still outside position_tolerance_mm,
     * keep enough translation speed to overcome drivetrain deadband.
     */
    float settling_min_speed_mm_s;
    float settling_max_speed_mm_s;

    uint32_t settle_time_ms;

    /* Intermediate event/WAIT hold parameters. */
    float hold_position_tolerance_mm;
    float hold_yaw_tolerance_deg;
    float hold_min_speed_mm_s;
    float hold_max_speed_mm_s;
    uint32_t hold_settle_time_ms;

    /* Convert motor stall diagnostics into a tracker fault. */
    bool motor_stall_fault_enable;

} trajectory_tracker_config_t;


#define TRAJECTORY_TRACKER_CONFIG_DEFAULT()                    \
    {                                                          \
        .position_kp =                                         \
            TRAJECTORY_TRACKER_DEFAULT_POSITION_KP,            \
        .yaw_kp =                                              \
            TRAJECTORY_TRACKER_DEFAULT_YAW_KP,                 \
        .max_feedback_speed_mm_s =                             \
            TRAJECTORY_TRACKER_DEFAULT_MAX_FEEDBACK_SPEED_MM_S, \
        .max_translation_speed_mm_s =                          \
            TRAJECTORY_TRACKER_DEFAULT_MAX_TRANSLATION_SPEED_MM_S, \
        .max_angular_speed_rad_s =                             \
            TRAJECTORY_TRACKER_DEFAULT_MAX_ANGULAR_SPEED_RAD_S, \
        .max_tracking_error_mm =                               \
            TRAJECTORY_TRACKER_DEFAULT_MAX_TRACKING_ERROR_MM,  \
        .odometry_timeout_ms =                                 \
            TRAJECTORY_TRACKER_DEFAULT_ODOMETRY_TIMEOUT_MS,    \
        .start_position_tolerance_mm =                         \
            TRAJECTORY_TRACKER_DEFAULT_START_POSITION_TOLERANCE_MM, \
        .start_yaw_tolerance_deg =                             \
            TRAJECTORY_TRACKER_DEFAULT_START_YAW_TOLERANCE_DEG, \
        .position_tolerance_mm =                               \
            TRAJECTORY_TRACKER_DEFAULT_POSITION_TOLERANCE_MM,  \
        .yaw_tolerance_deg =                                   \
            TRAJECTORY_TRACKER_DEFAULT_YAW_TOLERANCE_DEG,      \
        .settling_min_speed_mm_s =                             \
            TRAJECTORY_TRACKER_DEFAULT_SETTLING_MIN_SPEED_MM_S, \
        .settling_max_speed_mm_s =                             \
            TRAJECTORY_TRACKER_DEFAULT_SETTLING_MAX_SPEED_MM_S, \
        .settle_time_ms =                                      \
            TRAJECTORY_TRACKER_DEFAULT_SETTLE_TIME_MS,         \
        .hold_position_tolerance_mm =                          \
            TRAJECTORY_TRACKER_DEFAULT_HOLD_POSITION_TOLERANCE_MM, \
        .hold_yaw_tolerance_deg =                              \
            TRAJECTORY_TRACKER_DEFAULT_HOLD_YAW_TOLERANCE_DEG, \
        .hold_min_speed_mm_s =                                 \
            TRAJECTORY_TRACKER_DEFAULT_HOLD_MIN_SPEED_MM_S,    \
        .hold_max_speed_mm_s =                                 \
            TRAJECTORY_TRACKER_DEFAULT_HOLD_MAX_SPEED_MM_S,    \
        .hold_settle_time_ms =                                 \
            TRAJECTORY_TRACKER_DEFAULT_HOLD_SETTLE_TIME_MS,    \
        .motor_stall_fault_enable = true                       \
    }


/* ============================================================
 * 4. Runtime status for logging/tuning
 * ============================================================ */

typedef struct
{
    trajectory_tracker_state_t state;
    trajectory_tracker_error_t error;

    /* Frozen heading target from trajectory header. */
    float yaw_target_deg;

    /* Latest reference. */
    bool reference_valid;
    bool trajectory_end;
    uint32_t segment_index;
    float reference_x_mm;
    float reference_y_mm;
    float reference_speed_mm_s;
    float reference_tangent_x;
    float reference_tangent_y;

    /* Latest coherent odometry snapshot. */
    float odom_x_mm;
    float odom_y_mm;
    float odom_yaw_deg;
    uint32_t odom_update_count;
    int64_t odom_timestamp_us;
    int64_t odom_age_us;

    /* Tracking error in WORLD frame. */
    float error_x_mm;
    float error_y_mm;
    float position_error_mm;
    float yaw_error_deg;

    /* Feedforward and feedback in WORLD frame. */
    float ff_vx_world_mm_s;
    float ff_vy_world_mm_s;
    float fb_vx_world_mm_s;
    float fb_vy_world_mm_s;

    /* Final limited translation command in WORLD frame. */
    float command_vx_world_mm_s;
    float command_vy_world_mm_s;

    /* Command actually sent to motor_control in BODY frame. */
    float command_vx_body_mm_s;
    float command_vy_body_mm_s;
    float command_w_rad_s;

    /* Continuous in-tolerance time while HOLD_SETTLING/SETTLING. */
    uint32_t settle_elapsed_ms;
    bool hold_active;

} trajectory_tracker_status_t;


/* ============================================================
 * 5. Tracker instance
 * ============================================================ */

typedef struct
{
    bool initialized;

    trajectory_tracker_config_t config;

    trajectory_tracker_state_t state;
    trajectory_tracker_error_t error;

    /* Trajectory start pose copied at start(). */
    float start_x_mm;
    float start_y_mm;
    float yaw_target_deg;

    /* Seconds continuously inside final tolerances. */
    float settle_elapsed_s;

    bool hold_requested;

    trajectory_tracker_status_t status;

} trajectory_tracker_t;


#define TRAJECTORY_TRACKER_INITIALIZER                         \
    {                                                          \
        .initialized = false,                                  \
        .config = {0},                                         \
        .state = TRAJECTORY_TRACKER_IDLE,                      \
        .error = TRAJECTORY_TRACKER_ERROR_NONE,                \
        .start_x_mm = 0.0f,                                    \
        .start_y_mm = 0.0f,                                    \
        .yaw_target_deg = 0.0f,                                \
        .settle_elapsed_s = 0.0f,                              \
        .hold_requested = false,                               \
        .status = {0}                                          \
    }


/* ============================================================
 * 6. Public API
 * ============================================================ */

/**
 * @brief Initialize one tracker instance with an explicit configuration.
 *
 * No motor command is issued here.
 */
esp_err_t trajectory_tracker_init(
    trajectory_tracker_t *tracker,
    const trajectory_tracker_config_t *config);


/**
 * @brief Start/restart tracking for a trajectory header.
 *
 * The application must already have aligned odometry WORLD pose with the
 * header. This function verifies start position/yaw and refuses to start if
 * the mismatch exceeds configured tolerances.
 *
 * This function never resets odometry and never starts/stops the executor.
 */
esp_err_t trajectory_tracker_start(
    trajectory_tracker_t *tracker,
    const trajectory_file_header_t *header);


/**
 * Request/release a deadband-aware stationary hold. The hold state is used
 * before intermediate PEN/WAIT events and never marks the whole trajectory
 * finished.
 */
esp_err_t trajectory_tracker_set_hold_mode(
    trajectory_tracker_t *tracker,
    bool enabled);


/**
 * @brief Consume one executor reference, close the tracking loop, and command
 *        motor_control.
 *
 * Call once per trajectory control cycle after trajectory_executor_update().
 * Executor PAUSED references (speed=0, valid=true) remain actively position
 * held while tracker stays RUNNING.
 */
esp_err_t trajectory_tracker_update(
    trajectory_tracker_t *tracker,
    const trajectory_reference_t *reference,
    float dt_s);


/**
 * @brief Normal stop: command zero velocity and return tracker to IDLE.
 */
void trajectory_tracker_stop(
    trajectory_tracker_t *tracker);


bool trajectory_tracker_is_running(
    const trajectory_tracker_t *tracker);


bool trajectory_tracker_is_finished(
    const trajectory_tracker_t *tracker);


trajectory_tracker_state_t trajectory_tracker_get_state(
    const trajectory_tracker_t *tracker);


trajectory_tracker_error_t trajectory_tracker_get_error(
    const trajectory_tracker_t *tracker);


void trajectory_tracker_get_status(
    const trajectory_tracker_t *tracker,
    trajectory_tracker_status_t *status);


const char *trajectory_tracker_state_name(
    trajectory_tracker_state_t state);


const char *trajectory_tracker_error_name(
    trajectory_tracker_error_t error);


#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */

#ifdef TRAJECTORY_TRACKER_IMPLEMENTATION

#ifdef __cplusplus
extern "C" {
#endif

#define TRAJECTORY_TRACKER_TAG "traj_tracker"


static float trajectory_tracker_clampf(
    float value,
    float min_value,
    float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}


static float trajectory_tracker_wrap_deg_180(
    float angle_deg)
{
    angle_deg =
        fmodf(
            angle_deg + 180.0f,
            360.0f);

    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg - 180.0f;
}


static bool trajectory_tracker_config_is_valid(
    const trajectory_tracker_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if (!isfinite(config->position_kp) ||
        !isfinite(config->yaw_kp) ||
        !isfinite(config->max_feedback_speed_mm_s) ||
        !isfinite(config->max_translation_speed_mm_s) ||
        !isfinite(config->max_angular_speed_rad_s) ||
        !isfinite(config->max_tracking_error_mm) ||
        !isfinite(config->start_position_tolerance_mm) ||
        !isfinite(config->start_yaw_tolerance_deg) ||
        !isfinite(config->position_tolerance_mm) ||
        !isfinite(config->yaw_tolerance_deg) ||
        !isfinite(config->settling_min_speed_mm_s) ||
        !isfinite(config->settling_max_speed_mm_s) ||
        !isfinite(config->hold_position_tolerance_mm) ||
        !isfinite(config->hold_yaw_tolerance_deg) ||
        !isfinite(config->hold_min_speed_mm_s) ||
        !isfinite(config->hold_max_speed_mm_s))
    {
        return false;
    }

    if ((config->position_kp < 0.0f) ||
        (config->yaw_kp < 0.0f) ||
        (config->max_feedback_speed_mm_s < 0.0f) ||
        !(config->max_translation_speed_mm_s > 0.0f) ||
        (config->max_angular_speed_rad_s < 0.0f) ||
        !(config->max_tracking_error_mm > 0.0f) ||
        (config->odometry_timeout_ms == 0U) ||
        (config->start_position_tolerance_mm < 0.0f) ||
        (config->start_yaw_tolerance_deg < 0.0f) ||
        (config->position_tolerance_mm < 0.0f) ||
        (config->yaw_tolerance_deg < 0.0f) ||
        (config->settling_min_speed_mm_s < 0.0f) ||
        !(config->settling_max_speed_mm_s > 0.0f) ||
        (config->settling_min_speed_mm_s >
         config->settling_max_speed_mm_s) ||
        (config->settle_time_ms == 0U) ||
        (config->hold_position_tolerance_mm < 0.0f) ||
        (config->hold_yaw_tolerance_deg < 0.0f) ||
        (config->hold_min_speed_mm_s < 0.0f) ||
        !(config->hold_max_speed_mm_s > 0.0f) ||
        (config->hold_min_speed_mm_s >
         config->hold_max_speed_mm_s) ||
        (config->hold_settle_time_ms == 0U))
    {
        return false;
    }

    return true;
}


static bool trajectory_tracker_header_is_valid(
    const trajectory_file_header_t *header)
{
    if (header == NULL)
    {
        return false;
    }

    return
        isfinite(header->start_x_mm) &&
        isfinite(header->start_y_mm) &&
        isfinite(header->start_yaw_deg);
}


static bool trajectory_tracker_odom_is_finite(
    const chassis_odometry_state_t *odom)
{
    if (odom == NULL)
    {
        return false;
    }

    return
        isfinite(odom->x_mm) &&
        isfinite(odom->y_mm) &&
        isfinite(odom->yaw_deg) &&
        isfinite(odom->body_vx_mm_s) &&
        isfinite(odom->body_vy_mm_s) &&
        isfinite(odom->world_vx_mm_s) &&
        isfinite(odom->world_vy_mm_s) &&
        isfinite(odom->gyro_w_rad_s) &&
        isfinite(odom->encoder_w_rad_s);
}


static bool trajectory_tracker_reference_is_valid(
    const trajectory_reference_t *reference)
{
    if ((reference == NULL) || !reference->valid)
    {
        return false;
    }

    if (!isfinite(reference->x_mm) ||
        !isfinite(reference->y_mm) ||
        !isfinite(reference->tangent_x) ||
        !isfinite(reference->tangent_y) ||
        !isfinite(reference->speed_mm_s))
    {
        return false;
    }

    if (reference->speed_mm_s < 0.0f)
    {
        return false;
    }

    /*
     * A stationary empty trajectory may legitimately have tangent=(0,0).
     * A moving reference, however, must carry the unit tangent promised by
     * trajectory_executor.h.
     */
    if (reference->speed_mm_s >
        TRAJECTORY_TRACKER_SPEED_EPSILON_MM_S)
    {
        const float tangent_norm =
            sqrtf(
                reference->tangent_x * reference->tangent_x +
                reference->tangent_y * reference->tangent_y);

        if (!isfinite(tangent_norm) ||
            (tangent_norm < TRAJECTORY_TRACKER_TANGENT_NORM_MIN) ||
            (tangent_norm > TRAJECTORY_TRACKER_TANGENT_NORM_MAX))
        {
            return false;
        }
    }

    return true;
}


static void trajectory_tracker_limit_vector(
    float *x,
    float *y,
    float max_magnitude)
{
    if ((x == NULL) || (y == NULL))
    {
        return;
    }

    if (!(max_magnitude > 0.0f))
    {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }

    const float magnitude_sq =
        (*x) * (*x) +
        (*y) * (*y);

    const float max_sq =
        max_magnitude * max_magnitude;

    if (magnitude_sq <= max_sq)
    {
        return;
    }

    const float magnitude =
        sqrtf(magnitude_sq);

    if (!(magnitude > 0.0f) || !isfinite(magnitude))
    {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }

    const float scale =
        max_magnitude / magnitude;

    *x *= scale;
    *y *= scale;
}


static void trajectory_tracker_clear_status_command(
    trajectory_tracker_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    status->ff_vx_world_mm_s = 0.0f;
    status->ff_vy_world_mm_s = 0.0f;
    status->fb_vx_world_mm_s = 0.0f;
    status->fb_vy_world_mm_s = 0.0f;
    status->command_vx_world_mm_s = 0.0f;
    status->command_vy_world_mm_s = 0.0f;
    status->command_vx_body_mm_s = 0.0f;
    status->command_vy_body_mm_s = 0.0f;
    status->command_w_rad_s = 0.0f;
}


static esp_err_t trajectory_tracker_latch_error(
    trajectory_tracker_t *tracker,
    trajectory_tracker_error_t error,
    esp_err_t return_code,
    bool emergency_stop,
    const char *message)
{
    if (tracker != NULL)
    {
        tracker->state = TRAJECTORY_TRACKER_ERROR;
        tracker->error = error;
        tracker->settle_elapsed_s = 0.0f;

        tracker->status.state = tracker->state;
        tracker->status.error = tracker->error;
        tracker->status.settle_elapsed_ms = 0U;
        trajectory_tracker_clear_status_command(&tracker->status);
    }

    if (emergency_stop)
    {
        motor_emergency_stop();
    }
    else
    {
        motor_stop();
    }

    if (message != NULL)
    {
        ESP_LOGE(
            TRAJECTORY_TRACKER_TAG,
            "%s",
            message);
    }

    return return_code;
}


static esp_err_t trajectory_tracker_read_odom_checked(
    trajectory_tracker_t *tracker,
    chassis_odometry_state_t *odom,
    bool emergency_on_failure)
{
    if ((tracker == NULL) || (odom == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!chassis_odometry_is_ready())
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_ODOMETRY_NOT_READY,
            ESP_ERR_INVALID_STATE,
            emergency_on_failure,
            "Odometry is not ready");
    }

    chassis_odometry_get_state(odom);

    if (!odom->initialized || !odom->running)
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_ODOMETRY_NOT_READY,
            ESP_ERR_INVALID_STATE,
            emergency_on_failure,
            "Odometry snapshot is not running");
    }

    if (!trajectory_tracker_odom_is_finite(odom))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_NUMERIC,
            ESP_FAIL,
            emergency_on_failure,
            "Non-finite odometry value");
    }

    const int64_t now_us =
        esp_timer_get_time();

    const int64_t age_us =
        now_us - odom->timestamp_us;

    tracker->status.odom_x_mm = odom->x_mm;
    tracker->status.odom_y_mm = odom->y_mm;
    tracker->status.odom_yaw_deg = odom->yaw_deg;
    tracker->status.odom_update_count = odom->update_count;
    tracker->status.odom_timestamp_us = odom->timestamp_us;
    tracker->status.odom_age_us = age_us;

    const int64_t timeout_us =
        (int64_t)tracker->config.odometry_timeout_ms *
        1000LL;

    if ((odom->timestamp_us <= 0) ||
        (age_us < 0) ||
        (age_us > timeout_us))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_ODOMETRY_STALE,
            ESP_ERR_TIMEOUT,
            emergency_on_failure,
            "Odometry timestamp is stale");
    }

    return ESP_OK;
}


static void trajectory_tracker_update_status_reference(
    trajectory_tracker_t *tracker,
    const trajectory_reference_t *reference)
{
    if ((tracker == NULL) || (reference == NULL))
    {
        return;
    }

    tracker->status.reference_valid = reference->valid;
    tracker->status.trajectory_end = reference->trajectory_end;
    tracker->status.segment_index = reference->segment_index;
    tracker->status.reference_x_mm = reference->x_mm;
    tracker->status.reference_y_mm = reference->y_mm;
    tracker->status.reference_speed_mm_s = reference->speed_mm_s;
    tracker->status.reference_tangent_x = reference->tangent_x;
    tracker->status.reference_tangent_y = reference->tangent_y;
}


static esp_err_t trajectory_tracker_compute_and_command(
    trajectory_tracker_t *tracker,
    const trajectory_reference_t *reference,
    const chassis_odometry_state_t *odom,
    float dt_s)
{
    if ((tracker == NULL) ||
        (reference == NULL) ||
        (odom == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const float error_x_mm =
        reference->x_mm - odom->x_mm;

    const float error_y_mm =
        reference->y_mm - odom->y_mm;

    const float position_error_mm =
        sqrtf(
            error_x_mm * error_x_mm +
            error_y_mm * error_y_mm);

    const float yaw_error_deg =
        trajectory_tracker_wrap_deg_180(
            tracker->yaw_target_deg -
            odom->yaw_deg);

    if (!isfinite(error_x_mm) ||
        !isfinite(error_y_mm) ||
        !isfinite(position_error_mm) ||
        !isfinite(yaw_error_deg))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_NUMERIC,
            ESP_FAIL,
            true,
            "Non-finite tracking error");
    }

    tracker->status.error_x_mm = error_x_mm;
    tracker->status.error_y_mm = error_y_mm;
    tracker->status.position_error_mm = position_error_mm;
    tracker->status.yaw_error_deg = yaw_error_deg;

    if (position_error_mm >
        tracker->config.max_tracking_error_mm)
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_TRACKING_ERROR_EXCEEDED,
            ESP_FAIL,
            true,
            "Tracking error exceeded configured safety limit");
    }

    /*
     * trajectory_end owns final settling. A requested hold is used for an
     * intermediate Motion -> PEN/WAIT barrier and must never finish the
     * complete trajectory.
     */
    if (reference->trajectory_end)
    {
        tracker->state =
            TRAJECTORY_TRACKER_SETTLING;
    }
    else if (tracker->hold_requested)
    {
        tracker->state =
            TRAJECTORY_TRACKER_HOLD_SETTLING;
    }
    else if (tracker->state ==
             TRAJECTORY_TRACKER_HOLD_SETTLING)
    {
        tracker->state =
            TRAJECTORY_TRACKER_RUNNING;
        tracker->settle_elapsed_s = 0.0f;
    }

    float ff_vx_world_mm_s = 0.0f;
    float ff_vy_world_mm_s = 0.0f;

    if (tracker->state == TRAJECTORY_TRACKER_RUNNING)
    {
        ff_vx_world_mm_s =
            reference->tangent_x *
            reference->speed_mm_s;

        ff_vy_world_mm_s =
            reference->tangent_y *
            reference->speed_mm_s;
    }

    float fb_vx_world_mm_s =
        tracker->config.position_kp *
        error_x_mm;

    float fb_vy_world_mm_s =
        tracker->config.position_kp *
        error_y_mm;

    /*
     * Deadband-aware settling control.
     *
     * During normal tracking, keep the original 2-D P feedback unchanged.
     *
     * Once trajectory_end has been reached, or an intermediate hold has been
     * requested:
     *
     *   outside position tolerance:
     *       use P feedback direction, but clamp its magnitude to a useful
     *       terminal creep range so wheel commands do not disappear into
     *       the drivetrain low-speed deadband.
     *
     *   inside position tolerance:
     *       stop translation completely and let the settling timer /
     *       heading controller determine completion.
     */
    if ((tracker->state == TRAJECTORY_TRACKER_SETTLING) ||
        (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING))
    {
        const bool hold_settling =
            (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING);

        const float position_tolerance_mm =
            hold_settling
                ? tracker->config.hold_position_tolerance_mm
                : tracker->config.position_tolerance_mm;

        const float min_speed_mm_s =
            hold_settling
                ? tracker->config.hold_min_speed_mm_s
                : tracker->config.settling_min_speed_mm_s;

        const float max_speed_mm_s =
            hold_settling
                ? tracker->config.hold_max_speed_mm_s
                : tracker->config.settling_max_speed_mm_s;

        if (position_error_mm <=
            position_tolerance_mm)
        {
            fb_vx_world_mm_s = 0.0f;
            fb_vy_world_mm_s = 0.0f;
        }
        else if (position_error_mm > 1.0e-6f)
        {
            float settling_speed_mm_s =
                tracker->config.position_kp *
                position_error_mm;

            settling_speed_mm_s =
                trajectory_tracker_clampf(
                    settling_speed_mm_s,
                    min_speed_mm_s,
                    max_speed_mm_s);

            const float inv_error =
                1.0f / position_error_mm;

            fb_vx_world_mm_s =
                settling_speed_mm_s *
                error_x_mm *
                inv_error;

            fb_vy_world_mm_s =
                settling_speed_mm_s *
                error_y_mm *
                inv_error;
        }
        else
        {
            fb_vx_world_mm_s = 0.0f;
            fb_vy_world_mm_s = 0.0f;
        }
    }

    trajectory_tracker_limit_vector(
        &fb_vx_world_mm_s,
        &fb_vy_world_mm_s,
        tracker->config.max_feedback_speed_mm_s);


    float command_vx_world_mm_s =
        ff_vx_world_mm_s +
        fb_vx_world_mm_s;

    float command_vy_world_mm_s =
        ff_vy_world_mm_s +
        fb_vy_world_mm_s;

    trajectory_tracker_limit_vector(
        &command_vx_world_mm_s,
        &command_vy_world_mm_s,
        tracker->config.max_translation_speed_mm_s);

    const float theta_deg =
        trajectory_tracker_wrap_deg_180(
            odom->yaw_deg);

    const float theta_rad =
        theta_deg *
        TRAJECTORY_TRACKER_DEG_TO_RAD;

    const float c = cosf(theta_rad);
    const float s = sinf(theta_rad);

    const float command_vx_body_mm_s =
        c * command_vx_world_mm_s +
        s * command_vy_world_mm_s;

    const float command_vy_body_mm_s =
       -s * command_vx_world_mm_s +
        c * command_vy_world_mm_s;

    float command_w_rad_s =
        tracker->config.yaw_kp *
        yaw_error_deg *
        TRAJECTORY_TRACKER_DEG_TO_RAD;

    command_w_rad_s =
        trajectory_tracker_clampf(
            command_w_rad_s,
           -tracker->config.max_angular_speed_rad_s,
            tracker->config.max_angular_speed_rad_s);

    if (!isfinite(ff_vx_world_mm_s) ||
        !isfinite(ff_vy_world_mm_s) ||
        !isfinite(fb_vx_world_mm_s) ||
        !isfinite(fb_vy_world_mm_s) ||
        !isfinite(command_vx_world_mm_s) ||
        !isfinite(command_vy_world_mm_s) ||
        !isfinite(command_vx_body_mm_s) ||
        !isfinite(command_vy_body_mm_s) ||
        !isfinite(command_w_rad_s))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_NUMERIC,
            ESP_FAIL,
            true,
            "Non-finite tracker command");
    }

    tracker->status.ff_vx_world_mm_s =
        ff_vx_world_mm_s;
    tracker->status.ff_vy_world_mm_s =
        ff_vy_world_mm_s;
    tracker->status.fb_vx_world_mm_s =
        fb_vx_world_mm_s;
    tracker->status.fb_vy_world_mm_s =
        fb_vy_world_mm_s;
    tracker->status.command_vx_world_mm_s =
        command_vx_world_mm_s;
    tracker->status.command_vy_world_mm_s =
        command_vy_world_mm_s;
    tracker->status.command_vx_body_mm_s =
        command_vx_body_mm_s;
    tracker->status.command_vy_body_mm_s =
        command_vy_body_mm_s;
    tracker->status.command_w_rad_s =
        command_w_rad_s;

    if ((tracker->state == TRAJECTORY_TRACKER_SETTLING) ||
        (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING))
    {
        const bool hold_settling =
            (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING);

        const bool position_ok =
            position_error_mm <=
            (hold_settling
                ? tracker->config.hold_position_tolerance_mm
                : tracker->config.position_tolerance_mm);

        const bool yaw_ok =
            fabsf(yaw_error_deg) <=
            (hold_settling
                ? tracker->config.hold_yaw_tolerance_deg
                : tracker->config.yaw_tolerance_deg);

        if (position_ok && yaw_ok)
        {
            tracker->settle_elapsed_s += dt_s;
        }
        else
        {
            tracker->settle_elapsed_s = 0.0f;
        }

        const float required_settle_s =
            (float)(hold_settling
                ? tracker->config.hold_settle_time_ms
                : tracker->config.settle_time_ms) /
            1000.0f;

        if (tracker->settle_elapsed_s >=
            required_settle_s)
        {
            tracker->settle_elapsed_s =
                required_settle_s;

            if (hold_settling)
            {
                tracker->status.hold_active = true;
                tracker->status.state = tracker->state;
                tracker->status.error = tracker->error;
                tracker->status.settle_elapsed_ms =
                    tracker->config.hold_settle_time_ms;

                trajectory_tracker_clear_status_command(
                    &tracker->status);

                motor_stop();

                return ESP_OK;
            }

            tracker->state =
                TRAJECTORY_TRACKER_FINISHED;

            tracker->status.state = tracker->state;
            tracker->status.error = tracker->error;
            tracker->status.settle_elapsed_ms =
                tracker->config.settle_time_ms;

            trajectory_tracker_clear_status_command(
                &tracker->status);

            motor_stop();

            ESP_LOGI(
                TRAJECTORY_TRACKER_TAG,
                "Trajectory settled: position_error=%.2f mm yaw_error=%.2f deg",
                position_error_mm,
                yaw_error_deg);

            return ESP_OK;
        }
    }
    else
    {
        tracker->settle_elapsed_s = 0.0f;
    }

    const float settle_ms_f =
        tracker->settle_elapsed_s *
        1000.0f;

    tracker->status.settle_elapsed_ms =
        (settle_ms_f > 4294967295.0f) ?
        UINT32_MAX :
        (uint32_t)(settle_ms_f + 0.5f);
    tracker->status.hold_active =
        (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING);

    tracker->status.state = tracker->state;
    tracker->status.error = tracker->error;

    motor_set_velocity(
        command_vx_body_mm_s,
        command_vy_body_mm_s,
        command_w_rad_s);

    return ESP_OK;
}


esp_err_t trajectory_tracker_init(
    trajectory_tracker_t *tracker,
    const trajectory_tracker_config_t *config)
{
    if (tracker == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *tracker =
        (trajectory_tracker_t)
        TRAJECTORY_TRACKER_INITIALIZER;

    if (!trajectory_tracker_config_is_valid(config))
    {
        tracker->error =
            TRAJECTORY_TRACKER_ERROR_INVALID_CONFIG;

        tracker->status.state = tracker->state;
        tracker->status.error = tracker->error;

        return ESP_ERR_INVALID_ARG;
    }

    tracker->config = *config;
    tracker->initialized = true;
    tracker->state = TRAJECTORY_TRACKER_IDLE;
    tracker->error = TRAJECTORY_TRACKER_ERROR_NONE;
    tracker->hold_requested = false;
    tracker->status.state = tracker->state;
    tracker->status.error = tracker->error;
    tracker->status.hold_active = false;

    return ESP_OK;
}


esp_err_t trajectory_tracker_start(
    trajectory_tracker_t *tracker,
    const trajectory_file_header_t *header)
{
    if ((tracker == NULL) || (header == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tracker->initialized)
    {
        tracker->error =
            TRAJECTORY_TRACKER_ERROR_NOT_INITIALIZED;
        tracker->state =
            TRAJECTORY_TRACKER_ERROR;
        tracker->status.state = tracker->state;
        tracker->status.error = tracker->error;

        return ESP_ERR_INVALID_STATE;
    }

    /* start() is the explicit recovery point from FINISHED/ERROR. */
    tracker->state = TRAJECTORY_TRACKER_IDLE;
    tracker->error = TRAJECTORY_TRACKER_ERROR_NONE;
    tracker->settle_elapsed_s = 0.0f;
    tracker->status = (trajectory_tracker_status_t){0};
    tracker->status.state = tracker->state;
    tracker->status.error = tracker->error;

    if (!trajectory_tracker_header_is_valid(header))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_INVALID_ARGUMENT,
            ESP_ERR_INVALID_ARG,
            false,
            "Invalid trajectory header start pose");
    }

    if (!motor_control_is_ready())
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_MOTOR_NOT_READY,
            ESP_ERR_INVALID_STATE,
            false,
            "Motor control is not ready");
    }

    /* Clear a prior emergency latch while commanding zero motion. */
    motor_stop();

    chassis_odometry_state_t odom = {0};

    esp_err_t ret =
        trajectory_tracker_read_odom_checked(
            tracker,
            &odom,
            false);

    if (ret != ESP_OK)
    {
        return ret;
    }

    const float start_error_x_mm =
        header->start_x_mm -
        odom.x_mm;

    const float start_error_y_mm =
        header->start_y_mm -
        odom.y_mm;

    const float start_position_error_mm =
        sqrtf(
            start_error_x_mm * start_error_x_mm +
            start_error_y_mm * start_error_y_mm);

    const float start_yaw_error_deg =
        trajectory_tracker_wrap_deg_180(
            header->start_yaw_deg -
            odom.yaw_deg);

    tracker->status.reference_valid = false;
    tracker->status.reference_x_mm = header->start_x_mm;
    tracker->status.reference_y_mm = header->start_y_mm;
    tracker->status.error_x_mm = start_error_x_mm;
    tracker->status.error_y_mm = start_error_y_mm;
    tracker->status.position_error_mm = start_position_error_mm;
    tracker->status.yaw_error_deg = start_yaw_error_deg;

    if (!isfinite(start_position_error_mm) ||
        !isfinite(start_yaw_error_deg))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_NUMERIC,
            ESP_FAIL,
            false,
            "Non-finite trajectory start alignment error");
    }

    if ((start_position_error_mm >
         tracker->config.start_position_tolerance_mm) ||
        (fabsf(start_yaw_error_deg) >
         tracker->config.start_yaw_tolerance_deg))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_START_POSE_MISMATCH,
            ESP_ERR_INVALID_STATE,
            false,
            "Odometry WORLD pose is not aligned with trajectory start pose");
    }

    tracker->start_x_mm = header->start_x_mm;
    tracker->start_y_mm = header->start_y_mm;
    tracker->yaw_target_deg = header->start_yaw_deg;
    tracker->state = TRAJECTORY_TRACKER_RUNNING;
    tracker->error = TRAJECTORY_TRACKER_ERROR_NONE;
    tracker->hold_requested = false;
    tracker->settle_elapsed_s = 0.0f;

    tracker->status.state = tracker->state;
    tracker->status.error = tracker->error;
    tracker->status.yaw_target_deg = tracker->yaw_target_deg;
    tracker->status.hold_active = false;

    ESP_LOGI(
        TRAJECTORY_TRACKER_TAG,
        "Started tracker: start=(%.1f, %.1f) yaw_hold=%.1f deg",
        tracker->start_x_mm,
        tracker->start_y_mm,
        tracker->yaw_target_deg);

    return ESP_OK;
}


esp_err_t trajectory_tracker_set_hold_mode(
    trajectory_tracker_t *tracker,
    bool enabled)
{
    if ((tracker == NULL) || !tracker->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (tracker->state == TRAJECTORY_TRACKER_ERROR)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const bool was_requested = tracker->hold_requested;

    tracker->hold_requested = enabled;

    /*
     * The runner calls this function on every control cycle.  Preserve the
     * final-trajectory settling timer when the request is unchanged; resetting
     * it here would make FINAL_SETTLING unable to reach FINISHED.
     */
    if (was_requested != enabled)
    {
        tracker->settle_elapsed_s = 0.0f;

        if (enabled &&
            (tracker->state == TRAJECTORY_TRACKER_RUNNING))
        {
            tracker->state =
                TRAJECTORY_TRACKER_HOLD_SETTLING;
        }
        else if (!enabled &&
                 (tracker->state ==
                  TRAJECTORY_TRACKER_HOLD_SETTLING))
        {
            tracker->state =
                TRAJECTORY_TRACKER_RUNNING;
        }
    }

    tracker->status.state = tracker->state;
    tracker->status.hold_active =
        (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING);
    tracker->status.settle_elapsed_ms =
        (uint32_t)(tracker->settle_elapsed_s * 1000.0f);

    if (!enabled)
    {
        trajectory_tracker_clear_status_command(
            &tracker->status);
    }

    if (was_requested != enabled)
    {
        ESP_LOGI(
            TRAJECTORY_TRACKER_TAG,
            "Intermediate hold %s",
            enabled ? "enabled" : "released");
    }

    return ESP_OK;
}


esp_err_t trajectory_tracker_update(
    trajectory_tracker_t *tracker,
    const trajectory_reference_t *reference,
    float dt_s)
{
    if (tracker == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tracker->initialized)
    {
        tracker->error =
            TRAJECTORY_TRACKER_ERROR_NOT_INITIALIZED;
        tracker->state =
            TRAJECTORY_TRACKER_ERROR;
        tracker->status.state = tracker->state;
        tracker->status.error = tracker->error;

        return ESP_ERR_INVALID_STATE;
    }

    if (tracker->state == TRAJECTORY_TRACKER_ERROR)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (tracker->state == TRAJECTORY_TRACKER_IDLE)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (tracker->state == TRAJECTORY_TRACKER_FINISHED)
    {
        motor_stop();
        return ESP_OK;
    }

    if ((reference == NULL) ||
        !trajectory_tracker_reference_is_valid(reference))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_INVALID_REFERENCE,
            ESP_ERR_INVALID_ARG,
            true,
            "Invalid trajectory reference");
    }

    trajectory_tracker_update_status_reference(
        tracker,
        reference);

    if (!isfinite(dt_s) ||
        !(dt_s > 0.0f) ||
        (dt_s > TRAJECTORY_TRACKER_UPDATE_DT_MAX_S))
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_INVALID_DT,
            ESP_ERR_INVALID_ARG,
            true,
            "Invalid trajectory tracker dt");
    }

    if (!motor_control_is_ready())
    {
        return trajectory_tracker_latch_error(
            tracker,
            TRAJECTORY_TRACKER_ERROR_MOTOR_NOT_READY,
            ESP_ERR_INVALID_STATE,
            true,
            "Motor control became unavailable");
    }

    chassis_odometry_state_t odom = {0};

    esp_err_t ret =
        trajectory_tracker_read_odom_checked(
            tracker,
            &odom,
            true);

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (tracker->config.motor_stall_fault_enable)
    {
        motor_status_t motor = {0};
        motor_get_status(&motor);

        if (motor.A.stall_suspected ||
            motor.B.stall_suspected ||
            motor.D.stall_suspected)
        {
            ESP_LOGE(
                TRAJECTORY_TRACKER_TAG,
                "Motor stall detected: A=%d/%lu B=%d/%lu D=%d/%lu",
                (int)motor.A.stall_suspected,
                (unsigned long)motor.A.stall_elapsed_ms,
                (int)motor.B.stall_suspected,
                (unsigned long)motor.B.stall_elapsed_ms,
                (int)motor.D.stall_suspected,
                (unsigned long)motor.D.stall_elapsed_ms);

            return trajectory_tracker_latch_error(
                tracker,
                TRAJECTORY_TRACKER_ERROR_MOTOR_STALL,
                ESP_ERR_TIMEOUT,
                true,
                "Motor wheel did not produce expected encoder motion");
        }
    }

    tracker->status.yaw_target_deg =
        tracker->yaw_target_deg;

    return trajectory_tracker_compute_and_command(
        tracker,
        reference,
        &odom,
        dt_s);
}


void trajectory_tracker_stop(
    trajectory_tracker_t *tracker)
{
    if (tracker == NULL)
    {
        return;
    }

    /*
     * ERROR is latched by design. A normal stop must not silently clear the
     * emergency condition; start() is the only explicit recovery point.
     */
    if (tracker->state == TRAJECTORY_TRACKER_ERROR)
    {
        motor_emergency_stop();
        trajectory_tracker_clear_status_command(
            &tracker->status);
        return;
    }

    if (tracker->initialized)
    {
        motor_stop();
    }

    tracker->state = TRAJECTORY_TRACKER_IDLE;
    tracker->error = TRAJECTORY_TRACKER_ERROR_NONE;
    tracker->settle_elapsed_s = 0.0f;
    tracker->hold_requested = false;

    tracker->status.state = tracker->state;
    tracker->status.error = tracker->error;
    tracker->status.settle_elapsed_ms = 0U;
    tracker->status.hold_active = false;
    trajectory_tracker_clear_status_command(
        &tracker->status);
}


bool trajectory_tracker_is_running(
    const trajectory_tracker_t *tracker)
{
    if ((tracker == NULL) || !tracker->initialized)
    {
        return false;
    }

    return
        (tracker->state == TRAJECTORY_TRACKER_RUNNING) ||
        (tracker->state == TRAJECTORY_TRACKER_HOLD_SETTLING) ||
        (tracker->state == TRAJECTORY_TRACKER_SETTLING);
}


bool trajectory_tracker_is_finished(
    const trajectory_tracker_t *tracker)
{
    return
        (tracker != NULL) &&
        tracker->initialized &&
        (tracker->state == TRAJECTORY_TRACKER_FINISHED);
}


trajectory_tracker_state_t trajectory_tracker_get_state(
    const trajectory_tracker_t *tracker)
{
    if ((tracker == NULL) || !tracker->initialized)
    {
        return TRAJECTORY_TRACKER_ERROR;
    }

    return tracker->state;
}


trajectory_tracker_error_t trajectory_tracker_get_error(
    const trajectory_tracker_t *tracker)
{
    if ((tracker == NULL) || !tracker->initialized)
    {
        return TRAJECTORY_TRACKER_ERROR_NOT_INITIALIZED;
    }

    return tracker->error;
}


void trajectory_tracker_get_status(
    const trajectory_tracker_t *tracker,
    trajectory_tracker_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    if (tracker == NULL)
    {
        *status = (trajectory_tracker_status_t){0};
        status->state = TRAJECTORY_TRACKER_ERROR;
        status->error = TRAJECTORY_TRACKER_ERROR_NOT_INITIALIZED;
        return;
    }

    *status = tracker->status;
}


const char *trajectory_tracker_state_name(
    trajectory_tracker_state_t state)
{
    switch (state)
    {
        case TRAJECTORY_TRACKER_IDLE:
            return "IDLE";

        case TRAJECTORY_TRACKER_RUNNING:
            return "RUNNING";

        case TRAJECTORY_TRACKER_HOLD_SETTLING:
            return "HOLD_SETTLING";

        case TRAJECTORY_TRACKER_SETTLING:
            return "SETTLING";

        case TRAJECTORY_TRACKER_FINISHED:
            return "FINISHED";

        case TRAJECTORY_TRACKER_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}


const char *trajectory_tracker_error_name(
    trajectory_tracker_error_t error)
{
    switch (error)
    {
        case TRAJECTORY_TRACKER_ERROR_NONE:
            return "NONE";

        case TRAJECTORY_TRACKER_ERROR_NOT_INITIALIZED:
            return "NOT_INITIALIZED";

        case TRAJECTORY_TRACKER_ERROR_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";

        case TRAJECTORY_TRACKER_ERROR_INVALID_CONFIG:
            return "INVALID_CONFIG";

        case TRAJECTORY_TRACKER_ERROR_MOTOR_NOT_READY:
            return "MOTOR_NOT_READY";

        case TRAJECTORY_TRACKER_ERROR_ODOMETRY_NOT_READY:
            return "ODOMETRY_NOT_READY";

        case TRAJECTORY_TRACKER_ERROR_ODOMETRY_STALE:
            return "ODOMETRY_STALE";

        case TRAJECTORY_TRACKER_ERROR_START_POSE_MISMATCH:
            return "START_POSE_MISMATCH";

        case TRAJECTORY_TRACKER_ERROR_INVALID_REFERENCE:
            return "INVALID_REFERENCE";

        case TRAJECTORY_TRACKER_ERROR_INVALID_DT:
            return "INVALID_DT";

        case TRAJECTORY_TRACKER_ERROR_TRACKING_ERROR_EXCEEDED:
            return "TRACKING_ERROR_EXCEEDED";

        case TRAJECTORY_TRACKER_ERROR_MOTOR_STALL:
            return "MOTOR_STALL";

        case TRAJECTORY_TRACKER_ERROR_NUMERIC:
            return "NUMERIC";

        default:
            return "UNKNOWN";
    }
}


#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_TRACKER_IMPLEMENTATION */
#endif /* TRAJECTORY_TRACKER_H */
