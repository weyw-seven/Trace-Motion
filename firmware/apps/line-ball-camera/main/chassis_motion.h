#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

/*
 * Revised for the calibrated drivetrain:
 * - physical-unit motor_set_velocity(mm/s, mm/s, rad/s)
 * - +X forward, +Y left, +Yaw/+w CCW
 * - coherent odometry gyro rate used for heading/rotation feedback
 * - low-speed hunting reduced near distance/angle targets
 * - cross-track PD correction enabled in this experimental variant
 *
 * Public function/type interface is unchanged.
 */

/*
 * chassis_motion.h
 *
 * High-level chassis motion controller for:
 *   1) Move a specified distance along a specified chassis-relative direction,
 *      while holding the chassis heading.
 *   2) Rotate a specified relative angle with a specified maximum angular speed.
 *
 * Coordinate convention:
 *   vx > 0 : forward
 *   vy > 0 : left
 *   w  > 0 : counter-clockwise
 *
 * Units:
 *   position / distance : mm
 *   linear velocity     : mm/s
 *   angle               : degree
 *   angular velocity API: degree/s
 *   motor_set_velocity w: rad/s
 *
 * Single-header usage:
 *
 *   In exactly ONE .c file:
 *
 *       #define CHASSIS_MOTION_IMPLEMENTATION
 *       #include "chassis_motion.h"
 *
 *   In other .c files:
 *
 *       #include "chassis_motion.h"
 *
 * Required lower-layer headers:
 *   motor_control.h
 *   mpu6050.h
 *   chassis_odometry.h
 *
 * Required chassis_odometry.h public contract:
 *
 *   typedef struct
 *   {
 *       bool initialized;
 *       float x_mm;
 *       float y_mm;
 *       float yaw_deg;       // continuous relative yaw; do NOT wrap to +/-180
 *       float body_vx_mm_s;
 *       float body_vy_mm_s;
 *       float world_vx_mm_s;
 *       float world_vy_mm_s;
 *       float gyro_w_rad_s;    // +CCW, rad/s
 *       float encoder_w_rad_s;
 *       uint32_t update_count;
 *       int64_t timestamp_us;
 *   } chassis_odometry_state_t;
 *
 *   bool chassis_odometry_is_ready(void);
 *   void chassis_odometry_get_state(chassis_odometry_state_t *snapshot);
 *
 * The macros below can be overridden before including this file if the
 * odometry function/type names differ.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"
#include "mpu6050.h"
#include "chassis_odometry.h"

/*
 * Physical-unit motor command used by this layer:
 *   vx, vy : mm/s
 *   w      : rad/s
 *
 * Current motor_control implements this function.  Re-declaring the
 * prototype here also supports revisions where only Set_motor() was
 * present in the public declaration block.
 */
void motor_set_velocity(
    float vx_mm_s,
    float vy_mm_s,
    float w_rad_s);


#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================
 * 1. Odometry adapter
 * ============================================================ */

#ifndef CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE
#define CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE \
    chassis_odometry_state_t
#endif

#ifndef CHASSIS_MOTION_ODOMETRY_READY
#define CHASSIS_MOTION_ODOMETRY_READY() \
    chassis_odometry_is_ready()
#endif

#ifndef CHASSIS_MOTION_ODOMETRY_GET
#define CHASSIS_MOTION_ODOMETRY_GET(snapshot_ptr) \
    chassis_odometry_get_state((snapshot_ptr))
#endif


/* ============================================================
 * 2. Task configuration
 * ============================================================ */

#ifndef CHASSIS_MOTION_MOVE_MIN_SPEED_MM_S
#define CHASSIS_MOTION_MOVE_MIN_SPEED_MM_S    150.0f // 20.0f
#endif

#ifndef CHASSIS_MOTION_PERIOD_MS
#define CHASSIS_MOTION_PERIOD_MS              10U
#endif

#define CHASSIS_MOTION_DT \
    ((float)CHASSIS_MOTION_PERIOD_MS / 1000.0f)

#ifndef CHASSIS_MOTION_TASK_STACK_SIZE
#define CHASSIS_MOTION_TASK_STACK_SIZE        4096
#endif

/*
 * Recommended relation:
 *   MPU task    : 7
 *   motion task : 6
 *   motor task  : 5
 */
#ifndef CHASSIS_MOTION_TASK_PRIORITY
#define CHASSIS_MOTION_TASK_PRIORITY          6
#endif


/* ============================================================
 * 3. Mathematical constants
 * ============================================================ */

#define CHASSIS_MOTION_PI                     3.14159265358979323846f
#define CHASSIS_MOTION_DEG_TO_RAD             0.01745329251994329577f
#define CHASSIS_MOTION_RAD_TO_DEG             57.295779513082320876f


/* ============================================================
 * 4. Move-distance controller parameters
 * ============================================================ */

/*
 * Longitudinal position P controller:
 *
 *   v_along_target = Kp * remaining_distance
 *
 * Unit:
 *   Kp = 1/s
 */
#ifndef CHASSIS_MOTION_MOVE_KP
#define CHASSIS_MOTION_MOVE_KP                2.0f
#endif

/*
 * Cross-track correction:
 *
 *   v_cross_target = -Kp * cross_track_error
 */
#ifndef CHASSIS_MOTION_CROSS_KP
#define CHASSIS_MOTION_CROSS_KP               0.35f
#endif

#ifndef CHASSIS_MOTION_CROSS_SPEED_MAX_MM_S
#define CHASSIS_MOTION_CROSS_SPEED_MAX_MM_S   60.0f
#endif

/* Optional cross-track derivative term.  The default remains P-only/off. */
#ifndef CHASSIS_MOTION_ENABLE_CROSS_PD
#define CHASSIS_MOTION_ENABLE_CROSS_PD         1
#endif

#ifndef CHASSIS_MOTION_CROSS_KD
#define CHASSIS_MOTION_CROSS_KD               0.05f
#endif

#ifndef CHASSIS_MOTION_CROSS_RATE_FILTER_ALPHA
#define CHASSIS_MOTION_CROSS_RATE_FILTER_ALPHA 0.22f
#endif

#ifndef CHASSIS_MOTION_CROSS_RATE_DEADBAND_MM_S
#define CHASSIS_MOTION_CROSS_RATE_DEADBAND_MM_S 15.0f
#endif

/*
 * First-stage motion control keeps the path controller simple:
 * distance-along-path + heading hold.
 *
 * Cross-track error is still measured/reported, but correction is OFF
 * by default because tiny lateral corrections can fall into the
 * drivetrain's nonlinear low-speed region.
 */
#ifndef CHASSIS_MOTION_ENABLE_CROSS_TRACK
#define CHASSIS_MOTION_ENABLE_CROSS_TRACK     1
#endif

/*
 * Heading hold while translating:
 *
 *   w_target[dps] =
 *       KP * yaw_error[deg]
 *       - KD * gyro_z[dps]
 */
#ifndef CHASSIS_MOTION_HEADING_KP
#define CHASSIS_MOTION_HEADING_KP             2.0f
#endif

#ifndef CHASSIS_MOTION_HEADING_KD
#define CHASSIS_MOTION_HEADING_KD             0.06f
#endif

#ifndef CHASSIS_MOTION_HEADING_W_MAX_DEG_S
#define CHASSIS_MOTION_HEADING_W_MAX_DEG_S    95.0f
#endif


/* ============================================================
 * 5. Rotate controller parameters
 * ============================================================ */

#ifndef CHASSIS_MOTION_ROTATE_KP
#define CHASSIS_MOTION_ROTATE_KP              2.10f
#endif

#ifndef CHASSIS_MOTION_ROTATE_KD
#define CHASSIS_MOTION_ROTATE_KD              0.08f
#endif

#ifndef CHASSIS_MOTION_ROTATE_MIN_SPEED_DEG_S
#define CHASSIS_MOTION_ROTATE_MIN_SPEED_DEG_S 40.0f
#endif


/* ============================================================
 * 6. Acceleration / slew limits
 * ============================================================ */

#ifndef CHASSIS_MOTION_LINEAR_ACCEL_MM_S2
#define CHASSIS_MOTION_LINEAR_ACCEL_MM_S2     1500.0f
#endif

#ifndef CHASSIS_MOTION_CROSS_ACCEL_MM_S2
#define CHASSIS_MOTION_CROSS_ACCEL_MM_S2      250.0f
#endif

#ifndef CHASSIS_MOTION_ANGULAR_ACCEL_DEG_S2
#define CHASSIS_MOTION_ANGULAR_ACCEL_DEG_S2   120.0f
#endif


/* ============================================================
 * 7. Target completion tolerances
 * ============================================================ */

#ifndef CHASSIS_MOTION_DISTANCE_TOL_MM
#define CHASSIS_MOTION_DISTANCE_TOL_MM        5.0f
#endif

/*
 * Small completion band after the target is crossed.  Larger overshoot is
 * recovered once at the calibrated creep speed; this prevents an overshoot
 * from becoming an unrecoverable timeout while avoiding hunting in place.
 */
#ifndef CHASSIS_MOTION_DISTANCE_OVERSHOOT_TOL_MM
#define CHASSIS_MOTION_DISTANCE_OVERSHOOT_TOL_MM 10.0f
#endif

/* Once the goal enters the final band, small odometry noise must not restart
 * the creep command.  It is released only after this wider band is crossed. */
#ifndef CHASSIS_MOTION_DISTANCE_RELEASE_TOL_MM
#define CHASSIS_MOTION_DISTANCE_RELEASE_TOL_MM 10.0f
#endif

/*
 * The positive final-zone release band above is wider than the entry band,
 * but the negative overshoot side needs its own wider release band too.
 * Otherwise a small odometry fluctuation just past the overshoot limit can
 * immediately restart creep recovery.
 */
#ifndef CHASSIS_MOTION_DISTANCE_OVERSHOOT_RELEASE_TOL_MM
#define CHASSIS_MOTION_DISTANCE_OVERSHOOT_RELEASE_TOL_MM 15.0f
#endif

#ifndef CHASSIS_MOTION_CROSS_TOL_MM
#define CHASSIS_MOTION_CROSS_TOL_MM           15.0f
#endif

#ifndef CHASSIS_MOTION_CROSS_RELEASE_TOL_MM
#define CHASSIS_MOTION_CROSS_RELEASE_TOL_MM \
    (CHASSIS_MOTION_CROSS_TOL_MM + 5.0f)
#endif

#ifndef CHASSIS_MOTION_MOVE_YAW_TOL_DEG
#define CHASSIS_MOTION_MOVE_YAW_TOL_DEG       3.0f
#endif

#ifndef CHASSIS_MOTION_MOVE_YAW_RELEASE_TOL_DEG
#define CHASSIS_MOTION_MOVE_YAW_RELEASE_TOL_DEG \
    (CHASSIS_MOTION_MOVE_YAW_TOL_DEG + 2.0f)
#endif

#ifndef CHASSIS_MOTION_ROTATE_TOL_DEG
#define CHASSIS_MOTION_ROTATE_TOL_DEG         5.0f
#endif

#ifndef CHASSIS_MOTION_SETTLE_GYRO_DPS
#define CHASSIS_MOTION_SETTLE_GYRO_DPS        5.0f
#endif

#ifndef CHASSIS_MOTION_SETTLE_LINEAR_MM_S
#define CHASSIS_MOTION_SETTLE_LINEAR_MM_S     50.0f
#endif

#ifndef CHASSIS_MOTION_SETTLE_CYCLES
#define CHASSIS_MOTION_SETTLE_CYCLES          10U
#endif


/* ============================================================
 * 8. Safety / timeout
 * ============================================================ */

#ifndef CHASSIS_MOTION_ODOM_STALE_US
#define CHASSIS_MOTION_ODOM_STALE_US          200000LL
#endif

/*
 * timeout =
 *     BASE
 *     +
 *     FACTOR * ideal_motion_time
 *
 * The timeout is intentionally generous. It is only a safety guard.
 */
#ifndef CHASSIS_MOTION_TIMEOUT_BASE_MS
#define CHASSIS_MOTION_TIMEOUT_BASE_MS        2500U
#endif

#ifndef CHASSIS_MOTION_TIMEOUT_FACTOR
#define CHASSIS_MOTION_TIMEOUT_FACTOR         4.0f
#endif

#ifndef CHASSIS_MOTION_TIMEOUT_SETTLE_GRACE_MS
#define CHASSIS_MOTION_TIMEOUT_SETTLE_GRACE_MS 1000U
#endif


/* ============================================================
 * 9. Public types
 * ============================================================ */

typedef enum
{
    CHASSIS_MOTION_MODE_IDLE = 0,

    CHASSIS_MOTION_MODE_MOVE_DISTANCE,
    CHASSIS_MOTION_MODE_ROTATE,

    CHASSIS_MOTION_MODE_DONE,
    CHASSIS_MOTION_MODE_CANCELLED,
    CHASSIS_MOTION_MODE_ERROR

} chassis_motion_mode_t;


typedef enum
{
    CHASSIS_MOTION_ERROR_NONE = 0,

    CHASSIS_MOTION_ERROR_NOT_INITIALIZED,
    CHASSIS_MOTION_ERROR_MOTOR_NOT_READY,
    CHASSIS_MOTION_ERROR_MPU_NOT_READY,
    CHASSIS_MOTION_ERROR_ODOMETRY_NOT_READY,
    CHASSIS_MOTION_ERROR_ODOMETRY_STALE,
    CHASSIS_MOTION_ERROR_INVALID_ARGUMENT,
    CHASSIS_MOTION_ERROR_TIMEOUT

} chassis_motion_error_t;


typedef struct
{
    bool initialized;
    bool busy;
    bool target_reached;

    chassis_motion_mode_t mode;
    chassis_motion_error_t error;

    uint32_t command_id;

    /*
     * Current pose.
     */
    float x_mm;
    float y_mm;
    float yaw_deg;

    /*
     * Start pose of current command.
     */
    float start_x_mm;
    float start_y_mm;
    float start_yaw_deg;

    /*
     * Target pose / motion target.
     */
    float target_x_mm;
    float target_y_mm;
    float target_yaw_deg;

    /*
     * MOVE_DISTANCE status.
     */
    float direction_deg;
    float target_distance_mm;
    float traveled_distance_mm;
    float remaining_distance_mm;
    float cross_track_error_mm;

    /*
     * ROTATE status.
     */
    float target_angle_deg;
    float remaining_angle_deg;

    /*
     * Command sent to motor_control.
     */
    float command_vx_mm_s;
    float command_vy_mm_s;
    float command_w_rad_s;

    /*
     * Timing.
     */
    int64_t start_time_us;
    int64_t elapsed_time_us;

} chassis_motion_status_t;


/* ============================================================
 * 10. Public API
 * ============================================================ */

/**
 * @brief Initialize the high-level motion controller task.
 *
 * Call after:
 *   motor_control_init()
 *   mpu6050_init()
 *   chassis_odometry_init()
 *
 * All lower layers must already be ready.
 */
esp_err_t chassis_motion_init(void);


/**
 * @brief Move a specified distance along a chassis-relative direction.
 *
 * direction_deg:
 *      0   = forward
 *      +90 = left
 *      -90 = right
 *      180 = backward
 *
 * distance_mm:
 *      Travel distance in mm.
 *      Negative distance is accepted and is internally converted to
 *      positive distance with direction_deg += 180 degrees.
 *
 * max_speed_mm_s:
 *      Maximum translation speed.
 *
 * Important:
 *      The direction is defined relative to the chassis heading at the
 *      moment this command starts.
 *
 *      The high-level controller keeps the START heading using MPU6050
 *      yaw feedback. Therefore the actual motor w command may be nonzero
 *      even though the desired body rotation is zero.
 *
 * Non-blocking:
 *      Returns after accepting the command.
 */
esp_err_t chassis_motion_move_distance(
    float direction_deg,
    float distance_mm,
    float max_speed_mm_s);


/**
 * @brief Rotate a specified relative angle.
 *
 * angle_deg:
 *      > 0 : counter-clockwise
 *      < 0 : clockwise
 *
 * angular_speed_deg_s:
 *      Positive maximum angular speed magnitude in degree/s.
 *
 * Example:
 *      chassis_motion_rotate(+90.0f, 60.0f);
 *      chassis_motion_rotate(-90.0f, 60.0f);
 *
 * Non-blocking.
 */
esp_err_t chassis_motion_rotate(
    float angle_deg,
    float angular_speed_deg_s);


/**
 * @brief Normal controlled stop and return to IDLE.
 */
void chassis_motion_stop(void);


/**
 * @brief Cancel current high-level command.
 *
 * Motor output is set to zero and state becomes CANCELLED.
 */
void chassis_motion_cancel(void);


/**
 * @brief Immediate lower-layer emergency stop.
 */
void chassis_motion_emergency_stop(void);


/**
 * @brief Whether a MOVE_DISTANCE or ROTATE command is active.
 */
bool chassis_motion_is_busy(void);


/**
 * @brief Get an atomic status snapshot.
 */
void chassis_motion_get_status(
    chassis_motion_status_t *status);


/**
 * @brief Blocking convenience function.
 *
 * timeout:
 *      FreeRTOS timeout.
 *
 * Returns:
 *      ESP_OK              target reached
 *      ESP_ERR_TIMEOUT     wait timeout
 *      ESP_FAIL            command cancelled/error
 *
 * Do NOT call from the chassis_motion internal task.
 */
esp_err_t chassis_motion_wait(
    TickType_t timeout);


/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */

#ifdef CHASSIS_MOTION_IMPLEMENTATION


static const char *CHASSIS_MOTION_TAG =
    "CHASSIS_MOTION";


typedef struct
{
    chassis_motion_mode_t mode;

    uint32_t command_id;

    float start_x_mm;
    float start_y_mm;
    float start_yaw_deg;

    float target_x_mm;
    float target_y_mm;
    float target_yaw_deg;

    /*
     * MOVE_DISTANCE geometry in world frame.
     */
    float axis_x;
    float axis_y;

    float perp_x;
    float perp_y;

    float direction_deg;
    float distance_mm;
    float max_speed_mm_s;

    /*
     * ROTATE.
     */
    float angle_deg;
    float max_angular_speed_deg_s;

    /*
     * Slew-limited controller states.
     */
    float v_along_mm_s;
    float v_cross_mm_s;
    float w_deg_s;

    uint16_t settle_count;
    bool target_zone_latched;
    bool final_settle_latched;
    float cross_rate_filtered_mm_s;

    int64_t start_time_us;
    int64_t deadline_us;

} chassis_motion_goal_t;


static TaskHandle_t
    g_chassis_motion_task_handle = NULL;


static bool
    g_chassis_motion_initialized = false;


static uint32_t
    g_chassis_motion_next_command_id = 1U;


static chassis_motion_goal_t
    g_chassis_motion_goal = {0};


static chassis_motion_status_t
    g_chassis_motion_status = {0};


static portMUX_TYPE
    g_chassis_motion_lock =
        portMUX_INITIALIZER_UNLOCKED;


/* ============================================================
 * 11. Math helpers
 * ============================================================ */

static float chassis_motion_clampf(
    float value,
    float min_value,
    float max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}


static float chassis_motion_absf(
    float value)
{
    return (value >= 0.0f)
        ? value
        : -value;
}


static float chassis_motion_wrap_180(
    float angle_deg)
{
    while (angle_deg > 180.0f)
        angle_deg -= 360.0f;

    while (angle_deg < -180.0f)
        angle_deg += 360.0f;

    return angle_deg;
}


static float chassis_motion_slew(
    float current,
    float target,
    float max_delta)
{
    const float delta =
        target - current;

    if (delta > max_delta)
        return current + max_delta;

    if (delta < -max_delta)
        return current - max_delta;

    return target;
}


static float chassis_motion_hypot2(
    float x,
    float y)
{
    return sqrtf(
        x * x +
        y * y);
}


static bool chassis_motion_float_valid(
    float value)
{
    return isfinite(value);
}


/* ============================================================
 * 12. Dependency checks
 * ============================================================ */

static bool chassis_motion_mpu_ready(void)
{
    mpu6050_status_t status = {0};

    mpu6050_get_status(
        &status);

    return
        status.initialized &&
        status.sampling;
}


static bool chassis_motion_odometry_snapshot_valid(
    const CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE *odom)
{
    if (odom == NULL)
        return false;

    if (!odom->initialized)
        return false;

    if (odom->timestamp_us <= 0)
        return false;

    const int64_t now_us =
        esp_timer_get_time();

    const int64_t age_us =
        now_us -
        odom->timestamp_us;

    if ((age_us < 0) ||
        (age_us > CHASSIS_MOTION_ODOM_STALE_US))
    {
        return false;
    }

    return true;
}


static chassis_motion_error_t
chassis_motion_check_dependencies(
    CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE *odom)
{
    if (!motor_control_is_ready())
        return CHASSIS_MOTION_ERROR_MOTOR_NOT_READY;

    if (!chassis_motion_mpu_ready())
        return CHASSIS_MOTION_ERROR_MPU_NOT_READY;

    if (!CHASSIS_MOTION_ODOMETRY_READY())
        return CHASSIS_MOTION_ERROR_ODOMETRY_NOT_READY;

    if (odom != NULL)
    {
        CHASSIS_MOTION_ODOMETRY_GET(
            odom);

        if (!chassis_motion_odometry_snapshot_valid(
                odom))
        {
            return CHASSIS_MOTION_ERROR_ODOMETRY_STALE;
        }
    }

    return CHASSIS_MOTION_ERROR_NONE;
}


/* ============================================================
 * 13. State / status helpers
 * ============================================================ */

static bool chassis_motion_mode_is_busy(
    chassis_motion_mode_t mode)
{
    return
        (mode == CHASSIS_MOTION_MODE_MOVE_DISTANCE)
        ||
        (mode == CHASSIS_MOTION_MODE_ROTATE);
}


static void chassis_motion_set_terminal_state(
    chassis_motion_mode_t mode,
    chassis_motion_error_t error,
    bool target_reached)
{
    motor_stop();

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    g_chassis_motion_goal.mode =
        mode;

    g_chassis_motion_goal.v_along_mm_s =
        0.0f;

    g_chassis_motion_goal.v_cross_mm_s =
        0.0f;

    g_chassis_motion_goal.w_deg_s =
        0.0f;

    g_chassis_motion_goal.settle_count =
        0U;

    g_chassis_motion_status.mode =
        mode;

    g_chassis_motion_status.error =
        error;

    g_chassis_motion_status.busy =
        false;

    g_chassis_motion_status.target_reached =
        target_reached;

    g_chassis_motion_status.command_vx_mm_s =
        0.0f;

    g_chassis_motion_status.command_vy_mm_s =
        0.0f;

    g_chassis_motion_status.command_w_rad_s =
        0.0f;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);
}


static void chassis_motion_fail(
    chassis_motion_error_t error)
{
    ESP_LOGE(
        CHASSIS_MOTION_TAG,
        "motion failed, error=%d",
        (int)error);

    chassis_motion_set_terminal_state(
        CHASSIS_MOTION_MODE_ERROR,
        error,
        false);
}


static int64_t chassis_motion_calculate_deadline_us(
    float amount,
    float speed,
    float acceleration)
{
    const float distance =
        chassis_motion_absf(amount);

    const float speed_abs =
        chassis_motion_absf(speed);

    const float acceleration_abs =
        chassis_motion_absf(acceleration);

    float ideal_time_s = 0.0f;

    if ((distance > 0.0f) &&
        (speed_abs > 0.001f))
    {
        /*
         * Include the acceleration/deceleration profile.  The previous
         * distance/speed estimate is too optimistic for short, high-speed
         * moves because those moves never reach the requested cruise speed.
         */
        ideal_time_s =
            distance /
            speed_abs;

        if (acceleration_abs > 0.001f)
        {
            const float time_to_cruise =
                speed_abs /
                acceleration_abs;

            const float acceleration_distance =
                0.5f *
                acceleration_abs *
                time_to_cruise *
                time_to_cruise;

            const float profile_time =
                ((2.0f * acceleration_distance) >= distance)
                    ? (2.0f *
                       sqrtf(distance / acceleration_abs))
                    : (2.0f * time_to_cruise +
                       (distance -
                        2.0f * acceleration_distance) /
                       speed_abs);

            if (profile_time > ideal_time_s)
            {
                ideal_time_s = profile_time;
            }
        }
    }

    const float timeout_s =
        ((float)CHASSIS_MOTION_TIMEOUT_BASE_MS / 1000.0f)
        +
        CHASSIS_MOTION_TIMEOUT_FACTOR
        *
        ideal_time_s
        +
        ((float)CHASSIS_MOTION_TIMEOUT_SETTLE_GRACE_MS /
         1000.0f)
        +
        ((float)CHASSIS_MOTION_SETTLE_CYCLES *
         CHASSIS_MOTION_DT);

    return
        esp_timer_get_time()
        +
        (int64_t)(
            timeout_s *
            1000000.0f);
}


static void chassis_motion_update_status_common(
    const chassis_motion_goal_t *goal,
    const CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE *odom,
    float command_vx_mm_s,
    float command_vy_mm_s,
    float command_w_rad_s)
{
    if ((goal == NULL) ||
        (odom == NULL))
    {
        return;
    }

    const int64_t now_us =
        esp_timer_get_time();

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    /*
     * Do not publish stale calculations from an old command.
     */
    if (g_chassis_motion_goal.command_id !=
        goal->command_id)
    {
        taskEXIT_CRITICAL(
            &g_chassis_motion_lock);

        return;
    }

    g_chassis_motion_status.x_mm =
        odom->x_mm;

    g_chassis_motion_status.y_mm =
        odom->y_mm;

    g_chassis_motion_status.yaw_deg =
        odom->yaw_deg;

    g_chassis_motion_status.command_vx_mm_s =
        command_vx_mm_s;

    g_chassis_motion_status.command_vy_mm_s =
        command_vy_mm_s;

    g_chassis_motion_status.command_w_rad_s =
        command_w_rad_s;

    g_chassis_motion_status.elapsed_time_us =
        now_us -
        goal->start_time_us;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);
}


/* ============================================================
 * 14. MOVE_DISTANCE control
 * ============================================================ */

static void chassis_motion_step_move_distance(
    chassis_motion_goal_t goal,
    const CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE *odom)
{
    /*
     * Current displacement from start in world frame.
     */
    const float from_start_x =
        odom->x_mm -
        goal.start_x_mm;

    const float from_start_y =
        odom->y_mm -
        goal.start_y_mm;

    /*
     * Progress along commanded path.
     */
    const float traveled =
        from_start_x * goal.axis_x
        +
        from_start_y * goal.axis_y;

    const float remaining =
        goal.distance_mm -
        traveled;

    /*
     * Enter the final zone once, then keep the stop command latched while
     * odometry jitters inside the wider release band.  This prevents a
     * 4.9 mm -> 5.7 mm measurement fluctuation from restarting creep.
     */
    bool target_zone_latched =
        goal.target_zone_latched;

    if (target_zone_latched)
    {
        if ((remaining >
             CHASSIS_MOTION_DISTANCE_RELEASE_TOL_MM) ||
            (remaining <
             -CHASSIS_MOTION_DISTANCE_OVERSHOOT_RELEASE_TOL_MM))
        {
            target_zone_latched = false;
        }
    }
    else if ((remaining <=
              CHASSIS_MOTION_DISTANCE_TOL_MM) &&
             (remaining >=
              -CHASSIS_MOTION_DISTANCE_OVERSHOOT_TOL_MM))
    {
        target_zone_latched = true;
    }

    /*
     * Positive cross_track means the chassis is displaced toward
     * +perpendicular direction from the ideal line.
     */
    const float cross_track =
        from_start_x * goal.perp_x
        +
        from_start_y * goal.perp_y;

    /*
     * Translation controller in the path coordinate system.
     */
    float v_along_target = 0.0f;

    /*
     * Use a signed position error with a small asymmetric deadband:
     *
     *   remaining > +tolerance  : approach the target
     *   remaining < -overshoot  : one-way, low-speed recovery
     *   otherwise                : stop and settle
     *
     * The old implementation applied an 800 mm/s floor until the final
     * tolerance was reached and then refused to recover overshoot.  That
     * could produce a large overshoot followed by an unavoidable timeout.
     */
    if (target_zone_latched)
    {
        v_along_target = 0.0f;
    }
    else if (remaining >
        CHASSIS_MOTION_DISTANCE_TOL_MM)
    {
        const float distance_error =
            remaining -
            CHASSIS_MOTION_DISTANCE_TOL_MM;

        v_along_target =
            CHASSIS_MOTION_MOVE_KP *
            distance_error;

        v_along_target =
            chassis_motion_clampf(
                v_along_target,
                CHASSIS_MOTION_MOVE_MIN_SPEED_MM_S,
                goal.max_speed_mm_s);
    }
    else if (remaining <
             -CHASSIS_MOTION_DISTANCE_OVERSHOOT_TOL_MM)
    {
        /* Recover overshoot at the calibrated, low creep speed. */
        v_along_target =
            -CHASSIS_MOTION_MOVE_MIN_SPEED_MM_S;
    }


    float v_cross_target = 0.0f;

#if CHASSIS_MOTION_ENABLE_CROSS_TRACK
    const float cross_rate_raw_mm_s =
        odom->world_vx_mm_s * goal.perp_x
        +
        odom->world_vy_mm_s * goal.perp_y;

    const float cross_rate_alpha =
        chassis_motion_clampf(
            CHASSIS_MOTION_CROSS_RATE_FILTER_ALPHA,
            0.0f,
            1.0f);

    goal.cross_rate_filtered_mm_s +=
        cross_rate_alpha *
        (cross_rate_raw_mm_s -
         goal.cross_rate_filtered_mm_s);

#if CHASSIS_MOTION_ENABLE_CROSS_PD
    /*
     * The derivative term damps an active position correction.  It must not
     * wake the lateral actuator by itself: at the end of a move, encoder
     * quantization can make cross_rate jump while cross_track is already
     * inside tolerance, and the drivetrain then turns a tiny command into a
     * full static-friction PWM pulse.
     */
    const bool cross_error_active =
        chassis_motion_absf(cross_track) >
        CHASSIS_MOTION_CROSS_TOL_MM;

    if (cross_error_active)
    {
        v_cross_target =
            -(
                CHASSIS_MOTION_CROSS_KP *
                cross_track
                +
                CHASSIS_MOTION_CROSS_KD *
                goal.cross_rate_filtered_mm_s);
    }
#else
    if (chassis_motion_absf(cross_track) >
        CHASSIS_MOTION_CROSS_TOL_MM)
    {
        v_cross_target =
            -CHASSIS_MOTION_CROSS_KP *
            cross_track;
    }
#endif

    v_cross_target =
        chassis_motion_clampf(
            v_cross_target,
            -CHASSIS_MOTION_CROSS_SPEED_MAX_MM_S,
            CHASSIS_MOTION_CROSS_SPEED_MAX_MM_S);
#endif

    /*
     * Heading hold:
     * use wrapped error because the goal is to keep the same physical
     * heading, not to command multiple revolutions during translation.
     */
    const float yaw_error_deg =
        chassis_motion_wrap_180(
            goal.target_yaw_deg -
            odom->yaw_deg);

    const float gyro_z_dps =
        odom->gyro_w_rad_s *
        CHASSIS_MOTION_RAD_TO_DEG;

    float w_target_deg_s =
        CHASSIS_MOTION_HEADING_KP
        *
        yaw_error_deg
        -
        CHASSIS_MOTION_HEADING_KD
        *
        gyro_z_dps;

    const float motor_w_limit_deg_s =
        MOTOR_MAX_W_RAD_S
        *
        CHASSIS_MOTION_RAD_TO_DEG;

    float heading_w_limit_deg_s =
        CHASSIS_MOTION_HEADING_W_MAX_DEG_S;

    if (heading_w_limit_deg_s >
        motor_w_limit_deg_s)
    {
        heading_w_limit_deg_s =
            motor_w_limit_deg_s;
    }

    w_target_deg_s =
        chassis_motion_clampf(
            w_target_deg_s,
            -heading_w_limit_deg_s,
            heading_w_limit_deg_s);

    /*
     * Final settling gate.
     *
     * Once distance, cross-track position, and heading are all inside the
     * entry bands, stop issuing microscopic corrections and let the chassis
     * settle.  Those commands are below the drivetrain's reliably
     * controllable speed, so the motor layer's static-friction compensation
     * would otherwise turn them into alternating large PWM pulses.
     */
    const bool final_settle_was_latched =
        goal.final_settle_latched;

    bool final_settle_latched =
        final_settle_was_latched;

#if CHASSIS_MOTION_ENABLE_CROSS_TRACK
    const bool final_cross_entry_ok =
        chassis_motion_absf(cross_track) <=
        CHASSIS_MOTION_CROSS_TOL_MM;

    const bool final_cross_release =
        chassis_motion_absf(cross_track) >
        CHASSIS_MOTION_CROSS_RELEASE_TOL_MM;
#else
    const bool final_cross_entry_ok = true;
    const bool final_cross_release = false;
#endif

    const bool final_settle_entry =
        target_zone_latched &&
        final_cross_entry_ok &&
        (chassis_motion_absf(yaw_error_deg) <=
         CHASSIS_MOTION_MOVE_YAW_TOL_DEG);

    const bool final_settle_release =
        (remaining >
         CHASSIS_MOTION_DISTANCE_RELEASE_TOL_MM) ||
        (remaining <
         -CHASSIS_MOTION_DISTANCE_OVERSHOOT_RELEASE_TOL_MM) ||
        final_cross_release ||
        (chassis_motion_absf(yaw_error_deg) >
         CHASSIS_MOTION_MOVE_YAW_RELEASE_TOL_DEG);

    if (final_settle_latched)
    {
        if (final_settle_release)
        {
            final_settle_latched = false;
            goal.settle_count = 0U;
        }
    }
    else if (final_settle_entry)
    {
        final_settle_latched = true;
        goal.settle_count = 0U;
    }

    if (final_settle_latched)
    {
        v_along_target = 0.0f;
        v_cross_target = 0.0f;
        w_target_deg_s = 0.0f;
        goal.cross_rate_filtered_mm_s = 0.0f;
    }

    /*
     * Slew limits.
     */
    const float max_dv_along =
        CHASSIS_MOTION_LINEAR_ACCEL_MM_S2
        *
        CHASSIS_MOTION_DT;

    const float max_dv_cross =
        CHASSIS_MOTION_CROSS_ACCEL_MM_S2
        *
        CHASSIS_MOTION_DT;

    const float max_dw_deg_s =
        CHASSIS_MOTION_ANGULAR_ACCEL_DEG_S2
        *
        CHASSIS_MOTION_DT;

    /*
     * Always slew, including the transition to zero.  When an overshoot
     * recovery requests the opposite direction, first slew through zero so
     * the motor layer does not receive an abrupt sign reversal.
     */
    const bool direction_reversal =
        ((goal.v_along_mm_s > 0.0f) &&
         (v_along_target < 0.0f)) ||
        ((goal.v_along_mm_s < 0.0f) &&
         (v_along_target > 0.0f));

    if (final_settle_latched)
    {
        /* Stop immediately; do not spend the settle interval in the
         * drivetrain's low-speed PWM dead zone. */
        goal.v_along_mm_s = 0.0f;
        goal.v_cross_mm_s = 0.0f;
        goal.w_deg_s = 0.0f;
    }
    else
    {
        goal.v_along_mm_s =
            chassis_motion_slew(
                goal.v_along_mm_s,
                direction_reversal
                    ? 0.0f
                    : v_along_target,
                max_dv_along);

        goal.v_cross_mm_s =
            chassis_motion_slew(
                goal.v_cross_mm_s,
                v_cross_target,
                max_dv_cross);

        goal.w_deg_s =
            chassis_motion_slew(
                goal.w_deg_s,
                w_target_deg_s,
                max_dw_deg_s);
    }

    /*
     * Path-frame velocity -> world-frame velocity.
     */
    float vx_world =
        goal.v_along_mm_s * goal.axis_x
        +
        goal.v_cross_mm_s * goal.perp_x;

    float vy_world =
        goal.v_along_mm_s * goal.axis_y
        +
        goal.v_cross_mm_s * goal.perp_y;

#if CHASSIS_MOTION_ENABLE_CROSS_PD
    /* Keep the combined translational command within the requested speed. */
    const float translation_speed =
        chassis_motion_hypot2(
            vx_world,
            vy_world);

    if ((goal.max_speed_mm_s > 0.0f) &&
        (translation_speed > goal.max_speed_mm_s))
    {
        const float scale =
            goal.max_speed_mm_s /
            translation_speed;

        vx_world *= scale;
        vy_world *= scale;
    }
#endif

    /*
     * World-frame velocity -> current chassis/body frame velocity.
     */
    const float yaw_rad =
        odom->yaw_deg
        *
        CHASSIS_MOTION_DEG_TO_RAD;

    const float cy =
        cosf(yaw_rad);

    const float sy =
        sinf(yaw_rad);

    const float vx_body =
        cy * vx_world
        +
        sy * vy_world;

    const float vy_body =
        -sy * vx_world
        +
        cy * vy_world;

    const float w_rad_s =
        goal.w_deg_s
        *
        CHASSIS_MOTION_DEG_TO_RAD;

    /*
     * Completion test.
     */
    const float linear_speed =
        chassis_motion_hypot2(
            odom->body_vx_mm_s,
            odom->body_vy_mm_s);

    const bool position_ok =
        final_settle_latched
            ? ((remaining <=
                CHASSIS_MOTION_DISTANCE_RELEASE_TOL_MM) &&
               (remaining >=
                -CHASSIS_MOTION_DISTANCE_OVERSHOOT_RELEASE_TOL_MM))
            : ((remaining <=
                (target_zone_latched
                     ? CHASSIS_MOTION_DISTANCE_RELEASE_TOL_MM
                     : CHASSIS_MOTION_DISTANCE_TOL_MM)) &&
               (remaining >=
                -CHASSIS_MOTION_DISTANCE_OVERSHOOT_TOL_MM));

#if CHASSIS_MOTION_ENABLE_CROSS_TRACK
    const bool cross_ok =
        chassis_motion_absf(cross_track)
        <=
        (final_settle_latched
             ? CHASSIS_MOTION_CROSS_RELEASE_TOL_MM
             : CHASSIS_MOTION_CROSS_TOL_MM);
#else
    const bool cross_ok = true;
#endif

    const bool yaw_ok =
        chassis_motion_absf(yaw_error_deg)
        <=
        (final_settle_latched
             ? CHASSIS_MOTION_MOVE_YAW_RELEASE_TOL_DEG
             : CHASSIS_MOTION_MOVE_YAW_TOL_DEG);

    const bool motion_settled =
        (linear_speed <=
         CHASSIS_MOTION_SETTLE_LINEAR_MM_S)
        &&
        (chassis_motion_absf(gyro_z_dps)
         <=
         CHASSIS_MOTION_SETTLE_GYRO_DPS);

    if (final_settle_latched &&
        position_ok &&
        cross_ok &&
        yaw_ok &&
        motion_settled)
    {
        if (goal.settle_count <
            CHASSIS_MOTION_SETTLE_CYCLES)
        {
            goal.settle_count++;
        }
    }
    else
    {
        goal.settle_count =
            0U;
    }

    /*
     * Commit controller states only if command did not change.
     */
    bool command_still_active = false;

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    if ((g_chassis_motion_goal.command_id ==
         goal.command_id)
        &&
        (g_chassis_motion_goal.mode ==
         CHASSIS_MOTION_MODE_MOVE_DISTANCE))
    {
        g_chassis_motion_goal.v_along_mm_s =
            goal.v_along_mm_s;

        g_chassis_motion_goal.v_cross_mm_s =
            goal.v_cross_mm_s;

        g_chassis_motion_goal.w_deg_s =
            goal.w_deg_s;

        g_chassis_motion_goal.settle_count =
            goal.settle_count;

        g_chassis_motion_goal.target_zone_latched =
            target_zone_latched;

        g_chassis_motion_goal.final_settle_latched =
            final_settle_latched;

        g_chassis_motion_goal.cross_rate_filtered_mm_s =
            goal.cross_rate_filtered_mm_s;

        g_chassis_motion_status.traveled_distance_mm =
            traveled;

        g_chassis_motion_status.remaining_distance_mm =
            remaining;

        g_chassis_motion_status.cross_track_error_mm =
            cross_track;

        g_chassis_motion_status.remaining_angle_deg =
            yaw_error_deg;

        command_still_active =
            true;
    }

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);

    if (!command_still_active)
        return;

    if (final_settle_latched !=
        final_settle_was_latched)
    {
        if (final_settle_latched)
        {
            ESP_LOGI(
                CHASSIS_MOTION_TAG,
                "FINAL_SETTLE ENTER rem=%.1f cross=%.1f yaw=%.2f",
                (double)remaining,
                (double)cross_track,
                (double)yaw_error_deg);
        }
        else
        {
            ESP_LOGW(
                CHASSIS_MOTION_TAG,
                "FINAL_SETTLE RELEASE rem=%.1f cross=%.1f yaw=%.2f",
                (double)remaining,
                (double)cross_track,
                (double)yaw_error_deg);
        }
    }

    if (goal.settle_count >=
        CHASSIS_MOTION_SETTLE_CYCLES)
    {
        chassis_motion_set_terminal_state(
            CHASSIS_MOTION_MODE_DONE,
            CHASSIS_MOTION_ERROR_NONE,
            true);

        return;
    }

    motor_set_velocity(
        vx_body,
        vy_body,
        w_rad_s);

    chassis_motion_update_status_common(
        &goal,
        odom,
        vx_body,
        vy_body,
        w_rad_s);
}


/* ============================================================
 * 15. ROTATE control
 * ============================================================ */

static void chassis_motion_step_rotate(
    chassis_motion_goal_t goal,
    const CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE *odom)
{
    /*
     * IMPORTANT:
     * odom->yaw_deg must be continuous.
     *
     * Do NOT wrap this error because a request such as +270 deg or
     * +720 deg must preserve the requested signed rotation.
     */
    const float angle_error_deg =
        goal.target_yaw_deg -
        odom->yaw_deg;

    const float gyro_z_dps =
        odom->gyro_w_rad_s *
        CHASSIS_MOTION_RAD_TO_DEG;

    float w_target_deg_s =
        CHASSIS_MOTION_ROTATE_KP
        *
        angle_error_deg
        -
        CHASSIS_MOTION_ROTATE_KD
        *
        gyro_z_dps;

    w_target_deg_s =
        chassis_motion_clampf(
            w_target_deg_s,
            -goal.max_angular_speed_deg_s,
            goal.max_angular_speed_deg_s);


    if (chassis_motion_absf(angle_error_deg) >
        CHASSIS_MOTION_ROTATE_TOL_DEG)
    {
        float min_w =
            CHASSIS_MOTION_ROTATE_MIN_SPEED_DEG_S;

        if (min_w >
            goal.max_angular_speed_deg_s)
        {
            min_w =
                goal.max_angular_speed_deg_s;
        }

        if (chassis_motion_absf(w_target_deg_s) <
            min_w)
        {
            w_target_deg_s =
                (angle_error_deg > 0.0f)
                ? min_w
                : -min_w;
        }
    }
    else
    {
        w_target_deg_s =
            0.0f;
    }

    const float max_dw_deg_s =
        CHASSIS_MOTION_ANGULAR_ACCEL_DEG_S2
        *
        CHASSIS_MOTION_DT;

    if (w_target_deg_s == 0.0f)
    {
        goal.w_deg_s =
            0.0f;
    }
    else
    {
        goal.w_deg_s =
            chassis_motion_slew(
                goal.w_deg_s,
                w_target_deg_s,
                max_dw_deg_s);
    }

    const float w_rad_s =
        goal.w_deg_s
        *
        CHASSIS_MOTION_DEG_TO_RAD;

    const bool angle_ok =
        chassis_motion_absf(
            angle_error_deg)
        <=
        CHASSIS_MOTION_ROTATE_TOL_DEG;

    const bool rate_ok =
        chassis_motion_absf(
            gyro_z_dps)
        <=
        CHASSIS_MOTION_SETTLE_GYRO_DPS;

    if (angle_ok &&
        rate_ok)
    {
        if (goal.settle_count <
            CHASSIS_MOTION_SETTLE_CYCLES)
        {
            goal.settle_count++;
        }
    }
    else
    {
        goal.settle_count =
            0U;
    }

    bool command_still_active = false;

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    if ((g_chassis_motion_goal.command_id ==
         goal.command_id)
        &&
        (g_chassis_motion_goal.mode ==
         CHASSIS_MOTION_MODE_ROTATE))
    {
        g_chassis_motion_goal.w_deg_s =
            goal.w_deg_s;

        g_chassis_motion_goal.settle_count =
            goal.settle_count;

        g_chassis_motion_status.remaining_angle_deg =
            angle_error_deg;

        command_still_active =
            true;
    }

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);

    if (!command_still_active)
        return;

    if (goal.settle_count >=
        CHASSIS_MOTION_SETTLE_CYCLES)
    {
        chassis_motion_set_terminal_state(
            CHASSIS_MOTION_MODE_DONE,
            CHASSIS_MOTION_ERROR_NONE,
            true);

        return;
    }

    motor_set_velocity(
        0.0f,
        0.0f,
        w_rad_s);

    chassis_motion_update_status_common(
        &goal,
        odom,
        0.0f,
        0.0f,
        w_rad_s);
}


/* ============================================================
 * 16. Control task
 * ============================================================ */

static void chassis_motion_task(
    void *arg)
{
    (void)arg;

    TickType_t last_wake =
        xTaskGetTickCount();

    const TickType_t period =
        pdMS_TO_TICKS(
            CHASSIS_MOTION_PERIOD_MS);

    while (1)
    {
        chassis_motion_goal_t goal;

        taskENTER_CRITICAL(
            &g_chassis_motion_lock);

        goal =
            g_chassis_motion_goal;

        taskEXIT_CRITICAL(
            &g_chassis_motion_lock);

        if (!chassis_motion_mode_is_busy(
                goal.mode))
        {
            vTaskDelayUntil(
                &last_wake,
                period);

            continue;
        }

        CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE odom =
            {0};

        const chassis_motion_error_t dep_error =
            chassis_motion_check_dependencies(&odom);

        if (dep_error != CHASSIS_MOTION_ERROR_NONE)
        {
            // 不调用 fail，只记录警告，并停止输出（安全）
            ESP_LOGW(CHASSIS_MOTION_TAG,
                    "Dependency check failed (%d), motion halted until ready",
                    (int)dep_error);

            // 停止电机，但保留当前命令状态不变（或清除命令）
            motor_stop();

            // 将状态临时置为 IDLE，但不标记为错误，下次循环可重试
            taskENTER_CRITICAL(&g_chassis_motion_lock);
            g_chassis_motion_goal.mode = CHASSIS_MOTION_MODE_IDLE;
            g_chassis_motion_goal.v_along_mm_s = 0.0f;
            g_chassis_motion_goal.v_cross_mm_s = 0.0f;
            g_chassis_motion_goal.w_deg_s = 0.0f;
            g_chassis_motion_status.mode = CHASSIS_MOTION_MODE_IDLE;
            g_chassis_motion_status.busy = false;
            g_chassis_motion_status.error = CHASSIS_MOTION_ERROR_NONE; // 清除错误
            taskEXIT_CRITICAL(&g_chassis_motion_lock);

            vTaskDelayUntil(&last_wake, period);
            continue;
        }
        const int64_t now_us =
            esp_timer_get_time();

        if (now_us >
            goal.deadline_us)
        {
            chassis_motion_fail(
                CHASSIS_MOTION_ERROR_TIMEOUT);

            vTaskDelayUntil(
                &last_wake,
                period);

            continue;
        }

        if (goal.mode ==
            CHASSIS_MOTION_MODE_MOVE_DISTANCE)
        {
            chassis_motion_step_move_distance(
                goal,
                &odom);
        }
        else if (goal.mode ==
                 CHASSIS_MOTION_MODE_ROTATE)
        {
            chassis_motion_step_rotate(
                goal,
                &odom);
        }

        vTaskDelayUntil(
            &last_wake,
            period);
    }
}


/* ============================================================
 * 17. Public implementation
 * ============================================================ */

esp_err_t chassis_motion_init(void)
{
    if (g_chassis_motion_initialized)
        return ESP_OK;

    CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE odom =
        {0};

    const chassis_motion_error_t dep_error =
        chassis_motion_check_dependencies(
            &odom);

    if (dep_error !=
        CHASSIS_MOTION_ERROR_NONE)
    {
        ESP_LOGE(
            CHASSIS_MOTION_TAG,
            "dependency not ready, error=%d",
            (int)dep_error);

        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    g_chassis_motion_goal =
        (chassis_motion_goal_t){0};

    g_chassis_motion_goal.mode =
        CHASSIS_MOTION_MODE_IDLE;

    g_chassis_motion_status =
        (chassis_motion_status_t){0};

    g_chassis_motion_status.initialized =
        true;

    g_chassis_motion_status.mode =
        CHASSIS_MOTION_MODE_IDLE;

    g_chassis_motion_status.error =
        CHASSIS_MOTION_ERROR_NONE;

    g_chassis_motion_status.x_mm =
        odom.x_mm;

    g_chassis_motion_status.y_mm =
        odom.y_mm;

    g_chassis_motion_status.yaw_deg =
        odom.yaw_deg;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);

    const BaseType_t task_result =
        xTaskCreate(
            chassis_motion_task,
            "chassis_motion",
            CHASSIS_MOTION_TASK_STACK_SIZE,
            NULL,
            CHASSIS_MOTION_TASK_PRIORITY,
            &g_chassis_motion_task_handle);

    if (task_result != pdPASS)
    {
        g_chassis_motion_task_handle =
            NULL;

        return ESP_ERR_NO_MEM;
    }

    g_chassis_motion_initialized =
        true;

    ESP_LOGI(
        CHASSIS_MOTION_TAG,
        "initialized");

    return ESP_OK;
}


esp_err_t chassis_motion_move_distance(
    float direction_deg,
    float distance_mm,
    float max_speed_mm_s)
{
    if (!g_chassis_motion_initialized)
        return ESP_ERR_INVALID_STATE;

    if ((!chassis_motion_float_valid(
            direction_deg))
        ||
        (!chassis_motion_float_valid(
            distance_mm))
        ||
        (!chassis_motion_float_valid(
            max_speed_mm_s)))
    {
        return ESP_ERR_INVALID_ARG;
    }

    max_speed_mm_s =
        chassis_motion_absf(
            max_speed_mm_s);

    if (max_speed_mm_s < 1.0f)
        return ESP_ERR_INVALID_ARG;

    if (max_speed_mm_s <
        CHASSIS_MOTION_MOVE_MIN_SPEED_MM_S)
    {
        ESP_LOGW(
            CHASSIS_MOTION_TAG,
            "MOVE rejected: max speed %.1f mm/s is below "
            "minimum driveable speed %.1f mm/s",
            (double)max_speed_mm_s,
            (double)CHASSIS_MOTION_MOVE_MIN_SPEED_MM_S);

        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Convert negative distance to an equivalent positive-distance
     * direction.
     */
    if (distance_mm < 0.0f)
    {
        distance_mm =
            -distance_mm;

        direction_deg +=
            180.0f;
    }

    CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE odom =
        {0};

    const chassis_motion_error_t dep_error =
        chassis_motion_check_dependencies(
            &odom);

    if (dep_error !=
        CHASSIS_MOTION_ERROR_NONE)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const float physical_linear_limit =
        sqrtf(
            MOTOR_MAX_VX_MM_S *
            MOTOR_MAX_VX_MM_S
            +
            MOTOR_MAX_VY_MM_S *
            MOTOR_MAX_VY_MM_S);

    if (max_speed_mm_s >
        physical_linear_limit)
    {
        max_speed_mm_s =
            physical_linear_limit;
    }

    /*
     * Very small command: consider it already reached.
     */
    if (distance_mm <=
        CHASSIS_MOTION_DISTANCE_TOL_MM)
    {
        motor_stop();

        taskENTER_CRITICAL(
            &g_chassis_motion_lock);

        g_chassis_motion_status.mode =
            CHASSIS_MOTION_MODE_DONE;

        g_chassis_motion_status.busy =
            false;

        g_chassis_motion_status.target_reached =
            true;

        g_chassis_motion_status.error =
            CHASSIS_MOTION_ERROR_NONE;

        taskEXIT_CRITICAL(
            &g_chassis_motion_lock);

        return ESP_OK;
    }

    const float world_direction_deg =
        odom.yaw_deg
        +
        direction_deg;

    const float world_direction_rad =
        world_direction_deg
        *
        CHASSIS_MOTION_DEG_TO_RAD;

    chassis_motion_goal_t goal =
        {0};

    goal.mode =
        CHASSIS_MOTION_MODE_MOVE_DISTANCE;

    goal.start_x_mm =
        odom.x_mm;

    goal.start_y_mm =
        odom.y_mm;

    goal.start_yaw_deg =
        odom.yaw_deg;

    goal.axis_x =
        cosf(
            world_direction_rad);

    goal.axis_y =
        sinf(
            world_direction_rad);

    goal.perp_x =
        -goal.axis_y;

    goal.perp_y =
        goal.axis_x;

    goal.direction_deg =
        chassis_motion_wrap_180(
            direction_deg);

    goal.distance_mm =
        distance_mm;

    goal.max_speed_mm_s =
        max_speed_mm_s;

    goal.target_x_mm =
        goal.start_x_mm
        +
        distance_mm
        *
        goal.axis_x;

    goal.target_y_mm =
        goal.start_y_mm
        +
        distance_mm
        *
        goal.axis_y;

    goal.target_yaw_deg =
        goal.start_yaw_deg;

    goal.start_time_us =
        esp_timer_get_time();

    goal.deadline_us =
        chassis_motion_calculate_deadline_us(
            distance_mm,
            max_speed_mm_s,
            CHASSIS_MOTION_LINEAR_ACCEL_MM_S2);

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    goal.command_id =
        g_chassis_motion_next_command_id++;

    if (g_chassis_motion_next_command_id == 0U)
        g_chassis_motion_next_command_id = 1U;

    g_chassis_motion_goal =
        goal;

    g_chassis_motion_status =
        (chassis_motion_status_t){0};

    g_chassis_motion_status.initialized =
        true;

    g_chassis_motion_status.busy =
        true;

    g_chassis_motion_status.target_reached =
        false;

    g_chassis_motion_status.mode =
        CHASSIS_MOTION_MODE_MOVE_DISTANCE;

    g_chassis_motion_status.error =
        CHASSIS_MOTION_ERROR_NONE;

    g_chassis_motion_status.command_id =
        goal.command_id;

    g_chassis_motion_status.x_mm =
        odom.x_mm;

    g_chassis_motion_status.y_mm =
        odom.y_mm;

    g_chassis_motion_status.yaw_deg =
        odom.yaw_deg;

    g_chassis_motion_status.start_x_mm =
        goal.start_x_mm;

    g_chassis_motion_status.start_y_mm =
        goal.start_y_mm;

    g_chassis_motion_status.start_yaw_deg =
        goal.start_yaw_deg;

    g_chassis_motion_status.target_x_mm =
        goal.target_x_mm;

    g_chassis_motion_status.target_y_mm =
        goal.target_y_mm;

    g_chassis_motion_status.target_yaw_deg =
        goal.target_yaw_deg;

    g_chassis_motion_status.direction_deg =
        goal.direction_deg;

    g_chassis_motion_status.target_distance_mm =
        goal.distance_mm;

    g_chassis_motion_status.remaining_distance_mm =
        goal.distance_mm;

    g_chassis_motion_status.start_time_us =
        goal.start_time_us;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);

    ESP_LOGI(
        CHASSIS_MOTION_TAG,
        "MOVE id=%lu dir=%.1fdeg dist=%.1fmm vmax=%.1fmm/s",
        (unsigned long)goal.command_id,
        goal.direction_deg,
        goal.distance_mm,
        goal.max_speed_mm_s);

    return ESP_OK;
}


esp_err_t chassis_motion_rotate(
    float angle_deg,
    float angular_speed_deg_s)
{
    if (!g_chassis_motion_initialized)
        return ESP_ERR_INVALID_STATE;

    if ((!chassis_motion_float_valid(
            angle_deg))
        ||
        (!chassis_motion_float_valid(
            angular_speed_deg_s)))
    {
        return ESP_ERR_INVALID_ARG;
    }

    angular_speed_deg_s =
        chassis_motion_absf(
            angular_speed_deg_s);

    if (angular_speed_deg_s < 0.1f)
        return ESP_ERR_INVALID_ARG;

    CHASSIS_MOTION_ODOMETRY_SNAPSHOT_TYPE odom =
        {0};

    const chassis_motion_error_t dep_error =
        chassis_motion_check_dependencies(
            &odom);

    if (dep_error !=
        CHASSIS_MOTION_ERROR_NONE)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const float motor_w_limit_deg_s =
        MOTOR_MAX_W_RAD_S
        *
        CHASSIS_MOTION_RAD_TO_DEG;

    if (angular_speed_deg_s >
        motor_w_limit_deg_s)
    {
        angular_speed_deg_s =
            motor_w_limit_deg_s;
    }

    if (chassis_motion_absf(
            angle_deg)
        <=
        CHASSIS_MOTION_ROTATE_TOL_DEG)
    {
        motor_stop();

        taskENTER_CRITICAL(
            &g_chassis_motion_lock);

        g_chassis_motion_status.mode =
            CHASSIS_MOTION_MODE_DONE;

        g_chassis_motion_status.busy =
            false;

        g_chassis_motion_status.target_reached =
            true;

        g_chassis_motion_status.error =
            CHASSIS_MOTION_ERROR_NONE;

        taskEXIT_CRITICAL(
            &g_chassis_motion_lock);

        return ESP_OK;
    }

    chassis_motion_goal_t goal =
        {0};

    goal.mode =
        CHASSIS_MOTION_MODE_ROTATE;

    goal.start_x_mm =
        odom.x_mm;

    goal.start_y_mm =
        odom.y_mm;

    goal.start_yaw_deg =
        odom.yaw_deg;

    goal.target_x_mm =
        odom.x_mm;

    goal.target_y_mm =
        odom.y_mm;

    /*
     * Keep continuous yaw. This preserves requests such as +270 deg.
     */
    goal.target_yaw_deg =
        odom.yaw_deg
        +
        angle_deg;

    goal.angle_deg =
        angle_deg;

    goal.max_angular_speed_deg_s =
        angular_speed_deg_s;

    goal.start_time_us =
        esp_timer_get_time();

    goal.deadline_us =
        chassis_motion_calculate_deadline_us(
            angle_deg,
            angular_speed_deg_s,
            CHASSIS_MOTION_ANGULAR_ACCEL_DEG_S2);

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    goal.command_id =
        g_chassis_motion_next_command_id++;

    if (g_chassis_motion_next_command_id == 0U)
        g_chassis_motion_next_command_id = 1U;

    g_chassis_motion_goal =
        goal;

    g_chassis_motion_status =
        (chassis_motion_status_t){0};

    g_chassis_motion_status.initialized =
        true;

    g_chassis_motion_status.busy =
        true;

    g_chassis_motion_status.target_reached =
        false;

    g_chassis_motion_status.mode =
        CHASSIS_MOTION_MODE_ROTATE;

    g_chassis_motion_status.error =
        CHASSIS_MOTION_ERROR_NONE;

    g_chassis_motion_status.command_id =
        goal.command_id;

    g_chassis_motion_status.x_mm =
        odom.x_mm;

    g_chassis_motion_status.y_mm =
        odom.y_mm;

    g_chassis_motion_status.yaw_deg =
        odom.yaw_deg;

    g_chassis_motion_status.start_x_mm =
        goal.start_x_mm;

    g_chassis_motion_status.start_y_mm =
        goal.start_y_mm;

    g_chassis_motion_status.start_yaw_deg =
        goal.start_yaw_deg;

    g_chassis_motion_status.target_x_mm =
        goal.target_x_mm;

    g_chassis_motion_status.target_y_mm =
        goal.target_y_mm;

    g_chassis_motion_status.target_yaw_deg =
        goal.target_yaw_deg;

    g_chassis_motion_status.target_angle_deg =
        goal.angle_deg;

    g_chassis_motion_status.remaining_angle_deg =
        goal.angle_deg;

    g_chassis_motion_status.start_time_us =
        goal.start_time_us;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);

    ESP_LOGI(
        CHASSIS_MOTION_TAG,
        "ROTATE id=%lu angle=%.1fdeg wmax=%.1fdeg/s",
        (unsigned long)goal.command_id,
        goal.angle_deg,
        goal.max_angular_speed_deg_s);

    return ESP_OK;
}


void chassis_motion_stop(void)
{
    motor_stop();

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    g_chassis_motion_goal.mode =
        CHASSIS_MOTION_MODE_IDLE;

    g_chassis_motion_goal.v_along_mm_s =
        0.0f;

    g_chassis_motion_goal.v_cross_mm_s =
        0.0f;

    g_chassis_motion_goal.w_deg_s =
        0.0f;

    g_chassis_motion_goal.settle_count =
        0U;

    g_chassis_motion_status.mode =
        CHASSIS_MOTION_MODE_IDLE;

    g_chassis_motion_status.busy =
        false;

    g_chassis_motion_status.target_reached =
        false;

    g_chassis_motion_status.error =
        CHASSIS_MOTION_ERROR_NONE;

    g_chassis_motion_status.command_vx_mm_s =
        0.0f;

    g_chassis_motion_status.command_vy_mm_s =
        0.0f;

    g_chassis_motion_status.command_w_rad_s =
        0.0f;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);
}


void chassis_motion_cancel(void)
{
    motor_stop();

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    g_chassis_motion_goal.mode =
        CHASSIS_MOTION_MODE_CANCELLED;

    g_chassis_motion_goal.v_along_mm_s =
        0.0f;

    g_chassis_motion_goal.v_cross_mm_s =
        0.0f;

    g_chassis_motion_goal.w_deg_s =
        0.0f;

    g_chassis_motion_goal.settle_count =
        0U;

    g_chassis_motion_status.mode =
        CHASSIS_MOTION_MODE_CANCELLED;

    g_chassis_motion_status.busy =
        false;

    g_chassis_motion_status.target_reached =
        false;

    g_chassis_motion_status.command_vx_mm_s =
        0.0f;

    g_chassis_motion_status.command_vy_mm_s =
        0.0f;

    g_chassis_motion_status.command_w_rad_s =
        0.0f;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);
}


void chassis_motion_emergency_stop(void)
{
    motor_emergency_stop();

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    g_chassis_motion_goal.mode =
        CHASSIS_MOTION_MODE_CANCELLED;

    g_chassis_motion_status.mode =
        CHASSIS_MOTION_MODE_CANCELLED;

    g_chassis_motion_status.busy =
        false;

    g_chassis_motion_status.target_reached =
        false;

    g_chassis_motion_status.command_vx_mm_s =
        0.0f;

    g_chassis_motion_status.command_vy_mm_s =
        0.0f;

    g_chassis_motion_status.command_w_rad_s =
        0.0f;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);
}


bool chassis_motion_is_busy(void)
{
    bool busy;

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    busy =
        chassis_motion_mode_is_busy(
            g_chassis_motion_goal.mode);

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);

    return busy;
}


void chassis_motion_get_status(
    chassis_motion_status_t *status)
{
    if (status == NULL)
        return;

    taskENTER_CRITICAL(
        &g_chassis_motion_lock);

    *status =
        g_chassis_motion_status;

    taskEXIT_CRITICAL(
        &g_chassis_motion_lock);
}


esp_err_t chassis_motion_wait(
    TickType_t timeout)
{
    const TickType_t start_tick =
        xTaskGetTickCount();

    while (1)
    {
        chassis_motion_status_t status;

        chassis_motion_get_status(
            &status);

        if (!status.busy)
        {
            if (status.mode ==
                CHASSIS_MOTION_MODE_DONE)
            {
                return ESP_OK;
            }

            if ((status.mode ==
                 CHASSIS_MOTION_MODE_ERROR)
                ||
                (status.mode ==
                 CHASSIS_MOTION_MODE_CANCELLED))
            {
                return ESP_FAIL;
            }

            return ESP_OK;
        }

        if (timeout !=
            portMAX_DELAY)
        {
            const TickType_t elapsed =
                xTaskGetTickCount()
                -
                start_tick;

            if (elapsed >= timeout)
                return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(
            pdMS_TO_TICKS(
                CHASSIS_MOTION_PERIOD_MS));
    }
}


#endif /* CHASSIS_MOTION_IMPLEMENTATION */


#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_MOTION_PD_H */
