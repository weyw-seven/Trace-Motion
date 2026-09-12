#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. TB6612 hardware pins
 * ============================================================ */

#define MOTOR_STBY_PIN      GPIO_NUM_2

#define MOTOR_A_IN1         GPIO_NUM_40
#define MOTOR_A_IN2         GPIO_NUM_41
#define MOTOR_A_PWM         GPIO_NUM_42

#define MOTOR_B_IN1         GPIO_NUM_17
#define MOTOR_B_IN2         GPIO_NUM_16
#define MOTOR_B_PWM         GPIO_NUM_15

#define MOTOR_D_IN1         GPIO_NUM_10
#define MOTOR_D_IN2         GPIO_NUM_9
#define MOTOR_D_PWM         GPIO_NUM_3

/* ============================================================
 * 2. Encoder pins
 *
 * Fill these later. GPIO_NUM_NC = not configured.
 * ============================================================ */

#ifndef MOTOR_A_ENC_A
#define MOTOR_A_ENC_A       GPIO_NUM_39
#endif
#ifndef MOTOR_A_ENC_B
#define MOTOR_A_ENC_B       GPIO_NUM_38
#endif

#ifndef MOTOR_B_ENC_A
#define MOTOR_B_ENC_A       GPIO_NUM_18
#endif
#ifndef MOTOR_B_ENC_B
#define MOTOR_B_ENC_B       GPIO_NUM_8
#endif

#ifndef MOTOR_D_ENC_A
#define MOTOR_D_ENC_A       GPIO_NUM_11
#endif
#ifndef MOTOR_D_ENC_B
#define MOTOR_D_ENC_B       GPIO_NUM_12
#endif

/* ============================================================
 * 3. Encoder / chassis geometry
 *
 * CPR = actual PCNT counts per ONE wheel revolution.
 * Leave 0 until measured.
 * ============================================================ */

#ifndef MOTOR_A_ENCODER_CPR
#define MOTOR_A_ENCODER_CPR             450.0f
#endif
#ifndef MOTOR_B_ENCODER_CPR
#define MOTOR_B_ENCODER_CPR             420.0f
#endif
#ifndef MOTOR_D_ENCODER_CPR
#define MOTOR_D_ENCODER_CPR             420.0f
#endif

#ifndef MOTOR_WHEEL_DIAMETER_MM
#define MOTOR_WHEEL_DIAMETER_MM         55.0f
#endif

#ifndef MOTOR_CHASSIS_RADIUS_MM
#define MOTOR_CHASSIS_RADIUS_MM         95.0f
#endif

/* ============================================================
 * 4. PWM
 * ============================================================ */

#define MOTOR_PWM_FREQ_HZ               10000
#define MOTOR_LEDC_TIMER                LEDC_TIMER_0
#define MOTOR_LEDC_MODE                 LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_RES                  LEDC_TIMER_10_BIT
#define MOTOR_PWM_HW_MAX                1023U

#define MOTOR_LEDC_CH_A                 LEDC_CHANNEL_0
#define MOTOR_LEDC_CH_B                 LEDC_CHANNEL_1
#define MOTOR_LEDC_CH_D                 LEDC_CHANNEL_2

#ifndef MOTOR_A_PWM_LIMIT
#define MOTOR_A_PWM_LIMIT               600.0f
#endif
#ifndef MOTOR_B_PWM_LIMIT
#define MOTOR_B_PWM_LIMIT               600.0f
#endif
#ifndef MOTOR_D_PWM_LIMIT
#define MOTOR_D_PWM_LIMIT               600.0f
#endif

/* ============================================================
 * 5. Control loop
 * ============================================================ */

#ifndef MOTOR_CONTROL_PERIOD_MS
#define MOTOR_CONTROL_PERIOD_MS         10U
#endif

#define MOTOR_CONTROL_DT \
    ((float)MOTOR_CONTROL_PERIOD_MS / 1000.0f)

#ifndef MOTOR_TASK_STACK_SIZE
#define MOTOR_TASK_STACK_SIZE           4096
#endif

#ifndef MOTOR_TASK_PRIORITY
#define MOTOR_TASK_PRIORITY             5
#endif

/* ============================================================
 * 6. Chassis coordinates and inverse kinematics
 *
 * vx > 0 : forward
 * vy > 0 : left
 * w  > 0 : counter-clockwise
 *
 * vx, vy       : mm/s
 * w            : rad/s
 * wheel speed  : mm/s
 *
 * Initial standard 120-degree 3-omni matrix:
 *
 * vA =  0.000 vx + 1.000 vy + 1.000 Lw
 * vB = -0.866 vx - 0.500 vy + 1.000 Lw
 * vD = +0.866 vx - 0.500 vy + 1.000 Lw
 *
 * Verify wheel numbering and directions on the real chassis.
 * ============================================================ */

#ifndef MOTOR_A_KX
#define MOTOR_A_KX                     0.0f
#endif
#ifndef MOTOR_A_KY
#define MOTOR_A_KY                     -1.0f
#endif
#ifndef MOTOR_A_KW
#define MOTOR_A_KW                     1.0f
#endif

#ifndef MOTOR_B_KX
#define MOTOR_B_KX                    0.8660254f
#endif
#ifndef MOTOR_B_KY
#define MOTOR_B_KY                    0.5f
#endif
#ifndef MOTOR_B_KW
#define MOTOR_B_KW                    1.0f
#endif

#ifndef MOTOR_D_KX
#define MOTOR_D_KX                    -0.8660254f
#endif
#ifndef MOTOR_D_KY
#define MOTOR_D_KY                     0.5f
#endif
#ifndef MOTOR_D_KW
#define MOTOR_D_KW                     1.0f
#endif

/* Electrical motor direction correction. */
#ifndef MOTOR_A_POLARITY
#define MOTOR_A_POLARITY               -1.0f
#endif
#ifndef MOTOR_B_POLARITY
#define MOTOR_B_POLARITY               1.0f
#endif
#ifndef MOTOR_D_POLARITY
#define MOTOR_D_POLARITY               1.0f
#endif

/* Encoder count direction correction. */
#ifndef MOTOR_A_ENCODER_POLARITY
#define MOTOR_A_ENCODER_POLARITY       1.0f
#endif
#ifndef MOTOR_B_ENCODER_POLARITY
#define MOTOR_B_ENCODER_POLARITY       1.0f
#endif
#ifndef MOTOR_D_ENCODER_POLARITY
#define MOTOR_D_ENCODER_POLARITY       1.0f
#endif

/* ============================================================
 * 7. Wheel speed limits / zero zone
 * ============================================================ */

/* 0 = disabled until measured. */
#ifndef MOTOR_WHEEL_SPEED_LIMIT_MM_S
#define MOTOR_WHEEL_SPEED_LIMIT_MM_S   1500.0f
#endif

#ifndef MOTOR_TARGET_ZERO_MM_S
#define MOTOR_TARGET_ZERO_MM_S         5.0f
#endif

#ifndef MOTOR_ACTUAL_ZERO_MM_S
#define MOTOR_ACTUAL_ZERO_MM_S         5.0f
#endif

/* Encoder measurement low-pass only; command is NOT filtered. */
#ifndef MOTOR_ENCODER_FILTER_ALPHA
#define MOTOR_ENCODER_FILTER_ALPHA     1.00f
#endif

/*
 * Optional legacy scaling for the measured feed-forward intercept B.
 *
 * The PI controller below is now responsible for adapting to floor/load
 * changes independently on each wheel.  Therefore B scaling is disabled by
 * default.  MOTOR_PWM_MIN may still be set explicitly for comparison tests.
 *
 *     B_effective[i] = clamp(B_calibrated[i] * scale, 0, B_MAX)
 *     scale = max(1, PWM_MIN / min(B_calibrated))
 *
 * MOTOR_ENCODER_FILTER_ALPHA remains exclusively a measurement filter and is
 * intentionally unrelated to this calculation.
 */
#ifndef MOTOR_PWM_MIN
#ifdef PWM_MIN
#define MOTOR_PWM_MIN                  PWM_MIN
#else
#define MOTOR_PWM_MIN                  80.0f
#endif
#endif

#ifndef MOTOR_EFFECTIVE_B_MAX
#define MOTOR_EFFECTIVE_B_MAX          450.0f
#endif

#ifndef MOTOR_STARTUP_BOOST_ENABLE
#define MOTOR_STARTUP_BOOST_ENABLE     0
#endif

/* A hard running PWM floor prevents PI from correcting an over-speed wheel. */
#ifndef MOTOR_RUN_PWM_FLOOR_ENABLE
#define MOTOR_RUN_PWM_FLOOR_ENABLE     0
#endif

#ifndef MOTOR_PCNT_GLITCH_NS
#define MOTOR_PCNT_GLITCH_NS           1000U
#endif

#define MOTOR_PCNT_HIGH_LIMIT          30000
#define MOTOR_PCNT_LOW_LIMIT          -30000

/* ============================================================
 * 8. Feedforward calibration
 *
 * pwm_ff = sign(v) * (K * |v| + B)
 *
 * B represents practical running deadband compensation.
 * Tune forward and reverse separately.
 * ============================================================ */

/* Motor A */
#define MOTOR_A_FF_K_FWD    0.2200f
#define MOTOR_A_FF_B_FWD    58.9f

#define MOTOR_A_FF_K_REV    0.2310f
#define MOTOR_A_FF_B_REV    58.8f

/* Motor B */
#define MOTOR_B_FF_K_FWD    0.2138f
#define MOTOR_B_FF_B_FWD    58.9f

#define MOTOR_B_FF_K_REV    0.2262f
#define MOTOR_B_FF_B_REV    58.9f

/* Motor D */
#define MOTOR_D_FF_K_FWD    0.2074f
#define MOTOR_D_FF_B_FWD    59.0f

#define MOTOR_D_FF_K_REV    0.2135f
#define MOTOR_D_FF_B_REV    58.9f


/* ============================================================
 * 9. Wheel-speed PID
 *
 * Defaults are intentionally zero.
 * Tune in this order:
 *   feedforward -> Kp -> Ki -> Kd(if needed)
 * ============================================================ */

#ifndef MOTOR_A_PID_KP
#define MOTOR_A_PID_KP                 0.22f
#endif
#ifndef MOTOR_A_PID_KI
#define MOTOR_A_PID_KI                 8.0f
#endif
#ifndef MOTOR_A_PID_KD
#define MOTOR_A_PID_KD                 0.022f
#endif
#ifndef MOTOR_A_PID_I_LIMIT
#define MOTOR_A_PID_I_LIMIT            250.0f
#endif

#ifndef MOTOR_B_PID_KP
#define MOTOR_B_PID_KP                 0.18f
#endif
#ifndef MOTOR_B_PID_KI
#define MOTOR_B_PID_KI                 6.0f
#endif
#ifndef MOTOR_B_PID_KD
#define MOTOR_B_PID_KD                 0.020f
#endif
#ifndef MOTOR_B_PID_I_LIMIT
#define MOTOR_B_PID_I_LIMIT            250.0f
#endif

#ifndef MOTOR_D_PID_KP
#define MOTOR_D_PID_KP                 0.28f
#endif
#ifndef MOTOR_D_PID_KI
#define MOTOR_D_PID_KI                 6.0f
#endif
#ifndef MOTOR_D_PID_KD
#define MOTOR_D_PID_KD                 0.020f
#endif
#ifndef MOTOR_D_PID_I_LIMIT
#define MOTOR_D_PID_I_LIMIT            250.0f
#endif

/* Unwind accumulated load compensation faster after the load becomes lower. */
#ifndef MOTOR_PID_I_UNWIND_MULTIPLIER
#define MOTOR_PID_I_UNWIND_MULTIPLIER  3.0f
#endif

/* ============================================================
 * Normalized chassis command scale
 *
 * Public command:
 *
 *      vx, vy, w ∈ [-1, 1]
 *
 * Internal physical units:
 *
 *      vx, vy -> mm/s
 *      w      -> rad/s
 * ============================================================ */

#ifndef MOTOR_MAX_VX_MM_S
#define MOTOR_MAX_VX_MM_S       2000.0f
#endif

#ifndef MOTOR_MAX_VY_MM_S
#define MOTOR_MAX_VY_MM_S       2000.0f
#endif

#ifndef MOTOR_MAX_W_RAD_S
#define MOTOR_MAX_W_RAD_S       6.00f
#endif

#ifndef MOTOR_ENCODER_SPEED_WINDOW_SAMPLES
#define MOTOR_ENCODER_SPEED_WINDOW_SAMPLES   5U
#endif

#define MOTOR_A_START_PWM_FWD    308.0f /* legacy compatibility */
#define MOTOR_A_START_PWM_REV    330.0f /* legacy compatibility */

#define MOTOR_B_START_PWM_FWD    176.0f /* legacy compatibility */
#define MOTOR_B_START_PWM_REV    286.0f /* legacy compatibility */

#define MOTOR_D_START_PWM_FWD    286.0f /* legacy compatibility */
#define MOTOR_D_START_PWM_REV    198.0f /* legacy compatibility */

/*
 * Legacy per-wheel running-floor overrides are kept for source compatibility.
 * The new controller derives the active running floor from the compensated B
 * value, so these values are no longer used when startup boost is disabled.
 */
#ifndef MOTOR_A_RUN_PWM_FWD
#define MOTOR_A_RUN_PWM_FWD       MOTOR_A_FF_B_FWD
#endif
#ifndef MOTOR_A_RUN_PWM_REV
#define MOTOR_A_RUN_PWM_REV       MOTOR_A_FF_B_REV
#endif
#ifndef MOTOR_B_RUN_PWM_FWD
#define MOTOR_B_RUN_PWM_FWD       MOTOR_B_FF_B_FWD
#endif
#ifndef MOTOR_B_RUN_PWM_REV
#define MOTOR_B_RUN_PWM_REV       MOTOR_B_FF_B_REV
#endif
#ifndef MOTOR_D_RUN_PWM_FWD
#define MOTOR_D_RUN_PWM_FWD       MOTOR_D_FF_B_FWD
#endif
#ifndef MOTOR_D_RUN_PWM_REV
#define MOTOR_D_RUN_PWM_REV       MOTOR_D_FF_B_REV
#endif

/*
 * Wheel is considered successfully started once measured
 * speed exceeds this value.
 */
#define MOTOR_STARTUP_EXIT_SPEED_MM_S    15.0f

/*
 * Re-arm startup boost when a commanded wheel has stopped
 * for several consecutive control cycles.
 */
#ifndef MOTOR_STARTUP_REARM_CYCLES
#define MOTOR_STARTUP_REARM_CYCLES       3U
#endif

/* Adaptive static-friction compensation. */
#ifndef MOTOR_STARTUP_BOOST_RAMP_PWM_PER_S
#define MOTOR_STARTUP_BOOST_RAMP_PWM_PER_S  300.0f
#endif

#ifndef MOTOR_STARTUP_BOOST_RELEASE_PWM_PER_S
#define MOTOR_STARTUP_BOOST_RELEASE_PWM_PER_S 500.0f
#endif

#ifndef MOTOR_STARTUP_BOOST_MAX_PWM
#define MOTOR_STARTUP_BOOST_MAX_PWM       500.0f
#endif

#ifndef MOTOR_STARTUP_CONFIRM_CYCLES
#define MOTOR_STARTUP_CONFIRM_CYCLES      3U
#endif

/*
 * Diagnostic stall detection.  The chassis needs a short period to overcome
 * static friction after a low-speed correction or a Motion->Motion junction.
 * Keep the fault as a safety backstop, but do not turn a normal 0.3 s encoder
 * quiet period during fine settling into an emergency stop.
 */
#ifndef MOTOR_STALL_DETECT_TARGET_SPEED_MM_S
#define MOTOR_STALL_DETECT_TARGET_SPEED_MM_S 20.0f
#endif

#ifndef MOTOR_STALL_DETECT_ACTUAL_SPEED_MM_S
#define MOTOR_STALL_DETECT_ACTUAL_SPEED_MM_S 5.0f
#endif

#ifndef MOTOR_STALL_DETECT_TIME_MS
#define MOTOR_STALL_DETECT_TIME_MS          1200U
#endif


/* ============================================================
 * 10. Public API
 * ============================================================ */

typedef struct
{
    float target_speed_mm_s;
    float actual_speed_mm_s;
    int32_t encoder_count;
    float output_pwm;
    bool encoder_ready;
    bool startup_active;
    bool stall_suspected;
    uint32_t stall_elapsed_ms;
} motor_wheel_status_t;

typedef struct
{
    bool initialized;
    bool enabled;
    bool closed_loop_ready;
    bool raw_debug_mode;
    bool emergency_stop;

    float target_vx_mm_s;
    float target_vy_mm_s;
    float target_w_rad_s;

    motor_wheel_status_t A;
    motor_wheel_status_t B;
    motor_wheel_status_t D;
} motor_status_t;

esp_err_t motor_control_init(void);

void Set_motor(
    float vx_norm,
    float vy_norm,
    float w_norm);

void motor_set_velocity(
    float vx_mm_s,
    float vy_mm_s,
    float w_rad_s);

void motor_stop(void);
void motor_emergency_stop(void);
void motor_control_enable(bool enable);
bool motor_control_is_ready(void);
bool motor_is_emergency_stopped(void);
void motor_get_status(motor_status_t *status);

void motor_get_wheel_rps(
    float *rps_a,
    float *rps_b,
    float *rps_d);

/*
 * Calibration/debug only.
 * Direct signed PWM, bypassing kinematics/FF/PID.
 * Any later Set_motor() call exits raw mode.
 */
void motor_debug_set_raw_pwm(
    float pwm_a,
    float pwm_b,
    float pwm_d);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 *
 * Define MOTOR_CONTROL_IMPLEMENTATION in exactly one .c file.
 * ============================================================ */

#ifdef MOTOR_CONTROL_IMPLEMENTATION

#define MOTOR_PI 3.14159265358979323846f

static const char *MOTOR_TAG = "MOTOR";

typedef struct
{
    float vx;
    float vy;
    float w;
} motor_command_t;

typedef struct
{
    float A;
    float B;
    float D;
} motor_raw_pwm_command_t;

typedef struct
{
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t pwm_pin;
    ledc_channel_t pwm_channel;

    gpio_num_t enc_a_pin;
    gpio_num_t enc_b_pin;
    float cpr;

    float motor_polarity;
    float encoder_polarity;
    float pwm_limit;

    float ff_k_fwd;
    float ff_b_fwd;
    float ff_k_rev;
    float ff_b_rev;

    /* Original calibration values; ff_b_* above hold the effective values. */
    float ff_b_base_fwd;
    float ff_b_base_rev;

    float kp;
    float ki;
    float kd;
    float i_limit;

    float i_term;
    float previous_speed;
    int8_t integral_direction;

    float target_speed;
    float actual_speed;
    float output_pwm;

    int32_t encoder_count;
    int32_t previous_encoder_count;
    int8_t hw_direction;

    /* 50 ms encoder speed sliding window */
    int32_t speed_delta_history[MOTOR_ENCODER_SPEED_WINDOW_SAMPLES];
    int32_t speed_delta_sum;
    uint8_t speed_delta_index;
    uint8_t speed_delta_valid_samples;

    bool encoder_ready;

    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;

    float startup_pwm_fwd;
    float startup_pwm_rev;
    float run_pwm_fwd;
    float run_pwm_rev;

    bool startup_active;
    int8_t startup_direction;

    /* Adaptive breakaway boost and its smooth release after motion starts. */
    float startup_boost_pwm;
    bool startup_release_active;
    uint8_t startup_motion_cycles;

    /*
    * Consecutive cycles for which:
    *   target != 0
    *   startup is inactive
    *   actual wheel speed is near zero
    *
    * Used to re-arm startup boost after a wheel stalls/stops
    * during low-speed tracking.
    */
    uint8_t startup_stall_cycles;

    /* Consecutive control time with a meaningful command but no measured
     * movement. This is diagnostic state; the tracker decides whether it is
     * a fatal fault for the current trajectory. */
    uint32_t stall_cycles;
    bool stall_suspected;
} motor_wheel_t;

static motor_command_t g_motor_command = {0};
static motor_raw_pwm_command_t g_raw_pwm_command = {0};

static motor_wheel_t g_motor_A;
static motor_wheel_t g_motor_B;
static motor_wheel_t g_motor_D;

static motor_status_t g_motor_status = {0};

static bool g_motor_initialized = false;
static bool g_motor_enabled = false;
static bool g_closed_loop_ready = false;
static bool g_raw_debug_mode = false;
static bool g_emergency_stop = false;

static float g_motor_ff_b_scale = 1.0f;

static TaskHandle_t g_motor_task_handle = NULL;

static portMUX_TYPE g_motor_lock =
    portMUX_INITIALIZER_UNLOCKED;

static inline float motor_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float motor_effective_b(
    float calibrated_b,
    float scale,
    float pwm_limit)
{
    const float configured_max =
        (MOTOR_EFFECTIVE_B_MAX > 0.0f)
            ? MOTOR_EFFECTIVE_B_MAX
            : pwm_limit;

    /* Never let the safety cap invalidate the user's PWM_MIN request. */
    const float requested_max =
        (MOTOR_PWM_MIN > configured_max)
            ? pwm_limit
            : configured_max;

    const float effective_max =
        (requested_max < pwm_limit)
            ? requested_max
            : pwm_limit;

    return motor_clampf(
        calibrated_b * scale,
        0.0f,
        effective_max);
}

/*
 * Scale B globally so the weakest calibrated wheel/direction reaches the
 * configured environmental minimum.  Scaling is calculated once at init;
 * changing MOTOR_PWM_MIN therefore requires a normal firmware rebuild.
 */
static void motor_apply_environment_compensation(void)
{
    const float b_min = fminf(
        fminf(g_motor_A.ff_b_base_fwd, g_motor_A.ff_b_base_rev),
        fminf(
            fminf(g_motor_B.ff_b_base_fwd, g_motor_B.ff_b_base_rev),
            fminf(g_motor_D.ff_b_base_fwd, g_motor_D.ff_b_base_rev)));

    if ((MOTOR_PWM_MIN > 0.0f) && (b_min > 0.0f))
    {
        g_motor_ff_b_scale =
            fmaxf(1.0f, MOTOR_PWM_MIN / b_min);
    }
    else
    {
        g_motor_ff_b_scale = 1.0f;
    }

    g_motor_A.ff_b_fwd = motor_effective_b(
        g_motor_A.ff_b_base_fwd,
        g_motor_ff_b_scale,
        g_motor_A.pwm_limit);
    g_motor_A.ff_b_rev = motor_effective_b(
        g_motor_A.ff_b_base_rev,
        g_motor_ff_b_scale,
        g_motor_A.pwm_limit);
    g_motor_B.ff_b_fwd = motor_effective_b(
        g_motor_B.ff_b_base_fwd,
        g_motor_ff_b_scale,
        g_motor_B.pwm_limit);
    g_motor_B.ff_b_rev = motor_effective_b(
        g_motor_B.ff_b_base_rev,
        g_motor_ff_b_scale,
        g_motor_B.pwm_limit);
    g_motor_D.ff_b_fwd = motor_effective_b(
        g_motor_D.ff_b_base_fwd,
        g_motor_ff_b_scale,
        g_motor_D.pwm_limit);
    g_motor_D.ff_b_rev = motor_effective_b(
        g_motor_D.ff_b_base_rev,
        g_motor_ff_b_scale,
        g_motor_D.pwm_limit);

    /* The running floor now has exactly the same source as FF intercept B. */
    g_motor_A.run_pwm_fwd = g_motor_A.ff_b_fwd;
    g_motor_A.run_pwm_rev = g_motor_A.ff_b_rev;
    g_motor_B.run_pwm_fwd = g_motor_B.ff_b_fwd;
    g_motor_B.run_pwm_rev = g_motor_B.ff_b_rev;
    g_motor_D.run_pwm_fwd = g_motor_D.ff_b_fwd;
    g_motor_D.run_pwm_rev = g_motor_D.ff_b_rev;

    if ((MOTOR_PWM_MIN > 0.0f) && (MOTOR_EFFECTIVE_B_MAX > 0.0f) &&
        (g_motor_ff_b_scale * b_min > MOTOR_EFFECTIVE_B_MAX))
    {
        ESP_LOGW(
            MOTOR_TAG,
            "PWM_MIN %.1f requests B compensation above B_MAX %.1f; "
            "effective B values are clipped",
            (double)MOTOR_PWM_MIN,
            (double)MOTOR_EFFECTIVE_B_MAX);
    }

    ESP_LOGI(
        MOTOR_TAG,
        "B compensation: PWM_MIN=%.1f scale=%.3f "
        "effective=(A+ %.1f A- %.1f B+ %.1f B- %.1f D+ %.1f D- %.1f)",
        (double)MOTOR_PWM_MIN,
        (double)g_motor_ff_b_scale,
        (double)g_motor_A.ff_b_fwd,
        (double)g_motor_A.ff_b_rev,
        (double)g_motor_B.ff_b_fwd,
        (double)g_motor_B.ff_b_rev,
        (double)g_motor_D.ff_b_fwd,
        (double)g_motor_D.ff_b_rev);
}

static inline float motor_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline int8_t motor_signf(float x)
{
    if (x > 0.5f) return +1;
    if (x < -0.5f) return -1;
    return 0;
}

#if MOTOR_RUN_PWM_FLOOR_ENABLE || MOTOR_STARTUP_BOOST_ENABLE
static float motor_directional_pwm_floor(
    const motor_wheel_t *m,
    int8_t direction)
{
    if ((m == NULL) || (direction == 0))
    {
        return 0.0f;
    }

    return (direction > 0)
        ? m->run_pwm_fwd
        : m->run_pwm_rev;
}
#endif

static float motor_directional_startup_floor(
    const motor_wheel_t *m,
    int8_t direction)
{
    if ((m == NULL) || (direction == 0))
    {
        return 0.0f;
    }

    return (direction > 0)
        ? m->startup_pwm_fwd
        : m->startup_pwm_rev;
}

#if MOTOR_RUN_PWM_FLOOR_ENABLE || MOTOR_STARTUP_BOOST_ENABLE
static void motor_apply_directional_floor(
    float *output,
    int8_t direction,
    float floor_pwm)
{
    if ((output == NULL) || (direction == 0))
    {
        return;
    }

    if (direction > 0)
    {
        if (*output < floor_pwm)
        {
            *output = floor_pwm;
        }
    }
    else if (*output > -floor_pwm)
    {
        *output = -floor_pwm;
    }
}
#endif

static void motor_controller_reset(motor_wheel_t *m)
{
    if (m == NULL) return;

    m->i_term = 0.0f;
    m->previous_speed = m->actual_speed;
    m->integral_direction = 0;
    m->target_speed = 0.0f;
    m->output_pwm = 0.0f;
    
    m->startup_active = false;
    m->startup_direction = 0;
    m->startup_boost_pwm = 0.0f;
    m->startup_release_active = false;
    m->startup_motion_cycles = 0U;
    m->startup_stall_cycles = 0U;
    m->stall_cycles = 0U;
    m->stall_suspected = false;
}

static uint32_t motor_stall_detect_cycles(void)
{
    const uint32_t period_ms =
        (MOTOR_CONTROL_PERIOD_MS > 0U)
            ? MOTOR_CONTROL_PERIOD_MS
            : 1U;

    const uint32_t cycles =
        (MOTOR_STALL_DETECT_TIME_MS + period_ms - 1U) /
        period_ms;

    return (cycles > 0U) ? cycles : 1U;
}

static void motor_update_stall_diagnostic(
    motor_wheel_t *m,
    float target_speed)
{
    if (m == NULL)
    {
        return;
    }

    const int8_t target_direction =
        motor_signf(target_speed);

    const float actual_speed_in_target_direction =
        (float)target_direction * m->actual_speed;

    const bool meaningful_command =
        m->encoder_ready &&
        (target_direction != 0) &&
        (motor_absf(target_speed) >=
         MOTOR_STALL_DETECT_TARGET_SPEED_MM_S);

    const float startup_boost_cap =
        motor_clampf(
            MOTOR_STARTUP_BOOST_MAX_PWM,
            motor_directional_startup_floor(
                m,
                target_direction),
            m->pwm_limit);

    /* Do not report a fatal-looking stall while adaptive startup is still
     * ramping toward its allowed maximum. */
    const bool startup_grace =
        m->startup_active &&
        (m->startup_boost_pwm <
         (startup_boost_cap - 0.5f));

    const bool no_measured_motion =
        actual_speed_in_target_direction <
        MOTOR_STALL_DETECT_ACTUAL_SPEED_MM_S;

    if (!meaningful_command ||
        !no_measured_motion ||
        startup_grace)
    {
        m->stall_cycles = 0U;
        m->stall_suspected = false;
        return;
    }

    if (m->stall_cycles < UINT32_MAX)
    {
        m->stall_cycles++;
    }

    if (m->stall_cycles >=
        motor_stall_detect_cycles())
    {
        if (!m->stall_suspected)
        {
            ESP_LOGW(
                MOTOR_TAG,
                "Wheel stall suspected: target=%.1f actual=%.1f pwm=%.1f",
                (double)target_speed,
                (double)m->actual_speed,
                (double)m->output_pwm);
        }

        m->stall_suspected = true;
    }
}

static esp_err_t motor_pwm_channel_init(
    gpio_num_t gpio,
    ledc_channel_t channel)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };

    return ledc_channel_config(&cfg);
}

/*
 * Safe TB6612 write:
 * PWM -> 0 before changing IN1/IN2.
 * No delay, no reversal state machine.
 */
static void motor_write_pwm(
    motor_wheel_t *m,
    float logical_pwm)
{
    if (m == NULL) return;

    logical_pwm =
        motor_clampf(
            logical_pwm,
            -m->pwm_limit,
            m->pwm_limit);

    float physical_pwm =
        logical_pwm * m->motor_polarity;

    int8_t direction =
        motor_signf(physical_pwm);

    uint32_t duty =
        (uint32_t)(motor_absf(physical_pwm) + 0.5f);

    if (duty > MOTOR_PWM_HW_MAX)
        duty = MOTOR_PWM_HW_MAX;

    if (direction == 0)
        duty = 0;
    
    //换向保护
    //如果换向，先把pwm设为0
    if ((m->hw_direction != 0) &&
        (direction != 0) &&
        (m->hw_direction != direction))
    {
        ledc_set_duty(
            MOTOR_LEDC_MODE,
            m->pwm_channel,
            0);

        ledc_update_duty(
            MOTOR_LEDC_MODE,
            m->pwm_channel);
    }

    if (direction > 0)
    {
        gpio_set_level(m->in1_pin, 1);
        gpio_set_level(m->in2_pin, 0);
    }
    else if (direction < 0)
    {
        gpio_set_level(m->in1_pin, 0);
        gpio_set_level(m->in2_pin, 1);
    }
    else
    {
        gpio_set_level(m->in1_pin, 0);
        gpio_set_level(m->in2_pin, 0);
    }

    //换完方向，设置pwm
    ledc_set_duty(
        MOTOR_LEDC_MODE,
        m->pwm_channel,
        duty);

    ledc_update_duty(
        MOTOR_LEDC_MODE,
        m->pwm_channel);

    //更新状态
    m->hw_direction = direction;
    m->output_pwm = logical_pwm;
}

static void motor_write_all_zero(void)
{
    motor_write_pwm(&g_motor_A, 0.0f);
    motor_write_pwm(&g_motor_B, 0.0f);
    motor_write_pwm(&g_motor_D, 0.0f);
}


/* ============================================================
 * 14. Encoder / PCNT
 * ============================================================ */

static bool motor_encoder_configured(
    gpio_num_t enc_a,
    gpio_num_t enc_b,
    float cpr)
{
    return
        (enc_a != GPIO_NUM_NC) &&
        (enc_b != GPIO_NUM_NC) &&
        (cpr > 0.0f);
}

static esp_err_t motor_encoder_init(motor_wheel_t *m)
{
    if (m == NULL)
        return ESP_ERR_INVALID_ARG;

    //未填写参数，不准动！
    if (!motor_encoder_configured(
            m->enc_a_pin,
            m->enc_b_pin,
            m->cpr))
    {
        m->encoder_ready = false;
        return ESP_OK;
    }

    pcnt_unit_config_t unit_cfg = {
        .high_limit = MOTOR_PCNT_HIGH_LIMIT,
        .low_limit = MOTOR_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };

    ESP_RETURN_ON_ERROR(
        pcnt_new_unit(&unit_cfg, &m->pcnt_unit),
        MOTOR_TAG,
        "pcnt_new_unit failed");

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = MOTOR_PCNT_GLITCH_NS,
    };

    ESP_RETURN_ON_ERROR(
        pcnt_unit_set_glitch_filter(
            m->pcnt_unit,
            &filter_cfg),
        MOTOR_TAG,
        "pcnt glitch filter failed");

    pcnt_chan_config_t a_cfg = {
        .edge_gpio_num = m->enc_a_pin,
        .level_gpio_num = m->enc_b_pin,
    };

    pcnt_chan_config_t b_cfg = {
        .edge_gpio_num = m->enc_b_pin,
        .level_gpio_num = m->enc_a_pin,
    };

    ESP_RETURN_ON_ERROR(
        pcnt_new_channel(
            m->pcnt_unit,
            &a_cfg,
            &m->pcnt_chan_a),
        MOTOR_TAG,
        "pcnt channel A failed");

    ESP_RETURN_ON_ERROR(
        pcnt_new_channel(
            m->pcnt_unit,
            &b_cfg,
            &m->pcnt_chan_b),
        MOTOR_TAG,
        "pcnt channel B failed");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            m->pcnt_chan_a,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        MOTOR_TAG,
        "pcnt A edge action failed");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            m->pcnt_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        MOTOR_TAG,
        "pcnt A level action failed");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            m->pcnt_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        MOTOR_TAG,
        "pcnt B edge action failed");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            m->pcnt_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        MOTOR_TAG,
        "pcnt B level action failed");

    ESP_RETURN_ON_ERROR(
        pcnt_unit_add_watch_point(
            m->pcnt_unit,
            MOTOR_PCNT_HIGH_LIMIT),
        MOTOR_TAG,
        "pcnt high watch point failed");

    ESP_RETURN_ON_ERROR(
        pcnt_unit_add_watch_point(
            m->pcnt_unit,
            MOTOR_PCNT_LOW_LIMIT),
        MOTOR_TAG,
        "pcnt low watch point failed");

    ESP_RETURN_ON_ERROR(
        pcnt_unit_enable(m->pcnt_unit),
        MOTOR_TAG,
        "pcnt enable failed");

    ESP_RETURN_ON_ERROR(
        pcnt_unit_clear_count(m->pcnt_unit),
        MOTOR_TAG,
        "pcnt clear failed");

    ESP_RETURN_ON_ERROR(
        pcnt_unit_start(m->pcnt_unit),
        MOTOR_TAG,
        "pcnt start failed");

    /* Reset encoder speed sliding window */
    for (uint32_t i = 0;
        i < MOTOR_ENCODER_SPEED_WINDOW_SAMPLES;
        ++i)
    {
        m->speed_delta_history[i] = 0;
    }

    m->speed_delta_sum = 0;
    m->speed_delta_index = 0;
    m->speed_delta_valid_samples = 0;

    m->encoder_count = 0;
    m->previous_encoder_count = 0;
    m->actual_speed = 0.0f;
    m->encoder_ready = true;


    return ESP_OK;
}

/*
* 编码器调用
* 根据编码器输出更新当前速度
*/
/*
 * Encoder speed update
 *
 * Control loop still runs at 100 Hz / 10 ms.
 * Speed is estimated from a sliding encoder-count window.
 *
 * Default:
 *   5 samples x 10 ms = 50 ms speed measurement window.
 */
static void motor_encoder_update_speed(motor_wheel_t *m)
{
    if ((m == NULL) || !m->encoder_ready)
        return;

    int raw_count = 0;

    if (pcnt_unit_get_count(
            m->pcnt_unit,
            &raw_count) != ESP_OK)
    {
        return;
    }

    /*
     * Apply encoder logical polarity first.
     */
    const int32_t logical_count =
        (int32_t)(
            (float)raw_count *
            m->encoder_polarity);

    /*
     * Encoder increment during the latest 10 ms control period.
     */
    const int32_t delta =
        logical_count -
        m->previous_encoder_count;

    m->previous_encoder_count =
        logical_count;

    m->encoder_count =
        logical_count;

    /*
     * Update sliding-window delta sum.
     *
     * Remove the oldest delta from the sum,
     * insert the newest delta,
     * then add it to the sum.
     */
    m->speed_delta_sum -=
        m->speed_delta_history[
            m->speed_delta_index];

    m->speed_delta_history[
        m->speed_delta_index] = delta;

    m->speed_delta_sum += delta;

    /*
     * Advance circular-buffer index.
     */
    m->speed_delta_index++;

    if (m->speed_delta_index >=
        MOTOR_ENCODER_SPEED_WINDOW_SAMPLES)
    {
        m->speed_delta_index = 0;
    }

    /*
     * During startup the window is not full yet.
     *
     * Use 10ms, 20ms, 30ms... until the complete
     * 50ms window is available.
     */
    if (m->speed_delta_valid_samples <
        MOTOR_ENCODER_SPEED_WINDOW_SAMPLES)
    {
        m->speed_delta_valid_samples++;
    }

    /*
     * Configuration sanity check.
     */
    if ((m->cpr <= 0.0f) ||
        (MOTOR_WHEEL_DIAMETER_MM <= 0.0f))
    {
        m->actual_speed = 0.0f;
        return;
    }

    /*
     * Wheel circumference in mm.
     */
    const float circumference =
        MOTOR_PI *
        MOTOR_WHEEL_DIAMETER_MM;

    /*
     * Effective measurement window.
     *
     * Full window:
     *   5 * 0.01 = 0.05 s
     */
    const float window_dt =
        MOTOR_CONTROL_DT *
        (float)m->speed_delta_valid_samples;

    /*
     * Average wheel speed over the sliding window.
     */
    const float raw_speed =
        ((float)m->speed_delta_sum / m->cpr) *
        circumference /
        window_dt;

    /*
     * Optional additional first-order filtering.
     */
    const float alpha =
        motor_clampf(
            MOTOR_ENCODER_FILTER_ALPHA,
            0.0f,
            1.0f);

    m->actual_speed +=
        alpha *
        (raw_speed -
         m->actual_speed);
}


/* 
 * Inverse kinematics
 */
static void motor_inverse_kinematics(
    float vx,
    float vy,
    float w,
    float *a,
    float *b,
    float *d)
{
    const float L =
        MOTOR_CHASSIS_RADIUS_MM;

    float va =
        MOTOR_A_KX * vx +
        MOTOR_A_KY * vy +
        MOTOR_A_KW * L * w;

    float vb =
        MOTOR_B_KX * vx +
        MOTOR_B_KY * vy +
        MOTOR_B_KW * L * w;

    float vd =
        MOTOR_D_KX * vx +
        MOTOR_D_KY * vy +
        MOTOR_D_KW * L * w;

    //按最大速度来限制速度上限
    if (MOTOR_WHEEL_SPEED_LIMIT_MM_S > 0.0f)
    {
        float max_abs = motor_absf(va);

        if (motor_absf(vb) > max_abs)
            max_abs = motor_absf(vb);

        if (motor_absf(vd) > max_abs)
            max_abs = motor_absf(vd);

        if (max_abs > MOTOR_WHEEL_SPEED_LIMIT_MM_S)
        {
            const float scale =
                MOTOR_WHEEL_SPEED_LIMIT_MM_S /
                max_abs;

            va *= scale;
            vb *= scale;
            vd *= scale;
        }
    }

    //如果太小直接当作0处理
    if (motor_absf(va) < MOTOR_TARGET_ZERO_MM_S)
        va = 0.0f;

    if (motor_absf(vb) < MOTOR_TARGET_ZERO_MM_S)
        vb = 0.0f;

    if (motor_absf(vd) < MOTOR_TARGET_ZERO_MM_S)
        vd = 0.0f;

    if (a != NULL) *a = va;
    if (b != NULL) *b = vb;
    if (d != NULL) *d = vd;
}

/*
 * Feedforward + PID
 * 由逆动力学解出的值生成对应目标转速
 * 做线性映射，用b值去掉deadband
*/
static float motor_feedforward(
    const motor_wheel_t *m,
    float target_speed)
{
    if ((m == NULL) ||
        (motor_absf(target_speed) <
         MOTOR_TARGET_ZERO_MM_S))
    {
        return 0.0f;
    }

    const float speed =
        motor_absf(target_speed);

    if (target_speed > 0.0f)
    {
        return
            m->ff_k_fwd * speed +
            m->ff_b_fwd;
    }

    return -(
        m->ff_k_rev * speed +
        m->ff_b_rev);
}

static float motor_speed_control_step(
    motor_wheel_t *m,
    float target_speed)
{
    if (m == NULL)
        return 0.0f;

    if (motor_absf(target_speed) <
        MOTOR_TARGET_ZERO_MM_S)
    {
        target_speed = 0.0f;
    }

    m->target_speed =
        target_speed;


    const int8_t target_direction =
        (target_speed > 0.0f) ? +1 :
        (target_speed < 0.0f) ? -1 : 0;

    /*
     * Integral compensation represents the load in the current direction.
     * It must not survive a stop or reversal, otherwise the stored PWM can
     * produce a kick in the next command.
     */
    if (target_direction == 0)
    {
        m->i_term = 0.0f;
        m->integral_direction = 0;
    }
    else if (target_direction != m->integral_direction)
    {
        m->i_term = 0.0f;
        m->integral_direction = target_direction;
        m->previous_speed = m->actual_speed;
    }

 #if MOTOR_STARTUP_BOOST_ENABLE
    /*
    * Startup state management.
    *
    * Startup boost is armed when:
    *
    *   1. A new non-zero motion command begins;
    *   2. Wheel direction reverses;
    *   3. The target is still non-zero, but the wheel has
    *      actually stopped for several consecutive cycles.
    *
    * Case 3 is important for low-speed trajectory settling:
    *
    *      moving normally
    *          ->
    *      target speed becomes small
    *          ->
    *      wheel stops because of static friction
    *          ->
    *      tracker still requests motion
    *          ->
    *      re-arm startup boost
    */
    if (target_direction == 0)
    {
        /*
        * True zero command.
        *
        * Forget the previous direction so that the next
        * non-zero command always starts with startup boost.
        */
        m->startup_active = false;
        m->startup_direction = 0;
        m->startup_boost_pwm = 0.0f;
        m->startup_release_active = false;
        m->startup_motion_cycles = 0U;
        m->startup_stall_cycles = 0U;
    }
    else if (target_direction != m->startup_direction)
    {
        /*
        * New motion command or direction reversal.
        */
        m->startup_direction = target_direction;
        m->startup_active = true;
        m->startup_boost_pwm =
            motor_directional_startup_floor(
                m,
                target_direction);
        m->startup_release_active = false;
        m->startup_motion_cycles = 0U;
        m->startup_stall_cycles = 0U;
    }
    else if (!m->startup_active)
    {
        /*
        * Same commanded direction.
        *
        * If normal FF + PID can no longer keep the wheel moving
        * and measured speed remains approximately zero, re-arm
        * startup boost.
        */
        if (motor_absf(m->actual_speed) <=
            MOTOR_ACTUAL_ZERO_MM_S)
        {
            if (m->startup_stall_cycles <
                MOTOR_STARTUP_REARM_CYCLES)
            {
                m->startup_stall_cycles++;
            }

            if (m->startup_stall_cycles >=
                MOTOR_STARTUP_REARM_CYCLES)
            {
                m->startup_active = true;
                if (m->startup_boost_pwm <
                    motor_directional_startup_floor(
                        m,
                        target_direction))
                {
                    m->startup_boost_pwm =
                        motor_directional_startup_floor(
                            m,
                            target_direction);
                }
                m->startup_release_active = false;
                m->startup_motion_cycles = 0U;
                m->startup_stall_cycles = 0U;
            }
        }
        else
        {
            /*
            * Wheel is still moving normally.
            */
            m->startup_stall_cycles = 0U;
        }
    }
    else
    {
        /*
        * Startup boost is already active.
        */
        m->startup_stall_cycles = 0U;
    }

    /* The legacy startup state machine is compile-time optional. */
 #endif

    //核心反馈值：error
    const float error =
        target_speed -
        m->actual_speed;

    const float p_term =
        m->kp * error;

    const float d_term =
        -m->kd *
        (m->actual_speed -
         m->previous_speed) /
        MOTOR_CONTROL_DT;

    const float ff =
        motor_feedforward(
            m,
            target_speed);

    float i_gain_multiplier = 1.0f;

    if ((m->i_term * error) < 0.0f)
    {
        i_gain_multiplier =
            MOTOR_PID_I_UNWIND_MULTIPLIER;
    }

    float candidate_i = m->i_term;

    if ((target_direction != 0) && m->encoder_ready)
    {
        candidate_i +=
            m->ki *
            i_gain_multiplier *
            error *
            MOTOR_CONTROL_DT;
    }
    else
    {
        candidate_i = 0.0f;
    }

    candidate_i =
        motor_clampf(
            candidate_i,
            -m->i_limit,
            m->i_limit);

    float unsat =
        ff +
        p_term +
        candidate_i +
        d_term;

    float output =
        motor_clampf(
            unsat,
            -m->pwm_limit,
            m->pwm_limit);

    /*
     * Conditional integration anti-windup.
     */
    const bool not_saturated =
        (motor_absf(unsat - output) < 0.01f);

    const bool unwind_high =
        (output >= m->pwm_limit) &&
        (error < 0.0f);

    const bool unwind_low =
        (output <= -m->pwm_limit) &&
        (error > 0.0f);
  
    //防止积分饱和
    if (not_saturated ||
        unwind_high ||
        unwind_low)
    {
        m->i_term =
            candidate_i;
    }

    unsat =
        ff +
        p_term +
        m->i_term +
        d_term;

    /* PI may reduce the command below calibrated B when a wheel over-speeds. */
    output =
        motor_clampf(
            unsat,
            -m->pwm_limit,
            m->pwm_limit);

#if MOTOR_RUN_PWM_FLOOR_ENABLE
    if (target_direction != 0)
    {
        motor_apply_directional_floor(
            &output,
            target_direction,
            motor_directional_pwm_floor(
                m,
                target_direction));
    }
#endif

 #if MOTOR_STARTUP_BOOST_ENABLE
    /*
    * Adaptive startup / stall-recovery boost.
    *
    * The configured START_PWM is the initial floor.  If the wheel remains
    * still under real load, increase the floor gradually until movement is
    * detected.  This compensates for battery, surface, and chassis-load
    * differences without changing the public velocity API.
    *
    * A few consecutive motion samples are required before releasing the
    * boost.  Release is also gradual, avoiding the old start/stop pulse.
    *
    * Also check speed in the TARGET direction instead of using
    * abs(actual_speed). This is important during reversals.
    */
    if (m->startup_active)
    {
        float startup_exit_speed =
            motor_absf(target_speed);

        /*
        * Large commands:
        *     exit startup at MOTOR_STARTUP_EXIT_SPEED_MM_S.
        *
        * Small commands:
        *     exit when approximately reaching target speed.
        *
        * Never use an exit threshold below the actual-zero zone.
        */
        startup_exit_speed =
            motor_clampf(
                startup_exit_speed,
                MOTOR_ACTUAL_ZERO_MM_S,
                MOTOR_STARTUP_EXIT_SPEED_MM_S);

        /*
        * Positive only when actual wheel motion is in the
        * currently commanded direction.
        */
        const float actual_speed_in_target_direction =
            (float)target_direction *
            m->actual_speed;

        if ((target_direction != 0) &&
            (actual_speed_in_target_direction >=
            startup_exit_speed))
        {
            if (m->startup_motion_cycles <
                MOTOR_STARTUP_CONFIRM_CYCLES)
            {
                m->startup_motion_cycles++;
            }

            if (m->startup_motion_cycles >=
                MOTOR_STARTUP_CONFIRM_CYCLES)
            {
                /* Wheel has started; begin a smooth boost release. */
                m->startup_active = false;
                m->startup_release_active = true;
                m->startup_stall_cycles = 0U;
            }
        }
        else
        {
            m->startup_motion_cycles = 0U;

            const float boost_max =
                motor_clampf(
                    MOTOR_STARTUP_BOOST_MAX_PWM,
                    motor_directional_startup_floor(
                        m,
                        target_direction),
                    m->pwm_limit);

            m->startup_boost_pwm +=
                MOTOR_STARTUP_BOOST_RAMP_PWM_PER_S *
                MOTOR_CONTROL_DT;

            m->startup_boost_pwm =
                motor_clampf(
                    m->startup_boost_pwm,
                    motor_directional_startup_floor(
                        m,
                        target_direction),
                    boost_max);

            motor_apply_directional_floor(
                &output,
                target_direction,
                m->startup_boost_pwm);
        }
    }
    else if (m->startup_release_active &&
             (target_direction != 0))
    {
        const float run_floor =
            motor_directional_pwm_floor(
                m,
                target_direction);

        float release_target =
            motor_absf(output);

        if (release_target < run_floor)
        {
            release_target = run_floor;
        }

        const float release_step =
            MOTOR_STARTUP_BOOST_RELEASE_PWM_PER_S *
            MOTOR_CONTROL_DT;

        if (m->startup_boost_pwm >
            release_target)
        {
            m->startup_boost_pwm -= release_step;

            if (m->startup_boost_pwm <= release_target)
            {
                m->startup_boost_pwm = release_target;
                m->startup_release_active = false;
            }
        }
        else
        {
            m->startup_boost_pwm = release_target;
            m->startup_release_active = false;
        }

        motor_apply_directional_floor(
            &output,
            target_direction,
            m->startup_boost_pwm);
    }
    else if (target_direction != 0)
    {
        /* Normal running: retain the calibrated continuous-motion floor. */
        motor_apply_directional_floor(
            &output,
            target_direction,
            motor_directional_pwm_floor(
                m,
                target_direction));
    }
 #endif

    if (target_direction == 0)
    {
        m->startup_release_active = false;
        m->startup_motion_cycles = 0U;
        m->startup_boost_pwm = 0.0f;
    }

    output =
        motor_clampf(
            output,
            -m->pwm_limit,
            m->pwm_limit);

    /* Do not let a large correction actively reverse a commanded wheel. */
    if ((target_direction > 0) && (output < 0.0f))
    {
        output = 0.0f;
    }
    else if ((target_direction < 0) && (output > 0.0f))
    {
        output = 0.0f;
    }

    m->previous_speed =
        m->actual_speed;

    return output;
}

/* ============================================================
 * 17. Status
 * ============================================================ */

static void motor_publish_status(
    const motor_command_t *cmd)
{
    motor_status_t s = {0};

    s.initialized = g_motor_initialized;
    s.enabled = g_motor_enabled;
    s.closed_loop_ready = g_closed_loop_ready;
    s.raw_debug_mode = g_raw_debug_mode;
    s.emergency_stop = g_emergency_stop;

    if (cmd != NULL)
    {
        s.target_vx_mm_s = cmd->vx;
        s.target_vy_mm_s = cmd->vy;
        s.target_w_rad_s = cmd->w;
    }

    s.A.target_speed_mm_s = g_motor_A.target_speed;
    s.A.actual_speed_mm_s = g_motor_A.actual_speed;
    s.A.encoder_count = g_motor_A.encoder_count;
    s.A.output_pwm = g_motor_A.output_pwm;
    s.A.encoder_ready = g_motor_A.encoder_ready;
    s.A.startup_active = g_motor_A.startup_active;
    s.A.stall_suspected = g_motor_A.stall_suspected;
    s.A.stall_elapsed_ms =
        g_motor_A.stall_cycles * MOTOR_CONTROL_PERIOD_MS;

    s.B.target_speed_mm_s = g_motor_B.target_speed;
    s.B.actual_speed_mm_s = g_motor_B.actual_speed;
    s.B.encoder_count = g_motor_B.encoder_count;
    s.B.output_pwm = g_motor_B.output_pwm;
    s.B.encoder_ready = g_motor_B.encoder_ready;
    s.B.startup_active = g_motor_B.startup_active;
    s.B.stall_suspected = g_motor_B.stall_suspected;
    s.B.stall_elapsed_ms =
        g_motor_B.stall_cycles * MOTOR_CONTROL_PERIOD_MS;

    s.D.target_speed_mm_s = g_motor_D.target_speed;
    s.D.actual_speed_mm_s = g_motor_D.actual_speed;
    s.D.encoder_count = g_motor_D.encoder_count;
    s.D.output_pwm = g_motor_D.output_pwm;
    s.D.encoder_ready = g_motor_D.encoder_ready;
    s.D.startup_active = g_motor_D.startup_active;
    s.D.stall_suspected = g_motor_D.stall_suspected;
    s.D.stall_elapsed_ms =
        g_motor_D.stall_cycles * MOTOR_CONTROL_PERIOD_MS;

    taskENTER_CRITICAL(&g_motor_lock);
    g_motor_status = s;
    taskEXIT_CRITICAL(&g_motor_lock);
}

/* ============================================================
 * 18. 100 Hz motor task
 * ============================================================ */

static void motor_control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake =
        xTaskGetTickCount();

    const TickType_t period =
        pdMS_TO_TICKS(
            MOTOR_CONTROL_PERIOD_MS);

    while (1)
    {
        motor_command_t cmd;
        motor_raw_pwm_command_t raw;

        bool enabled;
        bool raw_mode;
        bool emergency;

        taskENTER_CRITICAL(&g_motor_lock);

        cmd = g_motor_command;
        raw = g_raw_pwm_command;
        enabled = g_motor_enabled;
        raw_mode = g_raw_debug_mode;
        emergency = g_emergency_stop;

        taskEXIT_CRITICAL(&g_motor_lock);

        /*
         * Encoder measurement runs in both normal and raw modes.
         */
        motor_encoder_update_speed(&g_motor_A);
        motor_encoder_update_speed(&g_motor_B);
        motor_encoder_update_speed(&g_motor_D);

        if ((!enabled) || emergency)
        {
            motor_write_all_zero();
            motor_publish_status(&cmd);

            vTaskDelayUntil(
                &last_wake,
                period);
            continue;
        }

        /*
         * Raw PWM calibration mode.
         */
        if (raw_mode)
        {
            g_motor_A.target_speed = 0.0f;
            g_motor_B.target_speed = 0.0f;
            g_motor_D.target_speed = 0.0f;

            motor_write_pwm(&g_motor_A, raw.A);
            motor_write_pwm(&g_motor_B, raw.B);
            motor_write_pwm(&g_motor_D, raw.D);

            motor_publish_status(&cmd);

            vTaskDelayUntil(
                &last_wake,
                period);
            continue;
        }

        /*
         * Normal chassis command is disabled until all required
         * encoder and geometry parameters are filled.
         */
        if (!g_closed_loop_ready)
        {
            motor_write_all_zero();
            motor_publish_status(&cmd);

            vTaskDelayUntil(
                &last_wake,
                period);
            continue;
        }

        float va;
        float vb;
        float vd;

        motor_inverse_kinematics(
            cmd.vx,
            cmd.vy,
            cmd.w,
            &va,
            &vb,
            &vd);

        const float pwm_a =
            motor_speed_control_step(
                &g_motor_A,
                va);

        const float pwm_b =
            motor_speed_control_step(
                &g_motor_B,
                vb);

        const float pwm_d =
            motor_speed_control_step(
                &g_motor_D,
                vd);

        motor_write_pwm(
            &g_motor_A,
            pwm_a);

        motor_write_pwm(
            &g_motor_B,
            pwm_b);

        motor_write_pwm(
            &g_motor_D,
            pwm_d);

        motor_update_stall_diagnostic(
            &g_motor_A,
            va);
        motor_update_stall_diagnostic(
            &g_motor_B,
            vb);
        motor_update_stall_diagnostic(
            &g_motor_D,
            vd);

        motor_publish_status(&cmd);

        vTaskDelayUntil(
            &last_wake,
            period);
    }
}

/* ============================================================
 * 19. Public API
 * ============================================================ */

 /**
 * @brief 标准化对外接口
 */
void Set_motor(
    float vx,
    float vy,
    float w)
{
    if (!g_motor_initialized)
        return;

    vx = motor_clampf(vx, -1.0f, 1.0f);
    vy = motor_clampf(vy, -1.0f, 1.0f);
    w  = motor_clampf(w,  -1.0f, 1.0f);

    const float magnitude =
        sqrtf(vx * vx + vy * vy);

    if (magnitude > 1.0f)
    {
        vx /= magnitude;
        vy /= magnitude;
    }

    const float vx_mm_s =
        vx * MOTOR_MAX_VX_MM_S;

    const float vy_mm_s =
        vy * MOTOR_MAX_VY_MM_S;

    const float w_rad_s =
        w * MOTOR_MAX_W_RAD_S;

    taskENTER_CRITICAL(&g_motor_lock);

    g_motor_command.vx = vx_mm_s;
    g_motor_command.vy = vy_mm_s;
    g_motor_command.w = w_rad_s;

    g_raw_debug_mode = false;

    taskEXIT_CRITICAL(&g_motor_lock);
}

/**
 * @brief 量纲单位对外接口
 */
void motor_set_velocity(
    float vx_mm_s,
    float vy_mm_s,
    float w_rad_s)
{
    if (!g_motor_initialized)
        return;

    vx_mm_s = motor_clampf(vx_mm_s, -MOTOR_MAX_VX_MM_S, MOTOR_MAX_VX_MM_S);
    vy_mm_s = motor_clampf(vy_mm_s, -MOTOR_MAX_VY_MM_S, MOTOR_MAX_VY_MM_S);
    w_rad_s  = motor_clampf(w_rad_s,  -MOTOR_MAX_W_RAD_S, MOTOR_MAX_W_RAD_S);

    taskENTER_CRITICAL(&g_motor_lock);

    g_motor_command.vx = vx_mm_s;
    g_motor_command.vy = vy_mm_s;
    g_motor_command.w = w_rad_s;

    g_raw_debug_mode = false;

    taskEXIT_CRITICAL(&g_motor_lock);
}

void motor_get_wheel_rps(
    float *rps_a,
    float *rps_b,
    float *rps_d)
{
    if (!g_motor_initialized)
    {
        if (rps_a) *rps_a = 0.0f;
        if (rps_b) *rps_b = 0.0f;
        if (rps_d) *rps_d = 0.0f;
        return;
    }

    taskENTER_CRITICAL(&g_motor_lock);

    const float circumference = MOTOR_PI * MOTOR_WHEEL_DIAMETER_MM;
    const float SPEED_DEADBAND = 0.1f;

    if (rps_a)
    {
        *rps_a = (motor_absf(g_motor_A.actual_speed) < SPEED_DEADBAND) 
                 ? 0.0f 
                 : g_motor_A.actual_speed / circumference;
    }

    if (rps_b)
    {
        *rps_b = (motor_absf(g_motor_B.actual_speed) < SPEED_DEADBAND) 
                 ? 0.0f 
                 : g_motor_B.actual_speed / circumference;
    }

    if (rps_d)
    {
        *rps_d = (motor_absf(g_motor_D.actual_speed) < SPEED_DEADBAND) 
                 ? 0.0f 
                 : g_motor_D.actual_speed / circumference;
    }

    taskEXIT_CRITICAL(&g_motor_lock);
}

void motor_stop(void)
{
    Set_motor(
        0.0f,
        0.0f,
        0.0f);
}

void motor_emergency_stop(void)
{
    if (!g_motor_initialized)
        return;

    taskENTER_CRITICAL(&g_motor_lock);

    g_motor_command =
        (motor_command_t){0};

    g_raw_pwm_command =
        (motor_raw_pwm_command_t){0};

    g_raw_debug_mode = false;
    g_emergency_stop = true;

    taskEXIT_CRITICAL(&g_motor_lock);

    motor_controller_reset(&g_motor_A);
    motor_controller_reset(&g_motor_B);
    motor_controller_reset(&g_motor_D);

    motor_write_all_zero();
}

void motor_control_enable(bool enable)
{
    if (!g_motor_initialized)
        return;

    if (!enable)
    {
        motor_emergency_stop();

        taskENTER_CRITICAL(&g_motor_lock);
        g_motor_enabled = false;
        taskEXIT_CRITICAL(&g_motor_lock);

        gpio_set_level(
            MOTOR_STBY_PIN,
            0);
        return;
    }

    motor_controller_reset(&g_motor_A);
    motor_controller_reset(&g_motor_B);
    motor_controller_reset(&g_motor_D);

    taskENTER_CRITICAL(&g_motor_lock);

    g_motor_command =
        (motor_command_t){0};

    g_raw_pwm_command =
        (motor_raw_pwm_command_t){0};

    g_raw_debug_mode = false;
    g_emergency_stop = false;
    g_motor_enabled = true;

    taskEXIT_CRITICAL(&g_motor_lock);

    gpio_set_level(
        MOTOR_STBY_PIN,
        1);
}

bool motor_control_is_ready(void)
{
    return g_closed_loop_ready;
}

bool motor_is_emergency_stopped(void)
{
    bool emergency_stop = false;

    taskENTER_CRITICAL(&g_motor_lock);
    emergency_stop = g_emergency_stop;
    taskEXIT_CRITICAL(&g_motor_lock);

    return emergency_stop;
}

void motor_get_status(
    motor_status_t *status)
{
    if (status == NULL)
        return;

    taskENTER_CRITICAL(&g_motor_lock);
    *status = g_motor_status;
    taskEXIT_CRITICAL(&g_motor_lock);
}

void motor_debug_set_raw_pwm(
    float pwm_a,
    float pwm_b,
    float pwm_d)
{
    if (!g_motor_initialized)
        return;

    taskENTER_CRITICAL(&g_motor_lock);

    g_raw_pwm_command.A =
        motor_clampf(
            pwm_a,
            -MOTOR_A_PWM_LIMIT,
            MOTOR_A_PWM_LIMIT);

    g_raw_pwm_command.B =
        motor_clampf(
            pwm_b,
            -MOTOR_B_PWM_LIMIT,
            MOTOR_B_PWM_LIMIT);

    g_raw_pwm_command.D =
        motor_clampf(
            pwm_d,
            -MOTOR_D_PWM_LIMIT,
            MOTOR_D_PWM_LIMIT);

    g_raw_debug_mode = true;

    taskEXIT_CRITICAL(&g_motor_lock);

    motor_controller_reset(&g_motor_A);
    motor_controller_reset(&g_motor_B);
    motor_controller_reset(&g_motor_D);
}

/* ============================================================
 * 20. Initialization
 * ============================================================ */

static void motor_load_configuration(void)
{
    g_motor_A = (motor_wheel_t){
        .in1_pin = MOTOR_A_IN1,
        .in2_pin = MOTOR_A_IN2,
        .pwm_pin = MOTOR_A_PWM,
        .pwm_channel = MOTOR_LEDC_CH_A,
        .enc_a_pin = MOTOR_A_ENC_A,
        .enc_b_pin = MOTOR_A_ENC_B,
        .cpr = MOTOR_A_ENCODER_CPR,
        .motor_polarity = MOTOR_A_POLARITY,
        .encoder_polarity = MOTOR_A_ENCODER_POLARITY,
        .pwm_limit = MOTOR_A_PWM_LIMIT,
        .ff_k_fwd = MOTOR_A_FF_K_FWD,
        .ff_b_fwd = MOTOR_A_FF_B_FWD,
        .ff_k_rev = MOTOR_A_FF_K_REV,
        .ff_b_rev = MOTOR_A_FF_B_REV,
        .ff_b_base_fwd = MOTOR_A_FF_B_FWD,
        .ff_b_base_rev = MOTOR_A_FF_B_REV,
        .kp = MOTOR_A_PID_KP,
        .ki = MOTOR_A_PID_KI,
        .kd = MOTOR_A_PID_KD,
        .i_limit = MOTOR_A_PID_I_LIMIT,
        .startup_pwm_fwd = MOTOR_A_START_PWM_FWD,
        .startup_pwm_rev = MOTOR_A_START_PWM_REV,
        .run_pwm_fwd = MOTOR_A_RUN_PWM_FWD,
        .run_pwm_rev = MOTOR_A_RUN_PWM_REV,
    };

    g_motor_B = (motor_wheel_t){
        .in1_pin = MOTOR_B_IN1,
        .in2_pin = MOTOR_B_IN2,
        .pwm_pin = MOTOR_B_PWM,
        .pwm_channel = MOTOR_LEDC_CH_B,
        .enc_a_pin = MOTOR_B_ENC_A,
        .enc_b_pin = MOTOR_B_ENC_B,
        .cpr = MOTOR_B_ENCODER_CPR,
        .motor_polarity = MOTOR_B_POLARITY,
        .encoder_polarity = MOTOR_B_ENCODER_POLARITY,
        .pwm_limit = MOTOR_B_PWM_LIMIT,
        .ff_k_fwd = MOTOR_B_FF_K_FWD,
        .ff_b_fwd = MOTOR_B_FF_B_FWD,
        .ff_k_rev = MOTOR_B_FF_K_REV,
        .ff_b_rev = MOTOR_B_FF_B_REV,
        .ff_b_base_fwd = MOTOR_B_FF_B_FWD,
        .ff_b_base_rev = MOTOR_B_FF_B_REV,
        .kp = MOTOR_B_PID_KP,
        .ki = MOTOR_B_PID_KI,
        .kd = MOTOR_B_PID_KD,
        .i_limit = MOTOR_B_PID_I_LIMIT,
        .startup_pwm_fwd = MOTOR_B_START_PWM_FWD,
        .startup_pwm_rev = MOTOR_B_START_PWM_REV,
        .run_pwm_fwd = MOTOR_B_RUN_PWM_FWD,
        .run_pwm_rev = MOTOR_B_RUN_PWM_REV,
    };

    g_motor_D = (motor_wheel_t){
        .in1_pin = MOTOR_D_IN1,
        .in2_pin = MOTOR_D_IN2,
        .pwm_pin = MOTOR_D_PWM,
        .pwm_channel = MOTOR_LEDC_CH_D,
        .enc_a_pin = MOTOR_D_ENC_A,
        .enc_b_pin = MOTOR_D_ENC_B,
        .cpr = MOTOR_D_ENCODER_CPR,
        .motor_polarity = MOTOR_D_POLARITY,
        .encoder_polarity = MOTOR_D_ENCODER_POLARITY,
        .pwm_limit = MOTOR_D_PWM_LIMIT,
        .ff_k_fwd = MOTOR_D_FF_K_FWD,
        .ff_b_fwd = MOTOR_D_FF_B_FWD,
        .ff_k_rev = MOTOR_D_FF_K_REV,
        .ff_b_rev = MOTOR_D_FF_B_REV,
        .ff_b_base_fwd = MOTOR_D_FF_B_FWD,
        .ff_b_base_rev = MOTOR_D_FF_B_REV,
        .kp = MOTOR_D_PID_KP,
        .ki = MOTOR_D_PID_KI,
        .kd = MOTOR_D_PID_KD,
        .i_limit = MOTOR_D_PID_I_LIMIT,
        .startup_pwm_fwd = MOTOR_D_START_PWM_FWD,
        .startup_pwm_rev = MOTOR_D_START_PWM_REV,
        .run_pwm_fwd = MOTOR_D_RUN_PWM_FWD,
        .run_pwm_rev = MOTOR_D_RUN_PWM_REV,
    };

    motor_apply_environment_compensation();
}

esp_err_t motor_control_init(void)
{
    if (g_motor_initialized)
        return ESP_OK;

    motor_load_configuration();

    gpio_config_t io = {
        .pin_bit_mask =
            (1ULL << MOTOR_STBY_PIN) |
            (1ULL << MOTOR_A_IN1) |
            (1ULL << MOTOR_A_IN2) |
            (1ULL << MOTOR_B_IN1) |
            (1ULL << MOTOR_B_IN2) |
            (1ULL << MOTOR_D_IN1) |
            (1ULL << MOTOR_D_IN2),

        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&io),
        MOTOR_TAG,
        "GPIO init failed");

    gpio_set_level(MOTOR_STBY_PIN, 0);

    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 0);
    gpio_set_level(MOTOR_D_IN1, 0);
    gpio_set_level(MOTOR_D_IN2, 0);

    ledc_timer_config_t timer = {
        .speed_mode = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_LEDC_RES,
        .timer_num = MOTOR_LEDC_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_RETURN_ON_ERROR(
        ledc_timer_config(&timer),
        MOTOR_TAG,
        "LEDC timer init failed");

    ESP_RETURN_ON_ERROR(
        motor_pwm_channel_init(
            MOTOR_A_PWM,
            MOTOR_LEDC_CH_A),
        MOTOR_TAG,
        "A PWM init failed");

    ESP_RETURN_ON_ERROR(
        motor_pwm_channel_init(
            MOTOR_B_PWM,
            MOTOR_LEDC_CH_B),
        MOTOR_TAG,
        "B PWM init failed");

    ESP_RETURN_ON_ERROR(
        motor_pwm_channel_init(
            MOTOR_D_PWM,
            MOTOR_LEDC_CH_D),
        MOTOR_TAG,
        "D PWM init failed");

    ESP_RETURN_ON_ERROR(
        motor_encoder_init(&g_motor_A),
        MOTOR_TAG,
        "A encoder init failed");

    ESP_RETURN_ON_ERROR(
        motor_encoder_init(&g_motor_B),
        MOTOR_TAG,
        "B encoder init failed");

    ESP_RETURN_ON_ERROR(
        motor_encoder_init(&g_motor_D),
        MOTOR_TAG,
        "D encoder init failed");

    g_closed_loop_ready =
        g_motor_A.encoder_ready &&
        g_motor_B.encoder_ready &&
        g_motor_D.encoder_ready &&
        (MOTOR_WHEEL_DIAMETER_MM > 0.0f) &&
        (MOTOR_CHASSIS_RADIUS_MM > 0.0f);

    g_motor_command =
        (motor_command_t){0};

    g_raw_pwm_command =
        (motor_raw_pwm_command_t){0};

    g_raw_debug_mode = false;
    g_motor_initialized = true;
    g_motor_enabled = true;

    gpio_set_level(
        MOTOR_STBY_PIN,
        1);

    BaseType_t task_result =
        xTaskCreate(
            motor_control_task,
            "motor_control",
            MOTOR_TASK_STACK_SIZE,
            NULL,
            MOTOR_TASK_PRIORITY,
            &g_motor_task_handle);

    if (task_result != pdPASS)
    {
        gpio_set_level(
            MOTOR_STBY_PIN,
            0);

        g_motor_initialized = false;
        g_motor_enabled = false;
        g_motor_task_handle = NULL;

        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        MOTOR_TAG,
        "Initialized: closed_loop_ready=%d",
        (int)g_closed_loop_ready);

    if (!g_closed_loop_ready)
    {
        ESP_LOGW(
            MOTOR_TAG,
            "Closed loop disabled. Fill encoder pins/CPR, wheel diameter and chassis radius. Raw PWM debug is available.");
    }

    motor_publish_status(
        &g_motor_command);

    return ESP_OK;
}

#endif /* MOTOR_CONTROL_IMPLEMENTATION */
#endif /* MOTOR_CONTROL_H */
