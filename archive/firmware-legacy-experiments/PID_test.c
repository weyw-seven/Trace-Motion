/*
 * motor_pid_tune_test_v2.c
 *
 * V2 fixes:
 *   - trace buffers moved out of the task stack into static storage
 *   - sampling/logging changed from 20 ms to 50 ms
 *     to match the 50 ms encoder-speed window and reduce UART load
 *
 * PID tuning test for motor_control.h
 *
 * Recommended tuning order:
 *
 *   1) Tune Kp with Ki=0, Kd=0
 *   2) Fix the selected Kp for each wheel
 *   3) Tune Ki
 *   4) Leave Kd=0 initially
 *
 * Why:
 *   - Feedforward is already calibrated.
 *   - Low-speed 38 mm/s behavior is intentionally skipped.
 *   - Tests stay in the continuous-running region (~180...360 mm/s wheel speed).
 *   - Derivative action is not useful until P/PI behavior is clean.
 *
 * The test directly changes the internal wheel PID gains at runtime.
 * This is TEST-ONLY code. It works because motor_control.h implementation
 * is included in this same translation unit.
 *
 * IMPORTANT:
 *   MOTOR_CONTROL_IMPLEMENTATION must exist in exactly ONE translation unit.
 *   If it is already defined elsewhere, remove/comment the define below.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include <float.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"


/* =========================================================================
 * User configuration
 * ========================================================================= */

/*
 * SAFETY:
 * Keep 0 until the chassis is safely lifted/restrained.
 */
#ifndef MOTOR_PID_TEST_ENABLE_MOTION
#define MOTOR_PID_TEST_ENABLE_MOTION           1
#endif

/*
 * Test modes.
 */
#define MOTOR_PID_TUNE_MODE_KP                 1
#define MOTOR_PID_TUNE_MODE_KI                 2

/*
 * First run:
 *   MOTOR_PID_TUNE_MODE_KP
 *
 * After choosing Kp values:
 *   MOTOR_PID_TUNE_MODE_KI
    */
#ifndef MOTOR_PID_TUNE_MODE
#define MOTOR_PID_TUNE_MODE                    MOTOR_PID_TUNE_MODE_KP
#endif


/* =========================================================================
 * Kp sweep
 * ========================================================================= */

/*
 * Kp candidates.
 *
 * Current controller used about 0.15.
 * Sweep both below and above it.
 */
static const float s_kp_candidates[] = {
    0.05f,
    0.08f,
    0.12f,
    0.15f,
    0.18f,
    0.22f,
    0.28f,
    0.35f,
};


/* =========================================================================
 * Ki sweep
 * ========================================================================= */

/*
 * Before running KI mode:
 *
 * Replace these with the selected Kp values from the Kp test.
 */
#ifndef MOTOR_PID_TEST_A_FIXED_KP
#define MOTOR_PID_TEST_A_FIXED_KP             0.15f
#endif

#ifndef MOTOR_PID_TEST_B_FIXED_KP
#define MOTOR_PID_TEST_B_FIXED_KP             0.15f
#endif

#ifndef MOTOR_PID_TEST_D_FIXED_KP
#define MOTOR_PID_TEST_D_FIXED_KP             0.15f
#endif

/*
 * Integral candidates.
 *
 * Units are PWM / (mm/s*s) in the current implementation.
 * Start conservatively.
 */
static const float s_ki_candidates[] = {
    0.00f,
    0.03f,
    0.06f,
    0.10f,
    0.16f,
    0.25f,
};


/* =========================================================================
 * Test timing
 * ========================================================================= */

/*
 * Kp needs transient information.
 */
#ifndef MOTOR_PID_KP_STEP_MS
#define MOTOR_PID_KP_STEP_MS                  1600U
#endif

/*
 * Ki needs a longer observation window.
 */
#ifndef MOTOR_PID_KI_STEP_MS
#define MOTOR_PID_KI_STEP_MS                  3000U
#endif

#ifndef MOTOR_PID_SAMPLE_MS
#define MOTOR_PID_SAMPLE_MS                     50U
#endif

/*
 * Stop between step directions/cases.
 *
 * This also flushes the 50 ms encoder speed window.
 */
#ifndef MOTOR_PID_STOP_SETTLE_MS
#define MOTOR_PID_STOP_SETTLE_MS               600U
#endif

/*
 * Ignore the first part of the response when calculating MAE,
 * because startup and the 50 ms speed-estimation window dominate it.
 */
#ifndef MOTOR_PID_METRIC_IGNORE_MS
#define MOTOR_PID_METRIC_IGNORE_MS             100U
#endif

/*
 * Steady-state metric uses the final window.
 */
#ifndef MOTOR_PID_KP_STEADY_WINDOW_MS
#define MOTOR_PID_KP_STEADY_WINDOW_MS          400U
#endif

#ifndef MOTOR_PID_KI_STEADY_WINDOW_MS
#define MOTOR_PID_KI_STEADY_WINDOW_MS          800U
#endif

/*
 * Settling band = +/- 10% of target.
 */
#ifndef MOTOR_PID_SETTLE_BAND
#define MOTOR_PID_SETTLE_BAND                  0.10f
#endif

/*
 * Rise-time threshold = 90% of target.
 */
#ifndef MOTOR_PID_RISE_FRACTION
#define MOTOR_PID_RISE_FRACTION                0.90f
#endif


/* =========================================================================
 * Test cases
 * ========================================================================= */

/*
 * All tests deliberately avoid the 38 mm/s region.
 *
 * Normalized Set_motor() commands:
 *
 * ROT +/-:
 *   w = +/-1.0 -> wheel targets about +/-190 mm/s on A/B/D.
 *
 * VX +/-:
 *   vx = +/-0.40
 *   B/D targets about +/-312 mm/s, A=0.
 *
 * VY +/-:
 *   vy = +/-0.40
 *   A target about -/+360 mm/s
 *   B/D target about +/-180 mm/s.
 *
 * Together these exercise:
 *   - both directions
 *   - all three wheels
 *   - ~180...360 mm/s operating range
 */
typedef struct {
    const char *name;
    float vx;
    float vy;
    float w;
} pid_test_case_t;

static const pid_test_case_t s_test_cases[] = {
    { "ROT_FWD",  0.00f,  0.00f, +1.00f },
    { "ROT_REV",  0.00f,  0.00f, -1.00f },

    { "VX_FWD",  +0.40f,  0.00f,  0.00f },
    { "VX_REV",  -0.40f,  0.00f,  0.00f },

    { "VY_LEFT",  0.00f, +0.40f,  0.00f },
    { "VY_RIGHT", 0.00f, -0.40f,  0.00f },
};


static const char *TAG = "MOTOR_PID_TEST";


/* =========================================================================
 * Data structures
 * ========================================================================= */

typedef enum {
    TEST_WHEEL_A = 0,
    TEST_WHEEL_B,
    TEST_WHEEL_D,
    TEST_WHEEL_COUNT
} test_wheel_t;

typedef struct {
    uint32_t t_ms;
    float target;
    float actual;
    float pwm;
} pid_sample_t;

/*
 * 3000 ms / 50 ms = 60 samples.
 * Add margin.
 */
#define MOTOR_PID_MAX_SAMPLES  80U

typedef struct {
    pid_sample_t sample[MOTOR_PID_MAX_SAMPLES];
    uint32_t count;
} wheel_trace_t;

typedef struct {
    float target_abs;

    float rise_time_ms;
    float settling_time_ms;

    float overshoot_pct;
    float mean_abs_error_pct;
    float steady_error_pct;

    float steady_mean_speed;
    float peak_projected_speed;

    bool reached_90pct;
    bool settled;
} pid_metric_t;

typedef struct {
    float sum_mae_pct;
    float sum_overshoot_pct;
    float sum_steady_error_pct;
    float sum_settle_ms;

    uint32_t active_tests;
    uint32_t failed_to_reach;
    uint32_t failed_to_settle;
} wheel_gain_accum_t;

/*
 * Static trace storage.
 *
 * Do NOT place this array inside run_one_case().
 * Even at 80 samples it is several KB; the original 180-sample
 * local array exceeded the ESP-IDF main task stack.
 */
static wheel_trace_t s_traces[TEST_WHEEL_COUNT];


/* =========================================================================
 * Helpers
 * ========================================================================= */

static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static const char *wheel_name(test_wheel_t wheel)
{
    switch (wheel) {
    case TEST_WHEEL_A: return "A";
    case TEST_WHEEL_B: return "B";
    case TEST_WHEEL_D: return "D";
    default: return "?";
    }
}

static const motor_wheel_status_t *get_wheel_status(
    const motor_status_t *s,
    test_wheel_t wheel)
{
    if (s == NULL) {
        return NULL;
    }

    switch (wheel) {
    case TEST_WHEEL_A: return &s->A;
    case TEST_WHEEL_B: return &s->B;
    case TEST_WHEEL_D: return &s->D;
    default: return NULL;
    }
}

static motor_wheel_t *get_internal_wheel(test_wheel_t wheel)
{
    switch (wheel) {
    case TEST_WHEEL_A: return &g_motor_A;
    case TEST_WHEEL_B: return &g_motor_B;
    case TEST_WHEEL_D: return &g_motor_D;
    default: return NULL;
    }
}

static void stop_and_settle(void)
{
    motor_stop();
    delay_ms(MOTOR_PID_STOP_SETTLE_MS);
}

/*
 * Test-only runtime PID gain update.
 *
 * Disable outputs while changing internal controller coefficients,
 * then re-enable and reset cleanly.
 */
static void apply_pid_gains(
    float a_kp, float a_ki, float a_kd,
    float b_kp, float b_ki, float b_kd,
    float d_kp, float d_ki, float d_kd)
{
    motor_control_enable(false);
    delay_ms(50U);

    g_motor_A.kp = a_kp;
    g_motor_A.ki = a_ki;
    g_motor_A.kd = a_kd;

    g_motor_B.kp = b_kp;
    g_motor_B.ki = b_ki;
    g_motor_B.kd = b_kd;

    g_motor_D.kp = d_kp;
    g_motor_D.ki = d_ki;
    g_motor_D.kd = d_kd;

    motor_controller_reset(&g_motor_A);
    motor_controller_reset(&g_motor_B);
    motor_controller_reset(&g_motor_D);

    motor_control_enable(true);
    delay_ms(100U);
}


/* =========================================================================
 * Trace collection
 * ========================================================================= */

static void clear_traces(wheel_trace_t traces[TEST_WHEEL_COUNT])
{
    for (int w = 0; w < TEST_WHEEL_COUNT; ++w) {
        traces[w].count = 0;
    }
}

static void append_sample(
    wheel_trace_t *trace,
    uint32_t t_ms,
    float target,
    float actual,
    float pwm)
{
    if (trace == NULL ||
        trace->count >= MOTOR_PID_MAX_SAMPLES)
    {
        return;
    }

    pid_sample_t *s =
        &trace->sample[trace->count++];

    s->t_ms = t_ms;
    s->target = target;
    s->actual = actual;
    s->pwm = pwm;
}


/* =========================================================================
 * Metrics
 * ========================================================================= */

static pid_metric_t analyze_trace(
    const wheel_trace_t *trace,
    uint32_t step_ms,
    uint32_t steady_window_ms)
{
    pid_metric_t m = {0};

    m.rise_time_ms = -1.0f;
    m.settling_time_ms = -1.0f;

    if (trace == NULL ||
        trace->count == 0U)
    {
        return m;
    }

    /*
     * Target should remain constant during one step.
     */
    float target = trace->sample[trace->count - 1U].target;
    const float target_abs = fabsf(target);

    m.target_abs = target_abs;

    /*
     * Ignore inactive wheel targets.
     */
    if (target_abs < 20.0f) {
        return m;
    }

    const float sign =
        (target >= 0.0f) ? +1.0f : -1.0f;

    float peak_projected = -FLT_MAX;

    float abs_error_sum = 0.0f;
    uint32_t error_samples = 0U;

    float steady_sum = 0.0f;
    uint32_t steady_samples = 0U;

    const uint32_t steady_start_ms =
        (step_ms > steady_window_ms)
        ? (step_ms - steady_window_ms)
        : 0U;

    /*
     * First pass:
     *   rise time
     *   peak/overshoot
     *   mean abs error
     *   steady-state mean
     */
    for (uint32_t i = 0;
         i < trace->count;
         ++i)
    {
        const pid_sample_t *s =
            &trace->sample[i];

        const float projected =
            s->actual * sign;

        if (projected > peak_projected) {
            peak_projected = projected;
        }

        if (!m.reached_90pct &&
            projected >=
                MOTOR_PID_RISE_FRACTION * target_abs)
        {
            m.reached_90pct = true;
            m.rise_time_ms = (float)s->t_ms;
        }

        if (s->t_ms >= MOTOR_PID_METRIC_IGNORE_MS) {
            const float e =
                fabsf(s->target - s->actual);

            abs_error_sum += e;
            ++error_samples;
        }

        if (s->t_ms >= steady_start_ms) {
            steady_sum += s->actual;
            ++steady_samples;
        }
    }

    if (peak_projected < 0.0f) {
        peak_projected = 0.0f;
    }

    m.peak_projected_speed =
        peak_projected;

    if (peak_projected > target_abs) {
        m.overshoot_pct =
            100.0f *
            (peak_projected - target_abs) /
            target_abs;
    }

    if (error_samples > 0U) {
        const float mae =
            abs_error_sum /
            (float)error_samples;

        m.mean_abs_error_pct =
            100.0f *
            mae /
            target_abs;
    }

    if (steady_samples > 0U) {
        m.steady_mean_speed =
            steady_sum /
            (float)steady_samples;

        m.steady_error_pct =
            100.0f *
            fabsf(target - m.steady_mean_speed) /
            target_abs;
    }

    /*
     * Settling time:
     *
     * Earliest sample for which every later sample remains
     * inside +/-10% target band.
     */
    const float band =
        MOTOR_PID_SETTLE_BAND *
        target_abs;

    for (uint32_t i = 0;
         i < trace->count;
         ++i)
    {
        bool all_inside = true;

        for (uint32_t j = i;
             j < trace->count;
             ++j)
        {
            const pid_sample_t *s =
                &trace->sample[j];

            if (fabsf(s->actual - target) > band) {
                all_inside = false;
                break;
            }
        }

        if (all_inside) {
            m.settled = true;
            m.settling_time_ms =
                (float)trace->sample[i].t_ms;
            break;
        }
    }

    return m;
}


/* =========================================================================
 * Accumulation / scoring
 * ========================================================================= */

static void accum_metric(
    wheel_gain_accum_t *a,
    const pid_metric_t *m,
    uint32_t step_ms)
{
    if (a == NULL ||
        m == NULL ||
        m->target_abs < 20.0f)
    {
        return;
    }

    ++a->active_tests;

    a->sum_mae_pct +=
        m->mean_abs_error_pct;

    a->sum_overshoot_pct +=
        m->overshoot_pct;

    a->sum_steady_error_pct +=
        m->steady_error_pct;

    if (m->settled) {
        a->sum_settle_ms +=
            m->settling_time_ms;
    } else {
        /*
         * Penalize non-settling response.
         */
        a->sum_settle_ms +=
            (float)step_ms;

        ++a->failed_to_settle;
    }

    if (!m->reached_90pct) {
        ++a->failed_to_reach;
    }
}

static float gain_score(
    const wheel_gain_accum_t *a)
{
    if (a == NULL ||
        a->active_tests == 0U)
    {
        return 1.0e9f;
    }

    const float n =
        (float)a->active_tests;

    const float mean_mae =
        a->sum_mae_pct / n;

    const float mean_overshoot =
        a->sum_overshoot_pct / n;

    const float mean_ss =
        a->sum_steady_error_pct / n;

    const float mean_settle =
        a->sum_settle_ms / n;

    /*
     * Lower score is better.
     *
     * Steady error and MAE matter most.
     * Overshoot is penalized moderately.
     * Slow/non-settling response is also penalized.
     */
    float score =
        1.00f * mean_mae +
        1.20f * mean_ss +
        0.80f * mean_overshoot +
        0.020f * mean_settle;

    score +=
        20.0f *
        (float)a->failed_to_reach;

    score +=
        15.0f *
        (float)a->failed_to_settle;

    return score;
}


/* =========================================================================
 * One chassis step test
 * ========================================================================= */

static void run_one_case(
    const pid_test_case_t *tc,
    uint32_t step_ms,
    uint32_t steady_window_ms,
    const char *mode_name,
    float candidate_value,
    wheel_gain_accum_t accum[TEST_WHEEL_COUNT])
{
    if (tc == NULL) {
        return;
    }

    stop_and_settle();

    /*
     * Reuse static storage instead of allocating large traces on task stack.
     */
    clear_traces(s_traces);

    ESP_LOGI(
        TAG,
        "CASE,%s,mode=%s,candidate=%.6f,"
        "vx=%.3f,vy=%.3f,w=%.3f",
        tc->name,
        mode_name,
        (double)candidate_value,
        (double)tc->vx,
        (double)tc->vy,
        (double)tc->w);

    Set_motor(
        tc->vx,
        tc->vy,
        tc->w);

    uint32_t elapsed = 0U;

    while (elapsed < step_ms) {
        delay_ms(MOTOR_PID_SAMPLE_MS);
        elapsed += MOTOR_PID_SAMPLE_MS;

        motor_status_t s = {0};
        motor_get_status(&s);

        for (int wi = 0;
             wi < TEST_WHEEL_COUNT;
             ++wi)
        {
            const motor_wheel_status_t *w =
                get_wheel_status(
                    &s,
                    (test_wheel_t)wi);

            if (w == NULL) {
                continue;
            }

            append_sample(
                &s_traces[wi],
                elapsed,
                w->target_speed_mm_s,
                w->actual_speed_mm_s,
                w->output_pwm);
        }

        /*
         * Compact trace output.
         * Useful for plotting later if needed.
         */
        ESP_LOGI(
            TAG,
            "CSV,PID_SAMPLE,%s,%s,%.6f,%" PRIu32 ","
            "A,%.3f,%.3f,%.3f,"
            "B,%.3f,%.3f,%.3f,"
            "D,%.3f,%.3f,%.3f",
            mode_name,
            tc->name,
            (double)candidate_value,
            elapsed,

            (double)s.A.target_speed_mm_s,
            (double)s.A.actual_speed_mm_s,
            (double)s.A.output_pwm,

            (double)s.B.target_speed_mm_s,
            (double)s.B.actual_speed_mm_s,
            (double)s.B.output_pwm,

            (double)s.D.target_speed_mm_s,
            (double)s.D.actual_speed_mm_s,
            (double)s.D.output_pwm);
    }

    /*
     * Analyze each wheel.
     */
    for (int wi = 0;
         wi < TEST_WHEEL_COUNT;
         ++wi)
    {
        pid_metric_t m =
            analyze_trace(
                &s_traces[wi],
                step_ms,
                steady_window_ms);

        if (m.target_abs < 20.0f) {
            continue;
        }

        accum_metric(
            &accum[wi],
            &m,
            step_ms);

        ESP_LOGI(
            TAG,
            "CSV,PID_METRIC,%s,%s,%.6f,%s,"
            "target_abs,%.3f,"
            "rise_ms,%.1f,"
            "settle_ms,%.1f,"
            "overshoot_pct,%.3f,"
            "mae_pct,%.3f,"
            "steady_error_pct,%.3f,"
            "steady_mean,%.3f,"
            "peak_projected,%.3f,"
            "reached90,%d,"
            "settled,%d",
            mode_name,
            tc->name,
            (double)candidate_value,
            wheel_name((test_wheel_t)wi),

            (double)m.target_abs,
            (double)m.rise_time_ms,
            (double)m.settling_time_ms,
            (double)m.overshoot_pct,
            (double)m.mean_abs_error_pct,
            (double)m.steady_error_pct,
            (double)m.steady_mean_speed,
            (double)m.peak_projected_speed,
            (int)m.reached_90pct,
            (int)m.settled);
    }

    stop_and_settle();
}


/* =========================================================================
 * Print candidate summary
 * ========================================================================= */

static void print_candidate_summary(
    const char *mode_name,
    float candidate,
    const wheel_gain_accum_t accum[TEST_WHEEL_COUNT])
{
    for (int wi = 0;
         wi < TEST_WHEEL_COUNT;
         ++wi)
    {
        const wheel_gain_accum_t *a =
            &accum[wi];

        if (a->active_tests == 0U) {
            continue;
        }

        const float n =
            (float)a->active_tests;

        const float mean_mae =
            a->sum_mae_pct / n;

        const float mean_overshoot =
            a->sum_overshoot_pct / n;

        const float mean_ss =
            a->sum_steady_error_pct / n;

        const float mean_settle =
            a->sum_settle_ms / n;

        const float score =
            gain_score(a);

        ESP_LOGI(
            TAG,
            "CSV,PID_RESULT,%s,%.6f,%s,"
            "score,%.3f,"
            "mean_mae_pct,%.3f,"
            "mean_overshoot_pct,%.3f,"
            "mean_steady_error_pct,%.3f,"
            "mean_settle_ms,%.1f,"
            "failed_reach,%" PRIu32 ","
            "failed_settle,%" PRIu32 ","
            "active_tests,%" PRIu32,
            mode_name,
            (double)candidate,
            wheel_name((test_wheel_t)wi),

            (double)score,
            (double)mean_mae,
            (double)mean_overshoot,
            (double)mean_ss,
            (double)mean_settle,

            a->failed_to_reach,
            a->failed_to_settle,
            a->active_tests);
    }
}


/* =========================================================================
 * Kp sweep
 * ========================================================================= */

static void run_kp_sweep(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# PHASE 1: Kp sweep");
    ESP_LOGI(
        TAG,
        "# Ki=0, Kd=0");
    ESP_LOGI(
        TAG,
        "############################################################");

    for (uint32_t ci = 0;
         ci < sizeof(s_kp_candidates) / sizeof(s_kp_candidates[0]);
         ++ci)
    {
        const float kp =
            s_kp_candidates[ci];

        ESP_LOGW(
            TAG,
            "Testing Kp = %.4f",
            (double)kp);

        /*
         * Same candidate on all wheels.
         *
         * PID_RESULT is printed separately per wheel, so afterwards
         * A/B/D may choose different Kp values.
         */
        apply_pid_gains(
            kp, 0.0f, 0.0f,
            kp, 0.0f, 0.0f,
            kp, 0.0f, 0.0f);

        wheel_gain_accum_t accum[TEST_WHEEL_COUNT] = {0};

        for (uint32_t ti = 0;
             ti < sizeof(s_test_cases) / sizeof(s_test_cases[0]);
             ++ti)
        {
            run_one_case(
                &s_test_cases[ti],
                MOTOR_PID_KP_STEP_MS,
                MOTOR_PID_KP_STEADY_WINDOW_MS,
                "KP",
                kp,
                accum);
        }

        print_candidate_summary(
            "KP",
            kp,
            accum);
    }
}


/* =========================================================================
 * Ki sweep
 * ========================================================================= */

static void run_ki_sweep(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# PHASE 2: Ki sweep");
    ESP_LOGI(
        TAG,
        "# Fixed Kp:");
    ESP_LOGI(
        TAG,
        "# A=%.4f B=%.4f D=%.4f",
        (double)MOTOR_PID_TEST_A_FIXED_KP,
        (double)MOTOR_PID_TEST_B_FIXED_KP,
        (double)MOTOR_PID_TEST_D_FIXED_KP);
    ESP_LOGI(
        TAG,
        "# Kd=0");
    ESP_LOGI(
        TAG,
        "############################################################");

    for (uint32_t ci = 0;
         ci < sizeof(s_ki_candidates) / sizeof(s_ki_candidates[0]);
         ++ci)
    {
        const float ki =
            s_ki_candidates[ci];

        ESP_LOGW(
            TAG,
            "Testing Ki = %.4f",
            (double)ki);

        /*
         * Same Ki candidate is tested on all wheels,
         * while Kp can already be wheel-specific.
         *
         * PID_RESULT remains per-wheel.
         */
        apply_pid_gains(
            MOTOR_PID_TEST_A_FIXED_KP, ki, 0.0f,
            MOTOR_PID_TEST_B_FIXED_KP, ki, 0.0f,
            MOTOR_PID_TEST_D_FIXED_KP, ki, 0.0f);

        wheel_gain_accum_t accum[TEST_WHEEL_COUNT] = {0};

        for (uint32_t ti = 0;
             ti < sizeof(s_test_cases) / sizeof(s_test_cases[0]);
             ++ti)
        {
            run_one_case(
                &s_test_cases[ti],
                MOTOR_PID_KI_STEP_MS,
                MOTOR_PID_KI_STEADY_WINDOW_MS,
                "KI",
                ki,
                accum);
        }

        print_candidate_summary(
            "KI",
            ki,
            accum);
    }
}


/* =========================================================================
 * Main
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# motor_control.h PID tuning test");
    ESP_LOGI(
        TAG,
        "############################################################");

    ESP_LOGI(
        TAG,
        "Low-speed 38 mm/s region is intentionally skipped.");

    ESP_LOGI(
        TAG,
        "CSV_HEADER,PID_RESULT,"
        "mode,candidate,wheel,"
        "score,mean_mae_pct,mean_overshoot_pct,"
        "mean_steady_error_pct,mean_settle_ms,"
        "failed_reach,failed_settle,active_tests");

    esp_err_t err =
        motor_control_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "motor_control_init failed: %s",
            esp_err_to_name(err));
        return;
    }

    delay_ms(200U);

    if (!motor_control_is_ready()) {
        ESP_LOGE(
            TAG,
            "motor_control is not closed-loop ready.");
        motor_emergency_stop();
        return;
    }

#if !MOTOR_PID_TEST_ENABLE_MOTION

    ESP_LOGW(TAG, "");
    ESP_LOGW(
        TAG,
        "MOTION TEST DISABLED.");
    ESP_LOGW(
        TAG,
        "Lift/restrain the chassis and set:");
    ESP_LOGW(
        TAG,
        "#define MOTOR_PID_TEST_ENABLE_MOTION 1");

    motor_emergency_stop();
    return;

#else

    ESP_LOGW(TAG, "");
    ESP_LOGW(
        TAG,
        "MOTION TEST ENABLED.");
    ESP_LOGW(
        TAG,
        "Keep chassis lifted/restrained.");

#if MOTOR_PID_TUNE_MODE == MOTOR_PID_TUNE_MODE_KP

    run_kp_sweep();

#elif MOTOR_PID_TUNE_MODE == MOTOR_PID_TUNE_MODE_KI

    run_ki_sweep();

#else

#error "Invalid MOTOR_PID_TUNE_MODE"

#endif

    motor_emergency_stop();

    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# PID tuning run complete");
    ESP_LOGI(
        TAG,
        "############################################################");

#if MOTOR_PID_TUNE_MODE == MOTOR_PID_TUNE_MODE_KP

    ESP_LOGI(
        TAG,
        "Send all CSV,PID_RESULT,KP lines.");
    ESP_LOGI(
        TAG,
        "Choose Kp per wheel from the lowest-score stable region,");
    ESP_LOGI(
        TAG,
        "not blindly from a single minimum score.");

#elif MOTOR_PID_TUNE_MODE == MOTOR_PID_TUNE_MODE_KI

    ESP_LOGI(
        TAG,
        "Send all CSV,PID_RESULT,KI lines.");
    ESP_LOGI(
        TAG,
        "Choose the smallest Ki that materially reduces steady error");
    ESP_LOGI(
        TAG,
        "without increasing overshoot/settling noticeably.");

#endif

#endif
}