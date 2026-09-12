#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"
#include "line_vision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. Control period
 * ============================================================ */
#ifndef LINE_CONTROL_PERIOD_MS
#define LINE_CONTROL_PERIOD_MS          20U
#endif

#define LINE_CONTROL_DT \
    ((float)LINE_CONTROL_PERIOD_MS / 1000.0f)

#ifndef LINE_TASK_STACK_SIZE
#define LINE_TASK_STACK_SIZE            3072U
#endif

#ifndef LINE_TASK_PRIORITY
#define LINE_TASK_PRIORITY              6U
#endif

/* ============================================================
 * 2. FOLLOW parameters
 *
 * Set_motor(vx, vy, w) uses normalized commands [-1, 1].
 * ============================================================ */
#ifndef LINE_KP
#define LINE_KP                         0.50f
#endif

#ifndef LINE_KD
#define LINE_KD                         0.035f
#endif

#ifndef LINE_D_ALPHA
#define LINE_D_ALPHA                    0.35f
#endif

#ifndef LINE_W_MAX
#define LINE_W_MAX                      0.50f
#endif

#ifndef LINE_V_MAX
#define LINE_V_MAX                      0.15f
#endif

#ifndef LINE_V_MIN
#define LINE_V_MIN                      0.04f
#endif

#define LINE_REVERSE_START              0.30f
#define LINE_REVERSE_V                  0.10f

/* ============================================================
 * 3. SEARCH / LOST / FINISH
 * ============================================================ */
#ifndef LINE_START_SEARCH_W
#define LINE_START_SEARCH_W             0.35f
#endif

#ifndef LINE_START_CONFIRM_COUNT
#define LINE_START_CONFIRM_COUNT        3U
#endif

#ifndef LINE_LOST_FORWARD_V
#define LINE_LOST_FORWARD_V            -0.03f
#endif

#ifndef LINE_LOST_TURN_W
#define LINE_LOST_TURN_W                0.28f
#endif

#ifndef LINE_LOST_FORWARD_MS
#define LINE_LOST_FORWARD_MS            100U
#endif

#ifndef LINE_LOST_SEARCH_W
#define LINE_LOST_SEARCH_W              0.32f
#endif

#ifndef LINE_LOST_TIMEOUT_MS
#define LINE_LOST_TIMEOUT_MS            2500U
#endif

#ifndef LINE_FINISH_CONFIRM_COUNT
#define LINE_FINISH_CONFIRM_COUNT       1U
#endif

#ifndef LINE_FINISH_V
#define LINE_FINISH_V                   0.05f
#endif

#ifndef LINE_FINISH_ENABLE_DELAY_MS
#define LINE_FINISH_ENABLE_DELAY_MS     1000U
#endif

/*
 * Visual equivalent of the old finish pattern:
 * after obstacle avoidance is completed, a mostly-black ROI is treated
 * as the finish mark. The vision module only reports black_ratio; the
 * task meaning of "finish" stays here in the tracker.
 */
#ifndef LINE_FINISH_BLACK_RATIO
#define LINE_FINISH_BLACK_RATIO           0.60f
#endif

#ifndef LINE_ERROR_M1_WEIGHT
#define LINE_ERROR_M1_WEIGHT              0.30f
#endif

#ifndef LINE_ERROR_M2_WEIGHT
#define LINE_ERROR_M2_WEIGHT              0.70f
#endif

#ifndef LINE_ERROR_DEADBAND
#define LINE_ERROR_DEADBAND               0.06f
#endif

#ifndef LINE_V_STRAIGHT_W
#define LINE_V_STRAIGHT_W                 0.12f
#endif

#ifndef LINE_V_SLOW_W
#define LINE_V_SLOW_W                     0.65f
#endif


/* ============================================================
 * 4. Debug
 * ============================================================ */
#ifndef LINE_DEBUG_ENABLE
#define LINE_DEBUG_ENABLE               1
#endif

#ifndef LINE_DEBUG_PERIOD_MS
#define LINE_DEBUG_PERIOD_MS            100U
#endif

/* ============================================================
 * 5. Public API
 * ============================================================ */
typedef enum
{
    LINE_STATE_SEARCH_START = 0,
    LINE_STATE_FOLLOW,
    LINE_STATE_LOST,
    LINE_STATE_FINISH_CONFIRM,
    LINE_STATE_FINISHED,
    LINE_STATE_STOPPED

} line_state_t;

typedef struct
{
    /* Keep the original infrared-version fields first for source compatibility. */
    line_state_t state;

    uint8_t pattern;          /* Vision build: always 0; no virtual IR pattern is fabricated. */
    bool line_found;

    float error;
    float d_filtered;

    float target_vx;
    float target_w;

    uint32_t lost_elapsed_ms;
    uint32_t follow_elapsed_ms;
    uint32_t ir_sample_count; /* Compatibility: mirrors latest vision sequence. */

    /* Vision-specific diagnostics appended after the old public fields. */
    bool vision_valid;
    uint32_t vision_sequence;
    float vision_center_x;
    float vision_black_ratio;

    /* Extended diagnostics used by the browser debug observer. */
    uint32_t control_sequence;
    bool measurement_new;
    bool avoid_done;

    float vision_roi_center_x;
    float vision_roi_half_width;
    uint32_t vision_black_pixels;
    uint32_t vision_roi_pixels;

    int64_t vision_frame_timestamp_us;
    int64_t vision_update_timestamp_us;
    float vision_dt_s;

} line_tracker_status_t;

void line_tracker_enable(bool enable);
bool line_tracker_is_enabled(void);

esp_err_t line_tracker_init(void);

void line_tracker_start(void);
void line_tracker_stop(void);
void line_tracker_reset(void);

void line_tracker_get_status(line_tracker_status_t *status);
void line_tracker_set_avoid_done(bool done);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 *
 * Define LINE_TRACKER_IMPLEMENTATION in exactly one .c file.
 * ============================================================ */
#ifdef LINE_TRACKER_IMPLEMENTATION

static const char *LINE_TAG = "LINE";

static line_tracker_status_t g_line_work = {0};
static line_tracker_status_t g_line_status = {0};

static TaskHandle_t g_line_task_handle = NULL;
static bool g_line_initialized = false;

static bool g_line_run_request = false;
static bool g_line_reset_request = false;

static portMUX_TYPE g_line_lock =
    portMUX_INITIALIZER_UNLOCKED;

static bool s_line_enabled = true;
static bool s_avoid_done = false;

/* ============================================================
 * 6. Public request flags
 * ============================================================ */
void line_tracker_enable(bool enable)
{
    taskENTER_CRITICAL(&g_line_lock);
    s_line_enabled = enable;
    taskEXIT_CRITICAL(&g_line_lock);
}

bool line_tracker_is_enabled(void)
{
    bool en;

    taskENTER_CRITICAL(&g_line_lock);
    en = s_line_enabled;
    taskEXIT_CRITICAL(&g_line_lock);

    return en;
}

void line_tracker_set_avoid_done(bool done)
{
    taskENTER_CRITICAL(&g_line_lock);
    s_avoid_done = done;
    taskEXIT_CRITICAL(&g_line_lock);

    ESP_LOGI(LINE_TAG, "avoid_done=%d", done);
}

/* ============================================================
 * 7. Internal algorithm state
 * ============================================================ */
static float g_prev_error = 0.0f;
static float g_d_filtered = 0.0f;

/*
 * Sign of the last valid steering direction:
 * +1 -> w > 0
 * -1 -> w < 0
 */
static int8_t g_last_turn_sign = 1;

static uint32_t g_start_confirm_count = 0U;
static uint32_t g_finish_confirm_count = 0U;
static uint32_t g_debug_elapsed_ms = 0U;

/* Camera-observation clock is independent from the 50 Hz actuator clock. */
static uint32_t g_control_sequence = 0U;
static bool g_have_vision_sequence = false;
static uint32_t g_last_vision_sequence = 0U;
static int64_t g_last_measurement_timestamp_us = 0;

/* PD history advances only when a genuinely new camera observation arrives. */
static bool g_pd_has_sample = false;
static int64_t g_prev_pd_timestamp_us = 0;

/* ============================================================
 * 8. Helpers
 * ============================================================ */
static inline float line_clampf(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static const char *line_state_name(line_state_t state)
{
    switch (state)
    {
        case LINE_STATE_SEARCH_START:   return "SEARCH";
        case LINE_STATE_FOLLOW:         return "FOLLOW";
        case LINE_STATE_LOST:           return "LOST";
        case LINE_STATE_FINISH_CONFIRM: return "FINISH_CHECK";
        case LINE_STATE_FINISHED:       return "FINISHED";
        case LINE_STATE_STOPPED:        return "STOPPED";
        default:                        return "UNKNOWN";
    }
}

static float line_apply_error_deadband(float error)
{
    const float deadband =
        line_clampf(
            LINE_ERROR_DEADBAND,
            0.0f,
            0.95f);

    const float magnitude = fabsf(error);

    if (magnitude <= deadband)
    {
        return 0.0f;
    }

    /*
     * Continuous dead-zone mapping:
     *
     * deadband -> 0
     * 1.0      -> 1.0
     *
     * Unlike a simple:
     *     if(abs(e)<dead) e=0;
     *
     * this does not create a sudden jump at the edge.
     */
    const float mapped =
        (magnitude - deadband) /
        (1.0f - deadband);

    return
        (error >= 0.0f)
        ? mapped
        : -mapped;
}

/*
 * This is the only conceptual replacement for line_pattern_to_error().
 *
 * Infrared version:
 *     discrete sensor positions -> weighted centroid -> error [-1, +1]
 *
 * Vision version:
 *     continuous ROI centroid -> normalized offset -> error [-1, +1]
 *
 * Error > 0 means the line lies to the right side of the ROI.
 */
static float line_visual_to_error(
    const line_vision_data_t *vision,
    bool *line_found)
{
    if (line_found != NULL)
    {
        *line_found = false;
    }

    if ((vision == NULL) ||
        (!vision->valid) ||
        (!vision->line_detected))
    {
        return 0.0f;
    }

    if (line_found != NULL)
    {
        *line_found = true;
    }

    /*
     * M1:
     * ordinary normalized centroid.
     *
     * M2:
     * signed quadratic spatial moment.
     *
     * M2 suppresses tiny center deviations while giving
     * progressively greater importance to far-away pixels.
     */
    float error =
        LINE_ERROR_M1_WEIGHT *
            vision->center_error +
        LINE_ERROR_M2_WEIGHT *
            vision->signed_moment2;

    error =
        line_clampf(
            error,
            -1.0f,
            +1.0f);

    error =
        line_apply_error_deadband(error);

    return error;
}

static bool line_visual_is_finish(
    const line_vision_data_t *vision)
{
    return
        (vision != NULL) &&
        vision->valid &&
        (vision->black_ratio >= LINE_FINISH_BLACK_RATIO);
}

static void line_reset_pd(
    float current_error,
    int64_t timestamp_us)
{
    g_prev_error = current_error;
    g_d_filtered = 0.0f;
    g_prev_pd_timestamp_us = timestamp_us;
    g_pd_has_sample = (timestamp_us > 0);
    g_line_work.d_filtered = 0.0f;
}

static void line_reset_algorithm(void)
{
    g_line_work.state = LINE_STATE_SEARCH_START;

    g_line_work.line_found = false;
    g_line_work.vision_valid = false;

    g_line_work.error = 0.0f;
    g_line_work.d_filtered = 0.0f;

    g_line_work.target_vx = 0.0f;
    g_line_work.target_w = 0.0f;

    g_line_work.lost_elapsed_ms = 0U;
    g_line_work.follow_elapsed_ms = 0U;

    g_line_work.vision_sequence = 0U;
    g_line_work.vision_center_x = 0.0f;
    g_line_work.vision_black_ratio = 0.0f;

    g_line_work.measurement_new = false;
    g_line_work.avoid_done = false;
    g_line_work.vision_roi_center_x = 0.0f;
    g_line_work.vision_roi_half_width = 0.0f;
    g_line_work.vision_black_pixels = 0U;
    g_line_work.vision_roi_pixels = 0U;
    g_line_work.vision_frame_timestamp_us = 0;
    g_line_work.vision_update_timestamp_us = 0;
    g_line_work.vision_dt_s = 0.0f;

    g_line_work.pattern = 0U;
    g_line_work.ir_sample_count = 0U;

    g_prev_error = 0.0f;
    g_d_filtered = 0.0f;
    g_pd_has_sample = false;
    g_prev_pd_timestamp_us = 0;

    g_last_turn_sign = 1;

    g_start_confirm_count = 0U;
    g_finish_confirm_count = 0U;
    g_debug_elapsed_ms = 0U;
}

static void line_set_command(float vx, float w)
{
    if (!s_line_enabled)
    {
        return;
    }

    vx = line_clampf(vx, -1.0f, 1.0f);
    w = line_clampf(w, -1.0f, 1.0f);

    g_line_work.target_vx = vx;
    g_line_work.target_w = w;

    Set_motor(vx, 0.0f, w);
}

static void line_publish_status(void)
{
    taskENTER_CRITICAL(&g_line_lock);
    g_line_status = g_line_work;
    taskEXIT_CRITICAL(&g_line_lock);
}

/* ============================================================
 * 9. FOLLOW controller -- unchanged from infrared version
 * ============================================================ */
static float line_calculate_w(
    float error,
    int64_t timestamp_us)
{
    if (!g_pd_has_sample)
    {
        g_d_filtered = 0.0f;
    }
    else if ((timestamp_us > 0) &&
             (timestamp_us > g_prev_pd_timestamp_us))
    {
        const float dt =
            (float)(timestamp_us - g_prev_pd_timestamp_us) * 1.0e-6f;

        /*
         * Camera is nominally 5 FPS (~0.200 s). Reject intervals that are
         * clearly a discontinuity or duplicate timestamp instead of creating
         * a derivative spike.
         */
        if ((dt >= 0.040f) && (dt <= 0.500f))
        {
            const float d_raw = (error - g_prev_error) / dt;

            /*
             * LINE_D_ALPHA remains a 20 ms reference tuning parameter.
             * Convert it to an equivalent first-order time constant and then
             * evaluate alpha at the actual camera-frame interval.
             */
            const float alpha_reference =
                line_clampf(LINE_D_ALPHA, 0.001f, 1.0f);

            float alpha = 1.0f;

            if (alpha_reference < 0.999f)
            {
                const float tau =
                    LINE_CONTROL_DT *
                    (1.0f - alpha_reference) /
                    alpha_reference;

                alpha = dt / (tau + dt);
                alpha = line_clampf(alpha, 0.0f, 1.0f);
            }

            g_d_filtered +=
                alpha * (d_raw - g_d_filtered);
        }
        else
        {
            g_d_filtered = 0.0f;
        }
    }

    /*
     * Error > 0 means line is on the right.
     * w > 0 is CCW, therefore steering correction is negative.
     */
    float w =
        -(LINE_KP * error +
          LINE_KD * g_d_filtered);

    w = line_clampf(w, -LINE_W_MAX, LINE_W_MAX);

    g_prev_error = error;
    g_prev_pd_timestamp_us = timestamp_us;
    g_pd_has_sample = (timestamp_us > 0);
    g_line_work.d_filtered = g_d_filtered;

    return w;
}

static float line_schedule_vx(float w)
{
    const float turn_ratio =
        line_clampf(
            fabsf(w) / LINE_W_MAX,
            0.0f,
            1.0f);

    /*
     * Small steering corrections should NOT change vehicle speed.
     */
    if (turn_ratio <= LINE_V_STRAIGHT_W)
    {
        return LINE_V_MAX;
    }

    /*
     * Normal cornering region.
     *
     * Smoothly reduce:
     *
     * V_MAX -> V_MIN
     *
     * using smoothstep rather than a linear mapping.
     */
    if (turn_ratio <= LINE_V_SLOW_W)
    {
        float t =
            (turn_ratio - LINE_V_STRAIGHT_W) /
            (LINE_V_SLOW_W - LINE_V_STRAIGHT_W);

        t = line_clampf(t, 0.0f, 1.0f);

        /*
         * smoothstep:
         *     3t^2 - 2t^3
         *
         * Zero slope at both ends, which avoids abrupt
         * speed changes.
         */
        const float smooth =
            t * t * (3.0f - 2.0f * t);

        return
            LINE_V_MAX -
            (LINE_V_MAX - LINE_V_MIN) *
            smooth;
    }

    /*
     * Transition from minimum forward speed to zero.
     */
    if (turn_ratio <= LINE_REVERSE_START)
    {
        const float t =
            (turn_ratio - LINE_V_SLOW_W) /
            (LINE_REVERSE_START - LINE_V_SLOW_W);

        return
            LINE_V_MIN *
            (1.0f - line_clampf(t, 0.0f, 1.0f));
    }

    /*
     * Extremely large steering demand:
     * gradually enter slight reverse motion.
     */
    const float t =
        (turn_ratio - LINE_REVERSE_START) /
        (1.0f - LINE_REVERSE_START);

    return
        line_clampf(t, 0.0f, 1.0f) *
        LINE_REVERSE_V;
}


static void line_update_last_turn(float error)
{
    if (error > 0.05f)
    {
        g_last_turn_sign = -1;
    }
    else if (error < -0.05f)
    {
        g_last_turn_sign = +1;
    }
}

static void line_copy_vision_status(
    const line_vision_data_t *vision,
    bool line_found,
    float error,
    bool new_measurement,
    float vision_dt_s,
    bool avoid_done)
{
    g_line_work.line_found = line_found;
    g_line_work.vision_valid = (vision != NULL) && vision->valid;
    g_line_work.error = error;

    g_line_work.control_sequence = g_control_sequence;
    g_line_work.measurement_new = new_measurement;
    g_line_work.avoid_done = avoid_done;

    if (vision != NULL)
    {
        g_line_work.vision_sequence = vision->sequence;
        g_line_work.vision_center_x = vision->line_center_x;
        g_line_work.vision_black_ratio = vision->black_ratio;
        g_line_work.vision_roi_center_x = vision->roi_center_x;
        g_line_work.vision_roi_half_width = vision->roi_half_width;
        g_line_work.vision_black_pixels = vision->black_pixels;
        g_line_work.vision_roi_pixels = vision->roi_pixels;
        g_line_work.vision_frame_timestamp_us = vision->frame_timestamp_us;
        g_line_work.vision_update_timestamp_us = vision->update_timestamp_us;
        g_line_work.ir_sample_count = vision->sequence;
    }
    else
    {
        g_line_work.vision_sequence = 0U;
        g_line_work.vision_center_x = 0.0f;
        g_line_work.vision_black_ratio = 0.0f;
        g_line_work.vision_roi_center_x = 0.0f;
        g_line_work.vision_roi_half_width = 0.0f;
        g_line_work.vision_black_pixels = 0U;
        g_line_work.vision_roi_pixels = 0U;
        g_line_work.vision_frame_timestamp_us = 0;
        g_line_work.vision_update_timestamp_us = 0;
        g_line_work.ir_sample_count = 0U;
    }

    g_line_work.vision_dt_s = vision_dt_s;

    /* Compatibility field: no fabricated infrared pattern. */
    g_line_work.pattern = 0U;
}

/* ============================================================
 * 10. State handlers -- same control policy as infrared version
 * ============================================================ */
static void line_state_search_start(
    bool line_found,
    float error,
    bool new_measurement,
    int64_t measurement_timestamp_us)
{
    if (line_found)
    {
        line_set_command(0.0f, 0.0f);

        if (!new_measurement)
        {
            return;
        }

        ++g_start_confirm_count;

        if (g_start_confirm_count >= LINE_START_CONFIRM_COUNT)
        {
            g_start_confirm_count = 0U;

            line_reset_pd(error, measurement_timestamp_us);
            line_update_last_turn(error);

            /* P-only command immediately; do not wait another camera frame. */
            const float w =
                line_calculate_w(error, measurement_timestamp_us);
            const float vx = line_schedule_vx(w);

            line_set_command(vx, w);
            g_line_work.state = LINE_STATE_FOLLOW;
        }

        return;
    }

    if (new_measurement)
    {
        g_start_confirm_count = 0U;
    }

    line_set_command(0.0f, LINE_START_SEARCH_W);
}

static void line_finish_now(void)
{
    line_set_command(0.0f, 0.0f);
    g_line_work.state = LINE_STATE_FINISHED;

    taskENTER_CRITICAL(&g_line_lock);
    g_line_run_request = false;
    taskEXIT_CRITICAL(&g_line_lock);
}

static void line_state_follow(
    bool line_found,
    float error,
    bool finish_detected,
    bool new_measurement,
    int64_t measurement_timestamp_us)
{
    g_line_work.follow_elapsed_ms += LINE_CONTROL_PERIOD_MS;

    /* A finish confirmation is a camera-frame event, not a 50 Hz tick event. */
    if (new_measurement &&
        s_avoid_done &&
        (g_line_work.follow_elapsed_ms >= LINE_FINISH_ENABLE_DELAY_MS) &&
        finish_detected)
    {
        g_finish_confirm_count = 1U;

        if (g_finish_confirm_count >= LINE_FINISH_CONFIRM_COUNT)
        {
            line_finish_now();
        }
        else
        {
            g_line_work.state = LINE_STATE_FINISH_CONFIRM;
            line_set_command(LINE_FINISH_V, 0.0f);
        }

        return;
    }

    /* Stale vision must still be able to enter LOST even without a new seq. */
    if (!line_found)
    {
        g_line_work.lost_elapsed_ms = 0U;
        g_line_work.state = LINE_STATE_LOST;
        return;
    }

    /* 50 Hz actuator clock simply holds the latest camera-derived command. */
    if (!new_measurement)
    {
        line_set_command(
            g_line_work.target_vx,
            g_line_work.target_w);
        return;
    }

    line_update_last_turn(error);

    const float w =
        line_calculate_w(error, measurement_timestamp_us);
    const float vx = line_schedule_vx(w);

    line_set_command(vx, w);
}

static void line_state_lost(
    bool line_found,
    float error,
    bool new_measurement,
    int64_t measurement_timestamp_us)
{
    g_line_work.lost_elapsed_ms += LINE_CONTROL_PERIOD_MS;

    /* Reacquisition requires an independent camera observation. */
    if (line_found && new_measurement)
    {
        line_reset_pd(error, measurement_timestamp_us);
        line_update_last_turn(error);

        const float w =
            line_calculate_w(error, measurement_timestamp_us);
        const float vx = line_schedule_vx(w);

        line_set_command(vx, w);

        g_line_work.lost_elapsed_ms = 0U;
        g_line_work.state = LINE_STATE_FOLLOW;
        return;
    }

    if (g_line_work.lost_elapsed_ms >= LINE_LOST_TIMEOUT_MS)
    {
        line_set_command(0.0f, 0.0f);
        g_line_work.state = LINE_STATE_STOPPED;

        taskENTER_CRITICAL(&g_line_lock);
        g_line_run_request = false;
        taskEXIT_CRITICAL(&g_line_lock);
        return;
    }

    if (g_line_work.lost_elapsed_ms < LINE_LOST_FORWARD_MS)
    {
        line_set_command(
            LINE_LOST_FORWARD_V,
            (float)g_last_turn_sign * LINE_LOST_TURN_W);
        return;
    }

    line_set_command(
        0.0f,
        (float)g_last_turn_sign * LINE_LOST_SEARCH_W);
}

static void line_state_finish_confirm(
    bool line_found,
    float error,
    bool finish_detected,
    bool new_measurement,
    bool vision_valid,
    int64_t measurement_timestamp_us)
{
    /* If the camera stream becomes stale, finish confirmation is cancelled. */
    if (!vision_valid)
    {
        g_finish_confirm_count = 0U;
        g_line_work.lost_elapsed_ms = 0U;
        g_line_work.state = LINE_STATE_LOST;
        return;
    }

    if (!new_measurement)
    {
        line_set_command(LINE_FINISH_V, 0.0f);
        return;
    }

    if ((!s_avoid_done) || (!finish_detected))
    {
        g_finish_confirm_count = 0U;

        if (line_found)
        {
            line_reset_pd(error, measurement_timestamp_us);
            line_update_last_turn(error);

            const float w =
                line_calculate_w(error, measurement_timestamp_us);
            const float vx = line_schedule_vx(w);

            line_set_command(vx, w);
            g_line_work.state = LINE_STATE_FOLLOW;
        }
        else
        {
            g_line_work.lost_elapsed_ms = 0U;
            g_line_work.state = LINE_STATE_LOST;
        }

        return;
    }

    ++g_finish_confirm_count;
    line_set_command(LINE_FINISH_V, 0.0f);

    if (g_finish_confirm_count >= LINE_FINISH_CONFIRM_COUNT)
    {
        line_finish_now();
    }
}

/* ============================================================
 * 11. Debug
 * ============================================================ */
static void line_debug_print(void)
{
#if LINE_DEBUG_ENABLE
    g_debug_elapsed_ms += LINE_CONTROL_PERIOD_MS;

    if (g_debug_elapsed_ms < LINE_DEBUG_PERIOD_MS)
    {
        return;
    }

    g_debug_elapsed_ms = 0U;
    
    /*
    ESP_LOGI(
        LINE_TAG,
        "STATE=%s VALID=%d FOUND=%d NEW=%d SEQ=%u CX=%.2f BLACK=%.3f E=%+.3f D=%+.2f VX=%.3f W=%+.3f VDT=%.3f AVOID_DONE=%d",
        line_state_name(g_line_work.state),
        g_line_work.vision_valid,
        g_line_work.line_found,
        g_line_work.measurement_new,
        (unsigned)g_line_work.vision_sequence,
        g_line_work.vision_center_x,
        g_line_work.vision_black_ratio,
        g_line_work.error,
        g_line_work.d_filtered,
        g_line_work.target_vx,
        g_line_work.target_w,
        g_line_work.vision_dt_s,
        g_line_work.avoid_done);
        */
#endif
}

/* ============================================================
 * 12. Control task
 * ============================================================ */
static void line_tracker_update(void)
{
    line_vision_data_t vision;
    line_vision_get_data(&vision);

    ++g_control_sequence;

    bool new_measurement = false;
    float vision_dt_s = g_line_work.vision_dt_s;

    if (vision.ready &&
        ((!g_have_vision_sequence) ||
         (vision.sequence != g_last_vision_sequence)))
    {
        new_measurement = true;

        if (g_have_vision_sequence &&
            (vision.frame_timestamp_us > 0) &&
            (g_last_measurement_timestamp_us > 0) &&
            (vision.frame_timestamp_us > g_last_measurement_timestamp_us))
        {
            vision_dt_s =
                (float)(vision.frame_timestamp_us -
                        g_last_measurement_timestamp_us) * 1.0e-6f;
        }
        else
        {
            vision_dt_s = 0.0f;
        }

        g_have_vision_sequence = true;
        g_last_vision_sequence = vision.sequence;
        g_last_measurement_timestamp_us = vision.frame_timestamp_us;
    }

    bool line_found = false;

    const float error =
        line_visual_to_error(
            &vision,
            &line_found);

    const bool finish_detected =
        line_visual_is_finish(&vision);

    bool run_request;
    bool reset_request;
    bool avoid_done;

    taskENTER_CRITICAL(&g_line_lock);
    run_request = g_line_run_request;
    reset_request = g_line_reset_request;
    avoid_done = s_avoid_done;
    g_line_reset_request = false;
    taskEXIT_CRITICAL(&g_line_lock);

    line_copy_vision_status(
        &vision,
        line_found,
        error,
        new_measurement,
        vision_dt_s,
        avoid_done);

    if (!run_request)
    {
        g_line_work.state = LINE_STATE_STOPPED;
        g_line_work.target_vx = 0.0f;
        g_line_work.target_w = 0.0f;

        line_debug_print();
        line_publish_status();
        return;
    }

    if (reset_request ||
        (g_line_work.state == LINE_STATE_STOPPED))
    {
        line_reset_algorithm();

        /* Preserve the current observation and stream clock after reset. */
        line_copy_vision_status(
            &vision,
            line_found,
            error,
            new_measurement,
            vision_dt_s,
            avoid_done);
    }

    switch (g_line_work.state)
    {
        case LINE_STATE_SEARCH_START:
            line_state_search_start(
                line_found,
                error,
                new_measurement,
                vision.frame_timestamp_us);
            break;

        case LINE_STATE_FOLLOW:
            line_state_follow(
                line_found,
                error,
                finish_detected,
                new_measurement,
                vision.frame_timestamp_us);
            break;

        case LINE_STATE_LOST:
            line_state_lost(
                line_found,
                error,
                new_measurement,
                vision.frame_timestamp_us);
            break;

        case LINE_STATE_FINISH_CONFIRM:
            line_state_finish_confirm(
                line_found,
                error,
                finish_detected,
                new_measurement,
                vision.valid,
                vision.frame_timestamp_us);
            break;

        case LINE_STATE_FINISHED:
            line_set_command(0.0f, 0.0f);
            break;

        case LINE_STATE_STOPPED:
        default:
            line_set_command(0.0f, 0.0f);
            break;
    }

    line_debug_print();
    line_publish_status();
}

static void line_tracker_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LINE_CONTROL_PERIOD_MS);

    while (1)
    {
        line_tracker_update();
        vTaskDelayUntil(&last_wake, period);
    }
}

/* ============================================================
 * 13. Public API
 * ============================================================ */
esp_err_t line_tracker_init(void)
{
    if (g_line_initialized)
    {
        return ESP_OK;
    }

    esp_err_t ret = motor_control_init();

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = line_vision_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            LINE_TAG,
            "line_vision_init failed: %s",
            esp_err_to_name(ret));
        return ret;
    }

    line_reset_algorithm();
    g_line_work.state = LINE_STATE_STOPPED;
    line_publish_status();

    BaseType_t task_result =
        xTaskCreate(
            line_tracker_task,
            "line_tracker",
            LINE_TASK_STACK_SIZE,
            NULL,
            LINE_TASK_PRIORITY,
            &g_line_task_handle);

    if (task_result != pdPASS)
    {
        g_line_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    g_line_initialized = true;

    ESP_LOGI(
        LINE_TAG,
        "Initialized: period=%ums Kp=%.3f Kd=%.4f V=[%.2f,%.2f] Wmax=%.2f finish_ratio=%.2f",
        (unsigned)LINE_CONTROL_PERIOD_MS,
        LINE_KP,
        LINE_KD,
        LINE_V_MIN,
        LINE_V_MAX,
        LINE_W_MAX,
        LINE_FINISH_BLACK_RATIO);

    return ESP_OK;
}

void line_tracker_start(void)
{
    if (!g_line_initialized)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);
    g_line_run_request = true;
    g_line_reset_request = true;
    taskEXIT_CRITICAL(&g_line_lock);
}

void line_tracker_stop(void)
{
    if (!g_line_initialized)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);
    g_line_run_request = false;
    g_line_reset_request = false;
    taskEXIT_CRITICAL(&g_line_lock);

    motor_stop();
}

void line_tracker_reset(void)
{
    if (!g_line_initialized)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);
    g_line_run_request = true;
    g_line_reset_request = true;
    taskEXIT_CRITICAL(&g_line_lock);
}

void line_tracker_get_status(line_tracker_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);
    *status = g_line_status;
    taskEXIT_CRITICAL(&g_line_lock);
}

#endif /* LINE_TRACKER_IMPLEMENTATION */
#endif /* LINE_TRACKER_H */
