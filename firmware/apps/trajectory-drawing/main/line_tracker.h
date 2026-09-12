#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Include motor first so infrared_sensor.h can perform its
 * optional compile-time pin-conflict checks.
 */
#include "motor_control.h"
#include "infrared_sensor.h"

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
#define LINE_TASK_STACK_SIZE            3072
#endif

#ifndef LINE_TASK_PRIORITY
#define LINE_TASK_PRIORITY              6
#endif


/* ============================================================
 * 2. FOLLOW parameters
 *
 * IMPORTANT:
 * Set_motor(vx, vy, w) uses normalized commands [-1, 1].
 * Therefore all velocity commands below are normalized too.
 * ============================================================ */

#ifndef LINE_KP
#define LINE_KP                         0.65f
#endif

#ifndef LINE_KD
#define LINE_KD                         0.055f
#endif

/*
 * Low-pass coefficient for D only:
 *
 * D_filtered += alpha * (D_raw - D_filtered)
 */
#ifndef LINE_D_ALPHA
#define LINE_D_ALPHA                    0.35f
#endif

#ifndef LINE_W_MAX
#define LINE_W_MAX                      1.0f
#endif

#ifndef LINE_V_MAX
#define LINE_V_MAX                      0.40f
#endif

#ifndef LINE_V_MIN
#define LINE_V_MIN                      0.08f
#endif

#define LINE_REVERSE_START     0.80f
#define LINE_REVERSE_V        -0.05f


/* ============================================================
 * 3. SEARCH / LOST / FINISH
 * ============================================================ */

#ifndef LINE_START_SEARCH_W
#define LINE_START_SEARCH_W             0.40f
#endif

#ifndef LINE_START_CONFIRM_COUNT
#define LINE_START_CONFIRM_COUNT        3U
#endif

/*
 * LOST phase 1:
 * keep moving slowly while turning toward the last seen side.
 */
#ifndef LINE_LOST_FORWARD_V
#define LINE_LOST_FORWARD_V             -0.03f
#endif

#ifndef LINE_LOST_TURN_W
#define LINE_LOST_TURN_W                0.45f
#endif

#ifndef LINE_LOST_FORWARD_MS
#define LINE_LOST_FORWARD_MS            100U
#endif

/*
 * LOST phase 2:
 * stop translation and rotate toward the same side.
 */
#ifndef LINE_LOST_SEARCH_W
#define LINE_LOST_SEARCH_W              0.45f
#endif

#ifndef LINE_LOST_TIMEOUT_MS
#define LINE_LOST_TIMEOUT_MS            2500U
#endif

#ifndef LINE_FINISH_PATTERN
#define LINE_FINISH_PATTERN             0x0FU
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
 * Finish detection:
 *
 * Original:
 *     0x0F -> L2 L1 R1 R2 all detected
 *
 * Relaxed:
 *     L1 or R1 detected is enough
 *
 * Pattern bits:
 *     L2 L1 R1 R2
 *      3  2  1  0
 */
#ifndef LINE_FINISH_CENTER_ENABLE
#define LINE_FINISH_CENTER_ENABLE 1
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
    line_state_t state;

    uint8_t pattern;
    bool line_found;

    float error;
    float d_filtered;

    /*
     * Normalized chassis commands sent to motor_control:
     * vx, w ∈ [-1, 1], vy is always 0 in this first version.
     */
    float target_vx;
    float target_w;

    uint32_t lost_elapsed_ms;
    uint32_t follow_elapsed_ms;
    uint32_t ir_sample_count;

} line_tracker_status_t;

void line_tracker_enable(bool enable);
bool line_tracker_is_enabled(void);

esp_err_t line_tracker_init(void);

void line_tracker_start(void);
void line_tracker_stop(void);
void line_tracker_reset(void);

void line_tracker_get_status(
    line_tracker_status_t *status);

/*
 * 设置是否已经完成过一次避障。
 *
 * obstacle_avoid 在真正完成避障后调用：
 * line_tracker_set_avoid_done(true);
 *
 * 该状态保持为 true，直到程序重新初始化。
 */
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

/*
 * Public functions only update requests.
 * The line task owns the algorithm state.
 */
static bool g_line_run_request = false;
static bool g_line_reset_request = false;

static portMUX_TYPE g_line_lock =
    portMUX_INITIALIZER_UNLOCKED;

/* Enable control to adjust obstacle avoid */
static bool s_line_enabled = true;

/*
 * 是否已经完成过一次避障。
 *
 * false：
 *     0000 不能判定为终点。
 *
 * true：
 *     0000 才允许进入 FINISH_CONFIRM。
 */
static bool s_avoid_done = false;

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

    ESP_LOGI(
        LINE_TAG,
        "avoid_done=%d",
        done
    );
}

/* ============================================================
 * 6. Internal algorithm state
 * ============================================================ */
static float g_prev_error = 0.0f;
static float g_d_filtered = 0.0f;

/*
 * Sign of the last valid steering direction:
 *
 * +1 -> w > 0
 * -1 -> w < 0
 */
static int8_t g_last_turn_sign = 1;

static uint32_t g_start_confirm_count = 0;
static uint32_t g_finish_confirm_count = 0;
static uint32_t g_debug_elapsed_ms = 0;

/* ============================================================
 * 7. Helpers
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

static const char *line_state_name(
    line_state_t state)
{
    switch (state)
    {
        case LINE_STATE_SEARCH_START:
            return "SEARCH";

        case LINE_STATE_FOLLOW:
            return "FOLLOW";

        case LINE_STATE_LOST:
            return "LOST";

        case LINE_STATE_FINISH_CONFIRM:
            return "FINISH_CHECK";

        case LINE_STATE_FINISHED:
            return "FINISHED";

        case LINE_STATE_STOPPED:
            return "STOPPED";

        default:
            return "UNKNOWN";
    }
}

/*
 * Binary weighted centroid.
 *
 * Sensor positions:
 *
 *      L2    L1    R1    R2
 *      -3    -1    +1    +3
 *
 * Result is normalized to [-1, +1].
 *
 * 1000 -> -1.000
 * 1100 -> -0.667
 * 0100 -> -0.267
 * 0110 ->  0.000
 * 0010 -> +0.267
 * 0011 -> +0.667
 * 0001 -> +1.000
 *
 * 0000 = 全黑，当前作为特殊终点候选状态。
 * 1111 = 全白，作为 LOST。
 */
static float line_pattern_to_error(
    uint8_t pattern,
    bool *line_found)
{
    const uint8_t l2 =
        (pattern >> 3) & 1U;

    const uint8_t l1 =
        (pattern >> 2) & 1U;

    const uint8_t r1 =
        (pattern >> 1) & 1U;

    const uint8_t r2 =
        (pattern >> 0) & 1U;

    const uint8_t count =
        l2 + l1 + r1 + r2;

    if (count == 0U)
    {
        if (line_found != NULL)
        {
            *line_found = false;
        }

        return 0.0f;
    }

    if (line_found != NULL)
    {
        *line_found = true;
    }

    const float weighted_sum =
        -3.0f * (float)l2 +
        -0.8f * (float)l1 +
        +0.8f * (float)r1 +
        +3.0f * (float)r2;

    return
        weighted_sum /
        ((float)count * 3.0f);
}

static bool line_is_finish_pattern(uint8_t pattern)
{
#if LINE_FINISH_CENTER_ENABLE

    /*
     * L1 or R1 detected
     *
     * bit2 = L1
     * bit1 = R1
     */
    return (pattern & 0x06U) != 0U;

#else

    return pattern == LINE_FINISH_PATTERN;

#endif
}

static void line_reset_pd(
    float current_error)
{
    g_prev_error =
        current_error;

    g_d_filtered =
        0.0f;

    g_line_work.d_filtered =
        0.0f;
}

static void line_reset_algorithm(void)
{
    g_line_work.state =
        LINE_STATE_SEARCH_START;

    g_line_work.pattern = 0U;
    g_line_work.line_found = false;

    g_line_work.error = 0.0f;
    g_line_work.d_filtered = 0.0f;

    g_line_work.target_vx = 0.0f;
    g_line_work.target_w = 0.0f;

    g_line_work.lost_elapsed_ms = 0U;
    g_line_work.follow_elapsed_ms = 0U;

    g_prev_error = 0.0f;
    g_d_filtered = 0.0f;

    g_last_turn_sign = 1;

    g_start_confirm_count = 0U;
    g_finish_confirm_count = 0U;
    g_debug_elapsed_ms = 0U;
}

static void line_set_command(
    float vx,
    float w)
{
    if (!s_line_enabled) return;

    vx =
        line_clampf(
            vx,
            -1.0f,
            1.0f);

    w =
        line_clampf(
            w,
            -1.0f,
            1.0f);

    g_line_work.target_vx = vx;
    g_line_work.target_w = w;

    Set_motor(
        vx,
        0.0f,
        w);
}

static void line_publish_status(void)
{
    taskENTER_CRITICAL(&g_line_lock);

    g_line_status =
        g_line_work;

    taskEXIT_CRITICAL(&g_line_lock);
}

/* ============================================================
 * 8. FOLLOW controller
 * ============================================================ */
static float line_calculate_w(
    float error)
{
    const float d_raw =
        (error - g_prev_error) /
        LINE_CONTROL_DT;

    const float alpha =
        line_clampf(
            LINE_D_ALPHA,
            0.0f,
            1.0f);

    g_d_filtered +=
        alpha *
        (d_raw -
         g_d_filtered);

    /*
     * Error > 0 means the line is on the right.
     * With motor_control convention w > 0 = CCW,
     * the robot therefore needs negative w.
     */
    float w =
        -(LINE_KP * error +
          LINE_KD * g_d_filtered);

    w =
        line_clampf(
            w,
            -LINE_W_MAX,
            LINE_W_MAX);

    g_prev_error =
        error;

    g_line_work.d_filtered =
        g_d_filtered;

    return w;
}

static float line_schedule_vx(float w)
{
    const float turn_ratio =
        line_clampf(
            fabsf(w) / LINE_W_MAX,
            0.0f,
            1.0f);

    if (turn_ratio <= LINE_REVERSE_START)
    {
        const float ratio =
            turn_ratio /
            LINE_REVERSE_START;

        return
            LINE_V_MAX -
            (LINE_V_MAX - 0.0f) *
            ratio;
    }

    const float ratio =
        (turn_ratio - LINE_REVERSE_START) /
        (1.0f - LINE_REVERSE_START);

    return
        ratio *
        LINE_REVERSE_V;
}

static void line_update_last_turn(
    float error)
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

/* ============================================================
 * 9. State handlers
 * ============================================================ */
static void line_state_search_start(
    bool line_found,
    float error)
{
    if (line_found)
    {
        line_set_command(
            0.0f,
            0.0f);

        g_start_confirm_count++;

        if (g_start_confirm_count >=
            LINE_START_CONFIRM_COUNT)
        {
            g_start_confirm_count = 0U;

            line_reset_pd(error);
            line_update_last_turn(error);

            g_line_work.state =
                LINE_STATE_FOLLOW;
        }

        return;
    }

    g_start_confirm_count = 0U;

    line_set_command(
        0.0f,
        LINE_START_SEARCH_W);
}

static void line_state_follow(
    uint8_t pattern,
    bool line_found,
    float error)
{
    g_line_work.follow_elapsed_ms +=
        LINE_CONTROL_PERIOD_MS;

    /*
     * 只有完成过一次避障以后，
     * 0000 才允许作为终点信号。
     *
     * 避障之前即使出现 0000，
     * 也不会进入 FINISH_CONFIRM。
     */
    if (s_avoid_done &&
        (g_line_work.follow_elapsed_ms >=
         LINE_FINISH_ENABLE_DELAY_MS) &&
        line_is_finish_pattern(pattern))
    {
        g_finish_confirm_count = 1U;

        g_line_work.state =
            LINE_STATE_FINISH_CONFIRM;

        line_set_command(
            LINE_FINISH_V,
            0.0f);

        return;
    }

    if (!line_found)
    {
        g_line_work.lost_elapsed_ms = 0U;

        g_line_work.state =
            LINE_STATE_LOST;

        return;
    }

    line_update_last_turn(error);

    const float w =
        line_calculate_w(error);

    const float vx =
        line_schedule_vx(w);

    line_set_command(
        vx,
        w);
}

static void line_state_lost(
    bool line_found,
    float error)
{
    g_line_work.lost_elapsed_ms +=
        LINE_CONTROL_PERIOD_MS;

    if (line_found)
    {
        line_reset_pd(error);
        line_update_last_turn(error);

        g_line_work.lost_elapsed_ms = 0U;

        g_line_work.state =
            LINE_STATE_FOLLOW;

        return;
    }

    if (g_line_work.lost_elapsed_ms >=
        LINE_LOST_TIMEOUT_MS)
    {
        line_set_command(
            0.0f,
            0.0f);

        g_line_work.state =
            LINE_STATE_STOPPED;

        taskENTER_CRITICAL(&g_line_lock);

        g_line_run_request =
            false;

        taskEXIT_CRITICAL(&g_line_lock);

        return;
    }

    /*
     * Phase 1:
     * slow forward motion + strong steering.
     */
    if (g_line_work.lost_elapsed_ms <
        LINE_LOST_FORWARD_MS)
    {
        line_set_command(
            LINE_LOST_FORWARD_V,
            (float)g_last_turn_sign *
            LINE_LOST_TURN_W);

        return;
    }

    /*
     * Phase 2:
     * rotate in place toward the last seen side.
     */
    line_set_command(
        0.0f,
        (float)g_last_turn_sign *
        LINE_LOST_SEARCH_W);
}

static void line_state_finish_confirm(
    uint8_t pattern,
    float error)
{
    /*
     * 避障完成状态必须仍然有效，
     * 并且红外必须持续检测到 0000。
     */
    if (!s_avoid_done ||
        !line_is_finish_pattern(pattern))
    {
        g_finish_confirm_count = 0U;

        line_reset_pd(error);

        g_line_work.state =
            LINE_STATE_FOLLOW;

        return;
    }

    g_finish_confirm_count++;

    line_set_command(
        LINE_FINISH_V,
        0.0f);

    if (g_finish_confirm_count >=
        LINE_FINISH_CONFIRM_COUNT)
    {
        line_set_command(
            0.0f,
            0.0f);

        g_line_work.state =
            LINE_STATE_FINISHED;

        taskENTER_CRITICAL(&g_line_lock);

        g_line_run_request =
            false;

        taskEXIT_CRITICAL(&g_line_lock);
    }
}

/* ============================================================
 * 10. Debug
 * ============================================================ */
static void line_debug_print(void)
{
#if LINE_DEBUG_ENABLE

    g_debug_elapsed_ms +=
        LINE_CONTROL_PERIOD_MS;

    if (g_debug_elapsed_ms <
        LINE_DEBUG_PERIOD_MS)
    {
        return;
    }

    g_debug_elapsed_ms = 0U;

    ESP_LOGI(
        LINE_TAG,
        "STATE=%s PAT=%u%u%u%u E=%+.3f D=%+.2f VX=%.3f W=%+.3f AVOID_DONE=%d",
        line_state_name(
            g_line_work.state),

        (g_line_work.pattern >> 3) & 1U,
        (g_line_work.pattern >> 2) & 1U,
        (g_line_work.pattern >> 1) & 1U,
        (g_line_work.pattern >> 0) & 1U,

        g_line_work.error,
        g_line_work.d_filtered,
        g_line_work.target_vx,
        g_line_work.target_w,
        s_avoid_done);

#endif
}

/* ============================================================
 * 11. Control task
 * ============================================================ */
static void line_tracker_update(void)
{
    infrared_sensor_data_t ir;

    infrared_sensor_get_data(
        &ir);

    bool line_found = false;

    const float error =
        line_pattern_to_error(
            ir.pattern,
            &line_found);

    g_line_work.pattern =
        ir.pattern;

    g_line_work.line_found =
        line_found;

    g_line_work.error =
        error;

    g_line_work.ir_sample_count =
        ir.sample_count;

    bool run_request;
    bool reset_request;

    taskENTER_CRITICAL(&g_line_lock);

    run_request =
        g_line_run_request;

    reset_request =
        g_line_reset_request;

    g_line_reset_request =
        false;

    taskEXIT_CRITICAL(&g_line_lock);

    if (!run_request)
    {
        g_line_work.state =
            LINE_STATE_STOPPED;

        g_line_work.target_vx = 0.0f;
        g_line_work.target_w = 0.0f;

        line_debug_print();
        line_publish_status();

        return;
    }

    if (reset_request ||
        (g_line_work.state ==
         LINE_STATE_STOPPED))
    {
        line_reset_algorithm();

        /*
         * Keep the latest sensor snapshot after reset.
         */
        g_line_work.pattern =
            ir.pattern;

        g_line_work.line_found =
            line_found;

        g_line_work.error =
            error;

        g_line_work.ir_sample_count =
            ir.sample_count;
    }

    switch (g_line_work.state)
    {
        case LINE_STATE_SEARCH_START:

            line_state_search_start(
                line_found,
                error);

            break;

        case LINE_STATE_FOLLOW:

            line_state_follow(
                ir.pattern,
                line_found,
                error);

            break;

        case LINE_STATE_LOST:

            line_state_lost(
                line_found,
                error);

            break;

        case LINE_STATE_FINISH_CONFIRM:

            line_state_finish_confirm(
                ir.pattern,
                error);

            break;

        case LINE_STATE_FINISHED:

            line_set_command(
                0.0f,
                0.0f);

            break;

        case LINE_STATE_STOPPED:
        default:

            line_set_command(
                0.0f,
                0.0f);

            break;
    }

    line_debug_print();
    line_publish_status();
}

static void line_tracker_task(void *arg)
{
    (void)arg;

    TickType_t last_wake =
        xTaskGetTickCount();

    const TickType_t period =
        pdMS_TO_TICKS(
            LINE_CONTROL_PERIOD_MS);

    while (1)
    {
        line_tracker_update();

        vTaskDelayUntil(
            &last_wake,
            period);
    }
}

/* ============================================================
 * 12. Public API
 * ============================================================ */
esp_err_t line_tracker_init(void)
{
    if (g_line_initialized)
    {
        return ESP_OK;
    }

    esp_err_t ret =
        motor_control_init();

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        infrared_sensor_init();

    if (ret != ESP_OK)
    {
        return ret;
    }

    line_reset_algorithm();

    g_line_work.state =
        LINE_STATE_STOPPED;

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
        "Initialized: period=%ums Kp=%.3f Kd=%.4f V=[%.2f,%.2f] Wmax=%.2f",
        (unsigned)LINE_CONTROL_PERIOD_MS,
        LINE_KP,
        LINE_KD,
        LINE_V_MIN,
        LINE_V_MAX,
        LINE_W_MAX);

    return ESP_OK;
}

void line_tracker_start(void)
{
    if (!g_line_initialized)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);

    g_line_run_request =
        true;

    g_line_reset_request =
        true;

    taskEXIT_CRITICAL(&g_line_lock);
}

void line_tracker_stop(void)
{
    if (!g_line_initialized)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);

    g_line_run_request =
        false;

    g_line_reset_request =
        false;

    taskEXIT_CRITICAL(&g_line_lock);

    /*
     * Stop immediately; do not wait for the next 20 ms line cycle.
     */
    motor_stop();
}

void line_tracker_reset(void)
{
    if (!g_line_initialized)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);

    g_line_run_request =
        true;

    g_line_reset_request =
        true;

    taskEXIT_CRITICAL(&g_line_lock);
}

void line_tracker_get_status(
    line_tracker_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&g_line_lock);

    *status =
        g_line_status;

    taskEXIT_CRITICAL(&g_line_lock);
}

#endif /* LINE_TRACKER_IMPLEMENTATION */
#endif /* LINE_TRACKER_H */