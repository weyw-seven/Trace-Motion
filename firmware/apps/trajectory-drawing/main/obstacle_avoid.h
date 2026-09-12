#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ultrasonic_sensor.h"
#include "line_tracker.h"
#include "motor_control.h"
#include "chassis_motion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 避障模式
 * ============================================================ */
typedef enum {
    AVOID_MODE_DIRECT = 0,
    AVOID_MODE_VISUAL
} obstacle_avoid_mode_t;

/* ============================================================
 * 2. 避障状态
 * ============================================================ */
typedef enum {
    AVOID_IDLE = 0,
    AVOID_SHIFT_LEFT,
    AVOID_EXTRA_LEFT,
    AVOID_FORWARD,
    AVOID_SHIFT_RIGHT,
    AVOID_TURN_TO_60,
    AVOID_VISUAL_FORWARD,
    AVOID_TURN_BACK,
    AVOID_FINISH
} obstacle_avoid_state_t;

/* ============================================================
 * 3. 初始化 / 任务
 * ============================================================ */
esp_err_t obstacle_avoid_init(void);
esp_err_t obstacle_avoid_start(void);
void obstacle_avoid_update(void);

/* ============================================================
 * 4. 模式接口
 * ============================================================ */
void obstacle_avoid_set_mode(obstacle_avoid_mode_t mode);
obstacle_avoid_mode_t obstacle_avoid_get_mode(void);

/* ============================================================
 * 5. 触发与状态
 * ============================================================ */
void obstacle_avoid_trigger(void);
bool obstacle_avoid_is_active(void);
obstacle_avoid_state_t obstacle_avoid_get_state(void);
void obstacle_avoid_set_line_found(bool found);
void obstacle_avoid_log_state(void);

/* ============================================================
 * 6. 超声波距离读取
 * ============================================================ */

/**
 * @brief 获取当前超声波测量距离
 * @return 当前距离，单位 cm；数据无效时返回负值
 */
float obstacle_avoid_get_distance_cm(void);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 *
 * 只能在一个 .c 文件中定义：
 * #define OBSTACLE_AVOID_IMPLEMENTATION
 * #include "obstacle_avoid.h"
 * ============================================================ */
#ifdef OBSTACLE_AVOID_IMPLEMENTATION

/* ============================================================
 * 6. 参数
 * ============================================================ */
#define AVOID_EXTRA_LEFT_DISTANCE_MM      115.0f
#define AVOID_EXTRA_LEFT_SPEED_MM_S       500.0f
#define AVOID_EXTRA_LEFT_SPEED_VY         0.08f
#define AVOID_SHIFT_LEFT_INVALID_SPEED_VY 0.04f
#define AVOID_SHIFT_LEFT_INVALID_HOLD_COUNT 3U
#define AVOID_FORWARD_DISTANCE_MM         350.0f
#define AVOID_FORWARD_SPEED_MM_S          500.0f
#define AVOID_SHIFT_RIGHT_DISTANCE_MM     500.0f
#define AVOID_SHIFT_RIGHT_SPEED_MM_S      500.0f

#define AVOID_OBSTACLE_CONFIRM_COUNT       3U
#define AVOID_SEARCH_TIMEOUT_MS            5000U
#define AVOID_CLEAR_CONFIRM_COUNT          3U
#define AVOID_SENSOR_EVAL_PERIOD_MS        60U
#define AVOID_SENSOR_INVALID_ABORT_COUNT   5U
#define AVOID_FORWARD_OBSTACLE_CONFIRM_COUNT 2U
#define AVOID_RIGHT_LINE_IGNORE_MS         100U
#define AVOID_RIGHT_LINE_CONFIRM_COUNT     3U
#define AVOID_UPDATE_PERIOD_MS             10U
#define AVOID_TASK_STACK_SIZE              4096U
#define AVOID_TASK_PRIORITY                 6U
#define AVOID_TAG                           "AVOID"

/* ============================================================
 * 7. 内部状态
 * ============================================================ */
static obstacle_avoid_state_t g_state = AVOID_IDLE;
static obstacle_avoid_mode_t g_mode = AVOID_MODE_DIRECT;
static bool g_active = false;
static bool g_line_found = false;
static TickType_t g_state_start_tick = 0;
static uint8_t g_clear_count = 0;
static uint8_t g_obstacle_confirm_count = 0;
static uint8_t g_sensor_invalid_count = 0;
static uint8_t g_forward_obstacle_count = 0;
static uint8_t g_line_confirm_count = 0;
static uint32_t g_last_line_sample_count = 0U;
static TickType_t g_last_sensor_eval_tick = 0;
static bool g_sensor_eval_initialized = false;
static TaskHandle_t g_avoid_task_handle = NULL;
static bool g_motion_started = false;
static portMUX_TYPE g_avoid_lock = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================
 * 8. 内部重置
 * ============================================================ */
static void obstacle_avoid_reset_internal(void)
{
    taskENTER_CRITICAL(&g_avoid_lock);
    g_state = AVOID_IDLE;
    g_active = false;
    g_line_found = false;
    g_clear_count = 0;
    g_state_start_tick = 0;
    g_obstacle_confirm_count = 0;
    g_sensor_invalid_count = 0;
    g_forward_obstacle_count = 0;
    g_line_confirm_count = 0;
    g_last_line_sample_count = 0U;
    g_last_sensor_eval_tick = 0;
    g_sensor_eval_initialized = false;
    g_motion_started = false;
    taskEXIT_CRITICAL(&g_avoid_lock);
}

/* ============================================================
 * 9. 状态切换工具
 * ============================================================ */
static void obstacle_avoid_enter(obstacle_avoid_state_t state)
{
    taskENTER_CRITICAL(&g_avoid_lock);
    g_state = state;
    g_state_start_tick = xTaskGetTickCount();
    g_clear_count = 0;
    g_sensor_invalid_count = 0;
    g_forward_obstacle_count = 0;
    g_line_confirm_count = 0;
    g_motion_started = false;
    taskEXIT_CRITICAL(&g_avoid_lock);

    ESP_LOGI(AVOID_TAG, "STATE -> %d", (int)state);
}

static uint32_t obstacle_avoid_elapsed_ms(void)
{
    return (uint32_t)((xTaskGetTickCount() - g_state_start_tick) *
                      portTICK_PERIOD_MS);
}

/* ============================================================
 * 10. 结束避障并恢复巡线
 * ============================================================ */
static void obstacle_avoid_finish_and_return(void)
{
    /* 所有正常退出路径都先取消可能残留的底盘运动。 */
    chassis_motion_cancel();
    Set_motor(0.0f, 0.0f, 0.0f);

    /* 巡线复位时仍保持输出关闭，避免短暂写出旧控制量。 */
    line_tracker_reset();
    obstacle_avoid_reset_internal();
    line_tracker_enable(true);

    ESP_LOGI(AVOID_TAG, "Avoid finished -> LINE_TRACK restored");
}

/* 传感器或运动状态异常时停车，并保持巡线关闭，避免继续撞向障碍物。 */
static void obstacle_avoid_abort_safe(const char *reason)
{
    chassis_motion_cancel();
    Set_motor(0.0f, 0.0f, 0.0f);
    line_tracker_stop();
    line_tracker_enable(false);
    obstacle_avoid_reset_internal();

    ESP_LOGE(
        AVOID_TAG,
        "Avoid aborted safely: %s",
        (reason != NULL) ? reason : "unknown"
    );
}

/* avoid 状态机保持 10 ms 更新，但超声确认只按实际测量节奏执行。 */
static bool obstacle_avoid_sensor_sample_due(void)
{
    const TickType_t now = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(AVOID_SENSOR_EVAL_PERIOD_MS);

    if (!g_sensor_eval_initialized ||
        (now - g_last_sensor_eval_tick) >= period) {
        g_last_sensor_eval_tick = now;
        g_sensor_eval_initialized = true;
        return true;
    }

    return false;
}

static bool obstacle_avoid_line_override(void)
{
    bool found;

    taskENTER_CRITICAL(&g_avoid_lock);
    found = g_line_found;
    taskEXIT_CRITICAL(&g_avoid_lock);

    return found;
}

/* ============================================================
 * 11. 各状态动作
 * ============================================================ */
static void obstacle_avoid_idle(void)
{
    /* IDLE 时不输出任何命令，由 line_tracker 控制 */
}

static void obstacle_avoid_shift_left(void)
{
    if (obstacle_avoid_elapsed_ms() >= AVOID_SEARCH_TIMEOUT_MS) {
        obstacle_avoid_abort_safe("left shift timeout");
        return;
    }

    const bool sample_due = obstacle_avoid_sensor_sample_due();

    /*
     * 传感器处于失效窗口时，只允许极低速、有限次数地横移。
     * 左移过程中障碍物离开超声波束可能产生一次 RISE_TIMEOUT；
     * 一次失效就立即置零会把正常的短暂丢波放大成“左移不执行”。
     * 连续失效超过窗口后仍然停车，由下面的故障计数负责保护。
     */
    if (!sample_due) {
        if (g_sensor_invalid_count == 0U) {
            Set_motor(0.0f, AVOID_EXTRA_LEFT_SPEED_VY, 0.0f);
        } else if (g_sensor_invalid_count <=
                   AVOID_SHIFT_LEFT_INVALID_HOLD_COUNT) {
            Set_motor(0.0f, AVOID_SHIFT_LEFT_INVALID_SPEED_VY, 0.0f);
        } else {
            Set_motor(0.0f, 0.0f, 0.0f);
        }
        return;
    }

    ultrasonic_data_t data = ultrasonic_sensor_get_data();

    if (!data.valid) {
        g_sensor_invalid_count++;
        g_clear_count = 0;

        ESP_LOGW(
            AVOID_TAG,
            "SHIFT_LEFT: ultrasonic invalid #%u distance=%.1f",
            (unsigned)g_sensor_invalid_count,
            data.distance_cm
        );

        if (g_sensor_invalid_count <=
            AVOID_SHIFT_LEFT_INVALID_HOLD_COUNT) {
            Set_motor(0.0f, AVOID_SHIFT_LEFT_INVALID_SPEED_VY, 0.0f);
        } else {
            Set_motor(0.0f, 0.0f, 0.0f);
        }

        if (g_sensor_invalid_count >= AVOID_SENSOR_INVALID_ABORT_COUNT) {
            obstacle_avoid_abort_safe("ultrasonic invalid during left shift");
        }
        return;
    }

    g_sensor_invalid_count = 0U;

    ESP_LOGI(
        AVOID_TAG,
        "SHIFT_LEFT: ultrasonic valid distance=%.1f obstacle=%d",
        data.distance_cm,
        (int)data.obstacle
    );

    Set_motor(0.0f, AVOID_EXTRA_LEFT_SPEED_VY, 0.0f);

    if (!data.obstacle) {
        g_clear_count++;

        if (g_clear_count >= AVOID_CLEAR_CONFIRM_COUNT) {
            obstacle_avoid_enter(AVOID_EXTRA_LEFT);
        }
    } else {
        g_clear_count = 0;
    }
}

static void obstacle_avoid_extra_left(void)
{
    if (obstacle_avoid_elapsed_ms() >= AVOID_SEARCH_TIMEOUT_MS) {
        obstacle_avoid_abort_safe("extra-left motion timeout");
        return;
    }

    if (!g_motion_started) {
        Set_motor(0.0f, 0.0f, 0.0f);

        /* 非阻塞等待底盘从手动横移切换到里程运动。 */
        if (obstacle_avoid_elapsed_ms() < 20U) {
            return;
        }

        esp_err_t ret = chassis_motion_move_distance(
            +90.0f,
            AVOID_EXTRA_LEFT_DISTANCE_MM,
            AVOID_EXTRA_LEFT_SPEED_MM_S
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                AVOID_TAG,
                "Extra left start failed: %s",
                esp_err_to_name(ret)
            );

            obstacle_avoid_abort_safe("extra-left motion start failed");
            return;
        }

        g_motion_started = true;

        ESP_LOGI(
            AVOID_TAG,
            "Extra left started: %.1f mm",
            AVOID_EXTRA_LEFT_DISTANCE_MM
        );

        return;
    }

    chassis_motion_status_t status;
    chassis_motion_get_status(&status);

    if (!status.busy) {
        if (status.mode == CHASSIS_MOTION_MODE_DONE) {
            ESP_LOGI(AVOID_TAG, "Extra left done");
            obstacle_avoid_enter(AVOID_FORWARD);
        } else {
            ESP_LOGW(
                AVOID_TAG,
                "Extra left not done, mode=%d error=%d",
                status.mode,
                status.error
            );

            obstacle_avoid_abort_safe("extra-left motion failed");
        }
    }
}

static void obstacle_avoid_forward(void)
{
    if (obstacle_avoid_elapsed_ms() >= AVOID_SEARCH_TIMEOUT_MS) {
        obstacle_avoid_abort_safe("forward motion timeout");
        return;
    }

    /* 前进阶段仍保留前方障碍物保护。 */
    if (obstacle_avoid_sensor_sample_due()) {
        const ultrasonic_data_t data = ultrasonic_sensor_get_data();

        if (data.valid && data.obstacle) {
            if (g_forward_obstacle_count <
                AVOID_FORWARD_OBSTACLE_CONFIRM_COUNT) {
                g_forward_obstacle_count++;
            }
        } else if (data.valid) {
            g_forward_obstacle_count = 0U;
        }

        if (g_forward_obstacle_count >=
            AVOID_FORWARD_OBSTACLE_CONFIRM_COUNT) {
            obstacle_avoid_abort_safe("obstacle detected during forward motion");
            return;
        }
    }

    if (!g_motion_started) {
        esp_err_t ret = chassis_motion_move_distance(
            0.0f,
            AVOID_FORWARD_DISTANCE_MM,
            AVOID_FORWARD_SPEED_MM_S
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                AVOID_TAG,
                "Forward start failed: %s",
                esp_err_to_name(ret)
            );

            obstacle_avoid_abort_safe("forward motion start failed");
            return;
        }

        g_motion_started = true;

        ESP_LOGI(
            AVOID_TAG,
            "Forward started: %.1f mm",
            AVOID_FORWARD_DISTANCE_MM
        );

        return;
    }

    chassis_motion_status_t status;
    chassis_motion_get_status(&status);

    if (!status.busy) {
        if (status.mode == CHASSIS_MOTION_MODE_DONE) {
            ESP_LOGI(AVOID_TAG, "Forward done");
            obstacle_avoid_enter(AVOID_SHIFT_RIGHT);
        } else {
            ESP_LOGW(
                AVOID_TAG,
                "Forward not done, mode=%d error=%d",
                status.mode,
                status.error
            );

            obstacle_avoid_abort_safe("forward motion failed");
        }
    }
}

static void obstacle_avoid_shift_right(void)
{
    if (!g_motion_started) {
        esp_err_t ret = chassis_motion_move_distance(
            -90.0f,
            AVOID_SHIFT_RIGHT_DISTANCE_MM,
            AVOID_SHIFT_RIGHT_SPEED_MM_S
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                AVOID_TAG,
                "Shift right start failed: %s",
                esp_err_to_name(ret)
            );

            obstacle_avoid_abort_safe("right-shift motion start failed");
            return;
        }

        g_motion_started = true;

        /* 记录右移起点的红外样本号，忽略旧的 line_found 状态。 */
        line_tracker_status_t initial_line_status;
        line_tracker_get_status(&initial_line_status);
        g_last_line_sample_count = initial_line_status.ir_sample_count;
        g_line_confirm_count = 0U;

        ESP_LOGI(
            AVOID_TAG,
            "Shift right started: %.1f mm",
            AVOID_SHIFT_RIGHT_DISTANCE_MM
        );

        return;
    }

    /* 右移过程中持续检测黑线 */
    line_tracker_status_t line_status;
    line_tracker_get_status(&line_status);

    const bool fresh_line_sample =
        line_status.ir_sample_count != g_last_line_sample_count;

    if (fresh_line_sample) {
        g_last_line_sample_count = line_status.ir_sample_count;

        if (obstacle_avoid_elapsed_ms() < AVOID_RIGHT_LINE_IGNORE_MS) {
            g_line_confirm_count = 0U;
        } else if (line_status.line_found) {
            if (g_line_confirm_count < AVOID_RIGHT_LINE_CONFIRM_COUNT) {
                g_line_confirm_count++;
            }
        } else {
            g_line_confirm_count = 0U;
        }
    }

    if (obstacle_avoid_elapsed_ms() >= AVOID_RIGHT_LINE_IGNORE_MS &&
        obstacle_avoid_line_override()) {
        g_line_confirm_count = AVOID_RIGHT_LINE_CONFIRM_COUNT;
    }

    if (g_line_confirm_count >= AVOID_RIGHT_LINE_CONFIRM_COUNT) {
        ESP_LOGI(
            AVOID_TAG,
            "Line found during right shift -> cancel motion"
        );

        chassis_motion_cancel();

        obstacle_avoid_enter(AVOID_FINISH);
        return;
    }

    /* 精确移动结束但仍未找到线 */
    if (obstacle_avoid_elapsed_ms() >= AVOID_SEARCH_TIMEOUT_MS) {
        obstacle_avoid_abort_safe("right shift timeout");
        return;
    }

    chassis_motion_status_t status;
    chassis_motion_get_status(&status);

    if (!status.busy) {
        if (status.mode == CHASSIS_MOTION_MODE_DONE) {
            ESP_LOGW(
                AVOID_TAG,
                "Shift right completed, line not found"
            );

            obstacle_avoid_finish_and_return();
        } else {
            obstacle_avoid_abort_safe("right-shift motion failed");
        }
    }
}

/* ============================================================
 * 预留视觉避障状态
 * ============================================================ */
static void obstacle_avoid_turn_to_60(void)
{
    Set_motor(0.0f, 0.0f, 0.0f);
}

static void obstacle_avoid_visual_forward(void)
{
    Set_motor(0.0f, 0.0f, 0.0f);
}

static void obstacle_avoid_turn_back(void)
{
    Set_motor(0.0f, 0.0f, 0.0f);
}

static void obstacle_avoid_finish(void)
{
    /*
     * 只有真正完成一次避障、重新找到黑线并进入 FINISH
     * 状态时，才通知 line_tracker：
     *
     * avoid_done = true
     *
     * 这样异常退出避障不会错误地开放终点判断。
     */
    line_tracker_set_avoid_done(true);

    obstacle_avoid_finish_and_return();
}

/* ============================================================
 * 12. 状态机入口
 * ============================================================ */
void obstacle_avoid_update(void)
{
    if (!obstacle_avoid_is_active()) {
        /* 空闲时自动检测障碍 */
        obstacle_avoid_trigger();
        obstacle_avoid_idle();
        return;
    }

    switch (obstacle_avoid_get_state()) {
        case AVOID_SHIFT_LEFT:
            obstacle_avoid_shift_left();
            break;

        case AVOID_EXTRA_LEFT:
            obstacle_avoid_extra_left();
            break;

        case AVOID_FORWARD:
            obstacle_avoid_forward();
            break;

        case AVOID_SHIFT_RIGHT:
            obstacle_avoid_shift_right();
            break;

        case AVOID_TURN_TO_60:
            obstacle_avoid_turn_to_60();
            break;

        case AVOID_VISUAL_FORWARD:
            obstacle_avoid_visual_forward();
            break;

        case AVOID_TURN_BACK:
            obstacle_avoid_turn_back();
            break;

        case AVOID_FINISH:
            obstacle_avoid_finish();
            break;

        case AVOID_IDLE:
        default:
            obstacle_avoid_idle();
            break;
    }
}

/* ============================================================
 * 13. 触发避障
 * ============================================================ */
void obstacle_avoid_trigger(void)
{
    if (obstacle_avoid_is_active()) return;

    /* 只有巡线模块处于启用状态时才允许触发避障 */
    if (!line_tracker_is_enabled()) return;

    /* 只在新的超声测量节奏上进行确认，避免重复计算同一快照。 */
    if (!obstacle_avoid_sensor_sample_due()) return;

    ultrasonic_data_t data = ultrasonic_sensor_get_data();

    if (!data.valid) {
        g_obstacle_confirm_count = 0;
        return;
    }

    if (data.obstacle) {
        g_obstacle_confirm_count++;
    } else {
        g_obstacle_confirm_count = 0;
    }

    if (g_obstacle_confirm_count < AVOID_OBSTACLE_CONFIRM_COUNT) {
        return;
    }


    /* 重置并激活避障 */
    obstacle_avoid_reset_internal();

    ESP_LOGW(
        AVOID_TAG,
        "OBSTACLE: %.1f cm",
        data.distance_cm
    );

    taskENTER_CRITICAL(&g_avoid_lock);
    g_active = true;
    const obstacle_avoid_mode_t mode = g_mode;
    taskEXIT_CRITICAL(&g_avoid_lock);

    /* 清掉可能残留的高级运动目标，再切断循迹输出。 */
    chassis_motion_cancel();
    line_tracker_enable(false);
    Set_motor(0.0f, 0.0f, 0.0f);

    switch (mode) {
        case AVOID_MODE_DIRECT:
            obstacle_avoid_enter(AVOID_SHIFT_LEFT);
            break;

        case AVOID_MODE_VISUAL:
            ESP_LOGW(
                AVOID_TAG,
                "VISUAL mode not implemented, fallback to DIRECT"
            );

            obstacle_avoid_enter(AVOID_SHIFT_LEFT);
            break;

        default:
            obstacle_avoid_finish_and_return();
            break;
    }
}

/* ============================================================
 * 14. 初始化
 * ============================================================ */
esp_err_t obstacle_avoid_init(void)
{
    obstacle_avoid_reset_internal();

    taskENTER_CRITICAL(&g_avoid_lock);
    g_mode = AVOID_MODE_DIRECT;
    g_avoid_task_handle = NULL;
    taskEXIT_CRITICAL(&g_avoid_lock);

    ESP_LOGI(
        AVOID_TAG,
        "Obstacle avoid initialized"
    );

    return ESP_OK;
}

/* ============================================================
 * 15. 后台任务
 * ============================================================ */
static void obstacle_avoid_task(void *arg)
{
    (void)arg;

    while (1) {
        obstacle_avoid_update();
        vTaskDelay(pdMS_TO_TICKS(AVOID_UPDATE_PERIOD_MS));
    }
}

esp_err_t obstacle_avoid_start(void)
{
    if (g_avoid_task_handle != NULL) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreate(
        obstacle_avoid_task,
        "obstacle_avoid",
        AVOID_TASK_STACK_SIZE,
        NULL,
        AVOID_TASK_PRIORITY,
        &g_avoid_task_handle
    );

    if (ret != pdPASS) {
        g_avoid_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        AVOID_TAG,
        "Obstacle avoid task started"
    );

    return ESP_OK;
}

/* ============================================================
 * 16. 模式设置/读取
 * ============================================================ */
void obstacle_avoid_set_mode(obstacle_avoid_mode_t mode)
{
    bool active;

    taskENTER_CRITICAL(&g_avoid_lock);
    active = g_active;
    if (!active) {
        g_mode = mode;
    }
    taskEXIT_CRITICAL(&g_avoid_lock);

    if (active) {
        ESP_LOGW(
            AVOID_TAG,
            "Cannot change mode while active"
        );
    }
}

obstacle_avoid_mode_t obstacle_avoid_get_mode(void)
{
    obstacle_avoid_mode_t mode;

    taskENTER_CRITICAL(&g_avoid_lock);
    mode = g_mode;
    taskEXIT_CRITICAL(&g_avoid_lock);

    return mode;
}

/* ============================================================
 * 17. 状态读取
 * ============================================================ */
bool obstacle_avoid_is_active(void)
{
    bool active;

    taskENTER_CRITICAL(&g_avoid_lock);
    active = g_active;
    taskEXIT_CRITICAL(&g_avoid_lock);

    return active;
}

obstacle_avoid_state_t obstacle_avoid_get_state(void)
{
    obstacle_avoid_state_t state;

    taskENTER_CRITICAL(&g_avoid_lock);
    state = g_state;
    taskEXIT_CRITICAL(&g_avoid_lock);

    return state;
}

void obstacle_avoid_set_line_found(bool found)
{
    taskENTER_CRITICAL(&g_avoid_lock);
    g_line_found = found;
    taskEXIT_CRITICAL(&g_avoid_lock);
}

void obstacle_avoid_log_state(void)
{
    obstacle_avoid_mode_t mode;
    obstacle_avoid_state_t state;
    bool active;
    bool line_found;
    uint8_t clear_count;

    taskENTER_CRITICAL(&g_avoid_lock);
    mode = g_mode;
    state = g_state;
    active = g_active;
    line_found = g_line_found;
    clear_count = g_clear_count;
    taskEXIT_CRITICAL(&g_avoid_lock);

    ESP_LOGI(
        AVOID_TAG,
        "mode=%d state=%d active=%d line=%d clear=%u",
        (int)mode,
        (int)state,
        active,
        line_found,
        clear_count
    );
}

/* ============================================================
 * 18. 超声波距离读取
 * ============================================================ */
float obstacle_avoid_get_distance_cm(void)
{
    ultrasonic_data_t data = ultrasonic_sensor_get_data();

    if (!data.valid) {
        return -1.0f;
    }

    return data.distance_cm;
}

#endif /* OBSTACLE_AVOID_IMPLEMENTATION */
#endif /* OBSTACLE_AVOID_H */
