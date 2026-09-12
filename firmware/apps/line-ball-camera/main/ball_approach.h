#ifndef BALL_APPROACH_H
#define BALL_APPROACH_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "ball_target.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 球接近控制模块
 *
 * 职责：
 * 1. 接收 ball_target 的目标状态；
 * 2. 没有发现球时，缓慢原地旋转搜索；
 * 3. 发现球后，根据球的横向位置进行对中；
 * 4. 球进入中央后，控制小车向前接近；
 * 5. ball_target 判断到达后停车。
 *
 * 本模块只生成运动指令，不直接调用电机控制函数。
 * ============================================================ */

/* =========================
 * 可调运动参数
 * ========================= */

/* 搜索球时的原地旋转角速度，正负方向由底盘控制层定义，如果搜索方向反了则取反 */
#define BALL_APPROACH_SEARCH_WZ         0.20f
/* 对准球时的最大角速度 */
#define BALL_APPROACH_ALIGN_WZ_MAX      0.15f
/* 对准球时的比例控制系数，wz = KP * normalized_x */
#define BALL_APPROACH_ALIGN_KP_WZ       0.35f //疑似过冲
/* 接近球时的前进速度，建议使用较低速度，便于标定安全停车距离 */
#define BALL_APPROACH_FORWARD_VX        0.08f
/* 球已经进入中央范围后，允许的最大横向角速度，防止对准过程中角速度过大 */
#define BALL_APPROACH_FORWARD_WZ_MAX    0.10f
    
/* =========================
 * 接近状态
 * ========================= */

typedef enum {
    BALL_APPROACH_IDLE = 0,
    BALL_APPROACH_SEARCH,
    BALL_APPROACH_ALIGN,
    BALL_APPROACH_FORWARD,
    BALL_APPROACH_REACHED,
    BALL_APPROACH_ERROR
} ball_approach_state_t;

/* =========================
 * 运动指令
 * ========================= */

typedef struct {
    float vx;      /* 前进速度 */
    float vy;      /* 横向速度（当前策略暂不使用） */
    float wz;      /* 原地/运动旋转角速度 */
    bool stop;     /* 是否要求停车 */
} ball_approach_command_t;

/* =========================
 * 状态信息
 * ========================= */

typedef struct {
    ball_approach_state_t state;
    ball_approach_command_t command;
} ball_approach_status_t;

/* ============================================================
 * 函数声明
 * ============================================================ */

esp_err_t ball_approach_init(void);
esp_err_t ball_approach_reset(void);
bool ball_approach_update(const ball_target_result_t *target, ball_approach_command_t *command);
ball_approach_state_t ball_approach_get_state(void);
bool ball_approach_get_status(ball_approach_status_t *status);
bool ball_approach_is_finished(void);
bool ball_approach_is_error(void);
const char *ball_approach_state_to_string(ball_approach_state_t state);

/* ============================================================
 * IMPLEMENTATION
 *
 * 在一个且仅一个 .c 文件中：
 * #define BALL_APPROACH_IMPLEMENTATION
 * #include "ball_approach.h"
 * 即可生成实现，不需要单独的 ball_approach.c。
 * ============================================================ */

#ifdef BALL_APPROACH_IMPLEMENTATION

#define BALL_APPROACH_TAG "BALL_APPROACH"

static ball_approach_state_t s_ball_approach_state = BALL_APPROACH_SEARCH;
static ball_approach_command_t s_ball_approach_command = {
    .vx = 0.0f,
    .vy = 0.0f,
    .wz = 0.0f,
    .stop = true
};

static float ball_approach_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void ball_approach_stop(ball_approach_command_t *command)
{
    if (command == NULL) return;
    command->vx = 0.0f;
    command->vy = 0.0f;
    command->wz = 0.0f;
    command->stop = true;
}

static void ball_approach_set_command(ball_approach_command_t *command, float vx, float vy, float wz)
{
    if (command == NULL) return;
    command->vx = vx;
    command->vy = vy;
    command->wz = wz;
    command->stop = false;
}

const char *ball_approach_state_to_string(ball_approach_state_t state)
{
    switch (state) {
        case BALL_APPROACH_IDLE:     return "IDLE";
        case BALL_APPROACH_SEARCH:   return "SEARCH";
        case BALL_APPROACH_ALIGN:    return "ALIGN";
        case BALL_APPROACH_FORWARD:  return "FORWARD";
        case BALL_APPROACH_REACHED:  return "REACHED";
        case BALL_APPROACH_ERROR:    return "ERROR";
        default:                     return "UNKNOWN";
    }
}

esp_err_t ball_approach_init(void)
{
    esp_err_t ret = ball_target_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(BALL_APPROACH_TAG, "ball_target_reset failed: %s", esp_err_to_name(ret));
        s_ball_approach_state = BALL_APPROACH_ERROR;
        ball_approach_stop(&s_ball_approach_command);
        return ret;
    }
    s_ball_approach_state = BALL_APPROACH_SEARCH;
    ball_approach_stop(&s_ball_approach_command);
    ESP_LOGI(BALL_APPROACH_TAG,
             "Initialized: search_wz=%.3f align_kp=%.3f align_wz_max=%.3f forward_vx=%.3f forward_wz_max=%.3f",
             (double)BALL_APPROACH_SEARCH_WZ, (double)BALL_APPROACH_ALIGN_KP_WZ,
             (double)BALL_APPROACH_ALIGN_WZ_MAX, (double)BALL_APPROACH_FORWARD_VX,
             (double)BALL_APPROACH_FORWARD_WZ_MAX);
    ESP_LOGI(BALL_APPROACH_TAG, "State -> %s", ball_approach_state_to_string(s_ball_approach_state));
    return ESP_OK;
}

esp_err_t ball_approach_reset(void)
{
    esp_err_t ret = ball_target_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(BALL_APPROACH_TAG, "ball_target_reset failed: %s", esp_err_to_name(ret));
        s_ball_approach_state = BALL_APPROACH_ERROR;
        ball_approach_stop(&s_ball_approach_command);
        return ret;
    }
    s_ball_approach_state = BALL_APPROACH_SEARCH;
    ball_approach_stop(&s_ball_approach_command);
    ESP_LOGI(BALL_APPROACH_TAG, "Reset -> %s", ball_approach_state_to_string(s_ball_approach_state));
    return ESP_OK;
}

bool ball_approach_update(const ball_target_result_t *target, ball_approach_command_t *command)
{
    const float IMAGE_W = 60.0f;   // 虽然未使用，保留以示一致
    if (target == NULL || command == NULL) {
        s_ball_approach_state = BALL_APPROACH_ERROR;
        ball_approach_stop(&s_ball_approach_command);
        if (command != NULL) *command = s_ball_approach_command;
        ESP_LOGE(BALL_APPROACH_TAG, "Invalid update argument");
        return false;
    }

    ball_approach_command_t next_command = { .vx = 0.0f, .vy = 0.0f, .wz = 0.0f, .stop = true };
    ball_approach_state_t previous_state = s_ball_approach_state;
    
    float wz = 0.0f;   // 局部变量

    if (target->reached || target->state == BALL_TARGET_STATE_REACHED) {
        s_ball_approach_state = BALL_APPROACH_REACHED;
        ball_approach_stop(&next_command);
    } else if (!target->detected || target->state == BALL_TARGET_STATE_LOST) {
        s_ball_approach_state = BALL_APPROACH_SEARCH;
        ball_approach_set_command(&next_command, 0.0f, 0.0f, BALL_APPROACH_SEARCH_WZ);
    } else {
        float offset_ratio = target->corrected_x_px / target->center_tolerance_px;
        wz = -BALL_APPROACH_ALIGN_KP_WZ * offset_ratio;
        wz = ball_approach_clamp(wz, -BALL_APPROACH_ALIGN_WZ_MAX, BALL_APPROACH_ALIGN_WZ_MAX);
        
        // 调试日志
        ESP_LOGI("DEBUG_ALIGN", 
            "corrected_x=%.1f, offset_ratio=%.2f, wz=%.3f, pos=%d",
            (double)target->corrected_x_px,
            (double)offset_ratio,
            (double)wz,
            target->position
        );
        
        bool is_center = (target->position == BALL_TARGET_POSITION_CENTER) &&
                         (target->corrected_x_px < 20.0f && target->corrected_x_px > -20.0f);
        
        if (is_center) {
            s_ball_approach_state = BALL_APPROACH_FORWARD;
            float fwz = -BALL_APPROACH_ALIGN_KP_WZ * offset_ratio;
            fwz = ball_approach_clamp(fwz, -BALL_APPROACH_FORWARD_WZ_MAX, BALL_APPROACH_FORWARD_WZ_MAX);
            ball_approach_set_command(&next_command, BALL_APPROACH_FORWARD_VX, 0.0f, fwz);
        } else {
            s_ball_approach_state = BALL_APPROACH_ALIGN;
            ball_approach_set_command(&next_command, 0.0f, 0.0f, wz);
        }
    }

    s_ball_approach_command = next_command;
    *command = next_command;

    if (previous_state != s_ball_approach_state) {
        ESP_LOGI(BALL_APPROACH_TAG, "State: %s -> %s",
                 ball_approach_state_to_string(previous_state),
                 ball_approach_state_to_string(s_ball_approach_state));
    }
    return true;
}

ball_approach_state_t ball_approach_get_state(void)
{
    return s_ball_approach_state;
}

bool ball_approach_get_status(ball_approach_status_t *status)
{
    if (status == NULL) return false;
    status->state = s_ball_approach_state;
    status->command = s_ball_approach_command;
    return true;
}

bool ball_approach_is_finished(void)
{
    return s_ball_approach_state == BALL_APPROACH_REACHED;
}

bool ball_approach_is_error(void)
{
    return s_ball_approach_state == BALL_APPROACH_ERROR;
}

#endif /* BALL_APPROACH_IMPLEMENTATION */

#ifdef __cplusplus
}
#endif

#endif /* BALL_APPROACH_H */