#ifndef BALL_TARGET_H
#define BALL_TARGET_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"          /* 新增：esp_timer_get_time() */
#include "colorball_vision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 球目标状态模块
 * ============================================================ */

/* =========================
 * 可调参数
 * ========================= */

#define BALL_TARGET_IMAGE_CENTER_X_PX        29.5f
#define BALL_TARGET_CAMERA_CENTER_OFFSET_PX  3.0f
#define BALL_TARGET_CENTER_TOLERANCE_PX      20.0f

#define BALL_TARGET_DETECT_CONFIRM_COUNT     3
#define BALL_TARGET_REACHED_CONFIRM_COUNT    3
#define BALL_TARGET_LOST_CONFIRM_COUNT       3

/*
 * 新增：detected 翻真之前，必须“连续看到球”的时长（毫秒）。
 *
 * 相机约 15 fps（每帧 ~66.7 ms），控制循环 50 Hz（每 20 ms 调用一次），
 * 因此同一个相机帧会被 ball_target_update() 调用约 3 次。
 * 单独靠 detect_count >= 3 无法区分“不同帧”和“同一帧刷多次”，
 * 所以这里再叠加一个时间条件：必须真的跨过至少两帧。
 *
 * 150 ms ≈ 两个相机帧，对真实球无损失，对单帧假阳直接挡掉。
 * 想更灵敏可以调到 100；想更稳可以调到 200。
 */
#ifndef BALL_TARGET_DETECT_CONFIRM_MS
#define BALL_TARGET_DETECT_CONFIRM_MS        150U
#endif

/* ========== 到达判断参数（顶部触发） ========== */

/*
 * 球底部 Y 坐标阈值（图像高度 40 像素）
 *
 * 注意：摄像头俯视，球靠近小车时在图像顶部（Y 值小）
 * 所以当 bottom_y <= 此值时触发到达
 *
 * 取值范围: 0 ~ 39
 * 根据日志，你希望 5~10 时触发，建议设为 10
 */
#define BALL_TARGET_REACHED_BOTTOM_THRESHOLD 10U

/*
 * 球从顶部消失检测：
 * 如果球之前在顶部区域（bottom_y <= 15）
 * 然后突然消失（detected 变 false）
 * 说明球已经从视野顶部出去了（被车身碰到）
 */
#define BALL_TARGET_DISAPPEAR_TOP_THRESHOLD 15U

/*
 * 球面积到达判断（可选，设为 0 禁用）
 */
#define BALL_TARGET_REACHED_AREA_THRESHOLD   0U

/* =========================
 * 球的横向位置
 * ========================= */

typedef enum {
    BALL_TARGET_POSITION_UNKNOWN = 0,
    BALL_TARGET_POSITION_LEFT,
    BALL_TARGET_POSITION_CENTER,
    BALL_TARGET_POSITION_RIGHT
} ball_target_position_t;

/* =========================
 * 球目标总体状态
 * ========================= */

typedef enum {
    BALL_TARGET_STATE_LOST = 0,
    BALL_TARGET_STATE_DETECTED,
    BALL_TARGET_STATE_LEFT,
    BALL_TARGET_STATE_CENTER,
    BALL_TARGET_STATE_RIGHT,
    BALL_TARGET_STATE_REACHED
} ball_target_state_t;

/* =========================
 * ball_target 输出数据
 * ========================= */

typedef struct {
    ball_target_state_t state;
    ball_target_position_t position;
    bool detected;
    bool reached;
    float normalized_x;
    float normalized_y;
    float corrected_x_px;
    float center_tolerance_px;
    uint32_t ball_pixels;
    uint16_t bottom_y;
    uint8_t detect_count;
    uint8_t reached_count;
    uint8_t lost_count;
} ball_target_result_t;

/* ============================================================
 * 函数声明
 * ============================================================ */

esp_err_t ball_target_init(void);
esp_err_t ball_target_reset(void);
bool ball_target_update(const colorball_vision_data_t *vision, ball_target_result_t *result);
ball_target_state_t ball_target_get_state(void);
ball_target_position_t ball_target_get_position(void);
bool ball_target_is_detected(void);
bool ball_target_is_reached(void);
bool ball_target_get_result(ball_target_result_t *out_result);
bool ball_target_is_valid_detection(const colorball_vision_data_t *vision);
bool ball_target_check_reached(const colorball_vision_data_t *vision);
ball_target_position_t ball_target_get_position_from_x(float corrected_x_px, float tolerance_px);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */

#ifdef BALL_TARGET_IMPLEMENTATION

#include <string.h>

#define BALL_TARGET_TAG "BALL_TARGET"

static ball_target_result_t s_ball_target = {
    .state = BALL_TARGET_STATE_LOST,
    .position = BALL_TARGET_POSITION_UNKNOWN,
    .detected = false,
    .reached = false,
    .normalized_x = 0.0f,
    .normalized_y = 0.0f,
    .ball_pixels = 0U,
    .bottom_y = 0U,
    .detect_count = 0U,
    .reached_count = 0U,
    .lost_count = 0U
};

/* 用于检测"球从顶部消失"的历史状态 */
static bool s_prev_ball_detected = false;
static uint16_t s_prev_bottom_y = 0;
static uint32_t s_top_disappear_count = 0;

/*
 * 新增：本次“连续有球”序列的起始时刻（微秒）。
 * 0 表示当前没有正在进行的连续序列。
 */
static int64_t s_detect_start_us = 0;

/* 控制循环比相机快；去抖和丢失计数只能由新图像帧推进。 */
static bool s_has_processed_vision = false;
static uint32_t s_last_processed_vision_sequence = 0;
static bool s_last_processed_vision_valid = false;
static bool s_last_processed_ball_detected = false;

esp_err_t ball_target_init(void)
{
    memset(&s_ball_target, 0, sizeof(s_ball_target));
    s_ball_target.state = BALL_TARGET_STATE_LOST;
    s_ball_target.position = BALL_TARGET_POSITION_UNKNOWN;

    s_prev_ball_detected = false;
    s_prev_bottom_y = 0;
    s_top_disappear_count = 0;

    /* 新增 */
    s_detect_start_us = 0;
    s_has_processed_vision = false;
    s_last_processed_vision_sequence = 0;
    s_last_processed_vision_valid = false;
    s_last_processed_ball_detected = false;

    ESP_LOGI(
        BALL_TARGET_TAG,
        "Initialized: center_tol=%.3f detect_confirm=%u detect_confirm_ms=%u "
        "lost_confirm=%u reached_confirm=%u top_threshold=%u "
        "disappear_threshold=%u area_threshold=%u",
        (double)BALL_TARGET_CENTER_TOLERANCE_PX,
        (unsigned)BALL_TARGET_DETECT_CONFIRM_COUNT,
        (unsigned)BALL_TARGET_DETECT_CONFIRM_MS,
        (unsigned)BALL_TARGET_LOST_CONFIRM_COUNT,
        (unsigned)BALL_TARGET_REACHED_CONFIRM_COUNT,
        (unsigned)BALL_TARGET_REACHED_BOTTOM_THRESHOLD,
        (unsigned)BALL_TARGET_DISAPPEAR_TOP_THRESHOLD,
        (unsigned)BALL_TARGET_REACHED_AREA_THRESHOLD);

    return ESP_OK;
}

esp_err_t ball_target_reset(void)
{
    memset(&s_ball_target, 0, sizeof(s_ball_target));
    s_ball_target.state = BALL_TARGET_STATE_LOST;
    s_ball_target.position = BALL_TARGET_POSITION_UNKNOWN;

    s_prev_ball_detected = false;
    s_prev_bottom_y = 0;
    s_top_disappear_count = 0;

    /* 新增 */
    s_detect_start_us = 0;
    s_has_processed_vision = false;
    s_last_processed_vision_sequence = 0;
    s_last_processed_vision_valid = false;
    s_last_processed_ball_detected = false;

    ESP_LOGI(BALL_TARGET_TAG, "Target state reset");

    return ESP_OK;
}

bool ball_target_is_valid_detection(const colorball_vision_data_t *vision)
{
    if (vision == NULL) {
        return false;
    }
    return vision->ready && vision->valid && vision->ball_detected;
}

ball_target_position_t ball_target_get_position_from_x(float corrected_x_px, float tolerance_px)
{
    if (corrected_x_px < -tolerance_px) {
        return BALL_TARGET_POSITION_LEFT;
    }
    if (corrected_x_px > tolerance_px) {
        return BALL_TARGET_POSITION_RIGHT;
    }
    return BALL_TARGET_POSITION_CENTER;
}

/* ============================================================
 * 到达判断（顶部触发版）
 * ============================================================ */
bool ball_target_check_reached(const colorball_vision_data_t *vision)
{
    if (!ball_target_is_valid_detection(vision)) {
        // ===== 检测"球从顶部消失" =====
        // 如果上一帧球在顶部区域（bottom_y <= 15），这一帧突然消失
        // 说明球已经从视野顶部出去了（被车身碰到了）
        if (s_prev_ball_detected && (s_prev_bottom_y <= BALL_TARGET_DISAPPEAR_TOP_THRESHOLD)) {
            s_top_disappear_count++;
            ESP_LOGI(BALL_TARGET_TAG, "Ball disappeared from top! count=%u, prev_y=%u",
                     (unsigned)s_top_disappear_count, (unsigned)s_prev_bottom_y);

            if (s_top_disappear_count >= BALL_TARGET_REACHED_CONFIRM_COUNT) {
                return true;
            }
        } else {
            s_top_disappear_count = 0;
        }
        return false;
    }

    // 球在视野中，重置顶部消失计数
    s_top_disappear_count = 0;

    // 记录当前帧状态（用于下一帧判断）
    s_prev_ball_detected = true;
    s_prev_bottom_y = vision->bbox_end_y;

    bool area_enabled = (BALL_TARGET_REACHED_AREA_THRESHOLD > 0U);
    bool top_enabled = (BALL_TARGET_REACHED_BOTTOM_THRESHOLD > 0U);

    // 面积到达判断
    bool area_reached = area_enabled &&
                        (vision->ball_pixels >= BALL_TARGET_REACHED_AREA_THRESHOLD);

    // ===== 顶部到达判断：球在图像顶部（bottom_y 很小） =====
    // 摄像头俯视，球靠近小车时在图像顶部
    bool top_reached = top_enabled &&
                       (vision->bbox_end_y <= BALL_TARGET_REACHED_BOTTOM_THRESHOLD);

    // 如果两个都禁用，返回 false
    if (!area_enabled && !top_enabled) {
        return false;
    }

    return area_reached || top_reached;
}

/* ============================================================
 * 主更新函数
 * ============================================================ */
bool ball_target_update(const colorball_vision_data_t *vision, ball_target_result_t *result)
{
    if (vision == NULL) {
        return false;
    }

    const bool observation_changed =
        !s_has_processed_vision ||
        vision->sequence != s_last_processed_vision_sequence ||
        vision->valid != s_last_processed_vision_valid ||
        vision->ball_detected != s_last_processed_ball_detected;

    if (!observation_changed) {
        if (result != NULL) {
            *result = s_ball_target;
        }
        return true;
    }

    s_has_processed_vision = true;
    s_last_processed_vision_sequence = vision->sequence;
    s_last_processed_vision_valid = vision->valid;
    s_last_processed_ball_detected = vision->ball_detected;

    if (ball_target_is_valid_detection(vision)) {
        s_ball_target.normalized_x = vision->normalized_x;
        s_ball_target.normalized_y = vision->normalized_y;
        s_ball_target.corrected_x_px = vision->center_x - BALL_TARGET_IMAGE_CENTER_X_PX - BALL_TARGET_CAMERA_CENTER_OFFSET_PX;
        s_ball_target.center_tolerance_px = BALL_TARGET_CENTER_TOLERANCE_PX;

        s_ball_target.ball_pixels = vision->ball_pixels;
        s_ball_target.bottom_y = vision->bbox_end_y;
        s_ball_target.lost_count = 0U;

        /* ---------- 修改开始：用“时长”而不是“调用次数”来确认 ---------- */

        const int64_t now_us = esp_timer_get_time();

        /* 本次“连续有球”序列的起点：只在序列第一帧（count==0）时记录 */
        if (s_ball_target.detect_count == 0U) {
            s_detect_start_us = now_us;
        }

        /* 保留原有计数器，用于日志观察；它不再是唯一判定条件 */
        if (s_ball_target.detect_count < BALL_TARGET_DETECT_CONFIRM_COUNT) {
            s_ball_target.detect_count++;
        }

        /* 新的确认条件：连续看到球的时长 ≥ BALL_TARGET_DETECT_CONFIRM_MS */
        const bool confirmed_by_time =
            (s_detect_start_us > 0) &&
            (now_us - s_detect_start_us) >=
                ((int64_t)BALL_TARGET_DETECT_CONFIRM_MS * 1000LL);

        if (s_ball_target.detect_count >= BALL_TARGET_DETECT_CONFIRM_COUNT &&
            confirmed_by_time) {
            s_ball_target.detected = true;
            s_ball_target.position = ball_target_get_position_from_x(
                s_ball_target.corrected_x_px,
                s_ball_target.center_tolerance_px);

            // 到达判断
            if (ball_target_check_reached(vision)) {
                if (s_ball_target.reached_count < BALL_TARGET_REACHED_CONFIRM_COUNT) {
                    s_ball_target.reached_count++;
                }
            } else {
                s_ball_target.reached_count = 0U;
            }

            if (s_ball_target.reached_count >= BALL_TARGET_REACHED_CONFIRM_COUNT) {
                s_ball_target.reached = true;
                s_ball_target.state = BALL_TARGET_STATE_REACHED;
            } else {
                s_ball_target.reached = false;
                switch (s_ball_target.position) {
                    case BALL_TARGET_POSITION_LEFT:
                        s_ball_target.state = BALL_TARGET_STATE_LEFT;
                        break;
                    case BALL_TARGET_POSITION_CENTER:
                        s_ball_target.state = BALL_TARGET_STATE_CENTER;
                        break;
                    case BALL_TARGET_POSITION_RIGHT:
                        s_ball_target.state = BALL_TARGET_STATE_RIGHT;
                        break;
                    default:
                        s_ball_target.state = BALL_TARGET_STATE_LOST;
                        break;
                }
            }
        } else {
            /* 计数够但时长不够 → 仍然按“未确认”处理 */
            s_ball_target.detected = false;
            s_ball_target.reached = false;
            s_ball_target.position = BALL_TARGET_POSITION_UNKNOWN;
            s_ball_target.reached_count = 0U;
            s_ball_target.state = BALL_TARGET_STATE_DETECTED;
        }
        /* ---------- 修改结束 ---------- */

        // 更新历史状态
        s_prev_ball_detected = true;
        s_prev_bottom_y = vision->bbox_end_y;

    } else {
        // ===== 没有检测到球 =====
        s_ball_target.detect_count = 0U;
        s_ball_target.reached_count = 0U;

        /* 新增：连续序列被打断 */
        s_detect_start_us = 0;

        // 检查是否球从顶部消失
        bool disappeared_from_top = false;
        if (s_prev_ball_detected && (s_prev_bottom_y <= BALL_TARGET_DISAPPEAR_TOP_THRESHOLD)) {
            s_top_disappear_count++;
            if (s_top_disappear_count >= BALL_TARGET_REACHED_CONFIRM_COUNT) {
                disappeared_from_top = true;
            }
        } else {
            s_top_disappear_count = 0;
        }

        if (disappeared_from_top) {
            s_ball_target.reached = true;
            s_ball_target.state = BALL_TARGET_STATE_REACHED;
            s_ball_target.detected = false;
            ESP_LOGI(BALL_TARGET_TAG, "Ball disappeared from top! REACHED=true");
        } else {
            if (s_ball_target.lost_count < BALL_TARGET_LOST_CONFIRM_COUNT) {
                s_ball_target.lost_count++;
            }
            if (s_ball_target.lost_count >= BALL_TARGET_LOST_CONFIRM_COUNT) {
                s_ball_target.detected = false;
                s_ball_target.reached = false;
                s_ball_target.position = BALL_TARGET_POSITION_UNKNOWN;
                s_ball_target.state = BALL_TARGET_STATE_LOST;
            }
        }
        /* 近端球从图像顶部消失时，保留前一帧信息直到连续缺失帧
         * 达到阈值；普通丢帧则立即清空历史。 */
        if (!s_prev_ball_detected ||
            s_prev_bottom_y > BALL_TARGET_DISAPPEAR_TOP_THRESHOLD) {
            s_prev_ball_detected = false;
        }
    }

    if (result != NULL) {
        *result = s_ball_target;
    }
    return true;
}

/* ============================================================
 * 公共 API
 * ============================================================ */
ball_target_state_t ball_target_get_state(void)
{
    return s_ball_target.state;
}

ball_target_position_t ball_target_get_position(void)
{
    return s_ball_target.position;
}

bool ball_target_is_detected(void)
{
    return s_ball_target.detected;
}

bool ball_target_is_reached(void)
{
    return s_ball_target.reached;
}

bool ball_target_get_result(ball_target_result_t *out_result)
{
    if (out_result == NULL) {
        return false;
    }
    *out_result = s_ball_target;
    return true;
}

#endif /* BALL_TARGET_IMPLEMENTATION */

#ifdef __cplusplus
}
#endif

#endif /* BALL_TARGET_H */
