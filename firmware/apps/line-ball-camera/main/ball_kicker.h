#ifndef BALL_KICKER_H
#define BALL_KICKER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "colorball_vision.h"
#include "ball_target.h"
#include "ball_approach.h"
#include "motor_control.h"
#include "chassis_motion.h"
#include "chassis_odometry.h"
#include "voice_demo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * ball_kicker
 *
 * 任务结构：
 *
 *     colorball_vision
 *            ↓
 *       ball_target
 *            ↓
 *      ball_approach
 *            ↓
 *       ball_kicker
 *
 * SEARCH / APPROACH：
 *     由视觉和 ball_approach 产生运动指令，
 *     ball_kicker 负责调用 Set_motor()。
 *
 * TO_BAG：
 *     由 chassis_motion 接管底盘，
 *     此阶段不再调用 Set_motor()。
 *
 * PUSH：      【已移除】推球阶段被删除，TO_BAG 完成后直接进入 FINISH_MOVE
 *
 * FINISH_MOVE：
 *     推球完成后先直线后撤，
 *     再返回本次捡球时的位置，
 *     最后（可选）校正 yaw 至 0°，
 *     然后开始下一颗球的 SEARCH。
 *
 * 坐标：
 *     WORLD +X = 任务开始时机器人朝向
 *     WORLD +Y = 任务开始时机器人左侧
 * ============================================================ */

/* ============================================================
 * 参数
 * ============================================================ */

#ifndef BALL_KICKER_PERIOD_MS
#define BALL_KICKER_PERIOD_MS                     20U
#endif

/* ============================================================
 * SEARCH 蛇形扫描参数
 * ============================================================ */

#ifndef BALL_KICKER_SCAN_HORIZONTAL_STEP_MM
#define BALL_KICKER_SCAN_HORIZONTAL_STEP_MM    150.0f
#endif

#ifndef BALL_KICKER_SCAN_INITIAL_LEFT_STEPS
#define BALL_KICKER_SCAN_INITIAL_LEFT_STEPS    1U
#endif

#ifndef BALL_KICKER_SCAN_HORIZONTAL_STEPS
#define BALL_KICKER_SCAN_HORIZONTAL_STEPS     2U
#endif

#ifndef BALL_KICKER_SCAN_FORWARD_STEP_MM
#define BALL_KICKER_SCAN_FORWARD_STEP_MM      100.0f
#endif

#ifndef BALL_KICKER_SCAN_SPEED_MM_S
#define BALL_KICKER_SCAN_SPEED_MM_S            600.0f
#endif

#ifndef BALL_KICKER_SCAN_MAX_FORWARD_ROWS
#define BALL_KICKER_SCAN_MAX_FORWARD_ROWS      5U
#endif

/* ============================================================
 * 袋子 WORLD 坐标
 * ============================================================ */

#ifndef BALL_KICKER_BALL1_BAG_X_MM
#define BALL_KICKER_BALL1_BAG_X_MM               650.0f
#endif

#ifndef BALL_KICKER_BALL1_BAG_Y_MM
#define BALL_KICKER_BALL1_BAG_Y_MM               -400.0f
#endif

#ifndef BALL_KICKER_BALL2_BAG_X_MM
#define BALL_KICKER_BALL2_BAG_X_MM               650.0f
#endif

#ifndef BALL_KICKER_BALL2_BAG_Y_MM
#define BALL_KICKER_BALL2_BAG_Y_MM              400.0f
#endif

#ifndef BALL_KICKER_BAG_SPEED_MM_S
#define BALL_KICKER_BAG_SPEED_MM_S              600.0f
#endif

#ifndef BALL_KICKER_BAG_POSITION_TOLERANCE_MM
#define BALL_KICKER_BAG_POSITION_TOLERANCE_MM    10.0f
#endif

/* ============================================================
 * PUSH  【已废弃】保留定义以防意外，但不会进入此状态
 * ============================================================ */

#ifndef BALL_KICKER_PUSH_VX
#define BALL_KICKER_PUSH_VX                       0.25f
#endif

#ifndef BALL_KICKER_PUSH_VY
#define BALL_KICKER_PUSH_VY                       0.0f
#endif

#ifndef BALL_KICKER_PUSH_WZ
#define BALL_KICKER_PUSH_WZ                       0.0f
#endif

#ifndef BALL_KICKER_PUSH_DURATION_MS
#define BALL_KICKER_PUSH_DURATION_MS              1000U
#endif

/* ============================================================
 * FINISH_MOVE
 *
 * PUSH 完成后：      （现已无 PUSH，直接由 TO_BAG 转入）
 *     1. 先沿当前车体 -X 方向直线后撤
 *     2. 再回到本次球被接住时的位置
 *     3. 可选：校正 yaw 至 0°
 *
 * 这里不追求毫米级精确，只要离开洞口并回到
 * 原来的搜索区域即可。
 * ============================================================ */

#ifndef BALL_KICKER_FINISH_BACKOFF_DISTANCE_MM
#define BALL_KICKER_FINISH_BACKOFF_DISTANCE_MM   200.0f
#endif

#ifndef BALL_KICKER_FINISH_BACKOFF_SPEED_MM_S
#define BALL_KICKER_FINISH_BACKOFF_SPEED_MM_S    650.0f
#endif

#ifndef BALL_KICKER_FINISH_RETURN_SPEED_MM_S
#define BALL_KICKER_FINISH_RETURN_SPEED_MM_S     600.0f
#endif

#ifndef BALL_KICKER_FINISH_RETURN_TOLERANCE_MM
#define BALL_KICKER_FINISH_RETURN_TOLERANCE_MM    30.0f
#endif

/* ============================================================
 * YAW 校正（新增）
 * ============================================================ */

#ifndef BALL_KICKER_ENABLE_YAW_CORRECTION
#define BALL_KICKER_ENABLE_YAW_CORRECTION         1
#endif

#ifndef BALL_KICKER_YAW_CORRECTION_TOL_DEG
#define BALL_KICKER_YAW_CORRECTION_TOL_DEG        10.0f
#endif

#ifndef BALL_KICKER_YAW_CORRECTION_SPEED_DEG_S
#define BALL_KICKER_YAW_CORRECTION_SPEED_DEG_S    60.0f
#endif

/* ============================================================
 * 球丢失保护
 * ============================================================ */

#ifndef BALL_KICKER_BALL_LOST_TIMEOUT_MS
#define BALL_KICKER_BALL_LOST_TIMEOUT_MS         300U
#endif

#ifndef BALL_KICKER_REACQUIRE_TIMEOUT_MS
#define BALL_KICKER_REACQUIRE_TIMEOUT_MS        9000U
#endif

#ifndef BALL_KICKER_REACQUIRE_SETTLE_MS
#define BALL_KICKER_REACQUIRE_SETTLE_MS          150U
#endif

#ifndef BALL_KICKER_REACQUIRE_SWEEP_MS
#define BALL_KICKER_REACQUIRE_SWEEP_MS           800U
#endif

#ifndef BALL_KICKER_REACQUIRE_RETURN_TOLERANCE_MM
#define BALL_KICKER_REACQUIRE_RETURN_TOLERANCE_MM 50.0f
#endif

#ifndef BALL_KICKER_REACQUIRE_MOVE_SPEED_MM_S
#define BALL_KICKER_REACQUIRE_MOVE_SPEED_MM_S    250.0f
#endif

#ifndef BALL_KICKER_REACQUIRE_ADVANCE_MM
#define BALL_KICKER_REACQUIRE_ADVANCE_MM          100.0f
#endif

/* ============================================================
 * 类型定义
 * ============================================================ */

typedef enum {
    BALL_KICKER_BALL1 = 0,
    BALL_KICKER_BALL2
} ball_kicker_ball_id_t;

typedef enum {
    BALL_KICKER_STATE_IDLE = 0,
    BALL_KICKER_STATE_SEARCH,
    BALL_KICKER_STATE_APPROACH,
    BALL_KICKER_STATE_TO_BAG,
    BALL_KICKER_STATE_PUSH,          // 保留但不再使用
    BALL_KICKER_STATE_FINISH_MOVE,
    BALL_KICKER_STATE_REACQUIRE,
    BALL_KICKER_STATE_BALL_DONE,
    BALL_KICKER_STATE_FINISHED,
    BALL_KICKER_STATE_ERROR
} ball_kicker_state_t;

/* ============================================================
 * FINISH_MOVE 内部状态（新增 YAW_CORRECT）
 * ============================================================ */

typedef enum {
    BALL_KICKER_FINISH_BACKOFF = 0,
    BALL_KICKER_FINISH_RETURN_ORIGIN,
    BALL_KICKER_FINISH_YAW_CORRECT       // 新增
} ball_kicker_finish_phase_t;

/* ============================================================
 * 状态信息
 * ============================================================ */

typedef struct {
    ball_kicker_state_t state;
    ball_kicker_ball_id_t current_ball;
    bool running;
    bool finished;
    bool error;
    bool ball_detected;
    bool ball_reached;
    ball_target_position_t position;
    float corrected_x_px;
    ball_approach_command_t approach_command;
    uint8_t balls_completed;

    float odom_x_mm;
    float odom_y_mm;
    float odom_yaw_deg;

    float bag_x_mm;
    float bag_y_mm;

    float bag_dx_mm;
    float bag_dy_mm;
} ball_kicker_status_t;

/* ============================================================
 * 内部变量
 * ============================================================ */

static const char *BALL_KICKER_TAG = "BALL_KICKER";

static ball_kicker_state_t s_ball_kicker_state =
    BALL_KICKER_STATE_IDLE;

static ball_kicker_ball_id_t s_current_ball =
    BALL_KICKER_BALL1;

static bool s_ball_kicker_running = false;
static bool s_ball_kicker_finished = false;
static bool s_ball_kicker_error = false;

/* 当前 chassis_motion 是否已经启动 */
static bool s_motion_started = false;

/* ============================================================
 * SEARCH 扫描状态
 * ============================================================ */

typedef enum {
    SCAN_PHASE_INITIAL_LEFT = 0,
    SCAN_PHASE_RIGHT,
    SCAN_PHASE_FORWARD_AFTER_RIGHT,
    SCAN_PHASE_LEFT,
    SCAN_PHASE_FORWARD_AFTER_LEFT,
    SCAN_PHASE_IDLE
} scan_phase_t;

static scan_phase_t s_scan_phase = SCAN_PHASE_IDLE;

/* 当前横向方向已经完成了多少个 150 mm 小段 */
static uint8_t s_scan_horizontal_step = 0;

/* 已经完成多少次前进 100 mm */
static uint8_t s_scan_forward_row = 0;

static bool s_scan_motion_started = false;
/* ============================================================
 * FINISH_MOVE 状态
 * ============================================================ */

static ball_kicker_finish_phase_t s_finish_phase =
    BALL_KICKER_FINISH_BACKOFF;

static bool s_finish_motion_started = false;

/* ============================================================
 * PUSH 开始时间（保留但不使用）
 * ============================================================ */

static int64_t s_push_start_us = 0;

/* ============================================================
 * 最近一次看到球的时间
 * ============================================================ */

static int64_t s_last_ball_seen_us = 0;

/* ============================================================
 * REACQUIRE 开始时间
 * ============================================================ */

static int64_t s_reacquire_start_us = 0;

typedef enum {
    REACQUIRE_SETTLE = 0,
    REACQUIRE_RETURN_LAST_VIEW,
    REACQUIRE_SWEEP_PRIMARY,
    REACQUIRE_ADVANCE,
    REACQUIRE_SWEEP_SECONDARY
} reacquire_phase_t;

static reacquire_phase_t s_reacquire_phase = REACQUIRE_SETTLE;
static int64_t s_reacquire_phase_start_us = 0;
static bool s_reacquire_motion_started = false;
static bool s_resume_search_after_reacquire = false;

/* ============================================================
 * 已完成球数量
 * ============================================================ */

static uint8_t s_balls_completed = 0;

/* ============================================================
 * SEARCH 起始 WORLD 原点
 *
 * ball_kicker_start() 时 odometry 被 reset 为 (0, 0, 0)。
 * 每颗球 PUSH 完成后，FINISH_MOVE 都以当前 odometry
 * 为起点，重新计算直接回到这个固定原点的方向和距离。
 * ============================================================ */

#define BALL_KICKER_SEARCH_ORIGIN_X_MM 0.0f
#define BALL_KICKER_SEARCH_ORIGIN_Y_MM 0.0f

/* ============================================================
 * 缓存视觉 / target / approach
 * ============================================================ */

static colorball_vision_data_t s_last_vision = {0};
static ball_target_result_t s_last_target = {0};
static ball_approach_command_t s_last_approach_command = {0};

/* ============================================================
 * 缓存 odometry
 * ============================================================ */

static chassis_odometry_state_t s_last_odom = {0};
static chassis_odometry_state_t s_last_seen_odom = {0};
static bool s_last_seen_odom_valid = false;
static float s_last_seen_corrected_x_px = 0.0f;

/* ============================================================
 * 当前袋子 WORLD 坐标
 * ============================================================ */

static float s_current_bag_x_mm = 0.0f;
static float s_current_bag_y_mm = 0.0f;

/* 当前剩余 WORLD 位移 */
static float s_bag_dx_mm = 0.0f;
static float s_bag_dy_mm = 0.0f;

/* ============================================================
 * 工具函数
 * ============================================================ */

static inline float ball_kicker_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static inline float ball_kicker_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* ============================================================
 * 直接控制电机
 *
 * 仅 SEARCH / APPROACH / REACQUIRE 使用。
 * TO_BAG / FINISH_MOVE 使用 chassis_motion。
 * ============================================================ */

static inline void ball_kicker_direct_stop(void)
{
    Set_motor(0.0f, 0.0f, 0.0f);
}

static inline void ball_kicker_set_motion(
    float vx,
    float vy,
    float wz)
{
    Set_motor(vx, vy, wz);
}

/* ============================================================
 * 状态字符串
 * ============================================================ */

static const char *ball_kicker_state_to_string(
    ball_kicker_state_t state)
{
    switch (state) {
        case BALL_KICKER_STATE_IDLE:
            return "IDLE";

        case BALL_KICKER_STATE_SEARCH:
            return "SEARCH";

        case BALL_KICKER_STATE_APPROACH:
            return "APPROACH";

        case BALL_KICKER_STATE_TO_BAG:
            return "TO_BAG";

        case BALL_KICKER_STATE_PUSH:
            return "PUSH";

        case BALL_KICKER_STATE_FINISH_MOVE:
            return "FINISH_MOVE";

        case BALL_KICKER_STATE_REACQUIRE:
            return "REACQUIRE";

        case BALL_KICKER_STATE_BALL_DONE:
            return "BALL_DONE";

        case BALL_KICKER_STATE_FINISHED:
            return "FINISHED";

        case BALL_KICKER_STATE_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

/* ============================================================
 * 设置状态
 * ============================================================ */

static void ball_kicker_set_state(
    ball_kicker_state_t new_state)
{
    if (s_ball_kicker_state == new_state)
        return;

    ESP_LOGI(
        BALL_KICKER_TAG,
        "STATE %s -> %s",
        ball_kicker_state_to_string(
            s_ball_kicker_state),
        ball_kicker_state_to_string(
            new_state));

    s_ball_kicker_state = new_state;

    if (new_state == BALL_KICKER_STATE_TO_BAG) {
        s_motion_started = false;
    }

    if (new_state == BALL_KICKER_STATE_PUSH) {
        s_push_start_us = esp_timer_get_time();
    }

    if (new_state == BALL_KICKER_STATE_FINISH_MOVE) {
        s_finish_phase =
            BALL_KICKER_FINISH_BACKOFF;

        s_finish_motion_started = false;
        s_motion_started = false;

        if (chassis_motion_is_busy()) {
            chassis_motion_cancel();
        }
    }

    if (new_state == BALL_KICKER_STATE_REACQUIRE) {
        s_reacquire_start_us =
            esp_timer_get_time();
        s_reacquire_phase = REACQUIRE_SETTLE;
        s_reacquire_phase_start_us = s_reacquire_start_us;
        s_reacquire_motion_started = false;
    }

    if (new_state == BALL_KICKER_STATE_SEARCH) {
        if (s_resume_search_after_reacquire) {
            s_resume_search_after_reacquire = false;
            return;
        }

        /*
        * SEARCH 每次重新开始，都从：
        *
        *   左150
        *   左150
        *
        * 开始。
        */
        s_scan_phase = SCAN_PHASE_INITIAL_LEFT;
        s_scan_horizontal_step = 0;
        s_scan_forward_row = 0;
        s_scan_motion_started = false;

        if (chassis_motion_is_busy()) {
            chassis_motion_cancel();
        }
    }
}

/* ============================================================
 * 获取当前球对应袋子 WORLD 坐标
 * ============================================================ */

static void ball_kicker_get_bag_position(
    float *x_mm,
    float *y_mm)
{
    if (x_mm == NULL || y_mm == NULL)
        return;

    if (s_current_ball == BALL_KICKER_BALL1) {
        *x_mm = BALL_KICKER_BALL1_BAG_X_MM;
        *y_mm = BALL_KICKER_BALL1_BAG_Y_MM;
    } else {
        *x_mm = BALL_KICKER_BALL2_BAG_X_MM;
        *y_mm = BALL_KICKER_BALL2_BAG_Y_MM;
    }
}

/* ============================================================
 * 更新 odometry 缓存
 * ============================================================ */

static bool ball_kicker_update_odometry(void)
{
    if (!chassis_odometry_is_ready())
        return false;

    chassis_odometry_get_state(
        &s_last_odom);

    return s_last_odom.initialized &&
           s_last_odom.running;
}

/* ============================================================
 * 更新当前到袋子的 WORLD 位移
 * ============================================================ */

static bool ball_kicker_update_bag_error(void)
{
    if (!ball_kicker_update_odometry())
        return false;

    s_bag_dx_mm =
        s_current_bag_x_mm -
        s_last_odom.x_mm;

    s_bag_dy_mm =
        s_current_bag_y_mm -
        s_last_odom.y_mm;

    return true;
}

/* ============================================================
 * WORLD X / Y → chassis_motion 相对方向
 *
 * chassis_motion：
 *     0°   = 当前车体 forward
 *     +90° = 当前车体 left
 *
 * WORLD：
 *     +X = 任务开始时 forward
 *     +Y = 任务开始时 left
 * ============================================================ */

static float ball_kicker_world_x_direction_deg(
    float yaw_deg)
{
    return ball_kicker_wrap_deg(-yaw_deg);
}

static float ball_kicker_world_y_direction_deg(
    float yaw_deg)
{
    return ball_kicker_wrap_deg(90.0f - yaw_deg);
}

/* ============================================================
 * 当前球是否丢失
 * ============================================================ */

static bool ball_kicker_ball_is_lost(
    const ball_target_result_t *target)
{
    if (target == NULL)
        return true;

    if (target->detected &&
        target->state != BALL_TARGET_STATE_LOST) {
        return false;
    }

    const int64_t now_us =
        esp_timer_get_time();

    const int64_t lost_us =
        now_us - s_last_ball_seen_us;

    return lost_us >=
           ((int64_t)
            BALL_KICKER_BALL_LOST_TIMEOUT_MS *
            1000LL);
}

/* ============================================================
 * 初始化
 * ============================================================ */

static esp_err_t ball_kicker_init(void)
{
    esp_err_t ret;

    ret = ball_target_init();

    if (ret != ESP_OK) {
        ESP_LOGE(
            BALL_KICKER_TAG,
            "ball_target_init failed: %s",
            esp_err_to_name(ret));
        return ret;
    }

    ret = ball_approach_init();

    if (ret != ESP_OK) {
        ESP_LOGE(
            BALL_KICKER_TAG,
            "ball_approach_init failed: %s",
            esp_err_to_name(ret));
        return ret;
    }

    s_ball_kicker_state =
        BALL_KICKER_STATE_IDLE;

    s_current_ball =
        BALL_KICKER_BALL1;

    s_ball_kicker_running = false;
    s_ball_kicker_finished = false;
    s_ball_kicker_error = false;

    s_motion_started = false;

    s_scan_phase = SCAN_PHASE_IDLE;
    s_scan_forward_row = 0;
    s_scan_motion_started = false;

    s_finish_phase =
        BALL_KICKER_FINISH_BACKOFF;

    s_finish_motion_started = false;

    s_push_start_us = 0;
    s_last_ball_seen_us = 0;
    s_reacquire_start_us = 0;
    s_reacquire_phase = REACQUIRE_SETTLE;
    s_reacquire_phase_start_us = 0;
    s_reacquire_motion_started = false;
    s_resume_search_after_reacquire = false;

    s_balls_completed = 0;

    s_last_vision =
        (colorball_vision_data_t){0};

    s_last_target =
        (ball_target_result_t){0};

    s_last_approach_command =
        (ball_approach_command_t){0};

    s_last_odom =
        (chassis_odometry_state_t){0};
    s_last_seen_odom =
        (chassis_odometry_state_t){0};
    s_last_seen_odom_valid = false;
    s_last_seen_corrected_x_px = 0.0f;

    s_current_bag_x_mm = 0.0f;
    s_current_bag_y_mm = 0.0f;

    s_bag_dx_mm = 0.0f;
    s_bag_dy_mm = 0.0f;

    ball_kicker_direct_stop();

    ESP_LOGI(
        BALL_KICKER_TAG,
        "Initialized");

    return ESP_OK;
}

/* ============================================================
 * 启动整个任务
 * ============================================================ */

static esp_err_t ball_kicker_start(void)
{
    if (s_ball_kicker_running)
        return ESP_OK;

    if (s_ball_kicker_error) {
        ESP_LOGE(
            BALL_KICKER_TAG,
            "Cannot start: ERROR state");
        return ESP_FAIL;
    }

    if (!chassis_odometry_is_ready()) {
        ESP_LOGE(
            BALL_KICKER_TAG,
            "Odometry is not ready");
        return ESP_ERR_INVALID_STATE;
    }

    /* 整个任务只 reset 一次 */
    esp_err_t ret =
        chassis_odometry_reset(
            0.0f,
            0.0f,
            0.0f);

    if (ret != ESP_OK) {
        ESP_LOGE(
            BALL_KICKER_TAG,
            "odometry reset failed: %s",
            esp_err_to_name(ret));
        return ret;
    }

    chassis_motion_stop();

    ret = ball_target_reset();

    if (ret != ESP_OK)
        return ret;

    ret = ball_approach_reset();

    if (ret != ESP_OK)
        return ret;

    s_current_ball =
        BALL_KICKER_BALL1;

    s_balls_completed = 0;

    s_ball_kicker_running = true;
    s_ball_kicker_finished = false;
    s_ball_kicker_error = false;

    s_motion_started = false;

    s_last_ball_seen_us =
        esp_timer_get_time();
    s_last_seen_odom_valid = false;
    s_last_seen_corrected_x_px = 0.0f;
    s_resume_search_after_reacquire = false;

    ball_kicker_get_bag_position(
        &s_current_bag_x_mm,
        &s_current_bag_y_mm);

    ball_kicker_update_odometry();

    ball_kicker_set_state(
        BALL_KICKER_STATE_SEARCH);

    ESP_LOGI(
        BALL_KICKER_TAG,
        "START: WORLD origin=(0,0,0)");

    ESP_LOGI(
        BALL_KICKER_TAG,
        "BALL1 BAG=(%.1f, %.1f)",
        (double)s_current_bag_x_mm,
        (double)s_current_bag_y_mm);

    return ESP_OK;
}

/* ============================================================
 * 停止整个任务
 * ============================================================ */

static esp_err_t ball_kicker_stop_task(void)
{
    ball_kicker_direct_stop();
    chassis_motion_stop();

    s_ball_kicker_running = false;

    if (s_ball_kicker_state !=
            BALL_KICKER_STATE_FINISHED &&
        s_ball_kicker_state !=
            BALL_KICKER_STATE_ERROR) {
        s_ball_kicker_state =
            BALL_KICKER_STATE_IDLE;
    }

    return ESP_OK;
}

/* ============================================================
 * SEARCH - 蛇形扫描
 *
 * 第一次：
 *     左 300
 *
 * 然后：
 *     前 100
 *     右 600
 *     前 100
 *     左 600
 *     前 100
 *     ...
 *
 * 这样不会出现一开始左-前-右导致右侧区域
 * 在第一轮没有覆盖的问题。
 * ============================================================ */

static void ball_kicker_update_search(const ball_target_result_t *target)
{
    /* ============================================================
     * 1. 发现球：立即停止当前扫描，进入 APPROACH
     * ============================================================ */
    if (target->detected && target->state != BALL_TARGET_STATE_LOST) {
        // check
        ESP_LOGW(BALL_KICKER_TAG,
             "SEARCH->APPROACH: detect_count=%u lost_count=%u "
             "state=%d pos=%d x_px=%.1f y_px=%u "
             "scan_phase=%d step=%u row=%u",
             (unsigned)target->detect_count,
             (unsigned)target->lost_count,
             (int)target->state,
             (int)target->position,
             (double)target->corrected_x_px,
             (unsigned)target->bottom_y,
             (int)s_scan_phase,
             (unsigned)s_scan_horizontal_step,
             (unsigned)s_scan_forward_row);

        if (s_scan_motion_started) {
            chassis_motion_cancel();
            s_scan_motion_started = false;
        }

        s_scan_phase = SCAN_PHASE_IDLE;
        s_scan_horizontal_step = 0;

        s_last_ball_seen_us = esp_timer_get_time();
        (void)voice_demo_play_clip(VOICE_CLIP_MAMA);

        ball_kicker_set_state(BALL_KICKER_STATE_APPROACH);
        return;
    }

    /* ============================================================
     * 2. 如果当前 150 mm / 100 mm 小段还没有启动
     * ============================================================ */
    if (!s_scan_motion_started) {

        float direction_deg = 0.0f;
        float distance_mm = 0.0f;
        bool valid_motion = true;

        switch (s_scan_phase) {

            /* ----------------------------------------------------
             * 初始左移：
             *
             *     左150
             *     左150
             *
             * 共 300 mm
             * ---------------------------------------------------- */
            case SCAN_PHASE_INITIAL_LEFT:

                direction_deg = +90.0f;   /* +90° = 左 */
                distance_mm = BALL_KICKER_SCAN_HORIZONTAL_STEP_MM;

                break;


            /* ----------------------------------------------------
             * 右移整行：
             *
             *     右150 × 4
             *
             * 共 600 mm
             * ---------------------------------------------------- */
            case SCAN_PHASE_RIGHT:

                direction_deg = -90.0f;   /* -90° = 右 */
                distance_mm = BALL_KICKER_SCAN_HORIZONTAL_STEP_MM;

                break;


            /* ----------------------------------------------------
             * 右移完成后前进100
             * ---------------------------------------------------- */
            case SCAN_PHASE_FORWARD_AFTER_RIGHT:

                direction_deg = 0.0f;     /* 前 */
                distance_mm = BALL_KICKER_SCAN_FORWARD_STEP_MM;

                break;


            /* ----------------------------------------------------
             * 左移整行：
             *
             *     左150 × 4
             *
             * 共 600 mm
             * ---------------------------------------------------- */
            case SCAN_PHASE_LEFT:

                direction_deg = +90.0f;   /* +90° = 左 */
                distance_mm = BALL_KICKER_SCAN_HORIZONTAL_STEP_MM;

                break;


            /* ----------------------------------------------------
             * 左移完成后前进100
             * ---------------------------------------------------- */
            case SCAN_PHASE_FORWARD_AFTER_LEFT:

                direction_deg = 0.0f;     /* 前 */
                distance_mm = BALL_KICKER_SCAN_FORWARD_STEP_MM;

                break;


            default:

                valid_motion = false;

                break;
        }

        /* --------------------------------------------------------
         * 当前 phase 不需要运动
         * -------------------------------------------------------- */
        if (!valid_motion) {
            return;
        }

        /* --------------------------------------------------------
         * 启动这一小段运动
         *
         * 注意：
         * 每次这里只走 150 mm 或 100 mm。
         * 绝不会一次发 300 / 600 mm。
         * -------------------------------------------------------- */
        esp_err_t ret = chassis_motion_move_distance(
            direction_deg,
            distance_mm,
            BALL_KICKER_SCAN_SPEED_MM_S
        );

        if (ret != ESP_OK) {

            ESP_LOGE(
                BALL_KICKER_TAG,
                "Scan move failed: %s phase=%d step=%u row=%u dir=%.1f dist=%.1f",
                esp_err_to_name(ret),
                (int)s_scan_phase,
                (unsigned)s_scan_horizontal_step,
                (unsigned)s_scan_forward_row,
                (double)direction_deg,
                (double)distance_mm
            );

            ball_kicker_direct_stop();
            chassis_motion_stop();

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
            return;
        }

        s_scan_motion_started = true;

        ESP_LOGI(
            BALL_KICKER_TAG,
            "SCAN START: phase=%d step=%u/%u row=%u dir=%+.0f dist=%.0f",
            (int)s_scan_phase,
            (unsigned)s_scan_horizontal_step,
            (unsigned)(
                s_scan_phase == SCAN_PHASE_INITIAL_LEFT
                    ? BALL_KICKER_SCAN_INITIAL_LEFT_STEPS
                    : BALL_KICKER_SCAN_HORIZONTAL_STEPS
            ),
            (unsigned)s_scan_forward_row,
            (double)direction_deg,
            (double)distance_mm
        );

        return;
    }


    /* ============================================================
     * 3. 当前这一小段运动正在执行
     * ============================================================ */

    if (chassis_motion_is_busy()) {
        return;
    }


    /* ============================================================
     * 4. 当前 150 / 100 mm 小段已经完成
     * ============================================================ */

    s_scan_motion_started = false;


    /* ============================================================
     * 5. 根据当前 phase 推进扫描状态
     * ============================================================ */

    switch (s_scan_phase) {

        /* ========================================================
         * 初始左移
         *
         * 左150
         * 左150
         *
         * 完成后 → 右移600
         * ======================================================== */
        case SCAN_PHASE_INITIAL_LEFT:

            s_scan_horizontal_step++;

            if (s_scan_horizontal_step >=
                BALL_KICKER_SCAN_INITIAL_LEFT_STEPS) {

                /*
                 * 左300已经完成。
                 *
                 * 下一步必须直接开始：
                 *     右150 × 4
                 */
                s_scan_horizontal_step = 0;
                s_scan_phase = SCAN_PHASE_RIGHT;

                ESP_LOGI(
                    BALL_KICKER_TAG,
                    "SCAN: initial LEFT 300 done -> RIGHT 600"
                );
            }

            break;


        /* ========================================================
         * 右移600
         *
         * 右150 × 4
         *
         * 完成后 → 前100
         * ======================================================== */
        case SCAN_PHASE_RIGHT:

            s_scan_horizontal_step++;

            if (s_scan_horizontal_step >=
                BALL_KICKER_SCAN_HORIZONTAL_STEPS) {

                s_scan_horizontal_step = 0;
                s_scan_phase = SCAN_PHASE_FORWARD_AFTER_RIGHT;

                ESP_LOGI(
                    BALL_KICKER_TAG,
                    "SCAN: RIGHT 600 done -> FORWARD 100"
                );
            }

            break;


        /* ========================================================
         * 右移完成后的前100
         *
         * 完成后 → 左移600
         * ======================================================== */
        case SCAN_PHASE_FORWARD_AFTER_RIGHT:

            s_scan_forward_row++;

            if (s_scan_forward_row >=
                BALL_KICKER_SCAN_MAX_FORWARD_ROWS) {

                ESP_LOGE(
                    BALL_KICKER_TAG,
                    "Scan timeout: no ball found"
                );

                ball_kicker_direct_stop();
                chassis_motion_stop();

                s_ball_kicker_error = true;
                s_ball_kicker_running = false;

                ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
                return;
            }

            s_scan_phase = SCAN_PHASE_LEFT;

            ESP_LOGI(
                BALL_KICKER_TAG,
                "SCAN: FORWARD 100 done -> LEFT 600"
            );

            break;


        /* ========================================================
         * 左移600
         *
         * 左150 × 4
         *
         * 完成后 → 前100
         * ======================================================== */
        case SCAN_PHASE_LEFT:

            s_scan_horizontal_step++;

            if (s_scan_horizontal_step >=
                BALL_KICKER_SCAN_HORIZONTAL_STEPS) {

                s_scan_horizontal_step = 0;
                s_scan_phase = SCAN_PHASE_FORWARD_AFTER_LEFT;

                ESP_LOGI(
                    BALL_KICKER_TAG,
                    "SCAN: LEFT 600 done -> FORWARD 100"
                );
            }

            break;


        /* ========================================================
         * 左移完成后的前100
         *
         * 完成后 → 右移600
         * ======================================================== */
        case SCAN_PHASE_FORWARD_AFTER_LEFT:

            s_scan_forward_row++;

            if (s_scan_forward_row >=
                BALL_KICKER_SCAN_MAX_FORWARD_ROWS) {

                ESP_LOGE(
                    BALL_KICKER_TAG,
                    "Scan timeout: no ball found"
                );

                ball_kicker_direct_stop();
                chassis_motion_stop();

                s_ball_kicker_error = true;
                s_ball_kicker_running = false;

                ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
                return;
            }

            s_scan_phase = SCAN_PHASE_RIGHT;

            ESP_LOGI(
                BALL_KICKER_TAG,
                "SCAN: FORWARD 100 done -> RIGHT 600"
            );

            break;


        default:

            s_scan_phase = SCAN_PHASE_INITIAL_LEFT;
            s_scan_horizontal_step = 0;
            s_scan_forward_row = 0;

            break;
    }
}

/* ============================================================
 * APPROACH
 * ============================================================ */

static void ball_kicker_update_approach(
    const ball_target_result_t *target)
{
    ball_approach_command_t command =
        {0};

    ball_approach_update(
        target,
        &command);

    s_last_approach_command =
        command;

    if (target->detected &&
        target->state !=
            BALL_TARGET_STATE_LOST) {

        s_last_ball_seen_us =
            esp_timer_get_time();
    }

    if (ball_kicker_ball_is_lost(target)) {
        ball_kicker_direct_stop();

        ball_kicker_set_state(
            BALL_KICKER_STATE_REACQUIRE);

        return;
    }

    if (target->reached ||
        target->state ==
            BALL_TARGET_STATE_REACHED) {

        ball_kicker_direct_stop();

        if (!ball_kicker_update_odometry()) {
            ESP_LOGE(
                BALL_KICKER_TAG,
                "Odometry unavailable after approach");

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            ball_kicker_direct_stop();

            ball_kicker_set_state(
                BALL_KICKER_STATE_ERROR);

            return;
        }

        ball_kicker_get_bag_position(
            &s_current_bag_x_mm,
            &s_current_bag_y_mm);

        ball_kicker_update_bag_error();

        ESP_LOGI(
            BALL_KICKER_TAG,
            "BALL%d REACHED: "
            "robot=(%.1f, %.1f, %.1f) "
            "bag=(%.1f, %.1f) "
            "remain=(%.1f, %.1f)",
            (int)s_current_ball + 1,
            (double)s_last_odom.x_mm,
            (double)s_last_odom.y_mm,
            (double)s_last_odom.yaw_deg,
            (double)s_current_bag_x_mm,
            (double)s_current_bag_y_mm,
            (double)s_bag_dx_mm,
            (double)s_bag_dy_mm);

        ball_kicker_set_state(
            BALL_KICKER_STATE_TO_BAG);

        return;
    }

    ball_kicker_set_motion(
        command.vx,
        command.vy,
        command.wz);
}

/* ============================================================
 * TO_BAG
 *
 * 直接从当前 odometry 位置斜向袋子：
 *
 *     dx = bag_x - odom_x
 *     dy = bag_y - odom_y
 *     distance = hypot(dx, dy)
 *     world_dir = atan2(dy, dx)
 *     body_dir = world_dir - current_yaw
 *
 * 整个 TO_BAG 只启动一次 chassis_motion_move_distance()。
 * 不依赖视觉，不在带球过程中重新纠偏。
 *
 * 【修改】TO_BAG 完成后不再进入 PUSH，直接进入 FINISH_MOVE。
 * ============================================================ */

static void ball_kicker_update_to_bag(
    const ball_target_result_t *target)
{
    (void)target;

    /* 当前这一次斜向移动尚未完成 */
    if (s_motion_started) {

        if (chassis_motion_is_busy())
            return;

        if (!ball_kicker_update_odometry()) {

            ESP_LOGE(
                BALL_KICKER_TAG,
                "TO_BAG: odometry unavailable after motion");

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            chassis_motion_stop();

            ball_kicker_set_state(
                BALL_KICKER_STATE_ERROR);

            return;
        }

        s_motion_started = false;

        ball_kicker_update_bag_error();

        ESP_LOGI(
            BALL_KICKER_TAG,
            "TO_BAG DONE: "
            "odom=(%.1f, %.1f, %.1f) "
            "remain=(%.1f, %.1f)",
            (double)s_last_odom.x_mm,
            (double)s_last_odom.y_mm,
            (double)s_last_odom.yaw_deg,
            (double)s_bag_dx_mm,
            (double)s_bag_dy_mm);

        /* ===== 修改点：不再进入 PUSH，直接进入 FINISH_MOVE ===== */
        if (ball_kicker_absf(s_bag_dx_mm) <=
                BALL_KICKER_BAG_POSITION_TOLERANCE_MM &&
            ball_kicker_absf(s_bag_dy_mm) <=
                BALL_KICKER_BAG_POSITION_TOLERANCE_MM) {

            ball_kicker_set_state(
                BALL_KICKER_STATE_FINISH_MOVE);   // 原为 PUSH

            return;
        }

        /*
         * 如果 chassis_motion 完成后仍有较大 odometry 误差，
         * 重新计算一次“当前位置 -> 袋子”的直线。
         * 仍然是斜向直达，不退回 X/Y 两段模式。
         */
        return;
    }

    if (!ball_kicker_update_bag_error()) {

        ESP_LOGE(
            BALL_KICKER_TAG,
            "TO_BAG: odometry unavailable");

        s_ball_kicker_error = true;
        s_ball_kicker_running = false;

        chassis_motion_stop();

        ball_kicker_set_state(
            BALL_KICKER_STATE_ERROR);

        return;
    }

    const float dx = s_bag_dx_mm;
    const float dy = s_bag_dy_mm;

    const float distance_mm =
        sqrtf(dx * dx + dy * dy);

    if (distance_mm <=
        BALL_KICKER_BAG_POSITION_TOLERANCE_MM) {

        ball_kicker_set_state(
            BALL_KICKER_STATE_FINISH_MOVE);   // 原为 PUSH

        return;
    }

    /*
     * atan2 的 WORLD 方向：
     *     +X = 0°
     *     +Y = +90°
     */
    const float direction_world_deg =
        atan2f(dy, dx) * 57.2957795f;

    /*
     * chassis_motion 的 direction_deg 是当前车体坐标，
     * 所以需要减去当前 WORLD yaw。
     */
    const float direction_body_deg =
        ball_kicker_wrap_deg(
            direction_world_deg -
            s_last_odom.yaw_deg);

    esp_err_t ret =
        chassis_motion_move_distance(
            direction_body_deg,
            distance_mm,
            BALL_KICKER_BAG_SPEED_MM_S);

    if (ret != ESP_OK) {

        ESP_LOGE(
            BALL_KICKER_TAG,
            "TO_BAG motion failed: %s",
            esp_err_to_name(ret));

        s_ball_kicker_error = true;
        s_ball_kicker_running = false;

        chassis_motion_stop();

        ball_kicker_set_state(
            BALL_KICKER_STATE_ERROR);

        return;
    }

    s_motion_started = true;

    ESP_LOGI(
        BALL_KICKER_TAG,
        "TO_BAG DIRECT: "
        "robot=(%.1f, %.1f, %.1f) "
        "bag=(%.1f, %.1f) "
        "delta=(%.1f, %.1f) "
        "world_dir=%.1f body_dir=%.1f dist=%.1f",
        (double)s_last_odom.x_mm,
        (double)s_last_odom.y_mm,
        (double)s_last_odom.yaw_deg,
        (double)s_current_bag_x_mm,
        (double)s_current_bag_y_mm,
        (double)dx,
        (double)dy,
        (double)direction_world_deg,
        (double)direction_body_deg,
        (double)distance_mm);
}

/* ============================================================
 * PUSH  【已废弃】保留函数但立即跳转，避免编译错误
 * ============================================================ */

static void ball_kicker_update_push(
    const ball_target_result_t *target)
{
    (void)target;
    // 如果意外进入此状态，直接跳到 FINISH_MOVE
    ESP_LOGW(BALL_KICKER_TAG, "PUSH state entered but is disabled, jumping to FINISH_MOVE");
    ball_kicker_direct_stop();
    ball_kicker_set_state(BALL_KICKER_STATE_FINISH_MOVE);
}

/* ============================================================
 * FINISH_MOVE
 *
 * 第一阶段：
 *     车体正后方直线后撤
 *
 * 第二阶段：
 *     返回本次球开始 TO_BAG 前的位置
 *
 * 第三阶段（新增）：
 *     可选校正 yaw 至 0°
 *
 * 注意：
 *     BACKOFF 结束后不直接开始 SEARCH。
 *     必须先 RETURN 到原搜索位置，再校正 yaw（若启用），
 *     然后才进入 BALL_DONE。
 * ============================================================ */

static void ball_kicker_update_finish_move(void)
{
    /* ------------------------------------------------------------
     * 当前 chassis_motion 正在执行
     * ------------------------------------------------------------ */
    if (s_finish_motion_started) {

        if (chassis_motion_is_busy())
            return;

        if (!ball_kicker_update_odometry()) {

            ESP_LOGE(
                BALL_KICKER_TAG,
                "FINISH_MOVE: odometry unavailable");

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            chassis_motion_stop();

            ball_kicker_set_state(
                BALL_KICKER_STATE_ERROR);

            return;
        }

        s_finish_motion_started = false;

        if (s_finish_phase ==
            BALL_KICKER_FINISH_BACKOFF) {

            ESP_LOGI(
                BALL_KICKER_TAG,
                "FINISH_MOVE BACKOFF DONE: "
                "odom=(%.1f, %.1f, %.1f)",
                (double)s_last_odom.x_mm,
                (double)s_last_odom.y_mm,
                (double)s_last_odom.yaw_deg);

            s_finish_phase =
                BALL_KICKER_FINISH_RETURN_ORIGIN;

            return;
        }

        if (s_finish_phase ==
            BALL_KICKER_FINISH_RETURN_ORIGIN) {

            const float dx =
                BALL_KICKER_SEARCH_ORIGIN_X_MM -
                s_last_odom.x_mm;

            const float dy =
                BALL_KICKER_SEARCH_ORIGIN_Y_MM -
                s_last_odom.y_mm;

            const float error_mm =
                sqrtf(dx * dx + dy * dy);

            ESP_LOGI(
                BALL_KICKER_TAG,
                "FINISH_MOVE RETURN ORIGIN DONE: "
                "odom=(%.1f, %.1f, %.1f) "
                "origin=(%.1f, %.1f) "
                "error=(%.1f, %.1f) dist=%.1f",
                (double)s_last_odom.x_mm,
                (double)s_last_odom.y_mm,
                (double)s_last_odom.yaw_deg,
                (double)BALL_KICKER_SEARCH_ORIGIN_X_MM,
                (double)BALL_KICKER_SEARCH_ORIGIN_Y_MM,
                (double)dx,
                (double)dy,
                (double)error_mm);

            if (error_mm <=
                BALL_KICKER_FINISH_RETURN_TOLERANCE_MM) {

                /* 返回原点完成，检查是否需要 yaw 校正 */
#if BALL_KICKER_ENABLE_YAW_CORRECTION
                const float current_yaw = s_last_odom.yaw_deg;
                const float yaw_error = ball_kicker_wrap_deg(current_yaw); // 目标为0，误差为当前值（考虑包裹到±180）
                if (ball_kicker_absf(yaw_error) > BALL_KICKER_YAW_CORRECTION_TOL_DEG) {
                    ESP_LOGI(BALL_KICKER_TAG, "Yaw error %.1f deg, entering YAW_CORRECT", (double)yaw_error);
                    s_finish_phase = BALL_KICKER_FINISH_YAW_CORRECT;
                    s_finish_motion_started = false;
                    return;
                }
#endif
                // 无需校正或校正已禁用
                ball_kicker_set_state(
                    BALL_KICKER_STATE_BALL_DONE);

                return;
            }

            /*
             * 仍有明显误差：
             * 从当前 odometry 再计算一次直接回原点。
             */
            return;
        }

        /* ---- 处理 YAW_CORRECT 阶段完成 ---- */
        if (s_finish_phase ==
            BALL_KICKER_FINISH_YAW_CORRECT) {
            // 旋转完成，检查是否达到目标
            if (!ball_kicker_update_odometry()) {
                ESP_LOGE(BALL_KICKER_TAG, "YAW_CORRECT: odometry unavailable");
                s_ball_kicker_error = true;
                s_ball_kicker_running = false;
                chassis_motion_stop();
                ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
                return;
            }
            const float current_yaw = s_last_odom.yaw_deg;
            const float yaw_error = ball_kicker_wrap_deg(current_yaw);
            if (ball_kicker_absf(yaw_error) <= BALL_KICKER_YAW_CORRECTION_TOL_DEG) {
                ESP_LOGI(BALL_KICKER_TAG, "Yaw corrected to %.1f deg", (double)current_yaw);
                ball_kicker_set_state(BALL_KICKER_STATE_BALL_DONE);
                return;
            } else {
                // 误差仍大，重新旋转（但此处简单处理：重试一次旋转，或直接报错）
                ESP_LOGW(BALL_KICKER_TAG, "Yaw correction incomplete, retrying");
                s_finish_motion_started = false;
                return;
            }
        }
    }

    /* ------------------------------------------------------------
     * 第一阶段：当前车体正后方后撤 200 mm
     * ------------------------------------------------------------ */
    if (s_finish_phase ==
        BALL_KICKER_FINISH_BACKOFF) {

        esp_err_t ret =
            chassis_motion_move_distance(
                180.0f,
                BALL_KICKER_FINISH_BACKOFF_DISTANCE_MM,
                BALL_KICKER_FINISH_BACKOFF_SPEED_MM_S);

        if (ret != ESP_OK) {

            ESP_LOGE(
                BALL_KICKER_TAG,
                "FINISH_MOVE BACKOFF failed: %s",
                esp_err_to_name(ret));

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            chassis_motion_stop();

            ball_kicker_set_state(
                BALL_KICKER_STATE_ERROR);

            return;
        }

        s_finish_motion_started = true;

        ESP_LOGI(
            BALL_KICKER_TAG,
            "FINISH_MOVE BACKOFF START: "
            "dist=%.1f speed=%.1f",
            (double)
                BALL_KICKER_FINISH_BACKOFF_DISTANCE_MM,
            (double)
                BALL_KICKER_FINISH_BACKOFF_SPEED_MM_S);

        return;
    }

    /* ------------------------------------------------------------
     * 第二阶段：从当前 odometry 直接回 WORLD (0, 0)
     * ------------------------------------------------------------ */
    if (s_finish_phase ==
        BALL_KICKER_FINISH_RETURN_ORIGIN) {

        if (!ball_kicker_update_odometry()) {

            ESP_LOGE(
                BALL_KICKER_TAG,
                "FINISH_MOVE RETURN ORIGIN: "
                "odometry unavailable");

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            chassis_motion_stop();

            ball_kicker_set_state(
                BALL_KICKER_STATE_ERROR);

            return;
        }

        const float dx =
            BALL_KICKER_SEARCH_ORIGIN_X_MM -
            s_last_odom.x_mm;

        const float dy =
            BALL_KICKER_SEARCH_ORIGIN_Y_MM -
            s_last_odom.y_mm;

        const float distance_mm =
            sqrtf(dx * dx + dy * dy);

        if (distance_mm <=
            BALL_KICKER_FINISH_RETURN_TOLERANCE_MM) {

            ESP_LOGI(
                BALL_KICKER_TAG,
                "FINISH_MOVE already at SEARCH ORIGIN: "
                "odom=(%.1f, %.1f)",
                (double)s_last_odom.x_mm,
                (double)s_last_odom.y_mm);

            // 检查是否需要 yaw 校正
#if BALL_KICKER_ENABLE_YAW_CORRECTION
            const float current_yaw = s_last_odom.yaw_deg;
            const float yaw_error = ball_kicker_wrap_deg(current_yaw);
            if (ball_kicker_absf(yaw_error) > BALL_KICKER_YAW_CORRECTION_TOL_DEG) {
                ESP_LOGI(BALL_KICKER_TAG, "Yaw error %.1f deg, entering YAW_CORRECT", (double)yaw_error);
                s_finish_phase = BALL_KICKER_FINISH_YAW_CORRECT;
                s_finish_motion_started = false;
                return;
            }
#endif
            ball_kicker_set_state(
                BALL_KICKER_STATE_BALL_DONE);

            return;
        }

        const float direction_world_deg =
            atan2f(dy, dx) *
            57.2957795f;

        const float direction_body_deg =
            ball_kicker_wrap_deg(
                direction_world_deg -
                s_last_odom.yaw_deg);

        esp_err_t ret =
            chassis_motion_move_distance(
                direction_body_deg,
                distance_mm,
                BALL_KICKER_FINISH_RETURN_SPEED_MM_S);

        if (ret != ESP_OK) {

            ESP_LOGE(
                BALL_KICKER_TAG,
                "FINISH_MOVE RETURN ORIGIN failed: %s",
                esp_err_to_name(ret));

            s_ball_kicker_error = true;
            s_ball_kicker_running = false;

            chassis_motion_stop();

            ball_kicker_set_state(
                BALL_KICKER_STATE_ERROR);

            return;
        }

        s_finish_motion_started = true;

        ESP_LOGI(
            BALL_KICKER_TAG,
            "FINISH_MOVE RETURN ORIGIN START: "
            "odom=(%.1f, %.1f, %.1f) "
            "delta=(%.1f, %.1f) "
            "world_dir=%.1f body_dir=%.1f dist=%.1f",
            (double)s_last_odom.x_mm,
            (double)s_last_odom.y_mm,
            (double)s_last_odom.yaw_deg,
            (double)dx,
            (double)dy,
            (double)direction_world_deg,
            (double)direction_body_deg,
            (double)distance_mm);

        return;
    }

    /* ------------------------------------------------------------
     * 第三阶段（新增）：YAW 校正
     * ------------------------------------------------------------ */
#if BALL_KICKER_ENABLE_YAW_CORRECTION
    if (s_finish_phase ==
        BALL_KICKER_FINISH_YAW_CORRECT) {

        // 如果校正尚未启动
        if (!s_finish_motion_started) {
            if (!ball_kicker_update_odometry()) {
                ESP_LOGE(BALL_KICKER_TAG, "YAW_CORRECT: odometry unavailable");
                s_ball_kicker_error = true;
                s_ball_kicker_running = false;
                chassis_motion_stop();
                ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
                return;
            }
            const float current_yaw = s_last_odom.yaw_deg;
            const float yaw_error = ball_kicker_wrap_deg(current_yaw); // 目标0°
            // 如果误差很小，直接跳到完成
            if (ball_kicker_absf(yaw_error) <= BALL_KICKER_YAW_CORRECTION_TOL_DEG) {
                ESP_LOGI(BALL_KICKER_TAG, "Yaw already within tolerance, skip correction");
                ball_kicker_set_state(BALL_KICKER_STATE_BALL_DONE);
                return;
            }
            // 启动旋转，角度为 -current_yaw（注意 chassis_motion_rotate 输入相对角度）
            // 注意：chassis_motion_rotate 的正负号：正 = 逆时针，我们想回到0，所以旋转 -current_yaw
            float rotate_angle = -current_yaw;
            ESP_LOGI(BALL_KICKER_TAG, "YAW_CORRECT: rotating %.1f deg at %.1f deg/s", 
                     (double)rotate_angle, (double)BALL_KICKER_YAW_CORRECTION_SPEED_DEG_S);
            esp_err_t ret = chassis_motion_rotate(
                rotate_angle,
                BALL_KICKER_YAW_CORRECTION_SPEED_DEG_S);
            if (ret != ESP_OK) {
                ESP_LOGE(BALL_KICKER_TAG, "YAW_CORRECT rotate failed: %s", esp_err_to_name(ret));
                s_ball_kicker_error = true;
                s_ball_kicker_running = false;
                chassis_motion_stop();
                ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
                return;
            }
            s_finish_motion_started = true;
            return;
        } else {
            // 已启动，等待完成
            if (chassis_motion_is_busy()) {
                return;
            }
            // 旋转完成，检查是否在容忍内
            if (!ball_kicker_update_odometry()) {
                ESP_LOGE(BALL_KICKER_TAG, "YAW_CORRECT: odometry unavailable after rotate");
                s_ball_kicker_error = true;
                s_ball_kicker_running = false;
                chassis_motion_stop();
                ball_kicker_set_state(BALL_KICKER_STATE_ERROR);
                return;
            }
            const float current_yaw = s_last_odom.yaw_deg;
            const float yaw_error = ball_kicker_wrap_deg(current_yaw);
            if (ball_kicker_absf(yaw_error) <= BALL_KICKER_YAW_CORRECTION_TOL_DEG) {
                ESP_LOGI(BALL_KICKER_TAG, "YAW_CORRECT done, yaw=%.1f", (double)current_yaw);
                ball_kicker_set_state(BALL_KICKER_STATE_BALL_DONE);
                return;
            } else {
                // 如果误差仍大，可以重试（这里简单再发一次）
                ESP_LOGW(BALL_KICKER_TAG, "YAW_CORRECT incomplete (%.1f), retrying", (double)yaw_error);
                s_finish_motion_started = false;  // 重新启动
                return;
            }
        }
    }
#endif
}

/* ============================================================
 * REACQUIRE
 * ============================================================ */

static void ball_kicker_update_reacquire(
    const ball_target_result_t *target)
{
    if (target != NULL &&
        target->detected &&
        target->state !=
            BALL_TARGET_STATE_LOST) {

        s_last_ball_seen_us =
            esp_timer_get_time();

        ball_kicker_set_state(
            BALL_KICKER_STATE_APPROACH);

        return;
    }

    const int64_t now_us =
        esp_timer_get_time();

    const int64_t elapsed_us =
        now_us -
        s_reacquire_start_us;

    if (elapsed_us >=
        ((int64_t)
         BALL_KICKER_REACQUIRE_TIMEOUT_MS *
         1000LL)) {

        ESP_LOGW(
            BALL_KICKER_TAG,
            "BALL%d reacquire timeout -> resume scan",
            (int)s_current_ball + 1);

        ball_kicker_direct_stop();
        s_resume_search_after_reacquire = true;
        ball_kicker_set_state(
            BALL_KICKER_STATE_SEARCH);

        return;
    }

    const int64_t phase_elapsed_us =
        now_us - s_reacquire_phase_start_us;

    switch (s_reacquire_phase) {
        case REACQUIRE_SETTLE:
            ball_kicker_direct_stop();
            if (phase_elapsed_us >=
                ((int64_t)BALL_KICKER_REACQUIRE_SETTLE_MS * 1000LL)) {
                s_reacquire_phase = REACQUIRE_RETURN_LAST_VIEW;
                s_reacquire_phase_start_us = now_us;
                ESP_LOGI(BALL_KICKER_TAG, "REACQUIRE: return to last view");
            }
            break;

        case REACQUIRE_RETURN_LAST_VIEW: {
            if (!s_last_seen_odom_valid || !ball_kicker_update_odometry()) {
                s_reacquire_phase = REACQUIRE_SWEEP_PRIMARY;
                s_reacquire_phase_start_us = now_us;
                break;
            }

            if (s_reacquire_motion_started) {
                if (chassis_motion_is_busy()) {
                    return;
                }
                s_reacquire_motion_started = false;
                s_reacquire_phase = REACQUIRE_SWEEP_PRIMARY;
                s_reacquire_phase_start_us = now_us;
                ESP_LOGI(BALL_KICKER_TAG, "REACQUIRE: last-view return done -> sweep");
                break;
            }

            const float dx = s_last_seen_odom.x_mm - s_last_odom.x_mm;
            const float dy = s_last_seen_odom.y_mm - s_last_odom.y_mm;
            const float distance_mm = sqrtf(dx * dx + dy * dy);
            if (distance_mm <= BALL_KICKER_REACQUIRE_RETURN_TOLERANCE_MM) {
                s_reacquire_phase = REACQUIRE_SWEEP_PRIMARY;
                s_reacquire_phase_start_us = now_us;
                break;
            }

            const float world_dir_deg = atan2f(dy, dx) * 57.2957795f;
            const float body_dir_deg = ball_kicker_wrap_deg(
                world_dir_deg - s_last_odom.yaw_deg);
            const esp_err_t ret = chassis_motion_move_distance(
                body_dir_deg, distance_mm,
                BALL_KICKER_REACQUIRE_MOVE_SPEED_MM_S);
            if (ret != ESP_OK) {
                ESP_LOGW(BALL_KICKER_TAG,
                         "REACQUIRE return unavailable: %s -> sweep",
                         esp_err_to_name(ret));
                s_reacquire_phase = REACQUIRE_SWEEP_PRIMARY;
                s_reacquire_phase_start_us = now_us;
                break;
            }
            s_reacquire_motion_started = true;
            ESP_LOGI(BALL_KICKER_TAG,
                     "REACQUIRE: return %.0f mm to last view", (double)distance_mm);
            break;
        }

        case REACQUIRE_SWEEP_PRIMARY:
        case REACQUIRE_SWEEP_SECONDARY: {
            const bool primary = (s_reacquire_phase == REACQUIRE_SWEEP_PRIMARY);
            const float preferred_wz =
                (s_last_seen_corrected_x_px >= 0.0f) ?
                -BALL_APPROACH_SEARCH_WZ : BALL_APPROACH_SEARCH_WZ;
            ball_kicker_set_motion(0.0f, 0.0f,
                                   primary ? preferred_wz : -preferred_wz);
            if (phase_elapsed_us >=
                ((int64_t)BALL_KICKER_REACQUIRE_SWEEP_MS * 1000LL)) {
                ball_kicker_direct_stop();
                s_reacquire_phase = primary ? REACQUIRE_ADVANCE : REACQUIRE_SETTLE;
                s_reacquire_phase_start_us = now_us;
                if (!primary) {
                    s_resume_search_after_reacquire = true;
                    ball_kicker_set_state(BALL_KICKER_STATE_SEARCH);
                }
            }
            break;
        }

        case REACQUIRE_ADVANCE:
            if (!s_reacquire_motion_started) {
                const esp_err_t ret = chassis_motion_move_distance(
                    0.0f, BALL_KICKER_REACQUIRE_ADVANCE_MM,
                    BALL_KICKER_REACQUIRE_MOVE_SPEED_MM_S);
                if (ret != ESP_OK) {
                    ESP_LOGW(BALL_KICKER_TAG,
                             "REACQUIRE advance unavailable: %s", esp_err_to_name(ret));
                    s_reacquire_phase = REACQUIRE_SWEEP_SECONDARY;
                    s_reacquire_phase_start_us = now_us;
                    break;
                }
                s_reacquire_motion_started = true;
                break;
            }
            if (!chassis_motion_is_busy()) {
                s_reacquire_motion_started = false;
                s_reacquire_phase = REACQUIRE_SWEEP_SECONDARY;
                s_reacquire_phase_start_us = now_us;
            }
            break;
    }
}

/* ============================================================
 * 当前球完成
 *
 * 注意：
 *     FINISH_MOVE 已经保证机器人回到原搜索区域，
 *     所以这里才开始下一颗球 SEARCH。
 * ============================================================ */

static void ball_kicker_update_ball_done(void)
{
    ball_kicker_direct_stop();

    s_balls_completed++;

    ESP_LOGI(
        BALL_KICKER_TAG,
        "BALL%d completed, total=%u",
        (int)s_current_ball + 1,
        (unsigned)s_balls_completed);

    if (s_balls_completed >= 2) {
        s_ball_kicker_running = false;
        s_ball_kicker_finished = true;

        ball_kicker_set_state(
            BALL_KICKER_STATE_FINISHED);

        ESP_LOGI(
            BALL_KICKER_TAG,
            "ALL BALLS COMPLETED");

        return;
    }

    s_current_ball =
        BALL_KICKER_BALL2;

    ball_target_reset();
    ball_approach_reset();

    s_last_ball_seen_us =
        esp_timer_get_time();

    ball_kicker_get_bag_position(
        &s_current_bag_x_mm,
        &s_current_bag_y_mm);

    ball_kicker_set_state(
        BALL_KICKER_STATE_SEARCH);
}

/* ============================================================
 * 主更新函数
 *
 * 每次：
 *     1. 获取视觉
 *     2. 更新 target
 *     3. 缓存 odometry
 *     4. 执行状态机
 * ============================================================ */

static bool ball_kicker_update(void)
{
    if (!s_ball_kicker_running)
        return false;

    colorball_vision_data_t vision = {0};
    ball_target_result_t target = {0};

    colorball_vision_get_data(&vision);

    if (!ball_target_update(
            &vision,
            &target)) {

        ESP_LOGW(
            BALL_KICKER_TAG,
            "ball_target_update failed");
    }

    s_last_vision = vision;
    s_last_target = target;

    if (target.detected &&
        target.state !=
            BALL_TARGET_STATE_LOST) {

        s_last_ball_seen_us =
            esp_timer_get_time();

        if (ball_kicker_update_odometry()) {
            s_last_seen_odom = s_last_odom;
            s_last_seen_odom_valid = true;
            s_last_seen_corrected_x_px = target.corrected_x_px;
        }
    }

    switch (s_ball_kicker_state) {
        case BALL_KICKER_STATE_SEARCH:
            ball_kicker_update_search(
                &target);
            break;

        case BALL_KICKER_STATE_APPROACH:
            ball_kicker_update_approach(
                &target);
            break;

        case BALL_KICKER_STATE_TO_BAG:
            ball_kicker_update_to_bag(
                &target);
            break;

        case BALL_KICKER_STATE_PUSH:
            ball_kicker_update_push(
                &target);
            break;

        case BALL_KICKER_STATE_FINISH_MOVE:
            ball_kicker_update_finish_move();
            break;

        case BALL_KICKER_STATE_REACQUIRE:
            ball_kicker_update_reacquire(
                &target);
            break;

        case BALL_KICKER_STATE_BALL_DONE:
            ball_kicker_update_ball_done();
            break;

        case BALL_KICKER_STATE_FINISHED:
        case BALL_KICKER_STATE_ERROR:
            ball_kicker_direct_stop();
            break;

        case BALL_KICKER_STATE_IDLE:
        default:
            ball_kicker_direct_stop();
            break;
    }

    return true;
}

/* ============================================================
 * 状态查询
 * ============================================================ */

static ball_kicker_state_t
ball_kicker_get_state(void)
{
    return s_ball_kicker_state;
}

static const char *
ball_kicker_get_state_string(void)
{
    return ball_kicker_state_to_string(
        s_ball_kicker_state);
}

static ball_kicker_ball_id_t
ball_kicker_get_current_ball(void)
{
    return s_current_ball;
}

static bool ball_kicker_is_running(void)
{
    return s_ball_kicker_running;
}

static bool ball_kicker_is_finished(void)
{
    return s_ball_kicker_finished;
}

static bool ball_kicker_is_error(void)
{
    return s_ball_kicker_error;
}

static uint8_t
ball_kicker_get_completed_count(void)
{
    return s_balls_completed;
}

/* ============================================================
 * 获取完整状态
 * ============================================================ */

static bool ball_kicker_get_status(
    ball_kicker_status_t *out_status)
{
    if (out_status == NULL)
        return false;

    out_status->state =
        s_ball_kicker_state;

    out_status->current_ball =
        s_current_ball;

    out_status->running =
        s_ball_kicker_running;

    out_status->finished =
        s_ball_kicker_finished;

    out_status->error =
        s_ball_kicker_error;

    out_status->ball_detected =
        s_last_target.detected;

    out_status->ball_reached =
        s_last_target.reached;

    out_status->position =
        s_last_target.position;

    out_status->corrected_x_px =
        s_last_target.corrected_x_px;

    out_status->approach_command =
        s_last_approach_command;

    out_status->balls_completed =
        s_balls_completed;

    out_status->odom_x_mm =
        s_last_odom.x_mm;

    out_status->odom_y_mm =
        s_last_odom.y_mm;

    out_status->odom_yaw_deg =
        s_last_odom.yaw_deg;

    out_status->bag_x_mm =
        s_current_bag_x_mm;

    out_status->bag_y_mm =
        s_current_bag_y_mm;

    out_status->bag_dx_mm =
        s_bag_dx_mm;

    out_status->bag_dy_mm =
        s_bag_dy_mm;

    return true;
}

/* ============================================================
 * Reset
 * ============================================================ */

static esp_err_t ball_kicker_reset(void)
{
    ball_kicker_direct_stop();
    chassis_motion_stop();

    esp_err_t ret;

    ret = ball_target_reset();

    if (ret != ESP_OK)
        return ret;

    ret = ball_approach_reset();

    if (ret != ESP_OK)
        return ret;

    s_ball_kicker_state =
        BALL_KICKER_STATE_IDLE;

    s_current_ball =
        BALL_KICKER_BALL1;

    s_ball_kicker_running = false;
    s_ball_kicker_finished = false;
    s_ball_kicker_error = false;

    s_motion_started = false;

    s_scan_phase =
        SCAN_PHASE_IDLE;

    s_scan_forward_row = 0;
    s_scan_motion_started = false;

    s_finish_phase =
        BALL_KICKER_FINISH_BACKOFF;

    s_finish_motion_started = false;

    s_push_start_us = 0;
    s_last_ball_seen_us = 0;
    s_reacquire_start_us = 0;

    s_balls_completed = 0;

    s_last_vision =
        (colorball_vision_data_t){0};

    s_last_target =
        (ball_target_result_t){0};

    s_last_approach_command =
        (ball_approach_command_t){0};

    s_last_odom =
        (chassis_odometry_state_t){0};

    s_current_bag_x_mm = 0.0f;
    s_current_bag_y_mm = 0.0f;

    s_bag_dx_mm = 0.0f;
    s_bag_dy_mm = 0.0f;

    return ESP_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* BALL_KICKER_H */
