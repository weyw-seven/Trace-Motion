/**
 * @file main.c
 * @brief 电机控制测试程序 – 演示 Z 字形、S 形及前后左右切换
 *
 * 依赖 motor_control.h 提供的 API，需确保该头文件已正确包含且实现已编译。
 * 测试流程：
 *   - 前、后、左、右、原地左转、原地右转（各持续 3 秒）
 *   - Z 字形：前进 + 横向正弦摆动（持续 6 秒，完成一个完整周期）
 *   - S 形：前进 + 转向角正弦变化（持续 6 秒）
 *   - 循环上述步骤
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

// 标签用于日志输出
static const char *TAG = "MOTOR_TEST";

// 每个阶段的持续时间（秒）
#define STAGE_DURATION_SEC  3.0f
#define ZIGZAG_DURATION_SEC 6.0f
#define S_DURATION_SEC      6.0f

// 循环更新间隔（毫秒）
#define LOOP_INTERVAL_MS    10

// 运动幅度（归一化 [-1, 1]）
#define SPEED_FWD           0.5f
#define SPEED_LAT           0.5f
#define SPEED_ROT           0.8f

// 阶段枚举
typedef enum {
    STAGE_FORWARD,
    STAGE_BACKWARD,
    STAGE_LEFT,
    STAGE_RIGHT,
    STAGE_ROT_CW,          // 顺时针旋转 (w 负)
    STAGE_ROT_CCW,         // 逆时针旋转 (w 正)
    STAGE_ZIGZAG,
    STAGE_S,
    STAGE_COUNT
} test_stage_t;

void app_main(void)
{
    // 1. 初始化电机控制
    ESP_LOGI(TAG, "Initializing motor control...");
    esp_err_t ret = motor_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor initialization failed (err=0x%x)", ret);
        return;
    }

    // 确保使能（默认已使能，此处显式调用无副作用）
    motor_control_enable(true);

    // 等待后台任务启动
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. 主循环，按阶段执行不同运动
    test_stage_t stage = STAGE_FORWARD;
    float stage_time = 0.0f;          // 当前阶段已运行时间（秒）
    float stage_duration = STAGE_DURATION_SEC;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOOP_INTERVAL_MS);

    while (1) {
        // 根据当前阶段设置命令
        float vx = 0.0f, vy = 0.0f, w = 0.0f;

        switch (stage) {
            case STAGE_FORWARD:
                vx = SPEED_FWD;
                break;
            case STAGE_BACKWARD:
                vx = -SPEED_FWD;
                break;
            case STAGE_LEFT:
                vy = SPEED_LAT;
                break;
            case STAGE_RIGHT:
                vy = -SPEED_LAT;
                break;
            case STAGE_ROT_CW:
                w = -SPEED_ROT;        // 顺时针
                break;
            case STAGE_ROT_CCW:
                w = SPEED_ROT;         // 逆时针
                break;
            case STAGE_ZIGZAG: {
                // Z 字形：前进同时横向摆动，完成一个完整正弦周期
                float t = stage_time / stage_duration;          // 0 → 1
                float swing = 0.8f * sinf(2.0f * M_PI * t);
                vx = SPEED_FWD;
                vy = swing;
                break;
            }
            case STAGE_S: {
                // S 形：前进同时转向角按正弦变化，形成 S 轨迹
                float t = stage_time / stage_duration;          // 0 → 1
                float turn = 0.6f * sinf(2.0f * M_PI * t);
                vx = SPEED_FWD;
                w = turn;
                break;
            }
            default:
                break;
        }

        // 发送命令
        Set_motor(vx, vy, w);

        // 打印当前状态（每 1 秒打印一次，避免刷屏）
        static int print_counter = 0;
        if (++print_counter >= (1000 / LOOP_INTERVAL_MS)) {
            print_counter = 0;
            const char *stage_names[] = {
                "FORWARD", "BACKWARD", "LEFT", "RIGHT",
                "ROT_CW", "ROT_CCW", "ZIGZAG", "S"
            };
            ESP_LOGI(TAG, "Stage: %s, vx=%.2f, vy=%.2f, w=%.2f, t=%.1f/%.1f",
                     stage_names[stage], vx, vy, w, stage_time, stage_duration);
        }

        // 更新时间
        vTaskDelayUntil(&last_wake, period);
        stage_time += (float)LOOP_INTERVAL_MS / 1000.0f;

        // 阶段切换
        if (stage_time >= stage_duration) {
            stage_time = 0.0f;
            stage = (stage + 1) % STAGE_COUNT;

            // 为特殊阶段设置不同时长
            if (stage == STAGE_ZIGZAG || stage == STAGE_S) {
                stage_duration = (stage == STAGE_ZIGZAG) ? ZIGZAG_DURATION_SEC : S_DURATION_SEC;
            } else {
                stage_duration = STAGE_DURATION_SEC;
            }

            // 切换时短暂停止，便于观察动作变化（可选）
            // motor_stop();
            // vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}