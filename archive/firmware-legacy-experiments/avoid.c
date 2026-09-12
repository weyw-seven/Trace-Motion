#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"

#define CHASSIS_KINEMATICS_IMPLEMENTATION
#include "chassis_kinematics.h"

#define CHASSIS_ODOMETRY_IMPLEMENTATION
#include "chassis_odometry.h"

#define CHASSIS_MOTION_IMPLEMENTATION
#include "chassis_motion.h"

#define INFRARED_SENSOR_IMPLEMENTATION
#include "infrared_sensor.h"

#define LINE_TRACKER_IMPLEMENTATION
#include "line_tracker.h"

#define ULTRASONIC_SENSOR_IMPLEMENTATION
#include "ultrasonic_sensor.h"

#define OBSTACLE_AVOID_IMPLEMENTATION
#include "obstacle_avoid.h"

#define OLED_DISPLAY_IMPLEMENTATION
#include "oled_display.h"

static const char *TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "System starting...");

    /* 1. 电机控制（必须最先） */
    ESP_ERROR_CHECK(motor_control_init());
    ESP_LOGI(TAG, "Motor initialized");

    /* 2. MPU6050（航向角，用于里程计） */
    ESP_ERROR_CHECK(mpu6050_init());
    ESP_LOGI(TAG, "MPU6050 initialized");

    /* 3. 里程计初始化 */
    ESP_ERROR_CHECK(chassis_odometry_init());
    ESP_LOGI(TAG, "Odometry initialized");

    /* 4. 重置里程计到原点（重要！） */
    ESP_ERROR_CHECK(chassis_odometry_reset(0.0f, 0.0f, 0.0f));
    ESP_LOGI(TAG, "Odometry reset");

    /* 5. 运动规划器初始化（依赖里程计） */
    ESP_ERROR_CHECK(chassis_motion_init());
    ESP_LOGI(TAG, "Motion planner initialized");

    /* test why extra left not done */
    chassis_odometry_state_t odom;
    chassis_odometry_get_state(&odom);
    ESP_LOGI(TAG, "Odometry: init=%d, x=%.1f, y=%.1f, yaw=%.2f, ts=%lld", 
            odom.initialized, odom.x_mm, odom.y_mm, odom.yaw_deg, (long long)odom.timestamp_us);

    /* 6. 巡线（内部会初始化红外传感器） */
    ESP_ERROR_CHECK(line_tracker_init());
    ESP_LOGI(TAG, "Line tracker initialized");

    /* 7. 超声波 */
    ESP_ERROR_CHECK(ultrasonic_sensor_init());
    ESP_ERROR_CHECK(ultrasonic_sensor_start());
    ESP_LOGI(TAG, "Ultrasonic initialized");

    /* 8. 避障模块（依赖 line_tracker、ultrasonic、chassis_motion） */
    ESP_ERROR_CHECK(obstacle_avoid_init());
    ESP_ERROR_CHECK(obstacle_avoid_start());
    ESP_LOGI(TAG, "Obstacle avoid started");

    ESP_ERROR_CHECK(oled_display_init());
    ESP_ERROR_CHECK(oled_display_start());
    ESP_LOGI(TAG,"OLED display started");

    // 所有 init 后
    vTaskDelay(pdMS_TO_TICKS(2000)); // 等待里程计稳定

    /* 9. 启动巡线（默认启用） */
    line_tracker_start();

    ESP_LOGI(TAG, "System ready, line tracking started");

    /* 主循环空转 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}