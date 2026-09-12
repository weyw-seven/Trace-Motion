/*
 * test_chassis_motion_pd.c
 *
 * Experimental chassis-motion test using chassis_motion_pd.h.  The project
 * CMake file still builds only main.c; copy this file to main.c before
 * flashing the test.
 *
 * This test intentionally uses normal velocity APIs only.  It does not write
 * raw PWM.  The PD variant enables cross-track correction, filters the
 * measured cross-track velocity, and keeps the combined translation within
 * the requested speed limit.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

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


static const char *TAG = "motion_pd_test";


/* Clearly editable test settings. */
#ifndef MOTION_PD_TEST_EXTERNAL_TIMEOUT_MS
#define MOTION_PD_TEST_EXTERNAL_TIMEOUT_MS 15000U
#endif

#ifndef MOTION_PD_TEST_LOG_PERIOD_MS
#define MOTION_PD_TEST_LOG_PERIOD_MS 100U
#endif

#ifndef MOTION_PD_TEST_PAUSE_MS
#define MOTION_PD_TEST_PAUSE_MS 1000U
#endif


typedef struct
{
    const char *name;
    float direction_deg;
    float distance_mm;
    float max_speed_mm_s;
} motion_pd_case_t;


static const motion_pd_case_t g_cases[] =
{
    {"PD_X_100MM",       0.0f,   100.0f, 200.0f},
    {"PD_Y_100MM",      90.0f,   100.0f, 200.0f},
    {"PD_DIAG_100MM",   45.0f,   100.0f, 200.0f},
    {"PD_X_25MM_FINAL",  0.0f,    25.0f, 300.0f},
    {"PD_BACK_80MM",   180.0f,    80.0f, 200.0f},
};


static const char *mode_name(chassis_motion_mode_t mode)
{
    switch (mode)
    {
        case CHASSIS_MOTION_MODE_IDLE:
            return "IDLE";
        case CHASSIS_MOTION_MODE_MOVE_DISTANCE:
            return "MOVE";
        case CHASSIS_MOTION_MODE_ROTATE:
            return "ROTATE";
        case CHASSIS_MOTION_MODE_DONE:
            return "DONE";
        case CHASSIS_MOTION_MODE_CANCELLED:
            return "CANCELLED";
        case CHASSIS_MOTION_MODE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}


static const char *error_name(chassis_motion_error_t error)
{
    switch (error)
    {
        case CHASSIS_MOTION_ERROR_NONE:
            return "NONE";
        case CHASSIS_MOTION_ERROR_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";
        case CHASSIS_MOTION_ERROR_TIMEOUT:
            return "TIMEOUT";
        case CHASSIS_MOTION_ERROR_MOTOR_NOT_READY:
            return "MOTOR_NOT_READY";
        case CHASSIS_MOTION_ERROR_MPU_NOT_READY:
            return "MPU_NOT_READY";
        case CHASSIS_MOTION_ERROR_ODOMETRY_NOT_READY:
            return "ODOMETRY_NOT_READY";
        case CHASSIS_MOTION_ERROR_ODOMETRY_STALE:
            return "ODOMETRY_STALE";
        default:
            return "OTHER";
    }
}


static void log_live_status(
    const motion_pd_case_t *test_case)
{
    chassis_motion_status_t motion = {0};
    chassis_odometry_state_t odom = {0};
    motor_status_t motor = {0};

    chassis_motion_get_status(&motion);
    chassis_odometry_get_state(&odom);
    motor_get_status(&motor);

    const float direction_rad =
        test_case->direction_deg *
        CHASSIS_MOTION_DEG_TO_RAD;

    const float perp_x =
        -sinf(direction_rad);

    const float perp_y =
        cosf(direction_rad);

    const float cross_rate =
        odom.world_vx_mm_s * perp_x
        +
        odom.world_vy_mm_s * perp_y;

    ESP_LOGI(
        TAG,
        "LIVE %s mode=%s busy=%d err=%s "
        "pose=(%+.1f,%+.1f,%+.2fdeg) "
        "along=%+.1f rem=%+.1f cross=%+.1f cross_rate=%+.1f "
        "world_v=(%+.1f,%+.1f) cmd=(%+.1f,%+.1f,%+.3f) "
        "wheel=A(%+.1f/%+.1f/%+.1f/%ld) "
        "B(%+.1f/%+.1f/%+.1f/%ld) "
        "D(%+.1f/%+.1f/%+.1f/%ld)",
        test_case->name,
        mode_name(motion.mode),
        (int)motion.busy,
        error_name(motion.error),
        (double)odom.x_mm,
        (double)odom.y_mm,
        (double)odom.yaw_deg,
        (double)motion.traveled_distance_mm,
        (double)motion.remaining_distance_mm,
        (double)motion.cross_track_error_mm,
        (double)cross_rate,
        (double)odom.world_vx_mm_s,
        (double)odom.world_vy_mm_s,
        (double)motion.command_vx_mm_s,
        (double)motion.command_vy_mm_s,
        (double)motion.command_w_rad_s,
        (double)motor.A.target_speed_mm_s,
        (double)motor.A.actual_speed_mm_s,
        (double)motor.A.output_pwm,
        (long)motor.A.encoder_count,
        (double)motor.B.target_speed_mm_s,
        (double)motor.B.actual_speed_mm_s,
        (double)motor.B.output_pwm,
        (long)motor.B.encoder_count,
        (double)motor.D.target_speed_mm_s,
        (double)motor.D.actual_speed_mm_s,
        (double)motor.D.output_pwm,
        (long)motor.D.encoder_count);
}


static bool run_case(const motion_pd_case_t *test_case)
{
    if (test_case == NULL)
    {
        return false;
    }

    chassis_motion_stop();
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(MOTION_PD_TEST_PAUSE_MS));

    const esp_err_t reset_ret =
        chassis_odometry_reset(0.0f, 0.0f, 0.0f);

    if (reset_ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "CASE %s odometry reset failed: %s",
            test_case->name,
            esp_err_to_name(reset_ret));
        return false;
    }

    ESP_LOGI(
        TAG,
        "CASE START %s dir=%.1f distance=%.1f speed=%.1f "
        "cross_kp=%.3f cross_kd=%.3f cross_tol=%.1f",
        test_case->name,
        (double)test_case->direction_deg,
        (double)test_case->distance_mm,
        (double)test_case->max_speed_mm_s,
        (double)CHASSIS_MOTION_CROSS_KP,
        (double)CHASSIS_MOTION_CROSS_KD,
        (double)CHASSIS_MOTION_CROSS_TOL_MM);

    const esp_err_t api_ret =
        chassis_motion_move_distance(
            test_case->direction_deg,
            test_case->distance_mm,
            test_case->max_speed_mm_s);

    if (api_ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "CASE END %s rejected: %s",
            test_case->name,
            esp_err_to_name(api_ret));
        return false;
    }

    const int64_t start_us = esp_timer_get_time();
    const int64_t timeout_us =
        (int64_t)MOTION_PD_TEST_EXTERNAL_TIMEOUT_MS * 1000LL;

    while (chassis_motion_is_busy())
    {
        if ((esp_timer_get_time() - start_us) >= timeout_us)
        {
            ESP_LOGE(
                TAG,
                "CASE %s external timeout; emergency stop",
                test_case->name);
            chassis_motion_emergency_stop();
            break;
        }

        log_live_status(test_case);
        vTaskDelay(pdMS_TO_TICKS(MOTION_PD_TEST_LOG_PERIOD_MS));
    }

    chassis_motion_status_t final_status = {0};
    chassis_odometry_state_t final_odom = {0};
    chassis_motion_get_status(&final_status);
    chassis_odometry_get_state(&final_odom);

    const bool pass =
        (final_status.mode == CHASSIS_MOTION_MODE_DONE) &&
        final_status.target_reached &&
        (final_status.error == CHASSIS_MOTION_ERROR_NONE);

    ESP_LOGI(
        TAG,
        "CASE END %s mode=%s error=%s pass=%d "
        "pose=(%+.1f,%+.1f,%+.2fdeg) along=%+.1f rem=%+.1f cross=%+.1f "
        "elapsed=%.2f s",
        test_case->name,
        mode_name(final_status.mode),
        error_name(final_status.error),
        (int)pass,
        (double)final_odom.x_mm,
        (double)final_odom.y_mm,
        (double)final_odom.yaw_deg,
        (double)final_status.traveled_distance_mm,
        (double)final_status.remaining_distance_mm,
        (double)final_status.cross_track_error_mm,
        (double)final_status.elapsed_time_us / 1000000.0);

    chassis_motion_stop();
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(MOTION_PD_TEST_PAUSE_MS));

    return pass;
}


static void motion_pd_test_task(void *arg)
{
    (void)arg;

    ESP_LOGI(
        TAG,
        "START cross-track PD test: enabled=%d kp=%.3f kd=%.3f "
        "tol=%.1f max_cross=%.1f filter=%.2f",
        (int)CHASSIS_MOTION_ENABLE_CROSS_PD,
        (double)CHASSIS_MOTION_CROSS_KP,
        (double)CHASSIS_MOTION_CROSS_KD,
        (double)CHASSIS_MOTION_CROSS_TOL_MM,
        (double)CHASSIS_MOTION_CROSS_SPEED_MAX_MM_S,
        (double)CHASSIS_MOTION_CROSS_RATE_FILTER_ALPHA);

    ESP_LOGW(
        TAG,
        "Active chassis test. Keep chassis level and clear; pen must be safe.");

    esp_err_t ret = motor_control_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "motor_control_init: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = mpu6050_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050_init: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = chassis_odometry_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "chassis_odometry_init: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = chassis_odometry_reset(0.0f, 0.0f, 0.0f);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "chassis_odometry_reset: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = chassis_motion_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "chassis_motion_init: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Keep chassis still for MPU startup calibration");
    vTaskDelay(pdMS_TO_TICKS(2500U));

    uint32_t pass_count = 0U;
    const uint32_t case_count =
        (uint32_t)(sizeof(g_cases) / sizeof(g_cases[0]));

    for (uint32_t i = 0U; i < case_count; ++i)
    {
        if (run_case(&g_cases[i]))
        {
            pass_count++;
        }
    }

    chassis_motion_stop();
    motor_stop();

    ESP_LOGI(
        TAG,
        "SUMMARY pass=%lu/%lu",
        (unsigned long)pass_count,
        (unsigned long)case_count);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}


void app_main(void)
{
    const BaseType_t created =
        xTaskCreate(
            motion_pd_test_task,
            "motion_pd_test",
            6144,
            NULL,
            5,
            NULL);

    if (created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create motion PD test task");
    }
}
