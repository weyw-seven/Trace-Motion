/*
 * odometry_push_monitor.c
 *
 * Passive hand-push odometry validation.
 *
 * Purpose:
 *   Verify the complete passive odometry chain without active motor motion:
 *
 *     hand push
 *       -> wheel encoders
 *       -> wheel travel
 *       -> chassis forward kinematics
 *       -> body dx/dy
 *       -> MPU yaw
 *       -> world x/y
 *
 * Motors are explicitly disabled (TB6612 STBY low), while the encoder
 * PCNT units and motor-control background task remain active.
 *
 * Recommended use:
 *   - Put the chassis on the floor.
 *   - Keep it level/still during MPU initialization/calibration.
 *   - After "PUSH TEST READY", push it manually.
 *   - For a clean quantitative test, reboot/reset and test one motion
 *     direction per run.
 *
 * Expected coordinate convention:
 *   +X : forward
 *   +Y : left
 *   +Yaw : counter-clockwise (viewed from above)
 *
 * The shared tm_chassis component owns all low-level implementations.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "motor_control.h"

#include "mpu6050.h"

#include "chassis_kinematics.h"

#include "chassis_odometry.h"


/* =====================================================================
 * Configuration
 * ===================================================================== */

/*
 * 100 ms = 10 Hz terminal refresh.
 * Odometry itself still runs at CHASSIS_ODOMETRY_PERIOD_MS (normally 10 ms).
 */
#ifndef ODOM_PUSH_PRINT_PERIOD_MS
#define ODOM_PUSH_PRINT_PERIOD_MS             100U
#endif

/*
 * Wait after odometry reset before declaring the test ready.
 */
#ifndef ODOM_PUSH_POST_RESET_SETTLE_MS
#define ODOM_PUSH_POST_RESET_SETTLE_MS        500U
#endif

/*
 * Warn if motors somehow become enabled again.
 */
#ifndef ODOM_PUSH_VERIFY_MOTOR_DISABLED
#define ODOM_PUSH_VERIFY_MOTOR_DISABLED         1
#endif


static const char *TAG = "ODOM_PUSH";


/* =====================================================================
 * Helpers
 * ===================================================================== */

static void print_test_instructions(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG,
        "============================================================");
    ESP_LOGI(TAG,
        "PASSIVE HAND-PUSH ODOMETRY MONITOR");
    ESP_LOGI(TAG,
        "============================================================");

    ESP_LOGI(TAG,
        "Coordinate convention:");
    ESP_LOGI(TAG,
        "  +X = chassis forward");
    ESP_LOGI(TAG,
        "  +Y = chassis left");
    ESP_LOGI(TAG,
        "  +Yaw = CCW viewed from above");

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG,
        "Suggested one-motion-per-run tests:");
    ESP_LOGI(TAG,
        "  1) Push forward  200 mm -> X ~= +200 mm, Y ~= 0");
    ESP_LOGI(TAG,
        "  2) Push backward 200 mm -> X ~= -200 mm, Y ~= 0");
    ESP_LOGI(TAG,
        "  3) Push left     200 mm -> Y ~= +200 mm, X ~= 0");
    ESP_LOGI(TAG,
        "  4) Push right    200 mm -> Y ~= -200 mm, X ~= 0");
    ESP_LOGI(TAG,
        "  5) Rotate CCW ~90 deg -> Yaw ~= +90 deg");
    ESP_LOGI(TAG,
        "  6) Rotate CW  ~90 deg -> Yaw ~= -90 deg");

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG,
        "For distance tests, try to keep heading near 0 deg.");
    ESP_LOGI(TAG,
        "For best accuracy, mark 200 mm on the floor with a ruler/tape.");
    ESP_LOGI(TAG,
        "============================================================");
}


/* =====================================================================
 * Main
 * ===================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG,
        "############################################################");
    ESP_LOGI(TAG,
        "# Passive hand-push odometry test");
    ESP_LOGI(TAG,
        "############################################################");

    /*
     * -------------------------------------------------------------
     * 1. Motor subsystem
     *
     * We need motor_control running because it owns/updates the encoder
     * PCNT counts and publishes motor_status_t.
     *
     * Then explicitly disable the driver output so the wheels are never
     * actively driven by this test.
     * -------------------------------------------------------------
     */
    esp_err_t ret =
        motor_control_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
            "motor_control_init failed: %s",
            esp_err_to_name(ret));
        return;
    }

    motor_stop();

    vTaskDelay(
        pdMS_TO_TICKS(50U));

    motor_control_enable(false);

    vTaskDelay(
        pdMS_TO_TICKS(100U));

    motor_status_t motor_status = {0};
    motor_get_status(&motor_status);

    ESP_LOGI(TAG,
        "MOTOR | initialized=%d enabled=%d closed_loop_ready=%d",
        (int)motor_status.initialized,
        (int)motor_status.enabled,
        (int)motor_status.closed_loop_ready);

    if (!motor_status.initialized ||
        !motor_status.closed_loop_ready)
    {
        ESP_LOGE(TAG,
            "Motor encoder subsystem is not ready.");
        return;
    }

    if (motor_status.enabled)
    {
        ESP_LOGE(TAG,
            "Motor driver is still enabled; aborting passive push test.");
        motor_emergency_stop();
        motor_control_enable(false);
        return;
    }

    /*
     * -------------------------------------------------------------
     * 2. MPU6050
     * -------------------------------------------------------------
     */
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG,
        "KEEP CHASSIS LEVEL AND COMPLETELY STILL");
    ESP_LOGW(TAG,
        "during MPU startup settling and calibration.");

    ret =
        mpu6050_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
            "mpu6050_init failed: %s",
            esp_err_to_name(ret));
        return;
    }

    vTaskDelay(
        pdMS_TO_TICKS(500U));

    mpu6050_status_t imu_status = {0};
    mpu6050_get_status(&imu_status);

    ESP_LOGI(TAG,
        "IMU | initialized=%d calibrated=%d sampling=%d "
        "samples=%" PRIu32 " errors=%" PRIu32,
        (int)imu_status.initialized,
        (int)imu_status.calibrated,
        (int)imu_status.sampling,
        imu_status.sample_count,
        imu_status.read_error_count);

    if (!imu_status.initialized ||
        !imu_status.calibrated ||
        !imu_status.sampling)
    {
        ESP_LOGE(TAG,
            "MPU is not ready.");
        return;
    }

    /*
     * -------------------------------------------------------------
     * 3. Odometry
     * -------------------------------------------------------------
     */
    ret =
        chassis_odometry_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
            "chassis_odometry_init failed: %s",
            esp_err_to_name(ret));
        return;
    }

    vTaskDelay(
        pdMS_TO_TICKS(100U));

    if (!chassis_odometry_is_ready())
    {
        ESP_LOGE(TAG,
            "Odometry did not become ready.");
        return;
    }

    /*
     * Current chassis position/heading becomes world origin.
     *
     * IMPORTANT:
     * Do not move the chassis during this short reset/settle interval.
     */
    ret =
        chassis_odometry_reset(
            0.0f,
            0.0f,
            0.0f);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
            "chassis_odometry_reset failed: %s",
            esp_err_to_name(ret));
        return;
    }

    vTaskDelay(
        pdMS_TO_TICKS(
            ODOM_PUSH_POST_RESET_SETTLE_MS));

    print_test_instructions();

    /*
     * Snapshot encoder origins after reset only for display.
     */
    motor_status_t origin_motor = {0};
    motor_get_status(&origin_motor);

    const int32_t origin_a =
        origin_motor.A.encoder_count;
    const int32_t origin_b =
        origin_motor.B.encoder_count;
    const int32_t origin_d =
        origin_motor.D.encoder_count;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG,
        "PUSH TEST READY -- you may move the chassis now.");
    ESP_LOGI(TAG,
        "Reset origin: encoder A=%" PRId32
        " B=%" PRId32
        " D=%" PRId32,
        origin_a,
        origin_b,
        origin_d);

    ESP_LOGI(TAG, "");

    uint32_t previous_update_count = 0U;
    int64_t previous_print_us =
        esp_timer_get_time();

    for (;;)
    {
        chassis_odometry_state_t odom = {0};
        mpu6050_snapshot_t imu = {0};

        motor_get_status(&motor_status);
        chassis_odometry_get_state(&odom);
        mpu6050_get_snapshot(&imu);

#if ODOM_PUSH_VERIFY_MOTOR_DISABLED
        /*
         * Safety guard: force outputs off if another piece of code
         * accidentally re-enables the motor driver.
         */
        if (motor_status.enabled)
        {
            ESP_LOGE(TAG,
                "SAFETY: motor became enabled! Disabling again.");
            motor_emergency_stop();
            motor_control_enable(false);
        }
#endif

        const int64_t now_us =
            esp_timer_get_time();

        const uint32_t update_delta =
            odom.update_count -
            previous_update_count;

        const double dt_print_s =
            (double)(
                now_us -
                previous_print_us) /
            1000000.0;

        const double odom_rate_hz =
            (previous_update_count != 0U &&
             dt_print_s > 0.0)
            ?
            ((double)update_delta /
             dt_print_s)
            :
            0.0;

        previous_update_count =
            odom.update_count;

        previous_print_us =
            now_us;

        const int32_t count_a =
            motor_status.A.encoder_count -
            origin_a;

        const int32_t count_b =
            motor_status.B.encoder_count -
            origin_b;

        const int32_t count_d =
            motor_status.D.encoder_count -
            origin_d;

        /*
         * Human-readable live line.
         */
        ESP_LOGI(TAG,
            "POSE | "
            "X=%+8.1f mm  Y=%+8.1f mm  Yaw=%+8.2f deg  Travel=%7.1f mm | "
            "BODY v=[%+7.1f,%+7.1f] mm/s | "
            "W gyro=%+6.3f enc=%+6.3f rad/s",
            (double)odom.x_mm,
            (double)odom.y_mm,
            (double)odom.yaw_deg,
            (double)odom.travel_distance_mm,
            (double)odom.body_vx_mm_s,
            (double)odom.body_vy_mm_s,
            (double)odom.gyro_w_rad_s,
            (double)odom.encoder_w_rad_s);

        ESP_LOGI(TAG,
            "WHEEL | "
            "count d[A,B,D]=[%" PRId32 ",%" PRId32 ",%" PRId32 "] | "
            "step_mm=[%+6.2f,%+6.2f,%+6.2f] | "
            "speed=[%+7.1f,%+7.1f,%+7.1f] mm/s | "
            "IMU gz=%+7.2f dps | odom=%.1fHz",
            count_a,
            count_b,
            count_d,
            (double)odom.delta_a_mm,
            (double)odom.delta_b_mm,
            (double)odom.delta_d_mm,
            (double)motor_status.A.actual_speed_mm_s,
            (double)motor_status.B.actual_speed_mm_s,
            (double)motor_status.D.actual_speed_mm_s,
            (double)imu.data.gz,
            odom_rate_hz);

        /*
         * Machine-readable line for copying back into ChatGPT.
         */
        ESP_LOGI(TAG,
            "CSV,ODOM_PUSH,"
            "x_mm,%.6f,"
            "y_mm,%.6f,"
            "yaw_deg,%.6f,"
            "travel_mm,%.6f,"
            "body_vx,%.6f,"
            "body_vy,%.6f,"
            "world_vx,%.6f,"
            "world_vy,%.6f,"
            "gyro_w,%.7f,"
            "encoder_w,%.7f,"
            "delta_a_mm,%.6f,"
            "delta_b_mm,%.6f,"
            "delta_d_mm,%.6f,"
            "count_a,%" PRId32 ","
            "count_b,%" PRId32 ","
            "count_d,%" PRId32 ","
            "imu_yaw,%.6f,"
            "imu_gz,%.6f,"
            "updates,%" PRIu32,
            (double)odom.x_mm,
            (double)odom.y_mm,
            (double)odom.yaw_deg,
            (double)odom.travel_distance_mm,
            (double)odom.body_vx_mm_s,
            (double)odom.body_vy_mm_s,
            (double)odom.world_vx_mm_s,
            (double)odom.world_vy_mm_s,
            (double)odom.gyro_w_rad_s,
            (double)odom.encoder_w_rad_s,
            (double)odom.delta_a_mm,
            (double)odom.delta_b_mm,
            (double)odom.delta_d_mm,
            count_a,
            count_b,
            count_d,
            (double)imu.attitude.yaw,
            (double)imu.data.gz,
            odom.update_count);

        vTaskDelay(
            pdMS_TO_TICKS(
                ODOM_PUSH_PRINT_PERIOD_MS));
    }
}
