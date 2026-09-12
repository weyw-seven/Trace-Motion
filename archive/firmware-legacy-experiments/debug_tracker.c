#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Single-header implementations.
 *
 * IMPORTANT:
 * Each *_IMPLEMENTATION macro must exist in exactly ONE .c file
 * in the whole project. If you already define one of them elsewhere,
 * remove that #define here.
 */

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"

#define CHASSIS_KINEMATICS_IMPLEMENTATION
#include "chassis_kinematics.h"

#define CHASSIS_ODOMETRY_IMPLEMENTATION
#include "chassis_odometry.h"

#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#include "trajectory_executor.h"

#define TRAJECTORY_TRACKER_IMPLEMENTATION
#include "trajectory_tracker.h"


static const char *TAG = "TRACKER_TEST";


/* ============================================================
 * Test configuration
 * ============================================================ */

#define TEST_TRAJECTORY_PATH              "/spiffs/test.traj"

#define TEST_CONTROL_PERIOD_MS            10U
#define TEST_DT_S                         0.010f

/* Print one combined tracker + motor status line every 100 ms. */
#define TEST_LOG_EVERY_N_UPDATES          10U

/*
 * Hard test timeout.
 * This is only a safety guard for the test program.
 */
#define TEST_TIMEOUT_MS                   30000U

/*
 * Delay after odometry reset so the background task can publish
 * a few coherent samples before tracker_start().
 */
#define TEST_AFTER_ODOM_RESET_DELAY_MS    100U


/* ============================================================
 * Tracker tuning used by this first real-car test
 * ============================================================ */

/*
 * These are deliberately conservative first-test values.
 *
 * If your trajectory file requests a speed above
 * TEST_TRACKER_MAX_TRANSLATION_SPEED_MM_S, the tracker will limit it.
 */
#define TEST_TRACKER_POSITION_KP                  1.00f
#define TEST_TRACKER_YAW_KP                       2.00f

#define TEST_TRACKER_MAX_FEEDBACK_SPEED_MM_S      220.0f
#define TEST_TRACKER_MAX_TRANSLATION_SPEED_MM_S   450.0f
#define TEST_TRACKER_MAX_ANGULAR_SPEED_RAD_S      1.50f

#define TEST_TRACKER_MAX_TRACKING_ERROR_MM        300.0f
#define TEST_TRACKER_ODOMETRY_TIMEOUT_MS          200U

#define TEST_TRACKER_START_POSITION_TOL_MM        30.0f
#define TEST_TRACKER_START_YAW_TOL_DEG            8.0f

#define TEST_TRACKER_POSITION_TOL_MM              15.0f
#define TEST_TRACKER_YAW_TOL_DEG                  3.0f
#define TEST_TRACKER_SETTLE_TIME_MS               500U


/* ============================================================
 * SPIFFS
 * ============================================================ */

static esp_err_t test_mount_spiffs(void)
{
    const esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "SPIFFS mount failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    size_t total = 0U;
    size_t used = 0U;

    ret = esp_spiffs_info(NULL, &total, &used);

    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS: total=%u used=%u",
            (unsigned)total,
            (unsigned)used);
    }

    return ESP_OK;
}


/* ============================================================
 * Hardware readiness
 * ============================================================ */

static esp_err_t test_wait_for_hardware(uint32_t timeout_ms)
{
    const int64_t start_us = esp_timer_get_time();

    while (true)
    {
        mpu6050_status_t imu_status = {0};

        mpu6050_get_status(&imu_status);

        const bool motor_ready =
            motor_control_is_ready();

        const bool imu_ready =
            imu_status.initialized &&
            imu_status.calibrated &&
            imu_status.sampling;

        const bool odom_ready =
            chassis_odometry_is_ready();

        if (motor_ready &&
            imu_ready &&
            odom_ready)
        {
            ESP_LOGI(
                TAG,
                "Hardware ready: motor=1 imu=1 odom=1");

            return ESP_OK;
        }

        const int64_t elapsed_ms =
            (esp_timer_get_time() - start_us) / 1000LL;

        if (elapsed_ms >= (int64_t)timeout_ms)
        {
            ESP_LOGE(
                TAG,
                "Hardware readiness timeout: "
                "motor=%d imu(init=%d cal=%d sampling=%d) odom=%d",
                (int)motor_ready,
                (int)imu_status.initialized,
                (int)imu_status.calibrated,
                (int)imu_status.sampling,
                (int)odom_ready);

            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* ============================================================
 * Debug logging
 * ============================================================ */

static void test_print_status(
    float elapsed_s,
    const trajectory_reference_t *ref)
{
    chassis_odometry_state_t odom = {0};
    motor_status_t motor = {0};

    chassis_odometry_get_state(&odom);
    motor_get_status(&motor);

    const float ex =
        ref->x_mm - odom.x_mm;

    const float ey =
        ref->y_mm - odom.y_mm;

    const float position_error =
        sqrtf(ex * ex + ey * ey);

    /*
     * One compact line is enough for real-car tuning.
     *
     * Wheel fields are:
     *     target_speed_mm_s / actual_speed_mm_s / output_pwm
     *
     * This lets us distinguish:
     *   1) target exists but wheel never starts  -> startup PWM issue
     *   2) wheel starts then stalls              -> low-speed FF issue
     *   3) wheel keeps moving but speed is off   -> closed-loop tuning issue
     */
    ESP_LOGI(
        TAG,
        "t=%6.2f "
        "seg=%lu "
        "ref=(%7.1f,%7.1f) "
        "odom=(%7.1f,%7.1f,%6.1fdeg) "
        "err=%6.1fmm "
        "vref=%6.1f "
        "cmd=(%6.1f,%6.1f,%5.2f) "
        "MOTOR "
        "A=(%6.1f/%6.1f/%6.1f) "
        "B=(%6.1f/%6.1f/%6.1f) "
        "D=(%6.1f/%6.1f/%6.1f)",
        elapsed_s,
        (unsigned long)ref->segment_index,
        ref->x_mm,
        ref->y_mm,
        odom.x_mm,
        odom.y_mm,
        odom.yaw_deg,
        position_error,
        ref->speed_mm_s,
        motor.target_vx_mm_s,
        motor.target_vy_mm_s,
        motor.target_w_rad_s,
        motor.A.target_speed_mm_s,
        motor.A.actual_speed_mm_s,
        motor.A.output_pwm,
        motor.B.target_speed_mm_s,
        motor.B.actual_speed_mm_s,
        motor.B.output_pwm,
        motor.D.target_speed_mm_s,
        motor.D.actual_speed_mm_s,
        motor.D.output_pwm);
}


/* ============================================================
 * Main tracker test
 * ============================================================ */

static esp_err_t test_run_tracker(const char *trajectory_path)
{
    esp_err_t ret = ESP_OK;

    trajectory_decoder_t decoder =
        TRAJECTORY_DECODER_INITIALIZER;

    trajectory_executor_t executor =
        TRAJECTORY_EXECUTOR_INITIALIZER;

    trajectory_tracker_t tracker = {0};

    trajectory_file_header_t header = {0};

    trajectory_reference_t reference = {0};

    /*
     * Conservative V1 tracker configuration.
     *
     * This matches the frozen Tracker V1 interface:
     * position P outer loop + fixed-yaw loop + WORLD->BODY
     * + safety limits + final settling.
     */
    const trajectory_tracker_config_t tracker_config =
    {
        .position_kp =
            TEST_TRACKER_POSITION_KP,

        .yaw_kp =
            TEST_TRACKER_YAW_KP,

        .max_feedback_speed_mm_s =
            TEST_TRACKER_MAX_FEEDBACK_SPEED_MM_S,

        .max_translation_speed_mm_s =
            TEST_TRACKER_MAX_TRANSLATION_SPEED_MM_S,

        .max_angular_speed_rad_s =
            TEST_TRACKER_MAX_ANGULAR_SPEED_RAD_S,

        .max_tracking_error_mm =
            TEST_TRACKER_MAX_TRACKING_ERROR_MM,

        .odometry_timeout_ms =
            TEST_TRACKER_ODOMETRY_TIMEOUT_MS,

        .start_position_tolerance_mm =
            TEST_TRACKER_START_POSITION_TOL_MM,

        .start_yaw_tolerance_deg =
            TEST_TRACKER_START_YAW_TOL_DEG,

        .position_tolerance_mm =
            TEST_TRACKER_POSITION_TOL_MM,

        .yaw_tolerance_deg =
            TEST_TRACKER_YAW_TOL_DEG,
        .settling_min_speed_mm_s = 15.0f,
        .settling_max_speed_mm_s = 40.0f,
        .settle_time_ms =
            TEST_TRACKER_SETTLE_TIME_MS,
    };


    /* --------------------------------------------------------
     * 1. Open trajectory file
     * -------------------------------------------------------- */

    ESP_LOGI(
        TAG,
        "Opening trajectory: %s",
        trajectory_path);

    ret =
        trajectory_decoder_open(
            &decoder,
            trajectory_path);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_decoder_open failed: %s",
            esp_err_to_name(ret));

        goto fail;
    }


    /* --------------------------------------------------------
     * 2. Initialize executor
     * -------------------------------------------------------- */

    ret =
        trajectory_executor_init(
            &executor,
            &decoder);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_executor_init failed: %s",
            esp_err_to_name(ret));

        goto fail;
    }


    ret =
        trajectory_executor_get_header(
            &executor,
            &header);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_executor_get_header failed: %s",
            esp_err_to_name(ret));

        goto fail;
    }


    ESP_LOGI(
        TAG,
        "Trajectory header: segments=%lu start=(%.1f, %.1f, %.1f deg)",
        (unsigned long)header.segment_count,
        header.start_x_mm,
        header.start_y_mm,
        header.start_yaw_deg);


    /* --------------------------------------------------------
     * 3. Align odometry WORLD frame to trajectory WORLD frame
     * -------------------------------------------------------- */

    motor_stop();

    ret =
        chassis_odometry_reset(
            header.start_x_mm,
            header.start_y_mm,
            header.start_yaw_deg);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "chassis_odometry_reset failed: %s",
            esp_err_to_name(ret));

        goto fail;
    }

    vTaskDelay(
        pdMS_TO_TICKS(
            TEST_AFTER_ODOM_RESET_DELAY_MS));


    {
        chassis_odometry_state_t odom = {0};

        chassis_odometry_get_state(&odom);

        ESP_LOGI(
            TAG,
            "Odometry aligned: (%.2f, %.2f, %.2f deg)",
            odom.x_mm,
            odom.y_mm,
            odom.yaw_deg);
    }


    /* --------------------------------------------------------
     * 4. Initialize and start tracker
     * -------------------------------------------------------- */

    ret =
        trajectory_tracker_init(
            &tracker,
            &tracker_config);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_tracker_init failed: %s",
            esp_err_to_name(ret));

        goto fail;
    }


    ret =
        trajectory_tracker_start(
            &tracker,
            &header);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_tracker_start failed: %s "
            "(tracker_error=%d)",
            esp_err_to_name(ret),
            (int)trajectory_tracker_get_error(&tracker));

        goto fail;
    }


    /* --------------------------------------------------------
     * 5. Start nominal trajectory
     * -------------------------------------------------------- */

    ret =
        trajectory_executor_start(
            &executor);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_executor_start failed: %s",
            esp_err_to_name(ret));

        goto fail;
    }


    ESP_LOGW(
        TAG,
        "REAL MOTION STARTING NOW");

    ESP_LOGI(
        TAG,
        "Control period = %u ms",
        (unsigned)TEST_CONTROL_PERIOD_MS);


    /* --------------------------------------------------------
     * 6. Real-time closed-loop test
     * -------------------------------------------------------- */

    TickType_t last_wake_time =
        xTaskGetTickCount();

    const int64_t test_start_us =
        esp_timer_get_time();

    uint32_t update_count = 0U;

    while (true)
    {
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(TEST_CONTROL_PERIOD_MS));


        /*
         * Executor owns nominal time/reference.
         */
        ret =
            trajectory_executor_update(
                &executor,
                TEST_DT_S,
                &reference);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "executor_update failed: %s "
                "(state=%d error=%d)",
                esp_err_to_name(ret),
                (int)trajectory_executor_get_state(&executor),
                (int)trajectory_executor_get_error(&executor));

            goto fail;
        }


        /*
         * Tracker reads odometry internally, closes the trajectory
         * error loop, converts WORLD velocity to BODY velocity,
         * and calls motor_set_velocity() itself.
         */
        ret =
            trajectory_tracker_update(
                &tracker,
                &reference,
                TEST_DT_S);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "tracker_update failed: %s "
                "(state=%d error=%d)",
                esp_err_to_name(ret),
                (int)trajectory_tracker_get_state(&tracker),
                (int)trajectory_tracker_get_error(&tracker));

            goto fail;
        }


        update_count++;


        if ((update_count %
             TEST_LOG_EVERY_N_UPDATES) == 0U ||
            reference.segment_start ||
            reference.segment_end ||
            reference.trajectory_end)
        {
            const float elapsed_s =
                (float)(
                    esp_timer_get_time() -
                    test_start_us) /
                1000000.0f;

            test_print_status(
                elapsed_s,
                &reference);
        }


        /*
         * IMPORTANT:
         *
         * Do NOT stop merely because executor is FINISHED.
         * At trajectory_end the tracker may still be in SETTLING,
         * using position/yaw feedback to pull the real chassis to
         * the final target.
         */
        if (trajectory_tracker_is_finished(
                &tracker))
        {
            break;
        }


        /*
         * Independent test-program timeout.
         */
        const int64_t elapsed_ms =
            (esp_timer_get_time() -
             test_start_us) /
            1000LL;

        if (elapsed_ms >=
            (int64_t)TEST_TIMEOUT_MS)
        {
            ESP_LOGE(
                TAG,
                "TEST TIMEOUT after %lld ms",
                (long long)elapsed_ms);

            ret = ESP_ERR_TIMEOUT;

            goto fail;
        }
    }


    /* --------------------------------------------------------
     * 7. Success report
     * -------------------------------------------------------- */

    motor_stop();

    {
        chassis_odometry_state_t odom = {0};

        chassis_odometry_get_state(&odom);

        const float final_ex =
            reference.x_mm - odom.x_mm;

        const float final_ey =
            reference.y_mm - odom.y_mm;

        const float final_error =
            sqrtf(
                final_ex * final_ex +
                final_ey * final_ey);

        ESP_LOGI(
            TAG,
            "========================================");

        ESP_LOGI(
            TAG,
            "TRACKER TEST PASSED");

        ESP_LOGI(
            TAG,
            "updates = %lu",
            (unsigned long)update_count);

        ESP_LOGI(
            TAG,
            "final ref  = (%.2f, %.2f)",
            reference.x_mm,
            reference.y_mm);

        ESP_LOGI(
            TAG,
            "final odom = (%.2f, %.2f, %.2f deg)",
            odom.x_mm,
            odom.y_mm,
            odom.yaw_deg);

        ESP_LOGI(
            TAG,
            "final position error = %.2f mm",
            final_error);

        ESP_LOGI(
            TAG,
            "tracker state=%d error=%d",
            (int)trajectory_tracker_get_state(&tracker),
            (int)trajectory_tracker_get_error(&tracker));

        ESP_LOGI(
            TAG,
            "========================================");
    }


    trajectory_executor_stop(
        &executor);

    trajectory_tracker_stop(
        &tracker);

    trajectory_decoder_close(
        &decoder);

    return ESP_OK;


fail:

    /*
     * Any unexpected failure in this real-car test stops the chassis.
     */
    motor_emergency_stop();

    trajectory_executor_stop(
        &executor);

    trajectory_tracker_stop(
        &tracker);

    trajectory_decoder_close(
        &decoder);

    ESP_LOGE(
        TAG,
        "========================================");

    ESP_LOGE(
        TAG,
        "TRACKER TEST FAILED: %s",
        esp_err_to_name(ret));

    ESP_LOGE(
        TAG,
        "========================================");

    return ret;
}


/* ============================================================
 * app_main
 * ============================================================ */

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(
        TAG,
        "========================================");

    ESP_LOGI(
        TAG,
        "Trajectory Tracker REAL-CAR Test");

    ESP_LOGI(
        TAG,
        "trajectory = %s",
        TEST_TRAJECTORY_PATH);

    ESP_LOGW(
        TAG,
        "This test WILL MOVE the chassis.");

    ESP_LOGI(
        TAG,
        "========================================");


    /* --------------------------------------------------------
     * 1. Mount trajectory storage
     * -------------------------------------------------------- */

    ret =
        test_mount_spiffs();

    if (ret != ESP_OK)
    {
        return;
    }


    /* --------------------------------------------------------
     * 2. Initialize low-level hardware/control
     * -------------------------------------------------------- */

    ret =
        motor_control_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "motor_control_init failed: %s",
            esp_err_to_name(ret));

        goto done;
    }


    ret =
        mpu6050_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "mpu6050_init failed: %s",
            esp_err_to_name(ret));

        motor_emergency_stop();

        goto done;
    }


    ret =
        chassis_odometry_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "chassis_odometry_init failed: %s",
            esp_err_to_name(ret));

        motor_emergency_stop();

        goto done;
    }


    ret =
        test_wait_for_hardware(
            5000U);

    if (ret != ESP_OK)
    {
        motor_emergency_stop();

        goto done;
    }


    /* --------------------------------------------------------
     * 3. Run the actual tracker test
     * -------------------------------------------------------- */

    ret =
        test_run_tracker(
            TEST_TRAJECTORY_PATH);


done:

    motor_stop();

    (void)chassis_odometry_deinit();

    (void)mpu6050_deinit();

    esp_vfs_spiffs_unregister(
        NULL);


    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "app_main finished successfully");
    }
    else
    {
        ESP_LOGE(
            TAG,
            "app_main finished with error: %s",
            esp_err_to_name(ret));
    }
}