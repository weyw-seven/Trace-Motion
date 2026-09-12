/*
 * mpu_realtime_monitor.c
 *
 * Simple real-time MPU6050 monitor.
 *
 * Purpose:
 *   Manually verify:
 *     - Roll / Pitch / Yaw polarity
 *     - Gyro X / Y / Z polarity
 *     - Approximate angle magnitude
 *     - Static gyro zero / noise
 *
 * Test procedure:
 *   1) Keep sensor/chassis LEVEL and completely still at boot.
 *   2) Wait for automatic calibration.
 *   3) Watch the continuously refreshed log.
 *   4) Manually tilt/rotate one axis at a time.
 *
 * Build note:
 *   MPU6050_IMPLEMENTATION must exist in exactly ONE translation unit.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"


/* =========================================================================
 * Configuration
 * ========================================================================= */

/*
 * Print period.
 *
 * 100 ms = 10 Hz terminal refresh.
 * MPU itself still samples at 200 Hz.
 */
#ifndef MPU_MONITOR_PRINT_PERIOD_MS
#define MPU_MONITOR_PRINT_PERIOD_MS        100U
#endif

/*
 * Reset yaw to zero once after initialization.
 */
#ifndef MPU_MONITOR_RESET_YAW_AT_START
#define MPU_MONITOR_RESET_YAW_AT_START       1
#endif

/*
 * Print raw ADC values too.
 * 0 = compact output
 * 1 = also print raw accel/gyro values
 */
#ifndef MPU_MONITOR_PRINT_RAW
#define MPU_MONITOR_PRINT_RAW                0
#endif


static const char *TAG = "MPU_MONITOR";


/* =========================================================================
 * Main
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# MPU6050 real-time angle / gyro monitor");
    ESP_LOGI(
        TAG,
        "############################################################");

    ESP_LOGI(
        TAG,
        "Sample=%u Hz | DLPF=%u | accel_alpha=%.2f | gyro_alpha=%.2f | comp_alpha=%.2f",
        (unsigned)MPU6050_SAMPLE_RATE_HZ,
        (unsigned)MPU6050_DLPF_CFG,
        (double)MPU6050_ACCEL_LPF_ALPHA,
        (double)MPU6050_GYRO_LPF_ALPHA,
        (double)MPU6050_COMPLEMENTARY_ALPHA
    );

    ESP_LOGW(TAG, "");
    ESP_LOGW(
        TAG,
        "Keep chassis LEVEL and COMPLETELY STILL during initialization/calibration."
    );

    esp_err_t ret =
        mpu6050_init();

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mpu6050_init failed: %s",
            esp_err_to_name(ret)
        );
        return;
    }

    /*
     * Let post-calibration filtering settle.
     */
    vTaskDelay(pdMS_TO_TICKS(1000U));

    mpu6050_status_t status = {0};
    mpu6050_get_status(&status);

    ESP_LOGI(
        TAG,
        "Status | initialized=%d calibrated=%d sampling=%d "
        "WHO_AM_I=0x%02X samples=%" PRIu32 " errors=%" PRIu32,
        (int)status.initialized,
        (int)status.calibrated,
        (int)status.sampling,
        status.who_am_i,
        status.sample_count,
        status.read_error_count
    );

    if (!status.initialized ||
        !status.calibrated ||
        !status.sampling)
    {
        ESP_LOGE(
            TAG,
            "MPU not ready."
        );

        (void)mpu6050_deinit();
        return;
    }

#if MPU_MONITOR_RESET_YAW_AT_START

    mpu6050_reset_yaw();
    vTaskDelay(pdMS_TO_TICKS(100U));

    ESP_LOGI(
        TAG,
        "Yaw reset: current physical heading = 0 deg."
    );

#endif

    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "Move ONE axis at a time and watch the sign/magnitude."
    );

    ESP_LOGI(
        TAG,
        "Columns:"
    );

    ESP_LOGI(
        TAG,
        "ATT[deg] = roll pitch yaw"
    );

    ESP_LOGI(
        TAG,
        "GYRO[dps] = gx gy gz"
    );

    ESP_LOGI(
        TAG,
        "ACC[g] = ax ay az"
    );

    ESP_LOGI(TAG, "");

    uint32_t previous_sample_count = 0U;
    int64_t previous_timestamp_us = 0;

    while (true)
    {
        mpu6050_snapshot_t snap = {0};

        mpu6050_get_snapshot(
            &snap
        );

        /*
         * Optional effective background sample-rate display.
         */
        double sample_rate_hz = 0.0;

        if ((previous_timestamp_us > 0) &&
            (snap.timestamp_us > previous_timestamp_us) &&
            (snap.sample_count > previous_sample_count))
        {
            const uint32_t dc =
                snap.sample_count -
                previous_sample_count;

            const int64_t dt_us =
                snap.timestamp_us -
                previous_timestamp_us;

            if (dt_us > 0)
            {
                sample_rate_hz =
                    ((double)dc * 1000000.0) /
                    (double)dt_us;
            }
        }

        previous_sample_count =
            snap.sample_count;

        previous_timestamp_us =
            snap.timestamp_us;

        ESP_LOGI(
            TAG,
            "ATT[deg] R=%+8.3f P=%+8.3f Y=%+9.3f | "
            "GYRO[dps] X=%+8.3f Y=%+8.3f Z=%+8.3f | "
            "ACC[g] X=%+7.4f Y=%+7.4f Z=%+7.4f | "
            "stationary=%d sample=%" PRIu32 " rate=%.1fHz",
            (double)snap.attitude.roll,
            (double)snap.attitude.pitch,
            (double)snap.attitude.yaw,

            (double)snap.data.gx,
            (double)snap.data.gy,
            (double)snap.data.gz,

            (double)snap.data.ax,
            (double)snap.data.ay,
            (double)snap.data.az,

            (int)snap.stationary,
            snap.sample_count,
            sample_rate_hz
        );

#if MPU_MONITOR_PRINT_RAW

        ESP_LOGI(
            TAG,
            "RAW | "
            "ACC[%6d %6d %6d] "
            "GYRO[%6d %6d %6d] "
            "TEMP=%6d",
            snap.raw.ax,
            snap.raw.ay,
            snap.raw.az,

            snap.raw.gx,
            snap.raw.gy,
            snap.raw.gz,

            snap.raw.temperature
        );

#endif

        vTaskDelay(
            pdMS_TO_TICKS(
                MPU_MONITOR_PRINT_PERIOD_MS
            )
        );
    }
}