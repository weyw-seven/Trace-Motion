#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Standalone line-tracking test.
 *
 * Define each header implementation exactly once in the project.
 */
#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

#define INFRARED_SENSOR_IMPLEMENTATION
#include "infrared_sensor.h"

#define LINE_TRACKER_IMPLEMENTATION
#include "line_tracker.h"


static const char *TAG = "LINE_TEST";


void app_main(void)
{
    ESP_ERROR_CHECK(
        line_tracker_init());

    if (!motor_control_is_ready())
    {
        ESP_LOGE(
            TAG,
            "motor_control is not ready; check encoder/geometry configuration.");

        motor_emergency_stop();
        return;
    }

    ESP_LOGI(
        TAG,
        "Starting line tracker in 2 seconds...");

    vTaskDelay(
        pdMS_TO_TICKS(2000));

    line_tracker_start();

    while (1)
    {
        line_tracker_status_t s;

        line_tracker_get_status(
            &s);

        ESP_LOGI(
            TAG,
            "state=%d pattern=0x%X error=%+.3f vx=%.3f w=%+.3f",
            (int)s.state,
            (unsigned)s.pattern,
            s.error,
            s.target_vx,
            s.target_w);

        vTaskDelay(
            pdMS_TO_TICKS(500));
    }
}