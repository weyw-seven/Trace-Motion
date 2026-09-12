/*
 * Production application entry point.
 *
 * Hardware and trajectory implementation macros intentionally live in the
 * dedicated implementation translation units. Keeping this file small makes
 * the N3 service entry point unambiguous and prevents legacy motion tests from
 * becoming part of the flashed application by accident.
 */

#include "esp_err.h"
#include "esp_log.h"

#include "n3_service.h"


static const char *TAG = "n3_app";


void app_main(void)
{
    const esp_err_t ret = n3_service_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start N3 service: %s", esp_err_to_name(ret));
    }
}
