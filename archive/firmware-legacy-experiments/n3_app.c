/*
 * Production N3 application entry point.
 *
 * Keep hardware/manual experiments in their own source file.  The component
 * build always selects this entry point so a hand-written motor test cannot
 * accidentally become the flashed N3 firmware.
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
