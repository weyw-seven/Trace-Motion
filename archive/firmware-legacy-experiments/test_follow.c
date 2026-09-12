#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"


/*
 * decoder 是 single-header。
 * 整个工程只能有一个 .c 定义这个宏。
 */
#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

static const char *TAG = "MAIN";


/* ============================================================
 * SPIFFS mount
 * ============================================================ */

static esp_err_t test_mount_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = false,
    };

    esp_err_t ret =
        esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "SPIFFS mount failed: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }


    size_t total = 0;
    size_t used = 0;

    ret =
        esp_spiffs_info(
            NULL,
            &total,
            &used
        );

    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS: total=%u, used=%u",
            (unsigned)total,
            (unsigned)used
        );
    }

    return ESP_OK;
}


/* ============================================================
 * Print one decoded segment
 * ============================================================ */

static void test_print_segment(
    uint32_t index,
    const trajectory_segment_t *segment)
{
    if (segment == NULL)
    {
        return;
    }

    switch (segment->type)
    {
        case TRAJECTORY_SEGMENT_LINE:
        {
            ESP_LOGI(
                TAG,
                "[%lu] LINE",
                (unsigned long)index
            );

            ESP_LOGI(
                TAG,
                "    end          = (%.2f, %.2f) mm",
                segment->geometry.line.end_x_mm,
                segment->geometry.line.end_y_mm
            );

            ESP_LOGI(
                TAG,
                "    speed        = %.2f mm/s",
                segment->speed_mm_s
            );

            ESP_LOGI(
                TAG,
                "    acceleration = %.2f mm/s^2",
                segment->acceleration_mm_s2
            );

            break;
        }


        case TRAJECTORY_SEGMENT_CIRCLE:
        {
            ESP_LOGI(
                TAG,
                "[%lu] CIRCLE",
                (unsigned long)index
            );

            ESP_LOGI(
                TAG,
                "    center       = (%.2f, %.2f) mm",
                segment->geometry.circle.center_x_mm,
                segment->geometry.circle.center_y_mm
            );

            ESP_LOGI(
                TAG,
                "    radius       = %.2f mm",
                segment->geometry.circle.radius_mm
            );

            ESP_LOGI(
                TAG,
                "    start_angle  = %.2f deg",
                segment->geometry.circle.start_angle_deg
            );

            ESP_LOGI(
                TAG,
                "    sweep        = %.2f deg",
                segment->geometry.circle.sweep_deg
            );

            ESP_LOGI(
                TAG,
                "    speed        = %.2f mm/s",
                segment->speed_mm_s
            );

            ESP_LOGI(
                TAG,
                "    acceleration = %.2f mm/s^2",
                segment->acceleration_mm_s2
            );

            break;
        }


        default:
        {
            ESP_LOGW(
                TAG,
                "[%lu] Unknown segment type: %d",
                (unsigned long)index,
                (int)segment->type
            );

            break;
        }
    }
}



/* ============================================================
 * app_main
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "Trajectory decoder test");
    ESP_LOGI(TAG, "==============================");


    /* --------------------------------------------------------
     * 1. Mount filesystem
     * -------------------------------------------------------- */

    ESP_ERROR_CHECK(
        test_mount_spiffs()
    );


    /* --------------------------------------------------------
     * 2. Create decoder
     * -------------------------------------------------------- */

    trajectory_decoder_t decoder =
        TRAJECTORY_DECODER_INITIALIZER;


    /* --------------------------------------------------------
     * 3. Open .traj file
     * -------------------------------------------------------- */

    const char *path =
        "/spiffs/test.traj";


    ESP_LOGI(
        TAG,
        "Opening: %s",
        path
    );


    esp_err_t ret =
        trajectory_decoder_open(
            &decoder,
            path
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_decoder_open failed: %s",
            esp_err_to_name(ret)
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "Trajectory opened successfully"
    );


    /* --------------------------------------------------------
     * 4. Read every segment
     * -------------------------------------------------------- */

    uint32_t index = 0;


    while (trajectory_decoder_has_next(&decoder))
    {
        trajectory_segment_t segment = {0};


        ret =
            trajectory_decoder_read_next(
                &decoder,
                &segment
            );


        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "Read segment %lu failed: %s",
                (unsigned long)index,
                esp_err_to_name(ret)
            );

            break;
        }


        test_print_segment(
            index,
            &segment
        );


        index++;
    }


    /* --------------------------------------------------------
     * 5. Close
     * -------------------------------------------------------- */

    trajectory_decoder_close(
        &decoder
    );


    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(
        TAG,
        "Decoder test finished, segments=%lu",
        (unsigned long)index
    );
    ESP_LOGI(TAG, "==============================");
}
