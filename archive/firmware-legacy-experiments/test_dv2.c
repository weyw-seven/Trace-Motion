/*
 * F1 decoder-only app_main example.
 *
 * Assumes SPIFFS is already mounted and:
 *     /spiffs/trj2_golden.traj
 * exists.
 *
 * DO NOT call trajectory_runner / executor / tracker / motor control here.
 */

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

static const char *TAG = "trj2_f1_test";

static esp_err_t mount_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = false,
    };

    esp_err_t ret =
        esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "SPIFFS mount failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    size_t total = 0;
    size_t used = 0;

    ret =
        esp_spiffs_info(
            "storage",
            &total,
            &used);

    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS mounted: total=%u used=%u",
            (unsigned)total,
            (unsigned)used);
    }

    return ESP_OK;
}


void app_main(void)
{
    ESP_LOGI(TAG, "=== F1 TRJ2 DECODER TEST START ===");

    esp_err_t ret =
        mount_spiffs();

    if (ret != ESP_OK)
    {
        return;
    }

    const char *path =
        "/spiffs/trj2_coverage.traj";

    FILE *check =
        fopen(path, "rb");

    if (check == NULL)
    {
        ESP_LOGE(
            TAG,
            "FILE NOT FOUND: %s",
            path);

        return;
    }

    fseek(check, 0, SEEK_END);

    long file_size =
        ftell(check);

    fclose(check);

    ESP_LOGI(
        TAG,
        "Found %s, size=%ld bytes",
        path,
        file_size);

    trajectory_decoder_t decoder =
        TRAJECTORY_DECODER_INITIALIZER;

    ret =
        trajectory_decoder_open(
            &decoder,
            path);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "decoder_open failed: %s",
            esp_err_to_name(ret));

        return;
    }

    trajectory_file_header_t header = {0};

    ret =
        trajectory_decoder_get_header(
            &decoder,
            &header);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "get_header failed: %s",
            esp_err_to_name(ret));

        trajectory_decoder_close(&decoder);
        return;
    }

    printf("\n=== TRJ HEADER ===\n");
    printf(
        "version      = %u\n",
        (unsigned)header.version);

    printf(
        "header_size  = %u\n",
        (unsigned)header.header_size);

    printf(
        "record_count = %lu\n",
        (unsigned long)header.record_count);

    printf(
        "record_size  = %u\n",
        (unsigned)header.record_size);

    printf(
        "start        = (%.3f, %.3f, %.3f deg)\n\n",
        header.start_x_mm,
        header.start_y_mm,
        header.start_yaw_deg);

    while (trajectory_decoder_has_next(&decoder))
    {
        uint32_t index =
            trajectory_decoder_get_next_record_index(
                &decoder);

        trajectory_record_t record = {0};

        ret =
            trajectory_decoder_read_next_record(
                &decoder,
                &record);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "record %lu failed: %s",
                (unsigned long)index,
                esp_err_to_name(ret));

            trajectory_decoder_close(&decoder);
            return;
        }

        printf(
            "[%lu] %s",
            (unsigned long)index,
            trajectory_decoder_record_type_name(
                record.type));

        switch (record.type)
        {
            case TRAJECTORY_RECORD_LINE:
                printf(
                    " speed=%.3f accel=%.3f "
                    "end=(%.3f, %.3f)",
                    record.speed_mm_s,
                    record.acceleration_mm_s2,
                    record.payload.line.end_x_mm,
                    record.payload.line.end_y_mm);
                break;

            case TRAJECTORY_RECORD_CIRCLE:
                printf(
                    " speed=%.3f accel=%.3f "
                    "center=(%.3f, %.3f) "
                    "r=%.3f start=%.3f sweep=%.3f",
                    record.speed_mm_s,
                    record.acceleration_mm_s2,
                    record.payload.circle.center_x_mm,
                    record.payload.circle.center_y_mm,
                    record.payload.circle.radius_mm,
                    record.payload.circle.start_angle_deg,
                    record.payload.circle.sweep_deg);
                break;

            case TRAJECTORY_RECORD_CUBIC_BEZIER:
                printf(
                    " speed=%.3f accel=%.3f "
                    "c1=(%.3f, %.3f) "
                    "c2=(%.3f, %.3f) "
                    "end=(%.3f, %.3f)",
                    record.speed_mm_s,
                    record.acceleration_mm_s2,
                    record.payload.cubic_bezier.control1_x_mm,
                    record.payload.cubic_bezier.control1_y_mm,
                    record.payload.cubic_bezier.control2_x_mm,
                    record.payload.cubic_bezier.control2_y_mm,
                    record.payload.cubic_bezier.end_x_mm,
                    record.payload.cubic_bezier.end_y_mm);
                break;

            case TRAJECTORY_RECORD_WAIT:
                printf(
                    " duration=%.3f s",
                    record.payload.wait.duration_s);
                break;

            case TRAJECTORY_RECORD_PEN_UP:
            case TRAJECTORY_RECORD_PEN_DOWN:
            default:
                break;
        }

        printf("\n");
    }

    trajectory_decoder_close(&decoder);

    printf(
        "\nPASS: TRJ2 decoder-only test completed\n");

    ESP_LOGI(
        TAG,
        "=== F1 TRJ2 DECODER TEST END ===");
}