#ifndef LINE_VISION_H
#define LINE_VISION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "image_decode.h"
#include "vision_shared_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. Camera / decode configuration
 *
 * This keeps the video path that was already validated:
 *   UVC MJPEG 480x320 @ 5 FPS
 *   TJpgDec scale=2 (1/4) -> grayscale 120x80
 * ============================================================ */
#ifndef LINE_VISION_CAMERA_WIDTH
#define LINE_VISION_CAMERA_WIDTH          VISION_SHARED_CAMERA_WIDTH
#endif

#ifndef LINE_VISION_CAMERA_HEIGHT
#define LINE_VISION_CAMERA_HEIGHT         VISION_SHARED_CAMERA_HEIGHT
#endif

#ifndef LINE_VISION_CAMERA_FPS
#define LINE_VISION_CAMERA_FPS            VISION_SHARED_CAMERA_FPS
#endif

#ifndef LINE_VISION_JPEG_SCALE
#define LINE_VISION_JPEG_SCALE            VISION_SHARED_JPEG_SCALE
#endif

#ifndef LINE_VISION_IMAGE_WIDTH
#define LINE_VISION_IMAGE_WIDTH           VISION_SHARED_IMAGE_WIDTH
#endif

#ifndef LINE_VISION_IMAGE_HEIGHT
#define LINE_VISION_IMAGE_HEIGHT          VISION_SHARED_IMAGE_HEIGHT
#endif

#ifndef LINE_VISION_POLL_MS
#define LINE_VISION_POLL_MS               20U
#endif

#ifndef LINE_VISION_STALE_MS
#define LINE_VISION_STALE_MS              500U
#endif

#ifndef LINE_VISION_TASK_STACK_SIZE
#define LINE_VISION_TASK_STACK_SIZE       4096U
#endif

#ifndef LINE_VISION_TASK_PRIORITY
#define LINE_VISION_TASK_PRIORITY         5U
#endif

#ifndef LINE_VISION_TASK_CORE
#define LINE_VISION_TASK_CORE             1
#endif

/* ============================================================
 * 2. ROI / segmentation configuration
 *
 * A pixel is treated as line pixel when gray < threshold.
 * The ROI is deliberately small, like a dense continuous
 * infrared sensor array.
 * ============================================================ */
#ifndef LINE_VISION_THRESHOLD
#define LINE_VISION_THRESHOLD             105U
#endif

#ifndef LINE_VISION_SCAN_START_ROW
#define LINE_VISION_SCAN_START_ROW        2U
#endif

#ifndef LINE_VISION_SCAN_END_ROW
#define LINE_VISION_SCAN_END_ROW          8U
#endif

#ifndef LINE_VISION_SCAN_START_COL
#define LINE_VISION_SCAN_START_COL        49U
#endif

#ifndef LINE_VISION_SCAN_END_COL
#define LINE_VISION_SCAN_END_COL          80U
#endif

#ifndef LINE_VISION_MIN_PIXELS
#define LINE_VISION_MIN_PIXELS            8U
#endif

/* ============================================================
 * 3. Observation returned to line_tracker
 *
 * line_vision answers only: "what does the camera see?"
 * It does NOT convert the observation into steering error.
 * ============================================================ */
typedef struct
{
    bool ready;                 /* At least one decoded frame has been published. */
    bool valid;                 /* Latest observation is not stale. */
    bool line_detected;         /* Enough dark pixels exist in ROI. */

    float line_center_x;        /* Dark-pixel centroid, in decoded-image pixels. */
    float roi_center_x;         /* Geometric center of active ROI. */
    float roi_half_width;       /* Half ROI span used by tracker normalization. */
    float center_error;
    float signed_moment2;
    float spread;
    float black_ratio;          /* Dark pixels / all ROI pixels. */

    uint32_t black_pixels;
    uint32_t roi_pixels;

    uint32_t sequence;          /* UVC frame sequence. */
    int64_t frame_timestamp_us; /* Timestamp from UVC cache. */
    int64_t update_timestamp_us;/* Time this decoded observation was published. */

} line_vision_data_t;

/* ============================================================
 * 4. Public API
 * ============================================================ */
esp_err_t line_vision_init(void);
bool line_vision_is_initialized(void);
bool line_vision_is_ready(void);
void line_vision_get_data(line_vision_data_t *out_data);

/* Debug/image-view helpers. Caller owns out_img->data. */
bool line_vision_get_latest_grayscale(grayscale_image_t *out_img);

/**
 * @brief Non-blocking debug snapshot of grayscale pixels + matching metadata.
 *
 * The image and line_vision_data_t are copied while holding the same mutex, so
 * sequence/timestamps always describe the returned pixels. This function never
 * waits for the vision mutex: if vision is publishing a frame, it returns false
 * immediately so debug traffic cannot delay the vision task.
 */
bool line_vision_try_get_grayscale_snapshot(
    grayscale_image_t *out_img,
    line_vision_data_t *out_data);

bool line_vision_generate_binary(grayscale_image_t *out_img, uint8_t threshold);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 *
 * Define LINE_VISION_IMPLEMENTATION in exactly one .c file.
 * ============================================================ */
#ifdef LINE_VISION_IMPLEMENTATION

/* line_vision owns the single shared decoder implementation in this project. */
#ifndef VISION_SHARED_DECODE_IMPLEMENTATION
#define VISION_SHARED_DECODE_IMPLEMENTATION
#define LINE_VISION_DEFINED_SHARED_DECODER_IMPLEMENTATION 1
#endif
#include "vision_shared_decode.h"
#ifdef LINE_VISION_DEFINED_SHARED_DECODER_IMPLEMENTATION
#undef VISION_SHARED_DECODE_IMPLEMENTATION
#undef LINE_VISION_DEFINED_SHARED_DECODER_IMPLEMENTATION
#endif

#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


static const char *LINE_VISION_TAG = "LINE_VISION";

static bool s_line_vision_initialized = false;
static TaskHandle_t s_line_vision_task_handle = NULL;
static SemaphoreHandle_t s_line_vision_mutex = NULL;

/* Public debug copy only; the decoder owns the immutable working GRAY8 plane. */
static uint8_t *s_line_vision_public_pixels = NULL;

static line_vision_data_t s_line_vision_latest = {0};

static void line_vision_free_local_resources(void)
{
    free(s_line_vision_public_pixels);
    s_line_vision_public_pixels = NULL;

    if (s_line_vision_mutex != NULL)
    {
        vSemaphoreDelete(s_line_vision_mutex);
        s_line_vision_mutex = NULL;
    }

    memset(&s_line_vision_latest, 0, sizeof(s_line_vision_latest));
}

static void *line_vision_alloc_pixels(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    if (p == NULL)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
#endif

    if (p == NULL)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }

    return p;
}

static bool line_vision_roi_geometry(
    uint16_t image_width,
    uint16_t image_height,
    uint16_t *out_start_row,
    uint16_t *out_end_row,
    uint16_t *out_start_col,
    uint16_t *out_end_col)
{
    if ((image_width == 0U) || (image_height == 0U))
    {
        return false;
    }

    uint16_t start_row = (uint16_t)LINE_VISION_SCAN_START_ROW;
    uint16_t end_row = (uint16_t)LINE_VISION_SCAN_END_ROW;
    uint16_t start_col = (uint16_t)LINE_VISION_SCAN_START_COL;
    uint16_t end_col = (uint16_t)LINE_VISION_SCAN_END_COL;

    if (start_row >= image_height)
    {
        return false;
    }

    if (start_col >= image_width)
    {
        return false;
    }

    if (end_row >= image_height)
    {
        end_row = (uint16_t)(image_height - 1U);
    }

    if (end_col >= image_width)
    {
        end_col = (uint16_t)(image_width - 1U);
    }

    if ((start_row > end_row) || (start_col > end_col))
    {
        return false;
    }

    if (out_start_row != NULL) *out_start_row = start_row;
    if (out_end_row != NULL) *out_end_row = end_row;
    if (out_start_col != NULL) *out_start_col = start_col;
    if (out_end_col != NULL) *out_end_col = end_col;

    return true;
}

static void line_vision_compute_observation(
    const grayscale_image_t *img,
    uint32_t sequence,
    int64_t frame_timestamp_us,
    line_vision_data_t *out)
{
    memset(out, 0, sizeof(*out));

    out->sequence = sequence;
    out->frame_timestamp_us = frame_timestamp_us;
    out->update_timestamp_us = esp_timer_get_time();

    if ((img == NULL) || (img->data == NULL))
    {
        return;
    }

    uint16_t start_row;
    uint16_t end_row;
    uint16_t start_col;
    uint16_t end_col;

    if (!line_vision_roi_geometry(
            img->width,
            img->height,
            &start_row,
            &end_row,
            &start_col,
            &end_col))
    {
        return;
    }

    const float roi_center_x =
        0.5f * ((float)start_col + (float)end_col);

    const float roi_half_width =
        0.5f * ((float)end_col - (float)start_col);

    out->roi_center_x = roi_center_x;
    out->roi_half_width = roi_half_width;

    uint64_t weighted_sum_x = 0U;

    float sum_d = 0.0f;
    float sum_signed_d2 = 0.0f;
    float sum_d2 = 0.0f;

    uint32_t black_count = 0U;
    uint32_t total_count = 0U;

    for (uint16_t y = start_row; y <= end_row; ++y)
    {
        const uint8_t *row =
            &img->data[(size_t)y * (size_t)img->width];

        for (uint16_t x = start_col; x <= end_col; ++x)
        {
            ++total_count;

            if (row[x] < (uint8_t)LINE_VISION_THRESHOLD)
            {
                ++black_count;

                weighted_sum_x += (uint64_t)x;

                const float d =
                    ((float)x - roi_center_x) /
                    roi_half_width;

                sum_d += d;

                const float d2 = d * d;

                sum_signed_d2 +=
                    (d >= 0.0f)
                    ? d2
                    : -d2;

                sum_d2 += d2;
            }
        }
    }

    out->ready = true;
    out->valid = true;
    out->black_pixels = black_count;
    out->roi_pixels = total_count;

    if (total_count > 0U)
    {
        out->black_ratio =
            (float)black_count / (float)total_count;
    }

    out->roi_center_x =
        0.5f * ((float)start_col + (float)end_col);

    /*
     * Use half the coordinate span, not half the pixel count.
     * Therefore a centroid exactly at start/end maps to -1/+1 later.
     */
    out->roi_half_width =
        0.5f * ((float)end_col - (float)start_col);

    if ((black_count >= (uint32_t)LINE_VISION_MIN_PIXELS) &&
        (roi_half_width > 0.0f))
    {
        const float inv_count =
            1.0f / (float)black_count;

        /*
        * Keep the original centroid for debug display
        * and backward compatibility.
        */
        out->line_center_x =
            (float)weighted_sum_x * inv_count;

        /*
        * M1 = mean(d)
        *
        * This is mathematically almost the same information
        * as the original normalized centroid.
        */
        out->center_error =
            sum_d * inv_count;

        /*
        * Signed second spatial moment:
        *
        * M2s = mean(sign(d) * d^2)
        */
        out->signed_moment2 =
            sum_signed_d2 * inv_count;

        /*
        * Normalized horizontal variance:
        *
        * Var(d) = E[d^2] - E[d]^2
        */
        const float mean_d2 =
            sum_d2 * inv_count;

        float spread =
            mean_d2 -
            out->center_error *
            out->center_error;

        if (spread < 0.0f)
        {
            spread = 0.0f;
        }

        out->spread = spread;

        out->line_detected = true;
    }

}

static bool line_vision_output_capacity_ok(
    const grayscale_image_t *out_img,
    size_t needed,
    size_t *out_capacity)
{
    if ((out_img == NULL) || (out_img->data == NULL))
    {
        return false;
    }

    size_t capacity = out_img->capacity_bytes;

    if ((capacity == 0U) &&
        (out_img->width != 0U) &&
        (out_img->height != 0U))
    {
        capacity =
            (size_t)out_img->width *
            (size_t)out_img->height;
    }

    if (capacity < needed)
    {
        return false;
    }

    if (out_capacity != NULL)
    {
        *out_capacity = capacity;
    }

    return true;
}

static void line_vision_task(void *arg)
{
    (void)arg;

    const size_t image_bytes =
        (size_t)LINE_VISION_IMAGE_WIDTH *
        (size_t)LINE_VISION_IMAGE_HEIGHT;

    uint32_t last_sequence = UINT32_MAX;

    ESP_LOGI(LINE_VISION_TAG, "Vision consumer started (shared single decoder)");

    while (1)
    {
        vision_shared_frame_t frame = {0};

        const esp_err_t frame_ret =
            vision_shared_decode_acquire_latest(
                last_sequence,
                &frame);

        if (frame_ret == ESP_OK)
        {
            const uint32_t frame_sequence = frame.sequence;
            const int64_t frame_timestamp_us = frame.frame_timestamp_us;

            grayscale_image_t view =
            {
                .data = (uint8_t *)frame.gray8,
                .capacity_bytes = image_bytes,
                .width = frame.width,
                .height = frame.height,
            };

            line_vision_data_t observation;
            line_vision_compute_observation(
                &view,
                frame_sequence,
                frame_timestamp_us,
                &observation);

            if (xSemaphoreTake(
                    s_line_vision_mutex,
                    pdMS_TO_TICKS(10)) == pdTRUE)
            {
                memcpy(
                    s_line_vision_public_pixels,
                    frame.gray8,
                    image_bytes);

                s_line_vision_latest = observation;
                xSemaphoreGive(s_line_vision_mutex);
            }

            last_sequence = frame_sequence;
            vision_shared_decode_release(&frame);
        }
        else if ((frame_ret != ESP_ERR_NOT_FOUND) &&
                 (frame_ret != ESP_ERR_INVALID_STATE) &&
                 (frame_ret != ESP_ERR_TIMEOUT))
        {
            ESP_LOGW(
                LINE_VISION_TAG,
                "Acquire shared frame failed: %s",
                esp_err_to_name(frame_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(LINE_VISION_POLL_MS));
    }
}

esp_err_t line_vision_init(void)
{
    if (s_line_vision_initialized)
    {
        return ESP_OK;
    }

    const size_t image_bytes =
        (size_t)LINE_VISION_IMAGE_WIDTH *
        (size_t)LINE_VISION_IMAGE_HEIGHT;

    if (image_bytes == 0U)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if ((LINE_VISION_CAMERA_WIDTH != VISION_SHARED_CAMERA_WIDTH) ||
        (LINE_VISION_CAMERA_HEIGHT != VISION_SHARED_CAMERA_HEIGHT) ||
        (LINE_VISION_JPEG_SCALE != VISION_SHARED_JPEG_SCALE) ||
        (LINE_VISION_IMAGE_WIDTH != VISION_SHARED_IMAGE_WIDTH) ||
        (LINE_VISION_IMAGE_HEIGHT != VISION_SHARED_IMAGE_HEIGHT))
    {
        ESP_LOGE(LINE_VISION_TAG, "line/shared decode geometry mismatch");
        return ESP_ERR_INVALID_SIZE;
    }

    if (!line_vision_roi_geometry(
            (uint16_t)LINE_VISION_IMAGE_WIDTH,
            (uint16_t)LINE_VISION_IMAGE_HEIGHT,
            NULL, NULL, NULL, NULL))
    {
        ESP_LOGE(LINE_VISION_TAG, "Invalid ROI configuration");
        return ESP_ERR_INVALID_ARG;
    }

    s_line_vision_mutex = xSemaphoreCreateMutex();
    if (s_line_vision_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_line_vision_public_pixels =
        (uint8_t *)line_vision_alloc_pixels(image_bytes);

    if (s_line_vision_public_pixels == NULL)
    {
        line_vision_free_local_resources();
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t ret = vision_shared_decode_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            LINE_VISION_TAG,
            "vision_shared_decode_init failed: %s",
            esp_err_to_name(ret));
        line_vision_free_local_resources();
        return ret;
    }

    const BaseType_t task_result =
        xTaskCreatePinnedToCore(
            line_vision_task,
            "line_vision",
            LINE_VISION_TASK_STACK_SIZE,
            NULL,
            LINE_VISION_TASK_PRIORITY,
            &s_line_vision_task_handle,
            LINE_VISION_TASK_CORE);

    if (task_result != pdPASS)
    {
        s_line_vision_task_handle = NULL;
        line_vision_free_local_resources();
        return ESP_ERR_NO_MEM;
    }

    s_line_vision_initialized = true;

    ESP_LOGI(
        LINE_VISION_TAG,
        "Initialized consumer: shared UVC=%ux%u@%u gray=%ux%u ROI=[r%u..%u,c%u..%u] threshold=%u",
        (unsigned)VISION_SHARED_CAMERA_WIDTH,
        (unsigned)VISION_SHARED_CAMERA_HEIGHT,
        (unsigned)VISION_SHARED_CAMERA_FPS,
        (unsigned)LINE_VISION_IMAGE_WIDTH,
        (unsigned)LINE_VISION_IMAGE_HEIGHT,
        (unsigned)LINE_VISION_SCAN_START_ROW,
        (unsigned)LINE_VISION_SCAN_END_ROW,
        (unsigned)LINE_VISION_SCAN_START_COL,
        (unsigned)LINE_VISION_SCAN_END_COL,
        (unsigned)LINE_VISION_THRESHOLD);

    return ESP_OK;
}

bool line_vision_is_initialized(void)
{
    return s_line_vision_initialized;
}

void line_vision_get_data(line_vision_data_t *out_data)
{
    if (out_data == NULL)
    {
        return;
    }

    memset(out_data, 0, sizeof(*out_data));

    if ((!s_line_vision_initialized) ||
        (s_line_vision_mutex == NULL))
    {
        return;
    }

    if (xSemaphoreTake(
            s_line_vision_mutex,
            pdMS_TO_TICKS(20)) != pdTRUE)
    {
        return;
    }

    *out_data = s_line_vision_latest;

    xSemaphoreGive(s_line_vision_mutex);

    if (!out_data->ready ||
        (out_data->update_timestamp_us <= 0))
    {
        out_data->valid = false;
        return;
    }

    const int64_t age_us =
        esp_timer_get_time() -
        out_data->update_timestamp_us;

    const int64_t stale_us =
        (int64_t)LINE_VISION_STALE_MS * 1000LL;

    out_data->valid =
        (age_us >= 0) &&
        (age_us <= stale_us);
}

bool line_vision_is_ready(void)
{
    line_vision_data_t data;
    line_vision_get_data(&data);
    return data.valid;
}

bool line_vision_get_latest_grayscale(grayscale_image_t *out_img)
{
    const size_t needed =
        (size_t)LINE_VISION_IMAGE_WIDTH *
        (size_t)LINE_VISION_IMAGE_HEIGHT;

    size_t capacity = 0U;

    if (!line_vision_output_capacity_ok(
            out_img,
            needed,
            &capacity))
    {
        return false;
    }

    if ((!s_line_vision_initialized) ||
        (s_line_vision_mutex == NULL) ||
        (s_line_vision_public_pixels == NULL))
    {
        return false;
    }

    if (xSemaphoreTake(
            s_line_vision_mutex,
            pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return false;
    }

    line_vision_data_t data = s_line_vision_latest;

    if (!data.ready)
    {
        xSemaphoreGive(s_line_vision_mutex);
        return false;
    }

    const int64_t age_us =
        esp_timer_get_time() -
        data.update_timestamp_us;

    if ((age_us < 0) ||
        (age_us > ((int64_t)LINE_VISION_STALE_MS * 1000LL)))
    {
        xSemaphoreGive(s_line_vision_mutex);
        return false;
    }

    memcpy(out_img->data, s_line_vision_public_pixels, needed);

    xSemaphoreGive(s_line_vision_mutex);

    out_img->width = (uint16_t)LINE_VISION_IMAGE_WIDTH;
    out_img->height = (uint16_t)LINE_VISION_IMAGE_HEIGHT;

    if (out_img->capacity_bytes == 0U)
    {
        out_img->capacity_bytes = capacity;
    }

    return true;
}

bool line_vision_try_get_grayscale_snapshot(
    grayscale_image_t *out_img,
    line_vision_data_t *out_data)
{
    const size_t needed =
        (size_t)LINE_VISION_IMAGE_WIDTH *
        (size_t)LINE_VISION_IMAGE_HEIGHT;

    size_t capacity = 0U;

    if ((out_data == NULL) ||
        !line_vision_output_capacity_ok(
            out_img,
            needed,
            &capacity))
    {
        return false;
    }

    memset(out_data, 0, sizeof(*out_data));

    if ((!s_line_vision_initialized) ||
        (s_line_vision_mutex == NULL) ||
        (s_line_vision_public_pixels == NULL))
    {
        return false;
    }

    /* Debug must never wait for vision. */
    if (xSemaphoreTake(s_line_vision_mutex, 0) != pdTRUE)
    {
        return false;
    }

    line_vision_data_t data = s_line_vision_latest;

    if (!data.ready)
    {
        xSemaphoreGive(s_line_vision_mutex);
        return false;
    }

    memcpy(out_img->data, s_line_vision_public_pixels, needed);

    xSemaphoreGive(s_line_vision_mutex);

    /* Keep stale semantics identical to line_vision_get_data(). */
    if (data.update_timestamp_us > 0)
    {
        const int64_t age_us =
            esp_timer_get_time() - data.update_timestamp_us;

        const int64_t stale_us =
            (int64_t)LINE_VISION_STALE_MS * 1000LL;

        data.valid =
            (age_us >= 0) &&
            (age_us <= stale_us);
    }
    else
    {
        data.valid = false;
    }

    *out_data = data;

    out_img->width = (uint16_t)LINE_VISION_IMAGE_WIDTH;
    out_img->height = (uint16_t)LINE_VISION_IMAGE_HEIGHT;

    if (out_img->capacity_bytes == 0U)
    {
        out_img->capacity_bytes = capacity;
    }

    return true;
}

bool line_vision_generate_binary(
    grayscale_image_t *out_img,
    uint8_t threshold)
{
    const size_t needed =
        (size_t)LINE_VISION_IMAGE_WIDTH *
        (size_t)LINE_VISION_IMAGE_HEIGHT;

    size_t capacity = 0U;

    if (!line_vision_output_capacity_ok(
            out_img,
            needed,
            &capacity))
    {
        return false;
    }

    if ((!s_line_vision_initialized) ||
        (s_line_vision_mutex == NULL) ||
        (s_line_vision_public_pixels == NULL))
    {
        return false;
    }

    if (xSemaphoreTake(
            s_line_vision_mutex,
            pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return false;
    }

    line_vision_data_t data = s_line_vision_latest;

    if (!data.ready)
    {
        xSemaphoreGive(s_line_vision_mutex);
        return false;
    }

    const int64_t age_us =
        esp_timer_get_time() -
        data.update_timestamp_us;

    if ((age_us < 0) ||
        (age_us > ((int64_t)LINE_VISION_STALE_MS * 1000LL)))
    {
        xSemaphoreGive(s_line_vision_mutex);
        return false;
    }

    for (size_t i = 0U; i < needed; ++i)
    {
        out_img->data[i] =
            (s_line_vision_public_pixels[i] < threshold)
            ? 255U
            : 0U;
    }

    xSemaphoreGive(s_line_vision_mutex);

    out_img->width = (uint16_t)LINE_VISION_IMAGE_WIDTH;
    out_img->height = (uint16_t)LINE_VISION_IMAGE_HEIGHT;

    if (out_img->capacity_bytes == 0U)
    {
        out_img->capacity_bytes = capacity;
    }

    return true;
}

#endif /* LINE_VISION_IMPLEMENTATION */
#endif /* LINE_VISION_H */
