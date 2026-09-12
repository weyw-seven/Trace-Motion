#ifndef VISION_SHARED_DECODE_H
#define VISION_SHARED_DECODE_H

/*
 * vision_shared_decode.h
 *
 * Internal single-JPEG-decode fan-out for the line + color-ball pipeline.
 * One UVC MJPEG frame is decoded once by Espressif esp_new_jpeg directly to
 * 120x80 RGB565. A lightweight post-pass preserves the existing horizontal
 * mirror convention while generating GRAY8, then publishes one immutable frame
 * shared by line_vision and colorball_vision.
 *
 * This header is intentionally NOT a replacement for either public vision API.
 * line_vision.h remains the control-facing interface; colorball_vision.h remains
 * the color detector/debug interface.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "image_decode.h" /* IMAGE_DECODE_MIRROR_X */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VISION_SHARED_CAMERA_WIDTH
#define VISION_SHARED_CAMERA_WIDTH              480U
#endif

#ifndef VISION_SHARED_CAMERA_HEIGHT
#define VISION_SHARED_CAMERA_HEIGHT             320U
#endif

/* Conservative bring-up baseline. Raise after checking decode headroom. */
#ifndef VISION_SHARED_CAMERA_FPS
#define VISION_SHARED_CAMERA_FPS                15U
#endif

/* Legacy compatibility knob. esp_new_jpeg uses the explicit output size below. */
#ifndef VISION_SHARED_JPEG_SCALE
#define VISION_SHARED_JPEG_SCALE                2U
#endif

#ifndef VISION_SHARED_IMAGE_WIDTH
#define VISION_SHARED_IMAGE_WIDTH               120U
#endif

#ifndef VISION_SHARED_IMAGE_HEIGHT
#define VISION_SHARED_IMAGE_HEIGHT              80U
#endif

#ifndef VISION_SHARED_POLL_MS
#define VISION_SHARED_POLL_MS                   10U
#endif

#ifndef VISION_SHARED_TASK_STACK_SIZE
#define VISION_SHARED_TASK_STACK_SIZE           5120U
#endif

#ifndef VISION_SHARED_TASK_PRIORITY
#define VISION_SHARED_TASK_PRIORITY             5U
#endif

#ifndef VISION_SHARED_TASK_CORE
#define VISION_SHARED_TASK_CORE                 0
#endif

#ifndef VISION_SHARED_CACHE_SLOTS
#define VISION_SHARED_CACHE_SLOTS               2U
#endif

#if (VISION_SHARED_CACHE_SLOTS < 2)
#error "VISION_SHARED_CACHE_SLOTS must be >= 2"
#endif

#define VISION_SHARED_RGB565_BYTES \
    ((size_t)VISION_SHARED_IMAGE_WIDTH * \
     (size_t)VISION_SHARED_IMAGE_HEIGHT * 2U)

#define VISION_SHARED_GRAY8_BYTES \
    ((size_t)VISION_SHARED_IMAGE_WIDTH * \
     (size_t)VISION_SHARED_IMAGE_HEIGHT)

typedef struct
{
    const uint8_t *rgb565;
    const uint8_t *gray8;

    uint16_t width;
    uint16_t height;
    uint16_t rgb565_stride_bytes;

    uint32_t sequence;
    int64_t frame_timestamp_us;
    int64_t decode_timestamp_us;
    uint32_t decode_time_us;

    /* Internal acquire/release token. Do not modify. */
    uint8_t _slot;
    bool _acquired;

} vision_shared_frame_t;

typedef struct
{
    uint32_t decoded_frames;
    uint32_t decode_errors;
    uint32_t dropped_no_publish_slot;
    uint32_t acquire_lock_busy;

    uint32_t last_decode_time_us;
    uint32_t max_decode_time_us;
    uint64_t total_decode_time_us;

} vision_shared_decode_stats_t;

esp_err_t vision_shared_decode_init(void);
bool vision_shared_decode_is_initialized(void);

/* Non-blocking. UINT32_MAX means "latest even if already seen". */
esp_err_t vision_shared_decode_acquire_latest(
    uint32_t after_sequence,
    vision_shared_frame_t *out_frame);

void vision_shared_decode_release(vision_shared_frame_t *frame);
void vision_shared_decode_get_stats(vision_shared_decode_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* VISION_SHARED_DECODE_H */

/* ========================================================================== */
/* IMPLEMENTATION                                                             */
/* ========================================================================== */

#if defined(VISION_SHARED_DECODE_IMPLEMENTATION) && \
    !defined(VISION_SHARED_DECODE_IMPLEMENTED)
#define VISION_SHARED_DECODE_IMPLEMENTED 1

#include <string.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "uvc_module.h"

#include <limits.h>
#include "esp_jpeg_dec.h"

static const char *VISION_SHARED_TAG = "VISION_SHARED";

typedef struct
{
    uint8_t *storage;
    uint8_t *rgb565;
    uint8_t *gray8;

    uint32_t sequence;
    int64_t frame_timestamp_us;
    int64_t decode_timestamp_us;
    uint32_t decode_time_us;

    uint32_t ref_count;
    bool valid;
    bool writing;

} vision_shared_slot_t;

static bool s_vision_shared_initialized = false;
static bool s_vision_shared_owns_uvc = false;
static TaskHandle_t s_vision_shared_task_handle = NULL;
static SemaphoreHandle_t s_vision_shared_mutex = NULL;

static jpeg_dec_handle_t s_vision_shared_jpeg_decoder = NULL;
static jpeg_dec_io_t s_vision_shared_jpeg_io = {0};
static jpeg_dec_header_info_t s_vision_shared_jpeg_info = {0};

/* esp_new_jpeg requires a 16-byte-aligned output buffer on ESP32-S3. */
static uint8_t *s_vision_shared_work_rgb565 = NULL;
static uint8_t *s_vision_shared_work_gray8 = NULL;

static vision_shared_slot_t s_vision_shared_slots[VISION_SHARED_CACHE_SLOTS];
static int s_vision_shared_latest_slot = -1;
static vision_shared_decode_stats_t s_vision_shared_stats = {0};

static void *vision_shared_alloc(size_t size, bool prefer_internal)
{
    void *p = NULL;

    if (prefer_internal)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

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

static void vision_shared_free_resources(void)
{
    if (s_vision_shared_jpeg_decoder != NULL)
    {
        (void)jpeg_dec_close(s_vision_shared_jpeg_decoder);
        s_vision_shared_jpeg_decoder = NULL;
    }

    if (s_vision_shared_work_rgb565 != NULL)
    {
        jpeg_free_align(s_vision_shared_work_rgb565);
        s_vision_shared_work_rgb565 = NULL;
    }

    free(s_vision_shared_work_gray8);
    s_vision_shared_work_gray8 = NULL;

    memset(&s_vision_shared_jpeg_io, 0, sizeof(s_vision_shared_jpeg_io));
    memset(&s_vision_shared_jpeg_info, 0, sizeof(s_vision_shared_jpeg_info));

    for (size_t i = 0U; i < VISION_SHARED_CACHE_SLOTS; ++i)
    {
        free(s_vision_shared_slots[i].storage);
        memset(&s_vision_shared_slots[i], 0, sizeof(s_vision_shared_slots[i]));
    }

    s_vision_shared_latest_slot = -1;

    if (s_vision_shared_mutex != NULL)
    {
        vSemaphoreDelete(s_vision_shared_mutex);
        s_vision_shared_mutex = NULL;
    }

    memset(&s_vision_shared_stats, 0, sizeof(s_vision_shared_stats));
}

static esp_err_t vision_shared_jpeg_error_to_esp(jpeg_error_t err)
{
    switch (err)
    {
        case JPEG_ERR_OK:
            return ESP_OK;
        case JPEG_ERR_NO_MEM:
            return ESP_ERR_NO_MEM;
        case JPEG_ERR_INVALID_PARAM:
            return ESP_ERR_INVALID_ARG;
        case JPEG_ERR_UNSUPPORT_FMT:
        case JPEG_ERR_UNSUPPORT_STD:
            return ESP_ERR_NOT_SUPPORTED;
        case JPEG_ERR_NO_MORE_DATA:
        case JPEG_ERR_BAD_DATA:
        case JPEG_ERR_FAIL:
        default:
            return ESP_FAIL;
    }
}

static inline uint16_t vision_shared_load_rgb565_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void vision_shared_store_rgb565_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static inline uint8_t vision_shared_rgb565_to_gray(uint16_t p)
{
    const uint8_t r5 = (uint8_t)((p >> 11) & 0x1FU);
    const uint8_t g6 = (uint8_t)((p >> 5) & 0x3FU);
    const uint8_t b5 = (uint8_t)(p & 0x1FU);

    /* Expand the quantized RGB565 channels back to 8-bit before luma. */
    const uint8_t r = (uint8_t)((r5 << 3) | (r5 >> 2));
    const uint8_t g = (uint8_t)((g6 << 2) | (g6 >> 4));
    const uint8_t b = (uint8_t)((b5 << 3) | (b5 >> 2));

    return (uint8_t)(
        ((uint16_t)77U * r +
         (uint16_t)150U * g +
         (uint16_t)29U * b) >> 8);
}

static void vision_shared_finish_rgb565_and_gray(void)
{
    const uint32_t w = VISION_SHARED_IMAGE_WIDTH;
    const uint32_t h = VISION_SHARED_IMAGE_HEIGHT;

    for (uint32_t y = 0U; y < h; ++y)
    {
        uint8_t *row =
            s_vision_shared_work_rgb565 + (size_t)y * (size_t)w * 2U;
        uint8_t *gray =
            s_vision_shared_work_gray8 + (size_t)y * (size_t)w;

#if IMAGE_DECODE_MIRROR_X
        uint32_t left = 0U;
        uint32_t right = w - 1U;

        while (left < right)
        {
            uint8_t *lp = row + (size_t)left * 2U;
            uint8_t *rp = row + (size_t)right * 2U;

            const uint16_t l = vision_shared_load_rgb565_le(lp);
            const uint16_t r = vision_shared_load_rgb565_le(rp);

            vision_shared_store_rgb565_le(lp, r);
            vision_shared_store_rgb565_le(rp, l);

            gray[left] = vision_shared_rgb565_to_gray(r);
            gray[right] = vision_shared_rgb565_to_gray(l);

            ++left;
            --right;
        }

        if (left == right)
        {
            gray[left] =
                vision_shared_rgb565_to_gray(
                    vision_shared_load_rgb565_le(
                        row + (size_t)left * 2U));
        }
#else
        for (uint32_t x = 0U; x < w; ++x)
        {
            gray[x] =
                vision_shared_rgb565_to_gray(
                    vision_shared_load_rgb565_le(
                        row + (size_t)x * 2U));
        }
#endif
    }
}

static esp_err_t vision_shared_decode_one(
    const uint8_t *jpeg_data,
    size_t jpeg_len)
{
    if ((jpeg_data == NULL) ||
        (jpeg_len < 4U) ||
        (jpeg_len > (size_t)INT_MAX) ||
        (s_vision_shared_jpeg_decoder == NULL) ||
        (s_vision_shared_work_rgb565 == NULL) ||
        (s_vision_shared_work_gray8 == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_vision_shared_jpeg_io, 0, sizeof(s_vision_shared_jpeg_io));
    memset(&s_vision_shared_jpeg_info, 0, sizeof(s_vision_shared_jpeg_info));

    s_vision_shared_jpeg_io.inbuf = (uint8_t *)jpeg_data;
    s_vision_shared_jpeg_io.inbuf_len = (int)jpeg_len;
    s_vision_shared_jpeg_io.outbuf = s_vision_shared_work_rgb565;

    jpeg_error_t jret =
        jpeg_dec_parse_header(
            s_vision_shared_jpeg_decoder,
            &s_vision_shared_jpeg_io,
            &s_vision_shared_jpeg_info);

    if (jret != JPEG_ERR_OK)
    {
        ESP_LOGW(
            VISION_SHARED_TAG,
            "esp_new_jpeg parse failed: %d",
            (int)jret);
        return vision_shared_jpeg_error_to_esp(jret);
    }

    /*
     * Accept source or scaled geometry here, then validate the decoder's actual
     * output-buffer requirement below. This avoids coupling the pipeline to a
     * header-reporting detail while still rejecting an unexpected camera mode.
     */
    const bool geometry_expected =
        ((s_vision_shared_jpeg_info.width == VISION_SHARED_IMAGE_WIDTH) &&
         (s_vision_shared_jpeg_info.height == VISION_SHARED_IMAGE_HEIGHT)) ||
        ((s_vision_shared_jpeg_info.width == VISION_SHARED_CAMERA_WIDTH) &&
         (s_vision_shared_jpeg_info.height == VISION_SHARED_CAMERA_HEIGHT));

    if (!geometry_expected)
    {
        ESP_LOGE(
            VISION_SHARED_TAG,
            "Unexpected JPEG geometry from decoder: %ux%u expected source=%ux%u or scaled=%ux%u",
            (unsigned)s_vision_shared_jpeg_info.width,
            (unsigned)s_vision_shared_jpeg_info.height,
            (unsigned)VISION_SHARED_CAMERA_WIDTH,
            (unsigned)VISION_SHARED_CAMERA_HEIGHT,
            (unsigned)VISION_SHARED_IMAGE_WIDTH,
            (unsigned)VISION_SHARED_IMAGE_HEIGHT);
        return ESP_ERR_INVALID_SIZE;
    }

    int required_out_bytes = 0;
    jret = jpeg_dec_get_outbuf_len(
        s_vision_shared_jpeg_decoder,
        &required_out_bytes);

    if ((jret != JPEG_ERR_OK) ||
        (required_out_bytes != (int)VISION_SHARED_RGB565_BYTES))
    {
        ESP_LOGE(
            VISION_SHARED_TAG,
            "esp_new_jpeg output size mismatch: err=%d got=%d expected=%u",
            (int)jret,
            required_out_bytes,
            (unsigned)VISION_SHARED_RGB565_BYTES);
        return (jret == JPEG_ERR_OK)
            ? ESP_ERR_INVALID_SIZE
            : vision_shared_jpeg_error_to_esp(jret);
    }

    jret = jpeg_dec_process(
        s_vision_shared_jpeg_decoder,
        &s_vision_shared_jpeg_io);

    if (jret != JPEG_ERR_OK)
    {
        ESP_LOGW(
            VISION_SHARED_TAG,
            "esp_new_jpeg decode failed: %d",
            (int)jret);
        return vision_shared_jpeg_error_to_esp(jret);
    }

    if ((s_vision_shared_jpeg_io.out_size > 0) &&
        (s_vision_shared_jpeg_io.out_size != (int)VISION_SHARED_RGB565_BYTES))
    {
        ESP_LOGE(
            VISION_SHARED_TAG,
            "esp_new_jpeg produced %d bytes, expected %u",
            s_vision_shared_jpeg_io.out_size,
            (unsigned)VISION_SHARED_RGB565_BYTES);
        return ESP_ERR_INVALID_SIZE;
    }

    vision_shared_finish_rgb565_and_gray();
    return ESP_OK;
}

static bool vision_shared_publish(
    uint32_t sequence,
    int64_t frame_timestamp_us,
    int64_t decode_timestamp_us,
    uint32_t decode_time_us)
{
    int slot_index = -1;

    if (xSemaphoreTake(s_vision_shared_mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    for (size_t i = 0U; i < VISION_SHARED_CACHE_SLOTS; ++i)
    {
        if (((int)i != s_vision_shared_latest_slot) &&
            (!s_vision_shared_slots[i].writing) &&
            (s_vision_shared_slots[i].ref_count == 0U))
        {
            slot_index = (int)i;
            s_vision_shared_slots[i].writing = true;
            break;
        }
    }

    if (slot_index < 0)
    {
        ++s_vision_shared_stats.dropped_no_publish_slot;
        xSemaphoreGive(s_vision_shared_mutex);
        return false;
    }

    xSemaphoreGive(s_vision_shared_mutex);

    vision_shared_slot_t *slot =
        &s_vision_shared_slots[slot_index];

    memcpy(
        slot->rgb565,
        s_vision_shared_work_rgb565,
        VISION_SHARED_RGB565_BYTES);

    memcpy(
        slot->gray8,
        s_vision_shared_work_gray8,
        VISION_SHARED_GRAY8_BYTES);

    if (xSemaphoreTake(s_vision_shared_mutex, portMAX_DELAY) != pdTRUE)
    {
        slot->writing = false;
        return false;
    }

    slot->sequence = sequence;
    slot->frame_timestamp_us = frame_timestamp_us;
    slot->decode_timestamp_us = decode_timestamp_us;
    slot->decode_time_us = decode_time_us;
    slot->valid = true;
    slot->writing = false;

    s_vision_shared_latest_slot = slot_index;

    ++s_vision_shared_stats.decoded_frames;
    s_vision_shared_stats.last_decode_time_us = decode_time_us;
    s_vision_shared_stats.total_decode_time_us += decode_time_us;
    if (decode_time_us > s_vision_shared_stats.max_decode_time_us)
    {
        s_vision_shared_stats.max_decode_time_us = decode_time_us;
    }

    xSemaphoreGive(s_vision_shared_mutex);
    return true;
}

static void vision_shared_task(void *arg)
{
    (void)arg;

    uint32_t last_sequence = UINT32_MAX;

    TickType_t poll_ticks =
        pdMS_TO_TICKS(VISION_SHARED_POLL_MS);

    /* Never allow this polling loop to become a zero-delay busy loop. */
    if (poll_ticks == 0)
    {
        poll_ticks = 1;
    }

    ESP_LOGI(
        VISION_SHARED_TAG,
        "Single decoder task started core=%d poll=%u ms ticks=%u",
        xPortGetCoreID(),
        (unsigned)VISION_SHARED_POLL_MS,
        (unsigned)poll_ticks);

    while (1)
    {
        uvc_module_frame_t frame = {0};

        const esp_err_t frame_ret =
            uvc_module_acquire_latest_frame(
                last_sequence,
                &frame);

        if (frame_ret == ESP_OK)
        {
            const uint32_t sequence = frame.sequence;
            const int64_t frame_timestamp_us =
                frame.timestamp_us;

            const int64_t decode_start =
                esp_timer_get_time();

            const esp_err_t decode_ret =
                vision_shared_decode_one(
                    frame.data,
                    frame.len);

            const int64_t decode_end =
                esp_timer_get_time();

            uvc_module_release_frame(&frame);
            last_sequence = sequence;

            const uint32_t decode_time_us =
                (uint32_t)(
                    (decode_end >= decode_start)
                    ? (decode_end - decode_start)
                    : 0);

            if (decode_ret == ESP_OK)
            {
                (void)vision_shared_publish(
                    sequence,
                    frame_timestamp_us,
                    decode_end,
                    decode_time_us);
            }
            else
            {
                if (xSemaphoreTake(
                        s_vision_shared_mutex,
                        portMAX_DELAY) == pdTRUE)
                {
                    ++s_vision_shared_stats.decode_errors;
                    xSemaphoreGive(
                        s_vision_shared_mutex);
                }

                ESP_LOGW(
                    VISION_SHARED_TAG,
                    "JPEG decode failed: seq=%u err=%s",
                    (unsigned)sequence,
                    esp_err_to_name(decode_ret));
            }
        }
        else if ((frame_ret != ESP_ERR_NOT_FOUND) &&
                 (frame_ret != ESP_ERR_INVALID_STATE) &&
                 (frame_ret != ESP_ERR_TIMEOUT))
        {
            ESP_LOGW(
                VISION_SHARED_TAG,
                "Acquire UVC frame failed: %s",
                esp_err_to_name(frame_ret));
        }

        /* Guaranteed >= 1 FreeRTOS tick. */
        vTaskDelay(poll_ticks);
    }
}


esp_err_t vision_shared_decode_init(void)
{
    if (s_vision_shared_initialized)
    {
        return ESP_OK;
    }

    if ((VISION_SHARED_IMAGE_WIDTH == 0U) ||
        (VISION_SHARED_IMAGE_HEIGHT == 0U) ||
        (VISION_SHARED_CAMERA_FPS == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_vision_shared_mutex = xSemaphoreCreateMutex();
    if (s_vision_shared_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_vision_shared_work_rgb565 =
        (uint8_t *)jpeg_calloc_align(VISION_SHARED_RGB565_BYTES, 16);

    s_vision_shared_work_gray8 =
        (uint8_t *)vision_shared_alloc(
            VISION_SHARED_GRAY8_BYTES,
            true);

    for (size_t i = 0U; i < VISION_SHARED_CACHE_SLOTS; ++i)
    {
        s_vision_shared_slots[i].storage =
            (uint8_t *)vision_shared_alloc(
                VISION_SHARED_RGB565_BYTES + VISION_SHARED_GRAY8_BYTES,
                false);

        if (s_vision_shared_slots[i].storage != NULL)
        {
            s_vision_shared_slots[i].rgb565 =
                s_vision_shared_slots[i].storage;
            s_vision_shared_slots[i].gray8 =
                s_vision_shared_slots[i].storage + VISION_SHARED_RGB565_BYTES;
        }
    }

    bool alloc_ok =
        (s_vision_shared_work_rgb565 != NULL) &&
        (s_vision_shared_work_gray8 != NULL);

    for (size_t i = 0U; i < VISION_SHARED_CACHE_SLOTS; ++i)
    {
        alloc_ok = alloc_ok &&
            (s_vision_shared_slots[i].storage != NULL);
    }

    if (!alloc_ok)
    {
        vision_shared_free_resources();
        return ESP_ERR_NO_MEM;
    }

    if (((VISION_SHARED_IMAGE_WIDTH % 8U) != 0U) ||
        ((VISION_SHARED_IMAGE_HEIGHT % 8U) != 0U))
    {
        ESP_LOGE(
            VISION_SHARED_TAG,
            "esp_new_jpeg scale output must be a multiple of 8: %ux%u",
            (unsigned)VISION_SHARED_IMAGE_WIDTH,
            (unsigned)VISION_SHARED_IMAGE_HEIGHT);
        vision_shared_free_resources();
        return ESP_ERR_INVALID_SIZE;
    }

    jpeg_dec_config_t jpeg_cfg = DEFAULT_JPEG_DEC_CONFIG();
    jpeg_cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    jpeg_cfg.scale.width = (uint16_t)VISION_SHARED_IMAGE_WIDTH;
    jpeg_cfg.scale.height = (uint16_t)VISION_SHARED_IMAGE_HEIGHT;
    jpeg_cfg.rotate = JPEG_ROTATE_0D;
    jpeg_cfg.block_enable = false;

    const jpeg_error_t jpeg_open_ret =
        jpeg_dec_open(&jpeg_cfg, &s_vision_shared_jpeg_decoder);

    if (jpeg_open_ret != JPEG_ERR_OK)
    {
        ESP_LOGE(
            VISION_SHARED_TAG,
            "jpeg_dec_open failed: %d",
            (int)jpeg_open_ret);
        vision_shared_free_resources();
        return vision_shared_jpeg_error_to_esp(jpeg_open_ret);
    }

    esp_err_t ret = ESP_OK;

    if (!uvc_module_is_running())
    {
        ret = uvc_module_init(
            (uint16_t)VISION_SHARED_CAMERA_WIDTH,
            (uint16_t)VISION_SHARED_CAMERA_HEIGHT,
            (uint8_t)VISION_SHARED_CAMERA_FPS);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                VISION_SHARED_TAG,
                "uvc_module_init failed: %s",
                esp_err_to_name(ret));
            vision_shared_free_resources();
            return ret;
        }

        s_vision_shared_owns_uvc = true;
    }
    else
    {
        ESP_LOGI(VISION_SHARED_TAG, "Reusing already-running UVC stream");
    }

    const BaseType_t task_ret =
        xTaskCreatePinnedToCore(
            vision_shared_task,
            "vision_decode",
            VISION_SHARED_TASK_STACK_SIZE,
            NULL,
            VISION_SHARED_TASK_PRIORITY,
            &s_vision_shared_task_handle,
            VISION_SHARED_TASK_CORE);

    if (task_ret != pdPASS)
    {
        s_vision_shared_task_handle = NULL;

        if (s_vision_shared_owns_uvc)
        {
            (void)uvc_module_stop();
            s_vision_shared_owns_uvc = false;
        }

        vision_shared_free_resources();
        return ESP_ERR_NO_MEM;
    }

    s_vision_shared_initialized = true;

    ESP_LOGI(
        VISION_SHARED_TAG,
        "Initialized esp_new_jpeg ONE-decode: UVC=%ux%u@%u target=%ux%u RGB565_LE+GRAY8 mirror=%u cache=%u",
        (unsigned)VISION_SHARED_CAMERA_WIDTH,
        (unsigned)VISION_SHARED_CAMERA_HEIGHT,
        (unsigned)VISION_SHARED_CAMERA_FPS,
        (unsigned)VISION_SHARED_IMAGE_WIDTH,
        (unsigned)VISION_SHARED_IMAGE_HEIGHT,
        (unsigned)(IMAGE_DECODE_MIRROR_X ? 1U : 0U),
        (unsigned)VISION_SHARED_CACHE_SLOTS);

    return ESP_OK;
}

bool vision_shared_decode_is_initialized(void)
{
    return s_vision_shared_initialized;
}

esp_err_t vision_shared_decode_acquire_latest(
    uint32_t after_sequence,
    vision_shared_frame_t *out_frame)
{
    if (out_frame == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_frame, 0, sizeof(*out_frame));

    if ((!s_vision_shared_initialized) ||
        (s_vision_shared_mutex == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_vision_shared_mutex, 0) != pdTRUE)
    {
        /* Counter update is deliberately best-effort; do not wait for it. */
        return ESP_ERR_TIMEOUT;
    }

    const int slot_index = s_vision_shared_latest_slot;

    if ((slot_index < 0) ||
        (!s_vision_shared_slots[slot_index].valid) ||
        s_vision_shared_slots[slot_index].writing)
    {
        xSemaphoreGive(s_vision_shared_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    vision_shared_slot_t *slot =
        &s_vision_shared_slots[slot_index];

    if ((after_sequence != UINT32_MAX) &&
        (slot->sequence == after_sequence))
    {
        xSemaphoreGive(s_vision_shared_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    ++slot->ref_count;

    out_frame->rgb565 = slot->rgb565;
    out_frame->gray8 = slot->gray8;
    out_frame->width = (uint16_t)VISION_SHARED_IMAGE_WIDTH;
    out_frame->height = (uint16_t)VISION_SHARED_IMAGE_HEIGHT;
    out_frame->rgb565_stride_bytes =
        (uint16_t)(VISION_SHARED_IMAGE_WIDTH * 2U);
    out_frame->sequence = slot->sequence;
    out_frame->frame_timestamp_us = slot->frame_timestamp_us;
    out_frame->decode_timestamp_us = slot->decode_timestamp_us;
    out_frame->decode_time_us = slot->decode_time_us;
    out_frame->_slot = (uint8_t)slot_index;
    out_frame->_acquired = true;

    xSemaphoreGive(s_vision_shared_mutex);
    return ESP_OK;
}

void vision_shared_decode_release(vision_shared_frame_t *frame)
{
    if ((frame == NULL) ||
        (!frame->_acquired) ||
        (s_vision_shared_mutex == NULL))
    {
        return;
    }

    if (xSemaphoreTake(s_vision_shared_mutex, portMAX_DELAY) == pdTRUE)
    {
        const size_t slot_index = (size_t)frame->_slot;

        if ((slot_index < VISION_SHARED_CACHE_SLOTS) &&
            (s_vision_shared_slots[slot_index].ref_count > 0U))
        {
            --s_vision_shared_slots[slot_index].ref_count;
        }

        xSemaphoreGive(s_vision_shared_mutex);
    }

    memset(frame, 0, sizeof(*frame));
}

void vision_shared_decode_get_stats(vision_shared_decode_stats_t *out_stats)
{
    if (out_stats == NULL)
    {
        return;
    }

    memset(out_stats, 0, sizeof(*out_stats));

    if (s_vision_shared_mutex == NULL)
    {
        return;
    }

    if (xSemaphoreTake(s_vision_shared_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *out_stats = s_vision_shared_stats;
        xSemaphoreGive(s_vision_shared_mutex);
    }
}

#endif /* VISION_SHARED_DECODE_IMPLEMENTATION && !IMPLEMENTED */
