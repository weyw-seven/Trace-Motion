#ifndef UVC_MODULE_H
#define UVC_MODULE_H

/*
 * uvc_module.h
 *
 * First-stage reliable UVC MJPEG capture wrapper for ESP32-S3 + espressif/usb_stream.
 *
 * Design goals:
 *   1) Keep the existing usb_stream.h backend.
 *   2) Never let the decoder read a JPEG buffer while the UVC callback overwrites it.
 *   3) Expose frame sequence/timestamp so consumers process each camera frame once.
 *   4) Keep the implementation simple enough for bring-up and diagnostics.
 *
 * IMPORTANT:
 *   A frame returned by uvc_module_acquire_latest_frame() is pinned.
 *   You MUST call uvc_module_release_frame() after finishing with it.
 *
 * Single-header usage:
 *
 *   In exactly ONE .c file:
 *
 *       #define UVC_MODULE_IMPLEMENTATION
 *       #include "uvc_module.h"
 *
 *   In other files:
 *
 *       #include "uvc_module.h"
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UVC_MODULE_BUFFER_SIZE
/*
 * Espressif's 480x320 usb_stream examples commonly use 55 KiB on ESP32-S3.
 * If your logs report oversize MJPEG frames, raise this value.
 */
#define UVC_MODULE_BUFFER_SIZE          (55U * 1024U)
#endif

#ifndef UVC_MODULE_CACHE_SLOTS
#define UVC_MODULE_CACHE_SLOTS          2U
#endif

#if (UVC_MODULE_CACHE_SLOTS < 2)
#error "UVC_MODULE_CACHE_SLOTS must be >= 2 for safe acquire/release operation"
#endif

#define UVC_MODULE_ANY_SEQUENCE         UINT32_MAX

#ifndef UVC_MODULE_ENABLE_UAC
#define UVC_MODULE_ENABLE_UAC           1
#endif

typedef struct
{
    const uint8_t *data;
    size_t len;

    uint32_t width;
    uint32_t height;
    uint32_t sequence;
    int64_t timestamp_us;

    /*
     * Internal token used by uvc_module_release_frame().
     * Do not modify these fields.
     */
    uint8_t _slot;
    bool _acquired;

} uvc_module_frame_t;

typedef struct
{
    uint32_t accepted_frames;
    uint32_t dropped_busy;
    uint32_t dropped_oversize;
    uint32_t dropped_non_mjpeg;
    uint32_t dropped_no_free_slot;
    size_t max_frame_len;

} uvc_module_stats_t;

/**
 * @brief Configure and start UVC MJPEG streaming.
 *
 * @param width   Requested camera width.
 * @param height  Requested camera height.
 * @param fps     Requested frames per second. Must be > 0.
 *
 * @note This function starts usb_stream but does not require that the camera
 *       is already connected. Use uvc_module_wait_connected() if desired.
 */
esp_err_t uvc_module_init(uint16_t width, uint16_t height, uint8_t fps);

/**
 * @brief Wait until usb_stream reports a connected device.
 *
 * @param timeout_ms Timeout in milliseconds. UINT32_MAX means wait forever.
 */
esp_err_t uvc_module_wait_connected(uint32_t timeout_ms);

/**
 * @brief Acquire the latest complete MJPEG frame without copying it.
 *
 * The returned buffer is pinned and will not be overwritten until
 * uvc_module_release_frame() is called.
 *
 * @param after_sequence
 *        UVC_MODULE_ANY_SEQUENCE: return the latest frame even if already seen.
 *        Otherwise: return ESP_ERR_NOT_FOUND when latest sequence equals it.
 *
 * @return ESP_OK on success.
 *         ESP_ERR_NOT_FOUND when no new frame is available.
 *         ESP_ERR_INVALID_STATE when the module is not running / no frame yet.
 */
esp_err_t uvc_module_acquire_latest_frame(
    uint32_t after_sequence,
    uvc_module_frame_t *out_frame);

/**
 * @brief Release a frame previously acquired.
 */
void uvc_module_release_frame(uvc_module_frame_t *frame);

bool uvc_module_is_running(void);
bool uvc_module_is_connected(void);
size_t uvc_module_buffer_size(void);

void uvc_module_get_stats(uvc_module_stats_t *out_stats);

/**
 * @brief Stop streaming and free buffers.
 *
 * Returns ESP_ERR_INVALID_STATE if a consumer still holds an acquired frame.
 */
esp_err_t uvc_module_stop(void);

#ifdef __cplusplus
}
#endif


/* ========================================================================== */
/* IMPLEMENTATION                                                             */
/* ========================================================================== */

#ifdef UVC_MODULE_IMPLEMENTATION

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "usb_stream.h"

static const char *UVC_MODULE_TAG = "UVC_MODULE";

typedef struct
{
    uint8_t *data;
    size_t len;

    uint32_t width;
    uint32_t height;
    uint32_t sequence;
    int64_t timestamp_us;

    uint32_t ref_count;
    bool valid;

} uvc_module_slot_t;

static SemaphoreHandle_t s_uvc_lock = NULL;

static uint8_t *s_uvc_xfer_a = NULL;
static uint8_t *s_uvc_xfer_b = NULL;
static uint8_t *s_uvc_driver_frame = NULL;

static uvc_module_slot_t s_uvc_slots[UVC_MODULE_CACHE_SLOTS];
static int s_uvc_latest_slot = -1;

static volatile bool s_uvc_running = false;
static volatile bool s_uvc_connected = false;

static uvc_module_stats_t s_uvc_stats;

static void *uvc_module_alloc_large(size_t size)
{
    void *p = NULL;

#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    /*
     * usb_stream examples have used SPIRAM for these large buffers.
     * Prefer it when available to preserve internal SRAM.
     */
    p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif

    if (p == NULL)
    {
        p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }

    return p;
}

static void uvc_module_free_all_buffers(void)
{
    for (size_t i = 0; i < UVC_MODULE_CACHE_SLOTS; ++i)
    {
        if (s_uvc_slots[i].data != NULL)
        {
            free(s_uvc_slots[i].data);
            s_uvc_slots[i].data = NULL;
        }

        memset(&s_uvc_slots[i], 0, sizeof(s_uvc_slots[i]));
    }

    if (s_uvc_xfer_a != NULL)
    {
        free(s_uvc_xfer_a);
        s_uvc_xfer_a = NULL;
    }

    if (s_uvc_xfer_b != NULL)
    {
        free(s_uvc_xfer_b);
        s_uvc_xfer_b = NULL;
    }

    if (s_uvc_driver_frame != NULL)
    {
        free(s_uvc_driver_frame);
        s_uvc_driver_frame = NULL;
    }

    s_uvc_latest_slot = -1;
}

static void uvc_module_stream_state_cb(usb_stream_state_t event, void *arg)
{
    (void)arg;

    switch (event)
    {
        case STREAM_CONNECTED:
            s_uvc_connected = true;
            ESP_LOGI(UVC_MODULE_TAG, "USB camera connected");
            break;

        case STREAM_DISCONNECTED:
            s_uvc_connected = false;
            ESP_LOGW(UVC_MODULE_TAG, "USB camera disconnected");
            break;

        default:
            ESP_LOGW(UVC_MODULE_TAG, "USB stream state event=%d", (int)event);
            break;
    }
}

/* ========================================================================== */
/* UAC (USB Audio Class) 扬声器配置                                            */
/* 必须在 usb_streaming_start() 之前调用 uac_streaming_config()               */
/* ========================================================================== */
#if UVC_MODULE_ENABLE_UAC

static void uvc_module_mic_cb(mic_frame_t *frame, void *arg)
{
    (void)arg;
    (void)frame;
    /* 不处理麦克风数据；如需回放，可自行调用 uac_spk_streaming_write() */
}

static void uvc_module_configure_uac(void)
{
    uac_config_t uac_cfg;
    memset(&uac_cfg, 0, sizeof(uac_cfg));

    uac_cfg.mic_bit_resolution      = UAC_BITS_ANY;
    uac_cfg.mic_samples_frequence   = UAC_FREQUENCY_ANY;
    uac_cfg.spk_bit_resolution      = 16;
    uac_cfg.spk_samples_frequence   = 16000;
    uac_cfg.spk_buf_size            = 4096;
    uac_cfg.mic_cb                  = uvc_module_mic_cb;
    uac_cfg.mic_cb_arg              = NULL;
    uac_cfg.flags                   = 0;

    esp_err_t ret = uac_streaming_config(&uac_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(UVC_MODULE_TAG,
                 "uac_streaming_config failed: %s (camera may have no speaker)",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(UVC_MODULE_TAG, "UAC configured: 16-bit 16kHz speaker");
    }
}

#endif /* UVC_MODULE_ENABLE_UAC */

static void uvc_module_internal_frame_cb(uvc_frame_t *frame, void *arg)
{
    (void)arg;

    if ((!s_uvc_running) || (frame == NULL) || (s_uvc_lock == NULL))
    {
        return;
    }

    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG)
    {
        s_uvc_stats.dropped_non_mjpeg++;
        return;
    }

    if ((frame->data == NULL) || (frame->data_bytes == 0U))
    {
        return;
    }

    if (frame->data_bytes > UVC_MODULE_BUFFER_SIZE)
    {
        s_uvc_stats.dropped_oversize++;

        if ((s_uvc_stats.dropped_oversize <= 5U) ||
            ((s_uvc_stats.dropped_oversize % 50U) == 0U))
        {
            ESP_LOGE(
                UVC_MODULE_TAG,
                "MJPEG frame too large: %u > %u bytes. Increase UVC_MODULE_BUFFER_SIZE.",
                (unsigned)frame->data_bytes,
                (unsigned)UVC_MODULE_BUFFER_SIZE);
        }

        return;
    }

    /*
     * Never block the usb_stream callback waiting for a consumer.
     * If the lock is momentarily busy, dropping one frame is preferable.
     */
    if (xSemaphoreTake(s_uvc_lock, 0) != pdTRUE)
    {
        s_uvc_stats.dropped_busy++;
        return;
    }

    int write_slot = -1;

    /*
     * Prefer a non-latest, unreferenced slot so the currently published
     * frame remains untouched until the new copy is complete.
     */
    for (size_t i = 0; i < UVC_MODULE_CACHE_SLOTS; ++i)
    {
        if (((int)i != s_uvc_latest_slot) &&
            (s_uvc_slots[i].ref_count == 0U))
        {
            write_slot = (int)i;
            break;
        }
    }

    /*
     * If every non-latest slot is pinned, the current latest slot itself may
     * still be overwritten when nobody has acquired it.
     */
    if ((write_slot < 0) &&
        (s_uvc_latest_slot >= 0) &&
        (s_uvc_slots[s_uvc_latest_slot].ref_count == 0U))
    {
        write_slot = s_uvc_latest_slot;
    }

    if (write_slot < 0)
    {
        s_uvc_stats.dropped_no_free_slot++;
        xSemaphoreGive(s_uvc_lock);
        return;
    }

    uvc_module_slot_t *slot = &s_uvc_slots[write_slot];

    memcpy(slot->data, frame->data, frame->data_bytes);

    slot->len = frame->data_bytes;
    slot->width = frame->width;
    slot->height = frame->height;
    slot->sequence = frame->sequence;
    slot->timestamp_us = esp_timer_get_time();
    slot->valid = true;

    s_uvc_latest_slot = write_slot;

    s_uvc_stats.accepted_frames++;
    if (slot->len > s_uvc_stats.max_frame_len)
    {
        s_uvc_stats.max_frame_len = slot->len;
    }

    xSemaphoreGive(s_uvc_lock);
}

esp_err_t uvc_module_init(uint16_t width, uint16_t height, uint8_t fps)
{
    if ((width == 0U) || (height == 0U) || (fps == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_uvc_running)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_uvc_stats, 0, sizeof(s_uvc_stats));
    memset(s_uvc_slots, 0, sizeof(s_uvc_slots));
    s_uvc_latest_slot = -1;
    s_uvc_connected = false;

    s_uvc_lock = xSemaphoreCreateMutex();
    if (s_uvc_lock == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_uvc_xfer_a = (uint8_t *)uvc_module_alloc_large(UVC_MODULE_BUFFER_SIZE);
    s_uvc_xfer_b = (uint8_t *)uvc_module_alloc_large(UVC_MODULE_BUFFER_SIZE);
    s_uvc_driver_frame = (uint8_t *)uvc_module_alloc_large(UVC_MODULE_BUFFER_SIZE);

    for (size_t i = 0; i < UVC_MODULE_CACHE_SLOTS; ++i)
    {
        s_uvc_slots[i].data =
            (uint8_t *)uvc_module_alloc_large(UVC_MODULE_BUFFER_SIZE);
    }

    bool alloc_ok =
        (s_uvc_xfer_a != NULL) &&
        (s_uvc_xfer_b != NULL) &&
        (s_uvc_driver_frame != NULL);

    for (size_t i = 0; i < UVC_MODULE_CACHE_SLOTS; ++i)
    {
        alloc_ok = alloc_ok && (s_uvc_slots[i].data != NULL);
    }

    if (!alloc_ok)
    {
        ESP_LOGE(
            UVC_MODULE_TAG,
            "Buffer allocation failed. Need about %u KiB for UVC buffers. "
            "For bring-up, disable Wi-Fi/WebStream or enable PSRAM.",
            (unsigned)(((3U + UVC_MODULE_CACHE_SLOTS) * UVC_MODULE_BUFFER_SIZE) / 1024U));

        uvc_module_free_all_buffers();
        vSemaphoreDelete(s_uvc_lock);
        s_uvc_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    /*
     * UVC frame interval uses 100 ns units:
     *     interval = 10,000,000 / fps
     */
    const uint32_t frame_interval = 10000000UL / (uint32_t)fps;

    uvc_config_t cfg =
    {
        .frame_width = width,
        .frame_height = height,
        .frame_interval = frame_interval,

        .xfer_buffer_size = UVC_MODULE_BUFFER_SIZE,
        .xfer_buffer_a = s_uvc_xfer_a,
        .xfer_buffer_b = s_uvc_xfer_b,

        .frame_buffer_size = UVC_MODULE_BUFFER_SIZE,
        .frame_buffer = s_uvc_driver_frame,

        .frame_cb = uvc_module_internal_frame_cb,
        .frame_cb_arg = NULL,
    };

    esp_err_t err = uvc_streaming_config(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(UVC_MODULE_TAG, "uvc_streaming_config failed: %s", esp_err_to_name(err));
        uvc_module_free_all_buffers();
        vSemaphoreDelete(s_uvc_lock);
        s_uvc_lock = NULL;
        return err;
    }

    err = usb_streaming_state_register(uvc_module_stream_state_cb, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(UVC_MODULE_TAG, "usb_streaming_state_register failed: %s", esp_err_to_name(err));
        uvc_module_free_all_buffers();
        vSemaphoreDelete(s_uvc_lock);
        s_uvc_lock = NULL;
        return err;
    }

#if UVC_MODULE_ENABLE_UAC
    uvc_module_configure_uac();
#endif

    /*
     * Set running before start so an early callback is accepted.
     */
    s_uvc_running = true;

    err = usb_streaming_start();
    if (err != ESP_OK)
    {
        s_uvc_running = false;
        ESP_LOGE(UVC_MODULE_TAG, "usb_streaming_start failed: %s", esp_err_to_name(err));
        uvc_module_free_all_buffers();
        vSemaphoreDelete(s_uvc_lock);
        s_uvc_lock = NULL;
        return err;
    }

    ESP_LOGI(
        UVC_MODULE_TAG,
        "Started: request=%ux%u @ %u FPS, JPEG buffer=%u bytes, cache_slots=%u",
        (unsigned)width,
        (unsigned)height,
        (unsigned)fps,
        (unsigned)UVC_MODULE_BUFFER_SIZE,
        (unsigned)UVC_MODULE_CACHE_SLOTS);

    return ESP_OK;
}

esp_err_t uvc_module_wait_connected(uint32_t timeout_ms)
{
    if (!s_uvc_running)
    {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks =
        (timeout_ms == UINT32_MAX)
        ? portMAX_DELAY
        : pdMS_TO_TICKS(timeout_ms);

    esp_err_t err = usb_streaming_connect_wait(ticks);

    if (err == ESP_OK)
    {
        s_uvc_connected = true;
    }

    return err;
}

esp_err_t uvc_module_acquire_latest_frame(
    uint32_t after_sequence,
    uvc_module_frame_t *out_frame)
{
    if (out_frame == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_frame, 0, sizeof(*out_frame));

    if ((!s_uvc_running) || (s_uvc_lock == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_uvc_lock, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (s_uvc_latest_slot < 0)
    {
        xSemaphoreGive(s_uvc_lock);
        return ESP_ERR_INVALID_STATE;
    }

    uvc_module_slot_t *slot = &s_uvc_slots[s_uvc_latest_slot];

    if (!slot->valid)
    {
        xSemaphoreGive(s_uvc_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if ((after_sequence != UVC_MODULE_ANY_SEQUENCE) &&
        (slot->sequence == after_sequence))
    {
        xSemaphoreGive(s_uvc_lock);
        return ESP_ERR_NOT_FOUND;
    }

    slot->ref_count++;

    out_frame->data = slot->data;
    out_frame->len = slot->len;
    out_frame->width = slot->width;
    out_frame->height = slot->height;
    out_frame->sequence = slot->sequence;
    out_frame->timestamp_us = slot->timestamp_us;
    out_frame->_slot = (uint8_t)s_uvc_latest_slot;
    out_frame->_acquired = true;

    xSemaphoreGive(s_uvc_lock);

    return ESP_OK;
}

void uvc_module_release_frame(uvc_module_frame_t *frame)
{
    if ((frame == NULL) ||
        (!frame->_acquired) ||
        (s_uvc_lock == NULL) ||
        (frame->_slot >= UVC_MODULE_CACHE_SLOTS))
    {
        return;
    }

    if (xSemaphoreTake(s_uvc_lock, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        uvc_module_slot_t *slot = &s_uvc_slots[frame->_slot];

        if (slot->ref_count > 0U)
        {
            slot->ref_count--;
        }

        xSemaphoreGive(s_uvc_lock);
    }

    frame->data = NULL;
    frame->len = 0U;
    frame->_acquired = false;
}

bool uvc_module_is_running(void)
{
    return s_uvc_running;
}

bool uvc_module_is_connected(void)
{
    return s_uvc_connected;
}

size_t uvc_module_buffer_size(void)
{
    return UVC_MODULE_BUFFER_SIZE;
}

void uvc_module_get_stats(uvc_module_stats_t *out_stats)
{
    if (out_stats == NULL)
    {
        return;
    }

    /*
     * Approximate diagnostic snapshot. Exact atomicity is not important here.
     */
    *out_stats = s_uvc_stats;
}

esp_err_t uvc_module_stop(void)
{
    if (!s_uvc_running)
    {
        return ESP_OK;
    }

    if (s_uvc_lock != NULL)
    {
        if (xSemaphoreTake(s_uvc_lock, pdMS_TO_TICKS(50)) != pdTRUE)
        {
            return ESP_ERR_TIMEOUT;
        }

        for (size_t i = 0; i < UVC_MODULE_CACHE_SLOTS; ++i)
        {
            if (s_uvc_slots[i].ref_count != 0U)
            {
                xSemaphoreGive(s_uvc_lock);
                ESP_LOGE(UVC_MODULE_TAG, "Cannot stop: frame slot %u still acquired", (unsigned)i);
                return ESP_ERR_INVALID_STATE;
            }
        }

        xSemaphoreGive(s_uvc_lock);
    }

    s_uvc_running = false;
    s_uvc_connected = false;

    /*
     * Call without assigning the return value so this remains compatible
     * with usb_stream releases where the function return type differs.
     */
    (void)usb_streaming_stop();

    /*
     * Give the backend task a short chance to observe the stop before freeing
     * buffers. This is bring-up code; if your usb_stream version documents a
     * stronger stop synchronization primitive, prefer that.
     */
    vTaskDelay(pdMS_TO_TICKS(50));

    uvc_module_free_all_buffers();

    if (s_uvc_lock != NULL)
    {
        vSemaphoreDelete(s_uvc_lock);
        s_uvc_lock = NULL;
    }

    ESP_LOGI(UVC_MODULE_TAG, "Stopped");
    return ESP_OK;
}

#endif /* UVC_MODULE_IMPLEMENTATION */

#endif /* UVC_MODULE_H */
