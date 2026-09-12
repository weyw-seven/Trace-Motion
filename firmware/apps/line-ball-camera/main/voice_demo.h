#ifndef VOICE_DEMO_H
#define VOICE_DEMO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 语音 Demo（PCM 文件 → USB UAC 扬声器）
 *
 * UAC 设备由 uvc_module_init() 通过 uac_streaming_config() 完成注册；
 * 本模块仅负责按需打开 SPIFFS 中的 PCM 并通过
 * uac_spk_streaming_write() 发送。
 * ============================================================ */

#ifndef VOICE_DEMO_FILE_AVOID2
#define VOICE_DEMO_FILE_AVOID2          "/spiffs/avoid2.pcm"
#endif
#ifndef VOICE_DEMO_FILE_MAMA
#define VOICE_DEMO_FILE_MAMA            "/spiffs/MaMa.pcm"
#endif

#ifndef VOICE_DEMO_CHUNK_SIZE
#define VOICE_DEMO_CHUNK_SIZE           1024U
#endif

#ifndef VOICE_DEMO_TASK_STACK
#define VOICE_DEMO_TASK_STACK           4096
#endif
#ifndef VOICE_DEMO_TASK_PRIORITY
#define VOICE_DEMO_TASK_PRIORITY        3
#endif

#ifndef VOICE_DEMO_WRITE_TIMEOUT_MS
#define VOICE_DEMO_WRITE_TIMEOUT_MS     1000U
#endif

typedef enum {
    VOICE_CLIP_NONE = 0,
    VOICE_CLIP_AVOID2,
    VOICE_CLIP_MAMA,
    VOICE_CLIP_COUNT
} voice_clip_t;

esp_err_t voice_demo_init(bool enabled);
void      voice_demo_set_enabled(bool enabled);
bool      voice_demo_is_enabled(void);
esp_err_t voice_demo_play_clip(voice_clip_t clip);
void      voice_demo_stop(void);
bool      voice_demo_is_playing(void);

#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */
#ifdef VOICE_DEMO_IMPLEMENTATION

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb_stream.h"

static const char *VOICE_TAG = "VOICE_DEMO";

static TaskHandle_t          s_task        = NULL;
static SemaphoreHandle_t     s_lock        = NULL;
static bool                  s_enabled     = false;
static bool                  s_initialized = false;
static volatile bool         s_playing     = false;
static volatile voice_clip_t s_pending     = VOICE_CLIP_NONE;

static const char *voice_clip_path(voice_clip_t clip)
{
    switch (clip) {
        case VOICE_CLIP_AVOID2: return VOICE_DEMO_FILE_AVOID2;
        case VOICE_CLIP_MAMA:   return VOICE_DEMO_FILE_MAMA;
        default:                return NULL;
    }
}

static void voice_play_file(const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        ESP_LOGE(VOICE_TAG, "Cannot open PCM: %s", path);
        return;
    }

    uint8_t *chunk = (uint8_t *)malloc(VOICE_DEMO_CHUNK_SIZE);
    if (chunk == NULL) {
        ESP_LOGE(VOICE_TAG, "no mem for chunk");
        fclose(fp);
        return;
    }

    size_t total_sent = 0;
    size_t n;
    while ((n = fread(chunk, 1, VOICE_DEMO_CHUNK_SIZE, fp)) > 0U) {
        esp_err_t ret = uac_spk_streaming_write(
            chunk, n, pdMS_TO_TICKS(VOICE_DEMO_WRITE_TIMEOUT_MS));
        if (ret != ESP_OK) {
            ESP_LOGE(VOICE_TAG,
                     "uac write failed at %u bytes, ret=%s",
                     (unsigned)total_sent, esp_err_to_name(ret));
            break;
        }
        total_sent += n;
    }

    free(chunk);
    fclose(fp);
    ESP_LOGI(VOICE_TAG, "PLAY done: %s, %u bytes",
             path, (unsigned)total_sent);
}

static void voice_task(void *arg)
{
    (void)arg;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        voice_clip_t clip = VOICE_CLIP_NONE;
        if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            clip = s_pending;
            s_pending = VOICE_CLIP_NONE;
            xSemaphoreGive(s_lock);
        }

        if (clip == VOICE_CLIP_NONE || !s_enabled) {
            continue;
        }

        const char *path = voice_clip_path(clip);
        if (path == NULL) {
            continue;
        }

        s_playing = true;
        ESP_LOGI(VOICE_TAG, "PLAY clip=%d file=%s", (int)clip, path);
        voice_play_file(path);
        s_playing = false;
    }
}

esp_err_t voice_demo_init(bool enabled)
{
    if (s_initialized) {
        s_enabled = enabled;
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreate(
        voice_task, "voice_demo",
        VOICE_DEMO_TASK_STACK, NULL,
        VOICE_DEMO_TASK_PRIORITY, &s_task);
    if (ok != pdPASS) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_enabled     = enabled;
    s_initialized = true;

    ESP_LOGI(VOICE_TAG, "init done: enabled=%d (USB UAC)", (int)enabled);
    return ESP_OK;
}

void voice_demo_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (!enabled) {
        voice_demo_stop();
    }
}

bool voice_demo_is_enabled(void)
{
    return s_enabled;
}

esp_err_t voice_demo_play_clip(voice_clip_t clip)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_enabled) {
        return ESP_OK;
    }
    if (clip <= VOICE_CLIP_NONE || clip >= VOICE_CLIP_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_pending = clip;
        xSemaphoreGive(s_lock);
    }
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
    return ESP_OK;
}

void voice_demo_stop(void)
{
    if (s_lock != NULL &&
        xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_pending = VOICE_CLIP_NONE;
        xSemaphoreGive(s_lock);
    }
    s_playing = false;
}

bool voice_demo_is_playing(void)
{
    return s_playing;
}

#endif /* VOICE_DEMO_IMPLEMENTATION */
#endif /* VOICE_DEMO_H */