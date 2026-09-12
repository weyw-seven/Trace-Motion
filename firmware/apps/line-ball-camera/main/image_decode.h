#ifndef IMAGE_DECODE_H
#define IMAGE_DECODE_H

/*
 * image_decode.h
 *
 * Bring-up JPEG -> grayscale decoder for ESP32-S3 using TJpgDec.
 *
 * Improvements over the original first version:
 *   - detailed jd_prepare()/jd_decomp() errors
 *   - JPEG SOI / SOF diagnostics
 *   - explicit progressive-JPEG rejection
 *   - work buffer sized for JD_FASTDECODE configuration
 *   - no 4 KiB decoder work buffer on the task stack
 *   - supports TJpgDec RGB888, RGB565, or grayscale output configurations
 *   - validates decoded dimensions against caller buffer
 *   - optional horizontal mirror (enabled by default to preserve prior behavior)
 *   - serializes decoder use so the cached work buffer is safe
 *
 * Single-header usage:
 *
 *   In exactly ONE .c file:
 *
 *       #define IMAGE_DECODE_IMPLEMENTATION
 *       #include "image_decode.h"
 *
 *   In other files:
 *
 *       #include "image_decode.h"
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMAGE_DECODE_MIRROR_X
#define IMAGE_DECODE_MIRROR_X 1
#endif

typedef struct
{
    uint8_t *data;

    /*
     * Optional. If 0, the decoder falls back to width*height as capacity,
     * which preserves compatibility with callers that only filled
     * data/width/height in the older struct style.
     */
    size_t capacity_bytes;

    /*
     * Input:
     *   0 means "accept JPEG-derived output dimension".
     *   non-zero means "require this exact dimension".
     *
     * Output:
     *   actual decoded grayscale dimensions.
     */
    uint16_t width;
    uint16_t height;

} grayscale_image_t;

typedef struct
{
    uint16_t width;
    uint16_t height;
    uint8_t sof_marker;   /* 0xC0 baseline, 0xC2 progressive, 0 when unknown */
    bool progressive;

} image_decode_jpeg_info_t;

/**
 * @brief Initialize reusable decoder resources.
 * Safe to call more than once.
 */
esp_err_t image_decode_init(void);

/**
 * @brief Free reusable decoder resources.
 */
void image_decode_deinit(void);

/**
 * @brief Lightweight JPEG header probe.
 *
 * Finds SOF0/SOF2 when present before SOS.
 */
esp_err_t image_decode_probe_jpeg(
    const uint8_t *jpeg_data,
    size_t jpeg_len,
    image_decode_jpeg_info_t *out_info);

/**
 * @brief Decode JPEG to 8-bit grayscale.
 *
 * scale:
 *   0 = 1/1
 *   1 = 1/2
 *   2 = 1/4
 *   3 = 1/8
 */
esp_err_t image_decode_jpeg_to_grayscale(
    const uint8_t *jpeg_data,
    size_t jpeg_len,
    uint8_t scale,
    grayscale_image_t *out_img);

/*
 * Backward-friendly bool wrapper matching the older project call style.
 */
static inline bool decode_jpeg_to_grayscale(
    const uint8_t *jpeg_data,
    size_t jpeg_len,
    uint8_t scale,
    grayscale_image_t *out_img)
{
    return image_decode_jpeg_to_grayscale(
               jpeg_data,
               jpeg_len,
               scale,
               out_img) == ESP_OK;
}

#ifdef __cplusplus
}
#endif


/* ========================================================================== */
/* IMPLEMENTATION                                                             */
/* ========================================================================== */

#ifdef IMAGE_DECODE_IMPLEMENTATION

#include <string.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef __has_include
#define __has_include(x) 0
#endif

/*
 * Match Espressif's wrapper logic:
 * ROM TJpgDec and external TJpgDec use different output callback return types.
 */
#if defined(CONFIG_JD_USE_ROM) && CONFIG_JD_USE_ROM

    #if __has_include("rom/tjpgd.h")
        #include "rom/tjpgd.h"
    #else
        #error "CONFIG_JD_USE_ROM is enabled but rom/tjpgd.h was not found"
    #endif

    #define IMAGE_DECODE_TJPGD_ROM 1

#else

    #if __has_include("tjpgd.h")
        #include "tjpgd.h"
        #define IMAGE_DECODE_TJPGD_ROM 0
    #elif __has_include("rom/tjpgd.h")
        #include "rom/tjpgd.h"
        #define IMAGE_DECODE_TJPGD_ROM 1
    #else
        #error "No TJpgDec header found (tjpgd.h or rom/tjpgd.h)"
    #endif

#endif

#ifndef JD_FASTDECODE
#define JD_FASTDECODE 0
#endif

#ifndef JD_FORMAT
/*
 * ESP32-S3 ROM TJpgDec is RGB888.
 */
#define JD_FORMAT 0
#endif

#if (JD_FASTDECODE == 2)
#define IMAGE_DECODE_WORKBUF_SIZE 65472U
#else
/*
 * 8 KiB comfortably covers the common RGB888 and RGB565 non-fast2 builds.
 * It is deliberately larger than the old 4 KiB stack buffer.
 */
#define IMAGE_DECODE_WORKBUF_SIZE 8192U
#endif

#if (IMAGE_DECODE_TJPGD_ROM)
typedef unsigned int image_decode_out_ret_t;
#else
typedef int image_decode_out_ret_t;
#endif

static const char *IMAGE_DECODE_TAG = "IMAGE_DECODE";

static SemaphoreHandle_t s_image_decode_mutex = NULL;
static uint8_t *s_image_decode_workbuf = NULL;
static bool s_image_decode_logged_config = false;

typedef struct
{
    const uint8_t *in_data;
    size_t in_len;
    size_t in_pos;

    uint8_t *out_data;
    uint16_t out_width;
    uint16_t out_height;

    bool output_overflow;

} image_decode_io_t;

static const char *image_decode_jresult_name(JRESULT r)
{
    switch (r)
    {
        case JDR_OK:   return "JDR_OK";
        case JDR_INTR: return "JDR_INTR";
        case JDR_INP:  return "JDR_INP";
        case JDR_MEM1: return "JDR_MEM1";
        case JDR_MEM2: return "JDR_MEM2";
        case JDR_PAR:  return "JDR_PAR";
        case JDR_FMT1: return "JDR_FMT1";
        case JDR_FMT2: return "JDR_FMT2";
        case JDR_FMT3: return "JDR_FMT3";
        default:       return "JDR_UNKNOWN";
    }
}

static UINT image_decode_input_cb(JDEC *jd, BYTE *buff, UINT nbyte)
{
    if ((jd == NULL) || (jd->device == NULL))
    {
        return 0;
    }

    image_decode_io_t *io = (image_decode_io_t *)jd->device;

    if (io->in_pos >= io->in_len)
    {
        return 0;
    }

    size_t remaining = io->in_len - io->in_pos;
    size_t take = (remaining < (size_t)nbyte) ? remaining : (size_t)nbyte;

    if (buff != NULL)
    {
        memcpy(buff, io->in_data + io->in_pos, take);
    }

    io->in_pos += take;
    return (UINT)take;
}

static inline uint8_t image_decode_rgb_to_gray(
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    /*
     * Integer approximation of:
     *   0.299 R + 0.587 G + 0.114 B
     */
    return (uint8_t)(
        ((uint16_t)77U  * r +
         (uint16_t)150U * g +
         (uint16_t)29U  * b) >> 8);
}

static image_decode_out_ret_t image_decode_output_cb(
    JDEC *jd,
    void *bitmap,
    JRECT *rect)
{
    if ((jd == NULL) ||
        (jd->device == NULL) ||
        (bitmap == NULL) ||
        (rect == NULL))
    {
        return 0;
    }

    image_decode_io_t *io = (image_decode_io_t *)jd->device;
    const uint8_t *src = (const uint8_t *)bitmap;

    const uint32_t block_w =
        (uint32_t)rect->right - (uint32_t)rect->left + 1U;

    const uint32_t block_h =
        (uint32_t)rect->bottom - (uint32_t)rect->top + 1U;

    for (uint32_t by = 0; by < block_h; ++by)
    {
        const uint32_t y = (uint32_t)rect->top + by;

        for (uint32_t bx = 0; bx < block_w; ++bx)
        {
            const uint32_t x = (uint32_t)rect->left + bx;
            const uint32_t src_pixel = by * block_w + bx;

            if ((x >= io->out_width) || (y >= io->out_height))
            {
                io->output_overflow = true;
                return 0;
            }

            uint8_t gray = 0;

#if (JD_FORMAT == 0)

            const uint8_t r = src[src_pixel * 3U + 0U];
            const uint8_t g = src[src_pixel * 3U + 1U];
            const uint8_t b = src[src_pixel * 3U + 2U];

            gray = image_decode_rgb_to_gray(r, g, b);

#elif (JD_FORMAT == 1)

            /*
             * TJpgDec RGB565 word on ESP32-S3 little-endian target.
             */
            const uint16_t p =
                (uint16_t)src[src_pixel * 2U + 0U] |
                ((uint16_t)src[src_pixel * 2U + 1U] << 8);

            const uint8_t r5 = (uint8_t)((p >> 11) & 0x1FU);
            const uint8_t g6 = (uint8_t)((p >> 5)  & 0x3FU);
            const uint8_t b5 = (uint8_t)(p & 0x1FU);

            const uint8_t r = (uint8_t)((r5 * 255U + 15U) / 31U);
            const uint8_t g = (uint8_t)((g6 * 255U + 31U) / 63U);
            const uint8_t b = (uint8_t)((b5 * 255U + 15U) / 31U);

            gray = image_decode_rgb_to_gray(r, g, b);

#elif (JD_FORMAT == 2)

            gray = src[src_pixel];

#else
#error "Unsupported JD_FORMAT. Expected 0=RGB888, 1=RGB565, or 2=grayscale."
#endif

#if IMAGE_DECODE_MIRROR_X
            const uint32_t dst_x = (uint32_t)io->out_width - 1U - x;
#else
            const uint32_t dst_x = x;
#endif

            io->out_data[y * (uint32_t)io->out_width + dst_x] = gray;
        }
    }

    return 1;
}

esp_err_t image_decode_init(void)
{
    if (s_image_decode_mutex == NULL)
    {
        s_image_decode_mutex = xSemaphoreCreateMutex();

        if (s_image_decode_mutex == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_image_decode_workbuf == NULL)
    {
        s_image_decode_workbuf =
            (uint8_t *)heap_caps_malloc(
                IMAGE_DECODE_WORKBUF_SIZE,
                MALLOC_CAP_8BIT);

        if (s_image_decode_workbuf == NULL)
        {
            ESP_LOGE(
                IMAGE_DECODE_TAG,
                "Failed to allocate TJpgDec work buffer: %u bytes",
                (unsigned)IMAGE_DECODE_WORKBUF_SIZE);

            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_image_decode_logged_config)
    {
        ESP_LOGI(
            IMAGE_DECODE_TAG,
            "TJpgDec backend=%s JD_FORMAT=%d JD_FASTDECODE=%d workbuf=%u bytes mirror_x=%d",
            IMAGE_DECODE_TJPGD_ROM ? "ROM" : "external",
            (int)JD_FORMAT,
            (int)JD_FASTDECODE,
            (unsigned)IMAGE_DECODE_WORKBUF_SIZE,
            (int)IMAGE_DECODE_MIRROR_X);

        s_image_decode_logged_config = true;
    }

    return ESP_OK;
}

void image_decode_deinit(void)
{
    if (s_image_decode_workbuf != NULL)
    {
        free(s_image_decode_workbuf);
        s_image_decode_workbuf = NULL;
    }

    if (s_image_decode_mutex != NULL)
    {
        vSemaphoreDelete(s_image_decode_mutex);
        s_image_decode_mutex = NULL;
    }

    s_image_decode_logged_config = false;
}

esp_err_t image_decode_probe_jpeg(
    const uint8_t *jpeg_data,
    size_t jpeg_len,
    image_decode_jpeg_info_t *out_info)
{
    if ((jpeg_data == NULL) ||
        (jpeg_len < 4U) ||
        (out_info == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_info, 0, sizeof(*out_info));

    if ((jpeg_data[0] != 0xFFU) || (jpeg_data[1] != 0xD8U))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    size_t i = 2U;

    while ((i + 3U) < jpeg_len)
    {
        if (jpeg_data[i] != 0xFFU)
        {
            ++i;
            continue;
        }

        while ((i < jpeg_len) && (jpeg_data[i] == 0xFFU))
        {
            ++i;
        }

        if (i >= jpeg_len)
        {
            break;
        }

        const uint8_t marker = jpeg_data[i++];

        if ((marker == 0xD8U) ||
            (marker == 0xD9U) ||
            (marker == 0x01U) ||
            ((marker >= 0xD0U) && (marker <= 0xD7U)))
        {
            if (marker == 0xD9U)
            {
                break;
            }

            continue;
        }

        if (marker == 0xDAU)
        {
            /*
             * Start of scan. SOF should already have appeared.
             */
            break;
        }

        if ((i + 1U) >= jpeg_len)
        {
            break;
        }

        const uint16_t seg_len =
            ((uint16_t)jpeg_data[i] << 8) |
            (uint16_t)jpeg_data[i + 1U];

        if ((seg_len < 2U) || ((i + seg_len) > jpeg_len))
        {
            return ESP_ERR_INVALID_SIZE;
        }

        if (((marker == 0xC0U) || (marker == 0xC2U)) &&
            (seg_len >= 8U))
        {
            out_info->sof_marker = marker;
            out_info->progressive = (marker == 0xC2U);

            out_info->height =
                ((uint16_t)jpeg_data[i + 3U] << 8) |
                (uint16_t)jpeg_data[i + 4U];

            out_info->width =
                ((uint16_t)jpeg_data[i + 5U] << 8) |
                (uint16_t)jpeg_data[i + 6U];

            return ESP_OK;
        }

        i += seg_len;
    }

    /*
     * A valid-looking JPEG but no SOF0/SOF2 found in the lightweight scan.
     * Let jd_prepare() be the final authority.
     */
    return ESP_ERR_NOT_FOUND;
}

esp_err_t image_decode_jpeg_to_grayscale(
    const uint8_t *jpeg_data,
    size_t jpeg_len,
    uint8_t scale,
    grayscale_image_t *out_img)
{
    if ((jpeg_data == NULL) ||
        (jpeg_len < 4U) ||
        (out_img == NULL) ||
        (out_img->data == NULL) ||
        (scale > 3U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((jpeg_data[0] != 0xFFU) || (jpeg_data[1] != 0xD8U))
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "Invalid JPEG SOI: len=%u head=%02X %02X",
            (unsigned)jpeg_len,
            jpeg_data[0],
            jpeg_data[1]);

        return ESP_ERR_INVALID_RESPONSE;
    }

    if ((jpeg_len >= 2U) &&
        ((jpeg_data[jpeg_len - 2U] != 0xFFU) ||
         (jpeg_data[jpeg_len - 1U] != 0xD9U)))
    {
        ESP_LOGW(
            IMAGE_DECODE_TAG,
            "JPEG does not end with FF D9: len=%u tail=%02X %02X; attempting decode anyway",
            (unsigned)jpeg_len,
            jpeg_data[jpeg_len - 2U],
            jpeg_data[jpeg_len - 1U]);
    }

#if defined(JD_USE_SCALE) && (JD_USE_SCALE == 0)
    if (scale != 0U)
    {
        ESP_LOGE(IMAGE_DECODE_TAG, "TJpgDec was built with JD_USE_SCALE=0");
        return ESP_ERR_NOT_SUPPORTED;
    }
#endif

    image_decode_jpeg_info_t probe;
    esp_err_t probe_err =
        image_decode_probe_jpeg(jpeg_data, jpeg_len, &probe);

    if ((probe_err == ESP_OK) && probe.progressive)
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "Progressive JPEG (SOF2) is not supported by TJpgDec: %ux%u",
            (unsigned)probe.width,
            (unsigned)probe.height);

        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = image_decode_init();
    if (err != ESP_OK)
    {
        return err;
    }

    if (xSemaphoreTake(s_image_decode_mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    const uint16_t expected_width = out_img->width;
    const uint16_t expected_height = out_img->height;

    size_t capacity = out_img->capacity_bytes;

    if ((capacity == 0U) &&
        (expected_width != 0U) &&
        (expected_height != 0U))
    {
        capacity =
            (size_t)expected_width *
            (size_t)expected_height;
    }

    if (capacity == 0U)
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "Output capacity unknown. Set capacity_bytes or preset width+height.");

        xSemaphoreGive(s_image_decode_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    image_decode_io_t io =
    {
        .in_data = jpeg_data,
        .in_len = jpeg_len,
        .in_pos = 0U,
        .out_data = out_img->data,
        .out_width = 0U,
        .out_height = 0U,
        .output_overflow = false,
    };

    JDEC jd;
    memset(&jd, 0, sizeof(jd));

    JRESULT jr =
        jd_prepare(
            &jd,
            image_decode_input_cb,
            s_image_decode_workbuf,
            (UINT)IMAGE_DECODE_WORKBUF_SIZE,
            &io);

    if (jr != JDR_OK)
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "jd_prepare failed: %s(%d) len=%u consumed=%u/%u "
            "head=%02X %02X tail=%02X %02X",
            image_decode_jresult_name(jr),
            (int)jr,
            (unsigned)jpeg_len,
            (unsigned)io.in_pos,
            (unsigned)io.in_len,
            jpeg_data[0],
            jpeg_data[1],
            jpeg_data[jpeg_len - 2U],
            jpeg_data[jpeg_len - 1U]);

        xSemaphoreGive(s_image_decode_mutex);
        return ESP_FAIL;
    }

    const uint32_t divisor = 1UL << scale;
    const uint16_t decoded_width = (uint16_t)(jd.width / divisor);
    const uint16_t decoded_height = (uint16_t)(jd.height / divisor);

    if ((decoded_width == 0U) || (decoded_height == 0U))
    {
        xSemaphoreGive(s_image_decode_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t required =
        (size_t)decoded_width *
        (size_t)decoded_height;

    if (required > capacity)
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "Output buffer too small: JPEG=%ux%u scale=%u -> %ux%u needs=%u bytes, capacity=%u",
            (unsigned)jd.width,
            (unsigned)jd.height,
            (unsigned)scale,
            (unsigned)decoded_width,
            (unsigned)decoded_height,
            (unsigned)required,
            (unsigned)capacity);

        xSemaphoreGive(s_image_decode_mutex);
        return ESP_ERR_NO_MEM;
    }

    if (((expected_width != 0U) && (expected_width != decoded_width)) ||
        ((expected_height != 0U) && (expected_height != decoded_height)))
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "Unexpected decoded size: JPEG=%ux%u scale=%u -> %ux%u, caller expected=%ux%u",
            (unsigned)jd.width,
            (unsigned)jd.height,
            (unsigned)scale,
            (unsigned)decoded_width,
            (unsigned)decoded_height,
            (unsigned)expected_width,
            (unsigned)expected_height);

        xSemaphoreGive(s_image_decode_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    io.out_width = decoded_width;
    io.out_height = decoded_height;

    /*
     * Clear output so an interrupted decode cannot leave old pixels looking
     * like a successful frame.
     */
    memset(out_img->data, 0, required);

    jr = jd_decomp(
        &jd,
        image_decode_output_cb,
        scale);

    if (jr != JDR_OK)
    {
        ESP_LOGE(
            IMAGE_DECODE_TAG,
            "jd_decomp failed: %s(%d) JPEG=%ux%u scale=%u consumed=%u/%u overflow=%d",
            image_decode_jresult_name(jr),
            (int)jr,
            (unsigned)jd.width,
            (unsigned)jd.height,
            (unsigned)scale,
            (unsigned)io.in_pos,
            (unsigned)io.in_len,
            (int)io.output_overflow);

        xSemaphoreGive(s_image_decode_mutex);
        return ESP_FAIL;
    }

    out_img->width = decoded_width;
    out_img->height = decoded_height;

    xSemaphoreGive(s_image_decode_mutex);
    return ESP_OK;
}

#endif /* IMAGE_DECODE_IMPLEMENTATION */

#endif /* IMAGE_DECODE_H */
