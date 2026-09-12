#ifndef TRAJECTORY_DECODER_H
#define TRAJECTORY_DECODER_H

/*
 * trajectory_decoder.h
 *
 * V1 .traj file decoder for the trajectory system.
 *
 * Responsibilities:
 *   .traj file -> validated header -> sequential trajectory_segment_t stream
 *
 * This module deliberately does NOT:
 *   - advance LINE/CIRCLE geometry
 *   - read odometry
 *   - decide segment completion
 *   - generate vx/vy/w
 *   - command motors
 *
 * Those responsibilities belong to trajectory_executor / trajectory_tracker.
 *
 * Coordinate convention:
 *   +X          : forward / world +X
 *   +Y          : left    / world +Y
 *   angle > 0   : counter-clockwise
 *
 * Units:
 *   position / radius : mm
 *   linear speed      : mm/s
 *   acceleration      : mm/s^2
 *   angle / sweep     : degree
 *
 * --------------------------------------------------------------------------
 * V1 binary format
 * --------------------------------------------------------------------------
 *
 * All integer and float fields are LITTLE-ENDIAN.
 * float fields are IEEE-754 binary32.
 *
 * Header: 32 bytes
 *
 *   offset  size  field
 *      0      4   magic = "TRJ1"
 *      4      2   version = 1
 *      6      2   header_size = 32
 *      8      4   segment_count
 *     12      2   record_size = 44
 *     14      2   flags = 0 for V1
 *     16      4   start_x_mm
 *     20      4   start_y_mm
 *     24      4   start_yaw_deg
 *     28      4   reserved = 0
 *
 * Segment record: 44 bytes
 *
 *   offset  size  field
 *      0      1   type
 *      1      1   flags
 *      2      2   reserved
 *      4      4   speed_mm_s
 *      8      4   acceleration_mm_s2
 *     12     32   data[8]
 *
 * LINE data mapping:
 *
 *   data[0] = end_x_mm
 *   data[1] = end_y_mm
 *   data[2..7] = 0
 *
 * CIRCLE data mapping:
 *
 *   data[0] = center_x_mm
 *   data[1] = center_y_mm
 *   data[2] = radius_mm
 *   data[3] = start_angle_deg
 *   data[4] = sweep_deg
 *   data[5..7] = 0
 *
 * CIRCLE semantics:
 *
 *   x(theta) = center_x + radius * cos(theta)
 *   y(theta) = center_y + radius * sin(theta)
 *
 *   theta_start = start_angle_deg
 *   theta_end   = start_angle_deg + sweep_deg
 *
 *   sweep_deg > 0 : CCW
 *   sweep_deg < 0 : CW
 *
 * Example:
 *
 *   type        = CIRCLE
 *   center      = (500, 250)
 *   radius      = 250
 *   start_angle = -90
 *   sweep       = 180
 *   speed       = 220
 *
 * This starts at (500, 0), travels CCW through the right side of the circle,
 * and ends at (500, 500).
 *
 * --------------------------------------------------------------------------
 * Python struct formats for the PC generator
 * --------------------------------------------------------------------------
 *
 *   HEADER_FMT = "<4sHHIHHfffI"    # 32 bytes
 *   RECORD_FMT = "<BBHff8f"        # 44 bytes
 *
 * --------------------------------------------------------------------------
 * Single-header usage
 * --------------------------------------------------------------------------
 *
 * In exactly ONE .c file:
 *
 *   #define TRAJECTORY_DECODER_IMPLEMENTATION
 *   #include "trajectory_decoder.h"
 *
 * In all other .c files:
 *
 *   #include "trajectory_decoder.h"
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================
 * 1. Format constants
 * ============================================================ */

#define TRAJECTORY_FILE_VERSION_V1          1U
#define TRAJECTORY_FILE_HEADER_SIZE_V1      32U
#define TRAJECTORY_FILE_RECORD_SIZE_V1      44U

#define TRAJECTORY_FILE_MAGIC_0             ((uint8_t)'T')
#define TRAJECTORY_FILE_MAGIC_1             ((uint8_t)'R')
#define TRAJECTORY_FILE_MAGIC_2             ((uint8_t)'J')
#define TRAJECTORY_FILE_MAGIC_3             ((uint8_t)'1')

#define TRAJECTORY_SEGMENT_DATA_FLOATS      8U


/* ============================================================
 * 2. Segment types
 * ============================================================ */

typedef enum
{
    TRAJECTORY_SEGMENT_NONE   = 0,
    TRAJECTORY_SEGMENT_LINE   = 1,
    TRAJECTORY_SEGMENT_CIRCLE = 2

} trajectory_segment_type_t;


/* ============================================================
 * 3. Decoded public data types
 * ============================================================ */

typedef struct
{
    uint16_t version;
    uint16_t header_size;

    uint32_t segment_count;

    uint16_t record_size;
    uint16_t flags;

    float start_x_mm;
    float start_y_mm;
    float start_yaw_deg;

} trajectory_file_header_t;


typedef struct
{
    float end_x_mm;
    float end_y_mm;

} trajectory_line_t;


typedef struct
{
    float center_x_mm;
    float center_y_mm;

    float radius_mm;

    /*
     * World-frame polar angle.
     *
     * 0 deg   : +X
     * 90 deg  : +Y
     * positive: CCW
     */
    float start_angle_deg;

    /*
     * Signed angular travel.
     *
     * > 0 : CCW
     * < 0 : CW
     *
     * Examples:
     *   +180 : CCW half-circle
     *   -90  : CW quarter-circle
     *   +360 : one full CCW circle
     */
    float sweep_deg;

} trajectory_circle_t;


typedef struct
{
    trajectory_segment_type_t type;

    uint8_t flags;

    float speed_mm_s;
    float acceleration_mm_s2;

    union
    {
        trajectory_line_t line;
        trajectory_circle_t circle;

    } geometry;

} trajectory_segment_t;


/* ============================================================
 * 4. Decoder state
 * ============================================================ */

typedef struct
{
    FILE *file;

    trajectory_file_header_t header;

    uint32_t next_segment_index;

    long segment_data_offset;

    bool opened;

} trajectory_decoder_t;


#define TRAJECTORY_DECODER_INITIALIZER \
    {                                   \
        .file = NULL,                   \
        .header = {0},                  \
        .next_segment_index = 0U,       \
        .segment_data_offset = 0L,      \
        .opened = false                 \
    }


/* ============================================================
 * 5. Public API
 * ============================================================ */

/**
 * @brief Open and validate a V1 .traj file.
 *
 * @param decoder Decoder instance.
 * @param path    Path readable by stdio/fopen(), e.g. "/sdcard/test.traj".
 *
 * @return ESP_OK on success.
 */
esp_err_t trajectory_decoder_open(
    trajectory_decoder_t *decoder,
    const char *path);


/**
 * @brief Close the file and reset decoder state.
 */
void trajectory_decoder_close(
    trajectory_decoder_t *decoder);


/**
 * @brief Rewind to the first segment.
 */
esp_err_t trajectory_decoder_rewind(
    trajectory_decoder_t *decoder);


/**
 * @brief Whether another segment is available.
 */
bool trajectory_decoder_has_next(
    const trajectory_decoder_t *decoder);


/**
 * @brief Read and decode the next segment.
 *
 * On success, decoder->next_segment_index advances by one.
 */
esp_err_t trajectory_decoder_read_next(
    trajectory_decoder_t *decoder,
    trajectory_segment_t *segment);


/**
 * @brief Read the already-decoded file header.
 */
esp_err_t trajectory_decoder_get_header(
    const trajectory_decoder_t *decoder,
    trajectory_file_header_t *header);


/**
 * @brief Index of the segment that will be returned by read_next().
 */
uint32_t trajectory_decoder_get_next_index(
    const trajectory_decoder_t *decoder);


/**
 * @brief Human-readable segment type for logging/debug.
 */
const char *trajectory_decoder_segment_type_name(
    trajectory_segment_type_t type);


#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */

#ifdef TRAJECTORY_DECODER_IMPLEMENTATION

#include <string.h>
#include <math.h>
#include <limits.h>

#include "esp_log.h"


#ifdef __cplusplus
extern "C" {
#endif


#define TRAJECTORY_DECODER_TAG "traj_decoder"


static uint16_t trajectory_decoder_u16_le(
    const uint8_t *p)
{
    return
        (uint16_t)p[0] |
        ((uint16_t)p[1] << 8);
}


static uint32_t trajectory_decoder_u32_le(
    const uint8_t *p)
{
    return
        (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}


static float trajectory_decoder_f32_le(
    const uint8_t *p)
{
    const uint32_t u =
        trajectory_decoder_u32_le(p);

    float value = 0.0f;

    /*
     * ESP32 uses IEEE-754 32-bit float. memcpy avoids strict-aliasing issues.
     */
    memcpy(
        &value,
        &u,
        sizeof(value));

    return value;
}


static bool trajectory_decoder_valid_motion_common(
    float speed_mm_s,
    float acceleration_mm_s2)
{
    if (!isfinite(speed_mm_s) ||
        !isfinite(acceleration_mm_s2))
    {
        return false;
    }

    if (speed_mm_s <= 0.0f)
    {
        return false;
    }

    /*
     * 0 means "no acceleration constraint specified in the file".
     */
    if (acceleration_mm_s2 < 0.0f)
    {
        return false;
    }

    return true;
}


static esp_err_t trajectory_decoder_decode_header(
    const uint8_t raw[TRAJECTORY_FILE_HEADER_SIZE_V1],
    trajectory_file_header_t *header)
{
    if ((raw == NULL) || (header == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((raw[0] != TRAJECTORY_FILE_MAGIC_0) ||
        (raw[1] != TRAJECTORY_FILE_MAGIC_1) ||
        (raw[2] != TRAJECTORY_FILE_MAGIC_2) ||
        (raw[3] != TRAJECTORY_FILE_MAGIC_3))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Invalid trajectory magic");

        return ESP_ERR_INVALID_RESPONSE;
    }

    header->version =
        trajectory_decoder_u16_le(&raw[4]);

    header->header_size =
        trajectory_decoder_u16_le(&raw[6]);

    header->segment_count =
        trajectory_decoder_u32_le(&raw[8]);

    header->record_size =
        trajectory_decoder_u16_le(&raw[12]);

    header->flags =
        trajectory_decoder_u16_le(&raw[14]);

    header->start_x_mm =
        trajectory_decoder_f32_le(&raw[16]);

    header->start_y_mm =
        trajectory_decoder_f32_le(&raw[20]);

    header->start_yaw_deg =
        trajectory_decoder_f32_le(&raw[24]);

    if (header->version != TRAJECTORY_FILE_VERSION_V1)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Unsupported trajectory version: %u",
            (unsigned)header->version);

        return ESP_ERR_NOT_SUPPORTED;
    }

    if (header->header_size < TRAJECTORY_FILE_HEADER_SIZE_V1)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Header too small: %u",
            (unsigned)header->header_size);

        return ESP_ERR_INVALID_SIZE;
    }

    if (header->record_size < TRAJECTORY_FILE_RECORD_SIZE_V1)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Segment record too small: %u",
            (unsigned)header->record_size);

        return ESP_ERR_INVALID_SIZE;
    }

    if (!isfinite(header->start_x_mm) ||
        !isfinite(header->start_y_mm) ||
        !isfinite(header->start_yaw_deg))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Non-finite start pose in header");

        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


static esp_err_t trajectory_decoder_decode_record(
    const uint8_t raw[TRAJECTORY_FILE_RECORD_SIZE_V1],
    trajectory_segment_t *segment)
{
    if ((raw == NULL) || (segment == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(
        segment,
        0,
        sizeof(*segment));

    segment->type =
        (trajectory_segment_type_t)raw[0];

    segment->flags =
        raw[1];

    segment->speed_mm_s =
        trajectory_decoder_f32_le(&raw[4]);

    segment->acceleration_mm_s2 =
        trajectory_decoder_f32_le(&raw[8]);

    if (!trajectory_decoder_valid_motion_common(
            segment->speed_mm_s,
            segment->acceleration_mm_s2))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Invalid speed/acceleration in segment");

        return ESP_ERR_INVALID_RESPONSE;
    }

    switch (segment->type)
    {
        case TRAJECTORY_SEGMENT_LINE:
        {
            segment->geometry.line.end_x_mm =
                trajectory_decoder_f32_le(&raw[12]);

            segment->geometry.line.end_y_mm =
                trajectory_decoder_f32_le(&raw[16]);

            if (!isfinite(segment->geometry.line.end_x_mm) ||
                !isfinite(segment->geometry.line.end_y_mm))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid LINE geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_SEGMENT_CIRCLE:
        {
            trajectory_circle_t *circle =
                &segment->geometry.circle;

            circle->center_x_mm =
                trajectory_decoder_f32_le(&raw[12]);

            circle->center_y_mm =
                trajectory_decoder_f32_le(&raw[16]);

            circle->radius_mm =
                trajectory_decoder_f32_le(&raw[20]);

            circle->start_angle_deg =
                trajectory_decoder_f32_le(&raw[24]);

            circle->sweep_deg =
                trajectory_decoder_f32_le(&raw[28]);

            if (!isfinite(circle->center_x_mm) ||
                !isfinite(circle->center_y_mm) ||
                !isfinite(circle->radius_mm) ||
                !isfinite(circle->start_angle_deg) ||
                !isfinite(circle->sweep_deg))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid CIRCLE geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (circle->radius_mm <= 0.0f)
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "CIRCLE radius must be > 0");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (fabsf(circle->sweep_deg) < 1.0e-6f)
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "CIRCLE sweep must be non-zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        default:
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "Unsupported segment type: %u",
                (unsigned)segment->type);

            return ESP_ERR_NOT_SUPPORTED;
        }
    }

    return ESP_OK;
}


esp_err_t trajectory_decoder_open(
    trajectory_decoder_t *decoder,
    const char *path)
{
    if ((decoder == NULL) || (path == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (decoder->opened)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(
        decoder,
        0,
        sizeof(*decoder));

    FILE *file =
        fopen(
            path,
            "rb");

    if (file == NULL)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Failed to open: %s",
            path);

        return ESP_FAIL;
    }

    uint8_t raw_header[TRAJECTORY_FILE_HEADER_SIZE_V1];

    const size_t header_read =
        fread(
            raw_header,
            1,
            sizeof(raw_header),
            file);

    if (header_read != sizeof(raw_header))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Trajectory header is truncated");

        fclose(file);

        return ESP_ERR_INVALID_SIZE;
    }

    trajectory_file_header_t header = {0};

    esp_err_t ret =
        trajectory_decoder_decode_header(
            raw_header,
            &header);

    if (ret != ESP_OK)
    {
        fclose(file);
        return ret;
    }

    /*
     * Validate that the declared segment block fits inside the file.
     * uint64_t arithmetic prevents multiplication overflow.
     */
    if (fseek(file, 0L, SEEK_END) != 0)
    {
        fclose(file);
        return ESP_FAIL;
    }

    const long file_size_long =
        ftell(file);

    if (file_size_long < 0L)
    {
        fclose(file);
        return ESP_FAIL;
    }

    const uint64_t expected_min_size =
        (uint64_t)header.header_size +
        (uint64_t)header.segment_count *
        (uint64_t)header.record_size;

    if ((uint64_t)file_size_long < expected_min_size)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Trajectory file truncated: size=%ld expected>=%llu",
            file_size_long,
            (unsigned long long)expected_min_size);

        fclose(file);

        return ESP_ERR_INVALID_SIZE;
    }

    if (fseek(
            file,
            (long)header.header_size,
            SEEK_SET) != 0)
    {
        fclose(file);
        return ESP_FAIL;
    }

    decoder->file =
        file;

    decoder->header =
        header;

    decoder->next_segment_index =
        0U;

    decoder->segment_data_offset =
        (long)header.header_size;

    decoder->opened =
        true;

    ESP_LOGI(
        TRAJECTORY_DECODER_TAG,
        "Opened %s: version=%u segments=%lu record_size=%u",
        path,
        (unsigned)header.version,
        (unsigned long)header.segment_count,
        (unsigned)header.record_size);

    return ESP_OK;
}


void trajectory_decoder_close(
    trajectory_decoder_t *decoder)
{
    if (decoder == NULL)
    {
        return;
    }

    if (decoder->file != NULL)
    {
        fclose(decoder->file);
    }

    memset(
        decoder,
        0,
        sizeof(*decoder));
}


esp_err_t trajectory_decoder_rewind(
    trajectory_decoder_t *decoder)
{
    if (decoder == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!decoder->opened ||
        (decoder->file == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (fseek(
            decoder->file,
            decoder->segment_data_offset,
            SEEK_SET) != 0)
    {
        return ESP_FAIL;
    }

    decoder->next_segment_index =
        0U;

    return ESP_OK;
}


bool trajectory_decoder_has_next(
    const trajectory_decoder_t *decoder)
{
    if ((decoder == NULL) ||
        !decoder->opened ||
        (decoder->file == NULL))
    {
        return false;
    }

    return
        decoder->next_segment_index <
        decoder->header.segment_count;
}


esp_err_t trajectory_decoder_read_next(
    trajectory_decoder_t *decoder,
    trajectory_segment_t *segment)
{
    if ((decoder == NULL) || (segment == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!decoder->opened ||
        (decoder->file == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!trajectory_decoder_has_next(decoder))
    {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t raw_record[TRAJECTORY_FILE_RECORD_SIZE_V1];

    const size_t record_read =
        fread(
            raw_record,
            1,
            sizeof(raw_record),
            decoder->file);

    if (record_read != sizeof(raw_record))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Segment %lu is truncated",
            (unsigned long)decoder->next_segment_index);

        return ESP_ERR_INVALID_SIZE;
    }

    /*
     * V1 understands the first 44 bytes. A future file may declare a larger
     * record_size while keeping the V1 prefix compatible.
     */
    const uint16_t extra_bytes =
        (uint16_t)(
            decoder->header.record_size -
            TRAJECTORY_FILE_RECORD_SIZE_V1
        );

    if (extra_bytes > 0U)
    {
        if (fseek(
                decoder->file,
                (long)extra_bytes,
                SEEK_CUR) != 0)
        {
            return ESP_FAIL;
        }
    }

    esp_err_t ret =
        trajectory_decoder_decode_record(
            raw_record,
            segment);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Failed to decode segment %lu",
            (unsigned long)decoder->next_segment_index);

        return ret;
    }

    decoder->next_segment_index++;

    return ESP_OK;
}


esp_err_t trajectory_decoder_get_header(
    const trajectory_decoder_t *decoder,
    trajectory_file_header_t *header)
{
    if ((decoder == NULL) || (header == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!decoder->opened)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *header =
        decoder->header;

    return ESP_OK;
}


uint32_t trajectory_decoder_get_next_index(
    const trajectory_decoder_t *decoder)
{
    if ((decoder == NULL) || !decoder->opened)
    {
        return 0U;
    }

    return decoder->next_segment_index;
}


const char *trajectory_decoder_segment_type_name(
    trajectory_segment_type_t type)
{
    switch (type)
    {
        case TRAJECTORY_SEGMENT_LINE:
            return "LINE";

        case TRAJECTORY_SEGMENT_CIRCLE:
            return "CIRCLE";

        default:
            return "UNKNOWN";
    }
}


#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_DECODER_IMPLEMENTATION */

#endif /* TRAJECTORY_DECODER_H */
