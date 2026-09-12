#ifndef TRAJECTORY_DECODER_H
#define TRAJECTORY_DECODER_H

/*
 * trajectory_decoder.h
 *
 * V1 + V2 .traj file decoder for the trajectory system.
 *
 * Responsibilities:
 *   .traj file -> validated header -> sequential trajectory_record_t stream
 *
 * Legacy compatibility:
 *   - V1 TRJ1 files remain supported.
 *   - trajectory_segment_t and trajectory_decoder_read_next() are preserved
 *     for the existing V1 Executor/Runner path.
 *   - trajectory_decoder_read_next() is intentionally V1-only.
 *   - V2 must use trajectory_decoder_read_next_record().
 *
 * This module deliberately does NOT:
 *   - advance LINE/CIRCLE/CUBIC_BEZIER geometry
 *   - execute PEN_UP / PEN_DOWN / WAIT
 *   - read odometry
 *   - decide motion completion
 *   - generate vx/vy/w
 *   - command motors
 *
 * Those responsibilities belong to later executor / runner / event layers.
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
 *   WAIT duration     : second
 *
 * ==========================================================================
 * Common binary framing
 * ==========================================================================
 *
 * All integer and float fields are LITTLE-ENDIAN.
 * float fields are IEEE-754 binary32.
 *
 * Header prefix: 32 bytes
 *
 *   offset  size  field
 *      0      4   magic = "TRJ1" or "TRJ2"
 *      4      2   version = 1 or 2
 *      6      2   header_size >= 32
 *      8      4   count
 *     12      2   record_size >= 44
 *     14      2   flags
 *     16      4   start_x_mm
 *     20      4   start_y_mm
 *     24      4   start_yaw_deg
 *     28      4   reserved
 *
 * Record prefix: 44 bytes
 *
 *   offset  size  field
 *      0      1   type
 *      1      1   flags
 *      2      2   reserved
 *      4      4   speed_mm_s
 *      8      4   acceleration_mm_s2
 *     12     32   data[8]
 *
 * ==========================================================================
 * V1 / TRJ1
 * ==========================================================================
 *
 * Header offset 8 is segment_count.
 *
 * Types:
 *   1 LINE
 *   2 CIRCLE
 *
 * V1 preserves the previous decoder's forward-compatible behavior:
 *   header_size >= 32
 *   record_size >= 44
 *
 * LINE:
 *   data[0] = end_x_mm
 *   data[1] = end_y_mm
 *
 * CIRCLE:
 *   data[0] = center_x_mm
 *   data[1] = center_y_mm
 *   data[2] = radius_mm
 *   data[3] = start_angle_deg
 *   data[4] = signed sweep_deg
 *
 * ==========================================================================
 * V2 / TRJ2
 * ==========================================================================
 *
 * Header offset 8 is record_count because Event records are now part of the
 * stream.
 *
 * Types:
 *   0x01 LINE
 *   0x02 CIRCLE
 *   0x03 CUBIC_BEZIER
 *   0x20 PEN_UP
 *   0x21 PEN_DOWN
 *   0x22 WAIT
 *
 * LINE:
 *   data[0] = end_x_mm
 *   data[1] = end_y_mm
 *   data[2..7] = 0
 *
 * CIRCLE:
 *   data[0] = center_x_mm
 *   data[1] = center_y_mm
 *   data[2] = radius_mm
 *   data[3] = start_angle_deg
 *   data[4] = signed sweep_deg
 *   data[5..7] = 0
 *
 * CUBIC_BEZIER:
 *   P0 = implicit current logical point
 *   data[0] = control1_x_mm
 *   data[1] = control1_y_mm
 *   data[2] = control2_x_mm
 *   data[3] = control2_y_mm
 *   data[4] = end_x_mm
 *   data[5] = end_y_mm
 *   data[6..7] = 0
 *
 * PEN_UP / PEN_DOWN:
 *   speed = 0
 *   acceleration = 0
 *   data[0..7] = 0
 *
 * WAIT:
 *   speed = 0
 *   acceleration = 0
 *   data[0] = duration_s > 0
 *   data[1..7] = 0
 *
 * V2 canonical prefix validation:
 *   header flags/reserved = 0
 *   record flags/reserved = 0
 *   unused data[] = 0
 *
 * Decoder responsibility note:
 *   CIRCLE continuity, LINE zero-length checks, and CUBIC_BEZIER P0-dependent
 *   degeneracy require the logical current point and therefore remain outside
 *   this byte decoder, exactly as V1 geometric continuity remained outside
 *   the old decoder.
 *
 * ==========================================================================
 * Single-header usage
 * ==========================================================================
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
#define TRAJECTORY_FILE_VERSION_V2          2U

#define TRAJECTORY_FILE_HEADER_SIZE_BASE    32U
#define TRAJECTORY_FILE_RECORD_SIZE_BASE    44U

/* Legacy V1 names preserved for source compatibility. */
#define TRAJECTORY_FILE_HEADER_SIZE_V1      TRAJECTORY_FILE_HEADER_SIZE_BASE
#define TRAJECTORY_FILE_RECORD_SIZE_V1      TRAJECTORY_FILE_RECORD_SIZE_BASE

#define TRAJECTORY_FILE_HEADER_SIZE_V2      TRAJECTORY_FILE_HEADER_SIZE_BASE
#define TRAJECTORY_FILE_RECORD_SIZE_V2      TRAJECTORY_FILE_RECORD_SIZE_BASE

#define TRAJECTORY_FILE_MAGIC_0             ((uint8_t)'T')
#define TRAJECTORY_FILE_MAGIC_1             ((uint8_t)'R')
#define TRAJECTORY_FILE_MAGIC_2             ((uint8_t)'J')

/* Legacy V1 alias preserved. */
#define TRAJECTORY_FILE_MAGIC_3             ((uint8_t)'1')

#define TRAJECTORY_FILE_MAGIC_V1_3          ((uint8_t)'1')
#define TRAJECTORY_FILE_MAGIC_V2_3          ((uint8_t)'2')

#define TRAJECTORY_RECORD_DATA_FLOATS       8U
#define TRAJECTORY_SEGMENT_DATA_FLOATS      TRAJECTORY_RECORD_DATA_FLOATS

#define TRAJECTORY_V2_SWEEP_EPSILON_DEG     1.0e-6f


/* ============================================================
 * 2. Legacy V1 segment types
 * ============================================================ */

typedef enum
{
    TRAJECTORY_SEGMENT_NONE   = 0,
    TRAJECTORY_SEGMENT_LINE   = 1,
    TRAJECTORY_SEGMENT_CIRCLE = 2

} trajectory_segment_type_t;


/* ============================================================
 * 3. Generalized V1/V2 record types
 * ============================================================ */

typedef enum
{
    TRAJECTORY_RECORD_NONE         = 0x00,

    TRAJECTORY_RECORD_LINE         = 0x01,
    TRAJECTORY_RECORD_CIRCLE       = 0x02,
    TRAJECTORY_RECORD_CUBIC_BEZIER = 0x03,

    TRAJECTORY_RECORD_PEN_UP       = 0x20,
    TRAJECTORY_RECORD_PEN_DOWN     = 0x21,
    TRAJECTORY_RECORD_WAIT         = 0x22

} trajectory_record_type_t;


/* ============================================================
 * 4. Decoded public data types
 * ============================================================ */

typedef struct
{
    uint16_t version;
    uint16_t header_size;

    /*
     * Generalized count.
     *
     * V1: equals segment_count.
     * V2: counts both Motion and Event records.
     */
    uint32_t record_count;

    /*
     * Legacy compatibility field.
     *
     * V1: equals record_count.
     * V2: always 0 because V2 count includes non-segment Events.
     */
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
     */
    float sweep_deg;

} trajectory_circle_t;


typedef struct
{
    /*
     * P0 is implicit current logical point.
     */
    float control1_x_mm;
    float control1_y_mm;

    float control2_x_mm;
    float control2_y_mm;

    float end_x_mm;
    float end_y_mm;

} trajectory_cubic_bezier_t;


typedef struct
{
    float duration_s;

} trajectory_wait_t;


/*
 * Existing V1 public type is preserved unchanged for Executor/Runner source
 * compatibility.
 */
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


/*
 * Generalized sequential decoder output.
 */
typedef struct
{
    trajectory_record_type_t type;

    uint8_t flags;

    float speed_mm_s;
    float acceleration_mm_s2;

    union
    {
        trajectory_line_t line;
        trajectory_circle_t circle;
        trajectory_cubic_bezier_t cubic_bezier;
        trajectory_wait_t wait;

    } payload;

} trajectory_record_t;


/* ============================================================
 * 5. Decoder state
 * ============================================================ */

typedef struct
{
    FILE *file;

    trajectory_file_header_t header;

    /*
     * Legacy field name retained for source compatibility.
     * It now means "index of the next record" for both V1 and V2.
     */
    uint32_t next_segment_index;

    /*
     * Legacy field name retained for source compatibility.
     * It now points to the first record for both V1 and V2.
     */
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
 * 6. Public API
 * ============================================================ */

/**
 * @brief Open and validate a V1/TRJ1 or V2/TRJ2 .traj file.
 *
 * This validates the 32-byte prefix and the declared record block size.
 * Individual records are validated when read.
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
 * @brief Rewind to the first record.
 */
esp_err_t trajectory_decoder_rewind(
    trajectory_decoder_t *decoder);


/**
 * @brief Whether another record is available.
 *
 * Legacy name retained. In V2 this counts Events as records.
 */
bool trajectory_decoder_has_next(
    const trajectory_decoder_t *decoder);


/**
 * @brief Read and decode the next generalized V1/V2 record.
 *
 * V1 returns LINE/CIRCLE records.
 * V2 returns LINE/CIRCLE/CUBIC_BEZIER/PEN_UP/PEN_DOWN/WAIT.
 */
esp_err_t trajectory_decoder_read_next_record(
    trajectory_decoder_t *decoder,
    trajectory_record_t *record);


/**
 * @brief Legacy V1-only segment API.
 *
 * Existing V1 Executor/Runner code can continue using this function.
 *
 * For V2 this returns ESP_ERR_NOT_SUPPORTED WITHOUT advancing the decoder.
 * This is intentional: old motion-only code must never accidentally consume
 * or execute V2 Event records.
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
 * @brief Index of the record that will be returned next.
 *
 * Legacy name preserved.
 */
uint32_t trajectory_decoder_get_next_index(
    const trajectory_decoder_t *decoder);


/**
 * @brief Generalized alias for trajectory_decoder_get_next_index().
 */
uint32_t trajectory_decoder_get_next_record_index(
    const trajectory_decoder_t *decoder);


/**
 * @brief Human-readable generalized record type for logging/debug.
 */
const char *trajectory_decoder_record_type_name(
    trajectory_record_type_t type);


/**
 * @brief Human-readable legacy segment type for logging/debug.
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


static bool trajectory_decoder_valid_event_common(
    float speed_mm_s,
    float acceleration_mm_s2)
{
    if (!isfinite(speed_mm_s) ||
        !isfinite(acceleration_mm_s2))
    {
        return false;
    }

    return
        (speed_mm_s == 0.0f) &&
        (acceleration_mm_s2 == 0.0f);
}


static bool trajectory_decoder_raw_data_is_zero(
    const uint8_t raw[TRAJECTORY_FILE_RECORD_SIZE_BASE],
    uint32_t first_data_index)
{
    if ((raw == NULL) ||
        (first_data_index >= TRAJECTORY_RECORD_DATA_FLOATS))
    {
        return false;
    }

    for (uint32_t i = first_data_index;
         i < TRAJECTORY_RECORD_DATA_FLOATS;
         ++i)
    {
        const float value =
            trajectory_decoder_f32_le(
                &raw[12U + (4U * i)]);

        if (!isfinite(value) ||
            (value != 0.0f))
        {
            return false;
        }
    }

    return true;
}


static esp_err_t trajectory_decoder_decode_header(
    const uint8_t raw[TRAJECTORY_FILE_HEADER_SIZE_BASE],
    trajectory_file_header_t *header)
{
    if ((raw == NULL) || (header == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((raw[0] != TRAJECTORY_FILE_MAGIC_0) ||
        (raw[1] != TRAJECTORY_FILE_MAGIC_1) ||
        (raw[2] != TRAJECTORY_FILE_MAGIC_2))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Invalid trajectory magic prefix");

        return ESP_ERR_INVALID_RESPONSE;
    }

    const bool magic_v1 =
        raw[3] == TRAJECTORY_FILE_MAGIC_V1_3;

    const bool magic_v2 =
        raw[3] == TRAJECTORY_FILE_MAGIC_V2_3;

    if (!magic_v1 && !magic_v2)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Unsupported trajectory magic TRJ%c",
            (char)raw[3]);

        return ESP_ERR_NOT_SUPPORTED;
    }

    memset(
        header,
        0,
        sizeof(*header));

    header->version =
        trajectory_decoder_u16_le(&raw[4]);

    header->header_size =
        trajectory_decoder_u16_le(&raw[6]);

    header->record_count =
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

    const uint32_t reserved =
        trajectory_decoder_u32_le(&raw[28]);

    if (magic_v1)
    {
        if (header->version != TRAJECTORY_FILE_VERSION_V1)
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "TRJ1 magic/version mismatch: version=%u",
                (unsigned)header->version);

            return ESP_ERR_INVALID_RESPONSE;
        }

        /*
         * Preserve legacy V1 field semantics.
         */
        header->segment_count =
            header->record_count;
    }
    else
    {
        if (header->version != TRAJECTORY_FILE_VERSION_V2)
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "TRJ2 magic/version mismatch: version=%u",
                (unsigned)header->version);

            return ESP_ERR_INVALID_RESPONSE;
        }

        /*
         * V2 count includes Events, so exposing it as "segment_count" to an
         * old Executor would be dangerous. Keep legacy field at zero.
         */
        header->segment_count =
            0U;

        if (header->flags != 0U)
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "TRJ2 header flags must be zero");

            return ESP_ERR_INVALID_RESPONSE;
        }

        if (reserved != 0U)
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "TRJ2 header reserved must be zero");

            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    if (header->header_size < TRAJECTORY_FILE_HEADER_SIZE_BASE)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Header too small: %u",
            (unsigned)header->header_size);

        return ESP_ERR_INVALID_SIZE;
    }

    if (header->record_size < TRAJECTORY_FILE_RECORD_SIZE_BASE)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Record too small: %u",
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


static esp_err_t trajectory_decoder_decode_record_v1(
    const uint8_t raw[TRAJECTORY_FILE_RECORD_SIZE_BASE],
    trajectory_record_t *record)
{
    if ((raw == NULL) || (record == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(
        record,
        0,
        sizeof(*record));

    record->type =
        (trajectory_record_type_t)raw[0];

    record->flags =
        raw[1];

    record->speed_mm_s =
        trajectory_decoder_f32_le(&raw[4]);

    record->acceleration_mm_s2 =
        trajectory_decoder_f32_le(&raw[8]);

    if (!trajectory_decoder_valid_motion_common(
            record->speed_mm_s,
            record->acceleration_mm_s2))
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Invalid V1 speed/acceleration");

        return ESP_ERR_INVALID_RESPONSE;
    }

    switch (record->type)
    {
        case TRAJECTORY_RECORD_LINE:
        {
            trajectory_line_t *line =
                &record->payload.line;

            line->end_x_mm =
                trajectory_decoder_f32_le(&raw[12]);

            line->end_y_mm =
                trajectory_decoder_f32_le(&raw[16]);

            if (!isfinite(line->end_x_mm) ||
                !isfinite(line->end_y_mm))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid V1 LINE geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_RECORD_CIRCLE:
        {
            trajectory_circle_t *circle =
                &record->payload.circle;

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
                    "Invalid V1 CIRCLE geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (circle->radius_mm <= 0.0f)
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "V1 CIRCLE radius must be > 0");

                return ESP_ERR_INVALID_RESPONSE;
            }

            /*
             * Preserve previous V1 behavior exactly: strictly less than 1e-6.
             */
            if (fabsf(circle->sweep_deg) < 1.0e-6f)
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "V1 CIRCLE sweep must be non-zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        default:
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "Unsupported V1 segment type: %u",
                (unsigned)record->type);

            return ESP_ERR_NOT_SUPPORTED;
        }
    }

    return ESP_OK;
}


static esp_err_t trajectory_decoder_decode_record_v2(
    const uint8_t raw[TRAJECTORY_FILE_RECORD_SIZE_BASE],
    trajectory_record_t *record)
{
    if ((raw == NULL) || (record == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(
        record,
        0,
        sizeof(*record));

    record->type =
        (trajectory_record_type_t)raw[0];

    record->flags =
        raw[1];

    const uint16_t reserved =
        trajectory_decoder_u16_le(&raw[2]);

    record->speed_mm_s =
        trajectory_decoder_f32_le(&raw[4]);

    record->acceleration_mm_s2 =
        trajectory_decoder_f32_le(&raw[8]);

    if (record->flags != 0U)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "TRJ2 record flags must be zero");

        return ESP_ERR_INVALID_RESPONSE;
    }

    if (reserved != 0U)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "TRJ2 record reserved must be zero");

        return ESP_ERR_INVALID_RESPONSE;
    }

    switch (record->type)
    {
        case TRAJECTORY_RECORD_LINE:
        {
            if (!trajectory_decoder_valid_motion_common(
                    record->speed_mm_s,
                    record->acceleration_mm_s2))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid TRJ2 LINE speed/acceleration");

                return ESP_ERR_INVALID_RESPONSE;
            }

            trajectory_line_t *line =
                &record->payload.line;

            line->end_x_mm =
                trajectory_decoder_f32_le(&raw[12]);

            line->end_y_mm =
                trajectory_decoder_f32_le(&raw[16]);

            if (!isfinite(line->end_x_mm) ||
                !isfinite(line->end_y_mm))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid TRJ2 LINE geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (!trajectory_decoder_raw_data_is_zero(
                    raw,
                    2U))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 LINE unused payload must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_RECORD_CIRCLE:
        {
            if (!trajectory_decoder_valid_motion_common(
                    record->speed_mm_s,
                    record->acceleration_mm_s2))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid TRJ2 CIRCLE speed/acceleration");

                return ESP_ERR_INVALID_RESPONSE;
            }

            trajectory_circle_t *circle =
                &record->payload.circle;

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
                    "Invalid TRJ2 CIRCLE geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (circle->radius_mm <= 0.0f)
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 CIRCLE radius must be > 0");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (fabsf(circle->sweep_deg) <=
                TRAJECTORY_V2_SWEEP_EPSILON_DEG)
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 CIRCLE sweep is too small");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (!trajectory_decoder_raw_data_is_zero(
                    raw,
                    5U))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 CIRCLE unused payload must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_RECORD_CUBIC_BEZIER:
        {
            if (!trajectory_decoder_valid_motion_common(
                    record->speed_mm_s,
                    record->acceleration_mm_s2))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid TRJ2 CUBIC_BEZIER speed/acceleration");

                return ESP_ERR_INVALID_RESPONSE;
            }

            trajectory_cubic_bezier_t *bezier =
                &record->payload.cubic_bezier;

            bezier->control1_x_mm =
                trajectory_decoder_f32_le(&raw[12]);

            bezier->control1_y_mm =
                trajectory_decoder_f32_le(&raw[16]);

            bezier->control2_x_mm =
                trajectory_decoder_f32_le(&raw[20]);

            bezier->control2_y_mm =
                trajectory_decoder_f32_le(&raw[24]);

            bezier->end_x_mm =
                trajectory_decoder_f32_le(&raw[28]);

            bezier->end_y_mm =
                trajectory_decoder_f32_le(&raw[32]);

            if (!isfinite(bezier->control1_x_mm) ||
                !isfinite(bezier->control1_y_mm) ||
                !isfinite(bezier->control2_x_mm) ||
                !isfinite(bezier->control2_y_mm) ||
                !isfinite(bezier->end_x_mm) ||
                !isfinite(bezier->end_y_mm))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "Invalid TRJ2 CUBIC_BEZIER geometry");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (!trajectory_decoder_raw_data_is_zero(
                    raw,
                    6U))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 CUBIC_BEZIER unused payload must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_RECORD_PEN_UP:
        case TRAJECTORY_RECORD_PEN_DOWN:
        {
            if (!trajectory_decoder_valid_event_common(
                    record->speed_mm_s,
                    record->acceleration_mm_s2))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 PEN event speed/acceleration must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (!trajectory_decoder_raw_data_is_zero(
                    raw,
                    0U))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 PEN event payload must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_RECORD_WAIT:
        {
            if (!trajectory_decoder_valid_event_common(
                    record->speed_mm_s,
                    record->acceleration_mm_s2))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 WAIT speed/acceleration must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            record->payload.wait.duration_s =
                trajectory_decoder_f32_le(&raw[12]);

            if (!isfinite(record->payload.wait.duration_s) ||
                (record->payload.wait.duration_s <= 0.0f))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 WAIT duration must be finite and > 0");

                return ESP_ERR_INVALID_RESPONSE;
            }

            if (!trajectory_decoder_raw_data_is_zero(
                    raw,
                    1U))
            {
                ESP_LOGE(
                    TRAJECTORY_DECODER_TAG,
                    "TRJ2 WAIT unused payload must be zero");

                return ESP_ERR_INVALID_RESPONSE;
            }

            break;
        }

        case TRAJECTORY_RECORD_NONE:
        default:
        {
            ESP_LOGE(
                TRAJECTORY_DECODER_TAG,
                "Unsupported TRJ2 record type: 0x%02X",
                (unsigned)record->type);

            return ESP_ERR_NOT_SUPPORTED;
        }
    }

    return ESP_OK;
}


static esp_err_t trajectory_decoder_decode_record_any(
    uint16_t version,
    const uint8_t raw[TRAJECTORY_FILE_RECORD_SIZE_BASE],
    trajectory_record_t *record)
{
    if (version == TRAJECTORY_FILE_VERSION_V1)
    {
        return trajectory_decoder_decode_record_v1(
            raw,
            record);
    }

    if (version == TRAJECTORY_FILE_VERSION_V2)
    {
        return trajectory_decoder_decode_record_v2(
            raw,
            record);
    }

    return ESP_ERR_NOT_SUPPORTED;
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

    uint8_t raw_header[TRAJECTORY_FILE_HEADER_SIZE_BASE];

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

    /*
     * uint64_t arithmetic prevents multiplication overflow.
     */
    const uint64_t expected_min_size =
        (uint64_t)header.header_size +
        (uint64_t)header.record_count *
        (uint64_t)header.record_size;

    if (expected_min_size > (uint64_t)LONG_MAX)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Trajectory file offset exceeds stdio long range");

        fclose(file);

        return ESP_ERR_INVALID_SIZE;
    }

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
        "Opened %s: version=%u records=%lu record_size=%u",
        path,
        (unsigned)header.version,
        (unsigned long)header.record_count,
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
        decoder->header.record_count;
}


esp_err_t trajectory_decoder_read_next_record(
    trajectory_decoder_t *decoder,
    trajectory_record_t *record)
{
    if ((decoder == NULL) || (record == NULL))
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

    /*
     * Seek from the declared record block start on every call. This keeps a
     * failed decode retry deterministic and naturally skips record extensions.
     */
    const uint64_t offset_u64 =
        (uint64_t)decoder->header.header_size +
        (uint64_t)decoder->next_segment_index *
        (uint64_t)decoder->header.record_size;

    if (offset_u64 > (uint64_t)LONG_MAX)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if (fseek(
            decoder->file,
            (long)offset_u64,
            SEEK_SET) != 0)
    {
        return ESP_FAIL;
    }

    uint8_t raw_record[TRAJECTORY_FILE_RECORD_SIZE_BASE];

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
            "Record %lu is truncated",
            (unsigned long)decoder->next_segment_index);

        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret =
        trajectory_decoder_decode_record_any(
            decoder->header.version,
            raw_record,
            record);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TRAJECTORY_DECODER_TAG,
            "Failed to decode record %lu",
            (unsigned long)decoder->next_segment_index);

        return ret;
    }

    decoder->next_segment_index++;

    return ESP_OK;
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

    /*
     * Safety boundary:
     * old V1 motion-only callers must never consume TRJ2 Events.
     */
    if (decoder->header.version !=
        TRAJECTORY_FILE_VERSION_V1)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    trajectory_record_t record = {0};

    esp_err_t ret =
        trajectory_decoder_read_next_record(
            decoder,
            &record);

    if (ret != ESP_OK)
    {
        return ret;
    }

    memset(
        segment,
        0,
        sizeof(*segment));

    segment->flags =
        record.flags;

    segment->speed_mm_s =
        record.speed_mm_s;

    segment->acceleration_mm_s2 =
        record.acceleration_mm_s2;

    switch (record.type)
    {
        case TRAJECTORY_RECORD_LINE:
        {
            segment->type =
                TRAJECTORY_SEGMENT_LINE;

            segment->geometry.line =
                record.payload.line;

            break;
        }

        case TRAJECTORY_RECORD_CIRCLE:
        {
            segment->type =
                TRAJECTORY_SEGMENT_CIRCLE;

            segment->geometry.circle =
                record.payload.circle;

            break;
        }

        default:
        {
            /*
             * Cannot occur for a valid V1 file, but keep the compatibility
             * boundary explicit.
             */
            return ESP_ERR_NOT_SUPPORTED;
        }
    }

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


uint32_t trajectory_decoder_get_next_record_index(
    const trajectory_decoder_t *decoder)
{
    return trajectory_decoder_get_next_index(
        decoder);
}


const char *trajectory_decoder_record_type_name(
    trajectory_record_type_t type)
{
    switch (type)
    {
        case TRAJECTORY_RECORD_LINE:
            return "LINE";

        case TRAJECTORY_RECORD_CIRCLE:
            return "CIRCLE";

        case TRAJECTORY_RECORD_CUBIC_BEZIER:
            return "CUBIC_BEZIER";

        case TRAJECTORY_RECORD_PEN_UP:
            return "PEN_UP";

        case TRAJECTORY_RECORD_PEN_DOWN:
            return "PEN_DOWN";

        case TRAJECTORY_RECORD_WAIT:
            return "WAIT";

        default:
            return "UNKNOWN";
    }
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
