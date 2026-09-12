#include "n3_service.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "n3_build_config.h"
#include "n3_hardware.h"
#include "n3_motion.h"
#include "n3_runner_bridge.h"
#include "trajectory_decoder.h"
#include "n3_transport.h"


#ifndef N3_JSON_LINE_MAX
#define N3_JSON_LINE_MAX 1024U
#endif

#ifndef N3_RESPONSE_CACHE_SIZE
#define N3_RESPONSE_CACHE_SIZE 8U
#endif

#ifndef N3_RESPONSE_MAX
#define N3_RESPONSE_MAX 512U
#endif

#ifndef N3_TELEMETRY_PERIOD_MS
#define N3_TELEMETRY_PERIOD_MS 500U
#endif

#ifndef N3_HELLO_PERIOD_MS
#define N3_HELLO_PERIOD_MS 1000U
#endif

#ifndef N3_TASK_STACK_SIZE
#define N3_TASK_STACK_SIZE 6144U
#endif

#ifndef N3_UPLOAD_MAX_SIZE
#define N3_UPLOAD_MAX_SIZE (512U * 1024U)
#endif

#ifndef N3_UPLOAD_JOB_ID_MAX
#define N3_UPLOAD_JOB_ID_MAX 64U
#endif

#ifndef N3_UPLOAD_TIMEOUT_MS
#define N3_UPLOAD_TIMEOUT_MS 5000U
#endif

#define N3_UPLOAD_TEMP_PATH "/spiffs/n3_upload.tmp"
#define N3_UPLOAD_CURRENT_PATH "/spiffs/n3_current.traj"


static const char *TAG = "n3_service";


typedef struct
{
    bool active;
    n3_transport_endpoint_t endpoint;
    FILE *file;
    char job_id[N3_UPLOAD_JOB_ID_MAX];
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t received_size;
    uint32_t running_crc32;
    int64_t last_activity_us;
    bool write_failed;
} n3_upload_state_t;


typedef struct
{
    bool valid;
    uint32_t id;
    uint32_t request_fingerprint;
    char response[N3_RESPONSE_MAX];
} n3_cached_response_t;


typedef struct
{
    SemaphoreHandle_t mutex;
    bool estop_active;
    bool pose_valid;
    float x_mm;
    float y_mm;
    float yaw_deg;
    uint32_t pose_seq;
    char runner[16];
    char tracker[16];
    char pen[16];
    char current_job[N3_UPLOAD_JOB_ID_MAX];
    uint32_t current_job_size;
    uint32_t current_job_crc32;
    char error[64];
    n3_upload_state_t upload;
    n3_cached_response_t response_cache[N3_RESPONSE_CACHE_SIZE];
    size_t response_cache_next;
} n3_state_t;


typedef struct
{
    n3_transport_endpoint_t endpoint;
    const char *name;
} n3_rx_task_arg_t;


typedef struct
{
    char bytes[N3_JSON_LINE_MAX];
    size_t length;
    bool overflow;
} n3_line_parser_t;


static n3_state_t s_state;
static bool s_started;
static bool s_spiffs_ready;
static n3_rx_task_arg_t s_uart_rx_arg =
{
    .endpoint = N3_TRANSPORT_ENDPOINT_UART0,
    .name = "uart0",
};
static n3_rx_task_arg_t s_usb_rx_arg =
{
    .endpoint = N3_TRANSPORT_ENDPOINT_USB_SERIAL_JTAG,
    .name = "usb_serial_jtag",
};
static n3_rx_task_arg_t s_wifi_tcp_rx_arg =
{
    .endpoint = N3_TRANSPORT_ENDPOINT_WIFI_TCP,
    .name = "wifi_tcp",
};


static bool json_get_finite_number(
    const cJSON *object,
    const char *name,
    double *value)
{
    if ((object == NULL) || (name == NULL) || (value == NULL))
    {
        return false;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if ((item == NULL) || !cJSON_IsNumber(item) || !isfinite(item->valuedouble))
    {
        return false;
    }

    *value = item->valuedouble;
    return true;
}


static bool json_get_uint32(
    const cJSON *object,
    const char *name,
    uint32_t *value)
{
    double number = 0.0;
    if (!json_get_finite_number(object, name, &number) ||
        number < 0.0 ||
        number > (double)UINT32_MAX ||
        floor(number) != number)
    {
        return false;
    }

    *value = (uint32_t)number;
    return true;
}


static bool json_get_crc32(
    const cJSON *object,
    const char *name,
    uint32_t *value)
{
    if ((object == NULL) || (name == NULL) || (value == NULL))
    {
        return false;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (item == NULL)
    {
        return false;
    }

    if (cJSON_IsNumber(item))
    {
        return json_get_uint32(object, name, value);
    }

    if (!cJSON_IsString(item) ||
        item->valuestring == NULL ||
        strlen(item->valuestring) != 8U)
    {
        return false;
    }

    for (size_t index = 0U; index < 8U; ++index)
    {
        if (!isxdigit((unsigned char)item->valuestring[index]))
        {
            return false;
        }
    }

    char *end = NULL;
    const unsigned long parsed = strtoul(item->valuestring, &end, 16);
    if ((end == NULL) || (*end != '\0') || parsed > UINT32_MAX)
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}


static uint32_t crc32_update(
    uint32_t crc,
    const uint8_t *bytes,
    size_t length)
{
    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= bytes[index];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 1U) != 0U
                ? (crc >> 1U) ^ 0xEDB88320U
                : (crc >> 1U);
        }
    }

    return crc;
}


static const char *json_get_string(
    const cJSON *object,
    const char *name)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    return (item != NULL && cJSON_IsString(item)) ? item->valuestring : NULL;
}


static void state_copy_text(
    char *destination,
    size_t destination_size,
    const char *source)
{
    if ((destination == NULL) || (destination_size == 0U))
    {
        return;
    }

    if (source == NULL)
    {
        destination[0] = '\0';
        return;
    }

    (void)snprintf(destination, destination_size, "%s", source);
}


static void upload_reset_locked(
    bool remove_temp)
{
    if (s_state.upload.file != NULL)
    {
        fclose(s_state.upload.file);
    }

    s_state.upload.file = NULL;
    if (remove_temp)
    {
        (void)remove(N3_UPLOAD_TEMP_PATH);
    }

    memset(&s_state.upload, 0, sizeof(s_state.upload));
}


static bool upload_begin(
    const cJSON *message,
    n3_transport_endpoint_t endpoint,
    const char **error_code,
    const char **error_message)
{
    const char *job_id = json_get_string(message, "job_id");
    uint32_t size = 0U;
    uint32_t crc32 = 0U;

    if ((job_id == NULL) || (job_id[0] == '\0') ||
        strlen(job_id) >= N3_UPLOAD_JOB_ID_MAX)
    {
        *error_code = "PROTOCOL";
        *error_message = "UPLOAD_BEGIN requires a short non-empty job_id";
        return false;
    }

    if (!json_get_uint32(message, "size", &size) ||
        size == 0U ||
        size > N3_UPLOAD_MAX_SIZE)
    {
        *error_code = "UPLOAD_SIZE";
        *error_message = "trajectory size is zero or exceeds the upload limit";
        return false;
    }

    if (!json_get_crc32(message, "crc32", &crc32))
    {
        *error_code = "PROTOCOL";
        *error_message = "UPLOAD_BEGIN requires an 8-digit crc32";
        return false;
    }

    if (!s_spiffs_ready)
    {
        *error_code = "NOT_READY";
        *error_message = "SPIFFS storage is not mounted";
        return false;
    }

    if (n3_runner_bridge_is_active() || n3_motion_is_active())
    {
        *error_code = "RUNNER_BUSY";
        *error_message = "cannot replace a trajectory while the runner is active";
        return false;
    }

    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        *error_code = "INTERNAL";
        *error_message = "state mutex unavailable";
        return false;
    }

    if (s_state.upload.active)
    {
        xSemaphoreGive(s_state.mutex);
        *error_code = "UPLOAD_BUSY";
        *error_message = "another trajectory upload is in progress";
        return false;
    }

    (void)remove(N3_UPLOAD_TEMP_PATH);
    FILE *file = fopen(N3_UPLOAD_TEMP_PATH, "wb");
    if (file == NULL)
    {
        xSemaphoreGive(s_state.mutex);
        *error_code = "STORAGE";
        *error_message = "cannot open the trajectory temporary file";
        return false;
    }

    memset(&s_state.upload, 0, sizeof(s_state.upload));
    s_state.upload.active = true;
    s_state.upload.endpoint = endpoint;
    s_state.upload.file = file;
    s_state.upload.expected_size = size;
    s_state.upload.expected_crc32 = crc32;
    s_state.upload.running_crc32 = 0xFFFFFFFFU;
    s_state.upload.last_activity_us = esp_timer_get_time();
    state_copy_text(
        s_state.upload.job_id,
        sizeof(s_state.upload.job_id),
        job_id);
    state_copy_text(s_state.runner, sizeof(s_state.runner), "UPLOADING");
    s_state.error[0] = '\0';

    xSemaphoreGive(s_state.mutex);
    return true;
}


static size_t upload_consume_bytes(
    n3_transport_endpoint_t endpoint,
    const uint8_t *bytes,
    size_t length)
{
    if ((bytes == NULL) || (length == 0U) ||
        (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE))
    {
        return 0U;
    }

    if (!s_state.upload.active ||
        s_state.upload.endpoint != endpoint ||
        s_state.upload.received_size >= s_state.upload.expected_size)
    {
        xSemaphoreGive(s_state.mutex);
        return 0U;
    }

    const size_t remaining =
        (size_t)(s_state.upload.expected_size - s_state.upload.received_size);
    const size_t consumed = (length < remaining) ? length : remaining;

    if (!s_state.upload.write_failed && s_state.upload.file != NULL)
    {
        const size_t written =
            fwrite(bytes, 1U, consumed, s_state.upload.file);
        if (written != consumed)
        {
            s_state.upload.write_failed = true;
        }
    }
    else
    {
        s_state.upload.write_failed = true;
    }

    s_state.upload.running_crc32 =
        crc32_update(
            s_state.upload.running_crc32,
            bytes,
            consumed);
    s_state.upload.received_size += (uint32_t)consumed;
    s_state.upload.last_activity_us = esp_timer_get_time();

    xSemaphoreGive(s_state.mutex);
    return consumed;
}


static bool validate_trajectory_file(void)
{
    trajectory_decoder_t decoder = TRAJECTORY_DECODER_INITIALIZER;
    esp_err_t ret = trajectory_decoder_open(&decoder, N3_UPLOAD_TEMP_PATH);
    if (ret != ESP_OK)
    {
        return false;
    }

    trajectory_file_header_t header = {0};
    ret = trajectory_decoder_get_header(&decoder, &header);
    if ((ret == ESP_OK) && (header.version != TRAJECTORY_FILE_VERSION_V2))
    {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    while ((ret == ESP_OK) && trajectory_decoder_has_next(&decoder))
    {
        trajectory_record_t record = {0};
        ret = trajectory_decoder_read_next_record(&decoder, &record);
    }

    trajectory_decoder_close(&decoder);
    return ret == ESP_OK;
}


static bool upload_finish(
    const cJSON *message,
    const char **error_code,
    const char **error_message)
{
    const char *job_id = json_get_string(message, "job_id");
    uint32_t declared_size = 0U;
    uint32_t declared_crc32 = 0U;
    const cJSON *size_item = cJSON_GetObjectItemCaseSensitive(message, "size");
    const cJSON *crc_item = cJSON_GetObjectItemCaseSensitive(message, "crc32");

    if ((job_id == NULL) || (job_id[0] == '\0'))
    {
        *error_code = "PROTOCOL";
        *error_message = "UPLOAD_END requires job_id";
        return false;
    }

    if ((size_item != NULL && !json_get_uint32(message, "size", &declared_size)) ||
        (crc_item != NULL && !json_get_crc32(message, "crc32", &declared_crc32)))
    {
        *error_code = "PROTOCOL";
        *error_message = "UPLOAD_END size or crc32 is invalid";
        return false;
    }

    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        *error_code = "INTERNAL";
        *error_message = "state mutex unavailable";
        return false;
    }

    if (!s_state.upload.active)
    {
        xSemaphoreGive(s_state.mutex);
        *error_code = "UPLOAD_STATE";
        *error_message = "no trajectory upload is active";
        return false;
    }

    const bool metadata_matches =
        strcmp(job_id, s_state.upload.job_id) == 0 &&
        (size_item == NULL || declared_size == s_state.upload.expected_size) &&
        (crc_item == NULL || declared_crc32 == s_state.upload.expected_crc32);

    if (!metadata_matches ||
        s_state.upload.received_size != s_state.upload.expected_size)
    {
        upload_reset_locked(true);
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        xSemaphoreGive(s_state.mutex);
        *error_code = "CRC_MISMATCH";
        *error_message = "trajectory size or upload metadata mismatch";
        return false;
    }

    if (s_state.upload.write_failed ||
        (s_state.upload.running_crc32 ^ 0xFFFFFFFFU) != s_state.upload.expected_crc32)
    {
        upload_reset_locked(true);
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        xSemaphoreGive(s_state.mutex);
        *error_code = "CRC_MISMATCH";
        *error_message = "trajectory bytes failed size or CRC32 validation";
        return false;
    }

    if ((s_state.upload.file == NULL) ||
        fflush(s_state.upload.file) != 0)
    {
        upload_reset_locked(true);
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        xSemaphoreGive(s_state.mutex);
        *error_code = "STORAGE";
        *error_message = "cannot flush the trajectory temporary file";
        return false;
    }

    fclose(s_state.upload.file);
    s_state.upload.file = NULL;

    if (!validate_trajectory_file())
    {
        upload_reset_locked(true);
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        xSemaphoreGive(s_state.mutex);
        *error_code = "TRJ_INVALID";
        *error_message = "trajectory decoder rejected the uploaded TRJ2 file";
        return false;
    }

    (void)remove(N3_UPLOAD_CURRENT_PATH);
    if (rename(N3_UPLOAD_TEMP_PATH, N3_UPLOAD_CURRENT_PATH) != 0)
    {
        upload_reset_locked(true);
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        xSemaphoreGive(s_state.mutex);
        *error_code = "STORAGE";
        *error_message = "cannot promote the validated trajectory file";
        return false;
    }

    state_copy_text(
        s_state.current_job,
        sizeof(s_state.current_job),
        s_state.upload.job_id);
    s_state.current_job_size = s_state.upload.expected_size;
    s_state.current_job_crc32 = s_state.upload.expected_crc32;
    memset(&s_state.upload, 0, sizeof(s_state.upload));
    state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
    s_state.error[0] = '\0';

    xSemaphoreGive(s_state.mutex);
    return true;
}


static bool upload_abort(
    const cJSON *message,
    const char **error_code,
    const char **error_message)
{
    const char *job_id = json_get_string(message, "job_id");
    if ((job_id == NULL) || (job_id[0] == '\0'))
    {
        *error_code = "PROTOCOL";
        *error_message = "UPLOAD_ABORT requires job_id";
        return false;
    }

    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        *error_code = "INTERNAL";
        *error_message = "state mutex unavailable";
        return false;
    }

    if (!s_state.upload.active || strcmp(job_id, s_state.upload.job_id) != 0)
    {
        xSemaphoreGive(s_state.mutex);
        *error_code = "UPLOAD_STATE";
        *error_message = "requested upload is not active";
        return false;
    }

    upload_reset_locked(true);
    state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
    xSemaphoreGive(s_state.mutex);
    return true;
}


static void set_state_error(
    const char *message)
{
    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) == pdTRUE)
    {
        state_copy_text(s_state.error, sizeof(s_state.error), message);
        xSemaphoreGive(s_state.mutex);
    }
}


static void upload_expire_if_stale(void)
{
    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t timeout_us = (int64_t)N3_UPLOAD_TIMEOUT_MS * 1000LL;
    if (s_state.upload.active &&
        (now_us - s_state.upload.last_activity_us) > timeout_us)
    {
        upload_reset_locked(true);
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        state_copy_text(s_state.error, sizeof(s_state.error), "trajectory upload timeout");
    }

    xSemaphoreGive(s_state.mutex);
}


static bool response_cache_find(
    uint32_t id,
    uint32_t request_fingerprint,
    char *response,
    size_t response_size)
{
    bool found = false;

    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    for (size_t index = 0U; index < N3_RESPONSE_CACHE_SIZE; ++index)
    {
        const n3_cached_response_t *entry = &s_state.response_cache[index];
        if (entry->valid &&
            entry->id == id &&
            entry->request_fingerprint == request_fingerprint)
        {
            state_copy_text(response, response_size, entry->response);
            found = true;
            break;
        }
    }

    xSemaphoreGive(s_state.mutex);
    return found;
}


static void response_cache_store(
    uint32_t id,
    uint32_t request_fingerprint,
    const char *response)
{
    if ((response == NULL) || (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE))
    {
        return;
    }

    n3_cached_response_t *entry =
        &s_state.response_cache[s_state.response_cache_next];
    entry->valid = true;
    entry->id = id;
    entry->request_fingerprint = request_fingerprint;
    state_copy_text(entry->response, sizeof(entry->response), response);
    s_state.response_cache_next =
        (s_state.response_cache_next + 1U) % N3_RESPONSE_CACHE_SIZE;

    xSemaphoreGive(s_state.mutex);
}


static uint32_t request_fingerprint(
    const char *line,
    size_t length)
{
    /* FNV-1a is sufficient here: this is a compact request identity used to
     * distinguish a retried command from a new connection that restarted its
     * command counter at one.  It is not a security or integrity checksum. */
    uint32_t hash = 2166136261U;

    if (line == NULL)
    {
        return hash;
    }

    for (size_t index = 0U; index < length; ++index)
    {
        hash ^= (uint8_t)line[index];
        hash *= 16777619U;
    }

    return hash;
}


static esp_err_t emit_serialized(
    const char *serialized,
    size_t length)
{
    if ((serialized == NULL) || (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Keep the JSON payload and its delimiter in one transport write.  STATUS
     * telemetry and command replies are produced by different tasks; writing
     * the delimiter separately allowed two frames to be merged into one line
     * under load, which the PC correctly rejected as "extra data". */
    char *frame = malloc(length + 1U);
    if (frame == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    memcpy(frame, serialized, length);
    frame[length] = '\n';
    const esp_err_t ret = n3_transport_write(frame, length + 1U);
    free(frame);
    return ret;
}


static esp_err_t emit_json(
    cJSON *object,
    uint32_t cache_id,
    uint32_t cache_fingerprint,
    bool cache_response)
{
    if (object == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char *serialized = cJSON_PrintUnformatted(object);
    if (serialized == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    const size_t length = strlen(serialized);
    esp_err_t ret = emit_serialized(serialized, length);

    if ((ret == ESP_OK) && cache_response && (length < N3_RESPONSE_MAX))
    {
        response_cache_store(cache_id, cache_fingerprint, serialized);
    }

    cJSON_free(serialized);
    return ret;
}


static void emit_cached_response(
    const char *response)
{
    if (response != NULL)
    {
        (void)emit_serialized(response, strlen(response));
    }
}


static void send_ack(
    uint32_t id,
    const char *ack_type,
    uint32_t request_key)
{
    cJSON *response = cJSON_CreateObject();
    if (response == NULL)
    {
        return;
    }

    cJSON_AddStringToObject(response, "type", "ACK");
    cJSON_AddNumberToObject(response, "id", id);
    cJSON_AddStringToObject(response, "ack_type", ack_type);
    (void)emit_json(response, id, request_key, true);
    cJSON_Delete(response);
}


static void send_error(
    uint32_t id,
    const char *code,
    const char *message,
    uint32_t request_key,
    bool cache_response)
{
    cJSON *response = cJSON_CreateObject();
    if (response == NULL)
    {
        return;
    }

    cJSON_AddStringToObject(response, "type", "ERROR");
    cJSON_AddNumberToObject(response, "id", id);
    cJSON_AddStringToObject(response, "code", code);
    cJSON_AddStringToObject(response, "message", message);
    (void)emit_json(response, id, request_key, cache_response);
    cJSON_Delete(response);
}


static void snapshot_state(
    bool *estop_active,
    bool *pose_valid,
    float *x_mm,
    float *y_mm,
    float *yaw_deg,
    uint32_t *pose_seq,
    char *runner,
    size_t runner_size,
    char *tracker,
    size_t tracker_size,
    char *pen,
    size_t pen_size,
    char *current_job,
    size_t current_job_size,
    char *error,
    size_t error_size)
{
    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    if (estop_active != NULL)
    {
        *estop_active = s_state.estop_active;
    }
    if (pose_valid != NULL)
    {
        *pose_valid = s_state.pose_valid;
    }
    if (x_mm != NULL)
    {
        *x_mm = s_state.x_mm;
    }
    if (y_mm != NULL)
    {
        *y_mm = s_state.y_mm;
    }
    if (yaw_deg != NULL)
    {
        *yaw_deg = s_state.yaw_deg;
    }
    if (pose_seq != NULL)
    {
        *pose_seq = ++s_state.pose_seq;
    }
    state_copy_text(runner, runner_size, s_state.runner);
    state_copy_text(tracker, tracker_size, s_state.tracker);
    state_copy_text(pen, pen_size, s_state.pen);
    state_copy_text(current_job, current_job_size, s_state.current_job);
    state_copy_text(error, error_size, s_state.error);

    xSemaphoreGive(s_state.mutex);
}


static void emit_hello(void)
{
    cJSON *hello = cJSON_CreateObject();
    cJSON *features = cJSON_CreateArray();
    if ((hello == NULL) || (features == NULL))
    {
        cJSON_Delete(hello);
        cJSON_Delete(features);
        return;
    }

    cJSON_AddStringToObject(hello, "type", "HELLO");
    cJSON_AddNumberToObject(hello, "protocol", 1);
    cJSON_AddStringToObject(hello, "firmware", "test-motor-n3");
    cJSON_AddStringToObject(hello, "build_profile", N3_BUILD_PROFILE);
    cJSON_AddStringToObject(hello, "execution_mode", N3_EXECUTION_MODE);
    cJSON_AddBoolToObject(hello, "hardware_enabled", N3_ENABLE_HARDWARE);
    cJSON_AddBoolToObject(hello, "motion_enabled", N3_ENABLE_MOTION);
    cJSON_AddBoolToObject(hello, "circle_enabled", N3_ENABLE_CIRCLE);
    cJSON_AddBoolToObject(hello, "rotate_enabled", N3_ENABLE_ROTATE_REL);
    cJSON_AddBoolToObject(hello, "pen_enabled", N3_ENABLE_PEN);
    cJSON_AddBoolToObject(hello, "simulation_enabled", N3_ENABLE_SIMULATION);
    cJSON_AddItemToArray(features, cJSON_CreateString("ping"));
    cJSON_AddItemToArray(features, cJSON_CreateString("status"));
    cJSON_AddItemToArray(features, cJSON_CreateString("pose"));
    cJSON_AddItemToArray(features, cJSON_CreateString("upload"));
#if N3_ENABLE_SIMULATION
    cJSON_AddItemToArray(features, cJSON_CreateString("run_simulation"));
#endif
#if N3_ENABLE_MOTION
    cJSON_AddItemToArray(features, cJSON_CreateString("run_line"));
#endif
#if N3_ENABLE_CIRCLE
    cJSON_AddItemToArray(features, cJSON_CreateString("run_circle"));
#endif
#if N3_ENABLE_ROTATE_REL
    cJSON_AddItemToArray(features, cJSON_CreateString("rotate_rel"));
#endif
    cJSON_AddItemToObject(hello, "features", features);
    (void)emit_json(hello, 0U, 0U, false);
    cJSON_Delete(hello);
}


static void emit_status(void)
{
    bool estop_active = false;
    bool pose_valid = false;
    char runner[16] = {0};
    char tracker[16] = {0};
    char pen[16] = {0};
    char current_job[N3_UPLOAD_JOB_ID_MAX] = {0};
    char error[64] = {0};
    n3_runner_status_t runner_status = {0};
    n3_hardware_status_t hardware_status = {0};
    n3_motion_status_t motion_status = {0};

    snapshot_state(
        &estop_active,
        &pose_valid,
        NULL,
        NULL,
        NULL,
        NULL,
        runner,
        sizeof(runner),
        tracker,
        sizeof(tracker),
        pen,
        sizeof(pen),
        current_job,
        sizeof(current_job),
        error,
        sizeof(error));

    n3_runner_bridge_get_status(&runner_status);
    n3_hardware_get_status(&hardware_status);
    n3_motion_get_status(&motion_status);
    const bool runner_active = n3_runner_bridge_is_active();
    const bool runtime_motion_ready =
        (N3_ENABLE_MOTION != 0)
            ? (hardware_status.motor_ready && hardware_status.odom_ready)
            : motion_status.ready;
    const bool runtime_motion_active =
        runner_active || motion_status.active;
    if (hardware_status.enabled)
    {
        pose_valid = hardware_status.pose_valid;
    }
    if (runner_status.valid)
    {
        state_copy_text(runner, sizeof(runner), runner_status.runner);
        state_copy_text(tracker, sizeof(tracker), runner_status.tracker);
        state_copy_text(pen, sizeof(pen), runner_status.pen);
        if (runner_status.error[0] != '\0')
        {
            state_copy_text(error, sizeof(error), runner_status.error);
        }
        if (!hardware_status.enabled && runner_status.pose_valid)
        {
            pose_valid = true;
        }
    }
    if (hardware_status.error[0] != '\0')
    {
        state_copy_text(error, sizeof(error), hardware_status.error);
    }
    if (motion_status.enabled)
    {
        state_copy_text(runner, sizeof(runner), motion_status.runner);
        state_copy_text(tracker, sizeof(tracker), motion_status.tracker);
        if (motion_status.error[0] != '\0')
        {
            state_copy_text(error, sizeof(error), motion_status.error);
        }
    }

    cJSON *status = cJSON_CreateObject();
    if (status == NULL)
    {
        return;
    }

    cJSON_AddStringToObject(status, "type", "STATUS");
    cJSON_AddStringToObject(status, "runner", runner);
    cJSON_AddStringToObject(status, "tracker", tracker);
    cJSON_AddStringToObject(status, "pen", pen);
    cJSON_AddStringToObject(status, "build_profile", N3_BUILD_PROFILE);
    cJSON_AddBoolToObject(status, "hardware_enabled", N3_ENABLE_HARDWARE);
    cJSON_AddBoolToObject(status, "motion_enabled", N3_ENABLE_MOTION);
    cJSON_AddBoolToObject(status, "circle_enabled", N3_ENABLE_CIRCLE);
    cJSON_AddBoolToObject(status, "rotate_enabled", N3_ENABLE_ROTATE_REL);
    cJSON_AddBoolToObject(status, "pen_enabled", N3_ENABLE_PEN);
    cJSON_AddStringToObject(
        status,
        "pen_state",
        runner_status.valid && runner_status.pen_state[0] != '\0'
            ? runner_status.pen_state
            : pen);
    cJSON_AddStringToObject(
        status,
        "pen_target",
        runner_status.valid && runner_status.pen_target[0] != '\0'
            ? runner_status.pen_target
            : pen);
    cJSON_AddBoolToObject(
        status,
        "pen_busy",
        runner_status.valid && runner_status.pen_busy);
    cJSON_AddBoolToObject(
        status,
        "pen_settling",
        runner_status.valid && runner_status.pen_settling);
    cJSON_AddNumberToObject(
        status,
        "pen_current_pulse_us",
        runner_status.valid ? runner_status.pen_current_pulse_us : 0U);
    cJSON_AddNumberToObject(
        status,
        "pen_target_pulse_us",
        runner_status.valid ? runner_status.pen_target_pulse_us : 0U);
    cJSON_AddStringToObject(
        status,
        "pen_error",
        runner_status.valid && runner_status.pen_error[0] != '\0'
            ? runner_status.pen_error
            : "NONE");
    cJSON_AddBoolToObject(status, "simulation_enabled", N3_ENABLE_SIMULATION);
    cJSON_AddBoolToObject(status, "hardware_initializing", hardware_status.initializing);
    cJSON_AddBoolToObject(status, "motion_ready", runtime_motion_ready);
    cJSON_AddBoolToObject(status, "motion_active", runtime_motion_active);
    cJSON_AddBoolToObject(status, "motion_safety_latched", motion_status.safety_latched);
    cJSON_AddStringToObject(
        status,
        "execution_mode",
        runner_status.valid && runner_status.execution_mode[0] != '\0'
            ? runner_status.execution_mode
            : N3_EXECUTION_MODE);
    cJSON_AddBoolToObject(
        status,
        "stop_requested",
        (runner_status.valid && runner_status.stop_requested) ||
        (motion_status.enabled && motion_status.stop_requested));
    cJSON_AddNumberToObject(
        status,
        "record_index",
        runner_status.valid ? runner_status.record_index : 0U);
    cJSON_AddNumberToObject(
        status,
        "record_count",
        runner_status.valid ? runner_status.record_count : 0U);
    cJSON_AddStringToObject(
        status,
        "record_type",
        runner_status.valid && runner_status.record_type[0] != '\0'
            ? runner_status.record_type
            : "NONE");
    if (runner_status.phase[0] != '\0')
    {
        cJSON_AddStringToObject(status, "tracker_phase", runner_status.phase);
    }
    else
    {
        cJSON_AddStringToObject(status, "tracker_phase", "NONE");
    }
    if (runner_status.error_code[0] != '\0')
    {
        cJSON_AddStringToObject(status, "runner_error", runner_status.error_code);
    }
    else
    {
        cJSON_AddStringToObject(status, "runner_error", "NONE");
    }
    cJSON_AddStringToObject(
        status,
        "tracker_error",
        runner_status.tracker_error[0] != '\0'
            ? runner_status.tracker_error
            : "NONE");
    cJSON_AddNumberToObject(status, "tracking_error_mm", runner_status.tracking_error_mm);
    cJSON_AddNumberToObject(status, "tracking_yaw_error_deg", runner_status.tracking_yaw_error_deg);
    cJSON_AddNumberToObject(status, "reference_x_mm", runner_status.reference_x_mm);
    cJSON_AddNumberToObject(status, "reference_y_mm", runner_status.reference_y_mm);
    cJSON_AddNumberToObject(status, "command_vx_body_mm_s", runner_status.command_vx_body_mm_s);
    cJSON_AddNumberToObject(status, "command_vy_body_mm_s", runner_status.command_vy_body_mm_s);
    cJSON_AddNumberToObject(status, "command_w_rad_s", runner_status.command_w_rad_s);
    cJSON_AddNumberToObject(status, "settle_elapsed_ms", runner_status.settle_elapsed_ms);
    cJSON_AddNumberToObject(status, "wheel_a_target_mm_s", runner_status.wheel_a_target_mm_s);
    cJSON_AddNumberToObject(status, "wheel_a_actual_mm_s", runner_status.wheel_a_actual_mm_s);
    cJSON_AddNumberToObject(status, "wheel_a_pwm", runner_status.wheel_a_pwm);
    cJSON_AddBoolToObject(status, "wheel_a_stall_suspected", runner_status.wheel_a_stall_suspected);
    cJSON_AddNumberToObject(status, "wheel_a_stall_elapsed_ms", runner_status.wheel_a_stall_elapsed_ms);
    cJSON_AddNumberToObject(status, "wheel_b_target_mm_s", runner_status.wheel_b_target_mm_s);
    cJSON_AddNumberToObject(status, "wheel_b_actual_mm_s", runner_status.wheel_b_actual_mm_s);
    cJSON_AddNumberToObject(status, "wheel_b_pwm", runner_status.wheel_b_pwm);
    cJSON_AddBoolToObject(status, "wheel_b_stall_suspected", runner_status.wheel_b_stall_suspected);
    cJSON_AddNumberToObject(status, "wheel_b_stall_elapsed_ms", runner_status.wheel_b_stall_elapsed_ms);
    cJSON_AddNumberToObject(status, "wheel_d_target_mm_s", runner_status.wheel_d_target_mm_s);
    cJSON_AddNumberToObject(status, "wheel_d_actual_mm_s", runner_status.wheel_d_actual_mm_s);
    cJSON_AddNumberToObject(status, "wheel_d_pwm", runner_status.wheel_d_pwm);
    cJSON_AddBoolToObject(status, "wheel_d_stall_suspected", runner_status.wheel_d_stall_suspected);
    cJSON_AddNumberToObject(status, "wheel_d_stall_elapsed_ms", runner_status.wheel_d_stall_elapsed_ms);
    cJSON_AddBoolToObject(status, "motor_ready", hardware_status.motor_ready);
    cJSON_AddBoolToObject(status, "odom_ready", hardware_status.odom_ready);
    cJSON_AddBoolToObject(status, "pose_valid", pose_valid);
    cJSON_AddBoolToObject(
        status,
        "estop",
        estop_active || hardware_status.estop || motion_status.safety_latched);
    if (current_job[0] != '\0')
    {
        cJSON_AddStringToObject(status, "current_job", current_job);
    }
    else
    {
        cJSON_AddNullToObject(status, "current_job");
    }
    if (error[0] != '\0')
    {
        cJSON_AddStringToObject(status, "error", error);
    }
    else
    {
        cJSON_AddNullToObject(status, "error");
    }
    (void)emit_json(status, 0U, 0U, false);
    cJSON_Delete(status);
}


static void emit_pose(void)
{
    bool pose_valid = false;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float yaw_deg = 0.0f;
    uint32_t pose_seq = 0U;
    n3_runner_status_t runner_status = {0};
    n3_hardware_status_t hardware_status = {0};

    snapshot_state(
        NULL,
        &pose_valid,
        &x_mm,
        &y_mm,
        &yaw_deg,
        &pose_seq,
        NULL,
        0U,
        NULL,
        0U,
        NULL,
        0U,
        NULL,
        0U,
        NULL,
        0U);

    n3_runner_bridge_get_status(&runner_status);
    n3_hardware_get_status(&hardware_status);
    if (hardware_status.pose_valid)
    {
        pose_valid = true;
        x_mm = hardware_status.x_mm;
        y_mm = hardware_status.y_mm;
        yaw_deg = hardware_status.yaw_deg;
    }
    else if (!hardware_status.enabled && runner_status.valid && runner_status.pose_valid)
    {
        pose_valid = true;
        x_mm = runner_status.x_mm;
        y_mm = runner_status.y_mm;
        yaw_deg = runner_status.yaw_deg;
    }
    if (hardware_status.enabled)
    {
        pose_valid = hardware_status.pose_valid;
    }

    cJSON *pose = cJSON_CreateObject();
    if (pose == NULL)
    {
        return;
    }

    cJSON_AddStringToObject(pose, "type", "POSE");
    cJSON_AddNumberToObject(pose, "seq", pose_seq);
    cJSON_AddNumberToObject(pose, "x_mm", x_mm);
    cJSON_AddNumberToObject(pose, "y_mm", y_mm);
    cJSON_AddNumberToObject(pose, "yaw_deg", yaw_deg);
    cJSON_AddNumberToObject(
        pose,
        "vx_world_mm_s",
        hardware_status.pose_valid
            ? hardware_status.vx_world_mm_s
            : (runner_status.valid ? runner_status.vx_world_mm_s : 0.0));
    cJSON_AddNumberToObject(
        pose,
        "vy_world_mm_s",
        hardware_status.pose_valid
            ? hardware_status.vy_world_mm_s
            : (runner_status.valid ? runner_status.vy_world_mm_s : 0.0));
    cJSON_AddNumberToObject(
        pose,
        "w_rad_s",
        hardware_status.pose_valid
            ? hardware_status.w_rad_s
            : (runner_status.valid ? runner_status.w_rad_s : 0.0));
    cJSON_AddNumberToObject(
        pose,
        "timestamp_ms",
        (double)(esp_timer_get_time() / 1000LL));
    cJSON_AddBoolToObject(pose, "pose_valid", pose_valid);
    (void)emit_json(pose, 0U, 0U, false);
    cJSON_Delete(pose);
}


static void update_state_for_command(
    const char *kind,
    const cJSON *message)
{
    if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    if (strcmp(kind, "ESTOP") == 0)
    {
        s_state.estop_active = true;
        state_copy_text(s_state.runner, sizeof(s_state.runner), "ESTOPPED");
        state_copy_text(s_state.pen, sizeof(s_state.pen), "UP");
    }
    else if (strcmp(kind, "CLEAR_ESTOP") == 0)
    {
        s_state.estop_active = false;
        s_state.error[0] = '\0';
        state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
    }
    else if (strcmp(kind, "STOP") == 0)
    {
        if (!s_state.estop_active)
        {
            state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
        }
        state_copy_text(s_state.pen, sizeof(s_state.pen), "UP");
    }
    else if (strcmp(kind, "RESET_POSE") == 0)
    {
        double value = 0.0;
        if (json_get_finite_number(message, "x_mm", &value))
        {
            s_state.x_mm = (float)value;
        }
        if (json_get_finite_number(message, "y_mm", &value))
        {
            s_state.y_mm = (float)value;
        }
        if (json_get_finite_number(message, "yaw_deg", &value))
        {
            s_state.yaw_deg = (float)value;
        }
    }
    else if (strcmp(kind, "ROTATE_REL") == 0)
    {
        s_state.error[0] = '\0';
        state_copy_text(s_state.runner, sizeof(s_state.runner), "ROTATING");
    }

    xSemaphoreGive(s_state.mutex);
}


static void handle_command(
    const cJSON *message,
    uint32_t request_key,
    n3_transport_endpoint_t endpoint)
{
    const char *kind = json_get_string(message, "type");
    uint32_t id = 0U;

    if (kind == NULL)
    {
        send_error(0U, "PROTOCOL", "message type is required", 0U, false);
        return;
    }

    if (!json_get_uint32(message, "id", &id))
    {
        send_error(0U, "PROTOCOL", "command id must be a non-negative integer", 0U, false);
        return;
    }

    char cached_response[N3_RESPONSE_MAX] = {0};
    if (response_cache_find(id, request_key, cached_response, sizeof(cached_response)))
    {
        emit_cached_response(cached_response);
        return;
    }

    if (strcmp(kind, "PING") == 0)
    {
        send_ack(id, "PING", request_key);

        cJSON *pong = cJSON_CreateObject();
        if (pong != NULL)
        {
            cJSON_AddStringToObject(pong, "type", "PONG");
            cJSON_AddNumberToObject(pong, "id", id);
            const char *nonce = json_get_string(message, "nonce");
            cJSON_AddStringToObject(pong, "nonce", nonce != NULL ? nonce : "");
            (void)emit_json(pong, 0U, 0U, false);
            cJSON_Delete(pong);
        }
        return;
    }

    if (strcmp(kind, "STOP") == 0 ||
        strcmp(kind, "ESTOP") == 0 ||
        strcmp(kind, "CLEAR_ESTOP") == 0)
    {
        ESP_LOGW(
            TAG,
            "N3 command %s received (id=%lu, endpoint=%d)",
            kind,
            (unsigned long)id,
            (int)endpoint);
        if (strcmp(kind, "CLEAR_ESTOP") == 0)
        {
            const esp_err_t clear_ret = n3_hardware_clear_estop();
            if (clear_ret != ESP_OK)
            {
                send_error(id, "HARDWARE_NOT_READY", "cannot clear emergency stop before hardware is ready", request_key, true);
                return;
            }
            n3_motion_clear_estop();
            n3_runner_bridge_clear_estop();
        }
        else
        {
            n3_motion_stop(strcmp(kind, "ESTOP") == 0);
            n3_hardware_stop(strcmp(kind, "ESTOP") == 0);
            n3_runner_bridge_request_stop(strcmp(kind, "ESTOP") == 0);
        }
        update_state_for_command(kind, message);
        send_ack(id, kind, request_key);
        emit_status();
        emit_pose();
        return;
    }

    if (strcmp(kind, "RESET_POSE") == 0)
    {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double yaw_deg = 0.0;
        if (!json_get_finite_number(message, "x_mm", &x_mm) ||
            !json_get_finite_number(message, "y_mm", &y_mm) ||
            !json_get_finite_number(message, "yaw_deg", &yaw_deg))
        {
            send_error(id, "PROTOCOL", "RESET_POSE requires finite x_mm, y_mm, yaw_deg", request_key, true);
            return;
        }

        if (n3_runner_bridge_is_active() || n3_motion_is_active())
        {
            send_error(id, "RUNNER_BUSY", "cannot reset pose while a trajectory is active", request_key, true);
            return;
        }

        const esp_err_t reset_ret = n3_hardware_reset_pose(
            (float)x_mm,
            (float)y_mm,
            (float)yaw_deg);
        if (reset_ret != ESP_OK)
        {
            send_error(id, "ODOM_NOT_READY", "odometry is not ready for RESET_POSE", request_key, true);
            return;
        }

        n3_runner_bridge_reset_pose(
            (float)x_mm,
            (float)y_mm,
            (float)yaw_deg);
        update_state_for_command(kind, message);
        send_ack(id, kind, request_key);
        emit_pose();
        return;
    }

    if (strcmp(kind, "UPLOAD_BEGIN") == 0)
    {
        const char *error_code = NULL;
        const char *error_message = NULL;
        if (!upload_begin(message, endpoint, &error_code, &error_message))
        {
            set_state_error(error_message);
            send_error(id, error_code, error_message, request_key, true);
            emit_status();
            return;
        }

        send_ack(id, "UPLOAD_READY", request_key);
        emit_status();
        return;
    }

    if (strcmp(kind, "UPLOAD_END") == 0)
    {
        const char *error_code = NULL;
        const char *error_message = NULL;
        if (!upload_finish(message, &error_code, &error_message))
        {
            set_state_error(error_message);
            send_error(id, error_code, error_message, request_key, true);
            emit_status();
            return;
        }

        send_ack(id, "UPLOAD_END", request_key);
        emit_status();
        return;
    }

    if (strcmp(kind, "UPLOAD_ABORT") == 0)
    {
        const char *error_code = NULL;
        const char *error_message = NULL;
        if (!upload_abort(message, &error_code, &error_message))
        {
            set_state_error(error_message);
            send_error(id, error_code, error_message, request_key, true);
            emit_status();
            return;
        }

        send_ack(id, "UPLOAD_ABORT", request_key);
        emit_status();
        return;
    }

    if (strcmp(kind, "RUN_TRAJECTORY") == 0)
    {
        const char *job_id = json_get_string(message, "job_id");
        uint32_t crc32 = 0U;
        char current_job[N3_UPLOAD_JOB_ID_MAX] = {0};
        uint32_t current_crc32 = 0U;
        bool estop_active = false;
        bool upload_active = false;
        n3_hardware_status_t hardware_status = {0};

        if (!json_get_crc32(message, "crc32", &crc32))
        {
            send_error(id, "PROTOCOL", "RUN_TRAJECTORY requires crc32", request_key, true);
            return;
        }

        if (xSemaphoreTake(s_state.mutex, portMAX_DELAY) == pdTRUE)
        {
            state_copy_text(current_job, sizeof(current_job), s_state.current_job);
            current_crc32 = s_state.current_job_crc32;
            estop_active = s_state.estop_active;
            upload_active = s_state.upload.active;
            xSemaphoreGive(s_state.mutex);
        }
        n3_hardware_get_status(&hardware_status);

        if ((job_id == NULL) ||
            strcmp(job_id, current_job) != 0 ||
            crc32 != current_crc32)
        {
            send_error(id, "JOB_MISMATCH", "requested trajectory is not the validated upload", request_key, true);
        }
        else if (estop_active)
        {
            send_error(id, "ESTOP_ACTIVE", "clear emergency stop before running", request_key, true);
        }
        else if (hardware_status.estop)
        {
            send_error(
                id,
                "ESTOP_ACTIVE",
                "motor safety latch is active; use Clear E-Stop / re-arm instead of board Reset",
                request_key,
                true);
        }
        else if (upload_active)
        {
            send_error(id, "UPLOAD_BUSY", "cannot run while a trajectory upload is active", request_key, true);
        }
        else if (n3_motion_is_active())
        {
            send_error(id, "RUNNER_BUSY", "cannot run a trajectory while ROTATE_REL is active", request_key, true);
        }
        else
        {
            const char *error_code = NULL;
            const char *error_message = NULL;
            const esp_err_t ret = n3_runner_bridge_start_event_only(
                N3_UPLOAD_CURRENT_PATH,
                &error_code,
                &error_message);
            if (ret != ESP_OK)
            {
                set_state_error(error_message);
                send_error(
                    id,
                    error_code != NULL ? error_code : "INTERNAL",
                    error_message != NULL ? error_message : "event runner failed",
                    request_key,
                    true);
                emit_status();
            }
            else
            {
                ESP_LOGI(
                    TAG,
                    "N3 RUN_TRAJECTORY accepted (id=%lu, job=%s, crc32=%08lX)",
                    (unsigned long)id,
                    job_id,
                    (unsigned long)crc32);
                send_ack(id, "RUN_TRAJECTORY", request_key);
                emit_status();
            }
        }
        return;
    }

    if (strcmp(kind, "ROTATE_REL") == 0)
    {
        double angle_deg = 0.0;
        double speed_deg_s = 0.0;
        if (!json_get_finite_number(message, "angle_deg", &angle_deg) ||
            !json_get_finite_number(message, "speed_deg_s", &speed_deg_s) ||
            speed_deg_s <= 0.0)
        {
            send_error(id, "PROTOCOL", "ROTATE_REL requires finite angle and positive speed", request_key, true);
            return;
        }

        bool estop_active = false;
        (void)xSemaphoreTake(s_state.mutex, portMAX_DELAY);
        estop_active = s_state.estop_active;
        xSemaphoreGive(s_state.mutex);
        if (estop_active)
        {
            send_error(id, "ESTOP_ACTIVE", "clear emergency stop before rotating", request_key, true);
        }
#if !N3_ENABLE_ROTATE_REL
        else
        {
            send_error(id, "MOTION_DISABLED", "this firmware profile does not enable ROTATE_REL", request_key, true);
        }
#else
        else if (n3_runner_bridge_is_active() || n3_motion_is_active())
        {
            send_error(id, "RUNNER_BUSY", "another trajectory or rotation is active", request_key, true);
        }
        else
        {
            n3_hardware_status_t hardware_status = {0};
            n3_hardware_get_status(&hardware_status);
            if (hardware_status.estop)
            {
                send_error(id, "ESTOP_ACTIVE", "clear emergency stop before rotating", request_key, true);
                return;
            }

            const char *error_code = NULL;
            const char *error_message = NULL;
            const esp_err_t ret = n3_motion_rotate(
                (float)angle_deg,
                (float)speed_deg_s,
                &error_code,
                &error_message);
            if (ret != ESP_OK)
            {
                set_state_error(error_message);
                send_error(
                    id,
                    error_code != NULL ? error_code : "MOTION_ERROR",
                    error_message != NULL ? error_message : "ROTATE_REL rejected",
                    request_key,
                    true);
            }
            else
            {
                update_state_for_command(kind, message);
                send_ack(id, kind, request_key);
                emit_status();
                emit_pose();
            }
        }
#endif
        return;
    }

    send_error(id, "UNKNOWN_COMMAND", kind, request_key, true);
}


static void process_line(
    const char *line,
    size_t length,
    n3_transport_endpoint_t endpoint)
{
    if ((line == NULL) || (length == 0U))
    {
        return;
    }

    const uint32_t request_key = request_fingerprint(line, length);
    cJSON *message = cJSON_ParseWithLength(line, length);
    if ((message == NULL) || !cJSON_IsObject(message))
    {
        send_error(0U, "PROTOCOL", "invalid JSON object", 0U, false);
        cJSON_Delete(message);
        return;
    }

    n3_motion_note_transport_activity();
    n3_runner_bridge_note_transport_activity();
    handle_command(message, request_key, endpoint);
    cJSON_Delete(message);
}


static void parser_reset(
    n3_line_parser_t *parser)
{
    parser->length = 0U;
    parser->overflow = false;
}


static void parser_consume_byte(
    n3_line_parser_t *parser,
    uint8_t byte,
    n3_transport_endpoint_t endpoint)
{
    if (byte == '\n')
    {
        if (!parser->overflow)
        {
            while (parser->length > 0U &&
                   isspace((unsigned char)parser->bytes[parser->length - 1U]))
            {
                --parser->length;
            }
            parser->bytes[parser->length] = '\0';
            process_line(parser->bytes, parser->length, endpoint);
        }
        else
        {
            send_error(0U, "PROTOCOL", "JSON line exceeds maximum length", 0U, false);
        }

        parser_reset(parser);
        return;
    }

    if (parser->overflow || byte == '\r')
    {
        return;
    }

    if (parser->length + 1U >= sizeof(parser->bytes))
    {
        parser->overflow = true;
        return;
    }

    parser->bytes[parser->length++] = (char)byte;
}


static void n3_rx_task(
    void *arg)
{
    const n3_rx_task_arg_t *task_arg = (const n3_rx_task_arg_t *)arg;
    n3_line_parser_t parser = {0};
    uint8_t buffer[64];

    ESP_LOGI(TAG, "N3 RX task started: %s", task_arg->name);

    for (;;)
    {
        const int received = n3_transport_read(
            task_arg->endpoint,
            buffer,
            sizeof(buffer),
            100U);
        if (received <= 0)
        {
            continue;
        }

        size_t offset = 0U;
        while (offset < (size_t)received)
        {
            const size_t consumed = upload_consume_bytes(
                task_arg->endpoint,
                &buffer[offset],
                (size_t)received - offset);
            if (consumed > 0U)
            {
                offset += consumed;
                continue;
            }

            parser_consume_byte(&parser, buffer[offset], task_arg->endpoint);
            ++offset;
        }
    }
}


static void n3_telemetry_task(
    void *arg)
{
    (void)arg;

    emit_hello();
    emit_status();
    emit_pose();
    int64_t last_hello_us = esp_timer_get_time();

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(N3_TELEMETRY_PERIOD_MS));
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_hello_us) >=
            ((int64_t)N3_HELLO_PERIOD_MS * 1000LL))
        {
            emit_hello();
            last_hello_us = now_us;
        }
        upload_expire_if_stale();
        emit_status();
        emit_pose();
    }
}


static esp_err_t n3_storage_init(void)
{
    const esp_vfs_spiffs_conf_t config =
    {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 4,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&config);
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_spiffs_ready = true;

    size_t total = 0U;
    size_t used = 0U;
    if (esp_spiffs_info("storage", &total, &used) == ESP_OK)
    {
        ESP_LOGI(TAG, "N3 storage ready: total=%u used=%u", (unsigned)total, (unsigned)used);
    }

    return ESP_OK;
}


esp_err_t n3_service_start(void)
{
    if (s_started)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_state, 0, sizeof(s_state));
    s_spiffs_ready = false;
    s_state.mutex = xSemaphoreCreateMutex();
    if (s_state.mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    state_copy_text(s_state.runner, sizeof(s_state.runner), "IDLE");
    state_copy_text(s_state.tracker, sizeof(s_state.tracker), "IDLE");
    state_copy_text(s_state.pen, sizeof(s_state.pen), "UP");

    esp_err_t ret = n3_storage_init();
    if (ret != ESP_OK)
    {
        state_copy_text(s_state.error, sizeof(s_state.error), "SPIFFS mount failed");
    }

    ret = n3_runner_bridge_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = n3_hardware_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = n3_motion_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = n3_transport_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (n3_transport_endpoint_enabled(N3_TRANSPORT_ENDPOINT_UART0) &&
        xTaskCreate(
            n3_rx_task,
            "n3_rx_uart0",
            N3_TASK_STACK_SIZE,
            &s_uart_rx_arg,
            6,
            NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    if (n3_transport_endpoint_enabled(N3_TRANSPORT_ENDPOINT_USB_SERIAL_JTAG) &&
        xTaskCreate(
            n3_rx_task,
            "n3_rx_usb",
            N3_TASK_STACK_SIZE,
            &s_usb_rx_arg,
            6,
            NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    if (n3_transport_endpoint_enabled(N3_TRANSPORT_ENDPOINT_WIFI_TCP) &&
        xTaskCreate(
            n3_rx_task,
            "n3_rx_wifi",
            N3_TASK_STACK_SIZE,
            &s_wifi_tcp_rx_arg,
            6,
            NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(
            n3_telemetry_task,
            "n3_telemetry",
            N3_TASK_STACK_SIZE,
            NULL,
            5,
            NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "N3 service started; hardware initialization is asynchronous");
    return ESP_OK;
}
