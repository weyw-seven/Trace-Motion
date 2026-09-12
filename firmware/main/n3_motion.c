#include "n3_motion.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "chassis_motion.h"
#include "n3_build_config.h"
#include "n3_hardware.h"


#ifndef N3_ROTATE_MAX_ANGLE_DEG
#define N3_ROTATE_MAX_ANGLE_DEG 30.0f
#endif

#ifndef N3_ROTATE_MIN_SPEED_DEG_S
#define N3_ROTATE_MIN_SPEED_DEG_S 10.0f
#endif

#ifndef N3_ROTATE_MAX_SPEED_DEG_S
#define N3_ROTATE_MAX_SPEED_DEG_S 45.0f
#endif

#ifndef N3_ROTATE_LINK_TIMEOUT_MS
#define N3_ROTATE_LINK_TIMEOUT_MS 2500U
#endif

#ifndef N3_MOTION_SUPERVISOR_PERIOD_MS
#define N3_MOTION_SUPERVISOR_PERIOD_MS 20U
#endif

#ifndef N3_MOTION_TASK_STACK_SIZE
#define N3_MOTION_TASK_STACK_SIZE 4096U
#endif

#ifndef N3_MOTION_SUPERVISOR_PRIORITY
#define N3_MOTION_SUPERVISOR_PRIORITY 4U
#endif


static const char *TAG = "n3_motion";


typedef struct
{
    SemaphoreHandle_t mutex;
    bool enabled;
    bool ready;
    bool active;
    bool target_reached;
    bool stop_requested;
    bool safety_latched;
    uint32_t command_id;
    float requested_angle_deg;
    float requested_speed_deg_s;
    float remaining_angle_deg;
    int64_t last_transport_us;
    char runner[16];
    char tracker[16];
    char error[64];
} n3_motion_state_t;


static n3_motion_state_t s_motion;


static void copy_text(
    char *destination,
    size_t destination_size,
    const char *source)
{
    if ((destination == NULL) || (destination_size == 0U))
    {
        return;
    }

    (void)snprintf(
        destination,
        destination_size,
        "%s",
        source != NULL ? source : "");
}


static void set_error_locked(
    const char *error)
{
    copy_text(s_motion.error, sizeof(s_motion.error), error);
}


static const char *low_level_error_text(
    chassis_motion_error_t error)
{
    switch (error)
    {
        case CHASSIS_MOTION_ERROR_MOTOR_NOT_READY:
            return "motor is not ready";
        case CHASSIS_MOTION_ERROR_MPU_NOT_READY:
            return "MPU6050 is not ready";
        case CHASSIS_MOTION_ERROR_ODOMETRY_NOT_READY:
            return "odometry is not ready";
        case CHASSIS_MOTION_ERROR_ODOMETRY_STALE:
            return "odometry data is stale";
        case CHASSIS_MOTION_ERROR_TIMEOUT:
            return "rotation controller timeout";
        case CHASSIS_MOTION_ERROR_INVALID_ARGUMENT:
            return "rotation argument is invalid";
        case CHASSIS_MOTION_ERROR_NOT_INITIALIZED:
            return "motion controller is not initialized";
        case CHASSIS_MOTION_ERROR_NONE:
        default:
            return "rotation controller error";
    }
}


static void update_low_level_status(void)
{
#if N3_ENABLE_ROTATE_REL
    chassis_motion_status_t low = {0};
    chassis_motion_get_status(&low);

    if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    s_motion.remaining_angle_deg = low.remaining_angle_deg;
    s_motion.command_id = low.command_id;

    if (s_motion.active && !low.busy)
    {
        s_motion.active = false;
        if (s_motion.stop_requested)
        {
            copy_text(s_motion.runner, sizeof(s_motion.runner), "STOPPED");
        }
        else if (s_motion.safety_latched)
        {
            copy_text(s_motion.runner, sizeof(s_motion.runner), "ESTOPPED");
        }
        else if (low.mode == CHASSIS_MOTION_MODE_DONE && low.target_reached)
        {
            s_motion.target_reached = true;
            copy_text(s_motion.runner, sizeof(s_motion.runner), "FINISHED");
        }
        else if (low.mode == CHASSIS_MOTION_MODE_ERROR)
        {
            copy_text(s_motion.runner, sizeof(s_motion.runner), "ERROR");
            set_error_locked(low_level_error_text(low.error));
        }
        else
        {
            copy_text(s_motion.runner, sizeof(s_motion.runner), "STOPPED");
        }
    }

    if (low.error != CHASSIS_MOTION_ERROR_NONE && !s_motion.active)
    {
        set_error_locked(low_level_error_text(low.error));
    }

    xSemaphoreGive(s_motion.mutex);
#endif
}


static void motion_supervisor_task(
    void *arg)
{
    (void)arg;

#if N3_ENABLE_ROTATE_REL
    for (;;)
    {
        n3_hardware_status_t hardware = {0};
        n3_hardware_get_status(&hardware);

        bool ready = false;
        bool active = false;
        bool safety_latched = false;
        int64_t last_transport_us = 0;
        if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
        {
            ready = s_motion.ready;
            active = s_motion.active;
            safety_latched = s_motion.safety_latched;
            last_transport_us = s_motion.last_transport_us;
            xSemaphoreGive(s_motion.mutex);
        }

        if (!ready && hardware.motor_ready && hardware.odom_ready && !hardware.estop)
        {
            const esp_err_t ret = chassis_motion_init();
            if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
            {
                if (ret == ESP_OK)
                {
                    s_motion.ready = true;
                    copy_text(s_motion.tracker, sizeof(s_motion.tracker), "ROTATE_REL");
                    s_motion.error[0] = '\0';
                    ESP_LOGI(TAG, "guarded ROTATE_REL controller ready");
                }
                else
                {
                    set_error_locked("motion controller initialization failed");
                }
                xSemaphoreGive(s_motion.mutex);
            }
        }

        if (active && !safety_latched &&
            ((esp_timer_get_time() - last_transport_us) >
             ((int64_t)N3_ROTATE_LINK_TIMEOUT_MS * 1000LL)))
        {
            ESP_LOGE(TAG, "transport heartbeat timeout; emergency stopping rotation");
            chassis_motion_emergency_stop();
            if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
            {
                s_motion.safety_latched = true;
                s_motion.stop_requested = true;
                set_error_locked("transport heartbeat timeout");
                xSemaphoreGive(s_motion.mutex);
            }
        }

        if (active && (!hardware.motor_ready || !hardware.odom_ready || hardware.estop))
        {
            ESP_LOGE(TAG, "hardware became unsafe during rotation; emergency stopping");
            chassis_motion_emergency_stop();
            if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
            {
                s_motion.safety_latched = true;
                s_motion.stop_requested = true;
                set_error_locked("hardware became unavailable during rotation");
                xSemaphoreGive(s_motion.mutex);
            }
        }

        update_low_level_status();
        vTaskDelay(pdMS_TO_TICKS(N3_MOTION_SUPERVISOR_PERIOD_MS));
    }
#else
    vTaskDelete(NULL);
#endif
}


esp_err_t n3_motion_init(void)
{
    memset(&s_motion, 0, sizeof(s_motion));
    s_motion.mutex = xSemaphoreCreateMutex();
    if (s_motion.mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_motion.enabled = (N3_ENABLE_ROTATE_REL != 0);
    copy_text(s_motion.runner, sizeof(s_motion.runner), "IDLE");
    copy_text(
        s_motion.tracker,
        sizeof(s_motion.tracker),
        N3_ENABLE_ROTATE_REL ? "STARTING" : "IDLE");
    s_motion.last_transport_us = esp_timer_get_time();

#if N3_ENABLE_ROTATE_REL
    if (xTaskCreate(
            motion_supervisor_task,
            "n3_motion_sup",
            N3_MOTION_TASK_STACK_SIZE,
            NULL,
            N3_MOTION_SUPERVISOR_PRIORITY,
            NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
#endif

    return ESP_OK;
}


esp_err_t n3_motion_rotate(
    float angle_deg,
    float speed_deg_s,
    const char **error_code,
    const char **error_message)
{
    if ((error_code == NULL) || (error_message == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *error_code = "INTERNAL";
    *error_message = "motion request failed";

#if !N3_ENABLE_ROTATE_REL
    (void)angle_deg;
    (void)speed_deg_s;
    *error_code = "MOTION_DISABLED";
    *error_message = "this firmware profile does not enable ROTATE_REL";
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!isfinite(angle_deg) || !isfinite(speed_deg_s) ||
        fabsf(angle_deg) < 0.1f || fabsf(angle_deg) > N3_ROTATE_MAX_ANGLE_DEG ||
        speed_deg_s < N3_ROTATE_MIN_SPEED_DEG_S ||
        speed_deg_s > N3_ROTATE_MAX_SPEED_DEG_S)
    {
        *error_code = "ROTATE_LIMIT";
        *error_message = "ROTATE_REL is limited to +/-30 deg and 10..45 deg/s";
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_ERR_NO_MEM;
    }

    const bool ready = s_motion.ready;
    const bool active = s_motion.active;
    const bool safety_latched = s_motion.safety_latched;
    xSemaphoreGive(s_motion.mutex);

    if (safety_latched)
    {
        *error_code = "ESTOP_ACTIVE";
        *error_message = "clear emergency stop before rotating";
        return ESP_ERR_INVALID_STATE;
    }
    if (!ready)
    {
        *error_code = "MOTOR_NOT_READY";
        *error_message = "motor, MPU6050, and odometry are not ready";
        return ESP_ERR_INVALID_STATE;
    }
    if (active || chassis_motion_is_busy())
    {
        *error_code = "RUNNER_BUSY";
        *error_message = "another rotation is active";
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t ret = chassis_motion_rotate(angle_deg, speed_deg_s);
    if (ret != ESP_OK)
    {
        *error_code = "MOTION_ERROR";
        *error_message = "low-level chassis motion rejected ROTATE_REL";
        return ret;
    }

    if (xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
    {
        s_motion.active = chassis_motion_is_busy();
        s_motion.target_reached = !s_motion.active;
        s_motion.stop_requested = false;
        s_motion.requested_angle_deg = angle_deg;
        s_motion.requested_speed_deg_s = speed_deg_s;
        s_motion.last_transport_us = esp_timer_get_time();
        s_motion.error[0] = '\0';
        copy_text(
            s_motion.runner,
            sizeof(s_motion.runner),
            s_motion.active ? "ROTATING" : "FINISHED");
        copy_text(s_motion.tracker, sizeof(s_motion.tracker), "ROTATE_REL");
        xSemaphoreGive(s_motion.mutex);
    }

    *error_code = NULL;
    *error_message = NULL;
    return ESP_OK;
#endif
}


void n3_motion_stop(
    bool emergency)
{
#if N3_ENABLE_ROTATE_REL
    bool active = false;
    if (s_motion.mutex != NULL &&
        xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
    {
        active = s_motion.active;
        s_motion.stop_requested = true;
        if (emergency)
        {
            s_motion.safety_latched = true;
        }
        xSemaphoreGive(s_motion.mutex);
    }

    if (active || chassis_motion_is_busy())
    {
        if (emergency)
        {
            chassis_motion_emergency_stop();
        }
        else
        {
            chassis_motion_stop();
        }
    }
#else
    (void)emergency;
#endif
}


void n3_motion_clear_estop(void)
{
#if N3_ENABLE_ROTATE_REL
    if (s_motion.mutex != NULL &&
        xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
    {
        s_motion.safety_latched = false;
        s_motion.active = false;
        s_motion.stop_requested = false;
        s_motion.target_reached = false;
        s_motion.error[0] = '\0';
        copy_text(s_motion.runner, sizeof(s_motion.runner), "IDLE");
        xSemaphoreGive(s_motion.mutex);
    }
#endif
}


void n3_motion_note_transport_activity(void)
{
    if (s_motion.mutex != NULL &&
        xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
    {
        s_motion.last_transport_us = esp_timer_get_time();
        xSemaphoreGive(s_motion.mutex);
    }
}


bool n3_motion_is_active(void)
{
    bool active = false;
    if (s_motion.mutex != NULL &&
        xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
    {
        active = s_motion.active;
        xSemaphoreGive(s_motion.mutex);
    }
    return active;
}


void n3_motion_get_status(
    n3_motion_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->enabled = (N3_ENABLE_ROTATE_REL != 0);
    if (s_motion.mutex != NULL &&
        xSemaphoreTake(s_motion.mutex, portMAX_DELAY) == pdTRUE)
    {
        status->enabled = s_motion.enabled;
        status->ready = s_motion.ready;
        status->active = s_motion.active;
        status->target_reached = s_motion.target_reached;
        status->stop_requested = s_motion.stop_requested;
        status->safety_latched = s_motion.safety_latched;
        status->command_id = s_motion.command_id;
        status->requested_angle_deg = s_motion.requested_angle_deg;
        status->requested_speed_deg_s = s_motion.requested_speed_deg_s;
        status->remaining_angle_deg = s_motion.remaining_angle_deg;
        copy_text(status->runner, sizeof(status->runner), s_motion.runner);
        copy_text(status->tracker, sizeof(status->tracker), s_motion.tracker);
        copy_text(status->error, sizeof(status->error), s_motion.error);
        xSemaphoreGive(s_motion.mutex);
    }
}
