#include "n3_hardware.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "n3_build_config.h"

#if N3_ENABLE_HARDWARE
#include "chassis_odometry.h"
#include "motor_control.h"
#include "mpu6050.h"
#endif


#ifndef N3_HARDWARE_TASK_STACK_SIZE
#define N3_HARDWARE_TASK_STACK_SIZE (12U * 1024U)
#endif

#ifndef N3_HARDWARE_TASK_PRIORITY
#define N3_HARDWARE_TASK_PRIORITY 5
#endif

#ifndef N3_HARDWARE_READY_TIMEOUT_MS
#define N3_HARDWARE_READY_TIMEOUT_MS 5000U
#endif

#ifndef N3_HARDWARE_MPU_INIT_ATTEMPTS
#define N3_HARDWARE_MPU_INIT_ATTEMPTS 2U
#endif


typedef struct
{
    SemaphoreHandle_t mutex;
    bool initializing;
    bool motor_ready;
    bool odom_ready;
    char error[64];
} n3_hardware_context_t;


static n3_hardware_context_t s_hardware;


static void copy_text(
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


#if N3_ENABLE_HARDWARE
static void set_failure(
    const char *message)
{
    if (s_hardware.mutex == NULL ||
        xSemaphoreTake(s_hardware.mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    s_hardware.initializing = false;
    s_hardware.motor_ready = false;
    s_hardware.odom_ready = false;
    copy_text(s_hardware.error, sizeof(s_hardware.error), message);
    xSemaphoreGive(s_hardware.mutex);
}


static void set_failure_with_error(
    const char *operation,
    esp_err_t error)
{
    char message[sizeof(s_hardware.error)] = {0};
    (void)snprintf(
        message,
        sizeof(message),
        "%s: %s",
        operation,
        esp_err_to_name(error));
    set_failure(message);
}
#endif


#if N3_ENABLE_HARDWARE
static void hardware_init_task(
    void *argument)
{
    (void)argument;

    esp_err_t ret = motor_control_init();
    if (ret != ESP_OK || !motor_control_is_ready())
    {
        motor_emergency_stop();
        if (ret != ESP_OK)
        {
            set_failure_with_error("motor/encoder init failed", ret);
        }
        else
        {
            set_failure("motor/encoder closed loop is not ready");
        }
        vTaskDelete(NULL);
        return;
    }

    /*
     * The passive odometry test deliberately powers the motor driver down
     * before MPU6050 startup calibration.  Keep the same invariant here:
     * encoder sampling stays alive, but the H-bridge cannot introduce
     * vibration or electrical noise while gyro bias is measured.
     */
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(50U));
    motor_control_enable(false);
    vTaskDelay(pdMS_TO_TICKS(100U));

    for (uint32_t attempt = 1U;
         attempt <= N3_HARDWARE_MPU_INIT_ATTEMPTS;
         ++attempt)
    {
        ret = mpu6050_init();
        if (ret == ESP_OK)
        {
            break;
        }

        if ((ret != ESP_ERR_INVALID_STATE) ||
            (attempt == N3_HARDWARE_MPU_INIT_ATTEMPTS))
        {
            break;
        }

        /* A not-still/not-level calibration is recoverable after settling. */
        vTaskDelay(pdMS_TO_TICKS(500U));
    }

    if (ret != ESP_OK)
    {
        motor_emergency_stop();
        set_failure_with_error("MPU6050 init failed", ret);
        vTaskDelete(NULL);
        return;
    }

    const TickType_t imu_deadline =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(N3_HARDWARE_READY_TIMEOUT_MS);
    for (;;)
    {
        mpu6050_status_t imu = {0};
        mpu6050_get_status(&imu);
        if (imu.initialized && imu.sampling)
        {
            break;
        }
        if (xTaskGetTickCount() >= imu_deadline)
        {
            motor_emergency_stop();
            set_failure_with_error(
                "MPU6050 ready timeout",
                ESP_ERR_TIMEOUT);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }

    ret = chassis_odometry_init();
    if (ret != ESP_OK)
    {
        motor_emergency_stop();
        set_failure_with_error("odometry init failed", ret);
        vTaskDelete(NULL);
        return;
    }

    const TickType_t deadline =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(N3_HARDWARE_READY_TIMEOUT_MS);
    while (!chassis_odometry_is_ready())
    {
        if (xTaskGetTickCount() >= deadline)
        {
            motor_emergency_stop();
            set_failure_with_error(
                "odometry ready timeout",
                ESP_ERR_TIMEOUT);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }

    /* Restore a cleared, zero-command motor state after quiet calibration. */
    motor_control_enable(true);
    motor_stop();
    if (s_hardware.mutex != NULL &&
        xSemaphoreTake(s_hardware.mutex, portMAX_DELAY) == pdTRUE)
    {
        s_hardware.initializing = false;
        s_hardware.motor_ready = true;
        s_hardware.odom_ready = true;
        s_hardware.error[0] = '\0';
        xSemaphoreGive(s_hardware.mutex);
    }

    vTaskDelete(NULL);
}
#endif


esp_err_t n3_hardware_init(void)
{
    if (s_hardware.mutex != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_hardware, 0, sizeof(s_hardware));
    s_hardware.mutex = xSemaphoreCreateMutex();
    if (s_hardware.mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

#if N3_ENABLE_HARDWARE
    s_hardware.initializing = true;
    if (xTaskCreate(
            hardware_init_task,
            "n3_hw_init",
            N3_HARDWARE_TASK_STACK_SIZE,
            NULL,
            N3_HARDWARE_TASK_PRIORITY,
            NULL) != pdPASS)
    {
        s_hardware.initializing = false;
        copy_text(s_hardware.error, sizeof(s_hardware.error), "hardware init task creation failed");
        return ESP_ERR_NO_MEM;
    }
#endif

    return ESP_OK;
}


void n3_hardware_get_status(
    n3_hardware_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    *status = (n3_hardware_status_t){0};
    status->enabled = (N3_ENABLE_HARDWARE != 0);

    if ((s_hardware.mutex != NULL) &&
        xSemaphoreTake(s_hardware.mutex, portMAX_DELAY) == pdTRUE)
    {
        status->initializing = s_hardware.initializing;
        status->motor_ready = s_hardware.motor_ready;
        status->odom_ready = s_hardware.odom_ready;
        copy_text(status->error, sizeof(status->error), s_hardware.error);
        xSemaphoreGive(s_hardware.mutex);
    }

#if N3_ENABLE_HARDWARE
    motor_status_t motor = {0};
    chassis_odometry_state_t odom = {0};
    motor_get_status(&motor);
    chassis_odometry_get_state(&odom);
    status->motor_ready = motor.initialized && motor.closed_loop_ready;
    status->odom_ready = odom.initialized && odom.running;
    status->pose_valid = status->odom_ready;
    status->estop = motor.emergency_stop;
    status->x_mm = odom.x_mm;
    status->y_mm = odom.y_mm;
    status->yaw_deg = odom.yaw_deg;
    status->vx_world_mm_s = odom.world_vx_mm_s;
    status->vy_world_mm_s = odom.world_vy_mm_s;
    status->w_rad_s = odom.gyro_w_rad_s;
#endif
}


void n3_hardware_stop(
    bool emergency)
{
#if N3_ENABLE_HARDWARE
    if (emergency)
    {
        motor_emergency_stop();
    }
    else
    {
        motor_stop();
    }
#else
    (void)emergency;
#endif
}


esp_err_t n3_hardware_clear_estop(void)
{
#if N3_ENABLE_HARDWARE
    if (!motor_control_is_ready())
    {
        return ESP_ERR_INVALID_STATE;
    }
    motor_control_enable(true);
#endif
    return ESP_OK;
}


esp_err_t n3_hardware_reset_pose(
    float x_mm,
    float y_mm,
    float yaw_deg)
{
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(yaw_deg))
    {
        return ESP_ERR_INVALID_ARG;
    }

#if N3_ENABLE_HARDWARE
    n3_hardware_status_t status = {0};
    n3_hardware_get_status(&status);
    if (!status.motor_ready || !status.odom_ready)
    {
        return ESP_ERR_INVALID_STATE;
    }
    return chassis_odometry_reset(x_mm, y_mm, yaw_deg);
#else
    return ESP_OK;
#endif
}
