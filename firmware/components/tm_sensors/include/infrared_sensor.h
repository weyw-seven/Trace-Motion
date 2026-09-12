#ifndef INFRARED_SENSOR_H
#define INFRARED_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * ============================================================
 * 4-channel infrared line sensor hardware layer
 *
 * Responsibilities:
 *   1) configure 4 GPIO inputs
 *   2) sample raw electrical levels continuously
 *   3) convert raw levels to a 4-bit "black line" pattern
 *   4) expose the latest snapshot
 *
 * It does NOT:
 *   - calculate line error
 *   - run PD control
 *   - decide SEARCH/FOLLOW/LOST/FINISH
 *   - control motors
 * ============================================================
 */

/*
 * Current project wiring seen in your log is 11/12/13/14.
 *
 * IMPORTANT:
 * GPIO11 is ALSO shown as an output earlier in your boot log.
 * That is a real pin conflict. Change the actual wiring/pin macros
 * so that no IR pin overlaps any motor pin before final testing.
 *
 * These #ifndef guards let you override pins before including this file.
 */
#ifndef IR_SENSOR_L2_PIN
#define IR_SENSOR_L2_PIN        GPIO_NUM_21
#endif

#ifndef IR_SENSOR_L1_PIN
#define IR_SENSOR_L1_PIN        GPIO_NUM_20
#endif

#ifndef IR_SENSOR_R1_PIN
#define IR_SENSOR_R1_PIN        GPIO_NUM_19
#endif

#ifndef IR_SENSOR_R2_PIN
#define IR_SENSOR_R2_PIN        GPIO_NUM_13
#endif

/*
 * 1: GPIO high means black line
 * 0: GPIO low  means black line
 *
 * If RAW changes but PAT stays inverted, flip this value.
 */
#ifndef IR_SENSOR_ACTIVE_LEVEL
#define IR_SENSOR_ACTIVE_LEVEL  0
#endif

#ifndef IR_SENSOR_SAMPLE_PERIOD_MS
#define IR_SENSOR_SAMPLE_PERIOD_MS  10U
#endif

#ifndef IR_SENSOR_TASK_STACK_SIZE
#define IR_SENSOR_TASK_STACK_SIZE   2048
#endif

#ifndef IR_SENSOR_TASK_PRIORITY
#define IR_SENSOR_TASK_PRIORITY     7
#endif

#ifndef IR_SENSOR_DEBUG_ENABLE
#define IR_SENSOR_DEBUG_ENABLE      0
#endif

#ifndef IR_SENSOR_DEBUG_PERIOD_MS
#define IR_SENSOR_DEBUG_PERIOD_MS   200U
#endif

/*
 * Optional compile-time conflict checks.
 * These work when motor_control.h has already defined the motor pin macros.
 */
#ifdef MOTOR_A_IN1
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_A_IN1, "IR L2 conflicts with MOTOR_A_IN1");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_A_IN1, "IR L1 conflicts with MOTOR_A_IN1");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_A_IN1, "IR R1 conflicts with MOTOR_A_IN1");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_A_IN1, "IR R2 conflicts with MOTOR_A_IN1");
#endif
#ifdef MOTOR_A_IN2
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_A_IN2, "IR L2 conflicts with MOTOR_A_IN2");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_A_IN2, "IR L1 conflicts with MOTOR_A_IN2");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_A_IN2, "IR R1 conflicts with MOTOR_A_IN2");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_A_IN2, "IR R2 conflicts with MOTOR_A_IN2");
#endif
#ifdef MOTOR_A_PWM
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_A_PWM, "IR L2 conflicts with MOTOR_A_PWM");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_A_PWM, "IR L1 conflicts with MOTOR_A_PWM");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_A_PWM, "IR R1 conflicts with MOTOR_A_PWM");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_A_PWM, "IR R2 conflicts with MOTOR_A_PWM");
#endif
#ifdef MOTOR_B_IN1
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_B_IN1, "IR L2 conflicts with MOTOR_B_IN1");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_B_IN1, "IR L1 conflicts with MOTOR_B_IN1");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_B_IN1, "IR R1 conflicts with MOTOR_B_IN1");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_B_IN1, "IR R2 conflicts with MOTOR_B_IN1");
#endif
#ifdef MOTOR_B_IN2
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_B_IN2, "IR L2 conflicts with MOTOR_B_IN2");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_B_IN2, "IR L1 conflicts with MOTOR_B_IN2");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_B_IN2, "IR R1 conflicts with MOTOR_B_IN2");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_B_IN2, "IR R2 conflicts with MOTOR_B_IN2");
#endif
#ifdef MOTOR_B_PWM
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_B_PWM, "IR L2 conflicts with MOTOR_B_PWM");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_B_PWM, "IR L1 conflicts with MOTOR_B_PWM");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_B_PWM, "IR R1 conflicts with MOTOR_B_PWM");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_B_PWM, "IR R2 conflicts with MOTOR_B_PWM");
#endif
#ifdef MOTOR_D_IN1
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_D_IN1, "IR L2 conflicts with MOTOR_D_IN1");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_D_IN1, "IR L1 conflicts with MOTOR_D_IN1");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_D_IN1, "IR R1 conflicts with MOTOR_D_IN1");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_D_IN1, "IR R2 conflicts with MOTOR_D_IN1");
#endif
#ifdef MOTOR_D_IN2
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_D_IN2, "IR L2 conflicts with MOTOR_D_IN2");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_D_IN2, "IR L1 conflicts with MOTOR_D_IN2");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_D_IN2, "IR R1 conflicts with MOTOR_D_IN2");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_D_IN2, "IR R2 conflicts with MOTOR_D_IN2");
#endif
#ifdef MOTOR_D_PWM
_Static_assert(IR_SENSOR_L2_PIN != MOTOR_D_PWM, "IR L2 conflicts with MOTOR_D_PWM");
_Static_assert(IR_SENSOR_L1_PIN != MOTOR_D_PWM, "IR L1 conflicts with MOTOR_D_PWM");
_Static_assert(IR_SENSOR_R1_PIN != MOTOR_D_PWM, "IR R1 conflicts with MOTOR_D_PWM");
_Static_assert(IR_SENSOR_R2_PIN != MOTOR_D_PWM, "IR R2 conflicts with MOTOR_D_PWM");
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * bit3 bit2 bit1 bit0
 *  L2   L1   R1   R2
 *
 * raw_*   = actual GPIO electrical level
 * pattern = normalized "black-line detected" bits
 */
typedef struct
{
    uint8_t raw_l2;
    uint8_t raw_l1;
    uint8_t raw_r1;
    uint8_t raw_r2;

    uint8_t pattern;
    uint8_t active_count;

    uint32_t sample_count;
    TickType_t tick;
} infrared_sensor_data_t;

/* Configure GPIO and start the independent continuous sampling task. */
esp_err_t infrared_sensor_init(void);

/* Copy the latest sampled snapshot. Safe to call from another task. */
void infrared_sensor_get_data(infrared_sensor_data_t *data);

/*
 * Directly read GPIO once, bypassing the background snapshot.
 * Useful for bring-up/debug.
 */
uint8_t infrared_sensor_read_pattern_now(void);

/* Print one snapshot immediately. */
void infrared_sensor_log_once(void);

bool infrared_sensor_is_initialized(void);

#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 *
 * Define INFRARED_SENSOR_IMPLEMENTATION in exactly one C file
 * before including this header.
 * ============================================================ */
#ifdef INFRARED_SENSOR_IMPLEMENTATION

static const char *IR_TAG = "IR";

static infrared_sensor_data_t g_ir_data;
static TaskHandle_t g_ir_task_handle = NULL;
static bool g_ir_initialized = false;

static portMUX_TYPE g_ir_lock =
    portMUX_INITIALIZER_UNLOCKED;

static inline bool infrared_level_is_active(int level)
{
    return level == IR_SENSOR_ACTIVE_LEVEL;
}

static uint8_t infrared_build_pattern(
    uint8_t l2,
    uint8_t l1,
    uint8_t r1,
    uint8_t r2)
{
    uint8_t pattern = 0;

    if (infrared_level_is_active(l2)) pattern |= (1U << 3);
    if (infrared_level_is_active(l1)) pattern |= (1U << 2);
    if (infrared_level_is_active(r1)) pattern |= (1U << 1);
    if (infrared_level_is_active(r2)) pattern |= (1U << 0);

    return pattern;
}

static uint8_t infrared_count_active(uint8_t pattern)
{
    uint8_t count = 0;
    if (pattern & 0x08U) count++;
    if (pattern & 0x04U) count++;
    if (pattern & 0x02U) count++;
    if (pattern & 0x01U) count++;
    return count;
}

static infrared_sensor_data_t infrared_sample_gpio(void)
{
    infrared_sensor_data_t data = {0};

    data.raw_l2 = (uint8_t)gpio_get_level(IR_SENSOR_L2_PIN);
    data.raw_l1 = (uint8_t)gpio_get_level(IR_SENSOR_L1_PIN);
    data.raw_r1 = (uint8_t)gpio_get_level(IR_SENSOR_R1_PIN);
    data.raw_r2 = (uint8_t)gpio_get_level(IR_SENSOR_R2_PIN);

    data.pattern = infrared_build_pattern(
        data.raw_l2,
        data.raw_l1,
        data.raw_r1,
        data.raw_r2);

    data.active_count = infrared_count_active(data.pattern);
    data.tick = xTaskGetTickCount();

    return data;
}

static void infrared_publish_sample(infrared_sensor_data_t data)
{
    taskENTER_CRITICAL(&g_ir_lock);

    data.sample_count = g_ir_data.sample_count + 1U;
    g_ir_data = data;

    taskEXIT_CRITICAL(&g_ir_lock);
}

static void infrared_sensor_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(IR_SENSOR_SAMPLE_PERIOD_MS);

#if IR_SENSOR_DEBUG_ENABLE
    uint32_t debug_elapsed_ms = 0;
#endif

    while (1)
    {
        infrared_sensor_data_t data =
            infrared_sample_gpio();

        infrared_publish_sample(data);

#if IR_SENSOR_DEBUG_ENABLE
        debug_elapsed_ms += IR_SENSOR_SAMPLE_PERIOD_MS;

        if (debug_elapsed_ms >= IR_SENSOR_DEBUG_PERIOD_MS)
        {
            debug_elapsed_ms = 0;

            ESP_LOGI(
                IR_TAG,
                "RAW=%u%u%u%u PAT=%u%u%u%u COUNT=%u SAMPLE=%lu",
                data.raw_l2, data.raw_l1, data.raw_r1, data.raw_r2,
                (data.pattern >> 3) & 1U,
                (data.pattern >> 2) & 1U,
                (data.pattern >> 1) & 1U,
                (data.pattern >> 0) & 1U,
                data.active_count,
                (unsigned long)(g_ir_data.sample_count));
        }
#endif

        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t infrared_sensor_init(void)
{
    if (g_ir_initialized)
    {
        return ESP_OK;
    }

    gpio_config_t io = {0};

    io.pin_bit_mask =
        (1ULL << IR_SENSOR_L2_PIN) |
        (1ULL << IR_SENSOR_L1_PIN) |
        (1ULL << IR_SENSOR_R1_PIN) |
        (1ULL << IR_SENSOR_R2_PIN);

    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK)
    {
        return ret;
    }

    infrared_sensor_data_t first =
        infrared_sample_gpio();

    infrared_publish_sample(first);

    BaseType_t task_result =
        xTaskCreate(
            infrared_sensor_task,
            "infrared_sensor",
            IR_SENSOR_TASK_STACK_SIZE,
            NULL,
            IR_SENSOR_TASK_PRIORITY,
            &g_ir_task_handle);

    if (task_result != pdPASS)
    {
        g_ir_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    g_ir_initialized = true;

    ESP_LOGI(
        IR_TAG,
        "Initialized: L2=%d L1=%d R1=%d R2=%d active=%d",
        (int)IR_SENSOR_L2_PIN,
        (int)IR_SENSOR_L1_PIN,
        (int)IR_SENSOR_R1_PIN,
        (int)IR_SENSOR_R2_PIN,
        IR_SENSOR_ACTIVE_LEVEL);

    return ESP_OK;
}

void infrared_sensor_get_data(infrared_sensor_data_t *data)
{
    if (data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&g_ir_lock);
    *data = g_ir_data;
    taskEXIT_CRITICAL(&g_ir_lock);
}

uint8_t infrared_sensor_read_pattern_now(void)
{
    return infrared_sample_gpio().pattern;
}

void infrared_sensor_log_once(void)
{
    infrared_sensor_data_t data;
    infrared_sensor_get_data(&data);

    ESP_LOGI(
        IR_TAG,
        "RAW=%u%u%u%u PAT=%u%u%u%u COUNT=%u SAMPLE=%lu",
        data.raw_l2, data.raw_l1, data.raw_r1, data.raw_r2,
        (data.pattern >> 3) & 1U,
        (data.pattern >> 2) & 1U,
        (data.pattern >> 1) & 1U,
        (data.pattern >> 0) & 1U,
        data.active_count,
        (unsigned long)data.sample_count);
}

bool infrared_sensor_is_initialized(void)
{
    return g_ir_initialized;
}

#endif /* INFRARED_SENSOR_IMPLEMENTATION */
#endif /* INFRARED_SENSOR_H */
