#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*
 * ============================================================
 * HC-SR04 超声波测距模块
 *
 * 职责：
 *   1) 周期触发 + 测量回波时间，换算距离
 *   2) 中值滤波，得到稳定距离值
 *   3) 判断是否进入障碍物检测范围
 *
 * 不负责：
 *   - 避障状态机（由 obstacle_avoid.h 完成）
 *   - 直接控制电机
 *
 * 用法（与 infrared_sensor.h 相同的单文件模式）：
 *   #define ULTRASONIC_SENSOR_IMPLEMENTATION
 *   #include "ultrasonic_sensor.h"
 * 只允许在一个 .c 文件（推荐 main.c）中这样 include。
 * ============================================================
 */

/* 引脚定义，接线不同可在 include 前用 #define 覆盖 */
#ifndef ULTRASONIC_TRIG_GPIO
#define ULTRASONIC_TRIG_GPIO        GPIO_NUM_13
#endif

#ifndef ULTRASONIC_ECHO_GPIO
#define ULTRASONIC_ECHO_GPIO        GPIO_NUM_1
#endif

/* 与电机引脚做编译期冲突检查（依赖 motor_control.h 先被 include） */
#ifdef MOTOR_A_IN1
_Static_assert(ULTRASONIC_TRIG_GPIO != MOTOR_A_IN1 && ULTRASONIC_TRIG_GPIO != MOTOR_A_IN2 && ULTRASONIC_TRIG_GPIO != MOTOR_A_PWM &&
               ULTRASONIC_TRIG_GPIO != MOTOR_B_IN1 && ULTRASONIC_TRIG_GPIO != MOTOR_B_IN2 && ULTRASONIC_TRIG_GPIO != MOTOR_B_PWM &&
               ULTRASONIC_TRIG_GPIO != MOTOR_D_IN1 && ULTRASONIC_TRIG_GPIO != MOTOR_D_IN2 && ULTRASONIC_TRIG_GPIO != MOTOR_D_PWM,
               "ULTRASONIC_TRIG_GPIO 与电机引脚冲突");
_Static_assert(ULTRASONIC_ECHO_GPIO != MOTOR_A_IN1 && ULTRASONIC_ECHO_GPIO != MOTOR_A_IN2 && ULTRASONIC_ECHO_GPIO != MOTOR_A_PWM &&
               ULTRASONIC_ECHO_GPIO != MOTOR_B_IN1 && ULTRASONIC_ECHO_GPIO != MOTOR_B_IN2 && ULTRASONIC_ECHO_GPIO != MOTOR_B_PWM &&
               ULTRASONIC_ECHO_GPIO != MOTOR_D_IN1 && ULTRASONIC_ECHO_GPIO != MOTOR_D_IN2 && ULTRASONIC_ECHO_GPIO != MOTOR_D_PWM,
               "ULTRASONIC_ECHO_GPIO 与电机引脚冲突");
#endif

/* 测距与判定参数 */
#ifndef ULTRASONIC_SAMPLE_PERIOD_MS
#define ULTRASONIC_SAMPLE_PERIOD_MS     60U
#endif
#ifndef ULTRASONIC_ECHO_TIMEOUT_US
#define ULTRASONIC_ECHO_TIMEOUT_US      30000
#endif
#ifndef ULTRASONIC_MIN_DISTANCE_CM
#define ULTRASONIC_MIN_DISTANCE_CM      2.0f
#endif
#ifndef ULTRASONIC_MAX_DISTANCE_CM
#define ULTRASONIC_MAX_DISTANCE_CM      400.0f
#endif
/* 障碍物判定距离 */
#ifndef ULTRASONIC_OBSTACLE_CM
#define ULTRASONIC_OBSTACLE_CM          10.0f
#endif
/* 中值滤波窗口 */
#ifndef ULTRASONIC_FILTER_SIZE
#define ULTRASONIC_FILTER_SIZE          3
#endif

/* 内部采集参数；已有配置宏和对外API保持不变。 */
#ifndef ULTRASONIC_STARTUP_SETTLE_MS
#define ULTRASONIC_STARTUP_SETTLE_MS    100U
#endif
#ifndef ULTRASONIC_STARTUP_DISCARD_COUNT
#define ULTRASONIC_STARTUP_DISCARD_COUNT 3U
#endif
#ifndef ULTRASONIC_INVALID_RESET_COUNT
#define ULTRASONIC_INVALID_RESET_COUNT  3U
#endif
#ifndef ULTRASONIC_RMT_RESOLUTION_HZ
#define ULTRASONIC_RMT_RESOLUTION_HZ    1000000U
#endif
#ifndef ULTRASONIC_RMT_SYMBOL_COUNT
#define ULTRASONIC_RMT_SYMBOL_COUNT     64U
#endif
#ifndef ULTRASONIC_RMT_GLITCH_NS
#define ULTRASONIC_RMT_GLITCH_NS        1000U
#endif
#ifndef ULTRASONIC_RMT_SIGNAL_MAX_NS
#define ULTRASONIC_RMT_SIGNAL_MAX_NS    (ULTRASONIC_ECHO_TIMEOUT_US * 1000U)
#endif
#ifndef ULTRASONIC_ERROR_LOG_PERIOD_MS
#define ULTRASONIC_ERROR_LOG_PERIOD_MS  1000U
#endif

_Static_assert(ULTRASONIC_TRIG_GPIO != ULTRASONIC_ECHO_GPIO,
               "ULTRASONIC_TRIG_GPIO 与 ULTRASONIC_ECHO_GPIO 不能相同");
_Static_assert(ULTRASONIC_FILTER_SIZE > 0 &&
               (ULTRASONIC_FILTER_SIZE % 2) == 1,
               "ULTRASONIC_FILTER_SIZE 必须是正奇数");
_Static_assert(ULTRASONIC_RMT_SYMBOL_COUNT > 0 &&
               (ULTRASONIC_RMT_SYMBOL_COUNT % 2) == 0,
               "ULTRASONIC_RMT_SYMBOL_COUNT 必须是正偶数");
_Static_assert(ULTRASONIC_RMT_RESOLUTION_HZ > 0,
               "ULTRASONIC_RMT_RESOLUTION_HZ 必须大于0");

#ifndef ULTRASONIC_TASK_STACK_SIZE
#define ULTRASONIC_TASK_STACK_SIZE      4096
#endif
#ifndef ULTRASONIC_TASK_PRIORITY
#define ULTRASONIC_TASK_PRIORITY        5
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float distance_cm;   /* 滤波后的距离，单位 cm */
    bool valid;           /* 本次测量是否有效 */
    bool obstacle;        /* 是否进入障碍物检测范围 */
} ultrasonic_data_t;

/* 初始化 GPIO */
esp_err_t ultrasonic_sensor_init(void);

/* 启动周期测量任务 */
esp_err_t ultrasonic_sensor_start(void);

/* 获取最近一次滤波后的测量结果 */
ultrasonic_data_t ultrasonic_sensor_get_data(void);

#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */
#ifdef ULTRASONIC_SENSOR_IMPLEMENTATION

#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_log.h"

#define ULTRASONIC_SAFE_MIN_PERIOD_MS   60U
#define ULTRASONIC_CAPTURE_WAIT_EXTRA_MS 10U

static const char *ULTRASONIC_TAG = "ULTRASONIC";

static ultrasonic_data_t g_ultrasonic_data = {0.0f, false, false};
static portMUX_TYPE g_ultrasonic_lock = portMUX_INITIALIZER_UNLOCKED;
static bool g_ultrasonic_initialized = false;
static bool g_ultrasonic_started = false;
static rmt_channel_handle_t g_ultrasonic_rmt_rx = NULL;
static bool g_ultrasonic_rmt_enabled = false;
static TaskHandle_t g_ultrasonic_task_handle = NULL;
static volatile size_t g_ultrasonic_rmt_symbol_count = 0U;
static rmt_symbol_word_t g_ultrasonic_rmt_symbols[ULTRASONIC_RMT_SYMBOL_COUNT];
static uint32_t g_ultrasonic_measure_sequence = 0U;

typedef enum {
    ULTRASONIC_MEASURE_OK = 0,
    ULTRASONIC_MEASURE_ECHO_STUCK_HIGH,
    ULTRASONIC_MEASURE_CAPTURE_TIMEOUT,
    ULTRASONIC_MEASURE_NO_HIGH_PULSE,
    ULTRASONIC_MEASURE_GLITCH,
    ULTRASONIC_MEASURE_OUT_OF_RANGE,
    ULTRASONIC_MEASURE_RMT_ERROR,
} ultrasonic_measure_status_t;

static const char *ultrasonic_measure_status_name(
    ultrasonic_measure_status_t status)
{
    switch (status) {
        case ULTRASONIC_MEASURE_OK:
            return "OK";
        case ULTRASONIC_MEASURE_ECHO_STUCK_HIGH:
            return "ECHO_STUCK_HIGH";
        case ULTRASONIC_MEASURE_CAPTURE_TIMEOUT:
            return "CAPTURE_TIMEOUT";
        case ULTRASONIC_MEASURE_NO_HIGH_PULSE:
            return "NO_HIGH_PULSE";
        case ULTRASONIC_MEASURE_GLITCH:
            return "GLITCH";
        case ULTRASONIC_MEASURE_OUT_OF_RANGE:
            return "OUT_OF_RANGE";
        case ULTRASONIC_MEASURE_RMT_ERROR:
            return "RMT_ERROR";
        default:
            return "UNKNOWN";
    }
}

/* RMT RX回调运行在ISR环境，只保存数量并唤醒测量任务。 */
static bool IRAM_ATTR ultrasonic_rmt_rx_done_callback(
    rmt_channel_handle_t rx_chan,
    const rmt_rx_done_event_data_t *edata,
    void *user_ctx)
{
    (void)rx_chan;

    if (edata != NULL) {
        g_ultrasonic_rmt_symbol_count = edata->num_symbols;
    } else {
        g_ultrasonic_rmt_symbol_count = 0U;
    }

    TaskHandle_t *task_handle = (TaskHandle_t *)user_ctx;
    if ((task_handle == NULL) || (*task_handle == NULL)) {
        return false;
    }

    BaseType_t high_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(*task_handle, &high_task_woken);
    return high_task_woken == pdTRUE;
}

/* 发送触发脉冲 */
static void ultrasonic_trigger(void)
{
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
}

/* 从RMT符号中提取第一个合理的高电平脉宽。 */
static bool ultrasonic_extract_high_ticks(
    const rmt_symbol_word_t *symbols,
    size_t symbol_count,
    uint32_t *high_ticks,
    ultrasonic_measure_status_t *status)
{
    bool found_high = false;
    uint32_t selected_ticks = 0U;
    const uint32_t glitch_ticks =
        (uint32_t)(((uint64_t)ULTRASONIC_RMT_GLITCH_NS *
                    ULTRASONIC_RMT_RESOLUTION_HZ +
                    999999999ULL) /
                   1000000000ULL);

    for (size_t i = 0U; i < symbol_count; i++) {
        const uint32_t duration[2] = {
            symbols[i].duration0,
            symbols[i].duration1,
        };
        const uint32_t level[2] = {
            symbols[i].level0,
            symbols[i].level1,
        };

        for (size_t part = 0U; part < 2U; part++) {
            if ((level[part] != 1U) ||
                (duration[part] < glitch_ticks)) {
                continue;
            }

            if (found_high) {
                if (status != NULL) {
                    *status = ULTRASONIC_MEASURE_GLITCH;
                }
                return false;
            }

            found_high = true;
            selected_ticks = duration[part];
        }
    }

    if (!found_high) {
        if (status != NULL) {
            *status = ULTRASONIC_MEASURE_NO_HIGH_PULSE;
        }
        return false;
    }

    if (high_ticks != NULL) {
        *high_ticks = selected_ticks;
    }
    return true;
}

/* 丢弃任务通知，避免上一轮迟到回调污染当前测量。 */
static void ultrasonic_drain_notifications(void)
{
    while (ulTaskNotifyTake(pdTRUE, 0U) > 0U) {
    }
}

/* 读取一次原始距离，使用RMT硬件捕获ECHO高电平宽度。 */
static float ultrasonic_measure_raw(ultrasonic_measure_status_t *status)
{
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = ULTRASONIC_RMT_GLITCH_NS,
        .signal_range_max_ns = ULTRASONIC_RMT_SIGNAL_MAX_NS,
        .flags.en_partial_rx = false,
    };
    const TickType_t capture_wait = pdMS_TO_TICKS(
        (ULTRASONIC_RMT_SIGNAL_MAX_NS / 1000000U) +
        ULTRASONIC_CAPTURE_WAIT_EXTRA_MS);

    if (status != NULL) {
        *status = ULTRASONIC_MEASURE_OK;
    }

    if ((g_ultrasonic_rmt_rx == NULL) ||
        !g_ultrasonic_rmt_enabled ||
        (g_ultrasonic_task_handle == NULL)) {
        if (status != NULL) {
            *status = ULTRASONIC_MEASURE_RMT_ERROR;
        }
        return -1.0f;
    }

    /* ECHO在启动接收前已经为高，不能把残留电平当成本轮回波。 */
    if (gpio_get_level(ULTRASONIC_ECHO_GPIO) != 0) {
        if (status != NULL) {
            *status = ULTRASONIC_MEASURE_ECHO_STUCK_HIGH;
        }
        return -1.0f;
    }

    ultrasonic_drain_notifications();
    g_ultrasonic_rmt_symbol_count = 0U;

    esp_err_t ret = rmt_receive(
        g_ultrasonic_rmt_rx,
        g_ultrasonic_rmt_symbols,
        sizeof(g_ultrasonic_rmt_symbols),
        &receive_config);
    if (ret != ESP_OK) {
        if (status != NULL) {
            *status = ULTRASONIC_MEASURE_RMT_ERROR;
        }
        return -1.0f;
    }

    ultrasonic_trigger();

    if (ulTaskNotifyTake(pdTRUE, capture_wait) == 0U) {
        if (status != NULL) {
            *status = ULTRASONIC_MEASURE_CAPTURE_TIMEOUT;
        }
        return -1.0f;
    }

    uint32_t high_ticks = 0U;
    if (!ultrasonic_extract_high_ticks(
            g_ultrasonic_rmt_symbols,
            g_ultrasonic_rmt_symbol_count,
            &high_ticks,
            status)) {
        return -1.0f;
    }

    const float pulse_us =
        ((float)high_ticks * 1000000.0f) /
        (float)ULTRASONIC_RMT_RESOLUTION_HZ;

    /* 声速约343m/s，距离 = 时间 × 声速 / 2。 */
    const float distance_cm = pulse_us * 0.0343f / 2.0f;

    if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM ||
        distance_cm > ULTRASONIC_MAX_DISTANCE_CM) {
        if (status != NULL) {
            *status = ULTRASONIC_MEASURE_OUT_OF_RANGE;
        }
        return -1.0f;
    }

    return distance_cm;
}

/* 冒泡排序取中值 */
static float ultrasonic_median_filter(float *data, size_t size)
{
    for (size_t i = 0; i + 1 < size; i++) {
        for (size_t j = i + 1; j < size; j++) {
            if (data[j] < data[i]) {
                float tmp = data[i];
                data[i] = data[j];
                data[j] = tmp;
            }
        }
    }
    return data[size / 2];
}

static void ultrasonic_log_measurement_failure(
    ultrasonic_measure_status_t status,
    uint32_t invalid_streak)
{
    static ultrasonic_measure_status_t last_status = ULTRASONIC_MEASURE_OK;
    static int64_t last_log_us = 0;
    const int64_t now_us = esp_timer_get_time();
    const int64_t log_period_us =
        (int64_t)ULTRASONIC_ERROR_LOG_PERIOD_MS * 1000LL;

    if ((invalid_streak == 1U) ||
        (status != last_status) ||
        ((now_us - last_log_us) >= log_period_us)) {
        ESP_LOGW(
            ULTRASONIC_TAG,
            "measurement failed: status=%s streak=%lu seq=%lu echo=%d",
            ultrasonic_measure_status_name(status),
            (unsigned long)invalid_streak,
            (unsigned long)g_ultrasonic_measure_sequence,
            gpio_get_level(ULTRASONIC_ECHO_GPIO));
        last_status = status;
        last_log_us = now_us;
    }
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    float buffer[ULTRASONIC_FILTER_SIZE];
    size_t write_index = 0U;
    size_t fill_count = 0U;
    bool obstacle_state = false;
    uint32_t invalid_streak = 0U;
    uint32_t startup_discard_remaining = ULTRASONIC_STARTUP_DISCARD_COUNT;
    const uint32_t period_ms =
        (ULTRASONIC_SAMPLE_PERIOD_MS < ULTRASONIC_SAFE_MIN_PERIOD_MS)
            ? ULTRASONIC_SAFE_MIN_PERIOD_MS
            : ULTRASONIC_SAMPLE_PERIOD_MS;
    const TickType_t period = pdMS_TO_TICKS(period_ms);
    TickType_t last_wake;

    memset(buffer, 0, sizeof(buffer));
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_STARTUP_SETTLE_MS));
    last_wake = xTaskGetTickCount();

    while (1) {
        ultrasonic_measure_status_t measure_status;
        float distance = ultrasonic_measure_raw(&measure_status);
        g_ultrasonic_measure_sequence++;

        if (startup_discard_remaining > 0U) {
            if ((measure_status == ULTRASONIC_MEASURE_OK) &&
                (distance > 0.0f)) {
                startup_discard_remaining--;
            }

            if (measure_status != ULTRASONIC_MEASURE_OK) {
                ultrasonic_log_measurement_failure(
                    measure_status,
                    1U);
            }

            taskENTER_CRITICAL(&g_ultrasonic_lock);
            g_ultrasonic_data.valid = false;
            g_ultrasonic_data.obstacle = false;
            taskEXIT_CRITICAL(&g_ultrasonic_lock);

            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        if (measure_status == ULTRASONIC_MEASURE_OK && distance > 0.0f) {
            if (invalid_streak > 0U) {
                ESP_LOGI(
                    ULTRASONIC_TAG,
                    "measurement recovered after %lu invalid sample(s)",
                    (unsigned long)invalid_streak);
            }
            invalid_streak = 0U;

            buffer[write_index] = distance;
            write_index = (write_index + 1U) % ULTRASONIC_FILTER_SIZE;

            if (fill_count < ULTRASONIC_FILTER_SIZE) {
                fill_count++;
            }

            if (fill_count == ULTRASONIC_FILTER_SIZE) {
                float filter_buffer[ULTRASONIC_FILTER_SIZE];
                memcpy(filter_buffer, buffer, sizeof(filter_buffer));
                const float filtered = ultrasonic_median_filter(
                    filter_buffer,
                    ULTRASONIC_FILTER_SIZE);

                /* 进入和退出障碍区使用轻微滞回，减少阈值附近抖动。 */
                if (!obstacle_state &&
                    filtered <= ULTRASONIC_OBSTACLE_CM) {
                    obstacle_state = true;
                } else if (obstacle_state &&
                           filtered >= (ULTRASONIC_OBSTACLE_CM + 2.0f)) {
                    obstacle_state = false;
                }

                taskENTER_CRITICAL(&g_ultrasonic_lock);
                g_ultrasonic_data.distance_cm = filtered;
                g_ultrasonic_data.valid = true;
                g_ultrasonic_data.obstacle = obstacle_state;
                taskEXIT_CRITICAL(&g_ultrasonic_lock);
            }
        } else {
            if (invalid_streak < UINT32_MAX) {
                invalid_streak++;
            }

            ultrasonic_log_measurement_failure(
                measure_status,
                invalid_streak);

            /* 单次丢波不清空滤波窗口，连续异常才重新预热。 */
            if (invalid_streak >= ULTRASONIC_INVALID_RESET_COUNT) {
                write_index = 0U;
                fill_count = 0U;
            }

            taskENTER_CRITICAL(&g_ultrasonic_lock);
            g_ultrasonic_data.valid = false;
            /* invalid期间保留障碍状态，避免故障被误判为路径安全。 */
            g_ultrasonic_data.obstacle = obstacle_state;
            taskEXIT_CRITICAL(&g_ultrasonic_lock);
        }

        /* 以触发时刻为基准，保证两次触发之间至少约 60 ms。 */
        vTaskDelayUntil(&last_wake, period);
    }
}

esp_err_t ultrasonic_sensor_init(void)
{
    if (g_ultrasonic_initialized) return ESP_OK;

    gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&echo_config);
    if (ret != ESP_OK) {
        ESP_LOGE(ULTRASONIC_TAG, "ECHO GPIO init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_config_t trig_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&trig_config);
    if (ret != ESP_OK) {
        ESP_LOGE(ULTRASONIC_TAG, "TRIG GPIO init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);

    const rmt_rx_channel_config_t rmt_config = {
        .gpio_num = ULTRASONIC_ECHO_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = ULTRASONIC_RMT_RESOLUTION_HZ,
        .mem_block_symbols = ULTRASONIC_RMT_SYMBOL_COUNT,
        .intr_priority = 0,
        .flags.invert_in = false,
        .flags.with_dma = false,
        .flags.io_loop_back = false,
        .flags.allow_pd = false,
    };

    ret = rmt_new_rx_channel(&rmt_config, &g_ultrasonic_rmt_rx);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ULTRASONIC_TAG,
            "RMT RX channel create failed: %s",
            esp_err_to_name(ret));
        g_ultrasonic_rmt_rx = NULL;
        return ret;
    }

    const rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = ultrasonic_rmt_rx_done_callback,
    };

    ret = rmt_rx_register_event_callbacks(
        g_ultrasonic_rmt_rx,
        &callbacks,
        &g_ultrasonic_task_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ULTRASONIC_TAG,
            "RMT RX callback registration failed: %s",
            esp_err_to_name(ret));
        (void)rmt_del_channel(g_ultrasonic_rmt_rx);
        g_ultrasonic_rmt_rx = NULL;
        return ret;
    }

    ret = rmt_enable(g_ultrasonic_rmt_rx);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ULTRASONIC_TAG,
            "RMT RX enable failed: %s",
            esp_err_to_name(ret));
        (void)rmt_del_channel(g_ultrasonic_rmt_rx);
        g_ultrasonic_rmt_rx = NULL;
        return ret;
    }
    g_ultrasonic_rmt_enabled = true;

    g_ultrasonic_initialized = true;
    ESP_LOGI(ULTRASONIC_TAG,
             "Initialized: TRIG=%d ECHO=%d period=%ums obstacle_cm=%.1f capture=RMT/%luHz",
             (int)ULTRASONIC_TRIG_GPIO,
             (int)ULTRASONIC_ECHO_GPIO,
             (unsigned)((ULTRASONIC_SAMPLE_PERIOD_MS < ULTRASONIC_SAFE_MIN_PERIOD_MS)
                            ? ULTRASONIC_SAFE_MIN_PERIOD_MS
                            : ULTRASONIC_SAMPLE_PERIOD_MS),
             ULTRASONIC_OBSTACLE_CM,
             (unsigned long)ULTRASONIC_RMT_RESOLUTION_HZ);
    return ESP_OK;
}

esp_err_t ultrasonic_sensor_start(void)
{
    if (!g_ultrasonic_initialized) return ESP_ERR_INVALID_STATE;
    if (g_ultrasonic_started) return ESP_OK;

    BaseType_t ret = xTaskCreate(ultrasonic_task, "ultrasonic_task",
                                 ULTRASONIC_TASK_STACK_SIZE, NULL,
                                 ULTRASONIC_TASK_PRIORITY,
                                 &g_ultrasonic_task_handle);
    if (ret != pdPASS) return ESP_ERR_NO_MEM;

    g_ultrasonic_started = true;
    return ESP_OK;
}

ultrasonic_data_t ultrasonic_sensor_get_data(void)
{
    ultrasonic_data_t data;
    taskENTER_CRITICAL(&g_ultrasonic_lock);
    data = g_ultrasonic_data;
    taskEXIT_CRITICAL(&g_ultrasonic_lock);
    return data;
}

#endif /* ULTRASONIC_SENSOR_IMPLEMENTATION */
#endif /* ULTRASONIC_SENSOR_H */
