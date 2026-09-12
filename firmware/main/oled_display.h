#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "obstacle_avoid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. OLED 硬件配置
 * OLED_M154_4P, PORT: IIC, Pixels: 128x64, DRIVER: SSD1306
 * SCL -> GPIO6, SDA -> GPIO7
 * ============================================================ */
#define OLED_I2C_PORT                  I2C_NUM_1
#define OLED_I2C_SCL_PIN               GPIO_NUM_6
#define OLED_I2C_SDA_PIN               GPIO_NUM_7
#define OLED_I2C_FREQ_HZ               400000
#define OLED_WIDTH                     128
#define OLED_HEIGHT                    64
#define OLED_PAGE_COUNT                8
#define OLED_ADDR_3C                   0x3C
#define OLED_ADDR_3D                   0x3D
#define OLED_UPDATE_PERIOD_MS          200U
#define OLED_TASK_STACK_SIZE           4096U
#define OLED_TASK_PRIORITY             2U
#define OLED_TAG                       "OLED"

/* ============================================================
 * 2. Public API
 * ============================================================ */
esp_err_t oled_display_init(void);
esp_err_t oled_display_start(void);
void oled_display_update(void);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * IMPLEMENTATION
 * 只能在一个 .c 文件中定义：
 * #define OLED_DISPLAY_IMPLEMENTATION
 * #include "oled_display.h"
 * ============================================================ */
#ifdef OLED_DISPLAY_IMPLEMENTATION

/* ============================================================
 * 3. 内部状态
 * ============================================================ */
static uint8_t g_oled_address = OLED_ADDR_3C;
static uint8_t g_oled_buffer[OLED_WIDTH * OLED_PAGE_COUNT];
static TaskHandle_t g_oled_task_handle = NULL;
static bool g_oled_initialized = false;
static i2c_master_bus_handle_t g_oled_bus = NULL;
static i2c_master_dev_handle_t g_oled_dev = NULL;

/* ============================================================
 * 4. 5x7 字符字体 (每个字符5字节，最低位在上)
 * ============================================================ */
static const uint8_t g_font_5x7[][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E}  /* 9 */
};

/* ============================================================
 * 5. SSD1306 底层 I2C
 * ============================================================ */
static esp_err_t oled_i2c_write(const uint8_t *data, size_t length) {
    if (g_oled_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit(
        g_oled_dev,
        data,
        length,
        100
    );
}

static esp_err_t oled_send_command(uint8_t command) {
    uint8_t data[2] = {0x00, command};
    return oled_i2c_write(data, sizeof(data));
}

static esp_err_t oled_send_commands(const uint8_t *commands, size_t length) {
    uint8_t data[32];
    if (length > sizeof(data) - 1) return ESP_ERR_INVALID_SIZE;
    data[0] = 0x00;
    memcpy(&data[1], commands, length);
    return oled_i2c_write(data, length + 1);
}

/* ============================================================
 * 6. OLED 地址检测
 * ============================================================ */
static esp_err_t oled_probe_address(uint8_t addr) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = OLED_I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t temp_dev = NULL;

    esp_err_t ret = i2c_master_bus_add_device(
        g_oled_bus,
        &dev_cfg,
        &temp_dev
    );

    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_probe(
        g_oled_bus,
        addr,
        50
    );

    i2c_master_bus_rm_device(temp_dev);

    return ret;
}


static esp_err_t oled_detect_address(void) {
    if (oled_probe_address(OLED_ADDR_3C) == ESP_OK) {
        g_oled_address = OLED_ADDR_3C;
        ESP_LOGI(OLED_TAG, "OLED found at 0x3C");
        return ESP_OK;
    }

    if (oled_probe_address(OLED_ADDR_3D) == ESP_OK) {
        g_oled_address = OLED_ADDR_3D;
        ESP_LOGI(OLED_TAG, "OLED found at 0x3D");
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

/* ============================================================
 * 7. SSD1306 初始化
 * ============================================================ */
static esp_err_t oled_ssd1306_init(void) {
    const uint8_t commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF
    };
    return oled_send_commands(commands, sizeof(commands));
}

/* ============================================================
 * 8. 显存操作
 * ============================================================ */
static void oled_clear(void) { memset(g_oled_buffer, 0, sizeof(g_oled_buffer)); }

static void oled_set_pixel(int x, int y, bool on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    const int page = y / 8;
    const uint8_t bit = 1U << (y % 8);
    const int index = page * OLED_WIDTH + x;
    if (on) g_oled_buffer[index] |= bit;
    else    g_oled_buffer[index] &= ~bit;
}

/* ============================================================
 * 9. 字符绘制
 * ============================================================ */
static void oled_draw_digit(int x, int y, char c) {
    if (c < '0' || c > '9') return;
    const uint8_t *glyph = g_font_5x7[c - '0'];
    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            oled_set_pixel(x + col, y + row, glyph[col] & (1U << row));
}

static void oled_draw_char(int x, int y, char c) {
    if (c >= '0' && c <= '9') { oled_draw_digit(x, y, c); return; }
    const uint8_t *glyph = NULL;
    static const uint8_t font_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t font_B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t font_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t font_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t font_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t font_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t font_N[5] = {0x7F, 0x02, 0x04, 0x08, 0x7F};
    static const uint8_t font_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t font_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t font_S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t font_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t font_V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
    static const uint8_t font_c[5] = {0x38, 0x44, 0x44, 0x44, 0x28};
    static const uint8_t font_m[5] = {0x7C, 0x04, 0x18, 0x04, 0x78};
    static const uint8_t font_plus[5] = {0x08, 0x08, 0x3E, 0x08, 0x08};
    static const uint8_t font_minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t font_dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    switch (c) {
        case 'A': glyph = font_A; break; case 'B': glyph = font_B; break; case 'C': glyph = font_C; break;
        case 'D': glyph = font_D; break; case 'I': glyph = font_I; break; case 'L': glyph = font_L; break;
        case 'N': glyph = font_N; break; case 'P': glyph = font_P; break; case 'R': glyph = font_R; break;
        case 'S': glyph = font_S; break; case 'T': glyph = font_T; break; case 'V': glyph = font_V; break;
        case 'c': glyph = font_c; break; case 'm': glyph = font_m; break; case '+': glyph = font_plus; break;
        case '-': glyph = font_minus; break; case '.': glyph = font_dot; break; case ':': glyph = font_colon; break;
        case ' ': default: return;
    }
    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            oled_set_pixel(x + col, y + row, glyph[col] & (1U << row));
}

static void oled_draw_string(int x, int y, const char *text) {
    if (text == NULL) return;
    while (*text != '\0') { oled_draw_char(x, y, *text); x += 6; text++; }
}

/* ============================================================
 * 10. OLED 刷新
 * ============================================================ */
static esp_err_t oled_refresh(void) {
    esp_err_t ret;
    ret = oled_send_command(0x21); if (ret != ESP_OK) return ret;
    ret = oled_send_command(0x00); if (ret != ESP_OK) return ret;
    ret = oled_send_command(0x7F); if (ret != ESP_OK) return ret;
    ret = oled_send_command(0x22); if (ret != ESP_OK) return ret;
    ret = oled_send_command(0x00); if (ret != ESP_OK) return ret;
    ret = oled_send_command(0x07); if (ret != ESP_OK) return ret;
    for (int offset = 0; offset < sizeof(g_oled_buffer); offset += 128) {
        uint8_t data[129];
        data[0] = 0x40;
        memcpy(&data[1], &g_oled_buffer[offset], 128);
        ret = oled_i2c_write(data, sizeof(data));
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

/* ============================================================
 * 11. 速度单位转换：RPS = mm/s / (PI × wheel_diameter_mm)
 * ============================================================ */
static float oled_mm_s_to_rps(float speed_mm_s) {
    const float wheel_diameter_mm = MOTOR_WHEEL_DIAMETER_MM;
    const float circumference_mm = 3.14159265358979323846f * wheel_diameter_mm;
    if (circumference_mm <= 0.0f) return 0.0f;
    return speed_mm_s / circumference_mm;
}

/* ============================================================
 * 12. OLED 内容更新
 * ============================================================ */
void oled_display_update(void) {
    if (!g_oled_initialized) return;
    motor_status_t motor_status = {0};
    motor_get_status(&motor_status);
    float rps_a, rps_b, rps_d;
    motor_get_wheel_rps(&rps_a, &rps_b, &rps_d);
    float distance_cm = obstacle_avoid_get_distance_cm();
    char line[32];
    oled_clear();
    snprintf(line, sizeof(line), "A:%+.2f RPS", rps_a); oled_draw_string(0, 0, line);
    snprintf(line, sizeof(line), "B:%+.2f RPS", rps_b); oled_draw_string(0, 16, line);
    snprintf(line, sizeof(line), "D:%+.2f RPS", rps_d); oled_draw_string(0, 32, line);
    if (distance_cm < 0.0f) snprintf(line, sizeof(line), "DIST:INVALID");
    else snprintf(line, sizeof(line), "DIST:%.1fcm", distance_cm);
    oled_draw_string(0, 48, line);
    esp_err_t ret = oled_refresh();
    if (ret != ESP_OK) ESP_LOGW(OLED_TAG, "Refresh failed: %s", esp_err_to_name(ret));
}

/* ============================================================
 * 13. OLED 后台任务
 * ============================================================ */
static void oled_display_task(void *arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(OLED_UPDATE_PERIOD_MS);
    while (1) { oled_display_update(); vTaskDelayUntil(&last_wake, period); }
}

/* ============================================================
 * 14. 初始化
 * ============================================================ */
esp_err_t oled_display_init(void) {
    if (g_oled_initialized) return ESP_OK;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = OLED_I2C_PORT,
        .sda_io_num = OLED_I2C_SDA_PIN,
        .scl_io_num = OLED_I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(
            &bus_cfg,
            &g_oled_bus
        ),
        OLED_TAG,
        "I2C bus create failed"
    );

    ESP_RETURN_ON_ERROR(
        oled_detect_address(),
        OLED_TAG,
        "OLED not found at 0x3C or 0x3D"
    );

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = g_oled_address,
        .scl_speed_hz = OLED_I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(
            g_oled_bus,
            &dev_cfg,
            &g_oled_dev
        ),
        OLED_TAG,
        "OLED device add failed"
    );
    ESP_RETURN_ON_ERROR(oled_detect_address(), OLED_TAG, "OLED not found at 0x3C or 0x3D");
    ESP_RETURN_ON_ERROR(oled_ssd1306_init(), OLED_TAG, "SSD1306 init failed");
    g_oled_initialized = true;
    oled_clear();
    ESP_RETURN_ON_ERROR(oled_refresh(), OLED_TAG, "Initial refresh failed");
    ESP_LOGI(OLED_TAG, "OLED initialized: SDA=%d SCL=%d ADDR=0x%02X", OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN, g_oled_address);
    return ESP_OK;
}

/* ============================================================
 * 15. 启动后台显示任务
 * ============================================================ */
esp_err_t oled_display_start(void) {
    if (!g_oled_initialized) return ESP_ERR_INVALID_STATE;
    if (g_oled_task_handle != NULL) return ESP_OK;
    BaseType_t ret = xTaskCreate(oled_display_task, "oled_display", OLED_TASK_STACK_SIZE, NULL, OLED_TASK_PRIORITY, &g_oled_task_handle);
    if (ret != pdPASS) { g_oled_task_handle = NULL; return ESP_ERR_NO_MEM; }
    ESP_LOGI(OLED_TAG, "OLED display task started");
    return ESP_OK;
}

#endif /* OLED_DISPLAY_IMPLEMENTATION */
#endif /* OLED_DISPLAY_H */