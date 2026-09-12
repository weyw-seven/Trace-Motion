#ifndef LINE_DEBUG_WIFI_H
#define LINE_DEBUG_WIFI_H

/*
 * line_debug_wifi.h
 *
 * Lightweight SoftAP used only by the line-debug browser.
 *
 * Single-header usage:
 *   In exactly ONE .c file:
 *       #define LINE_DEBUG_WIFI_IMPLEMENTATION
 *       #include "line_debug_wifi.h"
 *
 * Default laptop workflow:
 *   SSID:     TraceMotion-Setup
 *   password: supplied by the build configuration
 *   URL:      http://192.168.4.1/
 */

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LINE_DEBUG_WIFI_SSID
#define LINE_DEBUG_WIFI_SSID               "TraceMotion-Setup"
#endif

#ifndef LINE_DEBUG_WIFI_PASSWORD
/* Development placeholder: provide a private password before enabling Wi-Fi. */
#define LINE_DEBUG_WIFI_PASSWORD           "change-me-before-flashing"
#endif

#ifndef LINE_DEBUG_WIFI_CHANNEL
#define LINE_DEBUG_WIFI_CHANNEL            6U
#endif

#ifndef LINE_DEBUG_WIFI_MAX_CLIENTS
#define LINE_DEBUG_WIFI_MAX_CLIENTS        1U
#endif

#ifndef LINE_DEBUG_WIFI_DISABLE_POWER_SAVE
#define LINE_DEBUG_WIFI_DISABLE_POWER_SAVE 1
#endif

esp_err_t line_debug_wifi_init(void);
bool line_debug_wifi_is_ready(void);
const char *line_debug_wifi_get_ip_string(void);
uint32_t line_debug_wifi_get_client_count(void);

#ifdef __cplusplus
}
#endif

#ifdef LINE_DEBUG_WIFI_IMPLEMENTATION

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *LINE_DEBUG_WIFI_TAG = "DEBUG_WIFI";

static bool s_line_debug_wifi_ready = false;
static esp_netif_t *s_line_debug_wifi_ap_netif = NULL;
static char s_line_debug_wifi_ip[16] = "192.168.4.1";
static uint32_t s_line_debug_wifi_clients = 0U;
static portMUX_TYPE s_line_debug_wifi_lock = portMUX_INITIALIZER_UNLOCKED;

static void line_debug_wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        const wifi_event_ap_staconnected_t *event =
            (const wifi_event_ap_staconnected_t *)event_data;

        taskENTER_CRITICAL(&s_line_debug_wifi_lock);
        ++s_line_debug_wifi_clients;
        const uint32_t clients = s_line_debug_wifi_clients;
        taskEXIT_CRITICAL(&s_line_debug_wifi_lock);

        ESP_LOGI(
            LINE_DEBUG_WIFI_TAG,
            "Laptop/client connected, aid=%u clients=%u",
            event != NULL ? (unsigned)event->aid : 0U,
            (unsigned)clients);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        const wifi_event_ap_stadisconnected_t *event =
            (const wifi_event_ap_stadisconnected_t *)event_data;

        taskENTER_CRITICAL(&s_line_debug_wifi_lock);
        if (s_line_debug_wifi_clients > 0U)
        {
            --s_line_debug_wifi_clients;
        }
        const uint32_t clients = s_line_debug_wifi_clients;
        taskEXIT_CRITICAL(&s_line_debug_wifi_lock);

        ESP_LOGI(
            LINE_DEBUG_WIFI_TAG,
            "Laptop/client disconnected, aid=%u clients=%u",
            event != NULL ? (unsigned)event->aid : 0U,
            (unsigned)clients);
    }
}

static esp_err_t line_debug_wifi_init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (ret == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ret = nvs_flash_erase();
        if (ret != ESP_OK)
        {
            return ret;
        }

        ret = nvs_flash_init();
    }

    return ret;
}

esp_err_t line_debug_wifi_init(void)
{
    if (s_line_debug_wifi_ready)
    {
        return ESP_OK;
    }

    const size_t ssid_len = strlen(LINE_DEBUG_WIFI_SSID);
    const size_t password_len = strlen(LINE_DEBUG_WIFI_PASSWORD);

    if ((ssid_len == 0U) || (ssid_len > 32U))
    {
        ESP_LOGE(LINE_DEBUG_WIFI_TAG, "SSID length must be 1..32 bytes");
        return ESP_ERR_INVALID_ARG;
    }

    if ((password_len != 0U) &&
        ((password_len < 8U) || (password_len > 63U)))
    {
        ESP_LOGE(
            LINE_DEBUG_WIFI_TAG,
            "Password must be empty (open AP) or 8..63 bytes");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = line_debug_wifi_init_nvs();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = esp_netif_init();
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
    {
        return ret;
    }

    ret = esp_event_loop_create_default();
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
    {
        return ret;
    }

    /* Must be called only once unless the default AP netif is destroyed. */
    s_line_debug_wifi_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_line_debug_wifi_ap_netif == NULL)
    {
        return ESP_FAIL;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();

    ret = esp_wifi_init(&init_cfg);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        line_debug_wifi_event_handler,
        NULL);
    if (ret != ESP_OK)
    {
        return ret;
    }

    wifi_config_t ap_cfg = {0};

    memcpy(ap_cfg.ap.ssid, LINE_DEBUG_WIFI_SSID, ssid_len);
    ap_cfg.ap.ssid_len = (uint8_t)ssid_len;

    if (password_len > 0U)
    {
        memcpy(ap_cfg.ap.password, LINE_DEBUG_WIFI_PASSWORD, password_len);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }
    else
    {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ap_cfg.ap.channel = (uint8_t)LINE_DEBUG_WIFI_CHANNEL;
    ap_cfg.ap.max_connection = (uint8_t)LINE_DEBUG_WIFI_MAX_CLIENTS;

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        return ret;
    }

#if LINE_DEBUG_WIFI_DISABLE_POWER_SAVE
    /* Harmless in the debug build and keeps latency behavior predictable. */
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
#endif

    esp_netif_ip_info_t ip_info = {0};
    if (esp_netif_get_ip_info(
            s_line_debug_wifi_ap_netif,
            &ip_info) == ESP_OK)
    {
        (void)snprintf(
            s_line_debug_wifi_ip,
            sizeof(s_line_debug_wifi_ip),
            IPSTR,
            IP2STR(&ip_info.ip));
    }

    s_line_debug_wifi_ready = true;

    ESP_LOGI(LINE_DEBUG_WIFI_TAG, "========================================");
    ESP_LOGI(LINE_DEBUG_WIFI_TAG, "Debug SoftAP ready");
    ESP_LOGI(LINE_DEBUG_WIFI_TAG, "SSID: %s", LINE_DEBUG_WIFI_SSID);
    ESP_LOGI(LINE_DEBUG_WIFI_TAG, "Password: %s",
        password_len > 0U ? LINE_DEBUG_WIFI_PASSWORD : "<open>");
    ESP_LOGI(LINE_DEBUG_WIFI_TAG, "Open: http://%s/", s_line_debug_wifi_ip);
    ESP_LOGI(LINE_DEBUG_WIFI_TAG, "========================================");

    return ESP_OK;
}

bool line_debug_wifi_is_ready(void)
{
    return s_line_debug_wifi_ready;
}

const char *line_debug_wifi_get_ip_string(void)
{
    return s_line_debug_wifi_ip;
}

uint32_t line_debug_wifi_get_client_count(void)
{
    uint32_t clients;

    taskENTER_CRITICAL(&s_line_debug_wifi_lock);
    clients = s_line_debug_wifi_clients;
    taskEXIT_CRITICAL(&s_line_debug_wifi_lock);

    return clients;
}

#endif /* LINE_DEBUG_WIFI_IMPLEMENTATION */
#endif /* LINE_DEBUG_WIFI_H */
