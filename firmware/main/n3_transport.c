#include "n3_transport.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"


#ifndef N3_ENABLE_UART0
#define N3_ENABLE_UART0 1
#endif

#ifndef N3_ENABLE_USB_SERIAL_JTAG
#define N3_ENABLE_USB_SERIAL_JTAG 1
#endif

#ifndef N3_UART_BAUD_RATE
#define N3_UART_BAUD_RATE 115200
#endif

#ifndef N3_UART_RX_BUFFER_SIZE
#define N3_UART_RX_BUFFER_SIZE 2048
#endif

#ifndef N3_UART_TX_BUFFER_SIZE
#define N3_UART_TX_BUFFER_SIZE 2048
#endif

#ifndef N3_ENABLE_WIFI
#define N3_ENABLE_WIFI 0
#endif

#ifndef N3_WIFI_AP_SSID
/* A public release must provide its own network name at build time. */
#define N3_WIFI_AP_SSID "N3-Robot"
#endif

#ifndef N3_WIFI_AP_PASSWORD
/* This is intentionally an unusable deployment default, not a credential. */
#define N3_WIFI_AP_PASSWORD "change-me-before-flashing"
#endif

#ifndef N3_WIFI_TCP_PORT
#define N3_WIFI_TCP_PORT 5000
#endif

#ifndef N3_WIFI_LISTENER_RETRY_COUNT
#define N3_WIFI_LISTENER_RETRY_COUNT 10
#endif

#ifndef N3_WIFI_LISTENER_RETRY_DELAY_MS
#define N3_WIFI_LISTENER_RETRY_DELAY_MS 250
#endif


static const char *TAG = "n3_transport";

static bool s_uart_ready;
static bool s_usb_serial_jtag_ready;
static bool s_wifi_tcp_ready;
static SemaphoreHandle_t s_write_mutex;
static int s_wifi_listener_fd = -1;
static int s_wifi_client_fd = -1;


static void wifi_close_client_locked(void)
{
    if (s_wifi_client_fd >= 0)
    {
        shutdown(s_wifi_client_fd, SHUT_RDWR);
        close(s_wifi_client_fd);
        s_wifi_client_fd = -1;
    }
}


static void wifi_copy_text(
    uint8_t *destination,
    size_t destination_size,
    const char *source)
{
    if ((destination == NULL) || (destination_size == 0U))
    {
        return;
    }

    (void)snprintf((char *)destination, destination_size, "%s", source);
}


static esp_err_t wifi_open_listener(void)
{
    if (s_wifi_listener_fd >= 0)
    {
        return ESP_OK;
    }

    const int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listener < 0)
    {
        ESP_LOGW(TAG, "Wi-Fi TCP socket() failed: errno=%d", errno);
        return ESP_FAIL;
    }

    const int reuse = 1;
    (void)setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    const struct sockaddr_in address =
    {
        .sin_family = AF_INET,
        .sin_port = htons(N3_WIFI_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listener, (const struct sockaddr *)&address, sizeof(address)) != 0)
    {
        ESP_LOGW(
            TAG,
            "Wi-Fi TCP bind(0.0.0.0:%u) failed: errno=%d",
            (unsigned)N3_WIFI_TCP_PORT,
            errno);
        close(listener);
        return ESP_FAIL;
    }
    if (listen(listener, 1) != 0)
    {
        ESP_LOGW(TAG, "Wi-Fi TCP listen() failed: errno=%d", errno);
        close(listener);
        return ESP_FAIL;
    }

    s_wifi_listener_fd = listener;
    return ESP_OK;
}


static esp_err_t wifi_tcp_init(void)
{
#if !N3_ENABLE_WIFI
    return ESP_OK;
#else
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) return ret;
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    if (esp_netif_create_default_wifi_ap() == NULL) return ESP_FAIL;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&init);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    wifi_config_t config = {0};
    wifi_copy_text(config.ap.ssid, sizeof(config.ap.ssid), N3_WIFI_AP_SSID);
    wifi_copy_text(config.ap.password, sizeof(config.ap.password), N3_WIFI_AP_PASSWORD);
    config.ap.ssid_len = strlen(N3_WIFI_AP_SSID);
    config.ap.channel = 6;
    config.ap.max_connection = 1;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) return ret;
    ret = esp_wifi_set_config(WIFI_IF_AP, &config);
    if (ret != ESP_OK) return ret;
    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    for (int attempt = 1; attempt <= N3_WIFI_LISTENER_RETRY_COUNT; ++attempt)
    {
        ret = wifi_open_listener();
        if (ret == ESP_OK)
        {
            break;
        }

        ESP_LOGW(
            TAG,
            "Wi-Fi TCP listener retry %d/%d",
            attempt,
            N3_WIFI_LISTENER_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(N3_WIFI_LISTENER_RETRY_DELAY_MS));
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi TCP listener did not start");
        return ret;
    }
    s_wifi_tcp_ready = true;
    ESP_LOGI(
        TAG,
        "Wi-Fi AP ready: ssid=%s tcp=192.168.4.1:%u",
        N3_WIFI_AP_SSID,
        (unsigned)N3_WIFI_TCP_PORT);
    return ESP_OK;
#endif
}


static TickType_t timeout_to_ticks(
    uint32_t timeout_ms)
{
    if (timeout_ms == 0U)
    {
        return 0;
    }

    const TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    return (ticks == 0) ? 1 : ticks;
}


esp_err_t n3_transport_init(void)
{
    esp_err_t ret = ESP_OK;

    if (s_write_mutex == NULL)
    {
        s_write_mutex = xSemaphoreCreateMutex();
        if (s_write_mutex == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

#if N3_ENABLE_UART0
    const uart_config_t uart_config =
    {
        .baud_rate = N3_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_driver_install(
        UART_NUM_0,
        N3_UART_RX_BUFFER_SIZE,
        N3_UART_TX_BUFFER_SIZE,
        0,
        NULL,
        0);

    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(UART_NUM_0, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_uart_ready = true;
#endif

#if N3_ENABLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t usb_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

    ret = usb_serial_jtag_driver_install(&usb_config);
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(
            TAG,
            "usb_serial_jtag_driver_install failed: %s",
            esp_err_to_name(ret));
        return ret;
    }

    s_usb_serial_jtag_ready = true;
#endif

    ret = wifi_tcp_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi TCP init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!s_uart_ready && !s_usb_serial_jtag_ready && !s_wifi_tcp_ready)
    {
        ESP_LOGE(TAG, "no N3 transport endpoint is enabled");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(
        TAG,
        "N3 transport ready: uart0=%d usb_serial_jtag=%d wifi_tcp=%d",
        (int)s_uart_ready,
        (int)s_usb_serial_jtag_ready,
        (int)s_wifi_tcp_ready);

    return ESP_OK;
}


bool n3_transport_endpoint_enabled(
    n3_transport_endpoint_t endpoint)
{
    switch (endpoint)
    {
        case N3_TRANSPORT_ENDPOINT_UART0:
            return s_uart_ready;
        case N3_TRANSPORT_ENDPOINT_USB_SERIAL_JTAG:
            return s_usb_serial_jtag_ready;
        case N3_TRANSPORT_ENDPOINT_WIFI_TCP:
            return s_wifi_tcp_ready;
        default:
            return false;
    }
}


int n3_transport_read(
    n3_transport_endpoint_t endpoint,
    void *buffer,
    size_t length,
    uint32_t timeout_ms)
{
    if ((buffer == NULL) || (length == 0U))
    {
        return 0;
    }

    const TickType_t ticks = timeout_to_ticks(timeout_ms);

    switch (endpoint)
    {
        case N3_TRANSPORT_ENDPOINT_UART0:
#if N3_ENABLE_UART0
            return s_uart_ready
                ? uart_read_bytes(UART_NUM_0, buffer, length, ticks)
                : -1;
#else
            return -1;
#endif

        case N3_TRANSPORT_ENDPOINT_USB_SERIAL_JTAG:
#if N3_ENABLE_USB_SERIAL_JTAG
            return s_usb_serial_jtag_ready
                ? usb_serial_jtag_read_bytes(buffer, length, ticks)
                : -1;
#else
            return -1;
#endif

        case N3_TRANSPORT_ENDPOINT_WIFI_TCP:
#if N3_ENABLE_WIFI
        {
            if (!s_wifi_tcp_ready)
            {
                return -1;
            }

            int client_fd = -1;
            if (s_write_mutex != NULL &&
                xSemaphoreTake(s_write_mutex, portMAX_DELAY) == pdTRUE)
            {
                client_fd = s_wifi_client_fd;
                xSemaphoreGive(s_write_mutex);
            }

            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(s_wifi_listener_fd, &read_set);
            int max_fd = s_wifi_listener_fd;
            if (client_fd >= 0)
            {
                FD_SET(client_fd, &read_set);
                if (client_fd > max_fd)
                {
                    max_fd = client_fd;
                }
            }
            struct timeval timeout =
            {
                .tv_sec = timeout_ms / 1000U,
                .tv_usec = (timeout_ms % 1000U) * 1000U,
            };
            const int selected = select(max_fd + 1, &read_set, NULL, NULL, &timeout);
            if (selected <= 0)
            {
                return 0;
            }
            if (FD_ISSET(s_wifi_listener_fd, &read_set))
            {
                const int accepted = accept(s_wifi_listener_fd, NULL, NULL);
                if (accepted >= 0)
                {
                    if (s_write_mutex != NULL &&
                        xSemaphoreTake(s_write_mutex, portMAX_DELAY) == pdTRUE)
                    {
                        wifi_close_client_locked();
                        s_wifi_client_fd = accepted;
                        xSemaphoreGive(s_write_mutex);
                        ESP_LOGI(TAG, "Wi-Fi TCP client connected");
                    }
                    else
                    {
                        close(accepted);
                    }
                }
                return 0;
            }
            if (client_fd >= 0 && FD_ISSET(client_fd, &read_set))
            {
                const int received = recv(client_fd, buffer, length, 0);
                if (received <= 0 && s_write_mutex != NULL &&
                    xSemaphoreTake(s_write_mutex, portMAX_DELAY) == pdTRUE)
                {
                    if (s_wifi_client_fd == client_fd)
                    {
                        wifi_close_client_locked();
                        ESP_LOGI(TAG, "Wi-Fi TCP client disconnected");
                    }
                    xSemaphoreGive(s_write_mutex);
                }
                return received > 0 ? received : 0;
            }
            return 0;
        }
#else
            return -1;
#endif

        default:
            return -1;
    }
}


esp_err_t n3_transport_write(
    const void *buffer,
    size_t length)
{
    if ((buffer == NULL) || (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_write_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_write_mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

#if N3_ENABLE_UART0
    if (s_uart_ready)
    {
        const int written = uart_write_bytes(UART_NUM_0, buffer, length);
        if (written < 0 || (size_t)written != length)
        {
            ret = ESP_FAIL;
        }
    }
#endif

#if N3_ENABLE_WIFI
    if (s_wifi_client_fd >= 0)
    {
        const int written = send(s_wifi_client_fd, buffer, length, 0);
        if (written < 0 || (size_t)written != length)
        {
            wifi_close_client_locked();
            ret = ESP_FAIL;
        }
    }
#endif

#if N3_ENABLE_USB_SERIAL_JTAG
    if (s_usb_serial_jtag_ready)
    {
        const int written = usb_serial_jtag_write_bytes(
            buffer,
            length,
            pdMS_TO_TICKS(100));
        if (written < 0 || (size_t)written != length)
        {
            ret = ESP_FAIL;
        }
    }
#endif

    xSemaphoreGive(s_write_mutex);
    return ret;
}
