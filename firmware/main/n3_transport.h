#ifndef N3_TRANSPORT_H
#define N3_TRANSPORT_H

/*
 * N3 transport fan-out for the ESP32-S3 development board.
 *
 * The current sdkconfig exposes both UART0 and the native USB Serial/JTAG
 * console.  Keeping the transport behind this small interface lets the
 * protocol service work with either COM port while the board connection is
 * being settled.  Frames are written to both enabled endpoints; commands are
 * accepted from either endpoint.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    N3_TRANSPORT_ENDPOINT_UART0 = 0,
    N3_TRANSPORT_ENDPOINT_USB_SERIAL_JTAG,
    N3_TRANSPORT_ENDPOINT_WIFI_TCP,
    N3_TRANSPORT_ENDPOINT_COUNT
} n3_transport_endpoint_t;

esp_err_t n3_transport_init(void);

bool n3_transport_endpoint_enabled(
    n3_transport_endpoint_t endpoint);

int n3_transport_read(
    n3_transport_endpoint_t endpoint,
    void *buffer,
    size_t length,
    uint32_t timeout_ms);

esp_err_t n3_transport_write(
    const void *buffer,
    size_t length);

#endif /* N3_TRANSPORT_H */
