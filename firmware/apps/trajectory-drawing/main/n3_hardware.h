#ifndef N3_HARDWARE_H
#define N3_HARDWARE_H

#include <stdbool.h>

#include "esp_err.h"

typedef struct
{
    bool enabled;
    bool initializing;
    bool motor_ready;
    bool odom_ready;
    bool pose_valid;
    bool estop;
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vx_world_mm_s;
    float vy_world_mm_s;
    float w_rad_s;
    char error[64];
} n3_hardware_status_t;


/* Start asynchronous hardware initialization for hardware-enabled builds. */
esp_err_t n3_hardware_init(void);


void n3_hardware_get_status(
    n3_hardware_status_t *status);


/* These are safe no-ops in safe/simulation builds. */
void n3_hardware_stop(
    bool emergency);


esp_err_t n3_hardware_clear_estop(void);


esp_err_t n3_hardware_reset_pose(
    float x_mm,
    float y_mm,
    float yaw_deg);


#endif /* N3_HARDWARE_H */
