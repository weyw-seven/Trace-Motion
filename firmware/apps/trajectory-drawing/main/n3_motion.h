#ifndef N3_MOTION_H
#define N3_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct
{
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
    char runner[16];
    char tracker[16];
    char error[64];
} n3_motion_status_t;


/* Start the guarded motion supervisor. Safe/event-only profiles are no-ops. */
esp_err_t n3_motion_init(void);


/* Validate and start one bounded relative rotation. */
esp_err_t n3_motion_rotate(
    float angle_deg,
    float speed_deg_s,
    const char **error_code,
    const char **error_message);


/* Stop the active rotation; emergency=true latches the motion safety state. */
void n3_motion_stop(
    bool emergency);


/* Clear the motion-local emergency latch after hardware ESTOP is cleared. */
void n3_motion_clear_estop(void);


/* Called for every valid incoming protocol line, including PING heartbeats. */
void n3_motion_note_transport_activity(void);


bool n3_motion_is_active(void);


void n3_motion_get_status(
    n3_motion_status_t *status);


#endif /* N3_MOTION_H */
