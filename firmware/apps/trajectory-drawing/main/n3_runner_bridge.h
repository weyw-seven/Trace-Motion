#ifndef N3_RUNNER_BRIDGE_H
#define N3_RUNNER_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * N3 event-only runner.
 *
 * This bridge deliberately owns the no-hardware execution path. It accepts
 * only TRJ2 PEN_UP, PEN_DOWN, and WAIT records and never initializes or calls
 * the motor, MPU6050, odometry, tracker, or physical pen drivers.
 */

typedef enum
{
    N3_RUNNER_STATE_IDLE = 0,
    N3_RUNNER_STATE_STARTING,
    N3_RUNNER_STATE_RUNNING,
    N3_RUNNER_STATE_WAITING,
    N3_RUNNER_STATE_FINISHED,
    N3_RUNNER_STATE_STOPPED,
    N3_RUNNER_STATE_ESTOPPED,
    N3_RUNNER_STATE_ERROR

} n3_runner_state_t;


typedef struct
{
    bool valid;
    n3_runner_state_t state;
    bool stop_requested;
    bool emergency_stop;
    bool pen_down;
    bool pen_enabled;
    char pen_state[16];
    char pen_target[16];
    bool pen_busy;
    bool pen_settling;
    uint16_t pen_current_pulse_us;
    uint16_t pen_target_pulse_us;
    char pen_error[32];
    uint32_t record_index;
    uint32_t record_count;
    bool pose_valid;
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vx_world_mm_s;
    float vy_world_mm_s;
    float w_rad_s;
    char runner[16];
    char tracker[16];
    char phase[24];
    char record_type[20];
    char error_code[32];
    char tracker_error[32];
    char pen[8];
    char execution_mode[16];
    char error[64];
    float tracking_error_mm;
    float tracking_yaw_error_deg;
    float reference_x_mm;
    float reference_y_mm;
    float command_vx_body_mm_s;
    float command_vy_body_mm_s;
    float command_w_rad_s;
    uint32_t settle_elapsed_ms;
    float wheel_a_target_mm_s;
    float wheel_a_actual_mm_s;
    float wheel_a_pwm;
    bool wheel_a_stall_suspected;
    uint32_t wheel_a_stall_elapsed_ms;
    float wheel_b_target_mm_s;
    float wheel_b_actual_mm_s;
    float wheel_b_pwm;
    bool wheel_b_stall_suspected;
    uint32_t wheel_b_stall_elapsed_ms;
    float wheel_d_target_mm_s;
    float wheel_d_actual_mm_s;
    float wheel_d_pwm;
    bool wheel_d_stall_suspected;
    uint32_t wheel_d_stall_elapsed_ms;

} n3_runner_status_t;


esp_err_t n3_runner_bridge_init(void);


/* Called for every valid protocol line, including PC heartbeat PINGs. */
void n3_runner_bridge_note_transport_activity(void);


/* Start a validated event-only task from an on-device trajectory file. */
esp_err_t n3_runner_bridge_start_event_only(
    const char *path,
    const char **error_code,
    const char **error_message);


/* Request a cooperative stop. emergency=true latches ESTOPPED status. */
void n3_runner_bridge_request_stop(
    bool emergency);


/* Clear the bridge's ESTOPPED state after the task has stopped. */
void n3_runner_bridge_clear_estop(void);


/* Update the logical/simulated pose when no trajectory is active. */
void n3_runner_bridge_reset_pose(
    float x_mm,
    float y_mm,
    float yaw_deg);


bool n3_runner_bridge_is_active(void);


void n3_runner_bridge_get_status(
    n3_runner_status_t *status);


#endif /* N3_RUNNER_BRIDGE_H */
