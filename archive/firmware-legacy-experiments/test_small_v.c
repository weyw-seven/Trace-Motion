/*
 * Small-speed FF + PI hardware test.
 *
 * The test deliberately commands low physical speeds instead of raw PWM.
 * With the running floor disabled, each wheel's PI loop may raise or lower
 * PWM independently.  Verify that actual wheel speeds approach their targets,
 * startup_active stays false, and encoder deltas are non-zero.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"

static const char *TAG = "small_speed_pi";

#ifndef SMALL_SPEED_HOLD_MS
#define SMALL_SPEED_HOLD_MS       3000U
#endif

#ifndef SMALL_SPEED_SAMPLE_MS
#define SMALL_SPEED_SAMPLE_MS      100U
#endif

#ifndef SMALL_SPEED_SETTLE_MS
#define SMALL_SPEED_SETTLE_MS      600U
#endif

#ifndef SMALL_SPEED_LINEAR_MM_S
#define SMALL_SPEED_LINEAR_MM_S    20.0f
#endif

#ifndef SMALL_SPEED_YAW_RAD_S
#define SMALL_SPEED_YAW_RAD_S       0.21f
#endif

typedef struct
{
    const char *name;
    float vx_mm_s;
    float vy_mm_s;
    float w_rad_s;
} small_speed_case_t;

static void log_status(
    const char *name,
    uint32_t elapsed_ms)
{
    motor_status_t s = {0};
    motor_get_status(&s);

    ESP_LOGI(
        TAG,
        "%s t=%lu target=(%+.1f,%+.1f,%+.3f) "
        "A{t=%+.1f a=%+.1f pwm=%+.1f start=%d stall=%d} "
        "B{t=%+.1f a=%+.1f pwm=%+.1f start=%d stall=%d} "
        "D{t=%+.1f a=%+.1f pwm=%+.1f start=%d stall=%d}",
        name,
        (unsigned long)elapsed_ms,
        (double)s.target_vx_mm_s,
        (double)s.target_vy_mm_s,
        (double)s.target_w_rad_s,
        (double)s.A.target_speed_mm_s,
        (double)s.A.actual_speed_mm_s,
        (double)s.A.output_pwm,
        (int)s.A.startup_active,
        (int)s.A.stall_suspected,
        (double)s.B.target_speed_mm_s,
        (double)s.B.actual_speed_mm_s,
        (double)s.B.output_pwm,
        (int)s.B.startup_active,
        (int)s.B.stall_suspected,
        (double)s.D.target_speed_mm_s,
        (double)s.D.actual_speed_mm_s,
        (double)s.D.output_pwm,
        (int)s.D.startup_active,
        (int)s.D.stall_suspected);
}

static void run_case(const small_speed_case_t *test_case)
{
    motor_status_t before = {0};
    motor_status_t after = {0};

    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(SMALL_SPEED_SETTLE_MS));
    motor_get_status(&before);

    ESP_LOGI(
        TAG,
        "CASE START %s command=(vx=%+.2f vy=%+.2f w=%+.3f) "
        "Ki=(%.2f,%.2f,%.2f) B_scale_min=%.1f floor=%d",
        test_case->name,
        (double)test_case->vx_mm_s,
        (double)test_case->vy_mm_s,
        (double)test_case->w_rad_s,
        (double)MOTOR_A_PID_KI,
        (double)MOTOR_B_PID_KI,
        (double)MOTOR_D_PID_KI,
        (double)MOTOR_PWM_MIN,
        (int)MOTOR_RUN_PWM_FLOOR_ENABLE);

    motor_set_velocity(
        test_case->vx_mm_s,
        test_case->vy_mm_s,
        test_case->w_rad_s);

    const uint32_t sample_count =
        (SMALL_SPEED_HOLD_MS + SMALL_SPEED_SAMPLE_MS - 1U) /
        SMALL_SPEED_SAMPLE_MS;

    for (uint32_t i = 0U; i < sample_count; ++i)
    {
        vTaskDelay(pdMS_TO_TICKS(SMALL_SPEED_SAMPLE_MS));
        log_status(
            test_case->name,
            (i + 1U) * SMALL_SPEED_SAMPLE_MS);
    }

    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(SMALL_SPEED_SETTLE_MS));
    motor_get_status(&after);

    const int32_t da = after.A.encoder_count - before.A.encoder_count;
    const int32_t db = after.B.encoder_count - before.B.encoder_count;
    const int32_t dd = after.D.encoder_count - before.D.encoder_count;

    ESP_LOGI(
        TAG,
        "CASE END %s encoder_delta=(%ld,%ld,%ld) motion=%s",
        test_case->name,
        (long)da,
        (long)db,
        (long)dd,
        ((da != 0) || (db != 0) || (dd != 0)) ? "YES" : "NO");
}

static void small_speed_task(void *arg)
{
    (void)arg;

    const esp_err_t init_ret = motor_control_init();
    if (init_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "motor_control_init failed: %s", esp_err_to_name(init_ret));
        vTaskDelete(NULL);
        return;
    }

    motor_status_t ready = {0};
    motor_get_status(&ready);
    ESP_LOGI(
        TAG,
        "ready initialized=%d enabled=%d closed_loop=%d "
        "startup_flags=(%d,%d,%d)",
        (int)ready.initialized,
        (int)ready.enabled,
        (int)ready.closed_loop_ready,
        (int)ready.A.startup_active,
        (int)ready.B.startup_active,
        (int)ready.D.startup_active);

    if (!ready.closed_loop_ready)
    {
        ESP_LOGE(TAG, "closed-loop encoders are not ready; test aborted");
        motor_emergency_stop();
        vTaskDelete(NULL);
        return;
    }

    const small_speed_case_t cases[] = {
        {"forward", SMALL_SPEED_LINEAR_MM_S, 0.0f, 0.0f},
        {"backward", -SMALL_SPEED_LINEAR_MM_S, 0.0f, 0.0f},
        {"left", 0.0f, SMALL_SPEED_LINEAR_MM_S, 0.0f},
        {"right", 0.0f, -SMALL_SPEED_LINEAR_MM_S, 0.0f},
        {"yaw_ccw", 0.0f, 0.0f, SMALL_SPEED_YAW_RAD_S},
        {"yaw_cw", 0.0f, 0.0f, -SMALL_SPEED_YAW_RAD_S},
    };

    for (size_t i = 0U; i < (sizeof(cases) / sizeof(cases[0])); ++i)
    {
        run_case(&cases[i]);
    }

    motor_stop();
    ESP_LOGI(TAG, "small-speed FF + PI test complete");
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(
        small_speed_task,
        "small_speed_pi",
        4096,
        NULL,
        5,
        NULL);
}
