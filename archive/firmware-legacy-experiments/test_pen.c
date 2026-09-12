/*
 * main.c
 *
 * Runner + physical MG90S pen Motion/Event integration test.
 *
 * The application deliberately remains a single translation unit. All
 * single-header implementations used by the Runner are enabled here exactly
 * once. The default fixture is a small Motion/Event trajectory; compile-time
 * switches below also allow the chassis-only regression to reuse this file.
 */

#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#define MPU6050_IMPLEMENTATION
#define CHASSIS_KINEMATICS_IMPLEMENTATION
#define CHASSIS_ODOMETRY_IMPLEMENTATION
#define CHASSIS_MOTION_IMPLEMENTATION

#define PEN_CONTROL_IMPLEMENTATION
#define TRAJECTORY_DECODER_IMPLEMENTATION
#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#define TRAJECTORY_TRACKER_IMPLEMENTATION
#define TRAJECTORY_RUNNER_IMPLEMENTATION

#include "trajectory_runner.h"


static const char *TAG =
    "runner_pen";


#ifndef PEN_RUNNER_SPIFFS_PARTITION_LABEL
#define PEN_RUNNER_SPIFFS_PARTITION_LABEL "storage"
#endif

#ifndef PEN_RUNNER_TRAJECTORY_PATH
#define PEN_RUNNER_TRAJECTORY_PATH "/spiffs/trj2_smooth_small.traj"
#endif

#ifndef PEN_RUNNER_TEST_NAME
#define PEN_RUNNER_TEST_NAME "TRJ2 Motion + physical PEN Event"
#endif

/* Set to 0 for the chassis-only / motion-only regression. */
#ifndef PEN_RUNNER_ENABLE_PHYSICAL_PEN
#define PEN_RUNNER_ENABLE_PHYSICAL_PEN 0
#endif

#ifndef PEN_RUNNER_TASK_STACK_BYTES
#define PEN_RUNNER_TASK_STACK_BYTES (12U * 1024U)
#endif

#ifndef PEN_RUNNER_TASK_PRIORITY
#define PEN_RUNNER_TASK_PRIORITY 5
#endif


static trajectory_runner_config_t s_runner_config;
static trajectory_runner_t s_runner;
static trajectory_runner_status_t s_runner_status;


/*
 * Single, obvious hardware-parameter interface.
 *
 * These values come from the completed standalone actuator calibration.
 * Change this function when the mechanism, GPIO, or timing is changed.
 */
static void configure_runner(
    trajectory_runner_config_t *config)
{
    trajectory_runner_get_default_config(config);

    config->pen_enabled =
        (PEN_RUNNER_ENABLE_PHYSICAL_PEN != 0);

    if (config->pen_enabled)
    {
        pen_control_get_default_config(
            &config->pen);

        config->pen.gpio_num =
            GPIO_NUM_45;

        config->pen.timer_num =
            LEDC_TIMER_3;

        config->pen.channel =
            LEDC_CHANNEL_7;

        config->pen.positions_calibrated =
            true;

        config->pen.up_pulse_us =
            1540U;

        config->pen.down_pulse_us =
            1100U;

        config->pen.raise_time_ms =
            180U;

        config->pen.lower_time_ms =
            180U;

        config->pen.settle_time_ms =
            80U;
    }

    config->pen_action_timeout_ms =
        1000U;

    /* Initial conservative deadband-aware intermediate hold values. Tune
     * after collecting per-wheel diagnostics; do not silently lower these
     * below the measured chassis breakaway speed. */
    config->tracker.hold_min_speed_mm_s =
        20.0f;

    config->tracker.hold_max_speed_mm_s =
        50.0f;

    config->tracker.motor_stall_fault_enable =
        true;

    config->debug_log_enable =
        true;

    config->debug_log_period_ms =
        100U;
}


static esp_err_t mount_spiffs(void)
{
    const esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = PEN_RUNNER_SPIFFS_PARTITION_LABEL,
        .max_files = 8,
        .format_if_mount_failed = false,
    };

    esp_err_t ret =
        esp_vfs_spiffs_register(&conf);

    if ((ret != ESP_OK) &&
        (ret != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(
            TAG,
            "SPIFFS mount failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    size_t total = 0;
    size_t used = 0;

    ret =
        esp_spiffs_info(
            PEN_RUNNER_SPIFFS_PARTITION_LABEL,
            &total,
            &used);

    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS total=%u used=%u",
            (unsigned)total,
            (unsigned)used);
    }

    return ESP_OK;
}


static void log_final_status(
    esp_err_t run_ret)
{
    s_runner_status =
        (trajectory_runner_status_t){0};

    trajectory_runner_get_status(
        &s_runner,
        &s_runner_status);

    ESP_LOGI(
        TAG,
        "Result: ret=%s state=%s phase=%s error=%s pen=%s target=%s "
        "pen_state=%s pulse=%u target_pulse=%u tracker=%s "
        "cycles=%lu",
        esp_err_to_name(run_ret),
        trajectory_runner_state_name(
            s_runner_status.state),
        trajectory_runner_phase_name(
            s_runner_status.phase),
        trajectory_runner_error_name(
            s_runner_status.error),
        s_runner_status.pen_enabled ? "enabled" : "disabled",
        trajectory_runner_pen_state_name(
            s_runner_status.pen_target_state),
        pen_control_state_name(
            s_runner_status.pen.state),
        (unsigned)s_runner_status.pen.current_pulse_us,
        (unsigned)s_runner_status.pen.target_pulse_us,
        trajectory_tracker_state_name(
            s_runner_status.tracker.state),
        (unsigned long)s_runner_status.control_cycles);

    ESP_LOGI(
        TAG,
        "Final motor A{t=%.1f a=%.1f pwm=%.1f enc=%ld ready=%d startup=%d stall=%d/%lu} "
        "B{t=%.1f a=%.1f pwm=%.1f enc=%ld ready=%d startup=%d stall=%d/%lu} "
        "D{t=%.1f a=%.1f pwm=%.1f enc=%ld ready=%d startup=%d stall=%d/%lu}",
        (double)s_runner_status.motor.A.target_speed_mm_s,
        (double)s_runner_status.motor.A.actual_speed_mm_s,
        (double)s_runner_status.motor.A.output_pwm,
        (long)s_runner_status.motor.A.encoder_count,
        (int)s_runner_status.motor.A.encoder_ready,
        (int)s_runner_status.motor.A.startup_active,
        (int)s_runner_status.motor.A.stall_suspected,
        (unsigned long)s_runner_status.motor.A.stall_elapsed_ms,
        (double)s_runner_status.motor.B.target_speed_mm_s,
        (double)s_runner_status.motor.B.actual_speed_mm_s,
        (double)s_runner_status.motor.B.output_pwm,
        (long)s_runner_status.motor.B.encoder_count,
        (int)s_runner_status.motor.B.encoder_ready,
        (int)s_runner_status.motor.B.startup_active,
        (int)s_runner_status.motor.B.stall_suspected,
        (unsigned long)s_runner_status.motor.B.stall_elapsed_ms,
        (double)s_runner_status.motor.D.target_speed_mm_s,
        (double)s_runner_status.motor.D.actual_speed_mm_s,
        (double)s_runner_status.motor.D.output_pwm,
        (long)s_runner_status.motor.D.encoder_count,
        (int)s_runner_status.motor.D.encoder_ready,
        (int)s_runner_status.motor.D.startup_active,
        (int)s_runner_status.motor.D.stall_suspected,
        (unsigned long)s_runner_status.motor.D.stall_elapsed_ms);
}


static void runner_task(
    void *arg)
{
    (void)arg;

    ESP_LOGI(
        TAG,
        "%s START",
        PEN_RUNNER_TEST_NAME);

    ESP_LOGI(
        TAG,
        "Fixture: %s",
        PEN_RUNNER_TRAJECTORY_PATH);

    if (PEN_RUNNER_ENABLE_PHYSICAL_PEN != 0)
    {
        ESP_LOGI(
            TAG,
            "Physical pen: ENABLED; UP=1540 us DOWN=1100 us "
            "GPIO=45 TIMER=3 CHANNEL=7");
    }
    else
    {
        ESP_LOGI(
            TAG,
            "Physical pen: DISABLED (motion-only regression)");
    }

    ESP_LOGI(
        TAG,
        "Motor diag: stall target>=%.1f actual<=%.1f for %u ms; "
        "startup PWM A=%d/%d B=%d/%d D=%d/%d",
        (double)MOTOR_STALL_DETECT_TARGET_SPEED_MM_S,
        (double)MOTOR_STALL_DETECT_ACTUAL_SPEED_MM_S,
        (unsigned)MOTOR_STALL_DETECT_TIME_MS,
        (int)MOTOR_A_START_PWM_FWD,
        (int)MOTOR_A_START_PWM_REV,
        (int)MOTOR_B_START_PWM_FWD,
        (int)MOTOR_B_START_PWM_REV,
        (int)MOTOR_D_START_PWM_FWD,
        (int)MOTOR_D_START_PWM_REV);

    if (mount_spiffs() != ESP_OK)
    {
        vTaskDelete(NULL);
        return;
    }

    configure_runner(&s_runner_config);

    esp_err_t ret =
        trajectory_runner_init(
            &s_runner,
            &s_runner_config);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_runner_init failed: %s",
            esp_err_to_name(ret));

        vTaskDelete(NULL);
        return;
    }

    ret =
        trajectory_runner_run_file(
            &s_runner,
            PEN_RUNNER_TRAJECTORY_PATH);

    log_final_status(ret);

    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "%s PASS",
            PEN_RUNNER_TEST_NAME);
    }
    else
    {
        ESP_LOGE(
            TAG,
            "%s FAIL",
            PEN_RUNNER_TEST_NAME);
    }

    esp_vfs_spiffs_unregister(
        PEN_RUNNER_SPIFFS_PARTITION_LABEL);

    vTaskDelete(NULL);
}


void app_main(void)
{
    const BaseType_t created =
        xTaskCreate(
            runner_task,
            "runner_pen_test",
            PEN_RUNNER_TASK_STACK_BYTES,
            NULL,
            PEN_RUNNER_TASK_PRIORITY,
            NULL);

    if (created != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create Runner test task");
    }
}
