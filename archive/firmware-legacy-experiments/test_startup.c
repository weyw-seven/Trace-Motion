/*
 * test_startup.c
 *
 * Raw-PWM breakaway / startup-threshold test for one motor wheel.
 *
 * This is intentionally a separate single-translation-unit test.  The
 * project CMake file still builds only main.c; copy this file to main.c (or
 * temporarily rename it) when you want to flash this test.
 *
 * The default test is the failure case observed in the trajectory:
 * wheel A, reverse direction, PWM 100..300 in 20-count steps.
 * Raw PWM bypasses kinematics, FF, PID, and the normal startup state machine.
 * The measured first sustained-motion PWM is the value to use as the loaded
 * startup compensation for that wheel/direction.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"


static const char *TAG = "startup_test";


/* ============================================================
 * Test interface
 * ============================================================ */

#define STARTUP_TEST_WHEEL_A 0
#define STARTUP_TEST_WHEEL_B 1
#define STARTUP_TEST_WHEEL_D 2

/* Default: reproduce the A-wheel reverse failure. */
#ifndef STARTUP_TEST_WHEEL
#define STARTUP_TEST_WHEEL STARTUP_TEST_WHEEL_A
#endif

#ifndef STARTUP_TEST_DIRECTION
#define STARTUP_TEST_DIRECTION (-1)
#endif

/* Set to 1 to run A-, A+, B-, B+, D-, D+ in one boot. */
#ifndef STARTUP_TEST_RUN_ALL_DIRECTIONS
#define STARTUP_TEST_RUN_ALL_DIRECTIONS 1
#endif

#ifndef STARTUP_TEST_MIN_PWM
#define STARTUP_TEST_MIN_PWM 100
#endif

#ifndef STARTUP_TEST_MAX_PWM
#define STARTUP_TEST_MAX_PWM 400
#endif

#ifndef STARTUP_TEST_STEP_PWM
#define STARTUP_TEST_STEP_PWM 20
#endif

/* Time at each PWM level and time between levels. */
#ifndef STARTUP_TEST_HOLD_MS
#define STARTUP_TEST_HOLD_MS 800U
#endif

#ifndef STARTUP_TEST_SETTLE_MS
#define STARTUP_TEST_SETTLE_MS 500U
#endif

#ifndef STARTUP_TEST_SAMPLE_MS
#define STARTUP_TEST_SAMPLE_MS 100U
#endif

/* Used only for the automatic STARTED/NOT_STARTED label in the log. */
#ifndef STARTUP_TEST_MOTION_SPEED_MM_S
#define STARTUP_TEST_MOTION_SPEED_MM_S 5.0f
#endif

#ifndef STARTUP_TEST_REQUIRED_MOTION_SAMPLES
#define STARTUP_TEST_REQUIRED_MOTION_SAMPLES 3U
#endif


typedef struct
{
    int wheel;
    int direction;
} startup_test_case_t;


static const char *wheel_name(int wheel)
{
    switch (wheel)
    {
        case STARTUP_TEST_WHEEL_A:
            return "A";
        case STARTUP_TEST_WHEEL_B:
            return "B";
        case STARTUP_TEST_WHEEL_D:
            return "D";
        default:
            return "?";
    }
}


static const motor_wheel_status_t *wheel_status(
    const motor_status_t *status,
    int wheel)
{
    if (status == NULL)
    {
        return NULL;
    }

    switch (wheel)
    {
        case STARTUP_TEST_WHEEL_A:
            return &status->A;
        case STARTUP_TEST_WHEEL_B:
            return &status->B;
        case STARTUP_TEST_WHEEL_D:
            return &status->D;
        default:
            return NULL;
    }
}


static void set_single_wheel_pwm(
    int wheel,
    float pwm)
{
    float pwm_a = 0.0f;
    float pwm_b = 0.0f;
    float pwm_d = 0.0f;

    switch (wheel)
    {
        case STARTUP_TEST_WHEEL_A:
            pwm_a = pwm;
            break;
        case STARTUP_TEST_WHEEL_B:
            pwm_b = pwm;
            break;
        case STARTUP_TEST_WHEEL_D:
            pwm_d = pwm;
            break;
        default:
            break;
    }

    motor_debug_set_raw_pwm(
        pwm_a,
        pwm_b,
        pwm_d);
}


static void stop_all_motors(void)
{
    motor_debug_set_raw_pwm(
        0.0f,
        0.0f,
        0.0f);

    vTaskDelay(
        pdMS_TO_TICKS(STARTUP_TEST_SETTLE_MS));
}


static void log_status(
    const startup_test_case_t *test_case,
    int pwm,
    uint32_t elapsed_ms,
    int32_t baseline_count,
    uint32_t *motion_samples)
{
    motor_status_t status = {0};

    motor_get_status(&status);

    const motor_wheel_status_t *wheel =
        wheel_status(&status, test_case->wheel);

    if (wheel == NULL)
    {
        return;
    }

    const int32_t delta_count =
        wheel->encoder_count - baseline_count;

    const float signed_speed =
        (float)test_case->direction *
        wheel->actual_speed_mm_s;

    const bool direction_count_ok =
        (test_case->direction > 0)
            ? (delta_count >= 2)
            : (delta_count <= -2);

    const bool speed_ok =
        signed_speed >= STARTUP_TEST_MOTION_SPEED_MM_S;

    if (direction_count_ok && speed_ok)
    {
        if (*motion_samples < UINT32_MAX)
        {
            (*motion_samples)++;
        }
    }
    else
    {
        *motion_samples = 0U;
    }

    ESP_LOGI(
        TAG,
        "sample wheel=%s dir=%+d pwm=%d t=%lu ms "
        "enc=%ld delta=%ld actual=%+.1f output=%+.1f "
        "ready=%d motion_samples=%lu",
        wheel_name(test_case->wheel),
        test_case->direction,
        pwm,
        (unsigned long)elapsed_ms,
        (long)wheel->encoder_count,
        (long)delta_count,
        (double)wheel->actual_speed_mm_s,
        (double)wheel->output_pwm,
        (int)wheel->encoder_ready,
        (unsigned long)*motion_samples);
}


static bool run_one_pwm_level(
    const startup_test_case_t *test_case,
    int pwm_magnitude)
{
    if (test_case == NULL)
    {
        return false;
    }

    const int signed_pwm =
        (test_case->direction >= 0)
            ? pwm_magnitude
            : -pwm_magnitude;

    set_single_wheel_pwm(
        test_case->wheel,
        (float)signed_pwm);

    vTaskDelay(
        pdMS_TO_TICKS(STARTUP_TEST_SAMPLE_MS));

    motor_status_t initial_status = {0};
    motor_get_status(&initial_status);

    const motor_wheel_status_t *initial_wheel =
        wheel_status(&initial_status, test_case->wheel);

    if (initial_wheel == NULL)
    {
        return false;
    }

    const int32_t baseline_count =
        initial_wheel->encoder_count;

    ESP_LOGI(
        TAG,
        "LEVEL START wheel=%s dir=%+d pwm=%d baseline_enc=%ld",
        wheel_name(test_case->wheel),
        test_case->direction,
        signed_pwm,
        (long)baseline_count);

    uint32_t motion_samples = 0U;
    const uint32_t sample_count =
        (STARTUP_TEST_HOLD_MS + STARTUP_TEST_SAMPLE_MS - 1U) /
        STARTUP_TEST_SAMPLE_MS;

    for (uint32_t sample = 0U;
         sample < sample_count;
         ++sample)
    {
        log_status(
            test_case,
            signed_pwm,
            (sample + 1U) * STARTUP_TEST_SAMPLE_MS,
            baseline_count,
            &motion_samples);

        vTaskDelay(
            pdMS_TO_TICKS(STARTUP_TEST_SAMPLE_MS));
    }

    motor_status_t final_status = {0};
    motor_get_status(&final_status);

    const motor_wheel_status_t *final_wheel =
        wheel_status(&final_status, test_case->wheel);

    if (final_wheel == NULL)
    {
        return false;
    }

    const int32_t final_delta =
        final_wheel->encoder_count - baseline_count;

    const bool started =
        motion_samples >= STARTUP_TEST_REQUIRED_MOTION_SAMPLES;

    ESP_LOGI(
        TAG,
        "LEVEL END wheel=%s dir=%+d pwm=%d "
        "enc=%ld delta=%ld actual=%+.1f "
        "motion_samples=%lu started=%s",
        wheel_name(test_case->wheel),
        test_case->direction,
        signed_pwm,
        (long)final_wheel->encoder_count,
        (long)final_delta,
        (double)final_wheel->actual_speed_mm_s,
        (unsigned long)motion_samples,
        started ? "YES" : "NO");

    stop_all_motors();
    return started;
}


static void run_test_case(
    const startup_test_case_t *test_case)
{
    if (test_case == NULL)
    {
        return;
    }

    ESP_LOGI(
        TAG,
        "CASE START wheel=%s dir=%+d range=%d..%d step=%d "
        "hold=%u ms settle=%u ms",
        wheel_name(test_case->wheel),
        test_case->direction,
        STARTUP_TEST_MIN_PWM,
        STARTUP_TEST_MAX_PWM,
        STARTUP_TEST_STEP_PWM,
        (unsigned)STARTUP_TEST_HOLD_MS,
        (unsigned)STARTUP_TEST_SETTLE_MS);

    int first_started_pwm = 0;

    for (int pwm = STARTUP_TEST_MIN_PWM;
         pwm <= STARTUP_TEST_MAX_PWM;
         pwm += STARTUP_TEST_STEP_PWM)
    {
        const bool started =
            run_one_pwm_level(
                test_case,
                pwm);

        if ((first_started_pwm == 0) && started)
        {
            first_started_pwm = pwm;
            ESP_LOGI(
                TAG,
                "BREAKAWAY candidate wheel=%s dir=%+d pwm=%d",
                wheel_name(test_case->wheel),
                test_case->direction,
                pwm);
        }
    }

    if (first_started_pwm == 0)
    {
        ESP_LOGW(
            TAG,
            "CASE RESULT wheel=%s dir=%+d no sustained motion in range",
            wheel_name(test_case->wheel),
            test_case->direction);
    }
    else
    {
        ESP_LOGI(
            TAG,
            "CASE RESULT wheel=%s dir=%+d first_sustained_pwm=%d "
            "recommended_startup_pwm=%d",
            wheel_name(test_case->wheel),
            test_case->direction,
            first_started_pwm,
            first_started_pwm + (first_started_pwm / 10));
    }
}


static void startup_test_task(void *arg)
{
    (void)arg;

    ESP_LOGI(
        TAG,
        "START raw PWM startup-threshold test; "
        "all=%d min=%d max=%d step=%d",
        STARTUP_TEST_RUN_ALL_DIRECTIONS,
        STARTUP_TEST_MIN_PWM,
        STARTUP_TEST_MAX_PWM,
        STARTUP_TEST_STEP_PWM);

    ESP_LOGW(
        TAG,
        "Raw PWM test can move the chassis. Keep the pen up, "
        "secure the chassis, and keep hands clear of wheels.");

    esp_err_t ret = motor_control_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "motor_control_init failed: %s",
            esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(300U));

    motor_status_t ready_status = {0};
    motor_get_status(&ready_status);

    ESP_LOGI(
        TAG,
        "Motor initialized=%d enabled=%d closed_loop_ready=%d raw=%d",
        (int)ready_status.initialized,
        (int)ready_status.enabled,
        (int)ready_status.closed_loop_ready,
        (int)ready_status.raw_debug_mode);

#if STARTUP_TEST_RUN_ALL_DIRECTIONS
    const startup_test_case_t cases[] =
    {
        {STARTUP_TEST_WHEEL_A, -1},
        {STARTUP_TEST_WHEEL_A, +1},
        {STARTUP_TEST_WHEEL_B, -1},
        {STARTUP_TEST_WHEEL_B, +1},
        {STARTUP_TEST_WHEEL_D, -1},
        {STARTUP_TEST_WHEEL_D, +1},
    };

    for (size_t i = 0U;
         i < (sizeof(cases) / sizeof(cases[0]));
         ++i)
    {
        run_test_case(&cases[i]);
    }
#else
    const startup_test_case_t test_case =
    {
        .wheel = STARTUP_TEST_WHEEL,
        .direction =
            (STARTUP_TEST_DIRECTION >= 0) ? +1 : -1,
    };

    run_test_case(&test_case);
#endif

    stop_all_motors();
    motor_control_enable(false);

    ESP_LOGI(
        TAG,
        "DONE raw PWM startup-threshold test");

    vTaskDelete(NULL);
}


void app_main(void)
{
    BaseType_t result =
        xTaskCreate(
            startup_test_task,
            "startup_test",
            4096,
            NULL,
            5,
            NULL);

    if (result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create startup test task");
    }
}
