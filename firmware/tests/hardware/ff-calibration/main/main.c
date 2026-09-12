/*
 * test_ff_loaded.c
 *
 * Loaded feed-forward calibration for the three-wheel chassis.
 *
 * Test principle:
 *   1. Put the complete chassis on its normal test surface.
 *   2. Apply the same signed raw PWM to A/B/D.  With the calibrated
 *      kinematics this is an in-place rotation, so all three motors are
 *      loaded simultaneously without requiring a long test track.
 *   3. Start at a high PWM, then sweep DOWN.  Descending data measures the
 *      running PWM/speed relation rather than the static breakaway PWM.
 *   4. Fit PWM = K * abs(wheel_speed_mm_s) + B independently for every
 *      wheel and direction.
 *
 * Raw PWM is intentional in this calibration file: measuring FF through
 * motor_set_velocity() would include the old FF, PID, and startup state
 * machine in the result and make the fitted parameters circular.
 *
 * This file is the entry point of the standalone ff-calibration ESP-IDF
 * project and links the shared tm_chassis component.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"


static const char *TAG = "ff_loaded_test";


/* ============================================================
 * Clearly editable test settings
 * ============================================================ */

/* Must be high enough to start all three wheels under simultaneous load. */
#ifndef FF_TEST_START_PWM
#define FF_TEST_START_PWM              400
#endif

/* Lowest level included in the descending sweep. */
#ifndef FF_TEST_END_PWM
#define FF_TEST_END_PWM                60
#endif

#ifndef FF_TEST_STEP_PWM
#define FF_TEST_STEP_PWM                20
#endif

/* Let speed settle after changing PWM, then collect the measurement window. */
#ifndef FF_TEST_SETTLE_MS
#define FF_TEST_SETTLE_MS              350U
#endif

#ifndef FF_TEST_SAMPLE_MS
#define FF_TEST_SAMPLE_MS              100U
#endif

#ifndef FF_TEST_SAMPLE_COUNT
#define FF_TEST_SAMPLE_COUNT             8U
#endif

/* Pause before reversing so the chassis and supply can settle. */
#ifndef FF_TEST_DIRECTION_PAUSE_MS
#define FF_TEST_DIRECTION_PAUSE_MS    1500U
#endif

/* Criteria for accepting one wheel's point into the linear regression. */
#ifndef FF_TEST_MIN_SPEED_MM_S
#define FF_TEST_MIN_SPEED_MM_S           5.0f
#endif

#ifndef FF_TEST_MIN_GOOD_SAMPLES
#define FF_TEST_MIN_GOOD_SAMPLES          6U
#endif

/*
 * At low non-zero targets motor_control treats approximately this speed as
 * the edge of its usable range.  The reported hold_B estimate makes the FF
 * output at this speed reach the measured continuous-running PWM.
 */
#ifndef FF_TEST_LOW_TARGET_MM_S
#define FF_TEST_LOW_TARGET_MM_S           5.0f
#endif


typedef struct
{
    float sum_speed;
    float min_speed;
    float max_speed;
    uint32_t good_samples;
    uint32_t samples;
    int32_t encoder_delta;
    bool valid;
} wheel_measurement_t;


typedef struct
{
    double sum_x;
    double sum_y;
    double sum_xx;
    double sum_xy;
    uint32_t points;
    int lowest_running_pwm;
} linear_fit_t;


typedef struct
{
    wheel_measurement_t A;
    wheel_measurement_t B;
    wheel_measurement_t D;
} level_measurement_t;


static const motor_wheel_status_t *wheel_from_status(
    const motor_status_t *status,
    size_t wheel_index)
{
    if (status == NULL)
    {
        return NULL;
    }

    switch (wheel_index)
    {
        case 0U:
            return &status->A;
        case 1U:
            return &status->B;
        case 2U:
            return &status->D;
        default:
            return NULL;
    }
}


static wheel_measurement_t *wheel_measurement_from_level(
    level_measurement_t *level,
    size_t wheel_index)
{
    if (level == NULL)
    {
        return NULL;
    }

    switch (wheel_index)
    {
        case 0U:
            return &level->A;
        case 1U:
            return &level->B;
        case 2U:
            return &level->D;
        default:
            return NULL;
    }
}


static void stop_raw(void)
{
    motor_debug_set_raw_pwm(0.0f, 0.0f, 0.0f);
}


static void set_rotation_pwm(int signed_pwm)
{
    const float pwm = (float)signed_pwm;
    motor_debug_set_raw_pwm(pwm, pwm, pwm);
}


static void update_measurement(
    wheel_measurement_t *measurement,
    const motor_wheel_status_t *wheel,
    int direction)
{
    if ((measurement == NULL) || (wheel == NULL))
    {
        return;
    }

    const float speed_in_command_direction =
        (float)direction * wheel->actual_speed_mm_s;

    if (measurement->samples == 0U)
    {
        measurement->min_speed = speed_in_command_direction;
        measurement->max_speed = speed_in_command_direction;
    }
    else
    {
        if (speed_in_command_direction < measurement->min_speed)
        {
            measurement->min_speed = speed_in_command_direction;
        }

        if (speed_in_command_direction > measurement->max_speed)
        {
            measurement->max_speed = speed_in_command_direction;
        }
    }

    measurement->sum_speed += speed_in_command_direction;
    measurement->samples++;

    if (speed_in_command_direction >= FF_TEST_MIN_SPEED_MM_S)
    {
        measurement->good_samples++;
    }
}


static float measurement_mean(const wheel_measurement_t *measurement)
{
    if ((measurement == NULL) || (measurement->samples == 0U))
    {
        return 0.0f;
    }

    return measurement->sum_speed / (float)measurement->samples;
}


static level_measurement_t measure_level(int signed_pwm)
{
    level_measurement_t result = {0};
    motor_status_t before = {0};
    motor_status_t current = {0};
    motor_status_t after = {0};

    const int direction = (signed_pwm >= 0) ? +1 : -1;

    set_rotation_pwm(signed_pwm);
    vTaskDelay(pdMS_TO_TICKS(FF_TEST_SETTLE_MS));

    motor_get_status(&before);

    for (uint32_t sample = 0U;
         sample < FF_TEST_SAMPLE_COUNT;
         ++sample)
    {
        vTaskDelay(pdMS_TO_TICKS(FF_TEST_SAMPLE_MS));
        motor_get_status(&current);

        for (size_t wheel = 0U; wheel < 3U; ++wheel)
        {
            update_measurement(
                wheel_measurement_from_level(&result, wheel),
                wheel_from_status(&current, wheel),
                direction);
        }
    }

    motor_get_status(&after);

    const motor_wheel_status_t *before_wheels[3] =
    {
        &before.A,
        &before.B,
        &before.D,
    };

    const motor_wheel_status_t *after_wheels[3] =
    {
        &after.A,
        &after.B,
        &after.D,
    };

    for (size_t wheel = 0U; wheel < 3U; ++wheel)
    {
        wheel_measurement_t *measurement =
            wheel_measurement_from_level(&result, wheel);

        measurement->encoder_delta =
            after_wheels[wheel]->encoder_count -
            before_wheels[wheel]->encoder_count;

        const bool encoder_direction_ok =
            (direction > 0)
                ? (measurement->encoder_delta > 0)
                : (measurement->encoder_delta < 0);

        measurement->valid =
            encoder_direction_ok &&
            (measurement->good_samples >= FF_TEST_MIN_GOOD_SAMPLES) &&
            (measurement_mean(measurement) >= FF_TEST_MIN_SPEED_MM_S);
    }

    return result;
}


static void fit_add(
    linear_fit_t *fit,
    const wheel_measurement_t *measurement,
    int pwm_magnitude)
{
    if ((fit == NULL) ||
        (measurement == NULL) ||
        !measurement->valid)
    {
        return;
    }

    const double x = (double)measurement_mean(measurement);
    const double y = (double)pwm_magnitude;

    fit->sum_x += x;
    fit->sum_y += y;
    fit->sum_xx += x * x;
    fit->sum_xy += x * y;
    fit->points++;

    if ((fit->lowest_running_pwm == 0) ||
        (pwm_magnitude < fit->lowest_running_pwm))
    {
        fit->lowest_running_pwm = pwm_magnitude;
    }
}


static void print_level(
    int direction,
    int pwm_magnitude,
    const level_measurement_t *level)
{
    ESP_LOGI(
        TAG,
        "FF_DATA dir=%+d pwm=%d "
        "A{mean=%.1f min=%.1f max=%.1f denc=%ld good=%lu/%lu ok=%d} "
        "B{mean=%.1f min=%.1f max=%.1f denc=%ld good=%lu/%lu ok=%d} "
        "D{mean=%.1f min=%.1f max=%.1f denc=%ld good=%lu/%lu ok=%d}",
        direction,
        pwm_magnitude,
        (double)measurement_mean(&level->A),
        (double)level->A.min_speed,
        (double)level->A.max_speed,
        (long)level->A.encoder_delta,
        (unsigned long)level->A.good_samples,
        (unsigned long)level->A.samples,
        (int)level->A.valid,
        (double)measurement_mean(&level->B),
        (double)level->B.min_speed,
        (double)level->B.max_speed,
        (long)level->B.encoder_delta,
        (unsigned long)level->B.good_samples,
        (unsigned long)level->B.samples,
        (int)level->B.valid,
        (double)measurement_mean(&level->D),
        (double)level->D.min_speed,
        (double)level->D.max_speed,
        (long)level->D.encoder_delta,
        (unsigned long)level->D.good_samples,
        (unsigned long)level->D.samples,
        (int)level->D.valid);
}


static void print_fit_result(
    const char *wheel_name,
    int direction,
    const linear_fit_t *fit)
{
    if ((fit == NULL) || (fit->points < 2U))
    {
        ESP_LOGE(
            TAG,
            "FF_RESULT wheel=%s dir=%+d INVALID points=%lu; "
            "increase START_PWM or inspect encoder/power",
            wheel_name,
            direction,
            (unsigned long)((fit != NULL) ? fit->points : 0U));
        return;
    }

    const double n = (double)fit->points;
    const double denominator =
        n * fit->sum_xx - fit->sum_x * fit->sum_x;

    if ((denominator > -0.000001) &&
        (denominator < 0.000001))
    {
        ESP_LOGE(
            TAG,
            "FF_RESULT wheel=%s dir=%+d INVALID speed span too small",
            wheel_name,
            direction);
        return;
    }

    const double k =
        (n * fit->sum_xy - fit->sum_x * fit->sum_y) /
        denominator;

    const double fitted_b =
        (fit->sum_y - k * fit->sum_x) /
        n;

    const double hold_b =
        (double)fit->lowest_running_pwm -
        k * (double)FF_TEST_LOW_TARGET_MM_S;

    const double recommended_b =
        (hold_b > fitted_b) ? hold_b : fitted_b;

    ESP_LOGI(
        TAG,
        "FF_RESULT wheel=%s dir=%+d points=%lu "
        "fit_K=%.4f fit_B=%.1f lowest_running_pwm=%d "
        "hold_B=%.1f recommended_K=%.4f recommended_B=%.1f",
        wheel_name,
        direction,
        (unsigned long)fit->points,
        k,
        fitted_b,
        fit->lowest_running_pwm,
        hold_b,
        k,
        recommended_b);
}


static void run_direction(int direction)
{
    linear_fit_t fits[3] = {0};
    const char *wheel_names[3] = {"A", "B", "D"};

    ESP_LOGI(
        TAG,
        "CASE START dir=%+d PWM=%d..%d descending step=%d",
        direction,
        FF_TEST_START_PWM,
        FF_TEST_END_PWM,
        FF_TEST_STEP_PWM);

    /* A stopped start ensures this direction also verifies loaded breakaway. */
    stop_raw();
    vTaskDelay(pdMS_TO_TICKS(FF_TEST_DIRECTION_PAUSE_MS));

    for (int pwm = FF_TEST_START_PWM;
         pwm >= FF_TEST_END_PWM;
         pwm -= FF_TEST_STEP_PWM)
    {
        const level_measurement_t level =
            measure_level(direction * pwm);

        print_level(direction, pwm, &level);

        fit_add(&fits[0], &level.A, pwm);
        fit_add(&fits[1], &level.B, pwm);
        fit_add(&fits[2], &level.D, pwm);
    }

    stop_raw();

    for (size_t wheel = 0U; wheel < 3U; ++wheel)
    {
        print_fit_result(
            wheel_names[wheel],
            direction,
            &fits[wheel]);
    }

    ESP_LOGI(TAG, "CASE END dir=%+d", direction);
}


static void ff_loaded_test_task(void *arg)
{
    (void)arg;

    ESP_LOGW(
        TAG,
        "Vehicle will ROTATE on the floor. Raise the pen, clear the area, "
        "keep hands away, and prevent the USB cable from winding up.");

    ESP_LOGI(
        TAG,
        "START loaded FF test start=%d end=%d step=%d settle=%lu ms "
        "samples=%lu x %lu ms",
        FF_TEST_START_PWM,
        FF_TEST_END_PWM,
        FF_TEST_STEP_PWM,
        (unsigned long)FF_TEST_SETTLE_MS,
        (unsigned long)FF_TEST_SAMPLE_COUNT,
        (unsigned long)FF_TEST_SAMPLE_MS);

    const esp_err_t ret = motor_control_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "motor_control_init failed: %s",
            esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500U));

    motor_status_t status = {0};
    motor_get_status(&status);

    if (!status.closed_loop_ready)
    {
        ESP_LOGE(
            TAG,
            "Encoders/closed loop are not ready; calibration aborted");
        motor_control_enable(false);
        vTaskDelete(NULL);
        return;
    }

    /* Same-sign wheel speeds are pure rotation with the current kinematics. */
    run_direction(+1);
    run_direction(-1);

    stop_raw();
    vTaskDelay(pdMS_TO_TICKS(300U));
    motor_control_enable(false);

    ESP_LOGI(
        TAG,
        "DONE. Copy all six FF_RESULT lines and the nearby FF_DATA lines.");

    vTaskDelete(NULL);
}


void app_main(void)
{
    const BaseType_t created =
        xTaskCreate(
            ff_loaded_test_task,
            "ff_loaded_test",
            6144,
            NULL,
            5,
            NULL);

    if (created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create FF calibration task");
    }
}
