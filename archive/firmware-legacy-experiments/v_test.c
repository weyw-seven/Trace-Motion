/*
 * set_motor_discrete_speed_test.c
 *
 * Scan selected normalized Set_motor() command points and print:
 *
 *   1) normalized command vx/vy/w
 *   2) physical chassis target published by motor_control
 *   3) each wheel target / measured speed / PWM
 *   4) chassis velocity reconstructed from measured wheel speeds
 *
 * Coordinate convention:
 *   +vx = forward
 *   +vy = left
 *   +w  = CCW
 *
 * Set_motor() normalized input:
 *   vx, vy, w in [-1, +1]
 *
 * Current physical scales:
 *   vx = norm * MOTOR_MAX_VX_MM_S
 *   vy = norm * MOTOR_MAX_VY_MM_S
 *   w  = norm * MOTOR_MAX_W_RAD_S
 *
 * IMPORTANT:
 *   This program actively drives the chassis when TEST_ENABLE = 1.
 *
 * Recommended:
 *   First run with wheels lifted.
 *   Then, if safe, repeat on the floor to see loaded speed.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

#define CHASSIS_KINEMATICS_IMPLEMENTATION
#include "chassis_kinematics.h"


/* ============================================================
 * Safety gate
 * ============================================================ */

#ifndef TEST_ENABLE
#define TEST_ENABLE                         1
#endif


/* ============================================================
 * Select test axis
 * ============================================================ */

#define TEST_AXIS_VX                        1
#define TEST_AXIS_VY                        2
#define TEST_AXIS_W                         3
#define TEST_AXIS_ALL                       4

#ifndef TEST_AXIS
#define TEST_AXIS                           3
#endif


/* ============================================================
 * Timing
 * ============================================================ */

/*
 * Hold each command long enough for wheel speed to settle.
 */
#ifndef TEST_HOLD_MS
#define TEST_HOLD_MS                        1800U
#endif

/*
 * Stop between points.
 */
#ifndef TEST_STOP_MS
#define TEST_STOP_MS                        1000U
#endif

/*
 * Ignore the early startup transient before averaging.
 */
#ifndef TEST_SETTLE_MS
#define TEST_SETTLE_MS                       600U
#endif

#ifndef TEST_SAMPLE_MS
#define TEST_SAMPLE_MS                        50U
#endif


/* ============================================================
 * Discrete normalized points
 *
 * Includes 卤0.3 and 卤0.4 because these are particularly relevant
 * to the previous chassis tests.
 * ============================================================ */

static const float g_points[] = {
    -1.0f,
    -0.8f,
    -0.6f,
    -0.4f,
    -0.3f,
    -0.2f,
     0.0f,
     0.2f,
     0.3f,
     0.4f,
     0.6f,
     0.8f,
     1.0f
};

#define POINT_COUNT \
    (sizeof(g_points) / sizeof(g_points[0]))


static const char *TAG = "SET_MOTOR_SCAN";


typedef struct
{
    uint32_t n;

    double a_actual_sum;
    double b_actual_sum;
    double d_actual_sum;

    double a_pwm_sum;
    double b_pwm_sum;
    double d_pwm_sum;

    double vx_sum;
    double vy_sum;
    double w_sum;

} average_t;


static void average_push(
    average_t *avg,
    const motor_status_t *motor,
    const chassis_velocity_t *body)
{
    if ((avg == NULL) ||
        (motor == NULL) ||
        (body == NULL))
    {
        return;
    }

    avg->n++;

    avg->a_actual_sum +=
        motor->A.actual_speed_mm_s;

    avg->b_actual_sum +=
        motor->B.actual_speed_mm_s;

    avg->d_actual_sum +=
        motor->D.actual_speed_mm_s;

    avg->a_pwm_sum +=
        motor->A.output_pwm;

    avg->b_pwm_sum +=
        motor->B.output_pwm;

    avg->d_pwm_sum +=
        motor->D.output_pwm;

    avg->vx_sum +=
        body->vx_mm_s;

    avg->vy_sum +=
        body->vy_mm_s;

    avg->w_sum +=
        body->w_rad_s;
}


static void command_for_point(
    int axis,
    float point,
    float *vx,
    float *vy,
    float *w)
{
    *vx = 0.0f;
    *vy = 0.0f;
    *w  = 0.0f;

    if (axis == TEST_AXIS_VX)
    {
        *vx = point;
    }
    else if (axis == TEST_AXIS_VY)
    {
        *vy = point;
    }
    else if (axis == TEST_AXIS_W)
    {
        *w = point;
    }
}


static const char *axis_name(
    int axis)
{
    switch (axis)
    {
        case TEST_AXIS_VX:
            return "VX";

        case TEST_AXIS_VY:
            return "VY";

        case TEST_AXIS_W:
            return "W";

        default:
            return "?";
    }
}


static void print_theoretical_scale(void)
{
    ESP_LOGI(
        TAG,
        "============================================================");

    ESP_LOGI(
        TAG,
        "Set_motor normalized -> physical target:");

    ESP_LOGI(
        TAG,
        "  vx = norm * %.1f mm/s",
        (double)MOTOR_MAX_VX_MM_S);

    ESP_LOGI(
        TAG,
        "  vy = norm * %.1f mm/s",
        (double)MOTOR_MAX_VY_MM_S);

    ESP_LOGI(
        TAG,
        "  w  = norm * %.3f rad/s = norm * %.2f deg/s",
        (double)MOTOR_MAX_W_RAD_S,
        (double)(
            MOTOR_MAX_W_RAD_S *
            180.0f /
            3.14159265358979323846f));

    ESP_LOGI(
        TAG,
        "============================================================");
}


static void run_one_point(
    int axis,
    float point)
{
    float vx_norm = 0.0f;
    float vy_norm = 0.0f;
    float w_norm  = 0.0f;

    command_for_point(
        axis,
        point,
        &vx_norm,
        &vy_norm,
        &w_norm);

    ESP_LOGW(
        TAG,
        "POINT %s = %+.2f",
        axis_name(axis),
        (double)point);

    Set_motor(
        vx_norm,
        vy_norm,
        w_norm);

    /*
     * Let startup transient pass.
     */
    vTaskDelay(
        pdMS_TO_TICKS(
            TEST_SETTLE_MS));

    average_t avg = {0};

    const uint32_t measure_ms =
        (TEST_HOLD_MS > TEST_SETTLE_MS)
        ?
        (TEST_HOLD_MS - TEST_SETTLE_MS)
        :
        0U;

    uint32_t elapsed_ms = 0U;

    motor_status_t last_motor = {0};
    chassis_velocity_t last_body = {0};

    while (elapsed_ms <
           measure_ms)
    {
        motor_status_t motor = {0};

        motor_get_status(
            &motor);

        chassis_wheel_velocity_t wheel = {
            .a_mm_s =
                motor.A.actual_speed_mm_s,

            .b_mm_s =
                motor.B.actual_speed_mm_s,

            .d_mm_s =
                motor.D.actual_speed_mm_s
        };

        chassis_velocity_t body = {0};

        const esp_err_t ret =
            chassis_kinematics_forward(
                &wheel,
                &body);

        if (ret == ESP_OK)
        {
            average_push(
                &avg,
                &motor,
                &body);

            last_motor =
                motor;

            last_body =
                body;
        }

        vTaskDelay(
            pdMS_TO_TICKS(
                TEST_SAMPLE_MS));

        elapsed_ms +=
            TEST_SAMPLE_MS;
    }


    motor_stop();

    const double inv_n =
        (avg.n > 0U)
        ?
        (1.0 / (double)avg.n)
        :
        0.0;

    /*
     * Physical target published by motor_control.
     */
    const double target_vx =
        last_motor.target_vx_mm_s;

    const double target_vy =
        last_motor.target_vy_mm_s;

    const double target_w =
        last_motor.target_w_rad_s;

    const double target_w_dps =
        target_w *
        180.0 /
        3.14159265358979323846;

    const double avg_vx =
        avg.vx_sum *
        inv_n;

    const double avg_vy =
        avg.vy_sum *
        inv_n;

    const double avg_w =
        avg.w_sum *
        inv_n;

    const double avg_w_dps =
        avg_w *
        180.0 /
        3.14159265358979323846;

    ESP_LOGI(
        TAG,
        "RESULT %s %+.2f | "
        "target chassis=[vx=%+.1f vy=%+.1f w=%+.3f rad/s (%+.1f deg/s)]",
        axis_name(axis),
        (double)point,
        target_vx,
        target_vy,
        target_w,
        target_w_dps);

    ESP_LOGI(
        TAG,
        "ACTUAL chassis(avg) | "
        "vx=%+.1f mm/s vy=%+.1f mm/s "
        "w=%+.3f rad/s (%+.1f deg/s)",
        avg_vx,
        avg_vy,
        avg_w,
        avg_w_dps);

    ESP_LOGI(
        TAG,
        "WHEEL avg | "
        "A=%+.1f mm/s PWM=%+.1f | "
        "B=%+.1f mm/s PWM=%+.1f | "
        "D=%+.1f mm/s PWM=%+.1f",
        avg.a_actual_sum * inv_n,
        avg.a_pwm_sum * inv_n,
        avg.b_actual_sum * inv_n,
        avg.b_pwm_sum * inv_n,
        avg.d_actual_sum * inv_n,
        avg.d_pwm_sum * inv_n);

    ESP_LOGI(
        TAG,
        "WHEEL target(last) | "
        "A=%+.1f B=%+.1f D=%+.1f mm/s",
        (double)last_motor.A.target_speed_mm_s,
        (double)last_motor.B.target_speed_mm_s,
        (double)last_motor.D.target_speed_mm_s);

    ESP_LOGI(
        TAG,
        "CSV,SET_MOTOR,"
        "axis,%s,"
        "norm,%.3f,"
        "target_vx,%.6f,"
        "target_vy,%.6f,"
        "target_w_rad_s,%.7f,"
        "target_w_deg_s,%.6f,"
        "actual_vx,%.6f,"
        "actual_vy,%.6f,"
        "actual_w_rad_s,%.7f,"
        "actual_w_deg_s,%.6f,"
        "A_target,%.6f,A_actual,%.6f,A_pwm,%.6f,"
        "B_target,%.6f,B_actual,%.6f,B_pwm,%.6f,"
        "D_target,%.6f,D_actual,%.6f,D_pwm,%.6f,"
        "samples,%u",
        axis_name(axis),
        (double)point,
        target_vx,
        target_vy,
        target_w,
        target_w_dps,
        avg_vx,
        avg_vy,
        avg_w,
        avg_w_dps,
        (double)last_motor.A.target_speed_mm_s,
        avg.a_actual_sum * inv_n,
        avg.a_pwm_sum * inv_n,
        (double)last_motor.B.target_speed_mm_s,
        avg.b_actual_sum * inv_n,
        avg.b_pwm_sum * inv_n,
        (double)last_motor.D.target_speed_mm_s,
        avg.d_actual_sum * inv_n,
        avg.d_pwm_sum * inv_n,
        (unsigned)avg.n);

    ESP_LOGI(
        TAG,
        "------------------------------------------------------------");

    vTaskDelay(
        pdMS_TO_TICKS(
            TEST_STOP_MS));
}


static void run_axis(
    int axis)
{
    ESP_LOGW(
        TAG,
        "START AXIS %s",
        axis_name(axis));

    for (size_t i = 0;
         i < POINT_COUNT;
         ++i)
    {
        run_one_point(
            axis,
            g_points[i]);
    }

    motor_stop();

    ESP_LOGW(
        TAG,
        "DONE AXIS %s",
        axis_name(axis));
}


void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Set_motor discrete normalized-speed scan");

    print_theoretical_scale();

    esp_err_t ret =
        motor_control_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "motor_control_init failed: %s",
            esp_err_to_name(ret));
        return;
    }

    if (!motor_control_is_ready())
    {
        ESP_LOGE(
            TAG,
            "motor control / encoder closed loop not ready");
        return;
    }


#if !TEST_ENABLE

    ESP_LOGW(
        TAG,
        "ACTIVE TEST DISABLED.");

    ESP_LOGW(
        TAG,
        "Set TEST_ENABLE = 1 after selecting TEST_AXIS.");

    motor_stop();

    for (;;)
    {
        vTaskDelay(
            pdMS_TO_TICKS(
                1000U));
    }

#else

    ESP_LOGW(
        TAG,
        "ACTIVE TEST ENABLED.");

    vTaskDelay(
        pdMS_TO_TICKS(
            1500U));


#if TEST_AXIS == TEST_AXIS_ALL

    run_axis(
        TEST_AXIS_VX);

    run_axis(
        TEST_AXIS_VY);

    run_axis(
        TEST_AXIS_W);

#else

    run_axis(
        TEST_AXIS);

#endif


    motor_stop();

    ESP_LOGI(
        TAG,
        "ALL SELECTED TESTS FINISHED.");

    for (;;)
    {
        vTaskDelay(
            pdMS_TO_TICKS(
                1000U));
    }

#endif
}