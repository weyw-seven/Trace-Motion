/*
 * motor_control_test_main.c
 *
 * ESP-IDF test program for motor_control.h
 *
 * IMPORTANT:
 *   1) Put the chassis on a stand so all three wheels are off the ground
 *      before enabling MOTOR_TEST_ENABLE_MOTION.
 *   2) MOTOR_CONTROL_IMPLEMENTATION must be defined in exactly ONE .c file.
 *   3) This test assumes Set_motor() uses normalized commands [-1, 1],
 *      matching the current implementation in motor_control.h.
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

/* --------------------------------------------------------------------------
 * Test configuration
 * -------------------------------------------------------------------------- */

/*
 * 0: only API/state tests; no intentional motor motion.
 * 1: run raw-PWM polarity tests and chassis motion tests.
 *
 * Enable only after lifting the chassis so the wheels are free.
 */
#ifndef MOTOR_TEST_ENABLE_MOTION
#define MOTOR_TEST_ENABLE_MOTION        1
#endif

#ifndef MOTOR_TEST_RAW_PWM
#define MOTOR_TEST_RAW_PWM              220.0f
#endif

#ifndef MOTOR_TEST_RAW_RUN_MS
#define MOTOR_TEST_RAW_RUN_MS           800U
#endif

#ifndef MOTOR_TEST_COMMAND
#define MOTOR_TEST_COMMAND              0.20f
#endif

#ifndef MOTOR_TEST_CHASSIS_RUN_MS
#define MOTOR_TEST_CHASSIS_RUN_MS       1000U
#endif

#ifndef MOTOR_TEST_SETTLE_MS
#define MOTOR_TEST_SETTLE_MS            250U
#endif

#ifndef MOTOR_TEST_STATUS_WAIT_MS
#define MOTOR_TEST_STATUS_WAIT_MS       40U
#endif

#ifndef MOTOR_TEST_MIN_ENCODER_DELTA
#define MOTOR_TEST_MIN_ENCODER_DELTA    3
#endif

static const char *TAG = "MOTOR_TEST";

static int s_pass = 0;
static int s_fail = 0;

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static bool nearf_abs(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

static void check_bool(const char *name, bool condition)
{
    if (condition) {
        ++s_pass;
        ESP_LOGI(TAG, "[PASS] %s", name);
    } else {
        ++s_fail;
        ESP_LOGE(TAG, "[FAIL] %s", name);
    }
}

static void print_status(const char *label)
{
    motor_status_t s = {0};
    motor_get_status(&s);

    ESP_LOGI(TAG,
             "%s | init=%d enable=%d ready=%d raw=%d | "
             "cmd[vx=%.2f mm/s, vy=%.2f mm/s, w=%.3f rad/s]",
             label,
             (int)s.initialized,
             (int)s.enabled,
             (int)s.closed_loop_ready,
             (int)s.raw_debug_mode,
             s.target_vx_mm_s,
             s.target_vy_mm_s,
             s.target_w_rad_s);

    ESP_LOGI(TAG,
             "  A: target=%8.2f actual=%8.2f count=%" PRId32 " pwm=%8.2f",
             s.A.target_speed_mm_s,
             s.A.actual_speed_mm_s,
             s.A.encoder_count,
             s.A.output_pwm);

    ESP_LOGI(TAG,
             "  B: target=%8.2f actual=%8.2f count=%" PRId32 " pwm=%8.2f",
             s.B.target_speed_mm_s,
             s.B.actual_speed_mm_s,
             s.B.encoder_count,
             s.B.output_pwm);

    ESP_LOGI(TAG,
             "  D: target=%8.2f actual=%8.2f count=%" PRId32 " pwm=%8.2f",
             s.D.target_speed_mm_s,
             s.D.actual_speed_mm_s,
             s.D.encoder_count,
             s.D.output_pwm);
}

static void get_status_after_task(motor_status_t *s)
{
    delay_ms(MOTOR_TEST_STATUS_WAIT_MS);
    motor_get_status(s);
}

static void print_configuration(void)
{
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, "motor_control.h configuration");
    ESP_LOGI(TAG, "============================================================");

    ESP_LOGI(TAG,
             "Public command scale: vx=%.1f mm/s, vy=%.1f mm/s, w=%.3f rad/s",
             (double)MOTOR_MAX_VX_MM_S,
             (double)MOTOR_MAX_VY_MM_S,
             (double)MOTOR_MAX_W_RAD_S);

    ESP_LOGI(TAG,
             "Geometry: wheel_diameter=%.2f mm, chassis_radius=%.2f mm",
             (double)MOTOR_WHEEL_DIAMETER_MM,
             (double)MOTOR_CHASSIS_RADIUS_MM);

    ESP_LOGI(TAG,
             "CPR: A=%.1f, B=%.1f, D=%.1f",
             (double)MOTOR_A_ENCODER_CPR,
             (double)MOTOR_B_ENCODER_CPR,
             (double)MOTOR_D_ENCODER_CPR);

    ESP_LOGI(TAG,
             "Motor polarity:   A=%+.0f B=%+.0f D=%+.0f",
             (double)MOTOR_A_POLARITY,
             (double)MOTOR_B_POLARITY,
             (double)MOTOR_D_POLARITY);

    ESP_LOGI(TAG,
             "Encoder polarity: A=%+.0f B=%+.0f D=%+.0f",
             (double)MOTOR_A_ENCODER_POLARITY,
             (double)MOTOR_B_ENCODER_POLARITY,
             (double)MOTOR_D_ENCODER_POLARITY);

    ESP_LOGI(TAG,
             "IK A: [%+.7f, %+.7f, %+.7f]",
             (double)MOTOR_A_KX,
             (double)MOTOR_A_KY,
             (double)MOTOR_A_KW);

    ESP_LOGI(TAG,
             "IK B: [%+.7f, %+.7f, %+.7f]",
             (double)MOTOR_B_KX,
             (double)MOTOR_B_KY,
             (double)MOTOR_B_KW);

    ESP_LOGI(TAG,
             "IK D: [%+.7f, %+.7f, %+.7f]",
             (double)MOTOR_D_KX,
             (double)MOTOR_D_KY,
             (double)MOTOR_D_KW);

    ESP_LOGW(TAG,
             "Current header comment says +vx=forward, +vy=left, +w=CCW.");
    ESP_LOGW(TAG,
             "Verify the physical chassis direction during the six chassis tests.");
}

/* --------------------------------------------------------------------------
 * API / command-scaling tests
 * -------------------------------------------------------------------------- */

static void test_api_state_and_scaling(void)
{
    motor_status_t s = {0};

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== API / state tests ==========");

    get_status_after_task(&s);

    check_bool("status.initialized == true", s.initialized);
    check_bool("status.enabled == true after init", s.enabled);
    check_bool("status.closed_loop_ready matches motor_control_is_ready()",
               s.closed_loop_ready == motor_control_is_ready());

    /*
     * Disable hardware output before testing non-zero command scaling.
     * Set_motor() still stores the command while disabled, so the API can be
     * verified without intentionally moving the motors.
     */
    motor_control_enable(false);
    get_status_after_task(&s);

    check_bool("motor_control_enable(false) -> enabled=false",
               !s.enabled);

    check_bool("motor_control_is_ready is independent of enabled state",
               motor_control_is_ready() == s.closed_loop_ready);

    /*
     * Set_motor() implementation accepts normalized [-1,1] commands.
     * Verify the public status contains scaled physical units.
     */
    Set_motor(0.25f, -0.50f, 0.40f);
    get_status_after_task(&s);

    check_bool("Set_motor vx normalized scaling",
               nearf_abs(s.target_vx_mm_s,
                         0.25f * MOTOR_MAX_VX_MM_S,
                         0.5f));

    check_bool("Set_motor vy normalized scaling",
               nearf_abs(s.target_vy_mm_s,
                         -0.50f * MOTOR_MAX_VY_MM_S,
                         0.5f));

    check_bool("Set_motor w normalized scaling",
               nearf_abs(s.target_w_rad_s,
                         0.40f * MOTOR_MAX_W_RAD_S,
                         0.01f));

    /*
     * Translation vector normalization:
     * (1,1) is normalized to (1/sqrt(2), 1/sqrt(2)).
     */
    Set_motor(1.0f, 1.0f, 0.0f);
    get_status_after_task(&s);

    const float inv_sqrt2 = 0.70710678f;

    check_bool("translation vector magnitude limited to 1",
               nearf_abs(s.target_vx_mm_s,
                         inv_sqrt2 * MOTOR_MAX_VX_MM_S,
                         1.0f) &&
               nearf_abs(s.target_vy_mm_s,
                         inv_sqrt2 * MOTOR_MAX_VY_MM_S,
                         1.0f));

    /*
     * Raw mode should be entered by motor_debug_set_raw_pwm(),
     * and any later Set_motor() should exit raw mode.
     */
    motor_debug_set_raw_pwm(0.0f, 0.0f, 0.0f);
    get_status_after_task(&s);
    check_bool("motor_debug_set_raw_pwm enters raw mode",
               s.raw_debug_mode);

    Set_motor(0.0f, 0.0f, 0.0f);
    get_status_after_task(&s);
    check_bool("Set_motor exits raw mode",
               !s.raw_debug_mode);

    /*
     * While disabled, a Set_motor command is stored but output remains zero.
     * This verifies enable gating without moving the wheels.
     */
    Set_motor(0.20f, 0.0f, 0.0f);
    get_status_after_task(&s);

    check_bool("disabled controller keeps all PWM at zero",
               nearf_abs(s.A.output_pwm, 0.0f, 0.1f) &&
               nearf_abs(s.B.output_pwm, 0.0f, 0.1f) &&
               nearf_abs(s.D.output_pwm, 0.0f, 0.1f));

    motor_control_enable(true);
    motor_stop();
    delay_ms(MOTOR_TEST_SETTLE_MS);
    print_status("after re-enable + stop");
}

/* --------------------------------------------------------------------------
 * Raw PWM polarity tests
 * -------------------------------------------------------------------------- */

typedef enum {
    TEST_WHEEL_A,
    TEST_WHEEL_B,
    TEST_WHEEL_D,
} test_wheel_t;

static const char *wheel_name(test_wheel_t wheel)
{
    switch (wheel) {
    case TEST_WHEEL_A: return "A";
    case TEST_WHEEL_B: return "B";
    case TEST_WHEEL_D: return "D";
    default:           return "?";
    }
}

static void set_one_wheel_raw(test_wheel_t wheel, float pwm)
{
    switch (wheel) {
    case TEST_WHEEL_A:
        motor_debug_set_raw_pwm(pwm, 0.0f, 0.0f);
        break;
    case TEST_WHEEL_B:
        motor_debug_set_raw_pwm(0.0f, pwm, 0.0f);
        break;
    case TEST_WHEEL_D:
        motor_debug_set_raw_pwm(0.0f, 0.0f, pwm);
        break;
    }
}

static int32_t wheel_count(const motor_status_t *s, test_wheel_t wheel)
{
    switch (wheel) {
    case TEST_WHEEL_A: return s->A.encoder_count;
    case TEST_WHEEL_B: return s->B.encoder_count;
    case TEST_WHEEL_D: return s->D.encoder_count;
    default:           return 0;
    }
}

static float wheel_actual_speed(const motor_status_t *s, test_wheel_t wheel)
{
    switch (wheel) {
    case TEST_WHEEL_A: return s->A.actual_speed_mm_s;
    case TEST_WHEEL_B: return s->B.actual_speed_mm_s;
    case TEST_WHEEL_D: return s->D.actual_speed_mm_s;
    default:           return 0.0f;
    }
}

static void run_one_raw_polarity_test(test_wheel_t wheel)
{
    motor_status_t s0 = {0};
    motor_status_t s1 = {0};
    motor_status_t s2 = {0};
    motor_status_t s3 = {0};

    const char *name = wheel_name(wheel);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "---- Wheel %s raw polarity test ----", name);

    motor_debug_set_raw_pwm(0.0f, 0.0f, 0.0f);
    delay_ms(MOTOR_TEST_SETTLE_MS);
    motor_get_status(&s0);

    ESP_LOGW(TAG,
             "OBSERVE wheel %s now: +raw PWM should be your chosen POSITIVE wheel direction.",
             name);

    set_one_wheel_raw(wheel, +MOTOR_TEST_RAW_PWM);
    delay_ms(MOTOR_TEST_RAW_RUN_MS);
    motor_get_status(&s1);

    set_one_wheel_raw(wheel, 0.0f);
    delay_ms(MOTOR_TEST_SETTLE_MS);

    const int32_t dpos = wheel_count(&s1, wheel) - wheel_count(&s0, wheel);

    ESP_LOGI(TAG,
             "Wheel %s +PWM: encoder delta=%" PRId32 ", actual_speed=%.2f mm/s",
             name,
             dpos,
             (double)wheel_actual_speed(&s1, wheel));

    /*
     * Start the negative test from a fresh count snapshot.
     */
    motor_get_status(&s2);

    ESP_LOGW(TAG,
             "OBSERVE wheel %s now: -raw PWM should reverse the wheel.",
             name);

    set_one_wheel_raw(wheel, -MOTOR_TEST_RAW_PWM);
    delay_ms(MOTOR_TEST_RAW_RUN_MS);
    motor_get_status(&s3);

    set_one_wheel_raw(wheel, 0.0f);
    delay_ms(MOTOR_TEST_SETTLE_MS);

    const int32_t dneg = wheel_count(&s3, wheel) - wheel_count(&s2, wheel);

    ESP_LOGI(TAG,
             "Wheel %s -PWM: encoder delta=%" PRId32 ", actual_speed=%.2f mm/s",
             name,
             dneg,
             (double)wheel_actual_speed(&s3, wheel));

    char test_name[96];

    snprintf(test_name, sizeof(test_name),
             "wheel %s +logical PWM -> positive logical encoder count", name);
    check_bool(test_name, dpos >= MOTOR_TEST_MIN_ENCODER_DELTA);

    snprintf(test_name, sizeof(test_name),
             "wheel %s -logical PWM -> negative logical encoder count", name);
    check_bool(test_name, dneg <= -MOTOR_TEST_MIN_ENCODER_DELTA);

    if (dpos <= -MOTOR_TEST_MIN_ENCODER_DELTA &&
        dneg >= MOTOR_TEST_MIN_ENCODER_DELTA) {
        ESP_LOGE(TAG,
                 "Wheel %s motor/encoder logical polarities are opposite. "
                 "After confirming physical +wheel direction, flip MOTOR_%s_ENCODER_POLARITY.",
                 name, name);
    } else if (labs((long)dpos) < MOTOR_TEST_MIN_ENCODER_DELTA ||
               labs((long)dneg) < MOTOR_TEST_MIN_ENCODER_DELTA) {
        ESP_LOGE(TAG,
                 "Wheel %s encoder movement is too small. Check encoder wiring, CPR, PCNT pins, "
                 "PWM deadband, or increase MOTOR_TEST_RAW_PWM cautiously.",
                 name);
    }
}

/* --------------------------------------------------------------------------
 * Chassis kinematics tests
 * -------------------------------------------------------------------------- */

static void run_chassis_command(const char *name,
                                float vx,
                                float vy,
                                float w,
                                const char *physical_expectation)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "---- Chassis test: %s ----", name);
    ESP_LOGW(TAG, "Expected physical motion: %s", physical_expectation);

    Set_motor(vx, vy, w);
    delay_ms(MOTOR_TEST_STATUS_WAIT_MS);
    print_status("command active");

    delay_ms(MOTOR_TEST_CHASSIS_RUN_MS);

    motor_stop();
    delay_ms(MOTOR_TEST_SETTLE_MS);
    print_status("after motor_stop");
}

static void run_chassis_motion_tests(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== Chassis direction tests ==========");
    ESP_LOGW(TAG,
             "These tests verify the CURRENT kinematic signs against the physical chassis.");
    ESP_LOGW(TAG,
             "The header defines +vx=forward, +vy=left, +w=CCW.");

    run_chassis_command("+vx",
                        +MOTOR_TEST_COMMAND, 0.0f, 0.0f,
                        "forward");

    run_chassis_command("-vx",
                        -MOTOR_TEST_COMMAND, 0.0f, 0.0f,
                        "backward");

    run_chassis_command("+vy",
                        0.0f, +MOTOR_TEST_COMMAND, 0.0f,
                        "left");

    run_chassis_command("-vy",
                        0.0f, -MOTOR_TEST_COMMAND, 0.0f,
                        "right");

    run_chassis_command("+w",
                        0.0f, 0.0f, +MOTOR_TEST_COMMAND,
                        "counter-clockwise");

    run_chassis_command("-w",
                        0.0f, 0.0f, -MOTOR_TEST_COMMAND,
                        "clockwise");
}

/* --------------------------------------------------------------------------
 * Stop / emergency-stop behavior
 * -------------------------------------------------------------------------- */

static void test_stop_functions(void)
{
    motor_status_t s = {0};

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== Stop / enable tests ==========");

    motor_stop();
    get_status_after_task(&s);

    check_bool("motor_stop zeros chassis target",
               nearf_abs(s.target_vx_mm_s, 0.0f, 0.1f) &&
               nearf_abs(s.target_vy_mm_s, 0.0f, 0.1f) &&
               nearf_abs(s.target_w_rad_s, 0.0f, 0.001f));

    /*
     * Emergency stop writes PWM zero immediately in the implementation.
     * Status is task-published, so wait one task period before checking.
     */
    motor_emergency_stop();
    get_status_after_task(&s);

    check_bool("motor_emergency_stop -> all output PWM zero",
               nearf_abs(s.A.output_pwm, 0.0f, 0.1f) &&
               nearf_abs(s.B.output_pwm, 0.0f, 0.1f) &&
               nearf_abs(s.D.output_pwm, 0.0f, 0.1f));

    /*
     * NOTE:
     * Current motor_control.h makes emergency stop NON-LATCHING:
     * Set_motor() clears g_emergency_stop.
     * This test does not intentionally command motion here.
     */
    Set_motor(0.0f, 0.0f, 0.0f);
    get_status_after_task(&s);

    check_bool("Set_motor after emergency stop remains callable",
               nearf_abs(s.target_vx_mm_s, 0.0f, 0.1f) &&
               nearf_abs(s.target_vy_mm_s, 0.0f, 0.1f) &&
               nearf_abs(s.target_w_rad_s, 0.0f, 0.001f));
}

/* --------------------------------------------------------------------------
 * app_main
 * -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "############################################################");
    ESP_LOGI(TAG, "# motor_control.h functional / polarity test");
    ESP_LOGI(TAG, "############################################################");

    print_configuration();

    /*
     * Static sign check against the matrix documented in motor_control.h:
     *
     * documented:
     *   A = [ 0,      +1,   +1]
     *   B = [-0.866,  -0.5, +1]
     *   D = [+0.866,  -0.5, +1]
     *
     * current macros in the uploaded file are the global negative of this.
     */
    const bool matrix_matches_documented_signs =
        nearf_abs(MOTOR_A_KX,  0.0f,       0.0001f) &&
        nearf_abs(MOTOR_A_KY, +1.0f,       0.0001f) &&
        nearf_abs(MOTOR_A_KW, +1.0f,       0.0001f) &&
        nearf_abs(MOTOR_B_KX, -0.8660254f, 0.0001f) &&
        nearf_abs(MOTOR_B_KY, -0.5f,       0.0001f) &&
        nearf_abs(MOTOR_B_KW, +1.0f,       0.0001f) &&
        nearf_abs(MOTOR_D_KX, +0.8660254f, 0.0001f) &&
        nearf_abs(MOTOR_D_KY, -0.5f,       0.0001f) &&
        nearf_abs(MOTOR_D_KW, +1.0f,       0.0001f);

    check_bool("IK macro signs match the matrix documented in the header",
               matrix_matches_documented_signs);

    if (!matrix_matches_documented_signs) {
        ESP_LOGW(TAG,
                 "This FAIL is expected with the uploaded header: "
                 "the actual K* macros are the global negative of the documented matrix.");
    }

    esp_err_t err = motor_control_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "motor_control_init failed: %s (0x%x)",
                 esp_err_to_name(err),
                 (unsigned)err);
        return;
    }

    delay_ms(100);
    print_status("after init");

    test_api_state_and_scaling();
    test_stop_functions();

#if MOTOR_TEST_ENABLE_MOTION
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "MOTION TESTS ENABLED.");
    ESP_LOGW(TAG, "Keep the chassis lifted and observe wheel/chassis directions.");

    /*
     * Ensure the controller is enabled before raw tests.
     */
    motor_control_enable(true);
    motor_stop();
    delay_ms(MOTOR_TEST_SETTLE_MS);

    run_one_raw_polarity_test(TEST_WHEEL_A);
    run_one_raw_polarity_test(TEST_WHEEL_B);
    run_one_raw_polarity_test(TEST_WHEEL_D);

    /*
     * Exit raw mode before closed-loop chassis tests.
     */
    Set_motor(0.0f, 0.0f, 0.0f);
    delay_ms(MOTOR_TEST_SETTLE_MS);

    if (motor_control_is_ready()) {
        run_chassis_motion_tests();
    } else {
        ++s_fail;
        ESP_LOGE(TAG,
                 "[FAIL] closed loop is not ready; chassis tests skipped. "
                 "Check encoder pins/CPR and geometry.");
    }
#else
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG,
             "Motion tests are disabled. Set MOTOR_TEST_ENABLE_MOTION=1 "
             "after lifting the chassis.");
#endif

    motor_emergency_stop();
    delay_ms(MOTOR_TEST_STATUS_WAIT_MS);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, "TEST SUMMARY: PASS=%d FAIL=%d", s_pass, s_fail);
    ESP_LOGI(TAG, "============================================================");

    ESP_LOGI(TAG,
             "For polarity calibration: first make +raw PWM match the chosen "
             "positive wheel direction using MOTOR_*_POLARITY; then make the "
             "logical encoder count increase using MOTOR_*_ENCODER_POLARITY.");

    ESP_LOGI(TAG,
             "Finally verify +vx=forward, +vy=left, +w=CCW. "
             "If all three chassis axes are globally reversed, fix the IK matrix "
             "or its documented convention rather than hiding the issue in PID gains.");
}