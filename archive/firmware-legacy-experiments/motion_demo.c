
/*
 * chassis_motion_full_suite_test.c
 *
 * Full active traversal test for chassis_motion.h.
 *
 * Features:
 *   - multiple positive/negative rotations
 *   - four cardinal translations
 *   - four 45-degree diagonal translations
 *   - extra 30/60-degree oblique translations
 *   - negative-distance API behavior
 *   - stop / cancel / emergency-stop API behavior
 *   - failures NEVER abort the complete suite
 *
 * Coordinate convention:
 *   direction 0    = forward (+X)
 *   direction +90  = left    (+Y)
 *   direction -90  = right   (-Y)
 *   direction 180  = backward(-X)
 *   +rotation       = CCW
 *   -rotation       = CW
 *
 * IMPORTANT:
 *   TEST_ENABLE=1 actively drives the chassis.
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

#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"

#define CHASSIS_KINEMATICS_IMPLEMENTATION
#include "chassis_kinematics.h"

#define CHASSIS_ODOMETRY_IMPLEMENTATION
#include "chassis_odometry.h"

#define CHASSIS_MOTION_IMPLEMENTATION
#include "chassis_motion.h"


/* ============================================================
 * Safety / suite parameters
 * ============================================================ */

#ifndef TEST_ENABLE
#define TEST_ENABLE                     1
#endif

#ifndef TEST_MOVE_DISTANCE_MM
#define TEST_MOVE_DISTANCE_MM           300.0f
#endif

#ifndef TEST_MOVE_SPEED_MM_S
#define TEST_MOVE_SPEED_MM_S            550.0f
#endif

#ifndef TEST_ROTATE_SPEED_DEG_S
#define TEST_ROTATE_SPEED_DEG_S         120.0f
#endif

#ifndef TEST_PRINT_PERIOD_MS
#define TEST_PRINT_PERIOD_MS            100U
#endif

#ifndef TEST_PAUSE_MS
#define TEST_PAUSE_MS                   1000U
#endif

#ifndef TEST_PRE_START_MS
#define TEST_PRE_START_MS               2500U
#endif

#ifndef TEST_EXTERNAL_TIMEOUT_MS
#define TEST_EXTERNAL_TIMEOUT_MS        15000U
#endif

#ifndef TEST_INTERRUPT_AFTER_MS
#define TEST_INTERRUPT_AFTER_MS         450U
#endif

#ifndef TEST_INTERRUPT_DISTANCE_MM
#define TEST_INTERRUPT_DISTANCE_MM      1000.0f
#endif


typedef enum
{
    TC_MOVE = 0,
    TC_ROTATE,
    TC_STOP,
    TC_CANCEL,
    TC_ESTOP
} test_kind_t;


typedef struct
{
    const char *name;
    test_kind_t kind;
    float p1;
    float p2;
    float p3;
} test_case_t;


typedef struct
{
    uint32_t total;
    uint32_t pass;
    uint32_t fail;
    uint32_t rejected;
    uint32_t external_timeout;
} test_stats_t;


typedef struct
{
    chassis_odometry_state_t start_odom;
    chassis_odometry_state_t end_odom;
    chassis_motion_status_t end_motion;
    esp_err_t api_ret;
    esp_err_t wait_ret;
    bool timeout;
    bool interrupted;
} test_result_t;


static const char *TAG = "MOTION_SUITE";


/*
 * Translation tests are mostly paired with opposite directions so the
 * robot roughly returns to the same test area.
 */
static const test_case_t g_cases[] =
{
    /* ---------- rotation ---------- */
    { "ROT_CCW_45",  TC_ROTATE, +45.0f,  TEST_ROTATE_SPEED_DEG_S, 0.0f },
    { "ROT_CW_45",   TC_ROTATE, -45.0f,  TEST_ROTATE_SPEED_DEG_S, 0.0f },

    { "ROT_CCW_90",  TC_ROTATE, +90.0f,  TEST_ROTATE_SPEED_DEG_S, 0.0f },
    { "ROT_CW_90",   TC_ROTATE, -90.0f,  TEST_ROTATE_SPEED_DEG_S, 0.0f },

    { "ROT_CCW_135", TC_ROTATE, +135.0f, TEST_ROTATE_SPEED_DEG_S, 0.0f },
    { "ROT_CW_135",  TC_ROTATE, -135.0f, TEST_ROTATE_SPEED_DEG_S, 0.0f },

    { "ROT_CCW_180", TC_ROTATE, +180.0f, TEST_ROTATE_SPEED_DEG_S, 0.0f },
    { "ROT_CW_180",  TC_ROTATE, -180.0f, TEST_ROTATE_SPEED_DEG_S, 0.0f },

    /* continuous yaw / no wrap */
    { "ROT_CCW_270", TC_ROTATE, +270.0f, TEST_ROTATE_SPEED_DEG_S, 0.0f },
    { "ROT_CW_270",  TC_ROTATE, -270.0f, TEST_ROTATE_SPEED_DEG_S, 0.0f },

    /* ---------- cardinal ---------- */
    { "MOVE_FORWARD_0",     TC_MOVE,   0.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_BACKWARD_180",  TC_MOVE, 180.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    { "MOVE_LEFT_90",       TC_MOVE, +90.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_RIGHT_-90",     TC_MOVE, -90.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    /* ---------- four diagonals ---------- */
    { "MOVE_DIAG_45",       TC_MOVE,  +45.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_DIAG_-135",     TC_MOVE, -135.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    { "MOVE_DIAG_-45",      TC_MOVE,  -45.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_DIAG_135",      TC_MOVE, +135.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    /* ---------- extra arbitrary angles ---------- */
    { "MOVE_OBLIQUE_30",    TC_MOVE,  +30.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_OBLIQUE_-150",  TC_MOVE, -150.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    { "MOVE_OBLIQUE_60",    TC_MOVE,  +60.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_OBLIQUE_-120",  TC_MOVE, -120.0f, TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    /* ---------- negative distance semantics ---------- */
    { "MOVE_NEGATIVE_DISTANCE", TC_MOVE, 0.0f, -TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "MOVE_NEGATIVE_RETURN",   TC_MOVE, 0.0f, +TEST_MOVE_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },

    /* ---------- control APIs ---------- */
    { "API_STOP",   TC_STOP,   0.0f, TEST_INTERRUPT_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "API_CANCEL", TC_CANCEL, 0.0f, TEST_INTERRUPT_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
    { "API_ESTOP",  TC_ESTOP,  0.0f, TEST_INTERRUPT_DISTANCE_MM, TEST_MOVE_SPEED_MM_S },
};

#define TEST_CASE_COUNT \
    ((uint32_t)(sizeof(g_cases) / sizeof(g_cases[0])))


static const char *mode_name(chassis_motion_mode_t mode)
{
    switch (mode)
    {
        case CHASSIS_MOTION_MODE_IDLE:          return "IDLE";
        case CHASSIS_MOTION_MODE_MOVE_DISTANCE: return "MOVE";
        case CHASSIS_MOTION_MODE_ROTATE:        return "ROTATE";
        case CHASSIS_MOTION_MODE_DONE:          return "DONE";
        case CHASSIS_MOTION_MODE_CANCELLED:     return "CANCELLED";
        case CHASSIS_MOTION_MODE_ERROR:         return "ERROR";
        default:                                return "?";
    }
}


static const char *kind_name(test_kind_t kind)
{
    switch (kind)
    {
        case TC_MOVE:   return "MOVE";
        case TC_ROTATE: return "ROTATE";
        case TC_STOP:   return "STOP";
        case TC_CANCEL: return "CANCEL";
        case TC_ESTOP:  return "ESTOP";
        default:        return "?";
    }
}


static void print_live(const char *name)
{
    chassis_motion_status_t m = {0};
    chassis_odometry_state_t o = {0};
    motor_status_t motor = {0};

    chassis_motion_get_status(&m);
    chassis_odometry_get_state(&o);
    motor_get_status(&motor);

    ESP_LOGI(
        TAG,
        "LIVE %-24s | mode=%s busy=%d | "
        "x=%+7.1f y=%+7.1f yaw=%+7.2f | "
        "remD=%+7.1f cross=%+6.1f remA=%+7.2f | "
        "cmd=[%+6.1f,%+6.1f,%+5.3f]",
        name,
        mode_name(m.mode),
        (int)m.busy,
        (double)o.x_mm,
        (double)o.y_mm,
        (double)o.yaw_deg,
        (double)m.remaining_distance_mm,
        (double)m.cross_track_error_mm,
        (double)m.remaining_angle_deg,
        (double)m.command_vx_mm_s,
        (double)m.command_vy_mm_s,
        (double)m.command_w_rad_s);

    ESP_LOGI(
        TAG,
        "WHEEL %-23s | "
        "A[%+6.1f/%+6.1f] "
        "B[%+6.1f/%+6.1f] "
        "D[%+6.1f/%+6.1f]",
        name,
        (double)motor.A.target_speed_mm_s,
        (double)motor.A.actual_speed_mm_s,
        (double)motor.B.target_speed_mm_s,
        (double)motor.B.actual_speed_mm_s,
        (double)motor.D.target_speed_mm_s,
        (double)motor.D.actual_speed_mm_s);
}


static esp_err_t wait_case(
    const test_case_t *tc,
    test_result_t *r)
{
    const TickType_t start =
        xTaskGetTickCount();

    const TickType_t external_timeout =
        pdMS_TO_TICKS(TEST_EXTERNAL_TIMEOUT_MS);

    const TickType_t interrupt_after =
        pdMS_TO_TICKS(TEST_INTERRUPT_AFTER_MS);

    bool interrupt_sent = false;

    while (chassis_motion_is_busy())
    {
        const TickType_t elapsed =
            xTaskGetTickCount() - start;

        if (!interrupt_sent &&
            ((tc->kind == TC_STOP) ||
             (tc->kind == TC_CANCEL) ||
             (tc->kind == TC_ESTOP)) &&
            (elapsed >= interrupt_after))
        {
            interrupt_sent = true;
            r->interrupted = true;

            if (tc->kind == TC_STOP)
            {
                ESP_LOGW(TAG, "%s -> chassis_motion_stop()", tc->name);
                chassis_motion_stop();
            }
            else if (tc->kind == TC_CANCEL)
            {
                ESP_LOGW(TAG, "%s -> chassis_motion_cancel()", tc->name);
                chassis_motion_cancel();
            }
            else
            {
                ESP_LOGW(TAG, "%s -> chassis_motion_emergency_stop()", tc->name);
                chassis_motion_emergency_stop();
            }
        }

        if (elapsed >= external_timeout)
        {
            ESP_LOGE(
                TAG,
                "%s external timeout -> emergency stop; suite continues",
                tc->name);

            r->timeout = true;
            chassis_motion_emergency_stop();
            return ESP_ERR_TIMEOUT;
        }

        print_live(tc->name);

        vTaskDelay(
            pdMS_TO_TICKS(TEST_PRINT_PERIOD_MS));
    }

    return chassis_motion_wait(0);
}


static bool case_passed(
    const test_case_t *tc,
    const test_result_t *r)
{
    if ((tc->kind == TC_STOP) ||
        (tc->kind == TC_CANCEL) ||
        (tc->kind == TC_ESTOP))
    {
        return
            (r->api_ret == ESP_OK) &&
            r->interrupted &&
            !r->end_motion.busy;
    }

    return
        (r->api_ret == ESP_OK) &&
        (r->wait_ret == ESP_OK) &&
        (r->end_motion.mode == CHASSIS_MOTION_MODE_DONE) &&
        r->end_motion.target_reached;
}


static void print_result(
    uint32_t index,
    const test_case_t *tc,
    const test_result_t *r,
    bool pass)
{
    const float dx =
        r->end_odom.x_mm -
        r->start_odom.x_mm;

    const float dy =
        r->end_odom.y_mm -
        r->start_odom.y_mm;

    const float dyaw =
        r->end_odom.yaw_deg -
        r->start_odom.yaw_deg;

    float along = 0.0f;
    float cross = 0.0f;

    if (tc->kind == TC_MOVE)
    {
        float direction = tc->p1;
        float distance = tc->p2;

        if (distance < 0.0f)
        {
            distance = -distance;
            direction += 180.0f;
        }

        const float world_direction_rad =
            (r->start_odom.yaw_deg + direction) *
            0.01745329251994329577f;

        const float ax = cosf(world_direction_rad);
        const float ay = sinf(world_direction_rad);

        along = dx * ax + dy * ay;
        cross = dx * (-ay) + dy * ax;

        (void)distance;
    }

    ESP_LOGI(
        TAG,
        "RESULT %02lu/%02lu %-24s | %s | "
        "api=%s wait=%s mode=%s err=%d reached=%d | "
        "dx=%+.1f dy=%+.1f dyaw=%+.2f along=%+.1f cross=%+.1f",
        (unsigned long)(index + 1U),
        (unsigned long)TEST_CASE_COUNT,
        tc->name,
        pass ? "PASS" : "FAIL",
        esp_err_to_name(r->api_ret),
        esp_err_to_name(r->wait_ret),
        mode_name(r->end_motion.mode),
        (int)r->end_motion.error,
        (int)r->end_motion.target_reached,
        (double)dx,
        (double)dy,
        (double)dyaw,
        (double)along,
        (double)cross);

    ESP_LOGI(
        TAG,
        "CSV,MOTION_SUITE,"
        "index,%lu,"
        "name,%s,"
        "kind,%s,"
        "pass,%d,"
        "api_ret,%d,"
        "wait_ret,%d,"
        "mode,%d,"
        "error,%d,"
        "reached,%d,"
        "external_timeout,%d,"
        "start_x,%.6f,"
        "start_y,%.6f,"
        "start_yaw,%.6f,"
        "end_x,%.6f,"
        "end_y,%.6f,"
        "end_yaw,%.6f,"
        "dx,%.6f,"
        "dy,%.6f,"
        "dyaw,%.6f,"
        "along,%.6f,"
        "cross,%.6f,"
        "remaining_distance,%.6f,"
        "remaining_angle,%.6f",
        (unsigned long)(index + 1U),
        tc->name,
        kind_name(tc->kind),
        pass ? 1 : 0,
        (int)r->api_ret,
        (int)r->wait_ret,
        (int)r->end_motion.mode,
        (int)r->end_motion.error,
        (int)r->end_motion.target_reached,
        r->timeout ? 1 : 0,
        (double)r->start_odom.x_mm,
        (double)r->start_odom.y_mm,
        (double)r->start_odom.yaw_deg,
        (double)r->end_odom.x_mm,
        (double)r->end_odom.y_mm,
        (double)r->end_odom.yaw_deg,
        (double)dx,
        (double)dy,
        (double)dyaw,
        (double)along,
        (double)cross,
        (double)r->end_motion.remaining_distance_mm,
        (double)r->end_motion.remaining_angle_deg);
}


static bool run_case(
    uint32_t index,
    const test_case_t *tc,
    test_stats_t *stats)
{
    test_result_t r = {0};

    stats->total++;

    /*
     * Force recovery from any previous DONE / ERROR / CANCELLED state.
     */
    chassis_motion_stop();
    motor_stop();

    vTaskDelay(pdMS_TO_TICKS(200U));

    chassis_odometry_get_state(&r.start_odom);

    ESP_LOGW(
        TAG,
        "============================================================");

    ESP_LOGW(
        TAG,
        "CASE %02lu/%02lu %s (%s)",
        (unsigned long)(index + 1U),
        (unsigned long)TEST_CASE_COUNT,
        tc->name,
        kind_name(tc->kind));

    if (tc->kind == TC_ROTATE)
    {
        r.api_ret =
            chassis_motion_rotate(
                tc->p1,
                tc->p2);
    }
    else
    {
        /*
         * MOVE and API-interrupt tests all begin with a MOVE command.
         */
        r.api_ret =
            chassis_motion_move_distance(
                tc->p1,
                tc->p2,
                tc->p3);
    }

    if (r.api_ret != ESP_OK)
    {
        stats->rejected++;
        stats->fail++;

        r.wait_ret = r.api_ret;

        chassis_odometry_get_state(&r.end_odom);
        chassis_motion_get_status(&r.end_motion);

        print_result(index, tc, &r, false);

        /*
         * Failure does not stop suite.
         */
        chassis_motion_stop();
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(TEST_PAUSE_MS));
        return false;
    }

    r.wait_ret =
        wait_case(tc, &r);

    /*
     * Snapshot terminal state before clearing it.
     */
    chassis_odometry_get_state(&r.end_odom);
    chassis_motion_get_status(&r.end_motion);

    if (r.timeout)
        stats->external_timeout++;

    const bool pass =
        case_passed(tc, &r);

    if (pass)
        stats->pass++;
    else
        stats->fail++;

    print_result(index, tc, &r, pass);

    /*
     * Unconditional recovery: next test always runs.
     */
    chassis_motion_stop();
    motor_stop();

    vTaskDelay(
        pdMS_TO_TICKS(TEST_PAUSE_MS));

    return pass;
}


static void print_summary(
    const test_stats_t *stats)
{
    chassis_odometry_state_t odom = {0};

    chassis_odometry_get_state(&odom);

    ESP_LOGI(
        TAG,
        "============================================================");

    ESP_LOGI(
        TAG,
        "FULL SUITE FINISHED | "
        "total=%lu pass=%lu fail=%lu rejected=%lu external_timeout=%lu",
        (unsigned long)stats->total,
        (unsigned long)stats->pass,
        (unsigned long)stats->fail,
        (unsigned long)stats->rejected,
        (unsigned long)stats->external_timeout);

    ESP_LOGI(
        TAG,
        "FINAL ODOM | x=%+.2f y=%+.2f yaw=%+.2f travel=%.2f",
        (double)odom.x_mm,
        (double)odom.y_mm,
        (double)odom.yaw_deg,
        (double)odom.travel_distance_mm);

    ESP_LOGI(
        TAG,
        "CSV,MOTION_SUITE_SUMMARY,"
        "total,%lu,"
        "pass,%lu,"
        "fail,%lu,"
        "rejected,%lu,"
        "external_timeout,%lu,"
        "final_x,%.6f,"
        "final_y,%.6f,"
        "final_yaw,%.6f,"
        "travel,%.6f",
        (unsigned long)stats->total,
        (unsigned long)stats->pass,
        (unsigned long)stats->fail,
        (unsigned long)stats->rejected,
        (unsigned long)stats->external_timeout,
        (double)odom.x_mm,
        (double)odom.y_mm,
        (double)odom.yaw_deg,
        (double)odom.travel_distance_mm);

    ESP_LOGI(
        TAG,
        "============================================================");
}


void app_main(void)
{
    ESP_LOGI(
        TAG,
        "CHASSIS MOTION FULL TRAVERSAL SUITE");

    ESP_LOGI(
        TAG,
        "cases=%lu distance=%.1fmm move_speed=%.1fmm/s rotate_speed=%.1fdeg/s",
        (unsigned long)TEST_CASE_COUNT,
        (double)TEST_MOVE_DISTANCE_MM,
        (double)TEST_MOVE_SPEED_MM_S,
        (double)TEST_ROTATE_SPEED_DEG_S);

    ESP_LOGW(
        TAG,
        "Keep chassis level and still during MPU calibration.");

    esp_err_t ret =
        motor_control_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "motor_control_init: %s", esp_err_to_name(ret));
        return;
    }

    ret = mpu6050_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050_init: %s", esp_err_to_name(ret));
        return;
    }

    ret = chassis_odometry_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "chassis_odometry_init: %s", esp_err_to_name(ret));
        return;
    }

    ret = chassis_odometry_reset(
        0.0f,
        0.0f,
        0.0f);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "chassis_odometry_reset: %s", esp_err_to_name(ret));
        return;
    }

    ret = chassis_motion_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "chassis_motion_init: %s", esp_err_to_name(ret));
        return;
    }


#if !TEST_ENABLE

    ESP_LOGW(
        TAG,
        "ACTIVE TEST DISABLED. Set TEST_ENABLE=1 to execute.");

    motor_stop();

    for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000U));

#else

    ESP_LOGW(
        TAG,
        "ACTIVE TEST ENABLED. Suite starts in %.1f s.",
        (double)TEST_PRE_START_MS / 1000.0);

    vTaskDelay(
        pdMS_TO_TICKS(TEST_PRE_START_MS));

    test_stats_t stats = {0};

    for (uint32_t i = 0U;
         i < TEST_CASE_COUNT;
         ++i)
    {
        /*
         * Deliberately ignore per-case failure.
         * Every case in the table is attempted.
         */
        (void)run_case(
            i,
            &g_cases[i],
            &stats);
    }

    chassis_motion_stop();
    motor_stop();

    print_summary(&stats);

    for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000U));

#endif
}