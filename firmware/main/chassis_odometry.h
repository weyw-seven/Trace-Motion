#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"
#include "mpu6050.h"
#include "chassis_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * chassis_odometry.h
 *
 * Sensor-fused planar odometry for the 3-wheel omni chassis.
 *
 * Estimation policy:
 *
 *   Translation:
 *      wheel encoder increments -> forward kinematics -> dx/dy
 *
 *   Heading:
 *      MPU6050 continuous relative yaw
 *
 *   Angular velocity:
 *      MPU6050 gyro Z
 *
 * Coordinate frames:
 *
 *   BODY:
 *      +X forward
 *      +Y left
 *      +Yaw counter-clockwise
 *
 *   WORLD:
 *      At reset, BODY +X defines WORLD +X unless a different
 *      yaw is explicitly assigned by chassis_odometry_reset().
 *
 * Units:
 *      distance       : mm
 *      linear speed   : mm/s
 *      yaw angle      : degree
 *      angular speed  : rad/s
 * ============================================================ */

#ifndef CHASSIS_ODOMETRY_PERIOD_MS
#define CHASSIS_ODOMETRY_PERIOD_MS       10U
#endif

#ifndef CHASSIS_ODOMETRY_DISTANCE_SCALE
#define CHASSIS_ODOMETRY_DISTANCE_SCALE  1.18f
#endif

#ifndef CHASSIS_ODOMETRY_TASK_STACK_SIZE
#define CHASSIS_ODOMETRY_TASK_STACK_SIZE 4096
#endif

#ifndef CHASSIS_ODOMETRY_TASK_PRIORITY
#define CHASSIS_ODOMETRY_TASK_PRIORITY   6
#endif

#ifndef CHASSIS_ODOMETRY_PI
#define CHASSIS_ODOMETRY_PI              3.14159265358979323846f
#endif

/*
 * MPU yaw / gyro-Z sign relative to chassis convention:
 *
 *      chassis +Yaw = counter-clockwise
 *
 * Keep +1 when MPU Z is up and its positive yaw matches chassis CCW.
 * Change to -1 when the mounted sensor reports the opposite sign.
 */
#ifndef CHASSIS_ODOMETRY_IMU_YAW_SIGN
#define CHASSIS_ODOMETRY_IMU_YAW_SIGN    1.0f
#endif

#define CHASSIS_ODOMETRY_DEG_TO_RAD \
    (CHASSIS_ODOMETRY_PI / 180.0f)

#define CHASSIS_ODOMETRY_RAD_TO_DEG \
    (180.0f / CHASSIS_ODOMETRY_PI)

typedef struct
{
    bool initialized;
    bool running;

    /*
     * World-frame pose.
     */
    float x_mm;
    float y_mm;
    float yaw_deg;

    /*
     * Body-frame velocity from wheel encoders.
     */
    float body_vx_mm_s;
    float body_vy_mm_s;

    /*
     * World-frame linear velocity.
     */
    float world_vx_mm_s;
    float world_vy_mm_s;

    /*
     * Angular velocity.
     *
     * gyro_w_rad_s:
     *      primary angular-rate estimate from MPU6050 gyro Z.
     *
     * encoder_w_rad_s:
     *      wheel-encoder-only angular-rate estimate, useful for
     *      diagnostics / slip detection.
     */
    float gyro_w_rad_s;
    float encoder_w_rad_s;

    /*
     * Path length accumulated since last reset.
     * This is the sum of body-frame translation magnitudes.
     */
    float travel_distance_mm;

    /*
     * Latest wheel incremental travel used by odometry.
     */
    float delta_a_mm;
    float delta_b_mm;
    float delta_d_mm;

    uint32_t update_count;
    int64_t timestamp_us;

} chassis_odometry_state_t;


/**
 * @brief Start background odometry task.
 *
 * Requirements:
 *      motor_control_init() already completed
 *      motor closed loop / encoders ready
 *      mpu6050_init() already completed
 */
esp_err_t chassis_odometry_init(void);

/**
 * @brief Stop background odometry task.
 */
esp_err_t chassis_odometry_deinit(void);

/**
 * @brief Reset world pose without resetting MPU6050 itself.
 *
 * Current encoder counts become the new incremental origin.
 * Current MPU yaw is mapped to yaw_deg through an internal offset.
 */
esp_err_t chassis_odometry_reset(
    float x_mm,
    float y_mm,
    float yaw_deg);

/**
 * @brief Get one coherent odometry snapshot.
 */
void chassis_odometry_get_state(
    chassis_odometry_state_t *state);

bool chassis_odometry_is_ready(void);

float chassis_odometry_get_x_mm(void);
float chassis_odometry_get_y_mm(void);
float chassis_odometry_get_yaw_deg(void);

#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 *
 * Define CHASSIS_ODOMETRY_IMPLEMENTATION in exactly one .c file:
 *
 *      #define CHASSIS_ODOMETRY_IMPLEMENTATION
 *      #include "chassis_odometry.h"
 * ============================================================ */

#ifdef CHASSIS_ODOMETRY_IMPLEMENTATION

#include "freertos/semphr.h"


#ifndef CHASSIS_ODOMETRY_DEINIT_TIMEOUT_MS
#define CHASSIS_ODOMETRY_DEINIT_TIMEOUT_MS  1000U
#endif

/*
 * Generation counter:
 * increment whenever reset/init invalidates an in-flight update.
 * This is stronger than comparing yaw_offset floats, because a reset may
 * legitimately produce the same numerical yaw offset as before.
 */
static uint32_t
    g_chassis_odometry_generation = 0U;

static SemaphoreHandle_t
    g_chassis_odometry_task_exit_sem = NULL;

static chassis_odometry_state_t
    g_chassis_odometry_state = {0};

static TaskHandle_t
    g_chassis_odometry_task_handle = NULL;

static bool
    g_chassis_odometry_running = false;

static int32_t
    g_chassis_odometry_prev_count_a = 0;

static int32_t
    g_chassis_odometry_prev_count_b = 0;

static int32_t
    g_chassis_odometry_prev_count_d = 0;

/*
 * pose_yaw = mpu_continuous_yaw + yaw_offset
 *
 * This keeps odometry reset independent of mpu6050_reset_yaw().
 */
static float
    g_chassis_odometry_yaw_offset_deg = 0.0f;

static portMUX_TYPE
    g_chassis_odometry_lock =
        portMUX_INITIALIZER_UNLOCKED;


static int32_t chassis_odometry_count_delta(
    int32_t current,
    int32_t previous)
{
    /*
     * Encoder counters may wrap at INT32 boundaries during long runs.
     *
     * Unsigned subtraction is defined modulo 2^32. Casting the modular
     * difference back to int32_t gives the intended signed incremental
     * count as long as less than 2^31 counts occur between updates,
     * which is overwhelmingly true at a 10 ms odometry period.
     */
    return (int32_t)(
        (uint32_t)current -
        (uint32_t)previous
    );
}


static float chassis_odometry_wrap_deg_180(
    float angle_deg)
{
    /*
     * Keep the public yaw continuous.  Only reduce the argument supplied
     * to sinf/cosf so long-running yaw accumulation does not degrade trig
     * precision.
     */
    angle_deg =
        fmodf(
            angle_deg + 180.0f,
            360.0f
        );

    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg - 180.0f;
}


static float chassis_odometry_count_to_mm(
    int32_t delta_count,
    float cpr)
{
    if ((cpr <= 0.0f) ||
        (MOTOR_WHEEL_DIAMETER_MM <= 0.0f))
    {
        return 0.0f;
    }

    const float circumference_mm =
        CHASSIS_ODOMETRY_PI *
        MOTOR_WHEEL_DIAMETER_MM;

    return
        ((float)delta_count / cpr) *
        circumference_mm *
        CHASSIS_ODOMETRY_DISTANCE_SCALE;
}


static esp_err_t chassis_odometry_update_internal(void)
{
    motor_status_t motor_status = {0};
    mpu6050_snapshot_t imu = {0};

    motor_get_status(&motor_status);
    mpu6050_get_snapshot(&imu);

    if (!motor_status.initialized ||
        !motor_status.closed_loop_ready)
    {
        return ESP_ERR_INVALID_STATE;
    }

    mpu6050_status_t imu_status = {0};
    mpu6050_get_status(&imu_status);

    if (!imu_status.initialized ||
        !imu_status.sampling)
    {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t prev_a;
    int32_t prev_b;
    int32_t prev_d;

    float yaw_offset_deg;
    uint32_t generation;
    chassis_odometry_state_t previous;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);

    prev_a = g_chassis_odometry_prev_count_a;
    prev_b = g_chassis_odometry_prev_count_b;
    prev_d = g_chassis_odometry_prev_count_d;

    yaw_offset_deg =
        g_chassis_odometry_yaw_offset_deg;

    generation =
        g_chassis_odometry_generation;

    previous =
        g_chassis_odometry_state;

    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    const int32_t dc_a =
        chassis_odometry_count_delta(
            motor_status.A.encoder_count,
            prev_a
        );

    const int32_t dc_b =
        chassis_odometry_count_delta(
            motor_status.B.encoder_count,
            prev_b
        );

    const int32_t dc_d =
        chassis_odometry_count_delta(
            motor_status.D.encoder_count,
            prev_d
        );

    const float da_mm =
        chassis_odometry_count_to_mm(
            dc_a,
            MOTOR_A_ENCODER_CPR);

    const float db_mm =
        chassis_odometry_count_to_mm(
            dc_b,
            MOTOR_B_ENCODER_CPR);

    const float dd_mm =
        chassis_odometry_count_to_mm(
            dc_d,
            MOTOR_D_ENCODER_CPR);

    chassis_body_delta_t body_delta = {0};

    esp_err_t ret =
        chassis_kinematics_forward_delta(
            da_mm,
            db_mm,
            dd_mm,
            &body_delta);

    if (ret != ESP_OK)
    {
        return ret;
    }

    chassis_wheel_velocity_t wheel_velocity = {
        .a_mm_s = motor_status.A.actual_speed_mm_s,
        .b_mm_s = motor_status.B.actual_speed_mm_s,
        .d_mm_s = motor_status.D.actual_speed_mm_s
    };

    chassis_velocity_t body_velocity = {0};

    ret =
        chassis_kinematics_forward(
            &wheel_velocity,
            &body_velocity);

    if (ret != ESP_OK)
    {
        return ret;
    }

    /*
     * Heading comes from MPU6050 continuous yaw.
     */
    const float new_yaw_deg =
        CHASSIS_ODOMETRY_IMU_YAW_SIGN *
        imu.attitude.yaw +
        yaw_offset_deg;

    const float old_yaw_deg =
        previous.yaw_deg;

    const float delta_yaw_deg =
        new_yaw_deg -
        old_yaw_deg;

    /*
     * Midpoint-heading integration:
     *
     * Rotate the body-frame translation with the heading halfway
     * through the sample interval. This is more accurate than using
     * only the old or only the new heading while turning.
     */
    const float mid_yaw_deg =
        chassis_odometry_wrap_deg_180(
            old_yaw_deg +
            0.5f * delta_yaw_deg
        );

    const float mid_yaw_rad =
        mid_yaw_deg *
        CHASSIS_ODOMETRY_DEG_TO_RAD;

    const float c_mid = cosf(mid_yaw_rad);
    const float s_mid = sinf(mid_yaw_rad);

    const float world_dx_mm =
        c_mid * body_delta.dx_mm
        -
        s_mid * body_delta.dy_mm;

    const float world_dy_mm =
        s_mid * body_delta.dx_mm
        +
        c_mid * body_delta.dy_mm;

    const float yaw_for_trig_deg =
        chassis_odometry_wrap_deg_180(
            new_yaw_deg
        );

    const float yaw_rad =
        yaw_for_trig_deg *
        CHASSIS_ODOMETRY_DEG_TO_RAD;

    const float c_yaw = cosf(yaw_rad);
    const float s_yaw = sinf(yaw_rad);

    const float world_vx_mm_s =
        c_yaw * body_velocity.vx_mm_s
        -
        s_yaw * body_velocity.vy_mm_s;

    const float world_vy_mm_s =
        s_yaw * body_velocity.vx_mm_s
        +
        c_yaw * body_velocity.vy_mm_s;

    const float gyro_w_rad_s =
        CHASSIS_ODOMETRY_IMU_YAW_SIGN *
        imu.data.gz *
        CHASSIS_ODOMETRY_DEG_TO_RAD;

    const float translation_step_mm =
        sqrtf(
            body_delta.dx_mm * body_delta.dx_mm +
            body_delta.dy_mm * body_delta.dy_mm);

    chassis_odometry_state_t next = previous;

    next.initialized = true;
    next.running = true;

    next.x_mm += world_dx_mm;
    next.y_mm += world_dy_mm;
    next.yaw_deg = new_yaw_deg;

    next.body_vx_mm_s =
        body_velocity.vx_mm_s;

    next.body_vy_mm_s =
        body_velocity.vy_mm_s;

    next.world_vx_mm_s =
        world_vx_mm_s;

    next.world_vy_mm_s =
        world_vy_mm_s;

    next.gyro_w_rad_s =
        gyro_w_rad_s;

    next.encoder_w_rad_s =
        body_velocity.w_rad_s;

    next.travel_distance_mm +=
        translation_step_mm;

    next.delta_a_mm = da_mm;
    next.delta_b_mm = db_mm;
    next.delta_d_mm = dd_mm;

    next.update_count++;

    /*
     * Keep timestamps monotonic across reset() and cached IMU snapshots.
     * Immediately after reset, the latest IMU sample can be a few
     * milliseconds older than the reset timestamp.
     */
    int64_t next_timestamp_us =
        (imu.timestamp_us > 0)
        ? imu.timestamp_us
        : esp_timer_get_time();

    if (next_timestamp_us <
        previous.timestamp_us)
    {
        next_timestamp_us =
            esp_timer_get_time();

        if (next_timestamp_us <
            previous.timestamp_us)
        {
            next_timestamp_us =
                previous.timestamp_us;
        }
    }

    next.timestamp_us =
        next_timestamp_us;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);

    /*
     * Preserve any reset/init that occurred while this update was
     * outside the lock.  Generation comparison is race-safe even when a
     * reset produces exactly the same yaw_offset value as before.
     *
     * If generation changed, discard this entire sample INCLUDING its
     * encoder origins. The reset/init path has already captured fresh
     * encoder origins.
     */
    if (g_chassis_odometry_generation ==
        generation)
    {
        g_chassis_odometry_prev_count_a =
            motor_status.A.encoder_count;

        g_chassis_odometry_prev_count_b =
            motor_status.B.encoder_count;

        g_chassis_odometry_prev_count_d =
            motor_status.D.encoder_count;

        g_chassis_odometry_state = next;
    }

    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    return ESP_OK;
}


static void chassis_odometry_task(void *arg)
{
    (void)arg;

    TickType_t last_wake =
        xTaskGetTickCount();

    TickType_t period =
        pdMS_TO_TICKS(
            CHASSIS_ODOMETRY_PERIOD_MS);

    if (period < 1)
    {
        period = 1;
    }

    for (;;)
    {
        bool running;

        taskENTER_CRITICAL(
            &g_chassis_odometry_lock);

        running =
            g_chassis_odometry_running;

        taskEXIT_CRITICAL(
            &g_chassis_odometry_lock);

        if (!running)
        {
            break;
        }

        (void)chassis_odometry_update_internal();

        vTaskDelayUntil(
            &last_wake,
            period);
    }

    taskENTER_CRITICAL(
        &g_chassis_odometry_lock);

    g_chassis_odometry_state.running =
        false;

    g_chassis_odometry_task_handle =
        NULL;

    taskEXIT_CRITICAL(
        &g_chassis_odometry_lock);

    if (g_chassis_odometry_task_exit_sem != NULL)
    {
        xSemaphoreGive(
            g_chassis_odometry_task_exit_sem);
    }

    vTaskDelete(NULL);
}


esp_err_t chassis_odometry_reset(
    float x_mm,
    float y_mm,
    float yaw_deg)
{
    if (!isfinite(x_mm) ||
        !isfinite(y_mm) ||
        !isfinite(yaw_deg))
    {
        return ESP_ERR_INVALID_ARG;
    }

    motor_status_t motor_status = {0};
    mpu6050_status_t imu_status = {0};
    mpu6050_snapshot_t imu = {0};

    motor_get_status(&motor_status);
    mpu6050_get_status(&imu_status);
    mpu6050_get_snapshot(&imu);

    if (!motor_status.initialized ||
        !motor_status.closed_loop_ready ||
        !imu_status.initialized ||
        !imu_status.sampling)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const float imu_yaw_deg =
        CHASSIS_ODOMETRY_IMU_YAW_SIGN *
        imu.attitude.yaw;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);

    if (!g_chassis_odometry_state.initialized)
    {
        taskEXIT_CRITICAL(
            &g_chassis_odometry_lock);

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Invalidate any update that already captured old origins/state.
     */
    g_chassis_odometry_generation++;

    g_chassis_odometry_prev_count_a =
        motor_status.A.encoder_count;

    g_chassis_odometry_prev_count_b =
        motor_status.B.encoder_count;

    g_chassis_odometry_prev_count_d =
        motor_status.D.encoder_count;

    g_chassis_odometry_yaw_offset_deg =
        yaw_deg -
        imu_yaw_deg;

    const bool initialized =
        g_chassis_odometry_state.initialized;

    const bool running =
        g_chassis_odometry_running;

    g_chassis_odometry_state =
        (chassis_odometry_state_t){0};

    g_chassis_odometry_state.initialized =
        initialized;

    g_chassis_odometry_state.running =
        running;

    g_chassis_odometry_state.x_mm =
        x_mm;

    g_chassis_odometry_state.y_mm =
        y_mm;

    g_chassis_odometry_state.yaw_deg =
        yaw_deg;

    g_chassis_odometry_state.timestamp_us =
        esp_timer_get_time();

    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    return ESP_OK;
}


esp_err_t chassis_odometry_init(void)
{
    bool already_initialized;

    taskENTER_CRITICAL(
        &g_chassis_odometry_lock);

    already_initialized =
        g_chassis_odometry_state.initialized;

    taskEXIT_CRITICAL(
        &g_chassis_odometry_lock);

    if (already_initialized)
    {
        return ESP_OK;
    }

    if (g_chassis_odometry_task_exit_sem == NULL)
    {
        g_chassis_odometry_task_exit_sem =
            xSemaphoreCreateBinary();

        if (g_chassis_odometry_task_exit_sem == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    while (xSemaphoreTake(
               g_chassis_odometry_task_exit_sem,
               0) == pdTRUE)
    {
        /* drain stale exit notification */
    }

    motor_status_t motor_status = {0};
    mpu6050_status_t imu_status = {0};
    mpu6050_snapshot_t imu = {0};

    motor_get_status(&motor_status);
    mpu6050_get_status(&imu_status);
    mpu6050_get_snapshot(&imu);

    if (!motor_status.initialized ||
        !motor_status.closed_loop_ready)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!imu_status.initialized ||
        !imu_status.sampling)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!chassis_kinematics_is_valid())
    {
        return ESP_ERR_INVALID_STATE;
    }

    const float imu_yaw_deg =
        CHASSIS_ODOMETRY_IMU_YAW_SIGN *
        imu.attitude.yaw;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);

    g_chassis_odometry_generation++;

    g_chassis_odometry_prev_count_a =
        motor_status.A.encoder_count;

    g_chassis_odometry_prev_count_b =
        motor_status.B.encoder_count;

    g_chassis_odometry_prev_count_d =
        motor_status.D.encoder_count;

    /*
     * Default odometry origin:
     * current physical heading -> odometry yaw = 0 degrees.
     */
    g_chassis_odometry_yaw_offset_deg =
        -imu_yaw_deg;

    g_chassis_odometry_state =
        (chassis_odometry_state_t){0};

    g_chassis_odometry_state.initialized =
        true;

    g_chassis_odometry_state.running =
        true;

    g_chassis_odometry_state.timestamp_us =
        esp_timer_get_time();

    g_chassis_odometry_running =
        true;

    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    BaseType_t task_result =
        xTaskCreate(
            chassis_odometry_task,
            "chassis_odom",
            CHASSIS_ODOMETRY_TASK_STACK_SIZE,
            NULL,
            CHASSIS_ODOMETRY_TASK_PRIORITY,
            &g_chassis_odometry_task_handle);

    if (task_result != pdPASS)
    {
        taskENTER_CRITICAL(&g_chassis_odometry_lock);

        g_chassis_odometry_running =
            false;

        g_chassis_odometry_state.initialized =
            false;

        g_chassis_odometry_state.running =
            false;

        g_chassis_odometry_task_handle =
            NULL;

        taskEXIT_CRITICAL(&g_chassis_odometry_lock);

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}


esp_err_t chassis_odometry_deinit(void)
{
    bool initialized;
    TaskHandle_t task;

    taskENTER_CRITICAL(
        &g_chassis_odometry_lock);

    initialized =
        g_chassis_odometry_state.initialized;

    task =
        g_chassis_odometry_task_handle;

    taskEXIT_CRITICAL(
        &g_chassis_odometry_lock);

    if (!initialized)
    {
        return ESP_OK;
    }

    if ((task != NULL) &&
        (xTaskGetCurrentTaskHandle() == task))
    {
        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(
        &g_chassis_odometry_lock);

    g_chassis_odometry_generation++;

    g_chassis_odometry_running =
        false;

    taskEXIT_CRITICAL(
        &g_chassis_odometry_lock);

    /*
     * Let the odometry task exit itself.  Do not externally vTaskDelete()
     * a task that may be inside an update or critical section.
     */
    if (task != NULL)
    {
        if (g_chassis_odometry_task_exit_sem == NULL)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (xSemaphoreTake(
                g_chassis_odometry_task_exit_sem,
                pdMS_TO_TICKS(
                    CHASSIS_ODOMETRY_DEINIT_TIMEOUT_MS)
            ) != pdTRUE)
        {
            return ESP_ERR_TIMEOUT;
        }
    }

    taskENTER_CRITICAL(
        &g_chassis_odometry_lock);

    g_chassis_odometry_state =
        (chassis_odometry_state_t){0};

    g_chassis_odometry_prev_count_a = 0;
    g_chassis_odometry_prev_count_b = 0;
    g_chassis_odometry_prev_count_d = 0;

    g_chassis_odometry_yaw_offset_deg =
        0.0f;

    g_chassis_odometry_running =
        false;

    taskEXIT_CRITICAL(
        &g_chassis_odometry_lock);

    if (g_chassis_odometry_task_exit_sem != NULL)
    {
        vSemaphoreDelete(
            g_chassis_odometry_task_exit_sem);

        g_chassis_odometry_task_exit_sem =
            NULL;
    }

    return ESP_OK;
}


void chassis_odometry_get_state(
    chassis_odometry_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&g_chassis_odometry_lock);

    *state =
        g_chassis_odometry_state;

    taskEXIT_CRITICAL(&g_chassis_odometry_lock);
}


bool chassis_odometry_is_ready(void)
{
    bool ready;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);

    ready =
        g_chassis_odometry_state.initialized &&
        g_chassis_odometry_state.running;

    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    return ready;
}


float chassis_odometry_get_x_mm(void)
{
    float value;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);
    value = g_chassis_odometry_state.x_mm;
    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    return value;
}


float chassis_odometry_get_y_mm(void)
{
    float value;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);
    value = g_chassis_odometry_state.y_mm;
    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    return value;
}


float chassis_odometry_get_yaw_deg(void)
{
    float value;

    taskENTER_CRITICAL(&g_chassis_odometry_lock);
    value = g_chassis_odometry_state.yaw_deg;
    taskEXIT_CRITICAL(&g_chassis_odometry_lock);

    return value;
}

#endif /* CHASSIS_ODOMETRY_IMPLEMENTATION */
#endif /* CHASSIS_ODOMETRY_H */