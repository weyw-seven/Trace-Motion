/*
 * mpu6050_test_main.c
 *
 * Comprehensive performance / regression test for the supplied mpu6050.h.
 *
 * Test coverage:
 *   1. Init / WHO_AM_I / calibration / sampling status
 *   2. Effective background sample rate and read-error rate
 *   3. Static noise / bias / attitude drift
 *   4. Pause / resume behavior
 *   5. Direct I2C raw-read latency while background sampling is paused
 *   6. reset_yaw() / reset_attitude()
 *   7. Manual motion trace for Roll / Pitch / Yaw response
 *   8. Optional deinit / reinit lifecycle test
 *
 * IMPORTANT:
 *   MPU6050_IMPLEMENTATION must be defined in exactly ONE translation unit.
 *   If another .c file already defines it, remove/comment the define below.
 *
 * BEFORE BOOT:
 *   - Keep the MPU6050 completely stationary.
 *   - Keep the PCB level.
 *   - Keep the MPU6050 Z axis pointing upward.
 *
 * This matters because the supplied header performs automatic level
 * calibration during mpu6050_init().
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include <float.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define MOTOR_CONTROL_IMPLEMENTATION
#include "motor_control.h"

#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"


/* =========================================================================
 * Test configuration
 * ========================================================================= */

#ifndef MPU_TEST_RUN_STATIC
#define MPU_TEST_RUN_STATIC                 1
#endif

#ifndef MPU_TEST_RUN_PAUSE_RESUME
#define MPU_TEST_RUN_PAUSE_RESUME           1
#endif

#ifndef MPU_TEST_RUN_DIRECT_I2C
#define MPU_TEST_RUN_DIRECT_I2C             1
#endif

#ifndef MPU_TEST_RUN_RESET
#define MPU_TEST_RUN_RESET                  1
#endif

/*
 * Manual motion trace:
 *
 *   0..5 s    : level and still
 *   5..10 s   : rotate/tilt around sensor X axis, then return
 *   10..15 s  : rotate/tilt around sensor Y axis, then return
 *   15..20 s  : rotate around sensor Z axis about +90 deg and hold
 *   20..25 s  : keep completely still
 *
 * Follow the sensor board's X/Y/Z axis marks if available.
 */
#ifndef MPU_TEST_RUN_MANUAL_MOTION
#define MPU_TEST_RUN_MANUAL_MOTION          1
#endif

/*
 * Optional lifecycle test.
 * Default OFF because it destroys and recreates the I2C bus when
 * mpu6050_init() owns the bus.
 */
#ifndef MPU_TEST_RUN_REINIT
#define MPU_TEST_RUN_REINIT                 0
#endif


/* =========================================================================
 * Timing
 * ========================================================================= */

#ifndef MPU_TEST_STATIC_DURATION_MS
#define MPU_TEST_STATIC_DURATION_MS         10000U
#endif

#ifndef MPU_TEST_STATIC_POLL_MS
#define MPU_TEST_STATIC_POLL_MS             50U
#endif

#ifndef MPU_TEST_PAUSE_MS
#define MPU_TEST_PAUSE_MS                   500U
#endif

#ifndef MPU_TEST_RESUME_MS
#define MPU_TEST_RESUME_MS                  500U
#endif

#ifndef MPU_TEST_DIRECT_READ_COUNT
#define MPU_TEST_DIRECT_READ_COUNT          200U
#endif

#ifndef MPU_TEST_MANUAL_DURATION_MS
#define MPU_TEST_MANUAL_DURATION_MS         25000U
#endif

#ifndef MPU_TEST_MANUAL_POLL_MS
#define MPU_TEST_MANUAL_POLL_MS             100U
#endif

#ifndef MPU_TEST_MANUAL_COUNTDOWN_MS
#define MPU_TEST_MANUAL_COUNTDOWN_MS        3000U
#endif


/* =========================================================================
 * Suggested acceptance thresholds
 *
 * These are engineering guidance, not MPU6050 datasheet guarantees.
 * ========================================================================= */

#ifndef MPU_TEST_SAMPLE_RATE_MIN_HZ
#define MPU_TEST_SAMPLE_RATE_MIN_HZ         190.0f
#endif

#ifndef MPU_TEST_SAMPLE_RATE_MAX_HZ
#define MPU_TEST_SAMPLE_RATE_MAX_HZ         205.0f
#endif

#ifndef MPU_TEST_ACCEL_MAG_MIN_G
#define MPU_TEST_ACCEL_MAG_MIN_G            0.97f
#endif

#ifndef MPU_TEST_ACCEL_MAG_MAX_G
#define MPU_TEST_ACCEL_MAG_MAX_G            1.03f
#endif

#ifndef MPU_TEST_GYRO_MAG_MEAN_MAX_DPS
#define MPU_TEST_GYRO_MAG_MEAN_MAX_DPS      1.00f
#endif

#ifndef MPU_TEST_STATIONARY_RATIO_MIN
#define MPU_TEST_STATIONARY_RATIO_MIN       0.90f
#endif

#ifndef MPU_TEST_LEVEL_ANGLE_MAX_DEG
#define MPU_TEST_LEVEL_ANGLE_MAX_DEG        3.0f
#endif

/*
 * Yaw drift is reported separately because MPU6050 has no magnetometer.
 * <= 3 deg/min is marked GOOD, <= 10 deg/min WARN, larger FAIL.
 */
#ifndef MPU_TEST_YAW_DRIFT_GOOD_DPM
#define MPU_TEST_YAW_DRIFT_GOOD_DPM         3.0f
#endif

#ifndef MPU_TEST_YAW_DRIFT_WARN_DPM
#define MPU_TEST_YAW_DRIFT_WARN_DPM         10.0f
#endif


static const char *TAG = "MPU_TEST";


/* =========================================================================
 * Small statistics accumulator
 * ========================================================================= */

typedef struct {
    uint32_t n;
    double mean;
    double m2;
    double min;
    double max;
} stat_t;

static void stat_reset(stat_t *s)
{
    if (s == NULL) {
        return;
    }

    s->n = 0U;
    s->mean = 0.0;
    s->m2 = 0.0;
    s->min = DBL_MAX;
    s->max = -DBL_MAX;
}

static void stat_add(stat_t *s, double x)
{
    if (s == NULL) {
        return;
    }

    ++s->n;

    const double delta =
        x - s->mean;

    s->mean +=
        delta / (double)s->n;

    const double delta2 =
        x - s->mean;

    s->m2 +=
        delta * delta2;

    if (x < s->min) {
        s->min = x;
    }

    if (x > s->max) {
        s->max = x;
    }
}

static double stat_stddev(const stat_t *s)
{
    if ((s == NULL) || (s->n < 2U)) {
        return 0.0;
    }

    return sqrt(
        s->m2 /
        (double)(s->n - 1U));
}

static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static const char *pass_fail(bool pass)
{
    return pass ? "PASS" : "FAIL";
}

static float accel_mag(const mpu6050_data_t *d)
{
    return sqrtf(
        d->ax * d->ax +
        d->ay * d->ay +
        d->az * d->az);
}

static float gyro_mag(const mpu6050_data_t *d)
{
    return sqrtf(
        d->gx * d->gx +
        d->gy * d->gy +
        d->gz * d->gz);
}


/* =========================================================================
 * Header / status helpers
 * ========================================================================= */

static void print_status(const char *label)
{
    mpu6050_status_t s = {0};
    mpu6050_get_status(&s);

    ESP_LOGI(
        TAG,
        "%s | init=%d calibrated=%d stationary=%d sampling=%d "
        "WHO_AM_I=0x%02X samples=%" PRIu32 " errors=%" PRIu32,
        label,
        (int)s.initialized,
        (int)s.calibrated,
        (int)s.stationary,
        (int)s.sampling,
        s.who_am_i,
        s.sample_count,
        s.read_error_count);
}

static void print_bias(void)
{
    mpu6050_bias_t b = {0};
    mpu6050_get_bias(&b);

    ESP_LOGI(
        TAG,
        "BIAS | ACC[g]=[%+.6f %+.6f %+.6f] "
        "GYRO[dps]=[%+.6f %+.6f %+.6f]",
        (double)b.ax,
        (double)b.ay,
        (double)b.az,
        (double)b.gx,
        (double)b.gy,
        (double)b.gz);

    ESP_LOGI(
        TAG,
        "CSV,BIAS,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f",
        (double)b.ax,
        (double)b.ay,
        (double)b.az,
        (double)b.gx,
        (double)b.gy,
        (double)b.gz);
}


/* =========================================================================
 * 1. Static quality / effective sample-rate test
 * ========================================================================= */

static void run_static_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "============================================================");
    ESP_LOGI(
        TAG,
        "STATIC TEST: %u ms, keep sensor level and completely still",
        (unsigned)MPU_TEST_STATIC_DURATION_MS);
    ESP_LOGI(
        TAG,
        "============================================================");

    mpu6050_status_t st0 = {0};
    mpu6050_status_t st1 = {0};

    mpu6050_snapshot_t first = {0};
    mpu6050_snapshot_t last = {0};
    mpu6050_snapshot_t snap = {0};

    mpu6050_get_status(&st0);
    mpu6050_get_snapshot(&first);

    const int64_t wall_start_us =
        esp_timer_get_time();

    stat_t ax, ay, az;
    stat_t gx, gy, gz;
    stat_t amag, gmag;
    stat_t roll, pitch;

    stat_reset(&ax);
    stat_reset(&ay);
    stat_reset(&az);

    stat_reset(&gx);
    stat_reset(&gy);
    stat_reset(&gz);

    stat_reset(&amag);
    stat_reset(&gmag);

    stat_reset(&roll);
    stat_reset(&pitch);

    uint32_t stationary_samples = 0U;
    uint32_t observations = 0U;

    float yaw_first = first.attitude.yaw;
    float yaw_last = yaw_first;

    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < MPU_TEST_STATIC_DURATION_MS) {
        delay_ms(MPU_TEST_STATIC_POLL_MS);
        elapsed_ms += MPU_TEST_STATIC_POLL_MS;

        mpu6050_get_snapshot(&snap);

        stat_add(&ax, snap.data.ax);
        stat_add(&ay, snap.data.ay);
        stat_add(&az, snap.data.az);

        stat_add(&gx, snap.data.gx);
        stat_add(&gy, snap.data.gy);
        stat_add(&gz, snap.data.gz);

        stat_add(&amag, accel_mag(&snap.data));
        stat_add(&gmag, gyro_mag(&snap.data));

        stat_add(&roll, snap.attitude.roll);
        stat_add(&pitch, snap.attitude.pitch);

        if (snap.stationary) {
            ++stationary_samples;
        }

        ++observations;
        yaw_last = snap.attitude.yaw;
        last = snap;
    }

    const int64_t wall_end_us =
        esp_timer_get_time();

    mpu6050_get_status(&st1);

    const uint32_t sample_delta =
        st1.sample_count - st0.sample_count;

    const uint32_t error_delta =
        st1.read_error_count - st0.read_error_count;

    const double wall_dt_s =
        (double)(wall_end_us - wall_start_us) /
        1000000.0;

    const double sample_rate_wall_hz =
        (wall_dt_s > 0.0)
        ? ((double)sample_delta / wall_dt_s)
        : 0.0;

    double sample_rate_timestamp_hz =
        sample_rate_wall_hz;

    if ((last.timestamp_us > first.timestamp_us) &&
        (sample_delta > 1U))
    {
        const double sensor_dt_s =
            (double)(last.timestamp_us - first.timestamp_us) /
            1000000.0;

        if (sensor_dt_s > 0.0) {
            sample_rate_timestamp_hz =
                (double)sample_delta /
                sensor_dt_s;
        }
    }

    const double stationary_ratio =
        (observations > 0U)
        ? ((double)stationary_samples /
           (double)observations)
        : 0.0;

    const double yaw_delta_deg =
        (double)yaw_last -
        (double)yaw_first;

    const double yaw_drift_dpm =
        (wall_dt_s > 0.0)
        ? (yaw_delta_deg / wall_dt_s * 60.0)
        : 0.0;

    ESP_LOGI(
        TAG,
        "Sample rate: wall=%.3f Hz timestamp=%.3f Hz "
        "(samples=%" PRIu32 ", errors=%" PRIu32 ")",
        sample_rate_wall_hz,
        sample_rate_timestamp_hz,
        sample_delta,
        error_delta);

    ESP_LOGI(
        TAG,
        "ACC mean[g] = [%+.5f %+.5f %+.5f] | std = [%.5f %.5f %.5f]",
        ax.mean,
        ay.mean,
        az.mean,
        stat_stddev(&ax),
        stat_stddev(&ay),
        stat_stddev(&az));

    ESP_LOGI(
        TAG,
        "GYRO mean[dps] = [%+.5f %+.5f %+.5f] | std = [%.5f %.5f %.5f]",
        gx.mean,
        gy.mean,
        gz.mean,
        stat_stddev(&gx),
        stat_stddev(&gy),
        stat_stddev(&gz));

    ESP_LOGI(
        TAG,
        "|ACC| mean=%.5f g std=%.5f | |GYRO| mean=%.5f dps std=%.5f",
        amag.mean,
        stat_stddev(&amag),
        gmag.mean,
        stat_stddev(&gmag));

    ESP_LOGI(
        TAG,
        "ATT mean[deg] roll=%+.3f pitch=%+.3f | std=[%.3f %.3f]",
        roll.mean,
        pitch.mean,
        stat_stddev(&roll),
        stat_stddev(&pitch));

    ESP_LOGI(
        TAG,
        "Stationary ratio=%.1f%% | yaw drift=%+.4f deg / %.2fs = %+.3f deg/min",
        stationary_ratio * 100.0,
        yaw_delta_deg,
        wall_dt_s,
        yaw_drift_dpm);

    const bool rate_ok =
        (sample_rate_wall_hz >= MPU_TEST_SAMPLE_RATE_MIN_HZ) &&
        (sample_rate_wall_hz <= MPU_TEST_SAMPLE_RATE_MAX_HZ);

    const bool errors_ok =
        (error_delta == 0U);

    const bool amag_ok =
        (amag.mean >= MPU_TEST_ACCEL_MAG_MIN_G) &&
        (amag.mean <= MPU_TEST_ACCEL_MAG_MAX_G);

    const bool gmag_ok =
        (gmag.mean <= MPU_TEST_GYRO_MAG_MEAN_MAX_DPS);

    const bool stationary_ok =
        (stationary_ratio >= MPU_TEST_STATIONARY_RATIO_MIN);

    const bool level_ok =
        (fabs(roll.mean) <= MPU_TEST_LEVEL_ANGLE_MAX_DEG) &&
        (fabs(pitch.mean) <= MPU_TEST_LEVEL_ANGLE_MAX_DEG);

    ESP_LOGI(
        TAG,
        "[%s] effective sample rate %.2f Hz",
        pass_fail(rate_ok),
        sample_rate_wall_hz);

    ESP_LOGI(
        TAG,
        "[%s] I2C/background read errors delta=%" PRIu32,
        pass_fail(errors_ok),
        error_delta);

    ESP_LOGI(
        TAG,
        "[%s] static accel magnitude %.5f g",
        pass_fail(amag_ok),
        amag.mean);

    ESP_LOGI(
        TAG,
        "[%s] static gyro magnitude mean %.5f dps",
        pass_fail(gmag_ok),
        gmag.mean);

    ESP_LOGI(
        TAG,
        "[%s] stationary detector ratio %.1f%%",
        pass_fail(stationary_ok),
        stationary_ratio * 100.0);

    ESP_LOGI(
        TAG,
        "[%s] level attitude roll=%+.2f pitch=%+.2f deg",
        pass_fail(level_ok),
        roll.mean,
        pitch.mean);

    const char *yaw_grade =
        (fabs(yaw_drift_dpm) <= MPU_TEST_YAW_DRIFT_GOOD_DPM)
        ? "GOOD"
        : ((fabs(yaw_drift_dpm) <= MPU_TEST_YAW_DRIFT_WARN_DPM)
           ? "WARN"
           : "FAIL");

    ESP_LOGI(
        TAG,
        "[%s] yaw drift = %+.3f deg/min (gyro-only yaw)",
        yaw_grade,
        yaw_drift_dpm);

    ESP_LOGI(
        TAG,
        "CSV,STATIC,"
        "rate_wall_hz,%.5f,"
        "rate_timestamp_hz,%.5f,"
        "sample_delta,%" PRIu32 ","
        "error_delta,%" PRIu32 ","
        "ax_mean,%.7f,ay_mean,%.7f,az_mean,%.7f,"
        "ax_std,%.7f,ay_std,%.7f,az_std,%.7f,"
        "gx_mean,%.7f,gy_mean,%.7f,gz_mean,%.7f,"
        "gx_std,%.7f,gy_std,%.7f,gz_std,%.7f,"
        "acc_mag_mean,%.7f,acc_mag_std,%.7f,"
        "gyro_mag_mean,%.7f,gyro_mag_std,%.7f,"
        "roll_mean,%.7f,pitch_mean,%.7f,"
        "roll_std,%.7f,pitch_std,%.7f,"
        "stationary_ratio,%.7f,"
        "yaw_delta_deg,%.7f,"
        "yaw_drift_deg_min,%.7f",
        sample_rate_wall_hz,
        sample_rate_timestamp_hz,
        sample_delta,
        error_delta,
        ax.mean, ay.mean, az.mean,
        stat_stddev(&ax),
        stat_stddev(&ay),
        stat_stddev(&az),
        gx.mean, gy.mean, gz.mean,
        stat_stddev(&gx),
        stat_stddev(&gy),
        stat_stddev(&gz),
        amag.mean,
        stat_stddev(&amag),
        gmag.mean,
        stat_stddev(&gmag),
        roll.mean,
        pitch.mean,
        stat_stddev(&roll),
        stat_stddev(&pitch),
        stationary_ratio,
        yaw_delta_deg,
        yaw_drift_dpm);
}


/* =========================================================================
 * 2. Pause / resume
 * ========================================================================= */

static void run_pause_resume_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "============================================================");
    ESP_LOGI(TAG, "PAUSE / RESUME TEST");
    ESP_LOGI(
        TAG,
        "============================================================");

    mpu6050_status_t before = {0};
    mpu6050_status_t paused0 = {0};
    mpu6050_status_t paused1 = {0};
    mpu6050_status_t resumed = {0};

    mpu6050_get_status(&before);

    mpu6050_pause();
    mpu6050_get_status(&paused0);

    delay_ms(MPU_TEST_PAUSE_MS);

    mpu6050_get_status(&paused1);

    const uint32_t paused_delta =
        paused1.sample_count -
        paused0.sample_count;

    const bool pause_ok =
        !paused1.sampling &&
        (paused_delta == 0U);

    ESP_LOGI(
        TAG,
        "[%s] pause: sampling=%d sample_count delta=%" PRIu32,
        pass_fail(pause_ok),
        (int)paused1.sampling,
        paused_delta);

    mpu6050_resume();

    delay_ms(MPU_TEST_RESUME_MS);

    mpu6050_get_status(&resumed);

    const uint32_t resume_delta =
        resumed.sample_count -
        paused1.sample_count;

    const float expected_samples =
        ((float)MPU_TEST_RESUME_MS / 1000.0f) *
        (float)MPU6050_SAMPLE_RATE_HZ;

    const bool resume_ok =
        resumed.sampling &&
        (resume_delta >=
         (uint32_t)(expected_samples * 0.80f));

    ESP_LOGI(
        TAG,
        "[%s] resume: sampling=%d sample_count delta=%" PRIu32
        " expected~%.1f",
        pass_fail(resume_ok),
        (int)resumed.sampling,
        resume_delta,
        (double)expected_samples);

    ESP_LOGI(
        TAG,
        "CSV,PAUSE_RESUME,pause_delta,%" PRIu32
        ",resume_delta,%" PRIu32
        ",pause_ok,%d,resume_ok,%d",
        paused_delta,
        resume_delta,
        (int)pause_ok,
        (int)resume_ok);

    (void)before;
}


/* =========================================================================
 * 3. Direct raw-I2C latency test
 * ========================================================================= */

static void run_direct_i2c_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "============================================================");
    ESP_LOGI(
        TAG,
        "DIRECT I2C RAW READ TEST: %" PRIu32 " reads",
        (uint32_t)MPU_TEST_DIRECT_READ_COUNT);
    ESP_LOGI(
        TAG,
        "============================================================");

    /*
     * Pause background sampling to avoid measuring mutex contention.
     */
    mpu6050_pause();

    delay_ms(50U);

    stat_t latency_us;
    stat_reset(&latency_us);

    uint32_t ok_count = 0U;
    uint32_t error_count = 0U;

    mpu6050_raw_t raw = {0};

    const int64_t all_start_us =
        esp_timer_get_time();

    for (uint32_t i = 0;
         i < MPU_TEST_DIRECT_READ_COUNT;
         ++i)
    {
        const int64_t t0 =
            esp_timer_get_time();

        const esp_err_t ret =
            mpu6050_read_raw_now(&raw);

        const int64_t t1 =
            esp_timer_get_time();

        stat_add(
            &latency_us,
            (double)(t1 - t0));

        if (ret == ESP_OK) {
            ++ok_count;
        } else {
            ++error_count;
        }
    }

    const int64_t all_end_us =
        esp_timer_get_time();

    const double total_s =
        (double)(all_end_us - all_start_us) /
        1000000.0;

    const double throughput_hz =
        (total_s > 0.0)
        ? ((double)MPU_TEST_DIRECT_READ_COUNT /
           total_s)
        : 0.0;

    ESP_LOGI(
        TAG,
        "Direct read latency: mean=%.1f us std=%.1f us "
        "min=%.1f us max=%.1f us",
        latency_us.mean,
        stat_stddev(&latency_us),
        latency_us.min,
        latency_us.max);

    ESP_LOGI(
        TAG,
        "Direct raw throughput=%.1f reads/s | ok=%" PRIu32
        " errors=%" PRIu32,
        throughput_hz,
        ok_count,
        error_count);

    ESP_LOGI(
        TAG,
        "[%s] direct I2C read reliability",
        pass_fail(error_count == 0U));

    ESP_LOGI(
        TAG,
        "CSV,DIRECT_I2C,"
        "count,%" PRIu32 ",ok,%" PRIu32 ",errors,%" PRIu32 ","
        "mean_us,%.3f,std_us,%.3f,min_us,%.3f,max_us,%.3f,"
        "throughput_hz,%.3f",
        (uint32_t)MPU_TEST_DIRECT_READ_COUNT,
        ok_count,
        error_count,
        latency_us.mean,
        stat_stddev(&latency_us),
        latency_us.min,
        latency_us.max,
        throughput_hz);

    mpu6050_resume();
    delay_ms(200U);
}


/* =========================================================================
 * 4. Reset API test
 * ========================================================================= */

static void run_reset_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "============================================================");
    ESP_LOGI(
        TAG,
        "RESET API TEST - keep sensor level and still");
    ESP_LOGI(
        TAG,
        "============================================================");

    mpu6050_snapshot_t before = {0};
    mpu6050_snapshot_t after_yaw = {0};
    mpu6050_snapshot_t after_att = {0};

    mpu6050_get_snapshot(&before);

    mpu6050_reset_yaw();
    delay_ms(50U);
    mpu6050_get_snapshot(&after_yaw);

    const bool yaw_reset_ok =
        fabsf(after_yaw.attitude.yaw) < 1.0f;

    ESP_LOGI(
        TAG,
        "[%s] reset_yaw: before=%+.3f after=%+.3f deg",
        pass_fail(yaw_reset_ok),
        (double)before.attitude.yaw,
        (double)after_yaw.attitude.yaw);

    mpu6050_reset_attitude();
    delay_ms(150U);
    mpu6050_get_snapshot(&after_att);

    const bool attitude_ok =
        (fabsf(after_att.attitude.roll) <
         MPU_TEST_LEVEL_ANGLE_MAX_DEG) &&
        (fabsf(after_att.attitude.pitch) <
         MPU_TEST_LEVEL_ANGLE_MAX_DEG) &&
        (fabsf(after_att.attitude.yaw) < 1.0f);

    ESP_LOGI(
        TAG,
        "[%s] reset_attitude: R=%+.3f P=%+.3f Y=%+.3f deg",
        pass_fail(attitude_ok),
        (double)after_att.attitude.roll,
        (double)after_att.attitude.pitch,
        (double)after_att.attitude.yaw);

    ESP_LOGI(
        TAG,
        "CSV,RESET,yaw_before,%.5f,yaw_after,%.5f,"
        "roll_after,%.5f,pitch_after,%.5f,yaw2_after,%.5f,"
        "yaw_ok,%d,attitude_ok,%d",
        (double)before.attitude.yaw,
        (double)after_yaw.attitude.yaw,
        (double)after_att.attitude.roll,
        (double)after_att.attitude.pitch,
        (double)after_att.attitude.yaw,
        (int)yaw_reset_ok,
        (int)attitude_ok);
}


/* =========================================================================
 * 5. Manual dynamic motion trace
 * ========================================================================= */

static const char *manual_stage(uint32_t elapsed_ms)
{
    if (elapsed_ms < 5000U) {
        return "LEVEL_STILL";
    }

    if (elapsed_ms < 10000U) {
        return "MOVE_X_ROLL";
    }

    if (elapsed_ms < 15000U) {
        return "MOVE_Y_PITCH";
    }

    if (elapsed_ms < 20000U) {
        return "MOVE_Z_YAW";
    }

    return "FINAL_STILL";
}

static void run_manual_motion_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "============================================================");
    ESP_LOGI(TAG, "MANUAL MOTION TRACE");
    ESP_LOGI(
        TAG,
        "============================================================");

    ESP_LOGW(TAG, "Follow sensor PCB X/Y/Z axes.");
    ESP_LOGW(TAG, "0-5s   : keep LEVEL and STILL.");
    ESP_LOGW(TAG, "5-10s  : tilt around X axis, then return.");
    ESP_LOGW(TAG, "10-15s : tilt around Y axis, then return.");
    ESP_LOGW(TAG, "15-20s : rotate around Z axis about +90deg, then hold.");
    ESP_LOGW(TAG, "20-25s : keep completely STILL.");

    ESP_LOGI(
        TAG,
        "CSV_HEADER,MANUAL,t_ms,stage,"
        "ax,ay,az,gx,gy,gz,roll,pitch,yaw,stationary,sample_count");

    for (int sec = (int)(MPU_TEST_MANUAL_COUNTDOWN_MS / 1000U);
         sec > 0;
         --sec)
    {
        ESP_LOGW(
            TAG,
            "Manual test starts in %d...",
            sec);

        delay_ms(1000U);
    }

    mpu6050_reset_attitude();
    delay_ms(100U);

    uint32_t elapsed_ms = 0U;

    while (elapsed_ms <= MPU_TEST_MANUAL_DURATION_MS) {
        mpu6050_snapshot_t s = {0};
        mpu6050_get_snapshot(&s);

        const char *stage =
            manual_stage(elapsed_ms);

        ESP_LOGI(
            TAG,
            "CSV,MANUAL,%" PRIu32 ",%s,"
            "%.5f,%.5f,%.5f,"
            "%.4f,%.4f,%.4f,"
            "%.3f,%.3f,%.3f,"
            "%d,%" PRIu32,
            elapsed_ms,
            stage,
            (double)s.data.ax,
            (double)s.data.ay,
            (double)s.data.az,
            (double)s.data.gx,
            (double)s.data.gy,
            (double)s.data.gz,
            (double)s.attitude.roll,
            (double)s.attitude.pitch,
            (double)s.attitude.yaw,
            (int)s.stationary,
            s.sample_count);

        delay_ms(MPU_TEST_MANUAL_POLL_MS);
        elapsed_ms += MPU_TEST_MANUAL_POLL_MS;
    }

    ESP_LOGI(
        TAG,
        "Manual trace complete.");

    mpu6050_print_now();
}


/* =========================================================================
 * 6. Optional deinit / reinit
 * ========================================================================= */

static void run_reinit_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "============================================================");
    ESP_LOGI(TAG, "DEINIT / REINIT TEST");
    ESP_LOGI(
        TAG,
        "============================================================");

    const esp_err_t deinit_ret =
        mpu6050_deinit();

    ESP_LOGI(
        TAG,
        "deinit -> %s",
        esp_err_to_name(deinit_ret));

    delay_ms(200U);

    const esp_err_t init_ret =
        mpu6050_init();

    ESP_LOGI(
        TAG,
        "reinit -> %s",
        esp_err_to_name(init_ret));

    delay_ms(500U);

    mpu6050_status_t s = {0};
    mpu6050_get_status(&s);

    const bool ok =
        (deinit_ret == ESP_OK) &&
        (init_ret == ESP_OK) &&
        s.initialized &&
        s.calibrated &&
        s.sampling &&
        (s.who_am_i == MPU6050_WHO_AM_I_VALUE);

    ESP_LOGI(
        TAG,
        "[%s] deinit/reinit lifecycle",
        pass_fail(ok));

    ESP_LOGI(
        TAG,
        "CSV,REINIT,deinit_ret,%d,init_ret,%d,"
        "initialized,%d,calibrated,%d,sampling,%d,who_am_i,0x%02X",
        (int)deinit_ret,
        (int)init_ret,
        (int)s.initialized,
        (int)s.calibrated,
        (int)s.sampling,
        s.who_am_i);
}


/* =========================================================================
 * Main
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# MPU6050 PERFORMANCE / REGRESSION TEST");
    ESP_LOGI(
        TAG,
        "############################################################");

    ESP_LOGI(
        TAG,
        "Configured: I2C=%u Hz, sample=%u Hz, dt=%.4f s, "
        "gyro=+/-500 dps, accel=+/-4g",
        (unsigned)MPU6050_I2C_FREQ_HZ,
        (unsigned)MPU6050_SAMPLE_RATE_HZ,
        (double)MPU6050_SAMPLE_DT);

    ESP_LOGI(
        TAG,
        "Filters: DLPF_CFG=%u accel_LPF=%.2f gyro_LPF=%.2f "
        "complementary=%.2f",
        (unsigned)MPU6050_DLPF_CFG,
        (double)MPU6050_ACCEL_LPF_ALPHA,
        (double)MPU6050_GYRO_LPF_ALPHA,
        (double)MPU6050_COMPLEMENTARY_ALPHA);

    ESP_LOGW(
        TAG,
        "IMPORTANT: keep sensor LEVEL + STILL during init calibration.");

    const esp_err_t ret =
        mpu6050_init();

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mpu6050_init failed: %s (0x%x)",
            esp_err_to_name(ret),
            (unsigned)ret);
        return;
    }

    /*
     * Give the background task a short time to produce snapshots
     * after the automatic calibration finishes.
     */
    delay_ms(500U);

    print_status("after init");
    print_bias();
    mpu6050_print_now();

    mpu6050_status_t init_status = {0};
    mpu6050_get_status(&init_status);

    const bool init_ok =
        init_status.initialized &&
        init_status.calibrated &&
        init_status.sampling &&
        (init_status.who_am_i ==
         MPU6050_WHO_AM_I_VALUE);

    ESP_LOGI(
        TAG,
        "[%s] init/status/WHO_AM_I",
        pass_fail(init_ok));

#if MPU_TEST_RUN_STATIC
    run_static_test();
#endif

#if MPU_TEST_RUN_PAUSE_RESUME
    run_pause_resume_test();
#endif

#if MPU_TEST_RUN_DIRECT_I2C
    run_direct_i2c_test();
#endif

#if MPU_TEST_RUN_RESET
    run_reset_test();
#endif

#if MPU_TEST_RUN_MANUAL_MOTION
    run_manual_motion_test();
#endif

#if MPU_TEST_RUN_REINIT
    run_reinit_test();
#endif

    ESP_LOGI(TAG, "");
    ESP_LOGI(
        TAG,
        "############################################################");
    ESP_LOGI(
        TAG,
        "# TEST COMPLETE");
    ESP_LOGI(
        TAG,
        "############################################################");

    print_status("final");

    ESP_LOGI(
        TAG,
        "Send back these lines first:");
    ESP_LOGI(
        TAG,
        "  CSV,BIAS");
    ESP_LOGI(
        TAG,
        "  CSV,STATIC");
    ESP_LOGI(
        TAG,
        "  CSV,PAUSE_RESUME");
    ESP_LOGI(
        TAG,
        "  CSV,DIRECT_I2C");
    ESP_LOGI(
        TAG,
        "and the MANUAL section if you want attitude-response analysis.");
}