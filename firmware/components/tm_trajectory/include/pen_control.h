#ifndef PEN_CONTROL_H
#define PEN_CONTROL_H

/*
 * pen_control.h
 *
 * 1-DOF Z-axis pen actuator for an MG90S-class hobby servo.
 *
 * Mechanical model:
 *
 *     MG90S rotation
 *          ↓
 *     linkage / crank
 *          ↓
 *     vertical slider
 *          ↓
 *     passive compliant joint
 *          ↓
 *         pen
 *
 * This module controls ONLY vertical pen actuation.
 *
 * It deliberately does NOT know:
 *   - trajectory records
 *   - LINE / CIRCLE / CUBIC geometry
 *   - WORLD / BODY XY motion
 *   - trajectory_runner
 *   - trajectory_tracker
 *   - motor_control
 *
 * The omni-wheel chassis owns all XY motion. The optional passive joint is
 * purely mechanical and has no firmware state.
 *
 * --------------------------------------------------------------------------
 * Safety / calibration model
 * --------------------------------------------------------------------------
 *
 * Servo horn installation direction is not known in advance. Therefore this
 * module does NOT assume that a smaller pulse means UP or that a larger pulse
 * means DOWN.
 *
 * Before the mechanism is calibrated:
 *
 *   config.positions_calibrated = false
 *
 * init() commands only config.startup_pulse_us (default 1500 us) and enters
 * PEN_CONTROL_CALIBRATION. request_up()/request_down() are rejected.
 *
 * Calibration code may use:
 *
 *   pen_control_set_calibration_pulse_us()
 *
 * and then:
 *
 *   pen_control_set_calibrated_positions(up_us, down_us)
 *
 * Once calibrated, request_up()/request_down() perform non-blocking motion.
 *
 * In production:
 *
 *   config.positions_calibrated = true
 *   config.up_pulse_us   = measured UP value
 *   config.down_pulse_us = measured DOWN value
 *
 * init() then immediately commands the calibrated UP pulse as the startup
 * safe state and waits raise_time_ms + settle_time_ms before reporting UP.
 *
 * --------------------------------------------------------------------------
 * Non-blocking motion
 * --------------------------------------------------------------------------
 *
 * request_up()/request_down() return after setting a target. Call update()
 * periodically (for example from the Runner's existing 10 ms loop).
 *
 * PWM pulse width is linearly ramped from the current commanded pulse to the
 * target pulse over raise_time_ms / lower_time_ms. A request that reverses a
 * motion midway scales the duration according to the remaining pulse travel.
 *
 * After the target pulse is reached, state remains MOVING_UP/MOVING_DOWN for
 * settle_time_ms before becoming UP/DOWN.
 *
 * UP/DOWN means "the commanded motion/settle time has completed". A normal
 * hobby servo does not provide position feedback to this module.
 *
 * --------------------------------------------------------------------------
 * LEDC resource ownership
 * --------------------------------------------------------------------------
 *
 * This module configures exactly one LEDC timer + one LEDC channel supplied
 * by pen_control_config_t.
 *
 * The caller MUST choose resources that are not already owned by motor PWM or
 * another peripheral. No GPIO/timer/channel is silently hard-coded.
 *
 * Default config intentionally sets gpio_num = -1 so init() fails until the
 * application explicitly selects an output GPIO.
 *
 * --------------------------------------------------------------------------
 * Single-header usage
 * --------------------------------------------------------------------------
 *
 * In exactly ONE .c/.cpp file:
 *
 *     #define PEN_CONTROL_IMPLEMENTATION
 *     #include "pen_control.h"
 *
 * In all other files:
 *
 *     #include "pen_control.h"
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================
 * 1. Defaults
 * ============================================================ */

#ifndef PEN_CONTROL_DEFAULT_PWM_FREQUENCY_HZ
#define PEN_CONTROL_DEFAULT_PWM_FREQUENCY_HZ            50U
#endif

#ifndef PEN_CONTROL_DEFAULT_DUTY_RESOLUTION
#define PEN_CONTROL_DEFAULT_DUTY_RESOLUTION              LEDC_TIMER_14_BIT
#endif

#ifndef PEN_CONTROL_DEFAULT_SAFE_MIN_PULSE_US
#define PEN_CONTROL_DEFAULT_SAFE_MIN_PULSE_US             1000U
#endif

#ifndef PEN_CONTROL_DEFAULT_SAFE_MAX_PULSE_US
#define PEN_CONTROL_DEFAULT_SAFE_MAX_PULSE_US             2000U
#endif

#ifndef PEN_CONTROL_DEFAULT_STARTUP_PULSE_US
#define PEN_CONTROL_DEFAULT_STARTUP_PULSE_US              1500U
#endif

/*
 * Placeholder positions used only to fill default config.
 * positions_calibrated=false prevents request_up/down from using them.
 */
#ifndef PEN_CONTROL_DEFAULT_UP_PULSE_US
#define PEN_CONTROL_DEFAULT_UP_PULSE_US                   1200U
#endif

#ifndef PEN_CONTROL_DEFAULT_DOWN_PULSE_US
#define PEN_CONTROL_DEFAULT_DOWN_PULSE_US                 1800U
#endif

#ifndef PEN_CONTROL_DEFAULT_RAISE_TIME_MS
#define PEN_CONTROL_DEFAULT_RAISE_TIME_MS                 180U
#endif

#ifndef PEN_CONTROL_DEFAULT_LOWER_TIME_MS
#define PEN_CONTROL_DEFAULT_LOWER_TIME_MS                 180U
#endif

#ifndef PEN_CONTROL_DEFAULT_SETTLE_TIME_MS
#define PEN_CONTROL_DEFAULT_SETTLE_TIME_MS                80U
#endif


/* ============================================================
 * 2. State / error
 * ============================================================ */

typedef enum
{
    PEN_CONTROL_UNINITIALIZED = 0,

    PEN_CONTROL_CALIBRATION,

    PEN_CONTROL_MOVING_UP,
    PEN_CONTROL_UP,

    PEN_CONTROL_MOVING_DOWN,
    PEN_CONTROL_DOWN,

    PEN_CONTROL_ERROR

} pen_control_state_t;


typedef enum
{
    PEN_CONTROL_ERROR_NONE = 0,

    PEN_CONTROL_ERROR_INVALID_ARGUMENT,
    PEN_CONTROL_ERROR_INVALID_CONFIG,
    PEN_CONTROL_ERROR_NOT_INITIALIZED,
    PEN_CONTROL_ERROR_NOT_CALIBRATED,

    PEN_CONTROL_ERROR_LEDC_TIMER,
    PEN_CONTROL_ERROR_LEDC_CHANNEL,
    PEN_CONTROL_ERROR_PWM_WRITE,

    PEN_CONTROL_ERROR_INVALID_STATE

} pen_control_error_t;


/* ============================================================
 * 3. Configuration / status / instance
 * ============================================================ */

typedef struct
{
    /*
     * Must be explicitly selected by the application.
     * Default = -1, therefore default config cannot accidentally drive a pin.
     */
    int gpio_num;

    ledc_mode_t speed_mode;
    ledc_timer_t timer_num;
    ledc_channel_t channel;
    ledc_timer_bit_t duty_resolution;
    ledc_clk_cfg_t clock_source;

    uint32_t pwm_frequency_hz;

    uint16_t safe_min_pulse_us;
    uint16_t safe_max_pulse_us;

    /*
     * Used only while positions_calibrated=false.
     */
    uint16_t startup_pulse_us;

    bool positions_calibrated;

    /*
     * Direction-agnostic. Either value may be numerically larger.
     */
    uint16_t up_pulse_us;
    uint16_t down_pulse_us;

    /*
     * Time to move the full calibrated UP↔DOWN pulse distance.
     * Mid-motion reversal scales the requested duration proportionally.
     */
    uint32_t raise_time_ms;
    uint32_t lower_time_ms;

    /*
     * Extra time after the final target pulse is commanded.
     */
    uint32_t settle_time_ms;

} pen_control_config_t;


typedef struct
{
    bool initialized;
    bool positions_calibrated;

    pen_control_state_t state;
    pen_control_error_t error;

    bool busy;
    bool settling;

    uint16_t current_pulse_us;
    uint16_t target_pulse_us;

    uint16_t up_pulse_us;
    uint16_t down_pulse_us;

    float motion_progress;

    uint32_t motion_remaining_ms;
    uint32_t settle_remaining_ms;

    int gpio_num;
    ledc_timer_t timer_num;
    ledc_channel_t channel;
    uint32_t pwm_frequency_hz;

} pen_control_status_t;


typedef struct
{
    bool initialized;

    pen_control_config_t config;

    pen_control_state_t state;
    pen_control_error_t error;

    uint16_t current_pulse_us;
    uint16_t start_pulse_us;
    uint16_t target_pulse_us;

    int64_t motion_start_us;
    int64_t motion_end_us;
    int64_t settle_end_us;

} pen_control_t;


#define PEN_CONTROL_INITIALIZER                    \
    {                                              \
        .initialized = false,                      \
        .config = {0},                             \
        .state = PEN_CONTROL_UNINITIALIZED,        \
        .error = PEN_CONTROL_ERROR_NONE,           \
        .current_pulse_us = 0U,                    \
        .start_pulse_us = 0U,                      \
        .target_pulse_us = 0U,                     \
        .motion_start_us = 0,                      \
        .motion_end_us = 0,                        \
        .settle_end_us = 0                         \
    }


/* ============================================================
 * 4. Public API
 * ============================================================ */

/**
 * Fill configuration with conservative MG90S bring-up defaults.
 *
 * Important:
 *   gpio_num = -1
 *   positions_calibrated = false
 *
 * The caller must select an unused GPIO/timer/channel before init().
 */
void pen_control_get_default_config(
    pen_control_config_t *config);


/**
 * Configure LEDC and start the actuator.
 *
 * If positions_calibrated=false:
 *   output startup_pulse_us
 *   state = CALIBRATION
 *
 * If positions_calibrated=true:
 *   output up_pulse_us immediately as a safety command
 *   state = MOVING_UP until raise_time_ms + settle_time_ms expires
 *
 * This function does not block for servo motion.
 */
esp_err_t pen_control_init(
    pen_control_t *pen,
    const pen_control_config_t *config);


/**
 * Non-blocking request.
 *
 * If already UP / already moving toward UP, returns ESP_OK without restarting
 * the motion timer.
 */
esp_err_t pen_control_request_up(
    pen_control_t *pen);


/**
 * Non-blocking request.
 *
 * If already DOWN / already moving toward DOWN, returns ESP_OK without
 * restarting the motion timer.
 */
esp_err_t pen_control_request_down(
    pen_control_t *pen);


/**
 * Advance ramp/settle state using esp_timer_get_time().
 *
 * Call periodically while the system is running.
 */
esp_err_t pen_control_update(
    pen_control_t *pen);


/**
 * Calibration-only direct pulse command.
 *
 * Pulse must remain inside [safe_min_pulse_us, safe_max_pulse_us].
 * This cancels any active UP/DOWN motion and puts state in CALIBRATION.
 */
esp_err_t pen_control_set_calibration_pulse_us(
    pen_control_t *pen,
    uint16_t pulse_us);


/**
 * Mark measured UP and DOWN pulse widths as the current calibration.
 *
 * Values may be in either numeric order, but must be distinct and inside the
 * safe pulse range. This changes only runtime configuration; it does NOT
 * persist values to NVS.
 */
esp_err_t pen_control_set_calibrated_positions(
    pen_control_t *pen,
    uint16_t up_pulse_us,
    uint16_t down_pulse_us);


/**
 * Safety action.
 *
 * Immediately commands the calibrated UP pulse without a soft ramp. The
 * function does not wait for physical motion. If the previous state was ERROR
 * and the PWM write succeeds, this call acts as an explicit recovery action
 * and restarts the UP timing window.
 */
esp_err_t pen_control_emergency_up(
    pen_control_t *pen);


bool pen_control_is_ready(
    const pen_control_t *pen);

bool pen_control_is_busy(
    const pen_control_t *pen);

bool pen_control_is_up(
    const pen_control_t *pen);

bool pen_control_is_down(
    const pen_control_t *pen);

bool pen_control_is_calibrated(
    const pen_control_t *pen);


pen_control_state_t pen_control_get_state(
    const pen_control_t *pen);

pen_control_error_t pen_control_get_error(
    const pen_control_t *pen);


void pen_control_get_status(
    const pen_control_t *pen,
    pen_control_status_t *status);


const char *pen_control_state_name(
    pen_control_state_t state);

const char *pen_control_error_name(
    pen_control_error_t error);


#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 * ============================================================ */

#ifdef PEN_CONTROL_IMPLEMENTATION

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PEN_CONTROL_TAG "pen_control"


static bool pen_control_config_valid(
    const pen_control_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if (config->gpio_num < 0)
    {
        return false;
    }

    if (config->pwm_frequency_hz == 0U)
    {
        return false;
    }

    if ((config->safe_min_pulse_us == 0U) ||
        (config->safe_min_pulse_us >=
         config->safe_max_pulse_us))
    {
        return false;
    }

    if ((config->startup_pulse_us <
         config->safe_min_pulse_us) ||
        (config->startup_pulse_us >
         config->safe_max_pulse_us))
    {
        return false;
    }

    if ((config->up_pulse_us <
         config->safe_min_pulse_us) ||
        (config->up_pulse_us >
         config->safe_max_pulse_us) ||
        (config->down_pulse_us <
         config->safe_min_pulse_us) ||
        (config->down_pulse_us >
         config->safe_max_pulse_us))
    {
        return false;
    }

    if (config->positions_calibrated &&
        (config->up_pulse_us ==
         config->down_pulse_us))
    {
        return false;
    }

    if ((config->raise_time_ms == 0U) ||
        (config->lower_time_ms == 0U))
    {
        return false;
    }

    /*
     * LEDC duty_resolution enum values are the bit count on current ESP-IDF
     * targets. Keep a conservative generic numeric sanity check here; the
     * driver remains the final authority.
     */
    const int resolution_bits =
        (int)config->duty_resolution;

    if ((resolution_bits <= 0) ||
        (resolution_bits >= 31))
    {
        return false;
    }

    /*
     * Safe pulse must fit inside one PWM period.
     */
    const uint64_t max_pulse_frequency_product =
        (uint64_t)config->safe_max_pulse_us *
        (uint64_t)config->pwm_frequency_hz;

    if (max_pulse_frequency_product >=
        1000000ULL)
    {
        return false;
    }

    return true;
}


static uint32_t pen_control_pulse_us_to_duty(
    const pen_control_t *pen,
    uint16_t pulse_us)
{
    const uint32_t resolution_bits =
        (uint32_t)pen->config.duty_resolution;

    const uint64_t full_scale =
        1ULL << resolution_bits;

    /*
     * duty / full_scale
     * =
     * pulse_us / period_us
     * =
     * pulse_us * frequency_hz / 1e6
     */
    uint64_t numerator =
        (uint64_t)pulse_us *
        (uint64_t)pen->config.pwm_frequency_hz *
        full_scale;

    uint64_t duty =
        (numerator + 500000ULL) /
        1000000ULL;

    if (duty >= full_scale)
    {
        duty =
            full_scale - 1ULL;
    }

    return (uint32_t)duty;
}


static esp_err_t pen_control_write_pulse(
    pen_control_t *pen,
    uint16_t pulse_us)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if ((pulse_us <
         pen->config.safe_min_pulse_us) ||
        (pulse_us >
         pen->config.safe_max_pulse_us))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t duty =
        pen_control_pulse_us_to_duty(
            pen,
            pulse_us);

    /*
     * The pen position is software-ramped by pen_control_update(). Do not
     * use ledc_set_duty_and_update() here: that API goes through the LEDC
     * fade channel and requires ledc_fade_func_install(), which is not part
     * of this module's resource ownership. A normal duty write is sufficient
     * and works without the fade service.
     */
    esp_err_t ret =
        ledc_set_duty(
            pen->config.speed_mode,
            pen->config.channel,
            duty);

    if (ret == ESP_OK)
    {
        ret =
            ledc_update_duty(
                pen->config.speed_mode,
                pen->config.channel);
    }

    if (ret != ESP_OK)
    {
        return ret;
    }

    pen->current_pulse_us =
        pulse_us;

    return ESP_OK;
}


static esp_err_t pen_control_latch_error(
    pen_control_t *pen,
    pen_control_error_t error,
    esp_err_t ret,
    const char *message)
{
    if (pen != NULL)
    {
        pen->state =
            PEN_CONTROL_ERROR;

        pen->error =
            error;
    }

    if (message != NULL)
    {
        ESP_LOGE(
            PEN_CONTROL_TAG,
            "%s: %s",
            message,
            esp_err_to_name(ret));
    }

    return ret;
}


static uint32_t pen_control_scaled_move_time_ms(
    const pen_control_t *pen,
    uint16_t from_pulse_us,
    uint16_t to_pulse_us,
    uint32_t full_move_time_ms)
{
    const uint32_t full_delta =
        (pen->config.up_pulse_us >
         pen->config.down_pulse_us)
            ? (uint32_t)(
                pen->config.up_pulse_us -
                pen->config.down_pulse_us)
            : (uint32_t)(
                pen->config.down_pulse_us -
                pen->config.up_pulse_us);

    const uint32_t remaining_delta =
        (from_pulse_us > to_pulse_us)
            ? (uint32_t)(
                from_pulse_us -
                to_pulse_us)
            : (uint32_t)(
                to_pulse_us -
                from_pulse_us);

    if ((full_delta == 0U) ||
        (remaining_delta == 0U))
    {
        return 0U;
    }

    uint64_t scaled =
        (uint64_t)full_move_time_ms *
        (uint64_t)remaining_delta;

    scaled =
        (scaled + full_delta - 1U) /
        full_delta;

    if (scaled == 0U)
    {
        scaled =
            1U;
    }

    if (scaled > UINT32_MAX)
    {
        scaled =
            UINT32_MAX;
    }

    return (uint32_t)scaled;
}


static esp_err_t pen_control_start_motion(
    pen_control_t *pen,
    bool target_is_up)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!pen->config.positions_calibrated)
    {
        return pen_control_latch_error(
            pen,
            PEN_CONTROL_ERROR_NOT_CALIBRATED,
            ESP_ERR_INVALID_STATE,
            "Pen positions are not calibrated");
    }

    /*
     * Bring current commanded pulse up to date before reversing or retargeting.
     */
    esp_err_t ret =
        pen_control_update(
            pen);

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (target_is_up)
    {
        if ((pen->state == PEN_CONTROL_UP) ||
            (pen->state == PEN_CONTROL_MOVING_UP))
        {
            return ESP_OK;
        }
    }
    else
    {
        if ((pen->state == PEN_CONTROL_DOWN) ||
            (pen->state == PEN_CONTROL_MOVING_DOWN))
        {
            return ESP_OK;
        }
    }

    const uint16_t target =
        target_is_up
            ? pen->config.up_pulse_us
            : pen->config.down_pulse_us;

    const uint32_t full_time_ms =
        target_is_up
            ? pen->config.raise_time_ms
            : pen->config.lower_time_ms;

    const uint32_t move_time_ms =
        pen_control_scaled_move_time_ms(
            pen,
            pen->current_pulse_us,
            target,
            full_time_ms);

    const int64_t now =
        esp_timer_get_time();

    pen->start_pulse_us =
        pen->current_pulse_us;

    pen->target_pulse_us =
        target;

    pen->motion_start_us =
        now;

    pen->motion_end_us =
        now +
        ((int64_t)move_time_ms *
         1000LL);

    pen->settle_end_us =
        pen->motion_end_us +
        ((int64_t)pen->config.settle_time_ms *
         1000LL);

    pen->state =
        target_is_up
            ? PEN_CONTROL_MOVING_UP
            : PEN_CONTROL_MOVING_DOWN;

    pen->error =
        PEN_CONTROL_ERROR_NONE;

    /*
     * If no ramp is needed, ensure target is commanded immediately.
     */
    if (move_time_ms == 0U)
    {
        ret =
            pen_control_write_pulse(
                pen,
                target);

        if (ret != ESP_OK)
        {
            return pen_control_latch_error(
                pen,
                PEN_CONTROL_ERROR_PWM_WRITE,
                ret,
                "Failed to command pen target pulse");
        }
    }

    return ESP_OK;
}


void pen_control_get_default_config(
    pen_control_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    *config =
        (pen_control_config_t){0};

    /*
     * Intentionally invalid until caller explicitly selects a GPIO.
     */
    config->gpio_num =
        -1;

    config->speed_mode =
        LEDC_LOW_SPEED_MODE;

    config->timer_num =
        LEDC_TIMER_0;

    config->channel =
        LEDC_CHANNEL_0;

    config->duty_resolution =
        PEN_CONTROL_DEFAULT_DUTY_RESOLUTION;

    config->clock_source =
        LEDC_AUTO_CLK;

    config->pwm_frequency_hz =
        PEN_CONTROL_DEFAULT_PWM_FREQUENCY_HZ;

    config->safe_min_pulse_us =
        PEN_CONTROL_DEFAULT_SAFE_MIN_PULSE_US;

    config->safe_max_pulse_us =
        PEN_CONTROL_DEFAULT_SAFE_MAX_PULSE_US;

    config->startup_pulse_us =
        PEN_CONTROL_DEFAULT_STARTUP_PULSE_US;

    config->positions_calibrated =
        false;

    config->up_pulse_us =
        PEN_CONTROL_DEFAULT_UP_PULSE_US;

    config->down_pulse_us =
        PEN_CONTROL_DEFAULT_DOWN_PULSE_US;

    config->raise_time_ms =
        PEN_CONTROL_DEFAULT_RAISE_TIME_MS;

    config->lower_time_ms =
        PEN_CONTROL_DEFAULT_LOWER_TIME_MS;

    config->settle_time_ms =
        PEN_CONTROL_DEFAULT_SETTLE_TIME_MS;
}


esp_err_t pen_control_init(
    pen_control_t *pen,
    const pen_control_config_t *config)
{
    if ((pen == NULL) ||
        (config == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    *pen =
        (pen_control_t)
        PEN_CONTROL_INITIALIZER;

    if (!pen_control_config_valid(
            config))
    {
        pen->state =
            PEN_CONTROL_ERROR;

        pen->error =
            PEN_CONTROL_ERROR_INVALID_CONFIG;

        return ESP_ERR_INVALID_ARG;
    }

    pen->config =
        *config;

    ledc_timer_config_t timer_config =
    {
        .speed_mode =
            config->speed_mode,

        .duty_resolution =
            config->duty_resolution,

        .timer_num =
            config->timer_num,

        .freq_hz =
            config->pwm_frequency_hz,

        .clk_cfg =
            config->clock_source
    };

    esp_err_t ret =
        ledc_timer_config(
            &timer_config);

    if (ret != ESP_OK)
    {
        pen->state =
            PEN_CONTROL_ERROR;

        pen->error =
            PEN_CONTROL_ERROR_LEDC_TIMER;

        ESP_LOGE(
            PEN_CONTROL_TAG,
            "LEDC timer configuration failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    /*
     * Configure channel initially with duty=0, then use the thread-safe
     * set_duty_and_update API for the actual servo pulse.
     */
    ledc_channel_config_t channel_config =
    {
        .gpio_num =
            config->gpio_num,

        .speed_mode =
            config->speed_mode,

        .channel =
            config->channel,

        .intr_type =
            LEDC_INTR_DISABLE,

        .timer_sel =
            config->timer_num,

        .duty =
            0U,

        .hpoint =
            0
    };

    ret =
        ledc_channel_config(
            &channel_config);

    if (ret != ESP_OK)
    {
        pen->state =
            PEN_CONTROL_ERROR;

        pen->error =
            PEN_CONTROL_ERROR_LEDC_CHANNEL;

        ESP_LOGE(
            PEN_CONTROL_TAG,
            "LEDC channel configuration failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    pen->initialized =
        true;

    pen->error =
        PEN_CONTROL_ERROR_NONE;

    const int64_t now =
        esp_timer_get_time();

    if (config->positions_calibrated)
    {
        pen->start_pulse_us =
            config->up_pulse_us;

        pen->target_pulse_us =
            config->up_pulse_us;

        ret =
            pen_control_write_pulse(
                pen,
                config->up_pulse_us);

        if (ret != ESP_OK)
        {
            return pen_control_latch_error(
                pen,
                PEN_CONTROL_ERROR_PWM_WRITE,
                ret,
                "Initial UP PWM command failed");
        }

        /*
         * Initial physical servo position is unknown. Command UP immediately
         * and conservatively wait one full raise + settle window.
         */
        pen->motion_start_us =
            now;

        pen->motion_end_us =
            now +
            ((int64_t)config->raise_time_ms *
             1000LL);

        pen->settle_end_us =
            pen->motion_end_us +
            ((int64_t)config->settle_time_ms *
             1000LL);

        pen->state =
            PEN_CONTROL_MOVING_UP;

        ESP_LOGI(
            PEN_CONTROL_TAG,
            "Initialized calibrated pen: GPIO=%d UP=%u us DOWN=%u us",
            config->gpio_num,
            (unsigned)config->up_pulse_us,
            (unsigned)config->down_pulse_us);
    }
    else
    {
        pen->start_pulse_us =
            config->startup_pulse_us;

        pen->target_pulse_us =
            config->startup_pulse_us;

        ret =
            pen_control_write_pulse(
                pen,
                config->startup_pulse_us);

        if (ret != ESP_OK)
        {
            return pen_control_latch_error(
                pen,
                PEN_CONTROL_ERROR_PWM_WRITE,
                ret,
                "Initial calibration PWM command failed");
        }

        pen->motion_start_us =
            now;

        pen->motion_end_us =
            now;

        pen->settle_end_us =
            now;

        pen->state =
            PEN_CONTROL_CALIBRATION;

        ESP_LOGI(
            PEN_CONTROL_TAG,
            "Initialized uncalibrated pen at %u us. "
            "Use calibration pulse API before UP/DOWN.",
            (unsigned)config->startup_pulse_us);
    }

    return ESP_OK;
}


esp_err_t pen_control_request_up(
    pen_control_t *pen)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        if (pen != NULL)
        {
            pen->error =
                PEN_CONTROL_ERROR_NOT_INITIALIZED;
        }

        return ESP_ERR_INVALID_STATE;
    }

    return pen_control_start_motion(
        pen,
        true);
}


esp_err_t pen_control_request_down(
    pen_control_t *pen)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        if (pen != NULL)
        {
            pen->error =
                PEN_CONTROL_ERROR_NOT_INITIALIZED;
        }

        return ESP_ERR_INVALID_STATE;
    }

    return pen_control_start_motion(
        pen,
        false);
}


esp_err_t pen_control_update(
    pen_control_t *pen)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if ((pen->state != PEN_CONTROL_MOVING_UP) &&
        (pen->state != PEN_CONTROL_MOVING_DOWN))
    {
        return
            (pen->state == PEN_CONTROL_ERROR)
                ? ESP_ERR_INVALID_STATE
                : ESP_OK;
    }

    const int64_t now =
        esp_timer_get_time();

    if (now < pen->motion_end_us)
    {
        const int64_t duration_us =
            pen->motion_end_us -
            pen->motion_start_us;

        const int64_t elapsed_us =
            now -
            pen->motion_start_us;

        if (duration_us > 0)
        {
            const int32_t delta =
                (int32_t)pen->target_pulse_us -
                (int32_t)pen->start_pulse_us;

            int64_t scaled =
                (int64_t)delta *
                elapsed_us;

            /*
             * Round toward nearest pulse microsecond.
             */
            if (scaled >= 0)
            {
                scaled +=
                    duration_us / 2;
            }
            else
            {
                scaled -=
                    duration_us / 2;
            }

            const int32_t interpolated =
                (int32_t)pen->start_pulse_us +
                (int32_t)(scaled /
                          duration_us);

            uint16_t pulse =
                (uint16_t)interpolated;

            if (pulse <
                pen->config.safe_min_pulse_us)
            {
                pulse =
                    pen->config.safe_min_pulse_us;
            }

            if (pulse >
                pen->config.safe_max_pulse_us)
            {
                pulse =
                    pen->config.safe_max_pulse_us;
            }

            if (pulse !=
                pen->current_pulse_us)
            {
                const esp_err_t ret =
                    pen_control_write_pulse(
                        pen,
                        pulse);

                if (ret != ESP_OK)
                {
                    return pen_control_latch_error(
                        pen,
                        PEN_CONTROL_ERROR_PWM_WRITE,
                        ret,
                        "PWM ramp update failed");
                }
            }
        }

        return ESP_OK;
    }

    if (pen->current_pulse_us !=
        pen->target_pulse_us)
    {
        const esp_err_t ret =
            pen_control_write_pulse(
                pen,
                pen->target_pulse_us);

        if (ret != ESP_OK)
        {
            return pen_control_latch_error(
                pen,
                PEN_CONTROL_ERROR_PWM_WRITE,
                ret,
                "Final target PWM update failed");
        }
    }

    if (now < pen->settle_end_us)
    {
        return ESP_OK;
    }

    if (pen->state ==
        PEN_CONTROL_MOVING_UP)
    {
        pen->state =
            PEN_CONTROL_UP;
    }
    else
    {
        pen->state =
            PEN_CONTROL_DOWN;
    }

    return ESP_OK;
}


esp_err_t pen_control_set_calibration_pulse_us(
    pen_control_t *pen,
    uint16_t pulse_us)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if ((pulse_us <
         pen->config.safe_min_pulse_us) ||
        (pulse_us >
         pen->config.safe_max_pulse_us))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t ret =
        pen_control_write_pulse(
            pen,
            pulse_us);

    if (ret != ESP_OK)
    {
        return pen_control_latch_error(
            pen,
            PEN_CONTROL_ERROR_PWM_WRITE,
            ret,
            "Calibration PWM update failed");
    }

    const int64_t now =
        esp_timer_get_time();

    pen->start_pulse_us =
        pulse_us;

    pen->target_pulse_us =
        pulse_us;

    pen->motion_start_us =
        now;

    pen->motion_end_us =
        now;

    pen->settle_end_us =
        now;

    pen->state =
        PEN_CONTROL_CALIBRATION;

    pen->error =
        PEN_CONTROL_ERROR_NONE;

    return ESP_OK;
}


esp_err_t pen_control_set_calibrated_positions(
    pen_control_t *pen,
    uint16_t up_pulse_us,
    uint16_t down_pulse_us)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if ((up_pulse_us <
         pen->config.safe_min_pulse_us) ||
        (up_pulse_us >
         pen->config.safe_max_pulse_us) ||
        (down_pulse_us <
         pen->config.safe_min_pulse_us) ||
        (down_pulse_us >
         pen->config.safe_max_pulse_us) ||
        (up_pulse_us ==
         down_pulse_us))
    {
        return ESP_ERR_INVALID_ARG;
    }

    pen->config.up_pulse_us =
        up_pulse_us;

    pen->config.down_pulse_us =
        down_pulse_us;

    pen->config.positions_calibrated =
        true;

    pen->error =
        PEN_CONTROL_ERROR_NONE;

    ESP_LOGI(
        PEN_CONTROL_TAG,
        "Runtime calibration set: UP=%u us DOWN=%u us",
        (unsigned)up_pulse_us,
        (unsigned)down_pulse_us);

    return ESP_OK;
}


esp_err_t pen_control_emergency_up(
    pen_control_t *pen)
{
    if ((pen == NULL) ||
        !pen->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!pen->config.positions_calibrated)
    {
        pen->error =
            PEN_CONTROL_ERROR_NOT_CALIBRATED;

        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t ret =
        pen_control_write_pulse(
            pen,
            pen->config.up_pulse_us);

    if (ret != ESP_OK)
    {
        return pen_control_latch_error(
            pen,
            PEN_CONTROL_ERROR_PWM_WRITE,
            ret,
            "Emergency UP PWM command failed");
    }

    const int64_t now =
        esp_timer_get_time();

    pen->start_pulse_us =
        pen->config.up_pulse_us;

    pen->target_pulse_us =
        pen->config.up_pulse_us;

    pen->motion_start_us =
        now;

    pen->motion_end_us =
        now +
        ((int64_t)pen->config.raise_time_ms *
         1000LL);

    pen->settle_end_us =
        pen->motion_end_us +
        ((int64_t)pen->config.settle_time_ms *
         1000LL);

    pen->state =
        PEN_CONTROL_MOVING_UP;

    /*
     * Explicit safety recovery point.
     */
    pen->error =
        PEN_CONTROL_ERROR_NONE;

    ESP_LOGW(
        PEN_CONTROL_TAG,
        "Emergency UP commanded");

    return ESP_OK;
}


bool pen_control_is_ready(
    const pen_control_t *pen)
{
    return
        (pen != NULL) &&
        pen->initialized &&
        (pen->state != PEN_CONTROL_ERROR);
}


bool pen_control_is_busy(
    const pen_control_t *pen)
{
    return
        (pen != NULL) &&
        ((pen->state == PEN_CONTROL_MOVING_UP) ||
         (pen->state == PEN_CONTROL_MOVING_DOWN));
}


bool pen_control_is_up(
    const pen_control_t *pen)
{
    return
        (pen != NULL) &&
        (pen->state == PEN_CONTROL_UP);
}


bool pen_control_is_down(
    const pen_control_t *pen)
{
    return
        (pen != NULL) &&
        (pen->state == PEN_CONTROL_DOWN);
}


bool pen_control_is_calibrated(
    const pen_control_t *pen)
{
    return
        (pen != NULL) &&
        pen->initialized &&
        pen->config.positions_calibrated;
}


pen_control_state_t pen_control_get_state(
    const pen_control_t *pen)
{
    return
        (pen != NULL)
            ? pen->state
            : PEN_CONTROL_ERROR;
}


pen_control_error_t pen_control_get_error(
    const pen_control_t *pen)
{
    return
        (pen != NULL)
            ? pen->error
            : PEN_CONTROL_ERROR_INVALID_ARGUMENT;
}


void pen_control_get_status(
    const pen_control_t *pen,
    pen_control_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    *status =
        (pen_control_status_t){0};

    if (pen == NULL)
    {
        status->state =
            PEN_CONTROL_ERROR;

        status->error =
            PEN_CONTROL_ERROR_INVALID_ARGUMENT;

        return;
    }

    status->initialized =
        pen->initialized;

    status->positions_calibrated =
        pen->config.positions_calibrated;

    status->state =
        pen->state;

    status->error =
        pen->error;

    status->busy =
        pen_control_is_busy(
            pen);

    status->current_pulse_us =
        pen->current_pulse_us;

    status->target_pulse_us =
        pen->target_pulse_us;

    status->up_pulse_us =
        pen->config.up_pulse_us;

    status->down_pulse_us =
        pen->config.down_pulse_us;

    status->gpio_num =
        pen->config.gpio_num;

    status->timer_num =
        pen->config.timer_num;

    status->channel =
        pen->config.channel;

    status->pwm_frequency_hz =
        pen->config.pwm_frequency_hz;

    if (!status->busy)
    {
        status->motion_progress =
            ((pen->state == PEN_CONTROL_UP) ||
             (pen->state == PEN_CONTROL_DOWN))
                ? 1.0f
                : 0.0f;

        return;
    }

    const int64_t now =
        esp_timer_get_time();

    if (now < pen->motion_end_us)
    {
        const int64_t total =
            pen->motion_end_us -
            pen->motion_start_us;

        const int64_t elapsed =
            now -
            pen->motion_start_us;

        if (total > 0)
        {
            float progress =
                (float)elapsed /
                (float)total;

            if (progress < 0.0f)
            {
                progress =
                    0.0f;
            }

            if (progress > 1.0f)
            {
                progress =
                    1.0f;
            }

            status->motion_progress =
                progress;
        }

        const int64_t remaining_us =
            pen->motion_end_us -
            now;

        status->motion_remaining_ms =
            (uint32_t)(
                (remaining_us + 999LL) /
                1000LL);
    }
    else
    {
        status->motion_progress =
            1.0f;

        if (now <
            pen->settle_end_us)
        {
            status->settling =
                true;

            const int64_t remaining_us =
                pen->settle_end_us -
                now;

            status->settle_remaining_ms =
                (uint32_t)(
                    (remaining_us + 999LL) /
                    1000LL);
        }
    }
}


const char *pen_control_state_name(
    pen_control_state_t state)
{
    switch (state)
    {
        case PEN_CONTROL_UNINITIALIZED:
            return "UNINITIALIZED";

        case PEN_CONTROL_CALIBRATION:
            return "CALIBRATION";

        case PEN_CONTROL_MOVING_UP:
            return "MOVING_UP";

        case PEN_CONTROL_UP:
            return "UP";

        case PEN_CONTROL_MOVING_DOWN:
            return "MOVING_DOWN";

        case PEN_CONTROL_DOWN:
            return "DOWN";

        case PEN_CONTROL_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}


const char *pen_control_error_name(
    pen_control_error_t error)
{
    switch (error)
    {
        case PEN_CONTROL_ERROR_NONE:
            return "NONE";

        case PEN_CONTROL_ERROR_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";

        case PEN_CONTROL_ERROR_INVALID_CONFIG:
            return "INVALID_CONFIG";

        case PEN_CONTROL_ERROR_NOT_INITIALIZED:
            return "NOT_INITIALIZED";

        case PEN_CONTROL_ERROR_NOT_CALIBRATED:
            return "NOT_CALIBRATED";

        case PEN_CONTROL_ERROR_LEDC_TIMER:
            return "LEDC_TIMER";

        case PEN_CONTROL_ERROR_LEDC_CHANNEL:
            return "LEDC_CHANNEL";

        case PEN_CONTROL_ERROR_PWM_WRITE:
            return "PWM_WRITE";

        case PEN_CONTROL_ERROR_INVALID_STATE:
            return "INVALID_STATE";

        default:
            return "UNKNOWN";
    }
}


#ifdef __cplusplus
}
#endif

#endif /* PEN_CONTROL_IMPLEMENTATION */
#endif /* PEN_CONTROL_H */
