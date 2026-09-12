#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

int64_t g_fake_time_us=0;
uint32_t g_last_duty=0;
uint32_t g_write_count=0;
int g_ledc_write_result=0;

#define PEN_CONTROL_IMPLEMENTATION
#include "pen_control.h"

static pen_control_config_t make_config(bool calibrated)
{
    pen_control_config_t c;
    pen_control_get_default_config(&c);
    c.gpio_num=18;
    c.timer_num=LEDC_TIMER_3;
    c.channel=LEDC_CHANNEL_7;
    c.positions_calibrated=calibrated;
    c.up_pulse_us=1200;
    c.down_pulse_us=1800;
    c.raise_time_ms=200;
    c.lower_time_ms=300;
    c.settle_time_ms=50;
    return c;
}

static void test_default_is_safe(void)
{
    pen_control_config_t c;
    pen_control_get_default_config(&c);
    assert(c.gpio_num==-1);
    assert(!c.positions_calibrated);
    assert(c.startup_pulse_us==1500);
}

static void test_uncalibrated_mode(void)
{
    g_fake_time_us=0;g_write_count=0;
    pen_control_t p=PEN_CONTROL_INITIALIZER;
    pen_control_config_t c=make_config(false);

    assert(pen_control_init(&p,&c)==ESP_OK);
    assert(p.state==PEN_CONTROL_CALIBRATION);
    assert(p.current_pulse_us==1500);
    assert(g_last_duty==1229U);

    assert(pen_control_request_up(&p)==ESP_ERR_INVALID_STATE);
    assert(p.state==PEN_CONTROL_ERROR);

    /* Calibration direct command is an explicit recovery path. */
    assert(pen_control_set_calibration_pulse_us(&p,1450)==ESP_OK);
    assert(p.state==PEN_CONTROL_CALIBRATION);
    assert(p.current_pulse_us==1450);

    assert(pen_control_set_calibration_pulse_us(&p,999)==ESP_ERR_INVALID_ARG);

    assert(pen_control_set_calibrated_positions(&p,1300,1750)==ESP_OK);
    assert(pen_control_is_calibrated(&p));
}

static void run_until(pen_control_t *p, int64_t target_us, int64_t step_us)
{
    while(g_fake_time_us<target_us){
        g_fake_time_us+=step_us;
        assert(pen_control_update(p)==ESP_OK);
    }
}

static void test_nonblocking_down_up(void)
{
    g_fake_time_us=0;g_write_count=0;
    pen_control_t p=PEN_CONTROL_INITIALIZER;
    pen_control_config_t c=make_config(true);

    assert(pen_control_init(&p,&c)==ESP_OK);
    assert(p.state==PEN_CONTROL_MOVING_UP);

    run_until(&p,250000,10000);
    assert(p.state==PEN_CONTROL_UP);
    assert(p.current_pulse_us==1200);

    assert(pen_control_request_down(&p)==ESP_OK);
    assert(p.state==PEN_CONTROL_MOVING_DOWN);
    assert(pen_control_is_busy(&p));

    /* Half of 300 ms ramp: expected ~1500 us. */
    g_fake_time_us+=150000;
    assert(pen_control_update(&p)==ESP_OK);
    assert(p.current_pulse_us>=1499 && p.current_pulse_us<=1501);

    /* Repeating same direction is idempotent. */
    const int64_t old_end=p.motion_end_us;
    assert(pen_control_request_down(&p)==ESP_OK);
    assert(p.motion_end_us==old_end);

    run_until(&p,650000,10000);
    assert(p.state==PEN_CONTROL_DOWN);
    assert(p.current_pulse_us==1800);

    /* UP direction may be numerically smaller: direction-agnostic. */
    assert(pen_control_request_up(&p)==ESP_OK);
    run_until(&p,900000,10000);
    assert(p.state==PEN_CONTROL_UP);
    assert(p.current_pulse_us==1200);
}

static void test_mid_motion_reversal(void)
{
    g_fake_time_us=0;
    pen_control_t p=PEN_CONTROL_INITIALIZER;
    pen_control_config_t c=make_config(true);
    assert(pen_control_init(&p,&c)==ESP_OK);
    run_until(&p,250000,10000);
    assert(p.state==PEN_CONTROL_UP);

    assert(pen_control_request_down(&p)==ESP_OK);
    g_fake_time_us+=150000;
    assert(pen_control_update(&p)==ESP_OK);
    assert(p.current_pulse_us>=1499 && p.current_pulse_us<=1501);

    /* Reverse to UP from halfway; full raise=200 ms, so ~100 ms ramp. */
    assert(pen_control_request_up(&p)==ESP_OK);
    assert(p.state==PEN_CONTROL_MOVING_UP);
    assert((p.motion_end_us-g_fake_time_us)>=99000);
    assert((p.motion_end_us-g_fake_time_us)<=101000);

    run_until(&p,g_fake_time_us+170000,10000);
    assert(p.state==PEN_CONTROL_UP);
}

static void test_emergency_up(void)
{
    g_fake_time_us=0;
    pen_control_t p=PEN_CONTROL_INITIALIZER;
    pen_control_config_t c=make_config(true);
    assert(pen_control_init(&p,&c)==ESP_OK);
    run_until(&p,250000,10000);

    assert(pen_control_request_down(&p)==ESP_OK);
    g_fake_time_us+=50000;
    assert(pen_control_update(&p)==ESP_OK);

    assert(pen_control_emergency_up(&p)==ESP_OK);
    assert(p.current_pulse_us==1200);
    assert(p.state==PEN_CONTROL_MOVING_UP);

    run_until(&p,g_fake_time_us+260000,10000);
    assert(p.state==PEN_CONTROL_UP);
}

static void test_status(void)
{
    g_fake_time_us=0;
    pen_control_t p=PEN_CONTROL_INITIALIZER;
    pen_control_config_t c=make_config(true);
    assert(pen_control_init(&p,&c)==ESP_OK);

    pen_control_status_t s;
    pen_control_get_status(&p,&s);
    assert(s.initialized);
    assert(s.positions_calibrated);
    assert(s.busy);
    assert(s.state==PEN_CONTROL_MOVING_UP);
}

int main(void)
{
    test_default_is_safe();
    test_uncalibrated_mode();
    test_nonblocking_down_up();
    test_mid_motion_reversal();
    test_emergency_up();
    test_status();

    puts("pen_control host regression: PASS");
    return 0;
}
