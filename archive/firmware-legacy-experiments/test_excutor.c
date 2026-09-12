/*
 * esp32_executor_f2_1_test.c
 *
 * Real ESP32 / ESP-IDF test program for the F2.1 dual-source trajectory
 * executor.
 *
 * PURPOSE
 * -------
 * This is NOT a host/GCC test.
 * This file is intended to be compiled into the ESP32 firmware and run from
 * app_main().
 *
 * It does NOT:
 *   - mount SPIFFS
 *   - read .traj files
 *   - start trajectory_runner
 *   - start trajectory_tracker
 *   - command motors
 *   - command pen hardware
 *
 * It directly constructs LINE/CIRCLE Motion segments and validates the new
 * EXTERNAL executor API:
 *
 *   init_external()
 *   start() -> HOLDING
 *   begin_motion()
 *   advance_motion()
 *   finish()
 *
 * Tests:
 *   1. External start -> HOLDING
 *   2. Motion -> Event barrier -> HOLDING at zero speed
 *   3. Event -> Motion starts from zero at held XY
 *   4. Tangent Motion -> Motion keeps non-zero junction speed
 *   5. 90-degree Motion -> Motion forces zero junction speed
 *   6. LINE -> CIRCLE -> LINE Golden Motion window
 *   7. Explicit finish() is the only external trajectory_end
 *
 * IMPORTANT SINGLE-HEADER NOTE
 * ----------------------------
 * The project must define implementation macros in exactly ONE translation
 * unit.
 *
 * If your project ALREADY has something like:
 *
 *     #define TRAJECTORY_DECODER_IMPLEMENTATION
 *     #include "trajectory_decoder.h"
 *
 *     #define TRAJECTORY_EXECUTOR_IMPLEMENTATION
 *     #include "trajectory_executor.h"
 *
 * somewhere else, DO NOT define them here.
 *
 * If this file is the ONLY implementation translation unit, uncomment the
 * block below.
 */

/*
#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#include "trajectory_executor.h"
*/

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#include "trajectory_executor.h"


static const char *TAG = "exec_f2_1_test";

#define TEST_DT_S          0.010f
#define TEST_MAX_UPDATES   30000U

static unsigned s_pass_count = 0U;
static unsigned s_fail_count = 0U;


static bool test_closef(
    float actual,
    float expected,
    float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}


static void test_check(
    bool condition,
    const char *name)
{
    if (condition)
    {
        s_pass_count++;

        ESP_LOGI(
            TAG,
            "PASS: %s",
            name);
    }
    else
    {
        s_fail_count++;

        ESP_LOGE(
            TAG,
            "FAIL: %s",
            name);
    }
}


static trajectory_file_header_t make_v2_header(
    float start_x_mm,
    float start_y_mm,
    float start_yaw_deg)
{
    trajectory_file_header_t header = {0};

    header.version =
        TRAJECTORY_FILE_VERSION_V2;

    header.header_size =
        TRAJECTORY_FILE_HEADER_SIZE_V2;

    /*
     * The Executor does not use record_count for EXTERNAL Motion execution,
     * but keeping a realistic value makes the test Header representative.
     */
    header.record_count =
        0U;

    header.segment_count =
        0U;

    header.record_size =
        TRAJECTORY_FILE_RECORD_SIZE_V2;

    header.flags =
        0U;

    header.start_x_mm =
        start_x_mm;

    header.start_y_mm =
        start_y_mm;

    header.start_yaw_deg =
        start_yaw_deg;

    return header;
}


static trajectory_segment_t make_line(
    float end_x_mm,
    float end_y_mm,
    float speed_mm_s)
{
    trajectory_segment_t segment = {0};

    segment.type =
        TRAJECTORY_SEGMENT_LINE;

    segment.flags =
        0U;

    segment.speed_mm_s =
        speed_mm_s;

    segment.acceleration_mm_s2 =
        0.0f;

    segment.geometry.line.end_x_mm =
        end_x_mm;

    segment.geometry.line.end_y_mm =
        end_y_mm;

    return segment;
}


static trajectory_segment_t make_circle(
    float center_x_mm,
    float center_y_mm,
    float radius_mm,
    float start_angle_deg,
    float sweep_deg,
    float speed_mm_s)
{
    trajectory_segment_t segment = {0};

    segment.type =
        TRAJECTORY_SEGMENT_CIRCLE;

    segment.flags =
        0U;

    segment.speed_mm_s =
        speed_mm_s;

    segment.acceleration_mm_s2 =
        0.0f;

    segment.geometry.circle.center_x_mm =
        center_x_mm;

    segment.geometry.circle.center_y_mm =
        center_y_mm;

    segment.geometry.circle.radius_mm =
        radius_mm;

    segment.geometry.circle.start_angle_deg =
        start_angle_deg;

    segment.geometry.circle.sweep_deg =
        sweep_deg;

    return segment;
}


static bool update_until_boundary(
    trajectory_executor_t *executor,
    trajectory_reference_t *reference)
{
    for (uint32_t i = 0U;
         i < TEST_MAX_UPDATES;
         ++i)
    {
        const esp_err_t ret =
            trajectory_executor_update(
                executor,
                TEST_DT_S,
                reference);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "executor_update failed at step %lu: %s",
                (unsigned long)i,
                esp_err_to_name(ret));

            return false;
        }

        if (trajectory_executor_needs_advance(executor) ||
            trajectory_executor_is_holding(executor) ||
            trajectory_executor_is_finished(executor))
        {
            return true;
        }
    }

    ESP_LOGE(
        TAG,
        "Executor boundary timeout after %lu updates",
        (unsigned long)TEST_MAX_UPDATES);

    return false;
}


static bool start_external_executor(
    trajectory_executor_t *executor,
    const trajectory_file_header_t *header)
{
    esp_err_t ret =
        trajectory_executor_init_external(
            executor,
            header);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "init_external failed: %s",
            esp_err_to_name(ret));

        return false;
    }

    ret =
        trajectory_executor_start(
            executor);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "executor_start failed: %s",
            esp_err_to_name(ret));

        return false;
    }

    return true;
}


static void test_external_start_holding(void)
{
    ESP_LOGI(
        TAG,
        "=== TEST 1: external start -> HOLDING ===");

    const trajectory_file_header_t header =
        make_v2_header(
            12.0f,
            34.0f,
            0.0f);

    trajectory_executor_t executor =
        TRAJECTORY_EXECUTOR_INITIALIZER;

    const bool started =
        start_external_executor(
            &executor,
            &header);

    test_check(
        started,
        "external init/start succeeds");

    if (!started)
    {
        return;
    }

    test_check(
        trajectory_executor_get_source_mode(&executor) ==
            TRAJECTORY_EXECUTOR_SOURCE_EXTERNAL,
        "source mode is EXTERNAL");

    test_check(
        trajectory_executor_is_holding(&executor),
        "start enters HOLDING");

    trajectory_reference_t reference = {0};

    const esp_err_t ret =
        trajectory_executor_update(
            &executor,
            TEST_DT_S,
            &reference);

    test_check(
        ret == ESP_OK,
        "HOLDING update succeeds");

    if (ret != ESP_OK)
    {
        return;
    }

    test_check(
        reference.valid,
        "HOLDING reference is valid");

    test_check(
        !reference.trajectory_end,
        "HOLDING is not trajectory_end");

    test_check(
        test_closef(
            reference.x_mm,
            12.0f,
            1.0e-4f) &&
        test_closef(
            reference.y_mm,
            34.0f,
            1.0e-4f),
        "HOLDING reference stays at Header start XY");

    test_check(
        test_closef(
            reference.speed_mm_s,
            0.0f,
            1.0e-6f),
        "HOLDING speed is zero");
}


static void test_motion_event_motion_barrier(void)
{
    ESP_LOGI(
        TAG,
        "=== TEST 2: Motion -> Event barrier -> Motion ===");

    const trajectory_file_header_t header =
        make_v2_header(
            0.0f,
            0.0f,
            0.0f);

    trajectory_executor_t executor =
        TRAJECTORY_EXECUTOR_INITIALIZER;

    if (!start_external_executor(
            &executor,
            &header))
    {
        test_check(
            false,
            "barrier test executor starts");

        return;
    }

    /*
     * Simulated record stream:
     *
     *   record 1: LINE (0,0) -> (100,0)
     *   record 2: PEN_UP
     *   record 3: LINE (100,0) -> (200,0)
     *
     * Because record 2 is Event, record 1 receives next_motion=NULL.
     */
    const trajectory_segment_t line_a =
        make_line(
            100.0f,
            0.0f,
            300.0f);

    esp_err_t ret =
        trajectory_executor_begin_motion(
            &executor,
            &line_a,
            1U,
            NULL,
            0U);

    test_check(
        ret == ESP_OK,
        "begin first Motion before Event succeeds");

    if (ret != ESP_OK)
    {
        return;
    }

    trajectory_reference_t reference = {0};

    bool reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached,
        "first Motion reaches Event barrier");

    if (!reached)
    {
        return;
    }

    test_check(
        trajectory_executor_is_holding(&executor),
        "Motion->Event enters HOLDING");

    test_check(
        !trajectory_executor_needs_advance(&executor),
        "Event barrier does not request Motion advance");

    test_check(
        reference.segment_end,
        "Event barrier reference marks segment_end");

    test_check(
        !reference.trajectory_end,
        "Event barrier is not trajectory_end");

    test_check(
        test_closef(
            reference.x_mm,
            100.0f,
            1.0e-3f) &&
        test_closef(
            reference.y_mm,
            0.0f,
            1.0e-3f),
        "Event barrier holds Motion endpoint");

    test_check(
        test_closef(
            reference.speed_mm_s,
            0.0f,
            1.0e-3f),
        "Motion reaches Event with zero speed");

    ESP_LOGI(
        TAG,
        "Simulated Event: PEN_UP (no Executor call required)");

    /*
     * After the Event completes, the next Motion starts from the held point.
     */
    const trajectory_segment_t line_b =
        make_line(
            200.0f,
            0.0f,
            500.0f);

    ret =
        trajectory_executor_begin_motion(
            &executor,
            &line_b,
            3U,
            NULL,
            0U);

    test_check(
        ret == ESP_OK,
        "Event->Motion begin succeeds");

    if (ret != ESP_OK)
    {
        return;
    }

    trajectory_reference_t start_reference = {0};

    ret =
        trajectory_executor_get_reference(
            &executor,
            &start_reference);

    test_check(
        ret == ESP_OK,
        "Event->Motion start reference available");

    if (ret == ESP_OK)
    {
        test_check(
            start_reference.segment_index == 3U,
            "Event->Motion preserves source record index");

        test_check(
            test_closef(
                start_reference.x_mm,
                100.0f,
                1.0e-3f),
            "Event->Motion starts at held XY");

        test_check(
            test_closef(
                start_reference.speed_mm_s,
                0.0f,
                1.0e-6f),
            "Event->Motion starts from zero speed");
    }

    reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached &&
        trajectory_executor_is_holding(&executor),
        "second Motion reaches terminal HOLDING");

    if (!reached)
    {
        return;
    }

    test_check(
        !reference.trajectory_end,
        "terminal Motion still not trajectory_end before finish()");

    ret =
        trajectory_executor_finish(
            &executor);

    test_check(
        ret == ESP_OK,
        "explicit finish() succeeds");

    if (ret != ESP_OK)
    {
        return;
    }

    ret =
        trajectory_executor_update(
            &executor,
            TEST_DT_S,
            &reference);

    test_check(
        ret == ESP_OK &&
        reference.trajectory_end,
        "finish() produces trajectory_end=true");
}


static void test_tangent_motion_chain(void)
{
    ESP_LOGI(
        TAG,
        "=== TEST 3: tangent Motion -> Motion ===");

    const trajectory_file_header_t header =
        make_v2_header(
            0.0f,
            0.0f,
            0.0f);

    trajectory_executor_t executor =
        TRAJECTORY_EXECUTOR_INITIALIZER;

    if (!start_external_executor(
            &executor,
            &header))
    {
        test_check(
            false,
            "tangent chain executor starts");

        return;
    }

    const trajectory_segment_t line_a =
        make_line(
            100.0f,
            0.0f,
            300.0f);

    const trajectory_segment_t line_b =
        make_line(
            200.0f,
            0.0f,
            300.0f);

    esp_err_t ret =
        trajectory_executor_begin_motion(
            &executor,
            &line_a,
            1U,
            &line_b,
            5U);

    test_check(
        ret == ESP_OK,
        "tangent chain begin succeeds");

    if (ret != ESP_OK)
    {
        return;
    }

    trajectory_reference_t reference = {0};

    const bool reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached &&
        trajectory_executor_needs_advance(&executor),
        "tangent junction requests advance");

    if (!reached)
    {
        return;
    }

    test_check(
        reference.segment_end,
        "tangent junction marks segment_end");

    test_check(
        !reference.trajectory_end,
        "tangent junction is not trajectory_end");

    test_check(
        reference.speed_mm_s > 1.0f,
        "tangent junction keeps non-zero speed");

    ret =
        trajectory_executor_advance_motion(
            &executor,
            NULL,
            0U);

    test_check(
        ret == ESP_OK,
        "advance_motion promotes cached next Motion");

    if (ret != ESP_OK)
    {
        return;
    }

    ret =
        trajectory_executor_update(
            &executor,
            TEST_DT_S,
            &reference);

    test_check(
        ret == ESP_OK,
        "promoted Motion update succeeds");

    if (ret == ESP_OK)
    {
        test_check(
            reference.segment_index == 5U,
            "non-contiguous source record index is preserved");
    }
}


static void test_sharp_motion_chain(void)
{
    ESP_LOGI(
        TAG,
        "=== TEST 4: 90-degree Motion -> Motion ===");

    const trajectory_file_header_t header =
        make_v2_header(
            0.0f,
            0.0f,
            0.0f);

    trajectory_executor_t executor =
        TRAJECTORY_EXECUTOR_INITIALIZER;

    if (!start_external_executor(
            &executor,
            &header))
    {
        test_check(
            false,
            "sharp chain executor starts");

        return;
    }

    const trajectory_segment_t line_a =
        make_line(
            100.0f,
            0.0f,
            300.0f);

    const trajectory_segment_t line_b =
        make_line(
            100.0f,
            100.0f,
            300.0f);

    const esp_err_t ret =
        trajectory_executor_begin_motion(
            &executor,
            &line_a,
            0U,
            &line_b,
            1U);

    test_check(
        ret == ESP_OK,
        "sharp chain begin succeeds");

    if (ret != ESP_OK)
    {
        return;
    }

    trajectory_reference_t reference = {0};

    const bool reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached &&
        trajectory_executor_needs_advance(&executor),
        "sharp Motion still has next Motion");

    if (!reached)
    {
        return;
    }

    test_check(
        test_closef(
            reference.speed_mm_s,
            0.0f,
            1.0e-3f),
        "90-degree junction reaches zero speed");

    test_check(
        !reference.trajectory_end,
        "sharp junction is not trajectory_end");
}


static void test_golden_line_circle_line(void)
{
    ESP_LOGI(
        TAG,
        "=== TEST 5: Golden LINE -> CIRCLE -> LINE ===");

    const trajectory_file_header_t header =
        make_v2_header(
            0.0f,
            0.0f,
            0.0f);

    trajectory_executor_t executor =
        TRAJECTORY_EXECUTOR_INITIALIZER;

    if (!start_external_executor(
            &executor,
            &header))
    {
        test_check(
            false,
            "Golden executor starts");

        return;
    }

    const trajectory_segment_t line0 =
        make_line(
            500.0f,
            0.0f,
            300.0f);

    const trajectory_segment_t arc =
        make_circle(
            500.0f,
            250.0f,
            250.0f,
            -90.0f,
            180.0f,
            220.0f);

    const trajectory_segment_t line1 =
        make_line(
            0.0f,
            500.0f,
            300.0f);

    esp_err_t ret =
        trajectory_executor_begin_motion(
            &executor,
            &line0,
            0U,
            &arc,
            1U);

    test_check(
        ret == ESP_OK,
        "Golden LINE0/CIRCLE window begins");

    if (ret != ESP_OK)
    {
        return;
    }

    trajectory_reference_t reference = {0};

    bool reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached &&
        trajectory_executor_needs_advance(&executor),
        "Golden LINE0 reaches continuous CIRCLE junction");

    if (!reached)
    {
        return;
    }

    test_check(
        reference.speed_mm_s > 1.0f,
        "LINE0->CIRCLE junction keeps non-zero speed");

    ret =
        trajectory_executor_advance_motion(
            &executor,
            &line1,
            2U);

    test_check(
        ret == ESP_OK,
        "Golden advance to CIRCLE with LINE1 lookahead");

    if (ret != ESP_OK)
    {
        return;
    }

    reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached &&
        trajectory_executor_needs_advance(&executor),
        "Golden CIRCLE reaches continuous LINE1 junction");

    if (!reached)
    {
        return;
    }

    test_check(
        reference.speed_mm_s > 1.0f,
        "CIRCLE->LINE1 junction keeps non-zero speed");

    ret =
        trajectory_executor_advance_motion(
            &executor,
            NULL,
            0U);

    test_check(
        ret == ESP_OK,
        "Golden advance to final LINE1");

    if (ret != ESP_OK)
    {
        return;
    }

    reached =
        update_until_boundary(
            &executor,
            &reference);

    test_check(
        reached &&
        trajectory_executor_is_holding(&executor),
        "Golden final LINE enters HOLDING");

    if (!reached)
    {
        return;
    }

    test_check(
        test_closef(
            reference.x_mm,
            0.0f,
            1.0e-3f) &&
        test_closef(
            reference.y_mm,
            500.0f,
            1.0e-3f),
        "Golden endpoint is (0,500)");

    test_check(
        test_closef(
            reference.speed_mm_s,
            0.0f,
            1.0e-3f),
        "Golden final Motion stops at zero speed");

    test_check(
        !reference.trajectory_end,
        "Golden terminal Motion waits for explicit finish");

    ret =
        trajectory_executor_finish(
            &executor);

    test_check(
        ret == ESP_OK,
        "Golden explicit finish succeeds");

    if (ret == ESP_OK)
    {
        ret =
            trajectory_executor_update(
                &executor,
                TEST_DT_S,
                &reference);

        test_check(
            ret == ESP_OK &&
            reference.trajectory_end,
            "Golden finish produces final trajectory_end");
    }
}


void app_main(void)
{
    ESP_LOGI(
        TAG,
        "========================================");

    ESP_LOGI(
        TAG,
        "F2.1 ESP32 Executor test START");

    ESP_LOGI(
        TAG,
        "No Runner / Tracker / Motor / Pen is used");

    ESP_LOGI(
        TAG,
        "========================================");

    test_external_start_holding();
    test_motion_event_motion_barrier();
    test_tangent_motion_chain();
    test_sharp_motion_chain();
    test_golden_line_circle_line();

    ESP_LOGI(
        TAG,
        "========================================");

    ESP_LOGI(
        TAG,
        "F2.1 test summary: PASS=%u FAIL=%u",
        s_pass_count,
        s_fail_count);

    if (s_fail_count == 0U)
    {
        ESP_LOGI(
            TAG,
            "FINAL RESULT: PASS");
    }
    else
    {
        ESP_LOGE(
            TAG,
            "FINAL RESULT: FAIL");
    }

    ESP_LOGI(
        TAG,
        "========================================");
}