#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

/*
 * Single-header implementation.
 *
 * 注意：
 * 整个工程只能有一个 .c 定义这些 IMPLEMENTATION 宏。
 *
 * 如果你已经在别的 .c 里定义了
 * TRAJECTORY_DECODER_IMPLEMENTATION，
 * 那这里就删掉对应的 #define。
 */
#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#include "trajectory_executor.h"


static const char *TAG = "EXEC_TEST";


/* ============================================================
 * Test configuration
 * ============================================================ */

/*
 * Executor nominal update period.
 *
 * 这里不是依赖真实 wall-clock 时间，
 * 而是进行 deterministic simulation：
 *
 * 每调用一次 executor_update()
 * 就假设过去了 10 ms。
 */
#define TEST_DT_S                       0.010f


/*
 * 每多少次 update 打一次常规日志。
 *
 * 10 -> 每 100 ms 打一次。
 *
 * segment start/end 和 trajectory end
 * 无论如何都会打印。
 */
#define TEST_LOG_EVERY_N_UPDATES        10U


/*
 * 防止 executor 因 bug 永远不结束。
 *
 * 120000 * 10 ms = 1200 s
 */
#define TEST_MAX_UPDATES                120000U


/*
 * 数值检查容差。
 */
#define TEST_TANGENT_NORM_TOL           0.02f

#define TEST_PROGRESS_TOL               0.002f

#define TEST_FINAL_POSITION_TOL_MM      1.0f


/*
 * 这是我们当前 test.traj 的预期终点：
 *
 * LINE:
 *   (0,0) -> (500,0)
 *
 * CIRCLE:
 *   center=(500,250)
 *   radius=250
 *   start=-90 deg
 *   sweep=+180 deg
 *
 * LINE:
 *   (500,500) -> (0,500)
 */
#define TEST_EXPECT_FINAL_POSE          1

#define TEST_EXPECT_FINAL_X_MM          0.0f
#define TEST_EXPECT_FINAL_Y_MM          500.0f


/*
 * 是否顺便测试 pause/resume API。
 *
 * 在运行约 1 秒之后：
 *
 * RUNNING -> PAUSED -> RUNNING
 */
#define TEST_ENABLE_PAUSE_RESUME        1


/* ============================================================
 * Test statistics
 * ============================================================ */

typedef struct
{
    uint32_t update_count;
    uint32_t reference_count;
    uint32_t segment_transition_count;

    float simulated_time_s;

    bool have_previous_reference;

    trajectory_reference_t previous_reference;

} executor_test_stats_t;


/* ============================================================
 * SPIFFS
 * ============================================================ */

static esp_err_t test_mount_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf =
    {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = false,
    };


    esp_err_t ret =
        esp_vfs_spiffs_register(&conf);


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "SPIFFS mount failed: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }


    size_t total = 0U;
    size_t used = 0U;


    ret =
        esp_spiffs_info(
            NULL,
            &total,
            &used
        );


    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "SPIFFS: total=%u, used=%u",
            (unsigned)total,
            (unsigned)used
        );
    }
    else
    {
        ESP_LOGW(
            TAG,
            "esp_spiffs_info failed: %s",
            esp_err_to_name(ret)
        );
    }


    return ESP_OK;
}


/* ============================================================
 * Helpers
 * ============================================================ */

static float test_distance(
    float x0,
    float y0,
    float x1,
    float y1)
{
    const float dx = x1 - x0;
    const float dy = y1 - y0;

    return sqrtf(
        dx * dx +
        dy * dy
    );
}


static bool test_is_finite_reference(
    const trajectory_reference_t *ref)
{
    if (ref == NULL)
    {
        return false;
    }


    return
        isfinite(ref->x_mm) &&
        isfinite(ref->y_mm) &&

        isfinite(ref->tangent_x) &&
        isfinite(ref->tangent_y) &&

        isfinite(ref->curvature_inv_mm) &&

        isfinite(ref->speed_mm_s) &&
        isfinite(ref->acceleration_mm_s2) &&

        isfinite(ref->s_mm) &&
        isfinite(ref->segment_length_mm) &&
        isfinite(ref->progress);
}


/* ============================================================
 * Reference validation
 * ============================================================ */

static esp_err_t test_validate_reference(
    const trajectory_reference_t *ref)
{
    if (ref == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    if (!ref->valid)
    {
        ESP_LOGE(
            TAG,
            "Reference marked invalid"
        );

        return ESP_ERR_INVALID_STATE;
    }


    /*
     * 1. NaN / Inf check
     */
    if (!test_is_finite_reference(ref))
    {
        ESP_LOGE(
            TAG,
            "Reference contains NaN/Inf"
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    /*
     * 2. Tangent should be approximately unit length.
     */
    const float tangent_norm =
        sqrtf(
            ref->tangent_x * ref->tangent_x +
            ref->tangent_y * ref->tangent_y
        );


    if (fabsf(tangent_norm - 1.0f) >
        TEST_TANGENT_NORM_TOL)
    {
        ESP_LOGE(
            TAG,
            "Bad tangent norm: %.6f "
            "(tx=%.6f ty=%.6f)",
            tangent_norm,
            ref->tangent_x,
            ref->tangent_y
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    /*
     * 3. Segment length must be positive.
     */
    if (ref->segment_length_mm <= 0.0f)
    {
        ESP_LOGE(
            TAG,
            "Invalid segment length: %.6f mm",
            ref->segment_length_mm
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    /*
     * 4. s must remain inside the segment.
     */
    if ((ref->s_mm < -0.01f) ||
        (ref->s_mm >
            ref->segment_length_mm + 0.01f))
    {
        ESP_LOGE(
            TAG,
            "s out of range: "
            "s=%.3f length=%.3f",
            ref->s_mm,
            ref->segment_length_mm
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    /*
     * 5. progress should remain approximately 0..1.
     */
    if ((ref->progress < -TEST_PROGRESS_TOL) ||
        (ref->progress > 1.0f + TEST_PROGRESS_TOL))
    {
        ESP_LOGE(
            TAG,
            "Progress out of range: %.6f",
            ref->progress
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    /*
     * 6. Linear path speed should not be negative.
     *
     * Direction is represented by tangent,
     * not by negative scalar path speed.
     */
    if (ref->speed_mm_s < -0.01f)
    {
        ESP_LOGE(
            TAG,
            "Negative path speed: %.3f mm/s",
            ref->speed_mm_s
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    return ESP_OK;
}


/* ============================================================
 * Segment transition continuity check
 * ============================================================ */

static esp_err_t test_check_transition(
    const trajectory_reference_t *previous,
    const trajectory_reference_t *current)
{
    if ((previous == NULL) ||
        (current == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * Only interested in actual segment transitions.
     */
    if (previous->segment_index ==
        current->segment_index)
    {
        return ESP_OK;
    }


    const float jump_mm =
        test_distance(
            previous->x_mm,
            previous->y_mm,
            current->x_mm,
            current->y_mm
        );


    /*
     * During one 10 ms update, the reference is allowed
     * to move naturally along the path.
     *
     * This threshold therefore depends on speed instead
     * of demanding x/y to be numerically identical.
     *
     * A real geometry discontinuity such as a 20/50 mm
     * jump should still be caught.
     */
    const float vmax =
        fmaxf(
            previous->speed_mm_s,
            current->speed_mm_s
        );


    const float allowed_jump_mm =
        vmax * TEST_DT_S * 2.0f +
        2.0f;


    ESP_LOGI(
        TAG,
        "----------------------------------------"
    );

    ESP_LOGI(
        TAG,
        "SEGMENT TRANSITION: %lu -> %lu",
        (unsigned long)previous->segment_index,
        (unsigned long)current->segment_index
    );

    ESP_LOGI(
        TAG,
        "transition position jump = %.3f mm "
        "(allowed %.3f mm)",
        jump_mm,
        allowed_jump_mm
    );

    ESP_LOGI(
        TAG,
        "previous tangent = (%.4f, %.4f)",
        previous->tangent_x,
        previous->tangent_y
    );

    ESP_LOGI(
        TAG,
        "current  tangent = (%.4f, %.4f)",
        current->tangent_x,
        current->tangent_y
    );

    ESP_LOGI(
        TAG,
        "speed %.2f -> %.2f mm/s",
        previous->speed_mm_s,
        current->speed_mm_s
    );


    if (jump_mm > allowed_jump_mm)
    {
        ESP_LOGE(
            TAG,
            "REFERENCE DISCONTINUITY DETECTED"
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    return ESP_OK;
}


/* ============================================================
 * Print reference
 * ============================================================ */

static void test_print_reference(
    float time_s,
    const trajectory_reference_t *ref)
{
    if ((ref == NULL) ||
        (!ref->valid))
    {
        return;
    }


    const char *type_name =
        trajectory_decoder_segment_type_name(
            ref->source_type
        );


    ESP_LOGI(
        TAG,
        "t=%7.3f "
        "seg=%lu %-6s "
        "pos=(%8.2f,%8.2f) "
        "tan=(%7.4f,%7.4f) "
        "k=% .6f "
        "v=%7.2f "
        "a=%8.2f "
        "s=%8.2f/%8.2f "
        "p=%6.3f "
        "%s%s%s",
        time_s,

        (unsigned long)ref->segment_index,
        type_name,

        ref->x_mm,
        ref->y_mm,

        ref->tangent_x,
        ref->tangent_y,

        ref->curvature_inv_mm,

        ref->speed_mm_s,
        ref->acceleration_mm_s2,

        ref->s_mm,
        ref->segment_length_mm,

        ref->progress,

        ref->segment_start ? "[START]" : "",
        ref->segment_end   ? "[END]"   : "",
        ref->trajectory_end ? "[TRAJ_END]" : ""
    );
}


/* ============================================================
 * Final result check
 * ============================================================ */

static esp_err_t test_check_final_reference(
    const trajectory_reference_t *ref)
{
    if ((ref == NULL) ||
        (!ref->valid))
    {
        ESP_LOGE(
            TAG,
            "No valid final reference"
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


#if TEST_EXPECT_FINAL_POSE

    const float error_mm =
        test_distance(
            ref->x_mm,
            ref->y_mm,
            TEST_EXPECT_FINAL_X_MM,
            TEST_EXPECT_FINAL_Y_MM
        );


    ESP_LOGI(
        TAG,
        "Expected final = (%.3f, %.3f) mm",
        TEST_EXPECT_FINAL_X_MM,
        TEST_EXPECT_FINAL_Y_MM
    );

    ESP_LOGI(
        TAG,
        "Actual final   = (%.3f, %.3f) mm",
        ref->x_mm,
        ref->y_mm
    );

    ESP_LOGI(
        TAG,
        "Final error    = %.6f mm",
        error_mm
    );


    if (error_mm >
        TEST_FINAL_POSITION_TOL_MM)
    {
        ESP_LOGE(
            TAG,
            "Final position check FAILED"
        );

        return ESP_ERR_INVALID_RESPONSE;
    }

#endif


    /*
     * At trajectory completion progress should be 1.
     */
    if (fabsf(ref->progress - 1.0f) >
        TEST_PROGRESS_TOL)
    {
        ESP_LOGE(
            TAG,
            "Final progress != 1: %.6f",
            ref->progress
        );

        return ESP_ERR_INVALID_RESPONSE;
    }


    return ESP_OK;
}


/* ============================================================
 * Executor test
 * ============================================================ */

static esp_err_t test_executor(
    const char *path)
{
    esp_err_t ret;


    trajectory_decoder_t decoder =
        TRAJECTORY_DECODER_INITIALIZER;


    /*
     * executor_init() owns initialization of executor state,
     * so zero initialization is enough here.
     */
    trajectory_executor_t executor = {0};


    trajectory_reference_t ref = {0};


    executor_test_stats_t stats =
    {
        0
    };


    /* --------------------------------------------------------
     * 1. Open trajectory
     * -------------------------------------------------------- */

    ESP_LOGI(
        TAG,
        "Opening trajectory: %s",
        path
    );


    ret =
        trajectory_decoder_open(
            &decoder,
            path
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_decoder_open failed: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }


    /* --------------------------------------------------------
     * 2. Initialize executor
     * -------------------------------------------------------- */

    ret =
        trajectory_executor_init(
            &executor,
            &decoder
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_executor_init failed: %s",
            esp_err_to_name(ret)
        );

        trajectory_decoder_close(
            &decoder
        );

        return ret;
    }


    /* --------------------------------------------------------
     * 3. Start
     * -------------------------------------------------------- */

    ret =
        trajectory_executor_start(
            &executor
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "trajectory_executor_start failed: %s",
            esp_err_to_name(ret)
        );

        trajectory_decoder_close(
            &decoder
        );

        return ret;
    }


    ESP_LOGI(
        TAG,
        "Executor started"
    );


    bool finished = false;
    bool pause_resume_tested = false;


    /* --------------------------------------------------------
     * 4. Nominal 10 ms simulation
     * -------------------------------------------------------- */

    for (uint32_t step = 0U;
         step < TEST_MAX_UPDATES;
         ++step)
    {
#if TEST_ENABLE_PAUSE_RESUME

        /*
         * Exercise state-machine APIs once.
         */
        if ((!pause_resume_tested) &&
            (stats.simulated_time_s >= 1.0f))
        {
            ESP_LOGI(
                TAG,
                "Testing pause/resume..."
            );


            ret =
                trajectory_executor_pause(
                    &executor
                );


            if (ret != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "pause failed: %s",
                    esp_err_to_name(ret)
                );

                goto fail;
            }


            ret =
                trajectory_executor_resume(
                    &executor
                );


            if (ret != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "resume failed: %s",
                    esp_err_to_name(ret)
                );

                goto fail;
            }


            ESP_LOGI(
                TAG,
                "pause/resume OK"
            );


            pause_resume_tested = true;
        }

#endif


        memset(
            &ref,
            0,
            sizeof(ref)
        );


        ret =
            trajectory_executor_update(
                &executor,
                TEST_DT_S,
                &ref
            );


        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "executor_update failed "
                "at step=%lu t=%.3f: %s",
                (unsigned long)step,
                stats.simulated_time_s,
                esp_err_to_name(ret)
            );

            goto fail;
        }


        stats.update_count++;

        stats.simulated_time_s +=
            TEST_DT_S;


        /*
         * A running trajectory should normally provide
         * a valid nominal reference.
         */
        if (ref.valid)
        {
            stats.reference_count++;


            ret =
                test_validate_reference(
                    &ref
                );


            if (ret != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "Reference validation failed "
                    "at t=%.3f",
                    stats.simulated_time_s
                );

                goto fail;
            }


            /*
             * Detect segment transition.
             */
            if (stats.have_previous_reference)
            {
                if (stats.previous_reference.segment_index !=
                    ref.segment_index)
                {
                    stats.segment_transition_count++;


                    ret =
                        test_check_transition(
                            &stats.previous_reference,
                            &ref
                        );


                    if (ret != ESP_OK)
                    {
                        goto fail;
                    }
                }
            }


            /*
             * Periodic log + important events.
             */
            const bool periodic_log =
                ((stats.update_count %
                  TEST_LOG_EVERY_N_UPDATES) == 0U);


            if (periodic_log ||
                ref.segment_start ||
                ref.segment_end ||
                ref.trajectory_end)
            {
                test_print_reference(
                    stats.simulated_time_s,
                    &ref
                );
            }


            stats.previous_reference = ref;
            stats.have_previous_reference = true;
        }


        /*
         * Finished state is owned by executor.
         */
        if (trajectory_executor_is_finished(
                &executor))
        {
            finished = true;

            break;
        }
    }


    /* --------------------------------------------------------
     * 5. Test timeout protection
     * -------------------------------------------------------- */

    if (!finished)
    {
        ESP_LOGE(
            TAG,
            "Executor did not finish after "
            "%lu updates (%.3f simulated seconds)",
            (unsigned long)stats.update_count,
            stats.simulated_time_s
        );

        ret = ESP_ERR_TIMEOUT;

        goto fail;
    }


    /* --------------------------------------------------------
     * 6. Final validation
     * -------------------------------------------------------- */

    ret =
        test_check_final_reference(
            &stats.previous_reference
        );


    if (ret != ESP_OK)
    {
        goto fail;
    }


    /* --------------------------------------------------------
     * 7. Success report
     * -------------------------------------------------------- */

    ESP_LOGI(
        TAG,
        "========================================"
    );

    ESP_LOGI(
        TAG,
        "EXECUTOR TEST PASSED"
    );

    ESP_LOGI(
        TAG,
        "updates             = %lu",
        (unsigned long)stats.update_count
    );

    ESP_LOGI(
        TAG,
        "references          = %lu",
        (unsigned long)stats.reference_count
    );

    ESP_LOGI(
        TAG,
        "segment transitions = %lu",
        (unsigned long)stats.segment_transition_count
    );

    ESP_LOGI(
        TAG,
        "simulated time      = %.3f s",
        stats.simulated_time_s
    );

    ESP_LOGI(
        TAG,
        "final segment       = %lu",
        (unsigned long)
            stats.previous_reference.segment_index
    );

    ESP_LOGI(
        TAG,
        "final position      = (%.3f, %.3f) mm",
        stats.previous_reference.x_mm,
        stats.previous_reference.y_mm
    );

    ESP_LOGI(
        TAG,
        "final tangent       = (%.4f, %.4f)",
        stats.previous_reference.tangent_x,
        stats.previous_reference.tangent_y
    );

    ESP_LOGI(
        TAG,
        "final speed         = %.3f mm/s",
        stats.previous_reference.speed_mm_s
    );

    ESP_LOGI(
        TAG,
        "========================================"
    );


    trajectory_decoder_close(
        &decoder
    );


    return ESP_OK;


fail:

    ESP_LOGE(
        TAG,
        "========================================"
    );

    ESP_LOGE(
        TAG,
        "EXECUTOR TEST FAILED"
    );

    ESP_LOGE(
        TAG,
        "updates        = %lu",
        (unsigned long)stats.update_count
    );

    ESP_LOGE(
        TAG,
        "simulated time = %.3f s",
        stats.simulated_time_s
    );

    ESP_LOGE(
        TAG,
        "========================================"
    );


    trajectory_executor_stop(
        &executor
    );


    trajectory_decoder_close(
        &decoder
    );


    return ret;
}


/* ============================================================
 * app_main
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "========================================"
    );

    ESP_LOGI(
        TAG,
        "Trajectory Executor Test"
    );

    ESP_LOGI(
        TAG,
        "dt = %.3f s",
        TEST_DT_S
    );

    ESP_LOGI(
        TAG,
        "NO MOTOR / ODOMETRY / IMU"
    );

    ESP_LOGI(
        TAG,
        "========================================"
    );


    esp_err_t ret =
        test_mount_spiffs();


    if (ret != ESP_OK)
    {
        return;
    }


    ret =
        test_executor(
            "/spiffs/test.traj"
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Test returned: %s",
            esp_err_to_name(ret)
        );
    }


    esp_vfs_spiffs_unregister(
        NULL
    );


    ESP_LOGI(
        TAG,
        "app_main finished"
    );
}
