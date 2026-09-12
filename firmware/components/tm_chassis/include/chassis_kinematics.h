#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <math.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * chassis_kinematics.h
 *
 * 3-wheel omni chassis kinematics.
 *
 * Coordinate convention follows motor_control.h:
 *
 *      vx > 0 : forward
 *      vy > 0 : left
 *      w  > 0 : counter-clockwise
 *
 * Units:
 *
 *      vx, vy       : mm/s
 *      w            : rad/s
 *      wheel speed  : mm/s
 *
 * IMPORTANT:
 * This module uses MOTOR_A/B/D_KX/KY/KW and
 * MOTOR_CHASSIS_RADIUS_MM directly, so forward and inverse
 * kinematics always match motor_control.h's actual configuration.
 * ============================================================ */

typedef struct
{
    float vx_mm_s;
    float vy_mm_s;
    float w_rad_s;
} chassis_velocity_t;

typedef struct
{
    float a_mm_s;
    float b_mm_s;
    float d_mm_s;
} chassis_wheel_velocity_t;

typedef struct
{
    float dx_mm;
    float dy_mm;
    float dtheta_rad;
} chassis_body_delta_t;

/**
 * @brief Convert chassis velocity to wheel linear velocities.
 *
 * Pure mathematical conversion. No speed saturation, dead-zone,
 * feedforward or PID is applied here.
 */
esp_err_t chassis_kinematics_inverse(
    const chassis_velocity_t *chassis,
    chassis_wheel_velocity_t *wheels);

/**
 * @brief Convert wheel linear velocities to chassis velocity.
 *
 * This is the exact inverse of the matrix configured by
 * MOTOR_A/B/D_KX/KY/KW and MOTOR_CHASSIS_RADIUS_MM.
 */
esp_err_t chassis_kinematics_forward(
    const chassis_wheel_velocity_t *wheels,
    chassis_velocity_t *chassis);

/**
 * @brief Convert wheel incremental travel to body-frame pose delta.
 *
 * Input:
 *      da_mm, db_mm, dd_mm : each wheel travel increment
 *
 * Output:
 *      dx_mm       : robot-frame forward increment
 *      dy_mm       : robot-frame left increment
 *      dtheta_rad  : encoder-only yaw increment
 *
 * Odometry may choose to use an IMU yaw increment instead of
 * dtheta_rad for the final heading estimate.
 */
esp_err_t chassis_kinematics_forward_delta(
    float da_mm,
    float db_mm,
    float dd_mm,
    chassis_body_delta_t *delta);

/**
 * @brief Validate current geometry / matrix configuration.
 */
bool chassis_kinematics_is_valid(void);

#ifdef __cplusplus
}
#endif


/* ============================================================
 * IMPLEMENTATION
 *
 * Define CHASSIS_KINEMATICS_IMPLEMENTATION in exactly one .c file:
 *
 *      #define CHASSIS_KINEMATICS_IMPLEMENTATION
 *      #include "chassis_kinematics.h"
 * ============================================================ */

#ifdef CHASSIS_KINEMATICS_IMPLEMENTATION

#ifndef CHASSIS_KINEMATICS_DET_EPSILON
#define CHASSIS_KINEMATICS_DET_EPSILON  1.0e-6f
#endif

static float chassis_kinematics_det(void)
{
    const float L = MOTOR_CHASSIS_RADIUS_MM;

    const float a11 = MOTOR_A_KX;
    const float a12 = MOTOR_A_KY;
    const float a13 = MOTOR_A_KW * L;

    const float a21 = MOTOR_B_KX;
    const float a22 = MOTOR_B_KY;
    const float a23 = MOTOR_B_KW * L;

    const float a31 = MOTOR_D_KX;
    const float a32 = MOTOR_D_KY;
    const float a33 = MOTOR_D_KW * L;

    return
        a11 * (a22 * a33 - a23 * a32)
        - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31);
}

bool chassis_kinematics_is_valid(void)
{
    if (MOTOR_CHASSIS_RADIUS_MM <= 0.0f)
    {
        return false;
    }

    return fabsf(chassis_kinematics_det()) >
           CHASSIS_KINEMATICS_DET_EPSILON;
}

esp_err_t chassis_kinematics_inverse(
    const chassis_velocity_t *chassis,
    chassis_wheel_velocity_t *wheels)
{
    if ((chassis == NULL) || (wheels == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!chassis_kinematics_is_valid())
    {
        return ESP_ERR_INVALID_STATE;
    }

    const float L = MOTOR_CHASSIS_RADIUS_MM;

    wheels->a_mm_s =
        MOTOR_A_KX * chassis->vx_mm_s +
        MOTOR_A_KY * chassis->vy_mm_s +
        MOTOR_A_KW * L * chassis->w_rad_s;

    wheels->b_mm_s =
        MOTOR_B_KX * chassis->vx_mm_s +
        MOTOR_B_KY * chassis->vy_mm_s +
        MOTOR_B_KW * L * chassis->w_rad_s;

    wheels->d_mm_s =
        MOTOR_D_KX * chassis->vx_mm_s +
        MOTOR_D_KY * chassis->vy_mm_s +
        MOTOR_D_KW * L * chassis->w_rad_s;

    return ESP_OK;
}

esp_err_t chassis_kinematics_forward(
    const chassis_wheel_velocity_t *wheels,
    chassis_velocity_t *chassis)
{
    if ((wheels == NULL) || (chassis == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const float L = MOTOR_CHASSIS_RADIUS_MM;

    if (L <= 0.0f)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Matrix:
     *
     * [va]   [A_KX  A_KY  A_KW*L] [vx]
     * [vb] = [B_KX  B_KY  B_KW*L] [vy]
     * [vd]   [D_KX  D_KY  D_KW*L] [ w]
     *
     * Solve with Cramer's rule. This keeps the odometry matched
     * to any future changes of the MOTOR_*_K* macros.
     */

    const float a11 = MOTOR_A_KX;
    const float a12 = MOTOR_A_KY;
    const float a13 = MOTOR_A_KW * L;

    const float a21 = MOTOR_B_KX;
    const float a22 = MOTOR_B_KY;
    const float a23 = MOTOR_B_KW * L;

    const float a31 = MOTOR_D_KX;
    const float a32 = MOTOR_D_KY;
    const float a33 = MOTOR_D_KW * L;

    const float b1 = wheels->a_mm_s;
    const float b2 = wheels->b_mm_s;
    const float b3 = wheels->d_mm_s;

    const float det =
        a11 * (a22 * a33 - a23 * a32)
        - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31);

    if (fabsf(det) <= CHASSIS_KINEMATICS_DET_EPSILON)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const float det_vx =
        b1 * (a22 * a33 - a23 * a32)
        - a12 * (b2 * a33 - a23 * b3)
        + a13 * (b2 * a32 - a22 * b3);

    const float det_vy =
        a11 * (b2 * a33 - a23 * b3)
        - b1 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * b3 - b2 * a31);

    const float det_w =
        a11 * (a22 * b3 - b2 * a32)
        - a12 * (a21 * b3 - b2 * a31)
        + b1 * (a21 * a32 - a22 * a31);

    chassis->vx_mm_s = det_vx / det;
    chassis->vy_mm_s = det_vy / det;
    chassis->w_rad_s = det_w / det;

    return ESP_OK;
}

esp_err_t chassis_kinematics_forward_delta(
    float da_mm,
    float db_mm,
    float dd_mm,
    chassis_body_delta_t *delta)
{
    if (delta == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    chassis_wheel_velocity_t wheel_delta = {
        .a_mm_s = da_mm,
        .b_mm_s = db_mm,
        .d_mm_s = dd_mm
    };

    chassis_velocity_t body_delta = {0};

    const esp_err_t ret =
        chassis_kinematics_forward(
            &wheel_delta,
            &body_delta);

    if (ret != ESP_OK)
    {
        return ret;
    }

    /*
     * The kinematic matrix is linear, so applying the same inverse
     * to wheel displacement gives [dx, dy, dtheta].
     */
    delta->dx_mm = body_delta.vx_mm_s;
    delta->dy_mm = body_delta.vy_mm_s;
    delta->dtheta_rad = body_delta.w_rad_s;

    return ESP_OK;
}

#endif /* CHASSIS_KINEMATICS_IMPLEMENTATION */
#endif /* CHASSIS_KINEMATICS_H */
