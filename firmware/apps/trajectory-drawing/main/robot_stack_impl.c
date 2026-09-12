/*
 * Robot hardware implementation registry.
 *
 * The project keeps several public modules in single-header form. Each
 * implementation macro must be defined in exactly one translation unit. The
 * production build owns those definitions here so future N3 runtime code can
 * include the public headers without creating duplicate symbols.
 *
 * This file only provides implementations; it does not initialize hardware.
 * The safe N3 profile therefore still cannot drive motors merely because this
 * file is linked into the image.
 */

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
