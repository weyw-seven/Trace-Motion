/*
 * Trajectory implementation registry.
 *
 * The decoder is needed by the N3 upload validator. The executor is
 * hardware-free and is also used by the simulation profile. The hardware
 * motion profile owns the tracker and runner implementations here so the
 * N3 bridge can use the same production execution path as the rest of the
 * trajectory stack.
 */

#include "n3_build_config.h"

#define TRAJECTORY_DECODER_IMPLEMENTATION
#include "trajectory_decoder.h"

#define TRAJECTORY_EXECUTOR_IMPLEMENTATION
#include "trajectory_executor.h"

#if N3_ENABLE_MOTION
#define PEN_CONTROL_IMPLEMENTATION
#include "pen_control.h"

#define TRAJECTORY_TRACKER_IMPLEMENTATION
#include "trajectory_tracker.h"

#define TRAJECTORY_RUNNER_IMPLEMENTATION
#include "trajectory_runner.h"
#endif
