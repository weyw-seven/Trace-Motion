#ifndef N3_BUILD_CONFIG_H
#define N3_BUILD_CONFIG_H

/*
 * Build-time safety profile.
 *
 * These defaults deliberately describe the currently flashed safe firmware.
 * A future motion profile must opt in explicitly at build time; no runtime
 * command can turn these flags on.
 */

#ifndef N3_ENABLE_HARDWARE
#define N3_ENABLE_HARDWARE 0
#endif

#ifndef N3_ENABLE_MOTION
#define N3_ENABLE_MOTION 0
#endif

#ifndef N3_ENABLE_CIRCLE
#define N3_ENABLE_CIRCLE 0
#endif

#ifndef N3_ENABLE_ROTATE_REL
#define N3_ENABLE_ROTATE_REL 0
#endif

#ifndef N3_ENABLE_PEN
#define N3_ENABLE_PEN 0
#endif

#ifndef N3_ENABLE_SIMULATION
#define N3_ENABLE_SIMULATION 0
#endif

#if N3_ENABLE_MOTION && !N3_ENABLE_HARDWARE
#error "N3_ENABLE_MOTION requires N3_ENABLE_HARDWARE"
#endif

#if N3_ENABLE_CIRCLE && !N3_ENABLE_MOTION
#error "N3_ENABLE_CIRCLE requires N3_ENABLE_MOTION"
#endif

#if N3_ENABLE_ROTATE_REL && !N3_ENABLE_HARDWARE
#error "N3_ENABLE_ROTATE_REL requires N3_ENABLE_HARDWARE"
#endif

#if N3_ENABLE_ROTATE_REL && N3_ENABLE_MOTION
#error "N3_ENABLE_ROTATE_REL is a separate acceptance profile; do not combine it with N3_ENABLE_MOTION"
#endif

#if N3_ENABLE_PEN && !N3_ENABLE_HARDWARE
#error "N3_ENABLE_PEN requires N3_ENABLE_HARDWARE"
#endif

#if N3_ENABLE_PEN && !N3_ENABLE_MOTION
#error "N3_ENABLE_PEN requires the hardware trajectory runner"
#endif

#if N3_ENABLE_SIMULATION && N3_ENABLE_HARDWARE
#error "N3_ENABLE_SIMULATION is a hardware-free execution profile"
#endif

#if N3_ENABLE_SIMULATION && N3_ENABLE_MOTION
#error "N3_ENABLE_SIMULATION and N3_ENABLE_MOTION are mutually exclusive"
#endif

#if N3_ENABLE_MOTION && N3_ENABLE_CIRCLE && N3_ENABLE_PEN
#define N3_BUILD_PROFILE "motion-circle-pen"
#define N3_EXECUTION_MODE "HARDWARE_DRAW"
#elif N3_ENABLE_MOTION && N3_ENABLE_CIRCLE
#define N3_BUILD_PROFILE "motion-circle"
#define N3_EXECUTION_MODE "HARDWARE_PATH"
#elif N3_ENABLE_MOTION && N3_ENABLE_PEN
#define N3_BUILD_PROFILE "motion-line-pen"
#define N3_EXECUTION_MODE "HARDWARE_DRAW"
#elif N3_ENABLE_MOTION
#define N3_BUILD_PROFILE "motion-line"
#define N3_EXECUTION_MODE "HARDWARE_LINE"
#elif N3_ENABLE_ROTATE_REL
#define N3_BUILD_PROFILE "motion-rotate"
#define N3_EXECUTION_MODE "HARDWARE_ROTATE"
#elif N3_ENABLE_HARDWARE
#define N3_BUILD_PROFILE "hardware-check"
#define N3_EXECUTION_MODE "HARDWARE_CHECK"
#elif N3_ENABLE_SIMULATION
#define N3_BUILD_PROFILE "simulation"
#define N3_EXECUTION_MODE "SIMULATED"
#else
#define N3_BUILD_PROFILE "safe"
#define N3_EXECUTION_MODE "EVENT_ONLY"
#endif

#endif /* N3_BUILD_CONFIG_H */
