#ifndef N3_SERVICE_H
#define N3_SERVICE_H

#include "esp_err.h"

/* Start the N3 communication and upload service.  The default safe build
 * accepts and validates TRJ2 files but never starts a motor, odometry, pen,
 * or trajectory-runner task. Build capabilities are advertised in HELLO and
 * STATUS through build_profile and *_enabled fields. */
esp_err_t n3_service_start(void);

#endif /* N3_SERVICE_H */
