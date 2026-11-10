/* Trigger generation and ExposureActive capture on MCPWM group 0.
 *
 * This layer knows nothing about ROS. It hands finished pulse records to a caller that
 * either prints them (selftest) or publishes them (micro-ROS), which keeps the timing code
 * testable without a micro-ROS agent attached.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "arts_sync_config.h"
#include "esp_err.h"

typedef struct {
  uint32_t pulse_index;
  uint32_t trigger_ticks;
  uint32_t trigger_period_ticks; /* 0 on the first pulse */
  /* Trigger edge to the START of each camera's ExposureActive assertion. This is the
   * quantity decision 15 needs: it dates the photons, and its inter-camera difference is
   * the skew the paper reports. */
  uint32_t exposure_ticks[ARTS_SYNC_MAX_CAMERAS];
  /* START to END of the same assertion, i.e. how long the strobe was actually asserted.
   * Not needed to date a frame, and recorded anyway because it is the cheapest possible
   * check that the camera did what it was told: it should equal the configured exposure
   * time, it exposes a stuck ExposureAuto immediately, and if it instead comes out as the
   * remainder of the trigger period then ARTS_SYNC_STROBE_ACTIVE_LOW is set the wrong way
   * round and every "start" above is really an end. */
  uint32_t exposure_width_ticks[ARTS_SYNC_MAX_CAMERAS];
#ifdef CONFIG_ARTS_SYNC_EDGE_DIAG
  /* Every strobe capture event of this record as the ISR saw it: the edge type the driver
   * reported, the pin level read inside the ISR, and the tick relative to the trigger. */
  uint8_t diag_n[ARTS_SYNC_MAX_CAMERAS];
  uint8_t diag_edge_neg[ARTS_SYNC_MAX_CAMERAS][4];
  uint8_t diag_level[ARTS_SYNC_MAX_CAMERAS][4];
  uint32_t diag_ticks[ARTS_SYNC_MAX_CAMERAS][4];
#endif
} arts_sync_record_t;

/* Starts the hardware pulse train and fans it out to every trigger pad. */
esp_err_t arts_sync_start_trigger(void);

/* Creates the capture timer and one channel per camera (plus the loopback, if enabled).
 * Call after arts_sync_start_trigger(). */
esp_err_t arts_sync_start_capture(void);

/* Capture counter resolution, read back from the hardware rather than assumed: it follows
 * the MCPWM group clock and has changed between ESP-IDF versions. 0 before start. */
uint32_t arts_sync_capture_resolution_hz(void);

/* Pops one completed record. Returns false when none is pending. A record completes when
 * the FOLLOWING trigger edge arrives, because the exposure strobes of pulse N land tens of
 * microseconds after trigger N — so this lags real time by one pulse period by design. */
bool arts_sync_pop_record(arts_sync_record_t *out);

/* Records dropped because the consumer fell behind the ring buffer. Non-zero means the
 * publish path is too slow and the numbers have gaps. */
uint32_t arts_sync_dropped_records(void);

/* Strobe start edges ignored because their record already had a start (first start wins).
 * Non-zero means something other than an exposure is toggling a strobe line. */
uint32_t arts_sync_strobe_glitches(void);

/* Stage 5a only: software-timed square wave on the primary trigger pad, slow enough to see
 * on an LED. Does not return. Deliberately does not use MCPWM, whose 16-bit period register
 * cannot reach a visible rate. */
void arts_sync_run_blink(void);
