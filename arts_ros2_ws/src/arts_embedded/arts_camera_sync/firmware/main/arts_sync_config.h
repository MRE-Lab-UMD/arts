/* ARTS camera sync box — pin map and compile-time configuration.
 *
 * Board: Espressif ESP32-DevKitC-32E (ESP32-WROOM-32E).
 * Wiring, electrical limits and the bring-up ladder: docs/modules/pilot_flir_camera.md
 * Stage 5. Rationale for the isolated line and the resistor value: AGENTS.md decision 14.
 */
#pragma once

#include "sdkconfig.h"

/* ---- Trigger outputs -----------------------------------------------------------------
 * One MCPWM generator signal is routed to all three pads through the GPIO matrix, so the
 * edges are simultaneous to within pad skew. Toggling three pins in software instead would
 * put the cameras microseconds apart and defeat the entire measurement.
 * Each pad drives its own camera through its own 560 ohm resistor into Line 0 (pin 2,
 * black), so each pad sources only ~5.5 mA.
 */
#define ARTS_SYNC_TRIGGER_GPIO_PRIMARY 25 /* camera 1 (left)  — the generator's own pad */
#define ARTS_SYNC_TRIGGER_GPIO_2       26 /* camera 2 (right) — fanned out */
#define ARTS_SYNC_TRIGGER_GPIO_3       27 /* camera 3         — reserved, fanned out anyway */

/* ---- ExposureActive capture inputs ---------------------------------------------------
 * GPIO 34-39 are input-only and have no internal pull-ups, which is exactly why they suit
 * these signals: they cannot be driven by mistake, and the pull-up the open-collector
 * strobe needs is external anyway (1 kohm to 3V3).
 *
 * BOARD LABELS: GPIO 36 and 39 are silkscreened by their analog alternate function, so a
 * DevKitC (and every breakout for it) has NO terminal marked 36 or 39. GPIO 36 = "VP"
 * (SENSOR_VP), GPIO 39 = "VN" (SENSOR_VN). They are adjacent, so a swap is easy and silent.
 */
#define ARTS_SYNC_STROBE_GPIO_1 34
#define ARTS_SYNC_STROBE_GPIO_2 35
#define ARTS_SYNC_STROBE_GPIO_3 36 /* board label "VP" — reserved for camera 3 */

/* Trigger loopback: jumper from the primary trigger pad back into this pin. Also
 * input-only, so a wiring slip cannot make two outputs fight. Board label "VN". */
#define ARTS_SYNC_LOOPBACK_GPIO 39

/* LED for the Stage 5a visible test — reuses the primary trigger pad, so the LED occupies
 * the same wiring position a camera later will. */
#define ARTS_SYNC_BLINK_GPIO ARTS_SYNC_TRIGGER_GPIO_PRIMARY
#define ARTS_SYNC_BLINK_RATE_HZ 2

/* ---- Derived configuration ---------------------------------------------------------- */

#define ARTS_SYNC_MAX_CAMERAS 3
#define ARTS_SYNC_CAMERA_COUNT CONFIG_ARTS_SYNC_CAMERA_COUNT
#define ARTS_SYNC_RATE_HZ CONFIG_ARTS_SYNC_RATE_HZ
#define ARTS_SYNC_PULSE_WIDTH_US CONFIG_ARTS_SYNC_PULSE_WIDTH_US

#ifdef CONFIG_ARTS_SYNC_TRIGGER_LOOPBACK
#define ARTS_SYNC_USE_LOOPBACK 1
#else
#define ARTS_SYNC_USE_LOOPBACK 0
#endif

/* Strobe polarity AS MEASURED AT THE ESP32 PAD, which is not the same question as the
 * camera's internal signal and was got wrong on the first pass. Both edges are captured
 * regardless; this only decides which of the two is labelled the exposure START, and
 * therefore whether exposure_width_ticks comes out as the exposure time or as the rest of
 * the period. That makes the setting self-checking — see the Kconfig help. */
/* Kept as a plain boolean rather than an MCPWM enum: this header is included before any
 * driver header, so naming MCPWM_CAP_EDGE_* here would not compile. sync_engine.c maps it. */
#ifdef CONFIG_ARTS_SYNC_STROBE_ACTIVE_LOW
#define ARTS_SYNC_STROBE_ACTIVE_LOW 1
#else
#define ARTS_SYNC_STROBE_ACTIVE_LOW 0
#endif

/* MCPWM group 0 provides exactly three capture channels, and the loopback consumes one of
 * them. Fail at compile time rather than at 3 a.m. on the bench. */
#if (ARTS_SYNC_CAMERA_COUNT + ARTS_SYNC_USE_LOOPBACK) > 3
#error "ARTS camera sync: cameras + loopback exceeds the 3 capture channels of MCPWM group 0. \
Disable ARTS_SYNC_TRIGGER_LOOPBACK to free one."
#endif

/* 1 MHz gives 1 us comparator granularity for the pulse width and keeps period_ticks
 * inside the 16-bit MCPWM period register for any rate >= 16 Hz. It also divides the
 * 160 MHz group clock by exactly 160, so the driver needs no fractional prescaler. */
#define ARTS_SYNC_TIMER_RESOLUTION_HZ 1000000u

#define ARTS_SYNC_NO_CAPTURE 0xFFFFFFFFu
