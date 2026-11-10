/* ARTS camera sync box — application entry point.
 *
 * Three builds from one source, matching the Stage 5 bring-up ladder in
 * docs/modules/pilot_flir_camera.md:
 *
 *   BLINK     visible LED pulse, software timed, no camera connected     (Stage 5a)
 *   SELFTEST  real trigger + capture, results printed over the console   (Stage 5b/5c/5d)
 *   MICROROS  publishes arts_interfaces/CameraSyncPulse, console off     (Stage 5e)
 *
 * SELFTEST exists because the MICROROS build takes UART0 for its transport, which means the
 * console is gone and printf debugging is not available. Bring the hardware up under
 * SELFTEST first; switch to MICROROS only once the numbers look right.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "arts_sync_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sync_engine.h"

#ifdef CONFIG_ARTS_SYNC_MODE_MICROROS
#include <arts_interfaces/msg/camera_sync_pulse.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
/* MUST precede rmw_microros.h: it defines RMW_UXRCE_TRANSPORT_CUSTOM, and the custom-transport
 * prototypes inside rmw_microros.h are guarded on it. Omit it and rmw_uros_set_custom_transport
 * is only IMPLICITLY declared — which under C17 is an error, but an error that names the
 * function rather than the missing configuration. */
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "driver/uart.h"
#include "esp32_serial_transport.h"
#include "esp_system.h"

/* The transport is a build-time choice in sdkconfig.defaults.microros, and getting it wrong
 * otherwise produces a board that boots, runs and never establishes a session. Fail here
 * instead. */
#ifndef RMW_UXRCE_TRANSPORT_CUSTOM
#error "micro-ROS custom transport is not enabled: set CONFIG_MICRO_ROS_ESP_UART_TRANSPORT"
#endif
#endif

static const char *TAG = "arts_camera_sync";

/* ------------------------------------------------------------------------------- selftest */
#ifdef CONFIG_ARTS_SYNC_MODE_SELFTEST

typedef struct {
  uint32_t count;
  uint32_t min;
  uint32_t max;
  double sum;
} stat_t;

static void stat_add(stat_t *s, uint32_t v)
{
  if (s->count == 0u || v < s->min) {
    s->min = v;
  }
  if (s->count == 0u || v > s->max) {
    s->max = v;
  }
  s->sum += (double)v;
  s->count++;
}

static void stat_print(const char *label, const stat_t *s, double ns_per_tick)
{
  if (s->count == 0u) {
    printf("  %-22s no captures\n", label);
    return;
  }
  const double k = ns_per_tick / 1000.0; /* ticks -> microseconds */
  printf("  %-22s n=%-6" PRIu32 " mean %9.3f us   min %9.3f   max %9.3f   spread %8.3f\n",
         label, s->count, s->sum / (double)s->count * k, (double)s->min * k, (double)s->max * k,
         (double)(s->max - s->min) * k);
}

static void run_selftest(void)
{
  const double ns_per_tick = 1e9 / (double)arts_sync_capture_resolution_hz();

  stat_t period = {0};
  stat_t exposure[ARTS_SYNC_MAX_CAMERAS] = {0};
  stat_t width[ARTS_SYNC_MAX_CAMERAS] = {0};
  stat_t skew = {0}; /* |cam0 - cam1|, the number the paper reports */
  uint32_t pulses = 0;
  uint32_t missing[ARTS_SYNC_MAX_CAMERAS] = {0};

  TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(2000);

  while (true) {
    arts_sync_record_t rec;
    while (arts_sync_pop_record(&rec)) {
      pulses++;
      if (rec.trigger_period_ticks != 0u) {
        stat_add(&period, rec.trigger_period_ticks);
      }
      for (int i = 0; i < ARTS_SYNC_CAMERA_COUNT; ++i) {
        if (rec.exposure_ticks[i] == ARTS_SYNC_NO_CAPTURE) {
          missing[i]++;
        } else {
          stat_add(&exposure[i], rec.exposure_ticks[i]);
        }
        if (rec.exposure_width_ticks[i] != ARTS_SYNC_NO_CAPTURE) {
          stat_add(&width[i], rec.exposure_width_ticks[i]);
        }
#ifdef CONFIG_ARTS_SYNC_EDGE_DIAG
        bool odd = rec.exposure_ticks[i] == ARTS_SYNC_NO_CAPTURE || rec.diag_n[i] != 2u;
        for (int e = 0; e < rec.diag_n[i] && e < 4; ++e) {
          odd |= rec.diag_edge_neg[i][e] == rec.diag_level[i][e]; /* NEG with pin high, or POS low */
        }
        if (odd) {
          printf("DIAG pulse %" PRIu32 " cam%d n=%u:", rec.pulse_index, i, rec.diag_n[i]);
          for (int e = 0; e < rec.diag_n[i] && e < 4; ++e) {
            printf(" [%s lvl%u %.3f us]", rec.diag_edge_neg[i][e] ? "NEG" : "POS",
                   rec.diag_level[i][e], rec.diag_ticks[i][e] * ns_per_tick / 1000.0);
          }
          printf("\n");
        }
#endif
      }
      if (ARTS_SYNC_CAMERA_COUNT >= 2 && rec.exposure_ticks[0] != ARTS_SYNC_NO_CAPTURE &&
          rec.exposure_ticks[1] != ARTS_SYNC_NO_CAPTURE) {
        const uint32_t a = rec.exposure_ticks[0];
        const uint32_t b = rec.exposure_ticks[1];
        stat_add(&skew, (a > b) ? (a - b) : (b - a));
      }
    }

    if (xTaskGetTickCount() >= next_report) {
      next_report = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
      /* Every figure below is CUMULATIVE SINCE BOOT, deliberately: spread over thousands of
       * samples is the number worth having, and a 2 s window would only ever show 60. The
       * cost is that pulses recorded before the camera was streaming stay in the totals, so
       * a large "no strobe" count usually just dates from before the driver was started —
       * press the board's EN button to zero everything. */
      printf("\n--- pulses %" PRIu32 "  dropped-records %" PRIu32 "  strobe-glitches %" PRIu32
             "  %.3f ns/tick  (cumulative since boot)\n",
             pulses, arts_sync_dropped_records(), arts_sync_strobe_glitches(), ns_per_tick);
      stat_print("trigger period", &period, ns_per_tick);
      for (int i = 0; i < ARTS_SYNC_CAMERA_COUNT; ++i) {
        char label[32];
        snprintf(label, sizeof(label), "cam%d trigger->exposure", i);
        stat_print(label, &exposure[i], ns_per_tick);
        /* Should equal the configured exposure time. If it is instead the trigger period
         * minus the exposure, ARTS_SYNC_STROBE_ACTIVE_LOW is inverted and the line above is
         * a trigger-to-exposure-END figure. */
        snprintf(label, sizeof(label), "cam%d exposure WIDTH", i);
        stat_print(label, &width[i], ns_per_tick);
        if (missing[i] != 0u) {
          printf("  %-22s %" PRIu32 " pulses with no strobe\n", "", missing[i]);
        }
      }
      if (ARTS_SYNC_CAMERA_COUNT >= 2) {
        stat_print("|cam0-cam1| SKEW", &skew, ns_per_tick);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
#endif /* SELFTEST */

/* ------------------------------------------------------------------------------- micro-ROS */
#ifdef CONFIG_ARTS_SYNC_MODE_MICROROS

#define RCCHECK(fn)                                                                     \
  do {                                                                                  \
    const rcl_ret_t rc__ = (fn);                                                         \
    if (rc__ != RCL_RET_OK) {                                                            \
      ESP_LOGE(TAG, "%s failed: %d", #fn, (int)rc__);                                    \
      vTaskDelay(pdMS_TO_TICKS(2000));                                                   \
      esp_restart();                                                                     \
    }                                                                                   \
  } while (0)

static char s_frame_id[] = "camera_sync";

/* Consecutive failed publishes before assuming the agent session is unrecoverable. At 30 Hz
 * this is about ten seconds. */
#define ARTS_SYNC_PUBLISH_FAIL_LIMIT 300

/* Agent liveness. Over the serial transport a best-effort publish succeeds whether or not an
 * agent is listening, so the publish-failure reboot above never fires when the agent process
 * dies: observed 2026-10-28, the board kept triggering the cameras and published into the
 * void until it was reset by hand, and a restarted agent never saw it again. So ping the agent
 * every ARTS_SYNC_PING_PERIOD_MS and reboot after ARTS_SYNC_PING_FAIL_LIMIT consecutive
 * misses (~6 s); the reboot reconnects to whatever agent is running. Triggering pauses for
 * the reboot, and pulse_index restarts, so the gap is visible in the data. */
#define ARTS_SYNC_PING_PERIOD_MS 2000
#define ARTS_SYNC_PING_TIMEOUT_MS 100
#define ARTS_SYNC_PING_FAIL_LIMIT 3

static void run_microros(void)
{
  static size_t uart_port = UART_NUM_0;
  rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open, esp32_serial_close,
                               esp32_serial_write, esp32_serial_read);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_t node;
  /* Decision 18: the sync box serves the ECM's stereo pair, so it lives in /ecm as
   * stereo_sync and publishes /ecm/stereo_sync/pulse (was /arts_camera_sync/pulse). */
  RCCHECK(rclc_node_init_default(&node, "stereo_sync", "ecm", &support));

  rcl_publisher_t pub;
  /* Best effort: a lost pulse report is not worth retransmitting, and pulse_index makes the
   * gap visible to any consumer. */
  RCCHECK(rclc_publisher_init_best_effort(
      &pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(arts_interfaces, msg, CameraSyncPulse),
      "stereo_sync/pulse"));

  arts_interfaces__msg__CameraSyncPulse msg;
  memset(&msg, 0, sizeof(msg));
  msg.header.frame_id.data = s_frame_id;
  msg.header.frame_id.size = strlen(s_frame_id);
  msg.header.frame_id.capacity = sizeof(s_frame_id);
  msg.camera_count = ARTS_SYNC_CAMERA_COUNT;
  msg.tick_period_ns = (float)(1e9 / (double)arts_sync_capture_resolution_hz());

  /* The session clock drifts against the agent, so re-sync periodically. header.stamp is
   * only ever the coarse ROS-time anchor; the tick fields carry the precision. */
  bool synced = (rmw_uros_sync_session(1000) == RMW_RET_OK);
  TickType_t next_sync = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
  uint32_t publish_failures = 0;
  TickType_t next_ping = xTaskGetTickCount() + pdMS_TO_TICKS(ARTS_SYNC_PING_PERIOD_MS);
  uint32_t ping_failures = 0;

  while (true) {
    if (xTaskGetTickCount() >= next_ping) {
      next_ping = xTaskGetTickCount() + pdMS_TO_TICKS(ARTS_SYNC_PING_PERIOD_MS);
      if (rmw_uros_ping_agent(ARTS_SYNC_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
        ping_failures = 0;
      } else if (++ping_failures >= ARTS_SYNC_PING_FAIL_LIMIT) {
        esp_restart();
      }
    }

    if (xTaskGetTickCount() >= next_sync) {
      next_sync = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
      synced = (rmw_uros_sync_session(200) == RMW_RET_OK) || synced;
    }

    arts_sync_record_t rec;
    while (arts_sync_pop_record(&rec)) {
      const int64_t now_ns = synced ? rmw_uros_epoch_nanos() : 0;
      msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
      msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
      msg.pulse_index = rec.pulse_index;
      msg.trigger_ticks = rec.trigger_ticks;
      msg.trigger_period_ticks = rec.trigger_period_ticks;
      for (int i = 0; i < ARTS_SYNC_MAX_CAMERAS; ++i) {
        msg.exposure_delay_ticks[i] = rec.exposure_ticks[i];
        msg.exposure_width_ticks[i] = rec.exposure_width_ticks[i];
      }
      msg.time_synced = synced;

      /* A failed publish almost always means the agent session is gone, and nothing here
       * re-establishes it — so without this the board would keep triggering the cameras while
       * publishing nothing for the rest of the recording, which is the silent-data-loss shape
       * this whole box exists to remove. There is no console in this build to log to, so
       * rebooting is the recovery, consistent with RCCHECK above. The threshold is generous
       * because a single failure can just be a full history at best-effort QoS. */
      if (rcl_publish(&pub, &msg, NULL) == RCL_RET_OK) {
        publish_failures = 0;
      } else if (++publish_failures >= ARTS_SYNC_PUBLISH_FAIL_LIMIT) {
        esp_restart();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
#endif /* MICROROS */

/* ------------------------------------------------------------------------------- app_main */

void app_main(void)
{
#ifdef CONFIG_ARTS_SYNC_MODE_BLINK
  ESP_LOGI(TAG, "mode: BLINK (Stage 5a)");
  arts_sync_run_blink(); /* does not return */
#else
  ESP_ERROR_CHECK(arts_sync_start_trigger());
  ESP_ERROR_CHECK(arts_sync_start_capture());

#ifdef CONFIG_ARTS_SYNC_MODE_SELFTEST
  ESP_LOGI(TAG, "mode: SELFTEST — printing capture statistics every 2 s");
  run_selftest();
#else
  run_microros();
#endif
#endif
}
