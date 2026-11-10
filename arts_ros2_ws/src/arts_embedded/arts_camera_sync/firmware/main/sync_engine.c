#include "sync_engine.h"

#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/mcpwm_periph.h"

static const char *TAG = "arts_sync";

/* Power of two so the index arithmetic is a mask. */
#define RING_LEN 16u
#define RING_MASK (RING_LEN - 1u)

static arts_sync_record_t s_ring[RING_LEN];
static volatile uint32_t s_head; /* producer: the MCPWM ISR */
static volatile uint32_t s_tail; /* consumer: the publish task */
static volatile uint32_t s_dropped;

/* The record being filled. Touched only from the MCPWM group interrupt, which serializes
 * every capture channel in the group, so these need no lock. */
static arts_sync_record_t s_open;
static bool s_open_valid;
static uint32_t s_next_pulse_index;
static uint32_t s_last_trigger_ticks;
static bool s_have_last_trigger;

static mcpwm_cap_timer_handle_t s_cap_timer;
static uint32_t s_cap_resolution_hz;

static const int s_strobe_gpio[ARTS_SYNC_MAX_CAMERAS] = {
    ARTS_SYNC_STROBE_GPIO_1,
    ARTS_SYNC_STROBE_GPIO_2,
    ARTS_SYNC_STROBE_GPIO_3,
};

/* Channel index handed to the capture callback as user_ctx. It must be a REAL pointer into
 * internal RAM, which is why this array exists instead of the obvious (void *)(intptr_t)i
 * cast: with CONFIG_MCPWM_ISR_CACHE_SAFE the driver calls esp_ptr_internal() on user_ctx and
 * rejects anything else with ESP_ERR_INVALID_ARG. The cast trick ALSO passes NULL for index 0,
 * which the driver waves through, so channel 0 registers and channel 1 is the first to fail —
 * making it look like a fault specific to the second strobe pin. Verified on hardware
 * 2026-08-18.
 * Deliberately NOT const: const would place it in .rodata, which is flash-mapped, and an
 * IRAM_ATTR ISR must not touch flash. Plain static lands in .data, i.e. DRAM. */
static int s_strobe_ctx[ARTS_SYNC_MAX_CAMERAS] = {0, 1, 2};

/* The strobe pins again, for the ISR. Not const, for the same reason as s_strobe_ctx. */
static int s_strobe_pin[ARTS_SYNC_MAX_CAMERAS] = {
    ARTS_SYNC_STROBE_GPIO_1,
    ARTS_SYNC_STROBE_GPIO_2,
    ARTS_SYNC_STROBE_GPIO_3,
};

/* Strobe edges that arrived after this record already had a start, and were ignored. */
static volatile uint32_t s_strobe_glitches;

/* Pin level of camera idx's strobe, read now. */
static inline uint32_t IRAM_ATTR strobe_level(int idx)
{
  const int g = s_strobe_pin[idx];
  return (g < 32) ? (REG_READ(GPIO_IN_REG) >> g) & 1u : (REG_READ(GPIO_IN1_REG) >> (g - 32)) & 1u;
}

/* Which edge is the START is decided from the PIN LEVEL read inside the ISR, not from the
 * driver's cap_edge. cap_edge comes from the MCPWM cap_status register, and on this board it
 * labelled a rising edge as falling on ~0.1 % of exposures (18 of 16 157 pulses, 2026-10-28,
 * found with CONFIG_ARTS_SYNC_EDGE_DIAG): the capture TIME was right and the pin read high,
 * but the record lost both its start and, by the rule below, its end. The level is safe to use
 * because it holds for the whole exposure (5 ms high) or the gap (28 ms low), against an ISR
 * latency of microseconds. Why the register is wrong is not established.
 *
 * Polarity is measured, not assumed: on BFS-U3-63S4C the opto-coupled Line 1 idles LOW
 * against the external pull-up and rises while exposing, so the START is the rising edge —
 * the opposite of what an open-collector output suggests. See the Kconfig help. */
static inline bool IRAM_ATTR strobe_is_start(int idx)
{
  return strobe_level(idx) != (uint32_t)ARTS_SYNC_STROBE_ACTIVE_LOW;
}

/* Absolute capture tick of the open record's start edge, per camera, so the width can be
 * differenced when the end edge lands. ISR-only, like s_open. */
static uint32_t s_open_start_ticks[ARTS_SYNC_MAX_CAMERAS];

/* ---------------------------------------------------------------- capture interrupt path */

static void IRAM_ATTR commit_open_record(void)
{
  if (!s_open_valid) {
    return;
  }
  const uint32_t next = (s_head + 1u) & RING_MASK;
  if (next == s_tail) {
    s_dropped++; /* drop the oldest-unread rather than stall the interrupt */
  } else {
    s_ring[s_head] = s_open;
    s_head = next;
  }
  s_open_valid = false;
}

static void IRAM_ATTR open_record(uint32_t reference_ticks)
{
  /* Commit the PREVIOUS pulse before starting this one: a record is only known to be
   * complete once the next reference edge has arrived. */
  commit_open_record();

  s_open.pulse_index = s_next_pulse_index++;
  s_open.trigger_ticks = reference_ticks;
  /* Unsigned subtraction is wrap-correct even though the counter itself wraps. */
  s_open.trigger_period_ticks =
      s_have_last_trigger ? (reference_ticks - s_last_trigger_ticks) : 0u;
  for (int i = 0; i < ARTS_SYNC_MAX_CAMERAS; ++i) {
    s_open.exposure_ticks[i] = ARTS_SYNC_NO_CAPTURE;
    s_open.exposure_width_ticks[i] = ARTS_SYNC_NO_CAPTURE;
    s_open_start_ticks[i] = 0u;
#ifdef CONFIG_ARTS_SYNC_EDGE_DIAG
    s_open.diag_n[i] = 0u;
#endif
  }
  s_last_trigger_ticks = reference_ticks;
  s_have_last_trigger = true;
  s_open_valid = true;
}

#if ARTS_SYNC_USE_LOOPBACK
static bool IRAM_ATTR on_loopback_capture(mcpwm_cap_channel_handle_t chan,
                                          const mcpwm_capture_event_data_t *edata,
                                          void *user_ctx)
{
  (void)chan;
  (void)user_ctx;
  open_record(edata->cap_value);
  return false; /* no task woken from here */
}
#endif

static bool IRAM_ATTR on_strobe_capture(mcpwm_cap_channel_handle_t chan,
                                        const mcpwm_capture_event_data_t *edata,
                                        void *user_ctx)
{
  (void)chan;
  const int idx = *(const int *)user_ctx;

  if (idx >= ARTS_SYNC_MAX_CAMERAS) {
    return false;
  }
  const bool is_start = strobe_is_start(idx);

#if !ARTS_SYNC_USE_LOOPBACK
  /* No loopback channel, so camera 0's own exposure start is the reference every other
   * camera is measured against. Inter-camera skew is unaffected; only the absolute
   * trigger-to-exposure delay is lost. Only the START edge may open a record — letting the
   * end edge do it would emit two records per pulse. */
  if (idx == 0) {
    if (is_start && s_open_valid && s_open.exposure_ticks[0] != ARTS_SYNC_NO_CAPTURE &&
        edata->cap_value - s_open_start_ticks[0] <
            s_cap_resolution_hz / (2u * ARTS_SYNC_RATE_HZ)) {
      s_strobe_glitches++; /* a second start within half a period is not a new pulse */
    } else if (is_start) {
      open_record(edata->cap_value);
      s_open.exposure_ticks[0] = 0u;
      s_open_start_ticks[0] = edata->cap_value;
    } else if (s_open_valid && s_open.exposure_ticks[0] != ARTS_SYNC_NO_CAPTURE) {
      s_open.exposure_width_ticks[0] = edata->cap_value - s_open_start_ticks[0];
    }
    return false;
  }
#endif

  if (!s_open_valid) {
    return false;
  }
#ifdef CONFIG_ARTS_SYNC_EDGE_DIAG
  {
    const uint8_t n = s_open.diag_n[idx];
    if (n < 4u) {
      /* The edge as the DRIVER reported it, so this keeps measuring the register. */
      s_open.diag_edge_neg[idx][n] = (edata->cap_edge == MCPWM_CAP_EDGE_NEG);
      s_open.diag_level[idx][n] = strobe_level(idx);
      s_open.diag_ticks[idx][n] = edata->cap_value - s_open.trigger_ticks;
    }
    s_open.diag_n[idx] = n + 1u;
  }
#endif
  if (is_start && s_open.exposure_ticks[idx] != ARTS_SYNC_NO_CAPTURE) {
    /* The FIRST start wins. Observed 2026-10-28: a rising edge on both channels at the same
     * tick, 81 us after the real start, which used to overwrite it — an electrical event
     * common to both strobe lines, not an exposure. */
    s_strobe_glitches++;
  } else if (is_start) {
    s_open.exposure_ticks[idx] = edata->cap_value - s_open.trigger_ticks;
    s_open_start_ticks[idx] = edata->cap_value;
  } else if (s_open.exposure_ticks[idx] != ARTS_SYNC_NO_CAPTURE) {
    /* An end edge with no start in this record belongs to the previous pulse's exposure
     * running past the next trigger, which at any sane exposure cannot happen; dropping it
     * is safer than attributing it to the wrong pulse. */
    s_open.exposure_width_ticks[idx] = edata->cap_value - s_open_start_ticks[idx];
  }
  return false;
}

/* ------------------------------------------------------------------------ trigger output */

/* Routes the generator's output signal to an additional pad. The GPIO matrix allows one
 * peripheral signal to reach several pads, which is what keeps the camera edges
 * simultaneous; driving separate pins from software would not. */
static esp_err_t fan_out_trigger(int gpio_num)
{
  const gpio_config_t io = {
      .pin_bit_mask = 1ULL << gpio_num,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "fan-out gpio %d", gpio_num);

  const uint32_t sig = mcpwm_periph_signals.groups[0].operators[0].generators[0].pwm_sig;
  esp_rom_gpio_connect_out_signal(gpio_num, sig, false, false);
  return ESP_OK;
}

esp_err_t arts_sync_start_trigger(void)
{
  mcpwm_timer_handle_t timer = NULL;
  const mcpwm_timer_config_t timer_cfg = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = ARTS_SYNC_TIMER_RESOLUTION_HZ,
      .period_ticks = ARTS_SYNC_TIMER_RESOLUTION_HZ / ARTS_SYNC_RATE_HZ,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &timer), TAG, "trigger timer");

  mcpwm_oper_handle_t oper = NULL;
  const mcpwm_operator_config_t oper_cfg = {.group_id = 0};
  ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &oper), TAG, "operator");
  ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(oper, timer), TAG, "connect timer");

  mcpwm_cmpr_handle_t cmpr = NULL;
  const mcpwm_comparator_config_t cmpr_cfg = {.flags = {.update_cmp_on_tez = true}};
  ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cmpr_cfg, &cmpr), TAG, "comparator");
  ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(cmpr, ARTS_SYNC_PULSE_WIDTH_US), TAG,
                      "compare value");

  mcpwm_gen_handle_t gen = NULL;
  const mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = ARTS_SYNC_TRIGGER_GPIO_PRIMARY};
  ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &gen_cfg, &gen), TAG, "generator");

  /* High at the start of every period, low at the comparator: a positive pulse of
   * ARTS_SYNC_PULSE_WIDTH_US. Rising edge is the trigger, matching the camera's
   * trigger_activation: RisingEdge. */
  ESP_RETURN_ON_ERROR(
      mcpwm_generator_set_action_on_timer_event(
          gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                            MCPWM_GEN_ACTION_HIGH)),
      TAG, "timer action");
  ESP_RETURN_ON_ERROR(
      mcpwm_generator_set_action_on_compare_event(
          gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr,
                                              MCPWM_GEN_ACTION_LOW)),
      TAG, "compare action");

  ESP_RETURN_ON_ERROR(fan_out_trigger(ARTS_SYNC_TRIGGER_GPIO_2), TAG, "fan-out 2");
  ESP_RETURN_ON_ERROR(fan_out_trigger(ARTS_SYNC_TRIGGER_GPIO_3), TAG, "fan-out 3");

  ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timer), TAG, "enable timer");
  ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP), TAG,
                      "start timer");

  ESP_LOGI(TAG, "trigger running: %d Hz, %d us pulse, pads %d/%d/%d", ARTS_SYNC_RATE_HZ,
           ARTS_SYNC_PULSE_WIDTH_US, ARTS_SYNC_TRIGGER_GPIO_PRIMARY, ARTS_SYNC_TRIGGER_GPIO_2,
           ARTS_SYNC_TRIGGER_GPIO_3);
  return ESP_OK;
}

/* ----------------------------------------------------------------------- capture channels */

static esp_err_t add_capture_channel(int gpio_num, bool pos_edge, bool neg_edge,
                                     mcpwm_capture_event_cb_t cb, void *ctx)
{
  mcpwm_cap_channel_handle_t chan = NULL;
  const mcpwm_capture_channel_config_t cfg = {
      .gpio_num = gpio_num,
      .prescale = 1,
      .flags = {.pos_edge = pos_edge, .neg_edge = neg_edge},
  };
  ESP_RETURN_ON_ERROR(mcpwm_new_capture_channel(s_cap_timer, &cfg, &chan), TAG,
                      "capture channel gpio %d", gpio_num);

  const mcpwm_capture_event_callbacks_t cbs = {.on_cap = cb};
  ESP_RETURN_ON_ERROR(mcpwm_capture_channel_register_event_callbacks(chan, &cbs, ctx), TAG,
                      "capture callback gpio %d", gpio_num);
  ESP_RETURN_ON_ERROR(mcpwm_capture_channel_enable(chan), TAG, "enable capture gpio %d",
                      gpio_num);
  return ESP_OK;
}

esp_err_t arts_sync_start_capture(void)
{
  const mcpwm_capture_timer_config_t cap_cfg = {
      .group_id = 0,
      .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
  };
  ESP_RETURN_ON_ERROR(mcpwm_new_capture_timer(&cap_cfg, &s_cap_timer), TAG, "capture timer");

  if (mcpwm_capture_timer_get_resolution(s_cap_timer, &s_cap_resolution_hz) != ESP_OK ||
      s_cap_resolution_hz == 0u) {
    /* Fallback for ESP-IDF without the getter (pre-5.1). On the plain ESP32,
     * MCPWM_CAPTURE_CLK_SRC_DEFAULT is APB = 80 MHz, i.e. 12.5 ns per tick and a 32-bit
     * counter wrap every 53.7 s. Do NOT "correct" this to the 160 MHz figure that applies to
     * the MCPWM *timer* clock or to the S3's capture source: the constant scales
     * tick_period_ns, so a wrong value silently doubles or halves every published skew
     * without any other symptom. This is why the resolution is read back at all. */
    s_cap_resolution_hz = 80000000u;
    ESP_LOGW(TAG, "capture resolution not reported; assuming %" PRIu32 " Hz",
             s_cap_resolution_hz);
  }

#if ARTS_SYNC_USE_LOOPBACK
  /* Rising edge, matching the generator's active-high pulse. */
  ESP_RETURN_ON_ERROR(
      add_capture_channel(ARTS_SYNC_LOOPBACK_GPIO, true, false, on_loopback_capture, NULL),
      TAG, "loopback channel");
#endif

  for (int i = 0; i < ARTS_SYNC_CAMERA_COUNT; ++i) {
    /* BOTH edges. The extra edge costs one more interrupt per frame per camera — 30 a
     * second, nothing — and buys the exposure width, which is what turns the polarity
     * setting from an assumption into something the output itself contradicts when wrong. */
    ESP_RETURN_ON_ERROR(
        add_capture_channel(s_strobe_gpio[i], true, true, on_strobe_capture, &s_strobe_ctx[i]),
        TAG, "strobe channel %d", i);
  }

  ESP_RETURN_ON_ERROR(mcpwm_capture_timer_enable(s_cap_timer), TAG, "enable capture timer");
  ESP_RETURN_ON_ERROR(mcpwm_capture_timer_start(s_cap_timer), TAG, "start capture timer");

  ESP_LOGI(TAG, "capture running: %" PRIu32 " Hz (%.3f ns/tick), %d camera(s), loopback %s",
           s_cap_resolution_hz, 1e9 / (double)s_cap_resolution_hz, ARTS_SYNC_CAMERA_COUNT,
           ARTS_SYNC_USE_LOOPBACK ? "on" : "off");
  return ESP_OK;
}

uint32_t arts_sync_capture_resolution_hz(void) { return s_cap_resolution_hz; }

bool arts_sync_pop_record(arts_sync_record_t *out)
{
  if (s_tail == s_head) {
    return false;
  }
  *out = s_ring[s_tail];
  s_tail = (s_tail + 1u) & RING_MASK;
  return true;
}

uint32_t arts_sync_dropped_records(void) { return s_dropped; }

uint32_t arts_sync_strobe_glitches(void) { return s_strobe_glitches; }

/* ------------------------------------------------------------------------------ blink mode */

void arts_sync_run_blink(void)
{
  const gpio_config_t io = {
      .pin_bit_mask = 1ULL << ARTS_SYNC_BLINK_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io));

  const TickType_t half = pdMS_TO_TICKS(500 / ARTS_SYNC_BLINK_RATE_HZ);
  ESP_LOGI(TAG, "blink mode: GPIO %d at %d Hz, 50%% duty. No camera should be connected.",
           ARTS_SYNC_BLINK_GPIO, ARTS_SYNC_BLINK_RATE_HZ);

  int level = 0;
  while (true) {
    level = !level;
    gpio_set_level(ARTS_SYNC_BLINK_GPIO, level);
    vTaskDelay(half);
  }
}
