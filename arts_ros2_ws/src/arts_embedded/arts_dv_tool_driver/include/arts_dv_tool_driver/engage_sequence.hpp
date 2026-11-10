// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0
//
// EngageSequence: the instrument engagement and homing of one adapter's five disc servos.
// A port, AS IS, of collaborator A's tool_homing_node (intake commit 5f25078), whose
// current-capped position scheme comes from the adapter's designer: same operating modes,
// goal currents, profile velocities, stall rules, motor order and centring. Decided by the PI
// 2026-11-02 (docs/modules/dv_tool_driver.md, "Homing"). The steps:
//
//   goToStart    extended position mode (4), profile velocity 150, all discs to A's start
//                counts {100, 5000, 2100, 1700, 1800}; 5 s; no instrument fitted.
//   spin         current-based position mode (5), profile velocity 22, goal current 45 (disc 1)
//                / 50; discs 1, 4, 5, 2 in turn spun +20000 counts until each stalls (the
//                disc has dropped into the instrument and reached a stop); then torque off.
//   findLimits   (>= 5 s after spin) mode 5, profile velocity 30, goal current 100: disc 1 to
//                both stops (85 mA), centred; disc 4 (67 mA, profile velocity 20), centred;
//                then the jaw: disc 5 out (60 mA cap, 48 mA stall, profile velocity 15, disc 2
//                in freedrive), back at 150 mA pushing disc 2 across, both centred. Then
//                velocity mode (1), goal current 1193. The centred counts are the joint zero.
//   returnToStart  mode 5, profile velocity 30, goal current 200, discs back to the stored
//                (centred) counts, then velocity mode.
//   stop         torque off; final until the node restarts (A's emergency stop).
//
// Departures, none of which changes what the servos do: A ran all of this from one joystick
// button (spin, then limits after 5 s, automatically); here each moving step is its own
// request so the operator gives a go before each. A mode write that would not change the mode
// is skipped by the bus (it is an EEPROM write). Every bus write is checked; a failed one turns
// the torque off (A did not check them).
//
// TIMING: A's stall rules are per tick of its homing loop ("moved < 4 counts (jaw: 3) since the
// last tick, or current above the threshold, for 4 ticks"). A's timer is 10 ms, but each tick
// makes about a dozen single serial transactions, so A's real tick is that callback's
// duration. The thresholds sit close to the per-tick motion: the spin (profile velocity 22)
// moves 3.4 counts per 10 ms. In test/engage_sequence_test.cpp's simulated adapter, ticks of
// 8 ms or less lock discs in short of their stops and 10 ms mis-centres disc 4, while 10.5 to
// 100 ms home correctly. tick() must therefore run at A's effective period
// (engage_tick_period, measured on the bus by test/a_tick_period_probe.py: 12.0 ms on both
// buses, 2026-11-02), never at the node's 5 ms bus loop. tick() also ignores a call that
// comes less than 0.9 periods after the last tick it acted on: on 2026-11-02 the node's loop,
// held up by a request's two 50 ms pauses, caught up with back-to-back ticks and all four
// discs "locked in" within 17 ms without turning (torque then off; nothing moved). The loop
// no longer catches up either.
//
// ON THE ADAPTER (2026-11-02) THE SPIN DOES NOT WORK: with no instrument fitted, a free disc
// "locks in" after 4 ticks, because from a standing start it moves under 4 counts per tick and
// draws more than the stall current while accelerating. Collaborator A engages instruments
// manually with the gamepad; this routine is unproven (docs/modules/dv_tool_driver.md).

#ifndef ARTS_DV_TOOL_DRIVER__ENGAGE_SEQUENCE_HPP_
#define ARTS_DV_TOOL_DRIVER__ENGAGE_SEQUENCE_HPP_

#include <array>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <string>

namespace arts_dv_tool_driver
{

constexpr int kDiscs = 5;
using Counts = std::array<int32_t, kDiscs>;
using Currents = std::array<int16_t, kDiscs>;

constexpr uint8_t kModeVelocity = 1;
constexpr uint8_t kModeExtendedPosition = 4;
constexpr uint8_t kModeCurrentPosition = 5;
constexpr uint16_t kGoalCurrentMax = 1193;  // XC330-M288 current limit (A's value)

// The bus operations the sequence needs. Disc index 0..4 = servo ID 1..5.
class DiscBus
{
public:
  virtual ~DiscBus() = default;
  virtual bool torqueAll(bool on) = 0;
  virtual bool torque(int disc, bool on) = 0;
  virtual bool operatingMode(uint8_t mode) = 0;     // all servos (EEPROM)
  virtual bool profileVelocity(uint32_t units) = 0;  // all servos, 0.229 rev/min
  virtual bool goalCurrent(int disc, uint16_t units) = 0;
  virtual bool goalPositions(const Counts & counts) = 0;  // all servos
  virtual bool zeroVelocities() = 0;
  virtual void pause(int ms) = 0;
};

enum class Phase
{
  kVelocity,      // velocity mode, not engaged (the node's start-up state)
  kGoingToStart,
  kWaitForTool,   // at A's start counts, holding: fit the instrument, then spin
  kSpin,
  kSpinDone,      // all four discs engaged, torque off: findLimits after 5 s
  kLimits,
  kHomed,         // velocity mode, zero at the centred counts
  kReturning,
  kStopped,
  kFailed
};

inline const char * phaseName(Phase p)
{
  switch (p) {
    case Phase::kVelocity: return "velocity";
    case Phase::kGoingToStart: return "going_to_start";
    case Phase::kWaitForTool: return "wait_for_tool";
    case Phase::kSpin: return "spin";
    case Phase::kSpinDone: return "spin_done";
    case Phase::kLimits: return "limits";
    case Phase::kHomed: return "homed";
    case Phase::kReturning: return "returning";
    case Phase::kStopped: return "stopped";
    case Phase::kFailed: return "failed";
  }
  return "?";
}

class EngageSequence
{
public:
  using Log = std::function<void(const std::string &)>;

  // tick_period: A's loop period [s] that tick() is meant to be called at.
  EngageSequence(DiscBus & bus, double tick_period, Log log)
  : bus_(bus), tick_period_(tick_period), log_(std::move(log)) {}

  Phase phase() const {return phase_;}
  bool acceptsVelocity() const {return phase_ == Phase::kVelocity || phase_ == Phase::kHomed;}
  // The centred counts once homed (the joint zero).
  const Counts & homedCounts() const {return homed_;}

  // Each request returns "" if accepted, else why not.
  std::string goToStart(double t)
  {
    if (phase_ != Phase::kVelocity) {return refuse("go_to_start", "velocity");}
    if (!(bus_.torqueAll(false) && bus_.operatingMode(kModeExtendedPosition) &&
      bus_.torqueAll(true) && bus_.profileVelocity(150) && bus_.goalPositions(kStartCounts)))
    {
      return fail("go_to_start");
    }
    t0_ = t;
    enter(Phase::kGoingToStart, "moving to the start counts for 5 s");
    return "";
  }

  std::string spin(const Counts & pos)
  {
    if (phase_ != Phase::kWaitForTool) {return refuse("spin", "wait_for_tool");}
    if (!(modeSwitch(kModeCurrentPosition) && bus_.profileVelocity(22))) {return fail("spin");}
    for (int i = 0; i < kDiscs; ++i) {
      if (!bus_.goalCurrent(i, i == 0 ? 45 : 50)) {return fail("spin");}
    }
    motor_ = 0;
    stalls_ = 0;
    locked_ = 0;
    init_ = false;
    prev_ = pos[motor_];
    enter(Phase::kSpin, "spinning disc 1");
    return "";
  }

  std::string findLimits(const Counts & pos, double t)
  {
    if (phase_ != Phase::kSpinDone) {return refuse("find_limits", "spin_done");}
    if (t - t0_ < 5.0) {return "find_limits: A's 5 s settle after the spin has not elapsed";}
    if (!(modeSwitch(kModeCurrentPosition) && bus_.profileVelocity(30))) {
      return fail("find_limits");
    }
    for (int i = 0; i < kDiscs; ++i) {
      if (!bus_.goalCurrent(i, 100)) {return fail("find_limits");}
    }
    start_ = pos;
    motor_ = 0;
    dir_ = -1;
    step_ = 0;
    stalls_ = 0;
    init_ = false;
    prev_ = pos[motor_];
    enter(Phase::kLimits, "finding the limits of disc 1");
    return "";
  }

  std::string returnToStart()
  {
    if (phase_ != Phase::kHomed) {return refuse("return_to_start", "homed");}
    if (!(modeSwitch(kModeCurrentPosition) && bus_.profileVelocity(30))) {
      return fail("return_to_start");
    }
    for (int i = 0; i < kDiscs; ++i) {
      if (!bus_.goalCurrent(i, 200)) {return fail("return_to_start");}
    }
    if (!bus_.goalPositions(start_)) {return fail("return_to_start");}
    enter(Phase::kReturning, "returning the discs to the centred counts");
    return "";
  }

  void stop()
  {
    bus_.torqueAll(false);
    enter(Phase::kStopped, "stopped: torque off (restart the node to continue)");
  }

  // Called once per engage tick (A's effective loop period) with fresh reads.
  void tick(const Counts & pos, const Currents & cur, double t)
  {
    if (t - last_tick_ < 0.9 * tick_period_) {return;}
    last_tick_ = t;
    switch (phase_) {
      case Phase::kGoingToStart:
        if (t - t0_ >= 5.0) {
          start_ = pos;
          enter(Phase::kWaitForTool, "at the start counts: fit the instrument, then spin");
        }
        break;
      case Phase::kSpin: tickSpin(pos, cur, t); break;
      case Phase::kLimits:
        if (motor_ == 4) {tickJaw(pos, cur);} else {tickAxis(pos, cur);}
        break;
      case Phase::kReturning: {
          bool reached = true;
          for (int i = 0; i < kDiscs; ++i) {
            if (std::abs(pos[i] - start_[i]) > 25) {reached = false;}
          }
          if (reached) {
            if (!toVelocity()) {fail("return_to_start"); return;}
            enter(Phase::kHomed, "back at the centred counts, velocity mode");
          }
          break;
        }
      default: break;
    }
  }

private:
  static constexpr Counts kStartCounts{100, 5000, 2100, 1700, 1800};

  void enter(Phase p, const std::string & what)
  {
    phase_ = p;
    log_(std::string(phaseName(p)) + ": " + what);
  }
  std::string refuse(const char * req, const char * needed)
  {
    return std::string(req) + ": refused in phase " + phaseName(phase_) + " (needs " + needed +
           ")";
  }
  std::string fail(const char * req)
  {
    bus_.torqueAll(false);
    enter(Phase::kFailed, std::string(req) + ": a bus write failed, torque off");
    return std::string(req) + ": a bus write failed, torque off";
  }
  // A's mode switch: torque off, 50 ms, mode, 50 ms, torque on.
  bool modeSwitch(uint8_t mode)
  {
    if (!bus_.torqueAll(false)) {return false;}
    bus_.pause(50);
    if (!bus_.operatingMode(mode)) {return false;}
    bus_.pause(50);
    return bus_.torqueAll(true);
  }
  // A's return to velocity mode, goal current 1193. The goal velocities are zeroed before the
  // torque comes back (A left them to its next joystick message).
  bool toVelocity()
  {
    if (!bus_.torqueAll(false)) {return false;}
    bus_.pause(50);
    if (!bus_.operatingMode(kModeVelocity)) {return false;}
    bus_.pause(50);
    if (!bus_.zeroVelocities() || !bus_.torqueAll(true)) {return false;}
    for (int i = 0; i < kDiscs; ++i) {
      if (!bus_.goalCurrent(i, kGoalCurrentMax)) {return false;}
    }
    return true;
  }

  void tickSpin(const Counts & pos, const Currents & cur, double t)
  {
    const int32_t p = pos[motor_];
    if (!init_) {
      Counts target = start_;
      target[motor_] = p + 20000;
      if (!bus_.goalPositions(target)) {fail("spin"); return;}
      init_ = true;
    }
    const int limit = motor_ == 0 ? 40 : 45;
    if (std::abs(p - prev_) < 4 || std::abs(cur[motor_]) > limit) {++stalls_;} else {stalls_ = 0;}
    prev_ = p;
    if (stalls_ <= 3) {return;}
    start_[motor_] = p;
    log_("disc " + std::to_string(motor_ + 1) + " locked in at " + std::to_string(p));
    if (++locked_ >= 4) {
      if (!bus_.torqueAll(false)) {fail("spin"); return;}
      t0_ = t;
      enter(Phase::kSpinDone, "all four discs engaged, torque off; find_limits after 5 s");
      return;
    }
    motor_ = motor_ == 0 ? 3 : motor_ == 3 ? 4 : 1;  // A's order: 1, 4, 5, 2
    stalls_ = 0;
    init_ = false;
    prev_ = pos[motor_];
    log_("spinning disc " + std::to_string(motor_ + 1));
  }

  // Discs 1 (roll) and 4 (pitch): both stops, then the midpoint.
  void tickAxis(const Counts & pos, const Currents & cur)
  {
    const int32_t p = pos[motor_];
    const int id = motor_ + 1;
    if (step_ < 2) {
      if (!init_) {
        start_[motor_] = p + 20000 * dir_;
        if (!bus_.goalPositions(start_)) {fail("find_limits"); return;}
        init_ = true;
      }
      const int thresh = id == 4 ? 67 : 85;
      if (std::abs(p - prev_) < 4 || std::abs(cur[motor_]) > thresh) {++stalls_;} else {
        stalls_ = 0;
      }
      prev_ = p;
      if (stalls_ > 3) {
        if (step_ == 0) {
          limit1_ = p;
          log_("disc " + std::to_string(id) + " limit 1 at " + std::to_string(p));
          dir_ = -dir_;
          step_ = 1;
          stalls_ = -10;
        } else {
          limit2_ = p;
          log_("disc " + std::to_string(id) + " limit 2 at " + std::to_string(p));
          step_ = 2;
          stalls_ = -5;
        }
        init_ = false;
      }
      return;
    }
    const int32_t park = (limit1_ + limit2_) / 2;
    if (!init_) {
      log_("centring disc " + std::to_string(id) + " at " + std::to_string(park));
      start_[motor_] = park;
      if (!bus_.goalPositions(start_)) {fail("find_limits"); return;}
      init_ = true;
    }
    if (std::abs(park - p) >= 15) {stalls_ = 0; return;}
    if (++stalls_ <= 3) {return;}
    if (!bus_.goalCurrent(motor_, kGoalCurrentMax)) {fail("find_limits"); return;}
    if (motor_ == 0) {
      motor_ = 3;
      dir_ = -1;
      step_ = 0;
      stalls_ = 0;
      init_ = false;
      prev_ = pos[motor_];
      if (!bus_.profileVelocity(20)) {fail("find_limits"); return;}
      log_("disc 1 homed; finding the limits of disc 4");
    } else {
      motor_ = 4;
      dir_ = 1;
      step_ = 0;
      stalls_ = 0;
      init_ = false;
      if (!bus_.profileVelocity(15) || !bus_.goalCurrent(4, 60) || !bus_.torque(1, false)) {
        fail("find_limits");
        return;
      }
      prev_ = pos[4];
      log_("disc 4 homed; jaw: disc 5 out, disc 2 in freedrive");
    }
  }

  // The jaw: disc 5 out, then back at higher current pushing disc 2, then both centred.
  void tickJaw(const Counts & pos, const Currents & cur)
  {
    const int32_t p = pos[4];
    if (step_ < 2) {
      if (!init_) {
        start_[4] = p + 20000 * dir_;
        if (!bus_.goalPositions(start_)) {fail("find_limits"); return;}
        init_ = true;
      }
      const int stall_current = step_ == 0 ? 48 : 100;
      if (std::abs(p - prev_) < 3 || std::abs(cur[4]) > stall_current) {++stalls_;} else {
        stalls_ = 0;
      }
      prev_ = p;
      if (stalls_ <= (step_ == 0 ? 3 : 6)) {return;}
      if (step_ == 0) {
        limit1_ = p;
        log_("disc 5 limit 1 (out) at " + std::to_string(p));
        dir_ = -1;
        step_ = 1;
        stalls_ = -10;
        init_ = false;
        if (!bus_.goalCurrent(4, 150)) {fail("find_limits"); return;}
      } else {
        limit2_ = p;
        limit2_disc2_ = pos[1];
        log_("disc 5 limit 2 (pushed across) at " + std::to_string(p) + ", disc 2 pushed to " +
          std::to_string(limit2_disc2_));
        step_ = 2;
        stalls_ = -5;
        init_ = false;
      }
      return;
    }
    const int32_t park = (limit1_ + limit2_) / 2;
    const int32_t park2 = limit2_disc2_ - (park - limit2_);
    if (!init_) {
      log_("centring the jaw: disc 5 at " + std::to_string(park) + ", disc 2 at " +
        std::to_string(park2));
      if (!bus_.torque(1, true) || !bus_.goalCurrent(1, 60)) {fail("find_limits"); return;}
      start_[4] = park;
      start_[1] = park2;
      if (!bus_.goalPositions(start_)) {fail("find_limits"); return;}
      init_ = true;
    }
    if (std::abs(park - p) >= 45) {stalls_ = 0; return;}
    if (++stalls_ <= 3) {return;}
    homed_ = pos;
    if (!toVelocity()) {fail("find_limits"); return;}
    enter(Phase::kHomed, "jaw centred: homing complete, velocity mode");
  }

  DiscBus & bus_;
  double tick_period_;
  double last_tick_{-1e9};
  Log log_;
  Phase phase_{Phase::kVelocity};
  Counts start_{};
  Counts homed_{};
  double t0_{0.0};
  int motor_{0};
  int dir_{-1};
  int step_{0};
  int stalls_{0};
  int locked_{0};
  bool init_{false};
  int32_t prev_{0};
  int32_t limit1_{0};
  int32_t limit2_{0};
  int32_t limit2_disc2_{0};
};

}  // namespace arts_dv_tool_driver

#endif  // ARTS_DV_TOOL_DRIVER__ENGAGE_SEQUENCE_HPP_
