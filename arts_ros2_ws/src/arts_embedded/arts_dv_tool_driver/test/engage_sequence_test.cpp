// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0
//
// EngageSequence against a simulated adapter. Each simulated disc turns at the profile
// velocity toward its goal; once the instrument is fitted, discs 1, 2, 4 and 5 meet stops, and
// a disc held at a stop draws its goal current (the current-based position mode's cap). Disc 5
// can go below its own lower stop only by pushing disc 2 (in freedrive) with at least 120 mA,
// which is the jaw sweep the sequence relies on. This checks the port's logic, not the real
// instrument: the stops, ranges and push current here are invented.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "arts_dv_tool_driver/engage_sequence.hpp"

using arts_dv_tool_driver::Counts;
using arts_dv_tool_driver::Currents;
using arts_dv_tool_driver::EngageSequence;
using arts_dv_tool_driver::Phase;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Disc
{
  double pos{0.0};
  double goal{0.0};
  double lo{-kInf};
  double hi{kInf};
  bool torque{false};
  uint16_t goal_current{1193};
  double current{0.0};
};

class FakeAdapter : public arts_dv_tool_driver::DiscBus
{
public:
  std::vector<Disc> d = std::vector<Disc>(5);
  uint8_t mode{1};
  uint32_t profile{0};
  std::vector<uint8_t> modes_written;
  double jaw_push_lo{-kInf};   // disc 5's limit when pushing disc 2
  int fail_after{-1};          // fail the n-th write from now (0 = the next one)
  double peak_current_mode5{0.0};

  bool torqueAll(bool on) override
  {
    for (int i = 0; i < 5; ++i) {if (!torque(i, on)) {return false;}}
    return true;
  }
  bool torque(int i, bool on) override
  {
    if (!ok()) {return false;}
    d[i].torque = on;
    if (on) {d[i].goal = d[i].pos;}  // X series: torque on holds the present position
    return true;
  }
  bool operatingMode(uint8_t m) override
  {
    if (!ok()) {return false;}
    mode = m;
    modes_written.push_back(m);
    return true;
  }
  bool profileVelocity(uint32_t u) override {profile = u; return ok();}
  bool goalCurrent(int i, uint16_t u) override {d[i].goal_current = u; return ok();}
  bool goalPositions(const Counts & c) override
  {
    if (!ok()) {return false;}
    for (int i = 0; i < 5; ++i) {d[i].goal = c[i];}
    return true;
  }
  bool zeroVelocities() override {return ok();}
  void pause(int) override {}

  // The instrument drops on: each disc turns freely `free` counts before engaging its stops.
  void fitInstrument()
  {
    const double free[5] = {1500, 900, 0, 2200, 1200};
    const double range[5] = {6000, 3000, 0, 2600, 1800};
    for (int i : {0, 1, 3, 4}) {
      d[i].hi = d[i].pos + free[i];
      d[i].lo = d[i].hi - range[i];
    }
    jaw_push_lo = d[4].lo - 700;
  }

  void step(double dt)
  {
    const double speed = profile * 0.229 / 60.0 * 4096.0;  // counts/s
    for (int i = 0; i < 5; ++i) {
      Disc & x = d[i];
      x.current = 0.0;
      if (!x.torque || mode == 1) {continue;}
      const double cap = mode == 5 ? x.goal_current : 1193.0;
      double lo = x.lo;
      // Disc 5 pushes disc 2 (in freedrive) across only with enough current.
      if (i == 4 && cap >= 120.0 && !d[1].torque) {lo = jaw_push_lo;}
      const double want = x.goal - x.pos;
      const double move = std::clamp(want, -speed * dt, speed * dt);
      const double next = std::clamp(x.pos + move, lo, x.hi);
      const bool blocked = std::abs(next - (x.pos + move)) > 1e-9;
      if (i == 4) {  // disc 2 moves by the change in disc 5's travel below its own stop
        d[1].pos += std::min(next - d[4].lo, 0.0) - std::min(x.pos - d[4].lo, 0.0);
      }
      x.pos = next;
      x.current = blocked ? cap : (std::abs(move) > 0.5 ? 10.0 : 3.0);
      if (mode == 5) {peak_current_mode5 = std::max(peak_current_mode5, x.current);}
    }
  }
  Counts counts() const
  {
    Counts c{};
    for (int i = 0; i < 5; ++i) {c[i] = static_cast<int32_t>(std::lround(d[i].pos));}
    return c;
  }
  Currents currents() const
  {
    Currents c{};
    for (int i = 0; i < 5; ++i) {c[i] = static_cast<int16_t>(std::lround(d[i].current));}
    return c;
  }

private:
  bool ok()
  {
    if (fail_after < 0) {return true;}
    return fail_after-- != 0;
  }
};

struct Rig
{
  explicit Rig(double period)
  : period(period), seq(bus, period, [this](const std::string & m) {log.push_back(m);}) {}
  // Ticks at the given period until the phase is `until` (or `limit_s` of sim time pass).
  bool runUntil(Phase until, double limit_s = 600.0)
  {
    for (double end = t + limit_s; t < end; ) {
      t += period;
      bus.step(period);
      seq.tick(bus.counts(), bus.currents(), t);
      if (seq.phase() == until) {return true;}
    }
    return false;
  }
  void wait(double s) {runUntil(Phase::kFailed, s);}

  double period;
  double t{100.0};
  FakeAdapter bus;
  std::vector<std::string> log;
  EngageSequence seq;
};

// A's effective tick, for the simulation: a dozen ~2 ms serial transactions.
constexpr double kAPeriod = 0.024;
}  // namespace

TEST(EngageSequence, FullEngagementCentresEveryAxis)
{
  Rig r(kAPeriod);
  ASSERT_EQ(r.seq.goToStart(r.t), "");
  ASSERT_TRUE(r.runUntil(Phase::kWaitForTool));
  r.bus.fitInstrument();
  ASSERT_EQ(r.seq.spin(r.bus.counts()), "");
  ASSERT_TRUE(r.runUntil(Phase::kSpinDone));
  // Each engaged disc locked in AT its upper stop, not short of it.
  for (int i : {0, 1, 3, 4}) {
    EXPECT_NEAR(r.bus.d[i].pos, r.bus.d[i].hi, 1.0) << "disc " << i + 1;
  }
  r.wait(5.0);
  ASSERT_EQ(r.seq.findLimits(r.bus.counts(), r.t), "");
  ASSERT_TRUE(r.runUntil(Phase::kHomed));
  EXPECT_NEAR(r.bus.d[0].pos, (r.bus.d[0].lo + r.bus.d[0].hi) / 2, 15.0);
  EXPECT_NEAR(r.bus.d[3].pos, (r.bus.d[3].lo + r.bus.d[3].hi) / 2, 15.0);
  EXPECT_NEAR(r.bus.d[4].pos, (r.bus.jaw_push_lo + r.bus.d[4].hi) / 2, 45.0);
  EXPECT_EQ(r.bus.mode, arts_dv_tool_driver::kModeVelocity);
  // A's mode sequence: extended position, current position (spin), again (limits), velocity.
  EXPECT_EQ(r.bus.modes_written, (std::vector<uint8_t>{4, 5, 5, 1}));
  // Never above A's highest homing cap (150 mA) while current-capped.
  EXPECT_LE(r.bus.peak_current_mode5, 150.0);
  // The joint zero is the centred position.
  EXPECT_EQ(r.seq.homedCounts()[0], r.bus.counts()[0]);

  // Return to the centred counts after moving disc 1 away.
  r.bus.d[0].pos += 400;
  ASSERT_EQ(r.seq.returnToStart(), "");
  ASSERT_TRUE(r.runUntil(Phase::kHomed));
  EXPECT_NEAR(r.bus.d[0].pos, (r.bus.d[0].lo + r.bus.d[0].hi) / 2, 25.0);
}

TEST(EngageSequence, StepsAreRefusedOutOfOrder)
{
  Rig r(kAPeriod);
  EXPECT_NE(r.seq.spin(r.bus.counts()), "");
  EXPECT_NE(r.seq.findLimits(r.bus.counts(), r.t), "");
  EXPECT_NE(r.seq.returnToStart(), "");
  ASSERT_EQ(r.seq.goToStart(r.t), "");
  ASSERT_TRUE(r.runUntil(Phase::kWaitForTool));
  r.bus.fitInstrument();
  ASSERT_EQ(r.seq.spin(r.bus.counts()), "");
  ASSERT_TRUE(r.runUntil(Phase::kSpinDone));
  EXPECT_NE(r.seq.findLimits(r.bus.counts(), r.t), "") << "A's 5 s settle not enforced";
  EXPECT_TRUE(r.bus.modes_written == (std::vector<uint8_t>{4, 5}));
}

// The timing hazard (engage_sequence.hpp, TIMING): ticking at the node's own 5 ms bus loop,
// the spin moves 1.7 counts per tick, under A's 4-count stall rule, so disc 1 "locks in"
// without reaching its stop.
TEST(EngageSequence, TickingAtTheBusLoopRateLocksInFalsely)
{
  Rig r(0.005);
  ASSERT_EQ(r.seq.goToStart(r.t), "");
  ASSERT_TRUE(r.runUntil(Phase::kWaitForTool));
  r.bus.fitInstrument();
  ASSERT_EQ(r.seq.spin(r.bus.counts()), "");
  ASSERT_TRUE(r.runUntil(Phase::kSpinDone));
  EXPECT_GT(r.bus.d[0].hi - r.bus.d[0].pos, 1000.0);
}

// The 2026-11-02 adapter failure: ticks delivered back-to-back (a loop catching up) must not
// count as stalls. Each tick period here is followed by four extra calls 1 ms apart.
TEST(EngageSequence, BurstTicksAreIgnored)
{
  Rig r(kAPeriod);
  ASSERT_EQ(r.seq.goToStart(r.t), "");
  ASSERT_TRUE(r.runUntil(Phase::kWaitForTool));
  r.bus.fitInstrument();
  ASSERT_EQ(r.seq.spin(r.bus.counts()), "");
  for (int k = 0; k < 20000 && r.seq.phase() != Phase::kSpinDone; ++k) {
    r.t += r.period;
    r.bus.step(r.period);
    for (int b = 0; b < 5; ++b) {r.seq.tick(r.bus.counts(), r.bus.currents(), r.t + 0.001 * b);}
  }
  ASSERT_EQ(r.seq.phase(), Phase::kSpinDone);
  for (int i : {0, 1, 3, 4}) {
    EXPECT_NEAR(r.bus.d[i].pos, r.bus.d[i].hi, 1.0) << "disc " << i + 1;
  }
}

TEST(EngageSequence, AFailedWriteTurnsTheTorqueOff)
{
  Rig r(kAPeriod);
  ASSERT_EQ(r.seq.goToStart(r.t), "");
  ASSERT_TRUE(r.runUntil(Phase::kWaitForTool));
  r.bus.fitInstrument();
  r.bus.fail_after = 3;  // a write inside spin()'s mode switch / setup fails
  EXPECT_NE(r.seq.spin(r.bus.counts()), "");
  EXPECT_EQ(r.seq.phase(), Phase::kFailed);
  for (const auto & x : r.bus.d) {EXPECT_FALSE(x.torque);}
  EXPECT_NE(r.seq.goToStart(r.t), "");
}

TEST(EngageSequence, StopIsFinal)
{
  Rig r(kAPeriod);
  ASSERT_EQ(r.seq.goToStart(r.t), "");
  r.seq.stop();
  for (const auto & x : r.bus.d) {EXPECT_FALSE(x.torque);}
  EXPECT_EQ(r.seq.phase(), Phase::kStopped);
  EXPECT_NE(r.seq.goToStart(r.t), "");
}
