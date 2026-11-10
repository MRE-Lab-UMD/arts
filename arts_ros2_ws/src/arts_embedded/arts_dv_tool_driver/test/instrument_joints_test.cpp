// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0
//
// InstrumentJoints with the large needle driver's values (config/instruments/
// 471006_large_needle_driver.yaml, copied here; keep in step with the file).

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "arts_dv_tool_driver/instrument_joints.hpp"

using arts_dv_tool_driver::InstrumentJoints;

namespace
{
const std::vector<std::string> kJoints{"roll", "wrist_pitch", "wrist_yaw", "jaw"};
const std::vector<double> kCoupling{
  1.07, 0, 0, 0,
  0, -0.59, -0.9, 0.45,
  0, 0.94, 0, 0,
  0, 0.59, 0.9, 0.45};
const std::vector<double> kMin{-4.7124, -1.3963, -1.7453, -0.1745};
const std::vector<double> kMax{4.7124, 1.3963, 1.7453, 1.5708};

InstrumentJoints lnd() {return InstrumentJoints(kJoints, kCoupling, kMin, kMax);}

// Drive the instrument for `seconds` at 12 ms steps with a constant joint velocity target;
// joints integrated through the discs (q from the integrated disc angles, as the node does).
Eigen::VectorXd drive(
  const InstrumentJoints & k, Eigen::VectorXd q, const Eigen::VectorXd & qd, double seconds,
  double * worst_blade = nullptr)
{
  const Eigen::Map<const Eigen::Matrix4d> C(kCoupling.data());  // column-major view: C^T
  Eigen::VectorXd d = C.transpose() * q;
  for (double t = 0; t < seconds; t += 0.012) {
    d += k.discVelocities(k.joints(d), qd) * 0.012;
    if (worst_blade) {
      const Eigen::VectorXd j = k.joints(d);
      *worst_blade = std::max({*worst_blade, std::abs(j[2] + j[3] / 2), std::abs(j[2] - j[3] / 2)});
    }
  }
  return k.joints(d);
}
}  // namespace

TEST(InstrumentJoints, DiscsAndJointsRoundTrip)
{
  const auto k = lnd();
  const Eigen::Map<const Eigen::Matrix4d> Ct(kCoupling.data());
  Eigen::Vector4d q(0.3, -0.2, 0.5, 0.4);
  EXPECT_LT((k.joints(Ct.transpose() * q) - q).norm(), 1e-12);
}

TEST(InstrumentJoints, PurePitchTurnsTheCouplingColumn)
{
  const auto k = lnd();
  const Eigen::Vector4d v = k.discVelocities(Eigen::Vector4d::Zero(), Eigen::Vector4d(0, 0.1, 0, 0));
  EXPECT_NEAR(v[0], 0.0, 1e-12);
  EXPECT_NEAR(v[1], -0.059, 1e-12);
  EXPECT_NEAR(v[2], 0.094, 1e-12);
  EXPECT_NEAR(v[3], 0.059, 1e-12);
}

TEST(InstrumentJoints, EachJointStopsAtItsLimit)
{
  const auto k = lnd();
  for (int j = 0; j < 4; ++j) {
    for (double s : {1.0, -1.0}) {
      Eigen::Vector4d q(0, 0, 0, 0.4);
      Eigen::Vector4d qd = Eigen::Vector4d::Zero();
      qd[j] = s * M_PI;  // A's full-scale jog
      const Eigen::VectorXd end = drive(k, q, qd, 10.0);
      const double lim = s > 0 ? kMax[j] : kMin[j];
      EXPECT_LE(s * end[j], s * lim + 1e-6) << kJoints[j] << " overshoots " << lim;
      // Every joint stays within its limits, even the coupled ones.
      for (int i = 0; i < 4; ++i) {
        EXPECT_GE(end[i], kMin[i] - 1e-6) << kJoints[j] << " drove " << kJoints[i];
        EXPECT_LE(end[i], kMax[i] + 1e-6) << kJoints[j] << " drove " << kJoints[i];
      }
    }
  }
}

TEST(InstrumentJoints, OpeningTheJawAtFullYawKeepsEachBladeInside)
{
  const auto k = lnd();
  double worst = 0.0;
  const Eigen::VectorXd end = drive(
    k, Eigen::Vector4d(0, 0, 1.70, 0.0), Eigen::Vector4d(0, 0, 0, 2.0), 5.0, &worst);
  EXPECT_LE(worst, kMax[2] + 1e-6);
  EXPECT_GT(end[3], 0.5);  // it still opens, by bending the other blade away
}

TEST(InstrumentJoints, BeyondALimitOnlyInwardMotionIsAllowed)
{
  EXPECT_EQ(InstrumentJoints::limitNear(1.0, 1.5, -1.0, 1.4), 0.0);
  EXPECT_LT(InstrumentJoints::limitNear(-1.0, 1.5, -1.0, 1.4), 0.0);
  EXPECT_EQ(InstrumentJoints::limitNear(-1.0, -1.5, -1.4, 1.0), 0.0);
}

// Grip cap: discs 2 and 5 (rows 1 and 3) carry the jaw; disc 1 (roll) and disc 4 (pitch) do not.
TEST(InstrumentJoints, JawRowsAreTheDiscsTheJawDrives)
{
  EXPECT_EQ(lnd().jawRows(), (std::vector<std::size_t>{1, 3}));
}

TEST(InstrumentJoints, ClosingJawIsHeldAtTheCurrentCap)
{
  const auto k = lnd();
  Eigen::VectorXd qd(4);
  qd << 0.3, 0.2, 0.1, -1.7;  // closing jaw, with roll/pitch/yaw moving too
  EXPECT_TRUE(k.capJawClosing(qd, {0, 150, 0, 20}, 150.0));   // disc 2 at the cap
  EXPECT_EQ(qd[3], 0.0);
  EXPECT_EQ(qd[0], 0.3);  // the other joints untouched
  EXPECT_EQ(qd[1], 0.2);
  EXPECT_EQ(qd[2], 0.1);
  qd[3] = -1.7;
  EXPECT_TRUE(k.capJawClosing(qd, {0, 10, 0, -160}, 150.0));  // either sign, disc 5
  EXPECT_EQ(qd[3], 0.0);
}

TEST(InstrumentJoints, CapLeavesOpeningBelowCapAndOtherDiscsAlone)
{
  const auto k = lnd();
  Eigen::VectorXd qd(4);
  qd << 0, 0, 0, 1.0;  // OPENING at a high current: never blocked
  EXPECT_FALSE(k.capJawClosing(qd, {0, 900, 0, 900}, 150.0));
  EXPECT_EQ(qd[3], 1.0);
  qd[3] = -1.0;  // closing below the cap
  EXPECT_FALSE(k.capJawClosing(qd, {0, 149, 0, 149}, 150.0));
  EXPECT_EQ(qd[3], -1.0);
  // a high current on roll or pitch discs does not cap the jaw
  EXPECT_FALSE(k.capJawClosing(qd, {900, 0, 900, 0}, 150.0));
  EXPECT_FALSE(k.capJawClosing(qd, {0, 900, 0, 900}, 0.0));  // cap 0 = off
  EXPECT_EQ(qd[3], -1.0);
}

// The unitless jaw command: 0..90 opening share, below 0 grip with a cap rising to full at -10.
TEST(InstrumentJoints, JawCommandMapsOpeningAndGrip)
{
  using IJ = InstrumentJoints;
  const double jmin = kMin[3], jmax = kMax[3];
  auto g = IJ::jawFromCommand(90, jmin, jmax, 1000, 150);
  EXPECT_NEAR(g.target, jmax, 1e-12);
  EXPECT_EQ(g.cap, 150);
  g = IJ::jawFromCommand(45, jmin, jmax, 1000, 150);
  EXPECT_NEAR(g.target, jmax / 2, 1e-12);
  g = IJ::jawFromCommand(0, jmin, jmax, 1000, 150);
  EXPECT_EQ(g.target, 0.0);
  EXPECT_EQ(g.cap, 150);  // just closed: touch force only
  g = IJ::jawFromCommand(-5, jmin, jmax, 1000, 150);
  EXPECT_NEAR(g.target, jmin / 2, 1e-12);
  EXPECT_EQ(g.cap, 500);
  g = IJ::jawFromCommand(-10, jmin, jmax, 1000, 150);
  EXPECT_NEAR(g.target, jmin, 1e-12);
  EXPECT_EQ(g.cap, 1000);
  g = IJ::jawFromCommand(-1, jmin, jmax, 1000, 150);
  EXPECT_EQ(g.cap, 150);  // never below the touch cap
}

TEST(InstrumentJoints, JawCommandIsClamped)
{
  using IJ = InstrumentJoints;
  auto g = IJ::jawFromCommand(-1000, kMin[3], kMax[3], 1000, 150);
  EXPECT_NEAR(g.target, kMin[3], 1e-12);
  EXPECT_EQ(g.cap, 1000);
  g = IJ::jawFromCommand(1000, kMin[3], kMax[3], 1000, 150);
  EXPECT_NEAR(g.target, kMax[3], 1e-12);
  g = IJ::jawFromCommand(std::nan(""), kMin[3], kMax[3], 1000, 150);
  EXPECT_EQ(g.target, 0.0);
}

TEST(InstrumentJoints, SingularCouplingIsRefused)
{
  std::vector<double> c = kCoupling;
  for (int i = 4; i < 8; ++i) {c[i] = c[i + 8];}  // row 2 = row 4
  EXPECT_THROW(InstrumentJoints(kJoints, c, kMin, kMax), std::runtime_error);
}
