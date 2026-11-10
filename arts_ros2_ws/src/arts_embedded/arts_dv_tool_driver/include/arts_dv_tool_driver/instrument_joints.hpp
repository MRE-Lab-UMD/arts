// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0
//
// InstrumentJoints: an instrument's joints from its disc angles and its disc velocities from
// joint velocity targets. Collaborator A's teleop path, value for value (tool_kin_mdl,
// instrument_arm_node):
//   disc angles since the zero = coupling * joints          (A's MotorCntToJang, solved)
//   disc velocities           = coupling * joint velocities (A's JvelToMotorVel)
// and joint velocity targets slowed near the limits by v <= 6 (limit - q) (A's
// LimitVelocityNearJointLimit, 6 /s), applied to every joint and, as A does for its two jaw
// joints, to each blade (wrist_yaw +- jaw/2) with the wrist_yaw limits.
// Grip cap (dVRK's jaw ftmax, by motor current): a jaw CLOSING target is held at 0 while any
// disc the jaw drives (nonzero jaw column of the coupling) draws at least the cap; opening and
// the other joints are never touched. Shut jaws commanded below 0 then push at about the cap
// (disc_drive: 100 per degree of jaw min below 0, so 1000 at -10 deg).

#ifndef ARTS_DV_TOOL_DRIVER__INSTRUMENT_JOINTS_HPP_
#define ARTS_DV_TOOL_DRIVER__INSTRUMENT_JOINTS_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace arts_dv_tool_driver
{

class InstrumentJoints
{
public:
  InstrumentJoints(
    const std::vector<std::string> & joints, const std::vector<double> & coupling_row_major,
    const std::vector<double> & min, const std::vector<double> & max)
  : names_(joints), min_(min), max_(max)
  {
    const std::size_t n = joints.size();
    if (coupling_row_major.size() != n * n || min.size() != n || max.size() != n) {
      throw std::runtime_error("instrument: joints, coupling (n x n), min and max disagree");
    }
    C_.resize(n, n);
    for (std::size_t r = 0; r < n; ++r) {
      for (std::size_t c = 0; c < n; ++c) {C_(r, c) = coupling_row_major[r * n + c];}
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(C_);
    if (!lu.isInvertible()) {throw std::runtime_error("instrument coupling is singular");}
    Cinv_ = lu.inverse();
    for (std::size_t j = 0; j < n; ++j) {
      if (joints[j] == "wrist_yaw") {yaw_ = static_cast<int>(j);}
      if (joints[j] == "jaw") {jaw_ = static_cast<int>(j);}
    }
  }

  std::size_t size() const {return names_.size();}
  int jawIndex() const {return jaw_;}

  // Coupling rows (disc order) the jaw drives.
  std::vector<std::size_t> jawRows() const
  {
    std::vector<std::size_t> rows;
    if (jaw_ < 0) {return rows;}
    for (Eigen::Index r = 0; r < C_.rows(); ++r) {
      if (C_(r, jaw_) != 0.0) {rows.push_back(static_cast<std::size_t>(r));}
    }
    return rows;
  }

  // The unitless jaw command (PI, 2026-11-05), the same for every instrument and input:
  //   0 .. 90   opening, as a share of the instrument's jaw max (90 = fully open)
  //   0         jaws just closed
  //   0 .. -10  grip: the target goes below closed, toward the file's jaw min at -10, and the
  //             grip current cap rises linearly to full_cap at -10 (-5 -> half)
  // clamped to [-10, 90]. The cap never drops below floor_cap, so a free jaw still closes.
  struct JawGoal {double target; double cap;};
  static JawGoal jawFromCommand(
    double c, double jaw_min, double jaw_max, double full_cap, double floor_cap)
  {
    c = std::max(-10.0, std::min(90.0, std::isfinite(c) ? c : 0.0));
    if (c >= 0.0) {return {c / 90.0 * jaw_max, floor_cap};}
    return {c / 10.0 * -jaw_min, std::max(floor_cap, full_cap * -c / 10.0)};
  }

  // Hold a closing jaw target at 0 while a jaw disc's |current| >= cap (raw servo units, the
  // value read back from Present Current). row_current is in coupling-row order. cap <= 0 is
  // off. Returns true when it capped.
  bool capJawClosing(Eigen::VectorXd & qd, const std::vector<double> & row_current, double cap) const
  {
    if (cap <= 0.0 || jaw_ < 0 || qd[jaw_] >= 0.0) {return false;}
    for (auto r : jawRows()) {
      if (r < row_current.size() && std::abs(row_current[r]) >= cap) {
        qd[jaw_] = 0.0;
        return true;
      }
    }
    return false;
  }
  const std::vector<std::string> & names() const {return names_;}

  Eigen::VectorXd joints(const Eigen::VectorXd & disc_angles) const {return Cinv_ * disc_angles;}

  static double limitNear(double v, double q, double lo, double hi)
  {
    constexpr double kGain = 6.0;  // A's kToolLimitGain [1/s]
    if (v > 0) {v = q > hi ? 0.0 : std::min(v, kGain * (hi - q));}
    if (v < 0) {v = q < lo ? 0.0 : std::max(v, kGain * (lo - q));}
    return v;
  }

  // Disc velocities (in coupling-row order) for joint velocity targets at joints q.
  Eigen::VectorXd discVelocities(const Eigen::VectorXd & q, Eigen::VectorXd qd) const
  {
    for (Eigen::Index j = 0; j < qd.size(); ++j) {qd[j] = limitNear(qd[j], q[j], min_[j], max_[j]);}
    if (yaw_ >= 0 && jaw_ >= 0) {
      const double y = q[yaw_], o = q[jaw_];
      double b1 = qd[yaw_] + qd[jaw_] / 2.0, b2 = qd[yaw_] - qd[jaw_] / 2.0;
      b1 = limitNear(b1, y + o / 2.0, min_[yaw_], max_[yaw_]);
      b2 = limitNear(b2, y - o / 2.0, min_[yaw_], max_[yaw_]);
      qd[yaw_] = (b1 + b2) / 2.0;
      qd[jaw_] = b1 - b2;
    }
    return C_ * qd;
  }

private:
  std::vector<std::string> names_;
  std::vector<double> min_, max_;
  Eigen::MatrixXd C_, Cinv_;
  int yaw_{-1};
  int jaw_{-1};
};

}  // namespace arts_dv_tool_driver

#endif  // ARTS_DV_TOOL_DRIVER__INSTRUMENT_JOINTS_HPP_
