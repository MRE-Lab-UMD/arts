// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef ARTS_CONTROLLERS__TASK_QP_HPP_
#define ARTS_CONTROLLERS__TASK_QP_HPP_

#include <memory>

#include <Eigen/Core>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace arts_controllers
{

/// Decision 17's task layer as one small dense QP per control cycle:
///
///   min_qd  || W (J qd - v) ||^2 + damping ||qd - qd_rest||^2
///   s.t.    A qd = b                (the RCM, when enabled: 2 rows; plus the roll row, when
///                                    configured: 1 row, the UR must not roll the shaft)
///           c_lo <= c qd <= c_hi    (insertion depth, when the RCM is enabled: 1 row)
///           lo <= qd <= hi          (joint velocity and position limits)
///
/// The RCM is an EQUALITY, so the tip task can never trade it away; orientation keeps its
/// own weight instead of being demoted to make room (the weighting approach decision 17
/// rejects). Two solver instances are built at configure time, one per problem shape,
/// because a proxsuite problem's dimensions are fixed at construction; switching the RCM
/// is choosing which one runs, never an allocation.
///
/// No heap traffic after configure(): the matrices are preallocated members and proxsuite
/// keeps its workspace inside the QP object (checked by test/task_qp_test.cpp, which counts
/// allocations across solve()).
class TaskQp
{
public:
  using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using Vector6 = Eigen::Matrix<double, 6, 1>;

  static constexpr int kRcmRows = 2;

  /// Rows the caller fills when the RCM is enabled. Sized by configure().
  struct RcmRows
  {
    Eigen::Matrix<double, kRcmRows, Eigen::Dynamic> a;
    Eigen::Matrix<double, kRcmRows, 1> b;
    Eigen::Matrix<double, 1, Eigen::Dynamic> c;
    double c_lo{0.0};
    double c_hi{0.0};
  };

  /// Collaborator A's roll row (decision 17, 2026-11-03 amendment): one equality a qd = b,
  /// present in both problem shapes when configure() was asked for it.
  struct RollRow
  {
    Eigen::Matrix<double, 1, Eigen::Dynamic> a;
    double b{0.0};
  };

  /// Allocates both solver instances for `nv` joint velocities, with the roll row as an
  /// extra equality when `roll_row`. Not real-time safe.
  void configure(int nv, bool roll_row = false);

  /// One solve. `rcm` null means the RCM is off. Returns false (and leaves `qd` untouched)
  /// when the solver does not report SOLVED; the caller holds the arm in that case. `cold`
  /// starts from zero instead of the previous result (for a retry after a failure).
  /// `qd_rest` is what the damping pulls toward where the task leaves a direction free (a
  /// posture task); zero-length means zero.
  /// `roll` must be non-null exactly when configure() was asked for the roll row.
  bool solve(
    const Matrix6x & jacobian, const Vector6 & v, const Vector6 & weights, double damping,
    const Eigen::VectorXd & qd_rest, const Eigen::VectorXd & lo, const Eigen::VectorXd & hi,
    const RcmRows * rcm, Eigen::VectorXd & qd, bool cold = false, const RollRow * roll = nullptr);

  RcmRows & rcmRows() {return rcm_rows_;}
  RollRow & rollRow() {return roll_row_;}
  bool hasRollRow() const {return has_roll_;}

  /// Per-joint weight on the damping/posture term (all 1 after configure()): where the task
  /// leaves joints redundant, the solve moves the cheaper ones (weighted least norm). Size
  /// must be nv. Not real-time safe only in the sense of being a configure-time setting.
  void setJointWeights(const Eigen::VectorXd & weights) {joint_weights_ = weights;}

  /// Iterations and proxsuite status (QPSolverOutput as int) of the last solve.
  long lastIterations() const {return last_iterations_;}
  int lastStatus() const {return last_status_;}

private:
  using Qp = proxsuite::proxqp::dense::QP<double>;

  void applySettings(Qp & qp);

  int nv_{0};
  bool has_roll_{false};
  std::unique_ptr<Qp> qp_free_;
  std::unique_ptr<Qp> qp_rcm_;

  // Row-major because that is proxsuite's matrix type: a column-major argument is silently
  // copied into a row-major temporary on every call.
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  RowMatrix h_;
  Eigen::VectorXd g_;
  Eigen::VectorXd joint_weights_;
  Matrix6x weighted_jacobian_;
  Vector6 weighted_v_;
  RowMatrix c_;
  Eigen::VectorXd c_lo_;
  Eigen::VectorXd c_hi_;
  RowMatrix a_;
  Eigen::VectorXd b_;
  RowMatrix a_free_;  // the roll row alone (RCM off), when configured
  Eigen::VectorXd b_free_;
  RcmRows rcm_rows_;
  RollRow roll_row_;
  long last_iterations_{0};
  int last_status_{0};
};

}  // namespace arts_controllers

#endif  // ARTS_CONTROLLERS__TASK_QP_HPP_
