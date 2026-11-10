// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0

#include "arts_controllers/task_qp.hpp"

namespace arts_controllers
{

void TaskQp::applySettings(Qp & qp)
{
  // Velocities are rad/s and m/s; 1e-9 on the residual is far below what the arm resolves,
  // and the problem is tiny, so tight tolerances cost microseconds, not deadlines.
  qp.settings.eps_abs = 1e-9;
  qp.settings.eps_rel = 0.0;
  // A feasible problem warm-started from the last cycle converges in a handful of
  // iterations; one the boxes make infeasible otherwise burns ~200 before saying so, and the
  // controller relaxes and retries in the same cycle.
  qp.settings.max_iter = 50;
  qp.settings.verbose = false;
  // Consecutive cycles differ by micro-radians, so the previous solution is nearly the
  // answer.
  qp.settings.initial_guess =
    proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
}

void TaskQp::configure(int nv, bool roll_row)
{
  nv_ = nv;
  has_roll_ = roll_row;
  const int n_roll = roll_row ? 1 : 0;
  // Box constraints (lo <= qd <= hi) are handled natively, so the joint limits do not
  // need identity rows in C.
  qp_free_ = std::make_unique<Qp>(
    nv, n_roll, 0, true, proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
  qp_rcm_ = std::make_unique<Qp>(
    nv, kRcmRows + n_roll, 1, true, proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
  applySettings(*qp_free_);
  applySettings(*qp_rcm_);

  h_ = RowMatrix::Zero(nv, nv);
  g_ = Eigen::VectorXd::Zero(nv);
  joint_weights_ = Eigen::VectorXd::Ones(nv);
  weighted_jacobian_ = Matrix6x::Zero(6, nv);
  a_ = RowMatrix::Zero(kRcmRows + n_roll, nv);
  b_ = Eigen::VectorXd::Zero(kRcmRows + n_roll);
  a_free_ = RowMatrix::Zero(n_roll, nv);
  b_free_ = Eigen::VectorXd::Zero(n_roll);
  roll_row_.a = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, nv);
  roll_row_.b = 0.0;
  c_ = RowMatrix::Zero(1, nv);
  c_lo_ = Eigen::VectorXd::Zero(1);
  c_hi_ = Eigen::VectorXd::Zero(1);
  rcm_rows_.a = Eigen::Matrix<double, kRcmRows, Eigen::Dynamic>::Zero(kRcmRows, nv);
  rcm_rows_.b.setZero();
  rcm_rows_.c = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, nv);

  // init() allocates (preconditioner, factorization set-up), so both shapes are initialized
  // here with a well-posed dummy problem; every solve in the control cycle is an update().
  h_.setIdentity();
  const Eigen::VectorXd lo = Eigen::VectorXd::Constant(nv, -1.0);
  const Eigen::VectorXd hi = Eigen::VectorXd::Constant(nv, 1.0);
  a_.leftCols(kRcmRows + n_roll).setIdentity();
  c_(0, nv - 1) = 1.0;
  c_lo_[0] = -1.0;
  c_hi_[0] = 1.0;
  if (roll_row) {
    a_free_(0, 0) = 1.0;
    qp_free_->init(h_, g_, a_free_, b_free_, proxsuite::nullopt, proxsuite::nullopt,
                   proxsuite::nullopt, lo, hi);
  } else {
    qp_free_->init(h_, g_, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
                   proxsuite::nullopt, proxsuite::nullopt, lo, hi);
  }
  qp_rcm_->init(h_, g_, a_, b_, c_, c_lo_, c_hi_, lo, hi);
  qp_free_->solve();
  qp_rcm_->solve();
}

bool TaskQp::solve(
  const Matrix6x & jacobian, const Vector6 & v, const Vector6 & weights, double damping,
  const Eigen::VectorXd & qd_rest, const Eigen::VectorXd & lo, const Eigen::VectorXd & hi,
  const RcmRows * rcm, Eigen::VectorXd & qd, bool cold, const RollRow * roll)
{
  if ((roll != nullptr) != has_roll_) {
    return false;  // the problem shape was fixed at configure()
  }
  weighted_jacobian_.noalias() = weights.asDiagonal() * jacobian;
  weighted_v_ = weights.cwiseProduct(v);
  h_.noalias() = weighted_jacobian_.transpose() * weighted_jacobian_;
  h_.diagonal().array() += damping * joint_weights_.array();
  g_.noalias() = -weighted_jacobian_.transpose() * weighted_v_;
  if (qd_rest.size() == nv_) {
    g_.array() -= damping * joint_weights_.array() * qd_rest.array();
  }

  // The vectors go straight into the model: proxsuite's update() copies each one through
  // .eval(), a heap temporary per vector per cycle (found by test/task_qp_test.cpp). Its
  // setup() rescales from qp->model either way, so passing only the matrices and the box
  // (plain same-size assignments) is the same update without the allocations.
  Qp * qp = nullptr;
  if (rcm == nullptr && roll == nullptr) {
    qp = qp_free_.get();
    qp->model.g = g_;
    qp->update(h_, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
               proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, lo, hi);
  } else if (rcm == nullptr) {
    qp = qp_free_.get();
    a_free_.row(0) = roll->a;
    qp->model.g = g_;
    qp->model.b[0] = roll->b;
    qp->update(h_, proxsuite::nullopt, a_free_, proxsuite::nullopt, proxsuite::nullopt,
               proxsuite::nullopt, proxsuite::nullopt, lo, hi);
  } else {
    qp = qp_rcm_.get();
    a_.topRows<kRcmRows>() = rcm->a;
    c_ = rcm->c;
    qp->model.g = g_;
    qp->model.b.head<kRcmRows>() = rcm->b;
    if (roll != nullptr) {
      a_.row(kRcmRows) = roll->a;
      qp->model.b[kRcmRows] = roll->b;
    }
    qp->model.l[0] = rcm->c_lo;
    qp->model.u[0] = rcm->c_hi;
    qp->update(h_, proxsuite::nullopt, a_, proxsuite::nullopt, c_, proxsuite::nullopt,
               proxsuite::nullopt, lo, hi);
  }

  if (cold) {
    // A retry after a failed attempt starts from zero, not from the failed attempt's
    // iterate: in the controller, warm-started solves following infeasible ones were
    // declared PRIMAL_INFEASIBLE after 1-2 iterations on plainly feasible problems (mock,
    // 2026-11-01; not reproduced offline from the same data). The cold retry cleared them.
    // Zeroed in place: proxsuite's solve(x, y, z) copies through .eval() (allocates) and
    // switches initial_guess to WARM_START for good. WARM_START_WITH_PREVIOUS_RESULT starts
    // from `results`, so zeroing it is the same cold start.
    qp->results.x.setZero();
    qp->results.y.setZero();
    qp->results.z.setZero();
  }
  qp->solve();
  last_iterations_ = static_cast<long>(qp->results.info.iter);
  last_status_ = static_cast<int>(qp->results.info.status);
  if (qp->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ||
      !qp->results.x.allFinite())
  {
    return false;
  }
  qd = qp->results.x;
  return true;
}

}  // namespace arts_controllers
