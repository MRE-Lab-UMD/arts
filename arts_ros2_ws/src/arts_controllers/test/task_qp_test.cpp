// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0
//
// TaskQp: the RCM equality holds exactly, bounds hold, an unconstrained problem reduces to
// damped least squares, an infeasible one is reported, and a solve does not allocate.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>

#include "arts_controllers/task_qp.hpp"

namespace
{
std::atomic<bool> g_counting{false};
std::atomic<long> g_allocations{0};
}  // namespace

// Count at the malloc level: Eigen allocates with malloc, not operator new, so a counter
// on operator new alone passed a solve() that built an Eigen temporary every call (the
// mutation check, 2026-11-01). glibc exposes the real allocators as __libc_*.
extern "C" {
void * __libc_malloc(std::size_t);
void * __libc_calloc(std::size_t, std::size_t);
void * __libc_realloc(void *, std::size_t);
void * malloc(std::size_t size)
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  return __libc_malloc(size);
}
void * calloc(std::size_t n, std::size_t size)
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  return __libc_calloc(n, size);
}
void * realloc(void * p, std::size_t size)
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  return __libc_realloc(p, size);
}
}

using arts_controllers::TaskQp;

namespace
{
constexpr int kN = 6;

struct Fixture
{
  TaskQp qp;
  TaskQp::Matrix6x jacobian = TaskQp::Matrix6x::Random(6, kN);
  TaskQp::Vector6 v = TaskQp::Vector6::Random();
  TaskQp::Vector6 w = TaskQp::Vector6::Ones();
  Eigen::VectorXd lo = Eigen::VectorXd::Constant(kN, -10.0);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(kN, 10.0);
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd none;                                   // no posture task
  Eigen::VectorXd rest = Eigen::VectorXd::Constant(kN, 0.1);

  Fixture()
  {
    // Well conditioned (like an arm away from singularities), so the free solution is
    // well inside the +-10 box and the comparison tests the solver, not the bounds.
    std::srand(7);
    jacobian = 0.3 * TaskQp::Matrix6x::Random(6, kN) + 2.0 * TaskQp::Matrix6x::Identity(6, kN);
    qp.configure(kN);
  }
};
}  // namespace

TEST(TaskQp, FreeProblemIsDampedLeastSquares)
{
  Fixture f;
  ASSERT_TRUE(f.qp.solve(f.jacobian, f.v, f.w, 1e-8, f.none, f.lo, f.hi, nullptr, f.qd));
  const Eigen::VectorXd expected = f.jacobian.colPivHouseholderQr().solve(f.v);
  EXPECT_LT((f.qd - expected).norm(), 1e-5);
}

TEST(TaskQp, PostureActsOnlyWhereTheTaskIsFree)
{
  // A rank-deficient task (last column zero: that joint does not move the task) leaves one
  // direction free; the posture term must drive it to qd_rest there and leave the task met.
  Fixture f;
  f.jacobian.col(kN - 1).setZero();
  f.jacobian.col(0).setZero();
  f.v = f.jacobian * Eigen::VectorXd::Constant(kN, 0.05);  // reachable task
  ASSERT_TRUE(f.qp.solve(f.jacobian, f.v, f.w, 1e-4, f.rest, f.lo, f.hi, nullptr, f.qd));
  // In a free direction only the damping (1e-4) prices the error, so the solver's 1e-9
  // residual tolerance leaves ~1e-5 rad/s there: negligible as a velocity.
  EXPECT_NEAR(f.qd[kN - 1], 0.1, 1e-4);
  EXPECT_NEAR(f.qd[0], 0.1, 1e-4);
  EXPECT_LT((f.jacobian * f.qd - f.v).norm(), 1e-5);
}

TEST(TaskQp, JointWeightsMoveTheCheaperJoint)
{
  // Joints 0 and 1 drive the task identically (a redundant pair, like the UR wrist_3 and an
  // instrument roll disc about one axis): the weighted least-norm solution splits the
  // motion in inverse proportion to the weights, and the task is still met.
  Fixture f;
  f.jacobian.col(1) = f.jacobian.col(0);
  Eigen::VectorXd w = Eigen::VectorXd::Ones(kN);
  w[0] = 10.0;
  f.qp.setJointWeights(w);
  f.v = f.jacobian * Eigen::VectorXd::Constant(kN, 0.05);
  ASSERT_TRUE(f.qp.solve(f.jacobian, f.v, f.w, 1e-4, f.none, f.lo, f.hi, nullptr, f.qd));
  EXPECT_NEAR(f.qd[1] / f.qd[0], 10.0, 1e-3);
  EXPECT_LT((f.jacobian * f.qd - f.v).norm(), 1e-5);
}

TEST(TaskQp, BoundsHold)
{
  Fixture f;
  f.lo.setConstant(-0.01);
  f.hi.setConstant(0.01);
  f.v *= 100.0;
  ASSERT_TRUE(f.qp.solve(f.jacobian, f.v, f.w, 1e-8, f.none, f.lo, f.hi, nullptr, f.qd));
  EXPECT_LE(f.qd.maxCoeff(), 0.01 + 1e-7);
  EXPECT_GE(f.qd.minCoeff(), -0.01 - 1e-7);
}

TEST(TaskQp, RcmEqualityIsExactAndInsertionBoundHolds)
{
  Fixture f;
  auto & rows = f.qp.rcmRows();
  for (int k = 0; k < 50; ++k) {
    rows.a = Eigen::Matrix<double, 2, Eigen::Dynamic>::Random(2, kN);
    rows.b = Eigen::Vector2d::Random() * 0.1;
    rows.c = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN);
    rows.c_lo = -0.05;
    rows.c_hi = 0.05;
    f.v = TaskQp::Vector6::Random() * 10.0;  // a task that pulls hard against the rows
    ASSERT_TRUE(f.qp.solve(f.jacobian, f.v, f.w, 1e-6, f.none, f.lo, f.hi, &rows, f.qd)) << k;
    EXPECT_LT((rows.a * f.qd - rows.b).norm(), 1e-7) << k;
    const double c = (rows.c * f.qd)(0);
    EXPECT_LE(c, rows.c_hi + 1e-7) << k;
    EXPECT_GE(c, rows.c_lo - 1e-7) << k;
  }
}

TEST(TaskQp, InfeasibleIsReported)
{
  Fixture f;
  auto & rows = f.qp.rcmRows();
  rows.a.setZero();
  rows.a(0, 0) = 1.0;
  rows.a(1, 1) = 1.0;
  rows.b << 5.0, 0.0;  // qd0 = 5 but the box allows at most 1
  rows.c.setZero();
  rows.c_lo = -1.0;
  rows.c_hi = 1.0;
  f.lo.setConstant(-1.0);
  f.hi.setConstant(1.0);
  f.qd.setConstant(42.0);
  EXPECT_FALSE(f.qp.solve(f.jacobian, f.v, f.w, 1e-6, f.none, f.lo, f.hi, &rows, f.qd));
  EXPECT_EQ(f.qd[0], 42.0);  // untouched on failure
}

TEST(TaskQp, SolveDoesNotAllocate)
{
  Fixture f;
  auto & rows = f.qp.rcmRows();
  rows.a = Eigen::Matrix<double, 2, Eigen::Dynamic>::Random(2, kN);
  rows.c = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN);
  rows.c_lo = -1.0;
  rows.c_hi = 1.0;
  // configure() ran init() for both shapes; from here every solve must be heap free,
  // including the very first one and switching between the shapes.
  g_allocations = 0;
  g_counting = true;
  for (int k = 0; k < 500; ++k) {
    f.v[k % 6] += 1e-3;
    f.jacobian(k % 6, k % kN) += 1e-4;
    rows.b[k % 2] = 1e-3 * (k % 7);
    (void)f.qp.solve(
      f.jacobian, f.v, f.w, 1e-6, f.rest, f.lo, f.hi, (k % 2) ? &rows : nullptr, f.qd, k % 3 == 0);
  }
  g_counting = false;
  EXPECT_EQ(g_allocations.load(), 0);
}

TEST(TaskQp, FeasibleAfterInfeasibleSolves)
{
  // The controller retries with relaxed boxes after an infeasible attempt; the retry must
  // not inherit the failed attempt's state (warm start) and report a false infeasibility.
  Fixture f;
  auto & rows = f.qp.rcmRows();
  rows.a = Eigen::Matrix<double, 2, Eigen::Dynamic>::Random(2, kN);
  rows.b << 1e-4, -2e-4;
  rows.c = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN);
  rows.c_lo = -0.5;
  rows.c_hi = 1.0;
  int false_infeasible = 0;
  for (int k = 0; k < 200; ++k) {
    // An infeasible box (forces qd far from anything A allows) ...
    Eigen::VectorXd lo_bad = Eigen::VectorXd::Constant(kN, 0.9);
    Eigen::VectorXd hi_bad = Eigen::VectorXd::Constant(kN, 1.0);
    (void)f.qp.solve(f.jacobian, f.v, f.w, 1e-4, f.none, lo_bad, hi_bad, &rows, f.qd);
    // ... then the relaxed, clearly feasible one.
    f.v = TaskQp::Vector6::Random();
    if (!f.qp.solve(f.jacobian, f.v, f.w, 1e-4, f.none, f.lo, f.hi, &rows, f.qd)) {
      ++false_infeasible;
    }
  }
  EXPECT_EQ(false_infeasible, 0);
}

// Collaborator A's roll row (decision 17, 2026-11-03 amendment): nine joints, one extra
// equality, met exactly with the RCM off (1 row) and on (2 + 1 rows).
namespace
{
constexpr int kN9 = 9;
struct Fixture9
{
  TaskQp qp;
  TaskQp::Matrix6x jacobian;
  TaskQp::Vector6 v = TaskQp::Vector6::Random();
  TaskQp::Vector6 w = TaskQp::Vector6::Ones();
  Eigen::VectorXd lo = Eigen::VectorXd::Constant(kN9, -10.0);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(kN9, 10.0);
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(kN9);
  Eigen::VectorXd rest = Eigen::VectorXd::Constant(kN9, 0.1);
  Fixture9()
  {
    std::srand(11);
    jacobian = 0.3 * TaskQp::Matrix6x::Random(6, kN9) + 2.0 * TaskQp::Matrix6x::Identity(6, kN9);
    qp.configure(kN9, true);
  }
};
}  // namespace

TEST(TaskQp, RollRowIsExactWithAndWithoutRcm)
{
  Fixture9 f;
  auto & roll = f.qp.rollRow();
  auto & rows = f.qp.rcmRows();
  for (int k = 0; k < 50; ++k) {
    roll.a = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN9);
    roll.a.rightCols(3).setZero();  // the instrument joints do not enter it
    roll.b = 0.05 * (k % 5 - 2);
    rows.a = Eigen::Matrix<double, 2, Eigen::Dynamic>::Random(2, kN9);
    rows.b = Eigen::Vector2d::Random() * 0.1;
    rows.c = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN9);
    rows.c_lo = -0.05;
    rows.c_hi = 0.05;
    f.v = TaskQp::Vector6::Random() * 5.0;
    const bool rcm = k % 2;
    ASSERT_TRUE(f.qp.solve(
      f.jacobian, f.v, f.w, 1e-6, f.rest, f.lo, f.hi, rcm ? &rows : nullptr, f.qd, false,
      &roll)) << k;
    EXPECT_NEAR((roll.a * f.qd)(0), roll.b, 1e-7) << k;
    if (rcm) {
      EXPECT_LT((rows.a * f.qd - rows.b).norm(), 1e-7) << k;
    }
  }
}

TEST(TaskQp, RollShapeMismatchIsRefused)
{
  Fixture9 f;
  // Configured with the roll row: a solve without it is a programming error, refused.
  EXPECT_FALSE(f.qp.solve(f.jacobian, f.v, f.w, 1e-6, f.rest, f.lo, f.hi, nullptr, f.qd));
  Fixture plain;  // 6 joints, no roll row
  TaskQp::RollRow roll;
  roll.a = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, kN);
  EXPECT_FALSE(plain.qp.solve(
    plain.jacobian, plain.v, plain.w, 1e-6, plain.rest, plain.lo, plain.hi, nullptr, plain.qd,
    false, &roll));
}

TEST(TaskQp, RollSolveDoesNotAllocate)
{
  Fixture9 f;
  auto & roll = f.qp.rollRow();
  auto & rows = f.qp.rcmRows();
  roll.a = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN9);
  rows.a = Eigen::Matrix<double, 2, Eigen::Dynamic>::Random(2, kN9);
  rows.c = Eigen::Matrix<double, 1, Eigen::Dynamic>::Random(1, kN9);
  rows.c_lo = -1.0;
  rows.c_hi = 1.0;
  g_allocations = 0;
  g_counting = true;
  for (int k = 0; k < 500; ++k) {
    f.v[k % 6] += 1e-3;
    roll.b = 1e-3 * (k % 5);
    rows.b[k % 2] = 1e-3 * (k % 7);
    (void)f.qp.solve(
      f.jacobian, f.v, f.w, 1e-6, f.rest, f.lo, f.hi, (k % 2) ? &rows : nullptr, f.qd,
      k % 3 == 0, &roll);
  }
  g_counting = false;
  EXPECT_EQ(g_allocations.load(), 0);
}
