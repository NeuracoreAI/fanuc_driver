// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "fanuc_client/joint_stream_interpolator.hpp"

namespace fanuc_client
{
namespace
{

constexpr double kDt = 0.008;  // 8 ms Stream Motion period

JointStreamInterpolator makeInterpolator(const int n_joints = 6)
{
  JointStreamInterpolator interpolator(n_joints);
  std::vector<double> vmax(n_joints, 120.0);
  std::vector<double> amax(n_joints, 1200.0);
  std::vector<double> jmax(n_joints, 12000.0);
  interpolator.setLimits(vmax, amax, jmax, 1.0);
  interpolator.setMaxPositionStepDeg(2.0);
  return interpolator;
}

double maxStep(const Eigen::VectorXd& prev, const Eigen::VectorXd& next)
{
  return (next - prev).cwiseAbs().maxCoeff();
}

}  // namespace

TEST(JointStreamInterpolatorTest, ReachesStaticGoal)
{
  auto interpolator = makeInterpolator();
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start[0] = 10.0;
  start[1] = -20.0;
  interpolator.reset(start);

  const Eigen::VectorXd goal = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd cmd = start;
  for (int i = 0; i < 20000; ++i)
  {
    cmd = interpolator.step(goal, kDt);
  }

  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(cmd[i], goal[i], 0.05);
  }
}

TEST(JointStreamInterpolatorTest, StaticGoalRespectsVelocityAccelJerkLimits)
{
  constexpr double kVmax = 80.0;
  constexpr double kAmax = 800.0;
  constexpr double kJmax = 8000.0;
  constexpr double kSlack = 1.05;

  JointStreamInterpolator interpolator(1);
  interpolator.setLimits({ kVmax }, { kAmax }, { kJmax }, 1.0);
  interpolator.setMaxPositionStepDeg(50.0);  // do not hide OTG limits behind step cap

  Eigen::VectorXd start = Eigen::VectorXd::Zero(1);
  start[0] = 90.0;
  interpolator.reset(start);
  const Eigen::VectorXd goal = Eigen::VectorXd::Zero(1);

  Eigen::VectorXd prev = start;
  double prev_v = 0.0;
  double prev_a = 0.0;
  double peak_v = 0.0;
  double peak_a = 0.0;
  double peak_j = 0.0;

  for (int i = 0; i < 20000; ++i)
  {
    const Eigen::VectorXd cmd = interpolator.step(goal, kDt);
    const double v = (cmd[0] - prev[0]) / kDt;
    const double a = (v - prev_v) / kDt;
    const double j = (a - prev_a) / kDt;
    peak_v = std::max(peak_v, std::abs(v));
    peak_a = std::max(peak_a, std::abs(a));
    if (i > 1)
    {
      peak_j = std::max(peak_j, std::abs(j));
    }
    prev = cmd;
    prev_v = v;
    prev_a = a;
    if (std::abs(cmd[0] - goal[0]) < 1e-3 && std::abs(v) < 1e-2)
    {
      break;
    }
  }

  EXPECT_NEAR(prev[0], goal[0], 0.05);
  EXPECT_LE(peak_v, kVmax * kSlack);
  EXPECT_LE(peak_a, kAmax * kSlack);
  EXPECT_LE(peak_j, kJmax * kSlack);
}

TEST(JointStreamInterpolatorTest, StepsRespectVelocityLimit)
{
  auto interpolator = makeInterpolator();
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  interpolator.reset(start);

  const Eigen::VectorXd goal = Eigen::VectorXd::Constant(6, 90.0);
  Eigen::VectorXd prev = start;
  for (int i = 0; i < 20; ++i)
  {
    const Eigen::VectorXd next = interpolator.step(goal, kDt);
    const double step = maxStep(prev, next);
    EXPECT_LE(step, 120.0 * kDt + 0.05);
    prev = next;
  }
}

TEST(JointStreamInterpolatorTest, InterruptibleGoalChange)
{
  auto interpolator = makeInterpolator();
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  interpolator.reset(start);

  Eigen::VectorXd goal_a = Eigen::VectorXd::Zero(6);
  goal_a[0] = 45.0;
  Eigen::VectorXd goal_b = Eigen::VectorXd::Zero(6);
  goal_b[0] = -30.0;

  Eigen::VectorXd cmd = start;
  for (int i = 0; i < 100; ++i)
  {
    cmd = interpolator.step(goal_a, kDt);
  }
  EXPECT_GT(cmd[0], 5.0);

  for (int i = 0; i < 20000; ++i)
  {
    cmd = interpolator.step(goal_b, kDt);
  }
  EXPECT_NEAR(cmd[0], goal_b[0], 0.1);
  EXPECT_LT(cmd[0], 0.0);
}

TEST(JointStreamInterpolatorTest, ContinuousRetargetingStaysSmooth)
{
  // Rapidly changing goals (teleop-like): finite-diff jerk must stay near limits.
  constexpr double kVmax = 100.0;
  constexpr double kAmax = 1000.0;
  constexpr double kJmax = 10000.0;
  constexpr double kSlack = 1.15;  // finite-diff on discrete samples is slightly noisy

  JointStreamInterpolator interpolator(1);
  interpolator.setLimits({ kVmax }, { kAmax }, { kJmax }, 1.0);
  interpolator.setMaxPositionStepDeg(5.0);

  Eigen::VectorXd start = Eigen::VectorXd::Zero(1);
  interpolator.reset(start);

  Eigen::VectorXd prev = start;
  double prev_v = 0.0;
  double prev_a = 0.0;
  double peak_j = 0.0;

  for (int i = 0; i < 500; ++i)
  {
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(1);
    // Sweep a moving setpoint that reverses a few times.
    const double phase = static_cast<double>(i) * 0.05;
    goal[0] = 30.0 * std::sin(phase);
    if (i > 200 && i < 280)
    {
      goal[0] = -40.0;
    }

    const Eigen::VectorXd cmd = interpolator.step(goal, kDt);
    const double v = (cmd[0] - prev[0]) / kDt;
    const double a = (v - prev_v) / kDt;
    const double j = (a - prev_a) / kDt;
    if (i > 2)
    {
      peak_j = std::max(peak_j, std::abs(j));
    }
    prev = cmd;
    prev_v = v;
    prev_a = a;
  }

  EXPECT_LE(peak_j, kJmax * kSlack);
}

TEST(JointStreamInterpolatorTest, ResetClearsVelocity)
{
  auto interpolator = makeInterpolator();
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  interpolator.reset(start);

  const Eigen::VectorXd goal = Eigen::VectorXd::Constant(6, 30.0);
  for (int i = 0; i < 10; ++i)
  {
    interpolator.step(goal, kDt);
  }

  Eigen::VectorXd hold = Eigen::VectorXd::Constant(6, 5.0);
  interpolator.reset(hold);
  const Eigen::VectorXd cmd = interpolator.step(goal, kDt);
  EXPECT_LE((cmd - hold).cwiseAbs().maxCoeff(), 2.0 + 1e-6);
}

TEST(JointStreamInterpolatorTest, LargeGoalDoesNotTeleportInOneTick)
{
  JointStreamInterpolator interpolator(6);
  std::vector<double> vmax(6, 1.0e6);
  std::vector<double> amax(6, 1.0e8);
  std::vector<double> jmax(6, 1.0e10);
  interpolator.setLimits(vmax, amax, jmax, 1.0);
  interpolator.setMaxPositionStepDeg(2.0);

  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start[0] = 39.0;
  interpolator.reset(start);

  const Eigen::VectorXd goal = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd next = interpolator.step(goal, kDt);
  EXPECT_LE(std::abs(next[0] - start[0]), 2.0 + 1e-9);
  EXPECT_LT(next[0], start[0]);
}

TEST(JointStreamInterpolatorTest, DoesNotCrossGoalFasterThanStepCap)
{
  auto interpolator = makeInterpolator();
  interpolator.setMaxPositionStepDeg(2.0);

  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start[0] = 10.0;
  interpolator.reset(start);

  const Eigen::VectorXd goal = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd prev = start;
  for (int i = 0; i < 20; ++i)
  {
    const Eigen::VectorXd next = interpolator.step(goal, kDt);
    EXPECT_LE(std::abs(next[0] - prev[0]), 2.0 + 1e-9);
    prev = next;
  }
}

TEST(JointStreamInterpolatorTest, StartsSlowlyFromRest)
{
  // Jerk-limited OTG from rest: early |Δq| grows gradually, not vmax*dt on tick 0.
  JointStreamInterpolator interpolator(1);
  interpolator.setLimits({ 120.0 }, { 1200.0 }, { 12000.0 }, 1.0);
  interpolator.setMaxPositionStepDeg(10.0);

  Eigen::VectorXd start = Eigen::VectorXd::Zero(1);
  interpolator.reset(start);
  const Eigen::VectorXd goal = Eigen::VectorXd::Constant(1, 90.0);

  Eigen::VectorXd prev = start;
  std::vector<double> steps;
  for (int i = 0; i < 10; ++i)
  {
    const Eigen::VectorXd next = interpolator.step(goal, kDt);
    steps.push_back(std::abs(next[0] - prev[0]));
    prev = next;
  }

  EXPECT_LT(steps[0], 0.15);
  EXPECT_GT(steps[5], steps[0]);
  EXPECT_GT(steps[9], steps[3]);
}

TEST(JointStreamInterpolatorTest, PositionLimitsClampGoalAndCommand)
{
  JointStreamInterpolator interpolator(1);
  interpolator.setLimits({ 200.0 }, { 2000.0 }, { 20000.0 }, 1.0);
  interpolator.setMaxPositionStepDeg(50.0);
  interpolator.setPositionLimits({ -10.0 }, { 10.0 });

  Eigen::VectorXd start = Eigen::VectorXd::Zero(1);
  interpolator.reset(start);

  Eigen::VectorXd goal = Eigen::VectorXd::Constant(1, 90.0);
  Eigen::VectorXd cmd = start;
  for (int i = 0; i < 5000; ++i)
  {
    cmd = interpolator.step(goal, kDt);
    EXPECT_LE(cmd[0], 10.0 + 1e-6);
    EXPECT_GE(cmd[0], -10.0 - 1e-6);
  }
  EXPECT_NEAR(cmd[0], 10.0, 0.05);

  goal[0] = -90.0;
  for (int i = 0; i < 5000; ++i)
  {
    cmd = interpolator.step(goal, kDt);
    EXPECT_LE(cmd[0], 10.0 + 1e-6);
    EXPECT_GE(cmd[0], -10.0 - 1e-6);
  }
  EXPECT_NEAR(cmd[0], -10.0, 0.05);
}

}  // namespace fanuc_client
