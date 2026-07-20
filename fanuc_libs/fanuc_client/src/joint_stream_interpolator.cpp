// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_client/joint_stream_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fanuc_client
{
namespace
{
constexpr double kMinDtS = 1e-4;
constexpr double kPositionToleranceDeg = 1e-4;
constexpr double kVelocityToleranceDegS = 1e-3;
constexpr double kAccelerationToleranceDegS2 = 1e-2;
constexpr double kGoalChangeToleranceDeg = 1e-3;

// Rest-to-rest quintic s(τ)=6τ⁵−15τ⁴+10τ³ peak derivatives (w.r.t. τ):
//   max |s'|  = 15/8 = 1.875 at τ=0.5
//   max |s''| ≈ 5.7735 at τ≈0.211 / 0.789
//   max |s'''| = 60 at τ=0 / 1
constexpr double kQuinticPeakS1 = 1.875;          // → v_peak = k * dq / T
constexpr double kQuinticPeakS2 = 5.77350269189;  // → a_peak = k * dq / T²
constexpr double kQuinticPeakS3 = 60.0;           // → j_peak = k * dq / T³

constexpr int kPeakSampleCount = 24;
constexpr int kDurationInflateIters = 16;
constexpr double kDurationInflateFactor = 1.25;
constexpr double kPeakLimitSlack = 1.01;  // tiny numerical margin

double clampDouble(const double value, const double lo, const double hi)
{
  return std::max(lo, std::min(value, hi));
}

}  // namespace

JointStreamInterpolator::JointStreamInterpolator(const int n_joints)
  : n_joints_(std::max(n_joints, 1))
  , axes_(static_cast<std::size_t>(n_joints_))
{
  segment_.goal.assign(static_cast<std::size_t>(n_joints_), 0.0);
  segment_.coeffs.assign(static_cast<std::size_t>(n_joints_), QuinticCoeffs{});
}

void JointStreamInterpolator::clearSegment()
{
  segment_.active = false;
  segment_.T = 0.0;
  segment_.t = 0.0;
}

void JointStreamInterpolator::reset(const Eigen::VectorXd& joints)
{
  const int count = std::min(static_cast<int>(joints.size()), n_joints_);
  for (int i = 0; i < count; ++i)
  {
    axes_[static_cast<std::size_t>(i)].position = joints[i];
    axes_[static_cast<std::size_t>(i)].velocity = 0.0;
    axes_[static_cast<std::size_t>(i)].acceleration = 0.0;
  }
  for (int i = count; i < n_joints_; ++i)
  {
    axes_[static_cast<std::size_t>(i)].velocity = 0.0;
    axes_[static_cast<std::size_t>(i)].acceleration = 0.0;
  }
  clearSegment();
}

void JointStreamInterpolator::setLimits(const std::vector<double>& max_velocity_deg_s,
                                        const std::vector<double>& max_acceleration_deg_s2,
                                        const std::vector<double>& max_jerk_deg_s3, const double safety_scale)
{
  const double scale = std::max(safety_scale, 1e-3);
  for (int i = 0; i < n_joints_; ++i)
  {
    auto& axis = axes_[static_cast<std::size_t>(i)];
    const double vmax = i < static_cast<int>(max_velocity_deg_s.size()) ? max_velocity_deg_s[static_cast<std::size_t>(i)] :
                                                                          axis.max_velocity;
    const double amax = i < static_cast<int>(max_acceleration_deg_s2.size()) ?
                            max_acceleration_deg_s2[static_cast<std::size_t>(i)] :
                            axis.max_acceleration;
    const double jmax =
        i < static_cast<int>(max_jerk_deg_s3.size()) ? max_jerk_deg_s3[static_cast<std::size_t>(i)] : axis.max_jerk;
    axis.max_velocity = std::max(vmax * scale, 1e-3);
    axis.max_acceleration = std::max(amax * scale, 1e-3);
    axis.max_jerk = std::max(jmax * scale, 1e-3);
  }
}

void JointStreamInterpolator::setMaxPositionStepDeg(const double max_step_deg)
{
  max_position_step_deg_ = max_step_deg;
}

void JointStreamInterpolator::setPositionLimits(const std::vector<double>& lower_deg,
                                                const std::vector<double>& upper_deg)
{
  if (lower_deg.empty() || upper_deg.empty())
  {
    position_limits_enabled_ = false;
    position_lower_.clear();
    position_upper_.clear();
    return;
  }

  position_lower_.assign(static_cast<size_t>(n_joints_), -1.0e9);
  position_upper_.assign(static_cast<size_t>(n_joints_), 1.0e9);
  for (int i = 0; i < n_joints_; ++i)
  {
    const double lo = i < static_cast<int>(lower_deg.size()) ? lower_deg[static_cast<size_t>(i)] : -1.0e9;
    const double hi = i < static_cast<int>(upper_deg.size()) ? upper_deg[static_cast<size_t>(i)] : 1.0e9;
    position_lower_[static_cast<size_t>(i)] = std::min(lo, hi);
    position_upper_[static_cast<size_t>(i)] = std::max(lo, hi);
  }
  position_limits_enabled_ = true;
}

double JointStreamInterpolator::clampPosition(const int i, const double q) const
{
  if (!position_limits_enabled_)
  {
    return q;
  }
  return clampDouble(q, position_lower_[static_cast<size_t>(i)], position_upper_[static_cast<size_t>(i)]);
}

Eigen::VectorXd JointStreamInterpolator::command() const
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(n_joints_);
  for (int i = 0; i < n_joints_; ++i)
  {
    out[i] = axes_[static_cast<std::size_t>(i)].position;
  }
  return out;
}

double JointStreamInterpolator::smoothstepQuintic(const double tau)
{
  const double t = clampDouble(tau, 0.0, 1.0);
  // s(t) = 6t^5 - 15t^4 + 10t^3
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

double JointStreamInterpolator::durationRestToRest(const double dq_deg, const double vmax, const double amax,
                                                   const double jmax, const double dt)
{
  const double dq = std::abs(dq_deg);
  if (dq <= kPositionToleranceDeg)
  {
    return dt;
  }

  // Rest-to-rest peak scaling: v=k1*dq/T, a=k2*dq/T², j=k3*dq/T³.
  double T = kQuinticPeakS1 * dq / std::max(vmax, 1e-6);
  T = std::max(T, std::sqrt(kQuinticPeakS2 * dq / std::max(amax, 1e-6)));
  T = std::max(T, std::cbrt(kQuinticPeakS3 * dq / std::max(jmax, 1e-6)));
  return std::max(T, dt);
}

void JointStreamInterpolator::evalQuintic(const QuinticCoeffs& c, const double T, const double t, double& x, double& v,
                                          double& a, double& j)
{
  const double tau = (T > 1e-12) ? clampDouble(t / T, 0.0, 1.0) : 1.0;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;

  x = c.c0 + c.c1 * tau + c.c2 * tau2 + c.c3 * tau3 + c.c4 * tau4 + c.c5 * tau5;

  const double dx_dtau = c.c1 + 2.0 * c.c2 * tau + 3.0 * c.c3 * tau2 + 4.0 * c.c4 * tau3 + 5.0 * c.c5 * tau4;
  const double d2x_dtau2 = 2.0 * c.c2 + 6.0 * c.c3 * tau + 12.0 * c.c4 * tau2 + 20.0 * c.c5 * tau3;
  const double d3x_dtau3 = 6.0 * c.c3 + 24.0 * c.c4 * tau + 60.0 * c.c5 * tau2;

  if (T > 1e-12)
  {
    v = dx_dtau / T;
    a = d2x_dtau2 / (T * T);
    j = d3x_dtau3 / (T * T * T);
  }
  else
  {
    v = 0.0;
    a = 0.0;
    j = 0.0;
  }
}

bool JointStreamInterpolator::peaksWithinLimits(const QuinticCoeffs& c, const double T, const double vmax,
                                                 const double amax, const double jmax)
{
  if (T <= 1e-12)
  {
    return true;
  }

  const double v_lim = vmax * kPeakLimitSlack;
  const double a_lim = amax * kPeakLimitSlack;
  const double j_lim = jmax * kPeakLimitSlack;

  for (int s = 0; s <= kPeakSampleCount; ++s)
  {
    const double t = T * (static_cast<double>(s) / static_cast<double>(kPeakSampleCount));
    double x = 0.0;
    double v = 0.0;
    double a = 0.0;
    double j = 0.0;
    evalQuintic(c, T, t, x, v, a, j);
    if (std::abs(v) > v_lim || std::abs(a) > a_lim || std::abs(j) > j_lim)
    {
      return false;
    }
  }
  return true;
}

double JointStreamInterpolator::durationForAxis(const double x0, const double v0, const double a0, const double xf,
                                                const double vmax, const double amax, const double jmax, const double dt)
{
  const double dq = xf - x0;
  const bool at_goal = std::abs(dq) <= kPositionToleranceDeg && std::abs(v0) <= kVelocityToleranceDegS &&
                       std::abs(a0) <= kAccelerationToleranceDegS2;
  if (at_goal)
  {
    return dt;
  }

  // Rest-to-rest on remaining distance, plus coarse brake/settle time from state.
  double T = durationRestToRest(std::abs(dq), vmax, amax, jmax, dt);
  T = std::max(T, std::abs(v0) / std::max(amax, 1e-6));
  T = std::max(T, std::sqrt(std::abs(v0) / std::max(jmax, 1e-6)));
  T = std::max(T, std::abs(a0) / std::max(jmax, 1e-6));
  T = std::max(T, dt);

  for (int iter = 0; iter < kDurationInflateIters; ++iter)
  {
    const QuinticCoeffs coeffs = fitQuintic(x0, v0, a0, xf, T);
    if (peaksWithinLimits(coeffs, T, vmax, amax, jmax))
    {
      break;
    }
    T *= kDurationInflateFactor;
  }
  return T;
}

JointStreamInterpolator::QuinticCoeffs JointStreamInterpolator::fitQuintic(const double x0, const double v0,
                                                                           const double a0, const double xf,
                                                                           const double T)
{
  // x(τ)=Σ c_k τ^k with τ=t/T, matching (x,v,a) at 0 and (xf,0,0) at 1.
  // Rest-to-rest (v0=a0=0) reduces to s(τ)=10τ³−15τ⁴+6τ⁵.
  QuinticCoeffs c;
  c.c0 = x0;
  c.c1 = v0 * T;
  c.c2 = 0.5 * a0 * T * T;
  const double y1 = xf - c.c0 - c.c1 - c.c2;
  const double y2 = -c.c1 - 2.0 * c.c2;
  const double y3 = -2.0 * c.c2;
  c.c5 = 6.0 * y1 - 3.0 * y2 + 0.5 * y3;
  c.c4 = y2 - 3.0 * y1 - 2.0 * c.c5;
  c.c3 = y1 - c.c4 - c.c5;
  return c;
}

bool JointStreamInterpolator::goalChanged(const Eigen::VectorXd& target) const
{
  if (!segment_.active)
  {
    return true;
  }
  for (int i = 0; i < n_joints_; ++i)
  {
    const double raw_goal = i < target.size() ? target[i] : axes_[static_cast<std::size_t>(i)].position;
    const double goal = clampPosition(i, raw_goal);
    if (std::abs(goal - segment_.goal[static_cast<std::size_t>(i)]) > kGoalChangeToleranceDeg)
    {
      return true;
    }
  }
  return false;
}

void JointStreamInterpolator::planSegment(const Eigen::VectorXd& target, const double dt)
{
  double T = dt;
  bool any_motion = false;

  for (int i = 0; i < n_joints_; ++i)
  {
    const auto& axis = axes_[static_cast<std::size_t>(i)];
    const double raw_goal = i < target.size() ? target[i] : axis.position;
    const double goal = clampPosition(i, raw_goal);
    segment_.goal[static_cast<std::size_t>(i)] = goal;

    if (std::abs(goal - axis.position) > kPositionToleranceDeg || std::abs(axis.velocity) > kVelocityToleranceDegS ||
        std::abs(axis.acceleration) > kAccelerationToleranceDegS2)
    {
      any_motion = true;
      const double T_i =
          durationForAxis(axis.position, axis.velocity, axis.acceleration, goal, axis.max_velocity,
                          axis.max_acceleration, axis.max_jerk, dt);
      T = std::max(T, T_i);
    }
  }

  if (!any_motion)
  {
    clearSegment();
    return;
  }

  // Keep the first rest-to-rest tick under the hard |Δq| cap by enlarging T.
  if (max_position_step_deg_ > 0.0)
  {
    double max_abs_dq = 0.0;
    for (int i = 0; i < n_joints_; ++i)
    {
      max_abs_dq =
          std::max(max_abs_dq, std::abs(segment_.goal[static_cast<std::size_t>(i)] - axes_[static_cast<std::size_t>(i)].position));
    }
    if (max_abs_dq > kPositionToleranceDeg)
    {
      for (int iter = 0; iter < 8; ++iter)
      {
        const double tau = std::min(dt / T, 1.0);
        const double step0 = smoothstepQuintic(tau) * max_abs_dq;
        if (step0 <= max_position_step_deg_ + 1e-9)
        {
          break;
        }
        T *= std::max(step0 / max_position_step_deg_, 1.05);
      }
    }
  }

  // Re-check peaks after step-cap inflation (T only grew, so usually still OK;
  // for non-rest starts grow further if needed).
  for (int iter = 0; iter < kDurationInflateIters; ++iter)
  {
    bool ok = true;
    for (int i = 0; i < n_joints_; ++i)
    {
      const auto& axis = axes_[static_cast<std::size_t>(i)];
      const double goal = segment_.goal[static_cast<std::size_t>(i)];
      const QuinticCoeffs coeffs = fitQuintic(axis.position, axis.velocity, axis.acceleration, goal, T);
      if (!peaksWithinLimits(coeffs, T, axis.max_velocity, axis.max_acceleration, axis.max_jerk))
      {
        ok = false;
        break;
      }
    }
    if (ok)
    {
      break;
    }
    T *= kDurationInflateFactor;
  }

  for (int i = 0; i < n_joints_; ++i)
  {
    const auto& axis = axes_[static_cast<std::size_t>(i)];
    const double goal = segment_.goal[static_cast<std::size_t>(i)];
    segment_.coeffs[static_cast<std::size_t>(i)] =
        fitQuintic(axis.position, axis.velocity, axis.acceleration, goal, T);
  }

  segment_.T = T;
  segment_.t = 0.0;
  segment_.active = true;
}

Eigen::VectorXd JointStreamInterpolator::step(const Eigen::VectorXd& target, const double dt_s)
{
  const double dt = std::max(dt_s, kMinDtS);
  Eigen::VectorXd out = Eigen::VectorXd::Zero(n_joints_);

  if (goalChanged(target))
  {
    planSegment(target, dt);
  }

  if (!segment_.active)
  {
    for (int i = 0; i < n_joints_; ++i)
    {
      const double raw_goal = i < target.size() ? target[i] : axes_[static_cast<std::size_t>(i)].position;
      const double goal = clampPosition(i, raw_goal);
      axes_[static_cast<std::size_t>(i)].position = goal;
      axes_[static_cast<std::size_t>(i)].velocity = 0.0;
      axes_[static_cast<std::size_t>(i)].acceleration = 0.0;
      out[i] = goal;
    }
    return out;
  }

  segment_.t = std::min(segment_.t + dt, segment_.T);
  const bool finished = segment_.t >= segment_.T - 1e-12;
  bool step_cap_clipped = false;
  bool position_limit_clipped = false;

  for (int i = 0; i < n_joints_; ++i)
  {
    auto& axis = axes_[static_cast<std::size_t>(i)];
    const double goal = segment_.goal[static_cast<std::size_t>(i)];

    if (finished)
    {
      axis.position = goal;
      axis.velocity = 0.0;
      axis.acceleration = 0.0;
      out[i] = goal;
      continue;
    }

    double x = axis.position;
    double v = 0.0;
    double a = 0.0;
    double j = 0.0;
    evalQuintic(segment_.coeffs[static_cast<std::size_t>(i)], segment_.T, segment_.t, x, v, a, j);

    double delta = x - axis.position;
    if (max_position_step_deg_ > 0.0)
    {
      delta = clampDouble(delta, -max_position_step_deg_, max_position_step_deg_);
      x = axis.position + delta;
    }

    // Do not cross the goal this tick.
    const double error = goal - axis.position;
    if (error > 0.0)
    {
      x = std::min(x, goal);
    }
    else if (error < 0.0)
    {
      x = std::max(x, goal);
    }

    const double limited = clampPosition(i, x);
    if (std::abs(limited - x) > 1e-12)
    {
      x = limited;
      position_limit_clipped = true;
    }
    delta = x - axis.position;

    if (std::abs(goal - x) < kPositionToleranceDeg)
    {
      axis.position = goal;
      axis.velocity = 0.0;
      axis.acceleration = 0.0;
    }
    else if (position_limit_clipped ||
             (max_position_step_deg_ > 0.0 && std::abs(delta) >= max_position_step_deg_ - 1e-9))
    {
      // Soft limit or step cap clipped the committed profile — keep state
      // consistent; replan next tick.
      axis.position = x;
      axis.velocity = delta / dt;
      axis.acceleration = 0.0;
      step_cap_clipped = true;
    }
    else
    {
      axis.position = x;
      axis.velocity = v;
      axis.acceleration = a;
    }

    out[i] = axis.position;
  }

  if (finished || step_cap_clipped)
  {
    clearSegment();
  }

  return out;
}

std::vector<double> JointStreamInterpolator::defaultVelocityLimits(const int n_joints, const double fallback_deg_s)
{
  return std::vector<double>(static_cast<std::size_t>(std::max(n_joints, 1)), fallback_deg_s);
}

}  // namespace fanuc_client
