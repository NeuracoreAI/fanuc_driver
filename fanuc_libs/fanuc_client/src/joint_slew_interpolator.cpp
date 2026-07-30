// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_client/joint_slew_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fanuc_client
{

JointSlewInterpolator::JointSlewInterpolator(const int n_joints)
  : n_{ n_joints }
  , cmd_{ Eigen::VectorXd::Zero(n_joints) }
  , vel_{ Eigen::VectorXd::Zero(n_joints) }
  , target_{ Eigen::VectorXd::Zero(n_joints) }
  , max_vel_{ Eigen::VectorXd::Constant(n_joints, 60.0) }
  , max_acc_{ Eigen::VectorXd::Constant(n_joints, 300.0) }
{
}

void JointSlewInterpolator::setLimits(const double max_vel_deg_s, const double max_acc_deg_s2)
{
  max_vel_ = Eigen::VectorXd::Constant(n_, max_vel_deg_s);
  if (max_acc_deg_s2 <= 0.0)
  {
    max_acc_.reset();
  }
  else
  {
    max_acc_ = Eigen::VectorXd::Constant(n_, max_acc_deg_s2);
  }
}

void JointSlewInterpolator::setLimits(const std::vector<double>& max_vel_deg_s,
                                      const std::vector<double>& max_acc_deg_s2)
{
  max_vel_ = asJointArray(max_vel_deg_s, 60.0);
  if (max_acc_deg_s2.empty())
  {
    max_acc_.reset();
  }
  else
  {
    max_acc_ = asJointArray(max_acc_deg_s2, 300.0);
  }
}

void JointSlewInterpolator::reset(const Eigen::VectorXd& joints_deg)
{
  if (joints_deg.size() != n_)
  {
    throw std::invalid_argument("JointSlewInterpolator::reset size mismatch");
  }
  cmd_ = joints_deg;
  target_ = joints_deg;
  vel_.setZero();
  initialized_ = true;
}

void JointSlewInterpolator::setTarget(const Eigen::VectorXd& joints_deg)
{
  if (joints_deg.size() != n_)
  {
    throw std::invalid_argument("JointSlewInterpolator::setTarget size mismatch");
  }
  if (!initialized_)
  {
    reset(joints_deg);
    return;
  }
  target_ = joints_deg;
}

Eigen::VectorXd JointSlewInterpolator::step(const double dt_s)
{
  if (dt_s <= 0.0 || !initialized_)
  {
    return cmd_;
  }

  Eigen::VectorXd cmd = cmd_;
  Eigen::VectorXd vel = vel_;
  const Eigen::VectorXd& target = target_;
  const Eigen::VectorXd& vel_limit = max_vel_;

  if (!max_acc_.has_value())
  {
    const Eigen::VectorXd step = vel_limit * dt_s;
    const Eigen::VectorXd delta = (target - cmd).cwiseMax(-step).cwiseMin(step);
    cmd = cmd + delta;
    vel.setZero();
  }
  else
  {
    const Eigen::VectorXd& acc_limit = *max_acc_;
    const Eigen::VectorXd dist = target - cmd;
    Eigen::VectorXd v_decel(n_);
    for (int i = 0; i < n_; ++i)
    {
      const double d = dist[i];
      const double a = acc_limit[i];
      v_decel[i] = std::copysign(std::sqrt(std::max(2.0 * a * std::abs(d), 0.0)), d);
    }
    Eigen::VectorXd desired_vel = v_decel.cwiseMax(-vel_limit).cwiseMin(vel_limit);
    const Eigen::VectorXd dv_limit = acc_limit * dt_s;
    const Eigen::VectorXd dv = (desired_vel - vel).cwiseMax(-dv_limit).cwiseMin(dv_limit);
    vel = vel + dv;
    cmd = cmd + vel * dt_s;
  }

  cmd_ = cmd;
  vel_ = vel;
  return cmd_;
}

Eigen::VectorXd JointSlewInterpolator::asJointArray(const std::vector<double>& value, const double fallback) const
{
  if (value.empty())
  {
    return Eigen::VectorXd::Constant(n_, fallback);
  }
  if (value.size() == 1)
  {
    return Eigen::VectorXd::Constant(n_, value[0]);
  }
  if (static_cast<int>(value.size()) != n_)
  {
    throw std::invalid_argument("JointSlewInterpolator limit size mismatch");
  }
  return Eigen::Map<const Eigen::VectorXd>(value.data(), n_);
}

}  // namespace fanuc_client
