// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_client/joint_stream_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <ruckig/ruckig.hpp>

namespace fanuc_client
{
namespace
{
constexpr double kMinDtS = 1e-4;
constexpr double kDefaultDtS = 0.008;

double clampDouble(const double value, const double lo, const double hi)
{
  return std::max(lo, std::min(value, hi));
}

}  // namespace

struct JointStreamInterpolator::Impl
{
  using OTG = ruckig::Ruckig<ruckig::DynamicDOFs>;
  using Input = ruckig::InputParameter<ruckig::DynamicDOFs>;
  using Output = ruckig::OutputParameter<ruckig::DynamicDOFs>;

  explicit Impl(const int n_joints)
    : n_joints(std::max(n_joints, 1))
    , dt_s(kDefaultDtS)
    , otg(std::make_unique<OTG>(static_cast<size_t>(this->n_joints), kDefaultDtS))
    , input(static_cast<size_t>(this->n_joints))
    , output(static_cast<size_t>(this->n_joints))
    , max_velocity(static_cast<size_t>(this->n_joints), 120.0)
    , max_acceleration(static_cast<size_t>(this->n_joints), 1200.0)
    , max_jerk(static_cast<size_t>(this->n_joints), 12000.0)
  {
    for (int i = 0; i < this->n_joints; ++i)
    {
      input.current_position[static_cast<size_t>(i)] = 0.0;
      input.current_velocity[static_cast<size_t>(i)] = 0.0;
      input.current_acceleration[static_cast<size_t>(i)] = 0.0;
      input.target_position[static_cast<size_t>(i)] = 0.0;
      input.target_velocity[static_cast<size_t>(i)] = 0.0;
      input.target_acceleration[static_cast<size_t>(i)] = 0.0;
      input.max_velocity[static_cast<size_t>(i)] = max_velocity[static_cast<size_t>(i)];
      input.max_acceleration[static_cast<size_t>(i)] = max_acceleration[static_cast<size_t>(i)];
      input.max_jerk[static_cast<size_t>(i)] = max_jerk[static_cast<size_t>(i)];
    }
  }

  void ensureDt(const double dt)
  {
    if (std::abs(dt - dt_s) < 1e-9)
    {
      return;
    }
    dt_s = dt;
    otg = std::make_unique<OTG>(static_cast<size_t>(n_joints), dt_s);
  }

  void syncLimitsToInput()
  {
    for (int i = 0; i < n_joints; ++i)
    {
      input.max_velocity[static_cast<size_t>(i)] = max_velocity[static_cast<size_t>(i)];
      input.max_acceleration[static_cast<size_t>(i)] = max_acceleration[static_cast<size_t>(i)];
      input.max_jerk[static_cast<size_t>(i)] = max_jerk[static_cast<size_t>(i)];
    }
  }

  double clampPosition(const int i, const double q) const
  {
    if (!position_limits_enabled)
    {
      return q;
    }
    return clampDouble(q, position_lower[static_cast<size_t>(i)], position_upper[static_cast<size_t>(i)]);
  }

  int n_joints;
  double dt_s;
  double max_position_step_deg{ 2.0 };
  bool position_limits_enabled{ false };
  std::unique_ptr<OTG> otg;
  Input input;
  Output output;
  std::vector<double> max_velocity;
  std::vector<double> max_acceleration;
  std::vector<double> max_jerk;
  std::vector<double> position_lower;
  std::vector<double> position_upper;
};

JointStreamInterpolator::JointStreamInterpolator(const int n_joints)
  : impl_(std::make_unique<Impl>(n_joints))
{
}

JointStreamInterpolator::~JointStreamInterpolator() = default;

JointStreamInterpolator::JointStreamInterpolator(JointStreamInterpolator&&) noexcept = default;
JointStreamInterpolator& JointStreamInterpolator::operator=(JointStreamInterpolator&&) noexcept = default;

void JointStreamInterpolator::reset(const Eigen::VectorXd& joints)
{
  const int count = std::min(static_cast<int>(joints.size()), impl_->n_joints);
  for (int i = 0; i < count; ++i)
  {
    impl_->input.current_position[static_cast<size_t>(i)] = joints[i];
    impl_->input.current_velocity[static_cast<size_t>(i)] = 0.0;
    impl_->input.current_acceleration[static_cast<size_t>(i)] = 0.0;
    impl_->input.target_position[static_cast<size_t>(i)] = joints[i];
    impl_->input.target_velocity[static_cast<size_t>(i)] = 0.0;
    impl_->input.target_acceleration[static_cast<size_t>(i)] = 0.0;
  }
  for (int i = count; i < impl_->n_joints; ++i)
  {
    impl_->input.current_velocity[static_cast<size_t>(i)] = 0.0;
    impl_->input.current_acceleration[static_cast<size_t>(i)] = 0.0;
    impl_->input.target_velocity[static_cast<size_t>(i)] = 0.0;
    impl_->input.target_acceleration[static_cast<size_t>(i)] = 0.0;
  }
  impl_->otg->reset();
}

void JointStreamInterpolator::setLimits(const std::vector<double>& max_velocity_deg_s,
                                        const std::vector<double>& max_acceleration_deg_s2,
                                        const std::vector<double>& max_jerk_deg_s3, const double safety_scale)
{
  const double scale = std::max(safety_scale, 1e-3);
  for (int i = 0; i < impl_->n_joints; ++i)
  {
    const double vmax =
        i < static_cast<int>(max_velocity_deg_s.size()) ? max_velocity_deg_s[static_cast<size_t>(i)] :
                                                          impl_->max_velocity[static_cast<size_t>(i)];
    const double amax = i < static_cast<int>(max_acceleration_deg_s2.size()) ?
                            max_acceleration_deg_s2[static_cast<size_t>(i)] :
                            impl_->max_acceleration[static_cast<size_t>(i)];
    const double jmax = i < static_cast<int>(max_jerk_deg_s3.size()) ? max_jerk_deg_s3[static_cast<size_t>(i)] :
                                                                       impl_->max_jerk[static_cast<size_t>(i)];
    impl_->max_velocity[static_cast<size_t>(i)] = std::max(vmax * scale, 1e-3);
    impl_->max_acceleration[static_cast<size_t>(i)] = std::max(amax * scale, 1e-3);
    impl_->max_jerk[static_cast<size_t>(i)] = std::max(jmax * scale, 1e-3);
  }
  impl_->syncLimitsToInput();
  // Limits changed — force a fresh profile on the next update.
  impl_->otg->reset();
}

void JointStreamInterpolator::setMaxPositionStepDeg(const double max_step_deg)
{
  impl_->max_position_step_deg = max_step_deg;
}

double JointStreamInterpolator::maxPositionStepDeg() const
{
  return impl_->max_position_step_deg;
}

void JointStreamInterpolator::setPositionLimits(const std::vector<double>& lower_deg,
                                                const std::vector<double>& upper_deg)
{
  if (lower_deg.empty() || upper_deg.empty())
  {
    impl_->position_limits_enabled = false;
    impl_->position_lower.clear();
    impl_->position_upper.clear();
    return;
  }

  impl_->position_lower.assign(static_cast<size_t>(impl_->n_joints), -1.0e9);
  impl_->position_upper.assign(static_cast<size_t>(impl_->n_joints), 1.0e9);
  for (int i = 0; i < impl_->n_joints; ++i)
  {
    const double lo = i < static_cast<int>(lower_deg.size()) ? lower_deg[static_cast<size_t>(i)] : -1.0e9;
    const double hi = i < static_cast<int>(upper_deg.size()) ? upper_deg[static_cast<size_t>(i)] : 1.0e9;
    impl_->position_lower[static_cast<size_t>(i)] = std::min(lo, hi);
    impl_->position_upper[static_cast<size_t>(i)] = std::max(lo, hi);
  }
  impl_->position_limits_enabled = true;
}

Eigen::VectorXd JointStreamInterpolator::command() const
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(impl_->n_joints);
  for (int i = 0; i < impl_->n_joints; ++i)
  {
    out[i] = impl_->input.current_position[static_cast<size_t>(i)];
  }
  return out;
}

Eigen::VectorXd JointStreamInterpolator::step(const Eigen::VectorXd& target, const double dt_s)
{
  const double dt = std::max(dt_s, kMinDtS);
  impl_->ensureDt(dt);
  impl_->syncLimitsToInput();

  for (int i = 0; i < impl_->n_joints; ++i)
  {
    const double raw_goal = i < target.size() ? target[i] : impl_->input.current_position[static_cast<size_t>(i)];
    const double goal = impl_->clampPosition(i, raw_goal);
    impl_->input.target_position[static_cast<size_t>(i)] = goal;
    impl_->input.target_velocity[static_cast<size_t>(i)] = 0.0;
    impl_->input.target_acceleration[static_cast<size_t>(i)] = 0.0;
  }

  const ruckig::Result result = impl_->otg->update(impl_->input, impl_->output);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
  {
    static bool warned = false;
    if (!warned)
    {
      std::cerr << "JointStreamInterpolator: Ruckig update failed (code " << static_cast<int>(result)
                << "); holding command" << std::endl;
      warned = true;
    }
    return command();
  }

  Eigen::VectorXd out = Eigen::VectorXd::Zero(impl_->n_joints);
  bool state_overridden = false;

  for (int i = 0; i < impl_->n_joints; ++i)
  {
    const double prev = impl_->input.current_position[static_cast<size_t>(i)];
    const double otg_pos = impl_->output.new_position[static_cast<size_t>(i)];
    double next = otg_pos;
    double vel = impl_->output.new_velocity[static_cast<size_t>(i)];
    double acc = impl_->output.new_acceleration[static_cast<size_t>(i)];

    if (impl_->max_position_step_deg > 0.0)
    {
      const double delta = clampDouble(next - prev, -impl_->max_position_step_deg, impl_->max_position_step_deg);
      next = prev + delta;
      if (std::abs(next - otg_pos) > 1e-12)
      {
        const double prev_vel = impl_->input.current_velocity[static_cast<size_t>(i)];
        vel = delta / dt;
        acc = (vel - prev_vel) / dt;
        state_overridden = true;
      }
    }

    const double limited = impl_->clampPosition(i, next);
    if (std::abs(limited - next) > 1e-12)
    {
      const double prev_vel = impl_->input.current_velocity[static_cast<size_t>(i)];
      const double delta = limited - prev;
      next = limited;
      vel = delta / dt;
      acc = (vel - prev_vel) / dt;
      state_overridden = true;
    }

    out[i] = next;
    impl_->input.current_position[static_cast<size_t>(i)] = next;
    impl_->input.current_velocity[static_cast<size_t>(i)] = vel;
    impl_->input.current_acceleration[static_cast<size_t>(i)] = acc;
  }

  if (state_overridden)
  {
    // Capped / soft-limited state no longer matches Ruckig's last profile — replan next tick.
    impl_->otg->reset();
  }
  else
  {
    impl_->output.pass_to_input(impl_->input);
  }

  return out;
}

std::vector<double> JointStreamInterpolator::defaultVelocityLimits(const int n_joints, const double fallback_deg_s)
{
  return std::vector<double>(static_cast<std::size_t>(std::max(n_joints, 1)), fallback_deg_s);
}

}  // namespace fanuc_client
