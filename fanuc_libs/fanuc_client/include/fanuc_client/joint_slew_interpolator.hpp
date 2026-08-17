// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Core>
#include <optional>
#include <vector>

namespace fanuc_client
{

/**
 * Accel-limited joint-space slewer for Stream Motion command shaping.
 *
 * Each step() advances an open-loop command toward the latest goal under
 * per-axis velocity and acceleration limits so Stream Motion never sees a
 * raw joint jump.
 */
class JointSlewInterpolator
{
public:
  explicit JointSlewInterpolator(int n_joints = 9);

  void setLimits(double max_vel_deg_s, double max_acc_deg_s2);

  void setLimits(const std::vector<double>& max_vel_deg_s, const std::vector<double>& max_acc_deg_s2);

  /** Seed command and target; clear command velocity. */
  void reset(const Eigen::VectorXd& joints_deg);

  /** Sticky goal chased by subsequent step() calls. */
  void setTarget(const Eigen::VectorXd& joints_deg);

  Eigen::VectorXd command() const
  {
    return cmd_;
  }

  Eigen::VectorXd target() const
  {
    return target_;
  }

  /** Advance one tick toward the goal; return the new command (degrees). */
  Eigen::VectorXd step(double dt_s);

private:
  Eigen::VectorXd asJointArray(const std::vector<double>& value, double fallback) const;

  int n_;
  bool initialized_ = false;
  Eigen::VectorXd cmd_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd target_;
  Eigen::VectorXd max_vel_;
  std::optional<Eigen::VectorXd> max_acc_;
};

}  // namespace fanuc_client
