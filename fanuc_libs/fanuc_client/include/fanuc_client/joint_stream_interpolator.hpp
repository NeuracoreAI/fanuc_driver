// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace fanuc_client
{

/**
 * Online Trajectory Generator (OTG) for Stream Motion joint commands.
 *
 * Every control cycle replans from the current kinematic state (q, q̇, q̈) to the
 * latest joint goal under per-axis velocity, acceleration, and jerk limits
 * (Ruckig).  Goals may change at any time — near or far, stop, reverse — while
 * position/velocity/acceleration stay continuous.
 *
 * An optional hard |Δq| cap per cycle remains as a Stream Motion safety net.
 */
class JointStreamInterpolator
{
public:
  explicit JointStreamInterpolator(int n_joints = 9);
  ~JointStreamInterpolator();

  JointStreamInterpolator(const JointStreamInterpolator&) = delete;
  JointStreamInterpolator& operator=(const JointStreamInterpolator&) = delete;
  JointStreamInterpolator(JointStreamInterpolator&&) noexcept;
  JointStreamInterpolator& operator=(JointStreamInterpolator&&) noexcept;

  /** Align command state to *joints* (position only; velocity/accel cleared). */
  void reset(const Eigen::VectorXd& joints);

  /** Set per-axis limits (deg/s, deg/s², deg/s³). */
  void setLimits(const std::vector<double>& max_velocity_deg_s,
                 const std::vector<double>& max_acceleration_deg_s2,
                 const std::vector<double>& max_jerk_deg_s3, double safety_scale = 0.8);

  /**
   * Hard cap on |Δq| per control cycle (degrees).
   * Set <= 0 to disable (not recommended for Stream Motion).
   */
  void setMaxPositionStepDeg(double max_step_deg);

  double maxPositionStepDeg() const;

  /**
   * Per-axis soft position envelope (degrees).  Goals and command samples are
   * clamped into [lower, upper] to avoid MOTN-017 soft-limit faults.
   * Pass empty vectors (or call with both empty) to disable.
   */
  void setPositionLimits(const std::vector<double>& lower_deg, const std::vector<double>& upper_deg);

  /** Advance one control cycle toward *target* and return the command pose (deg). */
  Eigen::VectorXd step(const Eigen::VectorXd& target, double dt_s);

  /** Current command position (deg). */
  Eigen::VectorXd command() const;

  static std::vector<double> defaultVelocityLimits(int n_joints, double fallback_deg_s = 120.0);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fanuc_client
