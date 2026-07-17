// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Core>
#include <vector>

namespace fanuc_client
{

/**
 * Synchronized multi-joint S-curve interpolator for Stream Motion (J519).
 *
 * Uses the standard quintic smoothstep S-curve for Fanuc Stream Motion:
 *
 *   s(τ) = 6τ⁵ − 15τ⁴ + 10τ³ ,  τ ∈ [0, 1]
 *
 * which is the rest-to-rest case of a C² quintic.  A segment is *committed*
 * when the goal is accepted: subsequent ticks advance along that quintic
 * instead of re-planning every cycle (which under-sizes T mid-move and spikes
 * jerk).  When the goal changes (teleop), a new segment is fit from the
 * current (q, q̇, q̈) to (goal, 0, 0) over a shared duration T.
 *
 * Duration T is chosen from per-axis vel/acc/jerk limits and verified by
 * sampling peak derivatives so non-rest retargets stay inside the FANUC
 * threshold tables.  An optional hard |Δq| cap per cycle remains as a Stream
 * Motion safety net.
 */
class JointStreamInterpolator
{
public:
  explicit JointStreamInterpolator(int n_joints = 9);

  /** Align command state to *joints* and clear any active segment. */
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

  double maxPositionStepDeg() const
  {
    return max_position_step_deg_;
  }

  /** Advance one control cycle toward *target* and return the command pose (deg). */
  Eigen::VectorXd step(const Eigen::VectorXd& target, double dt_s);

  /** Current command position (deg). */
  Eigen::VectorXd command() const;

  static std::vector<double> defaultVelocityLimits(int n_joints, double fallback_deg_s = 120.0);

  /** Quintic smoothstep s(τ)=6τ⁵−15τ⁴+10τ³. */
  static double smoothstepQuintic(double tau);

private:
  struct AxisState
  {
    double position{ 0.0 };
    double velocity{ 0.0 };
    double acceleration{ 0.0 };
    double max_velocity{ 120.0 };
    double max_acceleration{ 1200.0 };
    double max_jerk{ 12000.0 };
  };

  struct QuinticCoeffs
  {
    double c0{ 0.0 };
    double c1{ 0.0 };
    double c2{ 0.0 };
    double c3{ 0.0 };
    double c4{ 0.0 };
    double c5{ 0.0 };
  };

  struct Segment
  {
    bool active{ false };
    double T{ 0.0 };
    double t{ 0.0 };
    std::vector<double> goal;
    std::vector<QuinticCoeffs> coeffs;
  };

  /** Minimum duration so rest-to-rest quintic peaks stay under axis limits. */
  static double durationRestToRest(double dq_deg, double vmax, double amax, double jmax, double dt);

  /**
   * Duration for a quintic from (x0,v0,a0) → (xf,0,0) that keeps sampled peak
   * |v|/|a|/|j| inside the axis limits (starts from rest-to-rest + brake heuristics).
   */
  static double durationForAxis(double x0, double v0, double a0, double xf, double vmax, double amax, double jmax,
                                double dt);

  /** True if sampled peaks of the quintic stay within limits. */
  static bool peaksWithinLimits(const QuinticCoeffs& c, double T, double vmax, double amax, double jmax);

  /** Fit quintic from (pos,vel,acc) to (xf, 0, 0) over duration T (τ∈[0,1]). */
  static QuinticCoeffs fitQuintic(double x0, double v0, double a0, double xf, double T);

  /** Evaluate quintic position / velocity / acceleration / jerk at time t ∈ [0, T]. */
  static void evalQuintic(const QuinticCoeffs& c, double T, double t, double& x, double& v, double& a, double& j);

  bool goalChanged(const Eigen::VectorXd& target) const;
  void clearSegment();
  void planSegment(const Eigen::VectorXd& target, double dt);

  int n_joints_;
  std::vector<AxisState> axes_;
  Segment segment_;
  double max_position_step_deg_{ 2.0 };
};

}  // namespace fanuc_client
