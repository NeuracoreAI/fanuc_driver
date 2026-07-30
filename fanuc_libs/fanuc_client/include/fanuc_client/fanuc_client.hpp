// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "fanuc_client/gpio_buffer.hpp"
#include "fanuc_client/joint_slew_interpolator.hpp"
#include "rmi/rmi.hpp"
#include "stream_motion/stream.hpp"

namespace fanuc_client
{

enum class ContactStopMode
{
  None = 0,
  SAFE = 1,
  STOP = 2,
  DSBL = 3,
  ESCP = 4,
};

struct RobotStatus
{
  bool in_error;
  bool tp_enabled;
  bool e_stopped;
  bool motion_possible;
  ContactStopMode contact_stop_mode;
  float safety_scale;
};

struct ForceSensor
{
  float force_x;
  float force_y;
  float force_z;
  float moment_x;
  float moment_y;
  float moment_z;
  uint32_t fs_type;
};

class FanucClient
{
public:
  FanucClient() = delete;
  /** Stream Motion only. STREAM_MOTN (IBGN) must already be running on the TP. */
  explicit FanucClient(std::string robot_ip, uint16_t stream_motion_port = 60015,
                       std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_interface = nullptr);

  FanucClient(const FanucClient&) = delete;
  FanucClient& operator=(const FanucClient&) = delete;

  ~FanucClient();

  void writeJointTarget(const Eigen::VectorXd& joint_targets);

  /**
   * Set the sticky joint goal (degrees) chased by the slew feeder thread.
   * Does not enqueue a raw jump — the slewer publishes shaped commands via
   * writeJointTarget into the existing Stream Motion queue.
   */
  void setJointGoal(const Eigen::VectorXd& joint_goal_deg);

  Eigen::VectorXd getJointGoal() const;

  void setStreamMaxVel(double max_vel_deg_s);
  double getStreamMaxVel() const;

  void setStreamMaxAcc(double max_acc_deg_s2);
  double getStreamMaxAcc() const;

  /** Soft position envelope (degrees) applied to slewed commands. Empty disables. */
  void setJointPositionLimits(const std::vector<double>& lower_deg, const std::vector<double>& upper_deg);

  /** Re-seed sticky goal + slewer from the latest measured joints. */
  void resetStreamCommandToMeasured();

  /** Re-seed sticky goal + slewer from an explicit pose (e.g. hold on disarm). */
  void resetStreamCommand(const Eigen::VectorXd& joints_deg);

  Eigen::Ref<const Eigen::VectorXd> readJointAngles();

  bool sendIOCommand() const;

  // Throws if it fails to start real-time communication
  void startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer = nullptr);

  void stopRealtimeStream();

  void stopStreaming();

  bool isStreaming();

  /**
   * Check that Stream Motion reports motion_possible (STREAM_MOTN / IBGN ready).
   * Does not start or stop the TP program — operator keeps STREAM_MOTN running.
   */
  bool startMotionControl();

  /** Disarm motion control locally (do_motn_ctrl=false). Leaves STREAM_MOTN running. */
  void stopMotionControl();

  bool getDoMotnCtrl() const
  {
    return do_motn_ctrl_.load(std::memory_order_relaxed);
  }

  void setDoMotnCtrl(bool do_motn_ctrl);

  bool getLimits(double v_peak, double payload, std::vector<double>& vel_limit, std::vector<double>& acc_limit,
                 std::vector<double>& jerk_limit) const;

  uint32_t getControlPeriod() const;

  void validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const;

  void setOutCmdInterpBuffTarget(uint32_t out_cmd_interp_buff_target)
  {
    out_cmd_interp_buff_target_ = out_cmd_interp_buff_target;
  }

  uint32_t getOutCmdInterpBuffTarget() const
  {
    return out_cmd_interp_buff_target_;
  }

  void setForceSensorType(uint32_t force_sensor_type)
  {
    force_sensor_type_ = force_sensor_type;
  }

  uint32_t getForceSensorType() const
  {
    return force_sensor_type_;
  }

  const RobotStatus& robot_status() const
  {
    return robot_status_;
  }

  const ForceSensor& force_sensor() const
  {
    return force_sensor_;
  }

  void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const;

  // Get static instance
  static FanucClient* get_instance()
  {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_;
  }

  // Get client (stream motion) version
  uint32_t getClientVersion() const
  {
    return client_version_;
  }

private:
  /** Setup signal handler for SIGINT */
  void setupSignalHandler();

  /** Restore previous signal handler */
  void restoreSignalHandler();

  /** Static signal handler function */
  static void signalHandler(int signal);

  /** Static pointer to current instance for signal handler */
  static FanucClient* instance_;

  /** Mutex to protect instance_ access from signal handler */
  static std::mutex instance_mutex_;

  /** Previous SIGINT handler to restore (portable; avoids POSIX sigaction) */
  static void (*previous_signal_handler_)(int);

private:
  void readStateFromQueue();

  void streamMotionThread(const Eigen::VectorXd& joint_angles);

  /** Accel-limited feeder that calls writeJointTarget into the existing queue. */
  void jointSlewThread();

  void applySlewLimitsLocked();

  Eigen::VectorXd clampToPositionLimits(const Eigen::VectorXd& joints) const;

  /** Grab the limits from the robot.*/
  void fetchRobotLimits();

  const std::string robot_ip_;
  const uint16_t stream_motion_port_;

  // Limits
  Eigen::MatrixXd vel_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd vel_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);

  // Manages stream motion connection
  std::atomic<bool> is_streaming_ = false;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_;

  std::array<double, stream_motion::kMaxAxisNumber> command_pos;
  Eigen::VectorXd last_joint_angles_ = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd last_joint_angles_cmd_ = Eigen::VectorXd::Zero(9);
  RobotStatus robot_status_;
  ForceSensor force_sensor_;
  bool in_motion_ = false;
  uint32_t control_period_ = 0;
  uint32_t client_version_ = 0;  // stream motion client version

  // IO data only accessed from the non-realitime thread.
  std::shared_ptr<GPIOBuffer> gpio_buffer_;

  // Real time thread data
  std::thread rt_thread_;
  std::thread slew_thread_;
  std::atomic<bool> slew_running_{ false };

  std::atomic<bool> do_motn_ctrl_{ false };

  // Python-tuned slew limits (defaults match example_fanuc configs).
  static constexpr double kDefaultStreamMaxVelDegS = 60.0;
  static constexpr double kDefaultStreamMaxAccDegS2 = 300.0;
  static constexpr double kSlewRateMultiplier = 4.0;
  std::atomic<double> stream_max_vel_deg_s_{ kDefaultStreamMaxVelDegS };
  std::atomic<double> stream_max_acc_deg_s2_{ kDefaultStreamMaxAccDegS2 };

  mutable std::mutex slew_mutex_;
  JointSlewInterpolator stream_slew_{ stream_motion::kMaxAxisNumber };
  Eigen::VectorXd joint_goal_ = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  std::vector<double> joint_pos_lower_deg_;
  std::vector<double> joint_pos_upper_deg_;

  // Output command interpolation buffer target size for stream motion control
  uint32_t out_cmd_interp_buff_target_;

  // Force sensor default type
  uint32_t force_sensor_type_;

  struct PQueueImpl;
  std::unique_ptr<PQueueImpl> p_queue_impl_;
};

/** Retained for ROS GPIO / hardware_interface callers; FanucClient no longer uses RMI. */
class RMISingleton
{
public:
  static std::shared_ptr<rmi::RMIConnectionInterface> creatNewRMIInstance(const std::string& robot_ip_address,
                                                                          uint16_t rmi_port = 16001);

  static std::shared_ptr<rmi::RMIConnectionInterface> getRMIInstance();

  static std::shared_ptr<rmi::RMIConnectionInterface>
  setRMIInstance(std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection);

private:
  RMISingleton();

  ~RMISingleton();

  RMISingleton(const RMISingleton&) = delete;
  RMISingleton& operator=(const RMISingleton&) = delete;

  static RMISingleton& getInstance();
  std::mutex mtx_;

  std::shared_ptr<rmi::RMIConnectionInterface> rmi_connection_interface_;
};

}  // namespace fanuc_client
