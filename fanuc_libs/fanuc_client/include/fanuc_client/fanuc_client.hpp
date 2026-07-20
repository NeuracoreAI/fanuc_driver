// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "fanuc_client/gpio_buffer.hpp"
#include "fanuc_client/joint_stream_interpolator.hpp"
#include "rmi/rmi.hpp"
#include "stream_motion/stream.hpp"

namespace fanuc_client
{

class JointCommandPlotter;

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
  explicit FanucClient(std::string robot_ip, uint16_t stream_motion_port = 60015, uint16_t rmi_port = 16001,
                       std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_interface = nullptr,
                       std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection_interface = nullptr);

  FanucClient(const FanucClient&) = delete;
  FanucClient& operator=(const FanucClient&) = delete;

  ~FanucClient();

  void writeJointTarget(const Eigen::VectorXd& joint_targets);

  void writeJointTargetRMI(const Eigen::VectorXd& joint_targets);

  Eigen::Ref<const Eigen::VectorXd> readJointAngles();

  Eigen::Ref<const Eigen::VectorXd> readJointAnglesRMI();

  bool sendIOCommand() const;

  // Throws if it fails to start real-time communication
  void startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer = nullptr);

  void stopRealtimeStream();

  void stopStreaming();

  bool isStreaming();

  void startRMI();

  bool startMotionControl();

  void stopMotionControl();

  bool getDoMotnCtrl() const
  {
    return do_motn_ctrl_;
  }

  void setDoMotnCtrl(bool do_motn_ctrl);

  bool getLimits(double v_peak, double payload, std::vector<double>& vel_limit, std::vector<double>& acc_limit,
                 std::vector<double>& jerk_limit) const;

  /**
   * Base v_peak used to index robot threshold tables for the stream OTG.
   * Each control cycle uses ``getStreamVPeak() * safety_scale`` from the status packet.
   * Valid range roughly 100..2000 (FANUC table scale).
   */
  void setStreamVPeak(double v_peak);
  double getStreamVPeak() const;

  /**
   * Hard |Δq| cap (degrees) applied to each OTG command cycle.
   * Set <= 0 to disable the absolute step cap (not recommended).
   */
  void setMaxCommandStepDeg(double max_step_deg);
  double getMaxCommandStepDeg() const;

  /**
   * Per-axis soft position envelope (degrees) for the stream command.
   * Empty vectors disable clamping.  Prevents MOTN-017 when goals exceed
   * controller soft limits that are tighter than the URDF.
   */
  void setJointPositionLimits(const std::vector<double>& lower_deg, const std::vector<double>& upper_deg);

  uint32_t getControlPeriod() const;

  void setPayloadSchedule(uint8_t payload_schedule) const;

  /** Send FRC_Reset over RMI — clears controller faults (same as TP RESET). */
  void resetController() const;

  /** Read active controller alarm text via RMI (FRC_ReadError). */
  std::string readControllerErrors() const;

  /** Re-seed the command buffer from measured joints. */
  void resetStreamCommandToMeasured();

  /**
   * Enable/disable the live C++ ImPlot window of pre-socket joint commands.
   * RT thread only try_enqueues samples; a separate thread renders.
   */
  void setCommandPlotEnabled(bool enabled);

  bool isCommandPlotEnabled() const
  {
    return command_plot_enabled_.load(std::memory_order_acquire);
  }

  void validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const;

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

  /** Previous signal handler to restore */
  static struct sigaction previous_sigaction_;

private:
  void readStateFromQueue();

  void streamMotionThread(const Eigen::VectorXd& joint_angles);

  /** Grab the limits from the robot.*/
  void fetchRobotLimits();

  /** Refresh OTG limits from cached threshold tables (override + payload). */
  void refreshStreamInterpolatorLimits(double v_peak, double payload);

  Eigen::VectorXd commandPoseForStream(const Eigen::VectorXd& measured, double dt_s);

  /** Non-blocking enqueue of command + measured for the plotter (no-op if disabled). */
  void recordOutgoingCommandForPlot(const Eigen::VectorXd& measured);

  const std::string robot_ip_;
  const uint16_t stream_motion_port_;
  const uint16_t rmi_port_;

  // Limits
  Eigen::MatrixXd vel_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd vel_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);

  // Manages stream motion connection
  std::atomic<bool> is_streaming_ = false;
  std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_;

  std::array<double, stream_motion::kMaxAxisNumber> command_pos;
  std::mutex command_mutex_;
  Eigen::VectorXd command_target_ = Eigen::VectorXd::Zero(9);
  bool command_target_valid_{ false };
  JointStreamInterpolator stream_interpolator_{ stream_motion::kMaxAxisNumber };
  int prime_remaining_{ 0 };
  static constexpr int kStreamHoldPrimeCycles = 8;
  /** Safety scale applied on top of robot threshold-table limits. */
  static constexpr double kInterpolationSafetyScale = 0.8;
  /** Default threshold-table index scale (FANUC tables top out near 2000). */
  static constexpr double kDefaultStreamVPeak = 400.0;
  /** Default hard |Δq| cap (deg) per control period. */
  static constexpr double kDefaultMaxCommandStepDeg = 2.0;
  std::atomic<double> stream_v_peak_{ kDefaultStreamVPeak };
  std::atomic<double> max_command_step_deg_{ kDefaultMaxCommandStepDeg };
  double stream_limit_payload_{ 0.0 };
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

  bool do_motn_ctrl_ = true;

  // Manages RMI connection
  std::shared_ptr<rmi::RMIConnectionInterface> rmi_connection_;
  std::atomic<bool> rmi_running_ = false;

  // Force sensor default type
  uint32_t force_sensor_type_;

  // Live pre-socket command plotter (optional; see FANUC_CLIENT_COMMAND_PLOT).
  std::atomic<bool> command_plot_enabled_{ false };
  std::chrono::steady_clock::time_point command_plot_t0_{};
#if defined(FANUC_CLIENT_COMMAND_PLOT)
  std::unique_ptr<JointCommandPlotter> command_plotter_;
#endif

  struct PQueueImpl;
  std::unique_ptr<PQueueImpl> p_queue_impl_;
};

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
