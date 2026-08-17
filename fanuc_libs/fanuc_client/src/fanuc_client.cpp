// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_client/fanuc_client.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <utility>

#include "fanuc_client/gpio_buffer.hpp"
#include "readerwriterqueue.h"
#include "stream_motion/packets.hpp"

namespace fanuc_client
{
// Static member initialization
FanucClient* FanucClient::instance_ = nullptr;
std::mutex FanucClient::instance_mutex_;
void (*FanucClient::previous_signal_handler_)(int) = nullptr;

namespace
{
constexpr double kFullPayload = 7.0;
constexpr auto kStatusPacketFailureMessage = "Invalid robot status packet. Make sure the robot connected can be "
                                             "reached on the network and is in a running state.";
constexpr auto kStatusStatusNotReadyMessage = "Stream motion control is not ready. Check if STREAM_MOTN is running "
                                              "on the teach pendant and the robot has no alarms.";

void AssertIsStreaming(const std::atomic<bool>& is_streaming)
{
  if (!is_streaming)
  {
    throw std::runtime_error(
        "Robot is not streaming. Please ensure the real-time stream is running before stream motion command.");
  }
}

void AssertNotStreaming(const std::atomic<bool>& is_streaming)
{
  if (is_streaming)
  {
    throw std::runtime_error(
        "Robot is currently streaming. Non-stream commands cannot be issued when stream motion is active.");
  }
}

constexpr ContactStopMode ToContactStopMode(stream_motion::ContactStopStatus status)
{
  using ::stream_motion::ContactStopStatus;
  switch (status)
  {
    case ContactStopStatus::SAFE:
      return ContactStopMode::SAFE;
    case ContactStopStatus::STOP:
      return ContactStopMode::STOP;
    case ContactStopStatus::DSBL:
      return ContactStopMode::DSBL;
    case ContactStopStatus::ESCP:
      return ContactStopMode::ESCP;
    case ContactStopStatus::None:
      return ContactStopMode::None;
  }
  return ContactStopMode::None;
}

}  // namespace

struct FanucClient::PQueueImpl
{
  using StampedEigen = std::tuple<std::chrono::duration<double>, Eigen::VectorXd>;
  // TODO: Consider merging the command and command_io queues.
  moodycamel::BlockingReaderWriterQueue<StampedEigen> command_queue_;
  moodycamel::BlockingReaderWriterQueue<std::array<uint8_t, 256>> command_io_queue_;
  moodycamel::BlockingReaderWriterQueue<stream_motion::RobotStatusPacket> robot_state_queue_;
};

FanucClient::FanucClient(std::string robot_ip, const uint16_t stream_motion_port,
                         std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_interface)
  : robot_ip_{ std::move(robot_ip) }
  , stream_motion_port_{ stream_motion_port }
  , stream_motion_{ stream_motion_interface == nullptr ?
                        std::make_unique<stream_motion::StreamMotionConnection>(robot_ip_, 1.0, stream_motion_port_) :
                        std::move(stream_motion_interface) }
  , command_pos{}
  , out_cmd_interp_buff_target_{ 8 }
  , force_sensor_type_{ 0 }
  , p_queue_impl_(std::make_unique<PQueueImpl>())
{
  stream_motion_->sendStopPacket();
  stream_motion::ControllerCapabilityResultPacket controller_capability;
  if (!stream_motion_->getControllerCapability(controller_capability) ||
      controller_capability.sampling_rate < 1 || controller_capability.sampling_rate > 100)
  {
    std::cerr << "Controller capability unavailable or invalid sampling_rate="
              << controller_capability.sampling_rate << "; defaulting control period to "
              << kDefaultControlPeriodMs << " ms." << std::endl;
    control_period_ = kDefaultControlPeriodMs;
    client_version_ = controller_capability.available_version;
  }
  else
  {
    control_period_ = controller_capability.sampling_rate;
    client_version_ = controller_capability.available_version;
  }
  fetchRobotLimits();

  setupSignalHandler();
}

FanucClient::~FanucClient()
{
  if (is_streaming_)
  {
    try
    {
      std::cout << "Stopping realtime stream during destruction" << std::endl;
      stopRealtimeStream();
    }
    catch (const std::exception& e)
    {
      // During destruction, network might already be down or robot disconnected
      // Log the error but don't throw to avoid std::terminate
      std::cerr << "Warning: Failed to stop realtime stream during FanucClient destruction: " << e.what() << std::endl;
    }
  }
  else
  {
    try
    {
      std::cout << "Sending stop packet during destruction" << std::endl;
      stream_motion_->sendStopPacket();
    }
    catch (const std::exception& e)
    {
      std::cerr << "Warning: Failed to send stop packet during destruction: " << e.what() << std::endl;
    }
    catch (...)
    {
      std::cerr << "Warning: Unknown exception during stop packet send" << std::endl;
    }
  }

  restoreSignalHandler();
}

void FanucClient::readStateFromQueue()
{
  stream_motion::RobotStatusPacket robot_status;
  bool updated = false;
  while (p_queue_impl_->robot_state_queue_.try_dequeue(robot_status))
  {
    updated = true;
  }
  if (!updated)
  {
    return;
  }

  for (Eigen::Index i = 0; i < robot_status.joint_angle.size(); ++i)
  {
    last_joint_angles_[i] = static_cast<double>(robot_status.joint_angle[i]);
  }

  if (gpio_buffer_ != nullptr)
  {
    gpio_buffer_->status_buffer() = robot_status.io_status;
  }

  robot_status_.in_error = robot_status.robot_status & 0x1;
  robot_status_.tp_enabled = robot_status.robot_status & 0x2;
  robot_status_.e_stopped = robot_status.robot_status & 0x4;
  robot_status_.motion_possible = robot_status.status & 0x1;
  robot_status_.contact_stop_mode = ToContactStopMode(robot_status.contact_stop_status);
  robot_status_.safety_scale = robot_status.safety_scale;

  in_motion_ = robot_status.status & 0x8;

  force_sensor_.force_x = robot_status.force_x;
  force_sensor_.force_y = robot_status.force_y;
  force_sensor_.force_z = robot_status.force_z;
  force_sensor_.moment_x = robot_status.moment_x;
  force_sensor_.moment_y = robot_status.moment_y;
  force_sensor_.moment_z = robot_status.moment_z;
  force_sensor_.fs_type = robot_status.fs_type;
}

void FanucClient::writeJointTarget(const Eigen::VectorXd& joint_targets)
{
  AssertIsStreaming(is_streaming_);
  readStateFromQueue();
  if (robot_status_.motion_possible && do_motn_ctrl_.load(std::memory_order_relaxed))
  {
    last_joint_angles_cmd_ = joint_targets;
  }
  else
  {
    last_joint_angles_cmd_ = last_joint_angles_;
  }

  if (joint_targets.size() != last_joint_angles_.size())
  {
    throw std::invalid_argument("Joint targets size does not match the size of last joint angles.");
  }
  auto cur_time_from_start = std::chrono::high_resolution_clock::now() - start_time_;
  p_queue_impl_->command_queue_.enqueue({ cur_time_from_start, last_joint_angles_cmd_ });
}

void FanucClient::setJointGoal(const Eigen::VectorXd& joint_goal_deg)
{
  Eigen::VectorXd goal = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  const Eigen::Index n = std::min<Eigen::Index>(joint_goal_deg.size(), stream_motion::kMaxAxisNumber);
  goal.head(n) = joint_goal_deg.head(n);
  goal = clampToPositionLimits(goal);
  std::lock_guard<std::mutex> lock(slew_mutex_);
  joint_goal_ = goal;
  stream_slew_.setTarget(goal);
}

Eigen::VectorXd FanucClient::getJointGoal() const
{
  std::lock_guard<std::mutex> lock(slew_mutex_);
  return joint_goal_;
}

void FanucClient::setStreamMaxVel(const double max_vel_deg_s)
{
  stream_max_vel_deg_s_.store(std::max(0.1, max_vel_deg_s), std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(slew_mutex_);
  applySlewLimitsLocked();
}

double FanucClient::getStreamMaxVel() const
{
  return stream_max_vel_deg_s_.load(std::memory_order_relaxed);
}

void FanucClient::setStreamMaxAcc(const double max_acc_deg_s2)
{
  stream_max_acc_deg_s2_.store(max_acc_deg_s2, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(slew_mutex_);
  applySlewLimitsLocked();
}

double FanucClient::getStreamMaxAcc() const
{
  return stream_max_acc_deg_s2_.load(std::memory_order_relaxed);
}

void FanucClient::setSlewRateMultiplier(const double multiplier)
{
  slew_rate_multiplier_.store(std::clamp(multiplier, kMinSlewRateMultiplier, kMaxSlewRateMultiplier),
                              std::memory_order_relaxed);
}

double FanucClient::getSlewRateMultiplier() const
{
  return slew_rate_multiplier_.load(std::memory_order_relaxed);
}

void FanucClient::setJointPositionLimits(const std::vector<double>& lower_deg, const std::vector<double>& upper_deg)
{
  std::lock_guard<std::mutex> lock(slew_mutex_);
  joint_pos_lower_deg_ = lower_deg;
  joint_pos_upper_deg_ = upper_deg;
}

void FanucClient::resetStreamCommandToMeasured()
{
  AssertIsStreaming(is_streaming_);
  readStateFromQueue();
  resetStreamCommand(last_joint_angles_);
}

void FanucClient::resetStreamCommand(const Eigen::VectorXd& joints_deg)
{
  Eigen::VectorXd pose = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  const Eigen::Index n = std::min<Eigen::Index>(joints_deg.size(), stream_motion::kMaxAxisNumber);
  pose.head(n) = joints_deg.head(n);
  pose = clampToPositionLimits(pose);
  std::lock_guard<std::mutex> lock(slew_mutex_);
  joint_goal_ = pose;
  stream_slew_.reset(pose);
}

void FanucClient::setDoMotnCtrl(const bool do_motn_ctrl)
{
  const bool prev = do_motn_ctrl_.load(std::memory_order_relaxed);
  if (prev == do_motn_ctrl)
  {
    return;
  }

  // On arm/disarm edges, snap slewer + goal to measured so command_pos cannot
  // diverge across the do_motn_ctrl transition (avoids MOTN-017 without Python priming).
  if (is_streaming_.load(std::memory_order_relaxed))
  {
    try
    {
      readStateFromQueue();
      resetStreamCommand(last_joint_angles_);
    }
    catch (const std::exception&)
    {
      // Stream may be tearing down; still apply the flag below.
    }
  }

  do_motn_ctrl_.store(do_motn_ctrl, std::memory_order_relaxed);
}

void FanucClient::applySlewLimitsLocked()
{
  stream_slew_.setLimits(stream_max_vel_deg_s_.load(std::memory_order_relaxed),
                         stream_max_acc_deg_s2_.load(std::memory_order_relaxed));
}

Eigen::VectorXd FanucClient::clampToPositionLimits(const Eigen::VectorXd& joints) const
{
  if (joint_pos_lower_deg_.empty() || joint_pos_upper_deg_.empty())
  {
    return joints;
  }
  Eigen::VectorXd out = joints;
  const Eigen::Index n =
      std::min<Eigen::Index>({ out.size(), static_cast<Eigen::Index>(joint_pos_lower_deg_.size()),
                               static_cast<Eigen::Index>(joint_pos_upper_deg_.size()) });
  for (Eigen::Index i = 0; i < n; ++i)
  {
    out[i] = std::clamp(out[i], joint_pos_lower_deg_[static_cast<size_t>(i)],
                        joint_pos_upper_deg_[static_cast<size_t>(i)]);
  }
  return out;
}

void FanucClient::jointSlewThread()
{
  // Never allow a zero period — that busy-spins writeJointTarget and starves UDP RX.
  const uint32_t period_ms = std::max(control_period_, kDefaultControlPeriodMs);
  const double period_s = static_cast<double>(period_ms) / 1000.0;

  while (slew_running_.load(std::memory_order_relaxed))
  {
    const auto t0 = std::chrono::steady_clock::now();
    const double multiplier =
        std::clamp(slew_rate_multiplier_.load(std::memory_order_relaxed), kMinSlewRateMultiplier,
                   kMaxSlewRateMultiplier);
    const double dt = period_s / multiplier;
    if (is_streaming_.load(std::memory_order_relaxed))
    {
      try
      {
        if (do_motn_ctrl_.load(std::memory_order_relaxed))
        {
          Eigen::VectorXd cmd;
          {
            std::lock_guard<std::mutex> lock(slew_mutex_);
            applySlewLimitsLocked();
            stream_slew_.setTarget(joint_goal_);
            cmd = stream_slew_.step(dt);
            cmd = clampToPositionLimits(cmd);
          }
          writeJointTarget(cmd);
        }
        else
        {
          // Keep command_pos glued to measured while disarmed (do_motn_ctrl=0).
          // writeJointTarget already substitutes last_joint_angles_ when disarmed.
          writeJointTarget(last_joint_angles_);
        }
      }
      catch (const std::exception&)
      {
        // Stream may be tearing down; exit quietly on the next flag check.
      }
    }
    std::this_thread::sleep_until(
        t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt)));
  }
}

Eigen::Ref<const Eigen::VectorXd> FanucClient::readJointAngles()
{
  AssertIsStreaming(is_streaming_);
  readStateFromQueue();

  return last_joint_angles_;
}

bool FanucClient::sendIOCommand() const
{
  if (gpio_buffer_ == nullptr)
  {
    // Nothing to command.
    return true;
  }

  return p_queue_impl_->command_io_queue_.try_enqueue(gpio_buffer_->command_buffer());
}

void FanucClient::streamMotionThread(const Eigen::VectorXd& joint_angles)
{
  stream_motion::RobotStatusPacket status;
  double command_timestamp = 0.0;
  double last_command_timestamp = 0.0;
  Eigen::VectorXd command = joint_angles;
  Eigen::VectorXd last_command = joint_angles;
  std::array<uint8_t, 256> command_io{};
  double ts_drift = 0.0;
  double dev_time = 0.0;
  double dev_time_prev = 0.0;
  bool motion_possible = false;

  while (is_streaming_)
  {
    if (!stream_motion_->getStatusPacket(status))
    {
      // Abort stream if we cannot get the status packet
      is_streaming_ = false;
    }
    else
    {
      if (status.status & 0x1)
      {
        motion_possible = true;
      }
      else
      {
        motion_possible = false;
      }
    }

    // set estimated time to first command's timestamp
    if ((dev_time == 0.0) && (p_queue_impl_->command_queue_.size_approx() != 0))
    {
      const PQueueImpl::StampedEigen* queue_entry = p_queue_impl_->command_queue_.peek();
      dev_time = std::get<0>(*queue_entry).count();
      dev_time_prev = dev_time;
    }
    else
    {
      // calculate drift
      double time_error = static_cast<double>(p_queue_impl_->command_queue_.size_approx()) -
                          static_cast<double>(out_cmd_interp_buff_target_);
      ts_drift = 0.99 * ts_drift + time_error * 0.000001;

      // push the time forward
      dev_time_prev = dev_time;
      dev_time += (getControlPeriod() / 1000.0) + ts_drift;
    }

    int size_before = p_queue_impl_->command_queue_.size_approx();

    // find the right interval to use.
    while (p_queue_impl_->command_queue_.size_approx() != 0)
    {
      const PQueueImpl::StampedEigen* queue_entry = p_queue_impl_->command_queue_.peek();
      last_command = command;
      last_command_timestamp = command_timestamp;
      command_timestamp = std::get<0>(*queue_entry).count();
      command = std::get<1>(*queue_entry);
      if (dev_time == 0.0)
      {
        dev_time = command_timestamp;
        dev_time_prev = command_timestamp;
      }
      if (command_timestamp >= dev_time_prev)
      {
        break;
      }
      p_queue_impl_->command_queue_.pop();
    }

    // Do interpolation.
    double alpha;
    if (command_timestamp - last_command_timestamp < 1e-6)
    {
      alpha = 0;
    }
    else
    {
      alpha = (dev_time_prev - last_command_timestamp) / (command_timestamp - last_command_timestamp);
    }
    alpha = std::min(alpha, 1.0);
    alpha = std::max(alpha, 0.0);
    for (Eigen::Index i = 0; i < status.joint_angle.size(); ++i)
    {
      command_pos[i] = alpha * command[i] + (1.0 - alpha) * last_command[i];
    }

    // Handle IO commands.
    while (p_queue_impl_->command_io_queue_.try_dequeue(command_io)) {}

    stream_motion_->sendCommand(command_pos, !is_streaming_, command_io,
                                ((motion_possible && do_motn_ctrl_.load(std::memory_order_relaxed)) ? 1 : 0));
    p_queue_impl_->robot_state_queue_.enqueue(status);
  }
}

void FanucClient::fetchRobotLimits()
{
  AssertNotStreaming(is_streaming_);

  for (int j = 0; j < stream_motion::kMaxAxisNumber; ++j)
  {
    stream_motion::RobotThresholdPacket robot_threshold_velocity;
    stream_motion::RobotThresholdPacket robot_threshold_acceleration;
    stream_motion::RobotThresholdPacket robot_threshold_jerk;
    if (!stream_motion_->getRobotLimits(j + 1, robot_threshold_velocity, robot_threshold_acceleration,
                                        robot_threshold_jerk))
    {
      throw std::runtime_error("Failed to get robot limits for axis " + std::to_string(j + 1) +
                               ". Ensure that the robot is reachable on the network by its IP.");
    }
    for (int index = 0; index < 20; ++index)
    {
      vel_limits_no_load_(j, index) = robot_threshold_velocity.no_payload[index];
      vel_limits_full_load_(j, index) = robot_threshold_velocity.full_payload[index];
      acc_limits_no_load_(j, index) = robot_threshold_acceleration.no_payload[index];
      acc_limits_full_load_(j, index) = robot_threshold_acceleration.full_payload[index];
      jerk_limits_no_load_(j, index) = robot_threshold_jerk.no_payload[index];
      jerk_limits_full_load_(j, index) = robot_threshold_jerk.full_payload[index];
    }
  }
}

bool FanucClient::getLimits(const double v_peak, const double payload, std::vector<double>& vel_limit,
                            std::vector<double>& acc_limit, std::vector<double>& jerk_limit) const
{
  const double v_max = 2000;
  const double v_min = v_max / 20;
  const double num = v_peak * 1.2 - v_min;
  const double denom = 1 / (v_max - v_min);
  const double pct = (std::min)((std::max)(num * denom, 0.0) * 19.0, 19.0);
  const int idx_l = (std::max)(static_cast<int>(pct), 0);
  const int idx_u = (std::min)(static_cast<int>(ceil(pct)), 19);
  const double idx_frac = pct - static_cast<double>(pct);
  Eigen::VectorXd vel_limit_no_load =
      idx_frac * (vel_limits_no_load_.col(idx_u) - vel_limits_no_load_.col(idx_l)) + vel_limits_no_load_.col(idx_l);
  Eigen::VectorXd acc_limit_no_load =
      idx_frac * (acc_limits_no_load_.col(idx_u) - acc_limits_no_load_.col(idx_l)) + acc_limits_no_load_.col(idx_l);
  Eigen::VectorXd jerk_limit_no_load =
      idx_frac * (jerk_limits_no_load_.col(idx_u) - jerk_limits_no_load_.col(idx_l)) + jerk_limits_no_load_.col(idx_l);
  Eigen::VectorXd vel_limit_full_load =
      idx_frac * (vel_limits_full_load_.col(idx_u) - vel_limits_full_load_.col(idx_l)) +
      vel_limits_full_load_.col(idx_l);
  Eigen::VectorXd acc_limit_full_load =
      idx_frac * (acc_limits_full_load_.col(idx_u) - acc_limits_full_load_.col(idx_l)) +
      acc_limits_full_load_.col(idx_l);
  Eigen::VectorXd jerk_limit_full_load =
      idx_frac * (jerk_limits_full_load_.col(idx_u) - jerk_limits_full_load_.col(idx_l)) +
      jerk_limits_full_load_.col(idx_l);
  const double payload_pct = payload / kFullPayload;
  vel_limit.resize(vel_limit_no_load.size(), 0.0);
  acc_limit.resize(acc_limit_full_load.size(), 0.0);
  jerk_limit.resize(jerk_limit_full_load.size(), 0.0);
  for (Eigen::Index i = 0; i < vel_limit_no_load.size(); ++i)
  {
    vel_limit[i] = payload_pct * (vel_limit_full_load[i] - vel_limit_no_load[i]) + vel_limit_no_load[i];
    acc_limit[i] = payload_pct * (acc_limit_full_load[i] - acc_limit_no_load[i]) + acc_limit_no_load[i];
    jerk_limit[i] = payload_pct * (jerk_limit_full_load[i] - jerk_limit_no_load[i]) + jerk_limit_no_load[i];
  }
  return true;
}

bool FanucClient::startMotionControl()
{
  AssertIsStreaming(is_streaming_);
  try
  {
    readStateFromQueue();
    if (robot_status_.motion_possible)
    {
      setDoMotnCtrl(true);
      return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      readStateFromQueue();
      if (robot_status_.motion_possible)
      {
        setDoMotnCtrl(true);
        return true;
      }
    }
    std::cerr << kStatusStatusNotReadyMessage << std::endl;
    return false;
  }
  catch (const std::runtime_error& e)
  {
    std::cerr << "Failed to check motion control readiness: " << e.what() << std::endl;
    return false;
  }
}

void FanucClient::stopMotionControl()
{
  AssertIsStreaming(is_streaming_);

  setDoMotnCtrl(false);

  // Wait for robot to stop motion (STREAM_MOTN stays running on the TP).
  const auto motion_pre_loop_time = std::chrono::steady_clock::now();
  while (true)
  {
    readStateFromQueue();
    if (!in_motion_)
    {
      break;
    }

    if (std::chrono::steady_clock::now() - motion_pre_loop_time > std::chrono::milliseconds(1000))
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// Throws if it fails to start real-time communication
void FanucClient::startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer)
{
  AssertNotStreaming(is_streaming_);

  stream_motion_->sendStopPacket();

  gpio_buffer_ = std::move(gpio_buffer);
  if (gpio_buffer_ != nullptr)
  {
    stream_motion_->configureGPIO(gpio_buffer_->toStreamMotionConfig());
  }

  // Wait for UDP status packets. STREAM_MOTN must already be running on the TP.
  stream_motion::RobotStatusPacket status;
  stream_motion_->sendStartPacket();
  stream_motion_->configureForceSensor(0, force_sensor_type_);
  const auto pre_loop_time = std::chrono::steady_clock::now();
  while (true)
  {
    if (stream_motion_->getStatusPacket(status))
    {
      // Accept any status for stream bring-up; motion_possible is checked separately.
      break;
    }
    if (std::chrono::steady_clock::now() - pre_loop_time > std::chrono::seconds(2))
    {
      throw std::runtime_error(kStatusPacketFailureMessage);
    }
  }
  start_time_ = std::chrono::high_resolution_clock::now();
  p_queue_impl_->robot_state_queue_.enqueue(status);
  is_streaming_ = true;
  last_joint_angles_ = Eigen::VectorXd::Zero(status.joint_angle.size());
  for (Eigen::Index i = 0; i < status.joint_angle.size(); ++i)
  {
    last_joint_angles_[i] = static_cast<double>(status.joint_angle[i]);
    command_pos[i] = static_cast<double>(status.joint_angle[i]);
  }
  stream_motion_->sendCommand(command_pos, false, {}, (do_motn_ctrl_.load(std::memory_order_relaxed) ? 1 : 0));

  {
    std::lock_guard<std::mutex> lock(slew_mutex_);
    joint_goal_ = last_joint_angles_;
    applySlewLimitsLocked();
    stream_slew_.reset(last_joint_angles_);
  }

  if (rt_thread_.joinable())
  {
    rt_thread_.join();
  }
  rt_thread_ = std::thread([this] { streamMotionThread(last_joint_angles_); });

  slew_running_.store(true, std::memory_order_relaxed);
  if (slew_thread_.joinable())
  {
    slew_thread_.join();
  }
  slew_thread_ = std::thread([this] { jointSlewThread(); });
}

void FanucClient::stopRealtimeStream()
{
  setDoMotnCtrl(false);
  slew_running_.store(false, std::memory_order_relaxed);
  if (slew_thread_.joinable())
  {
    slew_thread_.join();
  }
  is_streaming_ = false;
  if (rt_thread_.joinable())
  {
    rt_thread_.join();
  }

  // Wait for robot to stop motion
  stream_motion::RobotStatusPacket status;
  const auto motion_pre_loop_time = std::chrono::steady_clock::now();
  do
  {
    if (std::chrono::steady_clock::now() - motion_pre_loop_time > std::chrono::seconds(1))
    {
      break;
    }

    const auto status_pre_loop_time = std::chrono::steady_clock::now();
    while (!stream_motion_->getStatusPacket(status))
    {
      if (std::chrono::steady_clock::now() - status_pre_loop_time > std::chrono::seconds(1))
      {
        throw std::runtime_error(kStatusPacketFailureMessage);
      }
    }
  } while (status.status & 0x8);

  stream_motion_->sendStopPacket();
}

bool FanucClient::isStreaming()
{
  return is_streaming_;
}

uint32_t FanucClient::getControlPeriod() const
{
  return control_period_;
}

void FanucClient::validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const
{
  if (gpio_buffer != nullptr)
  {
    if (!stream_motion_->configureGPIO(gpio_buffer->toStreamMotionConfig()))
    {
      throw std::runtime_error(
          "Failed to configure GPIO buffer. Ensure the GPIO buffer is correctly set up for the robot.");
    }
  }
}

void FanucClient::stopStreaming()
{
  is_streaming_ = false;
}

void FanucClient::signalHandler(int signal)
{
  if (signal == SIGINT)
  {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance_ != nullptr)
    {
      // Stop streaming to allow proper cleanup
      instance_->stopStreaming();
    }
  }
}

void FanucClient::setupSignalHandler()
{
  std::lock_guard<std::mutex> lock(instance_mutex_);
  instance_ = this;
  previous_signal_handler_ = std::signal(SIGINT, signalHandler);
}

void FanucClient::restoreSignalHandler()
{
  std::lock_guard<std::mutex> lock(instance_mutex_);
  if (instance_ == this)
  {
    instance_ = nullptr;
    std::signal(SIGINT, previous_signal_handler_);
  }
}

void FanucClient::configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const
{
  stream_motion_->configureForceSensor(do_reset, force_sensor_type);
}

}  // namespace fanuc_client
