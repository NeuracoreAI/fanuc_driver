// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "fanuc_client/joint_command_sample.hpp"
#include "readerwriterqueue.h"

namespace fanuc_client
{

/**
 * Non-RT consumer thread: drains pre-socket command + measured samples, renders
 * a live ImGui/ImPlot window (one subplot per joint: cmd vs measured, fixed
 * time window, auto-fit Y), and appends every sample to a CSV.
 */
class JointCommandPlotter
{
public:
  using Queue = moodycamel::ReaderWriterQueue<JointCommandSample>;

  /**
   * @param queue SPSC queue filled by the RT send path.
   * @param n_joints Number of joint subplots (typically 6).
   * @param window_s Fixed scrolling time window width in seconds.
   * @param csv_path Output CSV path; empty => auto-generate
   *                 ``fanuc_stream_commands_<timestamp>.csv`` in the CWD.
   */
  JointCommandPlotter(Queue& queue, int n_joints = 6, double window_s = 10.0, std::string csv_path = {});
  ~JointCommandPlotter();

  JointCommandPlotter(const JointCommandPlotter&) = delete;
  JointCommandPlotter& operator=(const JointCommandPlotter&) = delete;

  void start();
  void stop();

  bool isRunning() const
  {
    return running_.load(std::memory_order_acquire);
  }

  const std::string& csvPath() const
  {
    return csv_path_;
  }

private:
  void threadMain();
  static std::string makeDefaultCsvPath();

  Queue& queue_;
  int n_joints_;
  double window_s_;
  std::string csv_path_;
  std::atomic<bool> stop_requested_{ false };
  std::atomic<bool> running_{ false };
  std::thread thread_;
};

}  // namespace fanuc_client
