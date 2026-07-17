// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_client/joint_command_plotter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

namespace fanuc_client
{
namespace
{
void glfwErrorCallback(int error, const char* description)
{
  std::cerr << "GLFW error " << error << ": " << description << std::endl;
}

void writeCsvHeader(std::ofstream& csv, const int n_joints)
{
  csv << "t_s";
  for (int i = 0; i < n_joints; ++i)
  {
    csv << ",J" << (i + 1) << "_cmd,J" << (i + 1) << "_meas";
  }
  csv << '\n';
}

void writeCsvRow(std::ofstream& csv, const JointCommandSample& sample, const int n_joints)
{
  csv << sample.t_s;
  for (int i = 0; i < n_joints; ++i)
  {
    csv << ',' << sample.command[static_cast<std::size_t>(i)] << ','
        << sample.measured[static_cast<std::size_t>(i)];
  }
  csv << '\n';
}

void drainQueueToCsv(JointCommandPlotter::Queue& queue, std::ofstream& csv, const int n_joints,
                     const std::atomic<bool>& stop_requested)
{
  JointCommandSample sample;
  while (!stop_requested.load(std::memory_order_acquire))
  {
    bool got_any = false;
    while (queue.try_dequeue(sample))
    {
      got_any = true;
      if (csv)
      {
        writeCsvRow(csv, sample, n_joints);
      }
    }
    if (got_any && csv)
    {
      csv.flush();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
}  // namespace

std::string JointCommandPlotter::makeDefaultCsvPath()
{
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const std::time_t t = clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << "fanuc_stream_commands_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
  return oss.str();
}

JointCommandPlotter::JointCommandPlotter(Queue& queue, const int n_joints, const double window_s,
                                         std::string csv_path)
  : queue_(queue)
  , n_joints_(std::max(1, std::min(n_joints, stream_motion::kMaxAxisNumber)))
  , window_s_(std::max(1.0, window_s))
  , csv_path_(csv_path.empty() ? makeDefaultCsvPath() : std::move(csv_path))
{
}

JointCommandPlotter::~JointCommandPlotter()
{
  stop();
}

void JointCommandPlotter::start()
{
  if (running_.load(std::memory_order_acquire))
  {
    return;
  }
  stop_requested_.store(false, std::memory_order_release);
  thread_ = std::thread([this] { threadMain(); });
}

void JointCommandPlotter::stop()
{
  stop_requested_.store(true, std::memory_order_release);
  if (thread_.joinable())
  {
    thread_.join();
  }
  running_.store(false, std::memory_order_release);
}

void JointCommandPlotter::threadMain()
{
  running_.store(true, std::memory_order_release);

  std::ofstream csv(csv_path_, std::ios::out | std::ios::trunc);
  if (!csv)
  {
    std::cerr << "JointCommandPlotter: failed to open CSV '" << csv_path_ << "'" << std::endl;
  }
  else
  {
    writeCsvHeader(csv, n_joints_);
    csv << std::setprecision(9);
    std::cout << "JointCommandPlotter: logging CSV -> " << csv_path_ << std::endl;
  }

  glfwSetErrorCallback(glfwErrorCallback);
  if (!glfwInit())
  {
    std::cerr << "JointCommandPlotter: glfwInit failed (no display?). "
                 "CSV logging continues without a window."
              << std::endl;
    drainQueueToCsv(queue_, csv, n_joints_, stop_requested_);
    if (csv)
    {
      csv.flush();
      std::cout << "JointCommandPlotter: CSV closed -> " << csv_path_ << std::endl;
    }
    running_.store(false, std::memory_order_release);
    return;
  }

#if defined(__APPLE__)
  const char* glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

  const int height = std::max(320, 160 * n_joints_);
  GLFWwindow* window =
      glfwCreateWindow(1600, height, "Stream Motion cmd vs measured", nullptr, nullptr);
  if (window == nullptr)
  {
    std::cerr << "JointCommandPlotter: glfwCreateWindow failed. "
                 "CSV logging continues without a window."
              << std::endl;
    glfwTerminate();
    drainQueueToCsv(queue_, csv, n_joints_, stop_requested_);
    if (csv)
    {
      csv.flush();
      std::cout << "JointCommandPlotter: CSV closed -> " << csv_path_ << std::endl;
    }
    running_.store(false, std::memory_order_release);
    return;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  std::deque<double> times;
  std::vector<std::deque<double>> cmd_series(static_cast<std::size_t>(n_joints_));
  std::vector<std::deque<double>> meas_series(static_cast<std::size_t>(n_joints_));
  std::vector<double> t_buf;
  std::vector<double> y_buf;
  t_buf.reserve(4096);
  y_buf.reserve(4096);
  double t_latest = 0.0;

  std::cout << "JointCommandPlotter: live window open (" << n_joints_
            << " joints, cmd+measured, fixed " << window_s_ << "s x-axis, auto-fit y)" << std::endl;

  while (!stop_requested_.load(std::memory_order_acquire) && !glfwWindowShouldClose(window))
  {
    glfwPollEvents();

    JointCommandSample sample;
    bool got_any = false;
    while (queue_.try_dequeue(sample))
    {
      got_any = true;
      t_latest = sample.t_s;
      times.push_back(sample.t_s);
      for (int i = 0; i < n_joints_; ++i)
      {
        cmd_series[static_cast<std::size_t>(i)].push_back(sample.command[static_cast<std::size_t>(i)]);
        meas_series[static_cast<std::size_t>(i)].push_back(sample.measured[static_cast<std::size_t>(i)]);
      }
      if (csv)
      {
        writeCsvRow(csv, sample, n_joints_);
      }
    }
    if (got_any && csv)
    {
      csv.flush();
    }

    // Keep only the fixed scrolling window for display (CSV keeps full history).
    if (!times.empty())
    {
      const double t_min = times.back() - window_s_;
      while (!times.empty() && times.front() < t_min)
      {
        times.pop_front();
        for (auto& s : cmd_series)
        {
          if (!s.empty())
          {
            s.pop_front();
          }
        }
        for (auto& s : meas_series)
        {
          if (!s.empty())
          {
            s.pop_front();
          }
        }
      }
      t_latest = times.back();
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Stream Motion cmd vs measured", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);

    ImGui::TextUnformatted("cmd = pre-socket OTG command  |  meas = status.joint_angle");
    ImGui::Text("window: %.1fs  |  y: auto-fit  |  samples: %zu  |  csv: %s", window_s_, times.size(),
                csv_path_.c_str());

    const double x_max = t_latest;
    const double x_min = x_max - window_s_;

    if (ImPlot::BeginSubplots("##joints", n_joints_, 1, ImVec2(-1, -1),
                             ImPlotSubplotFlags_LinkRows | ImPlotSubplotFlags_LinkAllX))
    {
      t_buf.assign(times.begin(), times.end());
      for (int i = 0; i < n_joints_; ++i)
      {
        const std::string title = "J" + std::to_string(i + 1) + " [deg]";
        if (ImPlot::BeginPlot(title.c_str(), ImVec2(-1, 0)))
        {
          ImPlot::SetupAxes("t (s)", "deg", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
          ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImPlotCond_Always);
          if (!t_buf.empty())
          {
            y_buf.assign(cmd_series[static_cast<std::size_t>(i)].begin(),
                         cmd_series[static_cast<std::size_t>(i)].end());
            ImPlot::PlotLine("cmd", t_buf.data(), y_buf.data(), static_cast<int>(t_buf.size()));
            y_buf.assign(meas_series[static_cast<std::size_t>(i)].begin(),
                         meas_series[static_cast<std::size_t>(i)].end());
            ImPlot::PlotLine("meas", t_buf.data(), y_buf.data(), static_cast<int>(t_buf.size()));
          }
          ImPlot::EndPlot();
        }
      }
      ImPlot::EndSubplots();
    }

    ImGui::End();
    ImGui::Render();

    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();

  if (csv)
  {
    csv.flush();
    std::cout << "JointCommandPlotter: CSV closed -> " << csv_path_ << std::endl;
  }
  std::cout << "JointCommandPlotter: window closed" << std::endl;
  running_.store(false, std::memory_order_release);
}

}  // namespace fanuc_client
