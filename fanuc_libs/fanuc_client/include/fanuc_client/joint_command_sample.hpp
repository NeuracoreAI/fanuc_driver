// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

#include "stream_motion/packets.hpp"

namespace fanuc_client
{

/** POD sample of command + measured joints immediately before UDP sendCommand. */
struct JointCommandSample
{
  double t_s{ 0.0 };
  std::array<double, stream_motion::kMaxAxisNumber> command{};
  std::array<double, stream_motion::kMaxAxisNumber> measured{};
};

}  // namespace fanuc_client
