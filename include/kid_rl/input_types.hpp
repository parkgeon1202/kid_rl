// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__INPUT_TYPES_HPP_
#define KID_RL__INPUT_TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>

namespace kid_rl
{

inline std::int64_t steady_time_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct CommandSample
{
  std::array<double, 3> velocity{};  // vx, vy, wz
  std::int64_t stamp_ns{0};
  bool valid{false};
};

struct ImuSample
{
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};
  std::int64_t stamp_ns{0};
  bool valid{false};
};

}  // namespace kid_rl

#endif  // KID_RL__INPUT_TYPES_HPP_
