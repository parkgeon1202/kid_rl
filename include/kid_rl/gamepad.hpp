// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__GAMEPAD_HPP_
#define KID_RL__GAMEPAD_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "kid_rl/input_types.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace kid_rl
{

class Gamepad
{
public:
  struct Config
  {
    std::string device_path;
    std::string name_hint{"Gulikit X"};
    std::uint16_t vendor_id{0x045e};
    std::uint16_t product_id{0x028e};
    double max_vx{0.4};
    double max_vy{0.3};
    double max_wz{0.7};
    // 스틱 위치 -> 속도 직접 매핑: cmd = stick [-1, 1] * max. |stick| < deadband 는 0.
    double deadband_vx{0.05};
    double deadband_vy{0.05};
    double deadband_wz{0.05};
  };

  Gamepad();
  ~Gamepad();

  Gamepad(const Gamepad &) = delete;
  Gamepad & operator=(const Gamepad &) = delete;

  bool start(const Config & config, std::string & error);
  void stop();
  CommandSample read_from_rt();
  bool running() const {return running_.load();}

private:
  bool open_device(std::string & error);
  bool configure_axes(std::string & error);
  void read_loop();
  void store_command();
  double normalize_axis(std::uint16_t code, std::int32_t value) const;

  Config config_;
  int fd_{-1};
  std::uint16_t left_x_code_{0};
  std::uint16_t left_y_code_{0};
  std::uint16_t right_x_code_{0};
  bool has_right_x_{false};
  double left_x_{0.0};
  double left_y_{0.0};
  double right_x_{0.0};
  std::atomic<bool> running_{false};
  std::thread thread_;
  realtime_tools::RealtimeBuffer<CommandSample> buffer_;
};

}  // namespace kid_rl

#endif  // KID_RL__GAMEPAD_HPP_
