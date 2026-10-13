// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/gamepad.hpp"

#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>

namespace kid_rl
{
namespace
{

constexpr int kFirstEventDevice = 0;
constexpr int kLastEventDevice = 63;

bool read_device_id(int fd, input_id & id)
{
  return ioctl(fd, EVIOCGID, &id) == 0;
}

std::string read_device_name(int fd)
{
  std::array<char, 256> name{};
  if (ioctl(fd, EVIOCGNAME(name.size()), name.data()) < 0) {return {};}
  return name.data();
}

bool has_axis(int fd, std::uint16_t code)
{
  input_absinfo info{};
  return ioctl(fd, EVIOCGABS(code), &info) == 0;
}

// 스틱 위치를 그대로 속도 명령으로 바꾼다: 정규화된 축 [-1, 1] * 최대값.
// deadband 안쪽은 0 (스틱 중립 떨림 제거).
double map_axis(double input, double deadband, double limit)
{
  if (std::abs(input) < deadband) {return 0.0;}
  return std::clamp(input, -1.0, 1.0) * limit;
}

}  // namespace

Gamepad::Gamepad()
{
  buffer_.writeFromNonRT(CommandSample{});
}

Gamepad::~Gamepad()
{
  stop();
}

bool Gamepad::start(const Config & config, std::string & error)
{
  stop();
  config_ = config;
  left_x_ = 0.0;
  left_y_ = 0.0;
  right_x_ = 0.0;
  buffer_.writeFromNonRT(CommandSample{});

  if (!open_device(error) || !configure_axes(error)) {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    return false;
  }

  running_.store(true);
  store_command();
  thread_ = std::thread(&Gamepad::read_loop, this);
  return true;
}

void Gamepad::stop()
{
  running_.store(false);
  if (thread_.joinable()) {thread_.join();}
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  buffer_.writeFromNonRT(CommandSample{});
}

CommandSample Gamepad::read_from_rt()
{
  const auto * sample = buffer_.readFromRT();
  return sample ? *sample : CommandSample{};
}

bool Gamepad::open_device(std::string & error)
{
  const auto try_open = [](const std::string & path) {
      return open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    };

  if (!config_.device_path.empty()) {
    fd_ = try_open(config_.device_path);
    if (fd_ < 0) {
      error = "could not open " + config_.device_path + ": " + std::strerror(errno);
      return false;
    }
    return true;
  }

  int name_match_fd = -1;
  for (int index = kFirstEventDevice; index <= kLastEventDevice; ++index) {
    const std::string path = "/dev/input/event" + std::to_string(index);
    const int candidate = try_open(path);
    if (candidate < 0) {continue;}

    input_id id{};
    if (read_device_id(candidate, id) && id.vendor == config_.vendor_id &&
      id.product == config_.product_id)
    {
      if (name_match_fd >= 0) {close(name_match_fd);}
      fd_ = candidate;
      config_.device_path = path;
      return true;
    }

    const std::string name = read_device_name(candidate);
    if (name_match_fd < 0 && !config_.name_hint.empty() &&
      name.find(config_.name_hint) != std::string::npos)
    {
      name_match_fd = candidate;
      config_.device_path = path;
    } else {
      close(candidate);
    }
  }

  if (name_match_fd >= 0) {
    fd_ = name_match_fd;
    return true;
  }
  error = "no matching /dev/input/event* device found";
  return false;
}

bool Gamepad::configure_axes(std::string & error)
{
  if (!has_axis(fd_, ABS_X) || !has_axis(fd_, ABS_Y)) {
    error = "gamepad does not expose ABS_X and ABS_Y";
    return false;
  }
  left_x_code_ = ABS_X;
  left_y_code_ = ABS_Y;

  for (const auto code : {ABS_RX, ABS_Z, ABS_RZ}) {
    if (has_axis(fd_, code)) {
      right_x_code_ = code;
      has_right_x_ = true;
      return true;
    }
  }
  error = "gamepad does not expose ABS_RX, ABS_Z, or ABS_RZ";
  return false;
}

double Gamepad::normalize_axis(std::uint16_t code, std::int32_t value) const
{
  input_absinfo info{};
  if (ioctl(fd_, EVIOCGABS(code), &info) < 0 || info.maximum == info.minimum) {return 0.0;}
  const double center = 0.5 * (static_cast<double>(info.maximum) + info.minimum);
  const double half_range = 0.5 * (static_cast<double>(info.maximum) - info.minimum);
  return std::clamp((static_cast<double>(value) - center) / half_range, -1.0, 1.0);
}

void Gamepad::store_command()
{
  const std::int64_t now_ns = steady_time_nanoseconds();

  // Same axis directions as gamepad_master2ik: left Y -> vx, left X -> vy,
  // right X -> yaw rate. 스틱을 끝까지 밀면 max_vx / max_vy / max_wz, 놓으면 바로 0.
  CommandSample sample;
  sample.velocity = {
    map_axis(-left_y_, config_.deadband_vx, config_.max_vx),
    map_axis(-left_x_, config_.deadband_vy, config_.max_vy),
    map_axis(-right_x_, config_.deadband_wz, config_.max_wz)};
  sample.stamp_ns = now_ns;
  sample.valid = true;
  buffer_.writeFromNonRT(sample);
}

void Gamepad::read_loop()
{
  pollfd descriptor{fd_, POLLIN, 0};
  while (running_.load()) {
    const int result = poll(&descriptor, 1, 20);
    if (result < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    // evdev emits only when an axis changes. Refresh the sample while the
    // device remains connected so a held stick does not trip the watchdog.
    if (result == 0) {
      store_command();
      continue;
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {break;}
    if (!(descriptor.revents & POLLIN)) {continue;}

    input_event event{};
    const ssize_t count = read(fd_, &event, sizeof(event));
    if (count < 0) {
      if (errno == EAGAIN || errno == EINTR) {continue;}
      break;
    }
    if (count != static_cast<ssize_t>(sizeof(event)) || event.type != EV_ABS) {continue;}

    bool command_changed = true;
    if (event.code == left_x_code_) {
      left_x_ = normalize_axis(event.code, event.value);
    } else if (event.code == left_y_code_) {
      left_y_ = normalize_axis(event.code, event.value);
    } else if (has_right_x_ && event.code == right_x_code_) {
      right_x_ = normalize_axis(event.code, event.value);
    } else {
      command_changed = false;
    }
    if (command_changed) {store_command();}
  }

  running_.store(false);
  buffer_.writeFromNonRT(CommandSample{});
}

}  // namespace kid_rl
