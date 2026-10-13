// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/bus_timing_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"

namespace kid_rl
{
namespace
{
enum Field : std::size_t
{
  kReadMs, kReadOk, kReadOkStartS, kReadOkEndS, kWriteSent, kWriteMs, kWriteDrainMs
};
}  // namespace

void BusTimingMonitor::configure(
  const std::string & robot_description, const rclcpp::Logger & logger)
{
  enabled_ = false;
  gpio_.clear();
  if (robot_description.empty()) {return;}

  std::vector<hardware_interface::HardwareInfo> hardware;
  try {
    hardware = hardware_interface::parse_control_resources_from_urdf(robot_description);
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger, "bus timing log disabled: could not parse robot_description: %s",
      error.what());
    return;
  }

  for (const auto & info : hardware) {
    for (const auto & gpio : info.gpios) {
      const auto type = gpio.parameters.find("type");
      if (type == gpio.parameters.end() || type->second != "bus_timing") {continue;}
      const bool complete = std::all_of(
        kInterfaces.begin(), kInterfaces.end(), [&gpio](const char * name) {
          return std::any_of(
            gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
            [name](const auto & state) {return state.name == name;});
        });
      if (!complete) {
        RCLCPP_WARN(
          logger, "bus timing log disabled: gpio '%s' does not declare all %zu interfaces",
          gpio.name.c_str(), kInterfaces.size());
        return;
      }
      gpio_ = gpio.name;
      enabled_ = true;
      RCLCPP_INFO(logger, "bus timing log enabled (gpio '%s')", gpio_.c_str());
      return;
    }
  }
}

void BusTimingMonitor::append_interface_names(std::vector<std::string> & names) const
{
  if (!enabled_) {return;}
  for (const char * name : kInterfaces) {
    names.push_back(gpio_ + "/" + name);
  }
}

bool BusTimingMonitor::map(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  const rclcpp::Logger & logger)
{
  if (!enabled_) {return true;}
  for (std::size_t f = 0; f < kInterfaces.size(); ++f) {
    bool found = false;
    for (std::size_t k = 0; k < interfaces.size(); ++k) {
      if (interfaces[k].get_prefix_name() == gpio_ &&
        interfaces[k].get_interface_name() == kInterfaces[f])
      {
        index_[f] = k;
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(logger, "Missing state interface %s/%s", gpio_.c_str(), kInterfaces[f]);
      return false;
    }
  }
  return true;
}

void BusTimingMonitor::fill(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  double now_s, CsvLogger::Sample & sample) const
{
  if (!enabled_) {return;}
  const auto get = [&](Field f) {
      const auto value = interfaces[index_[f]].get_optional();
      return value ? *value : std::numeric_limits<double>::quiet_NaN();
    };

  sample.read_ms = get(kReadMs);
  sample.read_ok = get(kReadOk) > 0.5 ? 1 : 0;
  // 관절값은 마지막으로 '성공한' SyncRead 의 [시작, 끝] 사이에 서보가 찍은 값이다.
  // 그래서 지금 쓰는 관절값의 나이는 (now - 끝) 이상, (now - 시작) 이하다.
  // 읽기가 실패한 틱에는 이전 성공 시각이 남아 있어 나이가 한 틱만큼 자란다.
  const double start_s = get(kReadOkStartS);
  const double end_s = get(kReadOkEndS);
  const bool have_read = start_s > 0.0 && end_s > 0.0;
  sample.joint_age_min_ms = have_read ? (now_s - end_s) * 1000.0 : -1.0;
  sample.joint_age_max_ms = have_read ? (now_s - start_s) * 1000.0 : -1.0;
  // write() 는 update() 다음에 돌므로 여기서 보이는 값은 '직전 틱'의 쓰기다.
  sample.prev_write_sent = get(kWriteSent) > 0.5 ? 1 : 0;
  sample.prev_write_ms = get(kWriteMs);
  sample.prev_write_drain_ms = get(kWriteDrainMs);
}

}  // namespace kid_rl
