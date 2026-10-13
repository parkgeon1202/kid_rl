// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/rx_missing_monitor.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"

namespace kid_rl
{

void RxMissingMonitor::configure(
  const std::string & robot_description, const std::vector<std::string> & joints,
  const rclcpp::Logger & logger)
{
  enabled_ = false;
  index_.assign(joints.size(), 0);
  if (robot_description.empty() || joints.empty()) {return;}

  std::vector<hardware_interface::HardwareInfo> hardware;
  try {
    hardware = hardware_interface::parse_control_resources_from_urdf(robot_description);
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger, "rx_missing_ids log disabled: could not parse robot_description: %s",
      error.what());
    return;
  }

  std::size_t declared = 0;
  for (const auto & joint : joints) {
    bool found = false;
    for (const auto & info : hardware) {
      for (const auto & component : info.joints) {
        if (component.name != joint) {continue;}
        found = std::any_of(
          component.state_interfaces.begin(), component.state_interfaces.end(),
          [](const auto & state) {return state.name == kInterfaceName;});
      }
    }
    if (found) {++declared;}
  }

  if (declared == joints.size()) {
    enabled_ = true;
    RCLCPP_INFO(logger, "rx_missing_ids log enabled (xacro track_rx=true)");
  } else if (declared > 0) {
    // 일부만 선언된 설정은 실수일 가능성이 높다. 반쪽 기록은 오해를 부르니 끈다.
    RCLCPP_WARN(
      logger, "rx_missing_ids log disabled: only %zu/%zu joints declare %s",
      declared, joints.size(), kInterfaceName);
  }
}

void RxMissingMonitor::append_interface_names(
  const std::vector<std::string> & joints, std::vector<std::string> & names) const
{
  if (!enabled_) {return;}
  for (const auto & joint : joints) {
    names.push_back(joint + "/" + kInterfaceName);
  }
}

bool RxMissingMonitor::map(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  const std::vector<std::string> & joints, const rclcpp::Logger & logger)
{
  if (!enabled_) {return true;}
  for (std::size_t i = 0; i < joints.size(); ++i) {
    bool found = false;
    for (std::size_t k = 0; k < interfaces.size(); ++k) {
      if (interfaces[k].get_prefix_name() == joints[i] &&
        interfaces[k].get_interface_name() == kInterfaceName)
      {
        index_[i] = k;
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(
        logger, "Missing state interface %s/%s", joints[i].c_str(), kInterfaceName);
      return false;
    }
  }
  return true;
}

void RxMissingMonitor::fill(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  CsvLogger::Sample & sample) const
{
  sample.rx_missing_count = 0;
  if (!enabled_) {return;}
  const std::size_t count = std::min(index_.size(), CsvLogger::kMaxJoints);
  for (std::size_t i = 0; i < count; ++i) {
    // 관절 순서로 모은다. 버스 응답 순서가 필요하면 stderr 의 "missing IDs" 줄을 본다.
    const auto missing = interfaces[index_[i]].get_optional();
    if (missing && *missing >= 0.0) {
      sample.rx_missing_ids[sample.rx_missing_count++] = static_cast<int>(*missing);
    }
  }
}

}  // namespace kid_rl
