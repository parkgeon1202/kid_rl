// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/motor_telemetry_monitor.hpp"

#include <algorithm>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"

namespace kid_rl
{

void MotorTelemetryMonitor::configure(
  const std::string & robot_description, const std::vector<std::string> & joints,
  const rclcpp::Logger & logger)
{
  joint_index_.clear();
  voltage_index_.clear();
  pwm_index_.clear();
  if (robot_description.empty() || joints.empty()) {return;}

  std::vector<hardware_interface::HardwareInfo> hardware;
  try {
    hardware = hardware_interface::parse_control_resources_from_urdf(robot_description);
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger, "motor telemetry log disabled: could not parse robot_description: %s",
      error.what());
    return;
  }

  const auto declares = [](const auto & gpio, const char * name) {
      return std::any_of(
        gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
        [name](const auto & state) {return state.name == name;});
    };
  for (std::size_t i = 0; i < joints.size() && i < CsvLogger::kMaxJoints; ++i) {
    const std::string gpio_name = joints[i] + kGpioSuffix;
    for (const auto & info : hardware) {
      for (const auto & gpio : info.gpios) {
        if (gpio.name == gpio_name && declares(gpio, kVoltageInterface) &&
          declares(gpio, kPwmInterface))
        {
          joint_index_.push_back(i);
        }
      }
    }
  }
  voltage_index_.assign(joint_index_.size(), 0);
  pwm_index_.assign(joint_index_.size(), 0);
  if (enabled()) {
    RCLCPP_INFO(
      logger, "motor telemetry log enabled for %zu joints (Present Input Voltage, Present PWM)",
      joint_index_.size());
  }
}

void MotorTelemetryMonitor::append_interface_names(
  const std::vector<std::string> & joints, std::vector<std::string> & names) const
{
  for (const std::size_t i : joint_index_) {
    names.push_back(joints[i] + kGpioSuffix + "/" + kVoltageInterface);
    names.push_back(joints[i] + kGpioSuffix + "/" + kPwmInterface);
  }
}

bool MotorTelemetryMonitor::map(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  const std::vector<std::string> & joints, const rclcpp::Logger & logger)
{
  const auto find = [&](const std::string & prefix, const char * name, std::size_t & out) {
      for (std::size_t k = 0; k < interfaces.size(); ++k) {
        if (interfaces[k].get_prefix_name() == prefix &&
          interfaces[k].get_interface_name() == name)
        {
          out = k;
          return true;
        }
      }
      RCLCPP_ERROR(logger, "Missing state interface %s/%s", prefix.c_str(), name);
      return false;
    };
  for (std::size_t m = 0; m < joint_index_.size(); ++m) {
    const std::string prefix = joints[joint_index_[m]] + kGpioSuffix;
    if (!find(prefix, kVoltageInterface, voltage_index_[m]) ||
      !find(prefix, kPwmInterface, pwm_index_[m]))
    {
      return false;
    }
  }
  return true;
}

double MotorTelemetryMonitor::pwm_to_duty(double raw)
{
  // 모델 파일에 Present PWM 의 단위·부호 정보가 없어 드라이버가 부호 없는 16 비트 값을
  // 그대로 넘긴다(dynamixel.cpp 의 fallback 분기). 음수는 65536 을 더한 값으로 온다.
  if (raw > 32767.0) {raw -= 65536.0;}
  return raw / kPwmFullScale;
}

void MotorTelemetryMonitor::fill(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  CsvLogger::Sample & sample) const
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t m = 0; m < joint_index_.size(); ++m) {
    const auto volts = interfaces[voltage_index_[m]].get_optional();
    const auto pwm = interfaces[pwm_index_[m]].get_optional();
    sample.motor_volts[m] = volts ? *volts : nan;
    sample.motor_duty[m] = pwm ? pwm_to_duty(*pwm) : nan;
  }
}

}  // namespace kid_rl
