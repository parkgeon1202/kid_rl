// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0
//
// MotorTelemetryMonitor 가 xacro 선언을 보고 켜지는지, Present PWM 부호 변환이 맞는지,
// CSV 에 vin_/duty_ 열이 맨 끝에 붙는지 검사한다. 서보 없이 돈다.
//
// 사용법: motor_telemetry_test OUTPUT_CSV

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "kid_rl/csv_logger.hpp"
#include "kid_rl/motor_telemetry_monitor.hpp"
#include "rclcpp/logging.hpp"

namespace
{

int failures = 0;

void check(bool ok, const std::string & what)
{
  if (!ok) {
    std::cerr << "FAIL: " << what << '\n';
    ++failures;
  }
}

std::string joint_block(const std::string & joint, bool telemetry)
{
  std::string block =
    "<joint name=\"" + joint + "\"><command_interface name=\"position\"/>"
    "<state_interface name=\"position\"/><state_interface name=\"velocity\"/></joint>"
    "<gpio name=\"" + joint + "_dxl\"><param name=\"type\">dxl</param><param name=\"ID\">1</param>"
    "<command_interface name=\"Goal Position\"/><state_interface name=\"Present Position\"/>";
  if (telemetry) {
    block += "<state_interface name=\"Present Input Voltage\"/>"
      "<state_interface name=\"Present PWM\"/>";
  }
  return block + "</gpio>";
}

std::string urdf(const std::vector<std::string> & joints, const std::vector<bool> & telemetry)
{
  std::string text = "<?xml version=\"1.0\"?><robot name=\"t\"><link name=\"base\"/>";
  std::string previous = "base";
  for (const auto & joint : joints) {
    text += "<link name=\"" + joint + "_link\"/><joint name=\"" + joint +
      "\" type=\"revolute\"><parent link=\"" + previous + "\"/><child link=\"" + joint +
      "_link\"/><axis xyz=\"0 0 1\"/><limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "</joint>";
    previous = joint + "_link";
  }
  text += "<ros2_control name=\"t\" type=\"system\"><hardware><plugin>x/Y</plugin></hardware>";
  for (std::size_t i = 0; i < joints.size(); ++i) {text += joint_block(joints[i], telemetry[i]);}
  return text + "</ros2_control></robot>";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: motor_telemetry_test OUTPUT_CSV\n";
    return 2;
  }
  using kid_rl::MotorTelemetryMonitor;
  const auto logger = rclcpp::get_logger("motor_telemetry_test");

  check(std::abs(MotorTelemetryMonitor::pwm_to_duty(885.0) - 1.0) < 1e-12, "full forward duty");
  check(std::abs(MotorTelemetryMonitor::pwm_to_duty(65536.0 - 885.0) + 1.0) < 1e-12,
    "full reverse duty arrives as an unsigned 16-bit value");
  check(MotorTelemetryMonitor::pwm_to_duty(0.0) == 0.0, "zero duty");

  const std::vector<std::string> joints{"a", "b", "c"};

  MotorTelemetryMonitor off;
  off.configure(urdf(joints, {false, false, false}), joints, logger);
  check(!off.enabled() && off.interface_count() == 0, "no declaration -> disabled");

  MotorTelemetryMonitor monitor;
  monitor.configure(urdf(joints, {true, false, true}), joints, logger);
  check(monitor.enabled() && monitor.interface_count() == 4, "two joints declared -> 4 interfaces");
  check(monitor.joint_index() == std::vector<std::size_t>({0, 2}), "monitored joints are a and c");
  std::vector<std::string> names;
  monitor.append_interface_names(joints, names);
  check(names == std::vector<std::string>({"a_dxl/Present Input Voltage", "a_dxl/Present PWM",
        "c_dxl/Present Input Voltage", "c_dxl/Present PWM"}), "interface names");

  kid_rl::CsvLogger csv;
  std::string error;
  check(csv.start(argv[1], joints, {"M0"}, 64, false, false, monitor.joint_index(), error),
    "logger start: " + error);
  kid_rl::CsvLogger::Sample sample;
  sample.joint_count = joints.size();
  sample.motor_volts[0] = 16.25;
  sample.motor_duty[0] = -0.5;
  sample.motor_volts[1] = 15.5;
  sample.motor_duty[1] = 0.25;
  csv.log(sample);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  csv.stop();

  std::ifstream file(argv[1]);
  std::string header, row;
  std::getline(file, header);
  std::getline(file, row);
  const std::string header_tail = ",vin_a,duty_a,vin_c,duty_c";
  check(header.size() > header_tail.size() &&
    header.compare(header.size() - header_tail.size(), header_tail.size(), header_tail) == 0,
    "header ends with the telemetry columns: " + header);
  const auto columns = [](const std::string & line) {
      std::size_t n = 1;
      for (const char c : line) {n += c == ',' ? 1 : 0;}
      return n;
    };
  check(columns(header) == columns(row), "row has as many columns as the header");
  const std::string row_tail = ",16.25,-0.5,15.5,0.25";
  check(row.size() > row_tail.size() &&
    row.compare(row.size() - row_tail.size(), row_tail.size(), row_tail) == 0,
    "row ends with the telemetry values: " + row);

  if (failures == 0) {std::cout << "motor_telemetry_test: all checks passed\n";}
  return failures == 0 ? 0 : 1;
}
