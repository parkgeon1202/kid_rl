// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__MOTOR_TELEMETRY_MONITOR_HPP_
#define KID_RL__MOTOR_TELEMETRY_MONITOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "kid_rl/csv_logger.hpp"
#include "rclcpp/logger.hpp"

namespace kid_rl
{

/// 서보의 Present Input Voltage 와 Present PWM 을 읽어 CSV 샘플에 넣는 진단용 부품.
///
/// 걷는 중 전원 전압이 얼마나 떨어지는지, 펌웨어가 실제로 내는 PWM 이 모터 모델이
/// 가정한 값과 같은지 보려고 넣었다(2026-10-13). 정책 입력에는 쓰이지 않는다.
///
/// 켜고 끄는 스위치는 xacro 다. 관절의 gpio(<관절>_dxl)가 두 state_interface 를 모두
/// 선언한 관절만 기록한다. 하나도 없으면 인터페이스를 요청하지 않고 CSV 열도 만들지
/// 않는다 -- 컨트롤러는 이 부품이 없던 때와 같다.
class MotorTelemetryMonitor
{
public:
  static constexpr char kVoltageInterface[] = "Present Input Voltage";
  static constexpr char kPwmInterface[] = "Present PWM";
  static constexpr char kGpioSuffix[] = "_dxl";
  /// Present PWM 레지스터의 100% 값. X/MX(2.0) 계열 공통.
  static constexpr double kPwmFullScale = 885.0;

  /// on_configure 에서 부른다. robot_description 을 파싱해 기록할 관절을 정한다.
  void configure(
    const std::string & robot_description, const std::vector<std::string> & joints,
    const rclcpp::Logger & logger);

  bool enabled() const {return !joint_index_.empty();}
  /// 요청하는 상태 인터페이스 총수 (기록하는 관절 수 x 2).
  std::size_t interface_count() const {return 2 * joint_index_.size();}
  /// 기록하는 관절의 joints 안 위치. CSV 열 순서와 같다.
  const std::vector<std::size_t> & joint_index() const {return joint_index_;}

  void append_interface_names(
    const std::vector<std::string> & joints, std::vector<std::string> & names) const;

  /// on_activate 에서 이름으로 인덱스를 찾는다. 꺼져 있으면 아무것도 안 하고 true.
  bool map(
    const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
    const std::vector<std::string> & joints, const rclcpp::Logger & logger);

  /// 제어 스레드에서 부른다. 할당하지 않는다.
  void fill(
    const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
    CsvLogger::Sample & sample) const;

  /// 드라이버가 부호 없이 넘긴 16 비트 Present PWM 을 듀티(-1..1)로 바꾼다.
  static double pwm_to_duty(double raw);

private:
  std::vector<std::size_t> joint_index_;
  std::vector<std::size_t> voltage_index_;
  std::vector<std::size_t> pwm_index_;
};

}  // namespace kid_rl

#endif  // KID_RL__MOTOR_TELEMETRY_MONITOR_HPP_
