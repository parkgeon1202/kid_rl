// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__BUS_TIMING_MONITOR_HPP_
#define KID_RL__BUS_TIMING_MONITOR_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "kid_rl/csv_logger.hpp"
#include "rclcpp/logger.hpp"

namespace kid_rl
{

/// DynamixelHardware 의 type=bus_timing gpio 가 내보내는 버스 타이밍 값을 읽어
/// CSV 샘플에 넣는 진단용 부품. RxMissingMonitor 와 같은 방식으로, robot_description
/// 에 그 gpio 가 필요한 인터페이스를 전부 선언했을 때만 스스로 켜진다. 꺼져 있으면
/// 인터페이스를 요청하지 않고 CSV 열도 만들지 않는다.
class BusTimingMonitor
{
public:
  /// 드라이버가 내보내는 이름과 같아야 한다 (dynamixel_hardware_interface.cpp
  /// SetupBusTiming). 순서는 index_ 의 순서다.
  static constexpr std::array<const char *, 7> kInterfaces{
    "read_ms", "read_ok", "read_ok_start_s", "read_ok_end_s",
    "write_sent", "write_ms", "write_drain_ms"};

  void configure(const std::string & robot_description, const rclcpp::Logger & logger);

  bool enabled() const {return enabled_;}
  std::size_t interface_count() const {return enabled_ ? kInterfaces.size() : 0;}

  void append_interface_names(std::vector<std::string> & names) const;

  bool map(
    const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
    const rclcpp::Logger & logger);

  /// 제어 스레드에서 부른다. 할당하지 않는다. now_s 는 steady_clock 초.
  void fill(
    const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
    double now_s, CsvLogger::Sample & sample) const;

private:
  bool enabled_{false};
  std::string gpio_;
  std::array<std::size_t, kInterfaces.size()> index_{};
};

}  // namespace kid_rl

#endif  // KID_RL__BUS_TIMING_MONITOR_HPP_
