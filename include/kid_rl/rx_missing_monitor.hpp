// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__RX_MISSING_MONITOR_HPP_
#define KID_RL__RX_MISSING_MONITOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "kid_rl/csv_logger.hpp"
#include "rclcpp/logger.hpp"

namespace kid_rl
{

/// DynamixelHardware 의 관절별 rx_missing_id 상태 인터페이스(이번 틱 SyncRead 에서
/// 응답이 빠진 서보 ID, 받았으면 -1)를 읽어 CSV 샘플에 모아 넣는 진단용 부품.
///
/// 켜고 끄는 스위치는 xacro 의 track_rx 하나다. 그 값에 따라 드라이버가 관절마다
/// rx_missing_id 를 선언하거나 말거나 하므로, 여기서는 robot_description 에 모든
/// 관절이 그 인터페이스를 선언했는지를 보고 스스로 켜진다. 꺼져 있으면 인터페이스를
/// 요청하지 않고 CSV 열도 만들지 않는다 -- 컨트롤러는 이 부품이 없던 때와 같다.
class RxMissingMonitor
{
public:
  static constexpr char kInterfaceName[] = "rx_missing_id";

  /// on_configure 에서 부른다. robot_description 을 파싱해 켤지 정한다.
  void configure(
    const std::string & robot_description, const std::vector<std::string> & joints,
    const rclcpp::Logger & logger);

  bool enabled() const {return enabled_;}
  /// 관절 하나당 요청하는 상태 인터페이스 수 (0 또는 1).
  std::size_t interfaces_per_joint() const {return enabled_ ? 1 : 0;}

  /// state_interface_configuration() 의 이름 목록에 이 부품의 인터페이스를 더한다.
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

private:
  bool enabled_{false};
  std::vector<std::size_t> index_;
};

}  // namespace kid_rl

#endif  // KID_RL__RX_MISSING_MONITOR_HPP_
