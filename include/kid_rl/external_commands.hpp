// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__EXTERNAL_COMMANDS_HPP_
#define KID_RL__EXTERNAL_COMMANDS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dynamixel_interfaces/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_interfaces/msg/dynamixel_msgs.hpp"
#include "dynamixel_interfaces/msg/dynamixel_state_msgs.hpp"
#include "dynamixel_interfaces/msg/pan_tilt.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace kid_rl
{

/// colcon_ws 의 dynamixel_hardware_interface 노드가 하던 토픽 입출력을 대신하는 부품.
///
///   /dynamixel_control  (DynamixelControlMsgs) <- motion_operator, humanoid_manipulation
///   /gripper_control    (DynamixelMsgs)        <- humanoid_manipulation
///   /RealsensePanTilt   (PanTilt, degree)      <- 비전 pan_tilt 노드 (tilt 만 쓴다)
///   /motor_status       (DynamixelStateMsgs)   -> motion_operator, humanoid_manipulation
///
/// 예전 노드는 배열 순서로 모터를 맞췄다. 여기서는 원소마다 id 를 받아 robot_description 의
/// <관절>_dxl gpio ID 와 맞춘다. 설정에 없는 id, 중복 id, 유한하지 않은 값, 그리퍼 토픽인데
/// 그리퍼 id 가 아닌 것, /dynamixel_control 에 tilt id 가 든 것(tilt 는 pan_tilt 토픽 전용)
/// 중 하나라도 있으면 메시지 전체를 버린다(일부만 적용하지 않는다).
///
/// 구독 콜백은 비실시간 스레드에서 돌고, 제어 스레드는 RealtimeBuffer 로 마지막 명령
/// 스냅샷만 읽는다. 명령을 어느 모드에서 받아들일지는 컨트롤러가 정한다.
class ExternalCommands
{
public:
  struct Config
  {
    std::string control_topic{"/dynamixel_control"};
    std::string gripper_topic{"/gripper_control"};
    std::string pan_tilt_topic{"/RealsensePanTilt"};
    std::string status_topic{"/motor_status"};
    int gripper_id{6};
    int tilt_id{23};
    double status_rate_hz{50.0};
  };

  /// 관절 하나의 마지막 외부 명령. seq 가 바뀌면 새 명령이다(0 = 받은 적 없음).
  struct MotionCommand
  {
    std::uint64_t seq{0};
    double goal{0.0};
    double velocity{0.0};
    double acceleration{0.0};
  };

  static constexpr char kGpioSuffix[] = "_dxl";

  /// on_configure 에서 부른다. slots 는 컨트롤러가 명령을 쓰는 관절 이름(정책 관절 다음에
  /// 고정 관절)이고, 각 관절의 ID 는 robot_description 에서 찾는다.
  bool configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const Config & config,
    const std::string & robot_description, const std::vector<std::string> & slots);
  /// on_cleanup 에서 부른다. 구독과 퍼블리셔를 놓는다.
  void reset();

  /// 슬롯 k 의 Dynamixel ID. 없으면 -1.
  int id_of(std::size_t slot) const {return slot_ids_[slot];}
  /// 그 ID 를 가진 슬롯. 없으면 -1.
  int slot_of(int id) const;

  /// 제어 스레드: 마지막 명령 스냅샷. 할당하지 않는다.
  const std::vector<MotionCommand> & read_from_rt();

  /// 제어 스레드: status_rate_hz 로 /motor_status 를 낸다. 할당하지 않는다.
  /// time 은 상태를 읽은 틱의 시각(header.stamp). 슬롯 k < policy 크기면 policy_* [k],
  /// 아니면 hold_* [k - policy 크기] 를 쓴다.
  void publish_status(
    const rclcpp::Time & time, double dt,
    const std::vector<double> & policy_positions, const std::vector<double> & policy_velocities,
    const std::vector<double> & hold_positions, const std::vector<double> & hold_velocities);

  /// 받은/버린 메시지 수 (진단용).
  std::uint64_t accepted() const;
  std::uint64_t rejected() const;

private:
  struct Update
  {
    std::size_t slot;
    double goal, velocity, acceleration;
  };
  /// 비실시간: 검증된 명령 묶음을 한 번에 반영한다.
  void apply(const std::vector<Update> & updates);
  bool validate(
    const dynamixel_interfaces::msg::DynamixelMsgs & motor, const char * topic,
    std::vector<bool> & seen, std::vector<Update> & out);
  void reject(const char * topic, const std::string & reason);

  void control_callback(dynamixel_interfaces::msg::DynamixelControlMsgs::SharedPtr msg);
  void gripper_callback(dynamixel_interfaces::msg::DynamixelMsgs::SharedPtr msg);
  void pan_tilt_callback(dynamixel_interfaces::msg::PanTilt::SharedPtr msg);

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  Config config_;
  std::vector<int> slot_ids_;
  std::array<int, 256> id_to_slot_{};
  int gripper_slot_{-1}, tilt_slot_{-1};

  // 비실시간 쪽 상태. 콜백끼리만 공유한다.
  mutable std::mutex mutex_;
  std::vector<MotionCommand> pending_;
  std::uint64_t next_seq_{0};
  std::uint64_t accepted_{0}, rejected_{0};
  realtime_tools::RealtimeBuffer<std::vector<MotionCommand>> buffer_;

  rclcpp::Subscription<dynamixel_interfaces::msg::DynamixelControlMsgs>::SharedPtr
    control_subscription_;
  rclcpp::Subscription<dynamixel_interfaces::msg::DynamixelMsgs>::SharedPtr gripper_subscription_;
  rclcpp::Subscription<dynamixel_interfaces::msg::PanTilt>::SharedPtr pan_tilt_subscription_;

  // 상태 퍼블리시. 순서는 ID 오름차순.
  using StatusMsg = dynamixel_interfaces::msg::DynamixelStateMsgs;
  rclcpp::Publisher<StatusMsg>::SharedPtr status_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<StatusMsg>> status_rt_publisher_;
  StatusMsg status_msg_;
  std::vector<std::size_t> status_slots_;
  double status_accumulator_s_{0.0};
};

}  // namespace kid_rl

#endif  // KID_RL__EXTERNAL_COMMANDS_HPP_
