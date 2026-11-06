// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/external_commands.hpp"

#include <cmath>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"

namespace kid_rl
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

bool ExternalCommands::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const Config & config,
  const std::string & robot_description, const std::vector<std::string> & slots)
{
  reset();
  node_ = node;
  config_ = config;
  const auto logger = node_->get_logger();
  slot_ids_.assign(slots.size(), -1);
  id_to_slot_.fill(-1);

  std::vector<hardware_interface::HardwareInfo> hardware;
  try {
    hardware = hardware_interface::parse_control_resources_from_urdf(robot_description);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "external commands: could not parse robot_description: %s", error.what());
    return false;
  }

  // 관절 이름 -> <관절>_dxl gpio 의 ID. 다른 부품(rx_missing_monitor 등)과 같은 규칙이다.
  for (std::size_t k = 0; k < slots.size(); ++k) {
    const std::string gpio_name = slots[k] + kGpioSuffix;
    for (const auto & info : hardware) {
      for (const auto & gpio : info.gpios) {
        if (gpio.name != gpio_name) {continue;}
        const auto id_param = gpio.parameters.find("ID");
        if (id_param == gpio.parameters.end()) {continue;}
        int id = -1;
        try {
          id = std::stoi(id_param->second);
        } catch (const std::exception &) {
          id = -1;
        }
        if (id < 0 || id > 252) {
          RCLCPP_ERROR(
            logger, "external commands: gpio '%s' has invalid ID '%s'", gpio_name.c_str(),
            id_param->second.c_str());
          return false;
        }
        if (id_to_slot_[static_cast<std::size_t>(id)] >= 0) {
          RCLCPP_ERROR(logger, "external commands: duplicated Dynamixel ID %d", id);
          return false;
        }
        slot_ids_[k] = id;
        id_to_slot_[static_cast<std::size_t>(id)] = static_cast<int>(k);
      }
    }
    if (slot_ids_[k] < 0) {
      RCLCPP_WARN(
        logger, "external commands: joint '%s' has no %s gpio ID; it cannot be addressed",
        slots[k].c_str(), gpio_name.c_str());
    }
  }

  gripper_slot_ = slot_of(config_.gripper_id);
  tilt_slot_ = slot_of(config_.tilt_id);
  if (!config_.gripper_topic.empty() && gripper_slot_ < 0) {
    RCLCPP_ERROR(
      logger, "external commands: gripper_id %d is not one of the controlled joints",
      config_.gripper_id);
    return false;
  }
  if (!config_.pan_tilt_topic.empty() && tilt_slot_ < 0) {
    RCLCPP_ERROR(
      logger, "external commands: tilt_id %d is not one of the controlled joints",
      config_.tilt_id);
    return false;
  }
  if (!std::isfinite(config_.status_rate_hz) || config_.status_rate_hz < 0.0) {
    RCLCPP_ERROR(logger, "external commands: status_rate_hz must be >= 0");
    return false;
  }

  // 상태 메시지 순서: ID 오름차순(0..6, 10..21, 22, 23). 받는 쪽은 순서가 아니라 id 로 찾는다.
  status_slots_.clear();
  for (int id = 0; id < 256; ++id) {
    if (id_to_slot_[static_cast<std::size_t>(id)] >= 0) {
      status_slots_.push_back(static_cast<std::size_t>(id_to_slot_[static_cast<std::size_t>(id)]));
    }
  }

  // id 는 고정이라 여기서 한 번만 채우고, 제어 스레드는 position/velocity/stamp 만 바꾼다.
  status_msg_ = StatusMsg();
  status_msg_.motor_state.resize(status_slots_.size());
  for (std::size_t m = 0; m < status_slots_.size(); ++m) {
    status_msg_.motor_state[m].id = static_cast<std::uint8_t>(slot_ids_[status_slots_[m]]);
  }

  pending_.assign(slots.size(), MotionCommand{});
  next_seq_ = 0;
  accepted_ = 0;
  rejected_ = 0;
  buffer_.writeFromNonRT(pending_);
  status_accumulator_s_ = 0.0;

  // colcon_ws 노드와 같은 QoS (명령은 best_effort, pan_tilt 는 기본 reliable).
  const auto best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  if (!config_.control_topic.empty()) {
    control_subscription_ =
      node_->create_subscription<dynamixel_interfaces::msg::DynamixelControlMsgs>(
      config_.control_topic, best_effort,
      std::bind(&ExternalCommands::control_callback, this, std::placeholders::_1));
  }
  if (!config_.gripper_topic.empty()) {
    gripper_subscription_ = node_->create_subscription<dynamixel_interfaces::msg::DynamixelMsgs>(
      config_.gripper_topic, best_effort,
      std::bind(&ExternalCommands::gripper_callback, this, std::placeholders::_1));
  }
  if (!config_.pan_tilt_topic.empty()) {
    pan_tilt_subscription_ = node_->create_subscription<dynamixel_interfaces::msg::PanTilt>(
      config_.pan_tilt_topic, rclcpp::QoS(10),
      std::bind(&ExternalCommands::pan_tilt_callback, this, std::placeholders::_1));
  }
  if (!config_.status_topic.empty() && config_.status_rate_hz > 0.0) {
    status_publisher_ = node_->create_publisher<StatusMsg>(config_.status_topic, best_effort);
    status_rt_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<StatusMsg>>(status_publisher_);
  }

  RCLCPP_INFO(
    logger,
    "external commands: %zu motors addressable by ID; control '%s', gripper '%s' (ID %d), "
    "pan_tilt '%s' (tilt ID %d), status '%s' at %.1f Hz",
    status_slots_.size(), config_.control_topic.c_str(), config_.gripper_topic.c_str(),
    config_.gripper_id, config_.pan_tilt_topic.c_str(), config_.tilt_id,
    config_.status_topic.c_str(), config_.status_rate_hz);
  return true;
}

void ExternalCommands::reset()
{
  control_subscription_.reset();
  gripper_subscription_.reset();
  pan_tilt_subscription_.reset();
  status_rt_publisher_.reset();
  status_publisher_.reset();
  node_.reset();
}

int ExternalCommands::slot_of(int id) const
{
  if (id < 0 || id > 255) {return -1;}
  return id_to_slot_[static_cast<std::size_t>(id)];
}

const std::vector<ExternalCommands::MotionCommand> & ExternalCommands::read_from_rt()
{
  return *buffer_.readFromRT();
}

void ExternalCommands::reject(const char * topic, const std::string & reason)
{
  // mutex_ 를 잡은 채로 불린다.
  ++rejected_;
  RCLCPP_ERROR_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Rejected %s message (%lu rejected so far): %s", topic,
    static_cast<unsigned long>(rejected_), reason.c_str());
}

bool ExternalCommands::validate(
  const dynamixel_interfaces::msg::DynamixelMsgs & motor, const char * topic,
  std::vector<bool> & seen, std::vector<Update> & out)
{
  const int slot = slot_of(motor.id);
  if (slot < 0) {
    reject(topic, "unknown motor id " + std::to_string(motor.id));
    return false;
  }
  if (seen[static_cast<std::size_t>(slot)]) {
    reject(topic, "duplicated motor id " + std::to_string(motor.id));
    return false;
  }
  seen[static_cast<std::size_t>(slot)] = true;
  if (!std::isfinite(motor.goal_position) || !std::isfinite(motor.profile_velocity) ||
    !std::isfinite(motor.profile_acceleration) || motor.profile_velocity < 0.0f ||
    motor.profile_acceleration < 0.0f)
  {
    reject(
      topic, "motor id " + std::to_string(motor.id) +
      " has a non-finite value or a negative profile");
    return false;
  }
  out.push_back(
    {static_cast<std::size_t>(slot), motor.goal_position, motor.profile_velocity,
      motor.profile_acceleration});
  return true;
}

void ExternalCommands::apply(const std::vector<Update> & updates)
{
  // mutex_ 를 잡은 채로 불린다. 한 메시지의 명령은 같은 seq 로 한꺼번에 보인다.
  const std::uint64_t seq = ++next_seq_;
  for (const auto & update : updates) {
    auto & command = pending_[update.slot];
    command.seq = seq;
    command.goal = update.goal;
    command.velocity = update.velocity;
    command.acceleration = update.acceleration;
  }
  ++accepted_;
  buffer_.writeFromNonRT(pending_);
}

void ExternalCommands::control_callback(
  dynamixel_interfaces::msg::DynamixelControlMsgs::SharedPtr msg)
{
  if (msg == nullptr) {return;}
  std::scoped_lock lock(mutex_);
  if (msg->motor_control.empty()) {
    reject("dynamixel_control", "motor_control is empty");
    return;
  }
  std::vector<bool> seen(pending_.size(), false);
  std::vector<Update> updates;
  updates.reserve(msg->motor_control.size());
  for (const auto & motor : msg->motor_control) {
    // tilt 의 명령 경로는 pan_tilt 토픽 하나다(예전 노드도 이 배열에서 tilt 를 뺐다).
    // 두 경로가 섞이면 비전 TF 가 가정한 각도와 실제 각도가 어긋난다.
    if (!config_.pan_tilt_topic.empty() && motor.id == config_.tilt_id) {
      reject(
        "dynamixel_control", "tilt id " + std::to_string(motor.id) +
        " is commanded only through " + config_.pan_tilt_topic);
      return;
    }
    if (!validate(motor, "dynamixel_control", seen, updates)) {return;}
  }
  apply(updates);
}

void ExternalCommands::gripper_callback(dynamixel_interfaces::msg::DynamixelMsgs::SharedPtr msg)
{
  if (msg == nullptr) {return;}
  std::scoped_lock lock(mutex_);
  // 전용 토픽이므로 id 가 그리퍼여야 한다. 다른 id 가 오면 보낸 쪽이 잘못 채운 것이다.
  if (msg->id != config_.gripper_id) {
    reject(
      "gripper_control", "id " + std::to_string(msg->id) + " is not the gripper id " +
      std::to_string(config_.gripper_id));
    return;
  }
  std::vector<bool> seen(pending_.size(), false);
  std::vector<Update> updates;
  if (validate(*msg, "gripper_control", seen, updates)) {apply(updates);}
}

void ExternalCommands::pan_tilt_callback(dynamixel_interfaces::msg::PanTilt::SharedPtr msg)
{
  if (msg == nullptr) {return;}
  std::scoped_lock lock(mutex_);
  // degree. colcon_ws 노드와 같이 프로파일 없이(속도 0) 바로 목표로 보낸다. pan 은 모터가 없다.
  if (msg->tilt < -180 || msg->tilt > 180) {
    reject("pan_tilt", "tilt " + std::to_string(msg->tilt) + " deg is outside [-180, 180]");
    return;
  }
  apply({{static_cast<std::size_t>(tilt_slot_), msg->tilt * kPi / 180.0, 0.0, 0.0}});
}

std::uint64_t ExternalCommands::accepted() const
{
  std::scoped_lock lock(mutex_);
  return accepted_;
}

std::uint64_t ExternalCommands::rejected() const
{
  std::scoped_lock lock(mutex_);
  return rejected_;
}

void ExternalCommands::publish_status(
  const rclcpp::Time & time, double dt,
  const std::vector<double> & policy_positions, const std::vector<double> & policy_velocities,
  const std::vector<double> & hold_positions, const std::vector<double> & hold_velocities)
{
  if (!status_rt_publisher_) {return;}
  status_accumulator_s_ += dt;
  if (status_accumulator_s_ + 1e-9 < 1.0 / config_.status_rate_hz) {return;}
  status_accumulator_s_ = 0.0;

  status_msg_.header.stamp = time;
  const std::size_t policy_count = policy_positions.size();
  for (std::size_t m = 0; m < status_slots_.size(); ++m) {
    const std::size_t k = status_slots_[m];
    const bool is_policy = k < policy_count;
    const std::size_t h = is_policy ? 0 : k - policy_count;
    auto & state = status_msg_.motor_state[m];
    state.position = is_policy ? policy_positions[k] : hold_positions[h];
    state.velocity = is_policy ? policy_velocities[k] : hold_velocities[h];
  }
  // 퍼블리셔 스레드가 바쁘면 이번 틱은 건너뛴다(제어 스레드를 막지 않는다).
  status_rt_publisher_->try_publish(status_msg_);
}

}  // namespace kid_rl
