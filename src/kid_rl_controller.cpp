// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/kid_rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace kid_rl
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

/// Largest period the controller integrates in one update. A longer gap means
/// the loop was starved; integrating it verbatim would produce a step change.
constexpr double kMaxIntegrationPeriodS = 0.1;
constexpr char kHardwareStateInterface[] = "hardware_state";

double clamp(double value, double low, double high) {return std::max(low, std::min(value, high));}

bool is_finite_vector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

std::int64_t to_nanoseconds(double seconds)
{
  return static_cast<std::int64_t>(seconds * 1e9);
}
}  // namespace

const char * KidRlController::mode_name(Mode mode)
{
  // Used only by set_mode()'s transition log, which is the primary tool for
  // reconstructing after the fact why a run stopped walking.
  switch (mode) {
    case Mode::WAITING_FOR_STATE: return "WAITING_FOR_STATE";
    case Mode::READY_POSE: return "READY_POSE";
    case Mode::SAFE_HOLD: return "SAFE_HOLD";
    case Mode::POLICY_RUNNING: return "POLICY_RUNNING";
    case Mode::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

controller_interface::CallbackReturn KidRlController::on_init()
{
  // Declares every parameter with the fallback used when the YAML omits it.
  // Runs before the node has a lifecycle state, so nothing here may touch the
  // hardware -- validation that needs the resolved joint count belongs in
  // on_configure(). The values that cannot survive a silent truncation (the
  // 16-bit gamepad IDs, the baud rate) throw instead of truncating.
  try {
    joints_ = auto_declare<std::vector<std::string>>("joints", {});
    default_positions_ = auto_declare<std::vector<double>>("default_positions", {});
    joint_position_min_ = auto_declare<std::vector<double>>("joint_position_min", {});
    joint_position_max_ = auto_declare<std::vector<double>>("joint_position_max", {});
    policy_rate_hz_ = auto_declare<double>("policy_rate_hz", 50.0);
    ready_pose_duration_s_ = auto_declare<double>("ready_pose_duration_s", 2.0);
    command_timeout_s_ = auto_declare<double>("command_timeout_s", 0.25);
    imu_timeout_s_ = auto_declare<double>("imu_timeout_s", 0.10);
    action_scale_ = auto_declare<double>("action_scale", 0.5);
    action_clip_ = auto_declare<double>("action_clip", 5.0);
    gait_swing_time_s_ = auto_declare<double>("gait_swing_time_s", 0.0);
    gait_double_support_band_ = auto_declare<double>("gait_double_support_band", 0.1);
    imu_gyro_filter_hz_ = auto_declare<double>("imu_gyro_filter_hz", 0.0);
    target_rate_limit_ = auto_declare<double>("target_rate_limit", 4.0);
    fall_gravity_z_threshold_ = auto_declare<double>("fall_gravity_z_threshold", -0.5);
    fall_debounce_s_ = auto_declare<double>("fall_debounce_s", 0.10);
    fault_recovery_s_ = auto_declare<double>("fault_recovery_s", 1.0);
    entry_position_tolerance_ = auto_declare<double>("entry_position_tolerance", 0.35);
    entry_check_skip_joints_ =
      auto_declare<std::vector<std::string>>("entry_check_skip_joints", {});
    fixed_target_joints_ =
      auto_declare<std::vector<std::string>>("fixed_target_joints", {});
    dry_run_policy_ = auto_declare<bool>("dry_run_policy", false);
    policy_path_ = auto_declare<std::string>("policy_path", "");

    gamepad_enabled_ = auto_declare<bool>("gamepad.enabled", true);
    gamepad_config_.device_path = auto_declare<std::string>("gamepad.device_path", "");
    gamepad_config_.name_hint =
      auto_declare<std::string>("gamepad.name_hint", "Gulikit X");
    const auto gamepad_vendor_id = auto_declare<std::int64_t>("gamepad.vendor_id", 0x045e);
    const auto gamepad_product_id = auto_declare<std::int64_t>("gamepad.product_id", 0x028e);
    if (gamepad_vendor_id < 0 || gamepad_vendor_id > 0xffff ||
      gamepad_product_id < 0 || gamepad_product_id > 0xffff)
    {
      throw std::invalid_argument("gamepad vendor_id/product_id must fit in 16 bits");
    }
    gamepad_config_.vendor_id = static_cast<std::uint16_t>(gamepad_vendor_id);
    gamepad_config_.product_id = static_cast<std::uint16_t>(gamepad_product_id);
    gamepad_config_.max_vx = auto_declare<double>("gamepad.max_vx", 0.4);
    gamepad_config_.max_vy = auto_declare<double>("gamepad.max_vy", 0.3);
    gamepad_config_.max_wz = auto_declare<double>("gamepad.max_wz", 0.7);
    gamepad_config_.deadband_vx = auto_declare<double>("gamepad.deadband_vx", 0.05);
    gamepad_config_.deadband_vy = auto_declare<double>("gamepad.deadband_vy", 0.05);
    gamepad_config_.deadband_wz = auto_declare<double>("gamepad.deadband_wz", 0.05);

    imu_enabled_ = auto_declare<bool>("imu.enabled", false);
    imu_config_.port = auto_declare<std::string>("imu.port", "");
    const auto imu_baudrate = auto_declare<std::int64_t>("imu.baudrate", 115200);
    if (imu_baudrate <= 0 || imu_baudrate > 10000000) {
      throw std::invalid_argument("imu.baudrate is invalid");
    }
    imu_config_.baudrate = static_cast<int>(imu_baudrate);
    const auto imu_format = auto_declare<std::string>("imu.format", "euler_gyro");
    if (!Imu::parse_format(imu_format, imu_config_.format)) {
      throw std::invalid_argument("imu.format must be euler_gyro or quaternion_gyro");
    }
    imu_config_.angle_scale = auto_declare<double>("imu.angle_scale", kPi / 180.0);
    imu_config_.gyro_scale = auto_declare<double>("imu.gyro_scale", kPi / 180.0);
    policy_slew_limit_ = auto_declare<double>("policy_slew_limit", 15.0);
    if (!std::isfinite(policy_slew_limit_) || policy_slew_limit_ < 0.0) {
      throw std::invalid_argument("policy_slew_limit must be finite and >= 0");
    }
    const auto slew_debounce = auto_declare<std::int64_t>("policy_slew_debounce_ticks", 3);
    if (slew_debounce < 1 || slew_debounce > 100) {
      throw std::invalid_argument("policy_slew_debounce_ticks must be between 1 and 100");
    }
    policy_slew_debounce_ticks_ = static_cast<int>(slew_debounce);
    log_enabled_ = auto_declare<bool>("log.enabled", true);
    log_path_ = auto_declare<std::string>("log.path", "");
    const auto log_decimation = auto_declare<std::int64_t>("log.decimation", 1);
    const auto log_buffer = auto_declare<std::int64_t>("log.buffer_samples", 4000);
    if (log_decimation <= 0 || log_decimation > 1000) {
      throw std::invalid_argument("log.decimation must be between 1 and 1000");
    }
    if (log_buffer < 64 || log_buffer > 1000000) {
      throw std::invalid_argument("log.buffer_samples must be between 64 and 1000000");
    }
    log_decimation_ = static_cast<int>(log_decimation);
    log_buffer_samples_ = static_cast<int>(log_buffer);
    const auto imu_axis_map = auto_declare<std::string>("imu.axis_map", "x,y,z");
    if (!Imu::parse_axis_map(imu_axis_map, imu_config_.axis_order, imu_config_.axis_sign)) {
      throw std::invalid_argument(
              "imu.axis_map must use x, y and z exactly once, e.g. \"x,y,z\" or \"-y,x,z\"");
    }
    // 비워 두면 자세와 같은 매핑을 쓴다. 이 장치처럼 두 신호의 규약이 다를 때만
    // 따로 지정한다.
    const auto imu_gyro_axis_map = auto_declare<std::string>("imu.gyro_axis_map", "");
    if (imu_gyro_axis_map.empty()) {
      imu_config_.gyro_axis_order = imu_config_.axis_order;
      imu_config_.gyro_axis_sign = imu_config_.axis_sign;
    } else if (!Imu::parse_axis_map(  // NOLINT(readability/braces)
        imu_gyro_axis_map, imu_config_.gyro_axis_order, imu_config_.gyro_axis_sign))
    {
      throw std::invalid_argument(
              "imu.gyro_axis_map must use x, y and z exactly once, e.g. \"y,x,-z\"");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration KidRlController::command_interface_configuration()
const
{
  // One position command per joint, claimed by name. INDIVIDUAL rather than ALL
  // so the controller_manager refuses to activate when the hardware does not
  // export exactly these, instead of handing over whatever it happens to have.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration KidRlController::state_interface_configuration() const
{
  // Position and velocity per joint; both feed the observation directly. The
  // order requested here is not relied upon -- map_interfaces() resolves the
  // indices by name at activation, since the manager may reorder them.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    // Exported by DynamixelHardware after decoding Hardware Error Status.
    // This currently reports an aggregate servo fault, not communication
    // freshness and not a per-servo bit mask; see the deployment audit.
    config.names.push_back(joint + "/" + kHardwareStateInterface);
  }
  // xacro track_rx=true 일 때만 관절마다 하나씩 더 붙는다.
  rx_missing_monitor_.append_interface_names(joints_, config.names);
  // xacro 에 type=bus_timing gpio 가 있을 때만 7개가 더 붙는다.
  bus_timing_monitor_.append_interface_names(config.names);
  // xacro 가 전압·PWM 을 선언한 관절마다 2개가 더 붙는다.
  motor_telemetry_monitor_.append_interface_names(joints_, config.names);
  return config;
}

controller_interface::CallbackReturn KidRlController::on_configure(const rclcpp_lifecycle::State &)
{
  // Validates the whole parameter set and sizes every buffer. This is the last
  // point where a bad configuration can be rejected cheaply, so the checks are
  // deliberately exhaustive: sizes, finiteness, ranges, duplicate joint names,
  // and each joint's default lying inside its own limits. Anything that slips
  // through becomes a hazard on a live robot instead of a startup error.
  const auto n = joints_.size();
  if (n == 0 || default_positions_.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "joints and default_positions must have equal, non-zero size");
    return CallbackReturn::ERROR;
  }
  if (joint_position_min_.empty()) {joint_position_min_.assign(n, -kPi);}
  if (joint_position_max_.empty()) {joint_position_max_.assign(n, kPi);}
  if (joint_position_min_.size() != n || joint_position_max_.size() != n ||
    !is_finite_vector(default_positions_) || !is_finite_vector(joint_position_min_) ||
    !is_finite_vector(joint_position_max_) || !std::isfinite(policy_rate_hz_) ||
    !std::isfinite(ready_pose_duration_s_) || !std::isfinite(command_timeout_s_) ||
    !std::isfinite(imu_timeout_s_) || !std::isfinite(action_scale_) ||
    !std::isfinite(action_clip_) || action_clip_ < 0.0 ||
    !std::isfinite(gait_swing_time_s_) || gait_swing_time_s_ < 0.0 ||
    !std::isfinite(gait_double_support_band_) || gait_double_support_band_ < 0.0 ||
    !std::isfinite(imu_gyro_filter_hz_) || imu_gyro_filter_hz_ < 0.0 ||
    gait_double_support_band_ >= 1.0 ||
    !std::isfinite(target_rate_limit_) ||
    !std::isfinite(fall_gravity_z_threshold_) ||
    !std::isfinite(fall_debounce_s_) || !std::isfinite(entry_position_tolerance_) ||
    !std::isfinite(fault_recovery_s_) ||
    policy_rate_hz_ <= 0.0 || ready_pose_duration_s_ <= 0.0 ||
    command_timeout_s_ <= 0.0 || imu_timeout_s_ <= 0.0 || action_scale_ < 0.0 ||
    target_rate_limit_ <= 0.0 || fall_debounce_s_ < 0.0 ||
    entry_position_tolerance_ < 0.0 ||
    fall_gravity_z_threshold_ < -1.0 || fall_gravity_z_threshold_ > 1.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller parameters");
    return CallbackReturn::ERROR;
  }
  entry_check_skip_.assign(n, false);
  for (const auto & name : entry_check_skip_joints_) {
    const auto it = std::find(joints_.begin(), joints_.end(), name);
    if (it == joints_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "entry_check_skip_joints: unknown joint '%s'", name.c_str());
      return CallbackReturn::ERROR;
    }
    entry_check_skip_[static_cast<std::size_t>(it - joints_.begin())] = true;
  }
  fixed_target_.assign(n, false);
  for (const auto & name : fixed_target_joints_) {
    const auto it = std::find(joints_.begin(), joints_.end(), name);
    if (it == joints_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "fixed_target_joints: unknown joint '%s'", name.c_str());
      return CallbackReturn::ERROR;
    }
    fixed_target_[static_cast<std::size_t>(it - joints_.begin())] = true;
  }
  if (!std::isfinite(gamepad_config_.max_vx) || !std::isfinite(gamepad_config_.max_vy) ||
    !std::isfinite(gamepad_config_.max_wz) || gamepad_config_.max_vx <= 0.0 ||
    gamepad_config_.max_vy <= 0.0 || gamepad_config_.max_wz <= 0.0 ||
    !std::isfinite(gamepad_config_.deadband_vx) || gamepad_config_.deadband_vx < 0.0 ||
    !std::isfinite(gamepad_config_.deadband_vy) || gamepad_config_.deadband_vy < 0.0 ||
    !std::isfinite(gamepad_config_.deadband_wz) || gamepad_config_.deadband_wz < 0.0 ||
    gamepad_config_.deadband_vx >= 1.0 || gamepad_config_.deadband_vy >= 1.0 ||
    gamepad_config_.deadband_wz >= 1.0 ||
    !std::isfinite(imu_config_.angle_scale) || !std::isfinite(imu_config_.gyro_scale) ||
    imu_config_.angle_scale <= 0.0 || imu_config_.gyro_scale <= 0.0 ||
    (imu_enabled_ && imu_config_.port.empty()))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid gamepad or IMU parameters");
    return CallbackReturn::ERROR;
  }
  const bool has_empty_joint = std::any_of(
    joints_.begin(), joints_.end(),
    [](const std::string & joint) {return joint.empty();});
  if (std::set<std::string>(joints_.begin(), joints_.end()).size() != n || has_empty_joint) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints must contain unique, non-empty names");
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (joint_position_min_[i] >= joint_position_max_[i] ||
      default_positions_[i] < joint_position_min_[i] ||
      default_positions_[i] > joint_position_max_[i])
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Invalid position limits/default for joint '%s'",
          joints_[i].c_str());
      return CallbackReturn::ERROR;
    }
  }

  const double update_rate = static_cast<double>(get_update_rate());
  if (update_rate > 0.0 && policy_rate_hz_ > update_rate) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
        "policy_rate_hz (%.1f) exceeds controller_manager update_rate (%.1f)",
      policy_rate_hz_, update_rate);
    return CallbackReturn::ERROR;
  }

  positions_.assign(n, 0.0);
  velocities_.assign(n, 0.0);
  activation_positions_.assign(n, 0.0);
  next_positions_.assign(n, 0.0);
  next_velocities_.assign(n, 0.0);
  targets_.assign(n, 0.0);
  actions_.assign(n, 0.0);
  last_actions_.assign(n, 0.0);
  // gyro(3) + projected gravity(3) + joint_pos(n) + joint_vel(n) + actions(n)
  // + command(3) [+ gait_phase(2)]. 위상 두 칸은 모델 메타데이터 observation_names 에
  // gait_phase 가 있을 때만 붙는다(아래 load 뒤에 크기를 다시 잡는다). 입력 폭이
  // 모델과 다르면 load 가 실패한다.
  observation_.assign(3 + 3 + 3 * n + 3, 0.0);
  uses_gait_phase_ = false;
  position_state_index_.assign(n, 0);
  velocity_state_index_.assign(n, 0);
  hardware_state_index_.assign(n, 0);
  rx_missing_monitor_.configure(get_robot_description(), joints_, get_node()->get_logger());
  bus_timing_monitor_.configure(get_robot_description(), get_node()->get_logger());
  motor_telemetry_monitor_.configure(get_robot_description(), joints_, get_node()->get_logger());
  position_command_index_.assign(n, 0);

  if (!dry_run_policy_) {
    if (policy_path_.empty()) {
      policy_path_ =
        ament_index_cpp::get_package_share_directory("kid_rl") + "/models/policy.onnx";
    }
    std::string policy_error;
    // build_observation() 이 조립하는 순서. 모델 메타데이터와 이름·순서가 같아야 한다.
    static const std::vector<std::string> kBaseObservationNames{
      "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions", "command"};
    if (!policy_runner_.load(
        policy_path_, kBaseObservationNames, observation_.size(), uses_gait_phase_,
        actions_.size(), joints_, default_positions_, action_scale_, policy_error))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not load policy '%s': %s",
        policy_path_.c_str(), policy_error.c_str());
      return CallbackReturn::ERROR;
    }
    if (uses_gait_phase_) {
      if (gait_swing_time_s_ <= 0.0) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Policy '%s' observes gait_phase but gait_swing_time_s is not set. It must equal "
          "the training _GAIT_SWING_TIME (mjlab_kid_RL kid_RL_env_cfg.py).",
          policy_path_.c_str());
        policy_runner_.reset();
        return CallbackReturn::ERROR;
      }
      gait_cycle_time_s_ = gait_cycle_time(gait_swing_time_s_, gait_double_support_band_);
      observation_.assign(observation_.size() + 2, 0.0);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Policy observes gait_phase: swing %.3f s, double support band %.3f -> cycle %.6f s, "
        "observation %zu", gait_swing_time_s_, gait_double_support_band_, gait_cycle_time_s_,
        observation_.size());
    }

    gyro_filter_.configure(imu_gyro_filter_hz_);
    if (gyro_filter_.enabled()) {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "gyro observation low-pass %.2f Hz (1st order, ~%.0f ms effective age at low "
        "frequency). The policy must be trained with the same filter.",
        imu_gyro_filter_hz_, 1000.0 / (6.283185307179586 * imu_gyro_filter_hz_));
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "gyro observation low-pass: off");
    }

    // Warm up ONNX Runtime outside update(). This pays lazy kernel setup costs
    // before the real-time controller starts and verifies the full I/O path.
    if (!policy_runner_.run(observation_, actions_) || !is_finite_vector(actions_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "ONNX policy warm-up inference failed");
      policy_runner_.reset();
      return CallbackReturn::ERROR;
    }
    std::fill(actions_.begin(), actions_.end(), 0.0);
  }

  policy_enabled_.store(false, std::memory_order_relaxed);

  policy_enable_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/enable_policy", 1,
    std::bind(&KidRlController::policy_enable_callback, this, std::placeholders::_1));

  if (dry_run_policy_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Configured %zu joints, observation size %zu, policy %.1f Hz. "
      "DRY RUN: the policy emits zero actions.",
      n, observation_.size(), policy_rate_hz_);
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Loaded ONNX policy '%s': observation [1,%zu], actions [1,%zu], %.1f Hz",
      policy_path_.c_str(), observation_.size(), actions_.size(), policy_rate_hz_);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KidRlController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Mirror of on_configure(): release the input devices and the subscription so
  // a reconfigure starts from a clean slate. Only reachable from `inactive`, so
  // the RT loop is stopped and tearing down the device threads here is safe.
  gamepad_.stop();
  imu_.stop();
  policy_runner_.reset();
  policy_enable_subscription_.reset();
  mode_ = Mode::WAITING_FOR_STATE;
  return CallbackReturn::SUCCESS;
}

bool KidRlController::map_interfaces()
{
  // Resolve interfaces by name instead of relying on the order in which the
  // controller_manager happened to hand them over.
  //
  // The scan is O(joints x interfaces), but it runs once per activation and
  // never in update(). At 25 joints it measures ~0.75 us; an index map costs
  // more to build than this costs to run, and would allocate a key string per
  // lookup where this allocates nothing (the handles return const std::string &
  // and std::string::operator== rejects on length before touching any bytes).
  //
  // State and command handles are unrelated types, so the lambda is generic
  // over the container rather than duplicated per kind.
  const auto find = [this](
    const auto & interfaces, const char * kind, const std::string & joint,
    const char * type, std::size_t & out)
    {
      for (std::size_t k = 0; k < interfaces.size(); ++k) {
        if (interfaces[k].get_prefix_name() == joint &&
          interfaces[k].get_interface_name() == type)
        {
          out = k;
          return true;
        }
      }
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing %s interface %s/%s", kind, joint.c_str(), type);
      return false;
    };

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (!find(
        state_interfaces_, "state", joints_[i],
        hardware_interface::HW_IF_POSITION, position_state_index_[i]) ||
      !find(
        state_interfaces_, "state", joints_[i],
        hardware_interface::HW_IF_VELOCITY, velocity_state_index_[i]) ||
      !find(
        state_interfaces_, "state", joints_[i],
        kHardwareStateInterface, hardware_state_index_[i]) ||
      !find(
        command_interfaces_, "command", joints_[i],
        hardware_interface::HW_IF_POSITION, position_command_index_[i]))
    {
      return false;
    }
  }
  return true;
}

controller_interface::CallbackReturn KidRlController::on_activate(const rclcpp_lifecycle::State &)
{
  // Claims the interfaces and takes the robot from "unknown pose" to a known
  // starting state. Every piece of carried-over state is cleared here -- the
  // actions, the observation, all three time accumulators -- so a
  // deactivate/activate cycle can never feed the policy data from the previous
  // run. A missing gamepad only warns (the command degrades to zero velocity,
  // which the policy was trained on); a missing IMU is fatal, since projected
  // gravity has no safe substitute.
  const std::size_t expected_state_interfaces =
    (3 + rx_missing_monitor_.interfaces_per_joint()) * joints_.size() +
    bus_timing_monitor_.interface_count() + motor_telemetry_monitor_.interface_count();
  if (command_interfaces_.size() != joints_.size() ||
    state_interfaces_.size() != expected_state_interfaces)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Interface count mismatch: %zu command (expected %zu), %zu state (expected %zu)",
      command_interfaces_.size(), joints_.size(), state_interfaces_.size(),
      expected_state_interfaces);
    return CallbackReturn::ERROR;
  }
  gyro_filter_.reset();
  if (!map_interfaces() ||
    !rx_missing_monitor_.map(state_interfaces_, joints_, get_node()->get_logger()) ||
    !bus_timing_monitor_.map(state_interfaces_, get_node()->get_logger()) ||
    !motor_telemetry_monitor_.map(state_interfaces_, joints_, get_node()->get_logger()) ||
    !read_joint_state() || !hardware_state_ok())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not read a valid joint state at activation");
    return CallbackReturn::ERROR;
  }

  std::string input_error;
  if (gamepad_enabled_ && !gamepad_.start(gamepad_config_, input_error)) {
    RCLCPP_WARN(
      get_node()->get_logger(), "Gamepad is unavailable; velocity command stays zero: %s",
      input_error.c_str());
  }
  if (imu_enabled_ && !imu_.start(imu_config_, input_error)) {
    gamepad_.stop();
    RCLCPP_ERROR(get_node()->get_logger(), "Could not start IMU: %s", input_error.c_str());
    return CallbackReturn::ERROR;
  }

  // 기록은 활성화될 때 시작한다. 파일을 못 열어도 활성화를 막지는 않는다.
  // 로깅 때문에 로봇을 못 띄우는 것이 더 나쁘기 때문이다.
  log_tick_ = 0;
  log_time_s_ = 0.0;
  if (log_enabled_) {
    std::vector<std::string> mode_names;
    for (const auto mode : {Mode::WAITING_FOR_STATE, Mode::READY_POSE, Mode::SAFE_HOLD,
        Mode::POLICY_RUNNING, Mode::FAULT})
    {
      mode_names.emplace_back(mode_name(mode));
    }
    std::string log_error;
    if (logger_.start(
        log_path_, joints_, mode_names, static_cast<std::size_t>(log_buffer_samples_),
        rx_missing_monitor_.enabled(), bus_timing_monitor_.enabled(),
        motor_telemetry_monitor_.joint_index(), log_error))
    {
      RCLCPP_INFO(get_node()->get_logger(), "CSV log: %s", logger_.path().c_str());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "CSV log disabled: %s", log_error.c_str());
    }
  }

  activation_positions_ = positions_;
  targets_ = positions_;
  std::fill(actions_.begin(), actions_.end(), 0.0);
  std::fill(last_actions_.begin(), last_actions_.end(), 0.0);
  std::fill(observation_.begin(), observation_.end(), 0.0);
  action_clip_warned_ = false;
  gait_clock_.restart();
  ready_pose_elapsed_s_ = 0.0;
  policy_accumulator_s_ = 0.0;
  fall_timer_s_ = 0.0;
  fault_clear_s_ = 0.0;
  fault_recoverable_ = true;
  // Activation always starts from the measured pose, never from a stale target.
  set_mode(Mode::READY_POSE, "activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KidRlController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop the device threads and drop back to WAITING_FOR_STATE. The joint
  // targets are deliberately left alone: ros2_control stops calling update()
  // on deactivation, so whatever holds the joints afterwards is outside this
  // controller's authority.
  gamepad_.stop();
  imu_.stop();
  // 남은 샘플을 모두 파일에 쓰고 닫는다. 이걸 빼먹으면 마지막 몇 초, 즉 보통
  // 가장 보고 싶은 구간이 사라진다.
  if (logger_.running()) {
    const std::size_t dropped = logger_.dropped();
    const std::string path = logger_.path();
    logger_.stop();
    RCLCPP_INFO(
      get_node()->get_logger(), "CSV log closed: %s%s", path.c_str(),
      dropped > 0 ? " (buffer overflowed; some samples were dropped)" : "");
  }
  mode_ = Mode::WAITING_FOR_STATE;
  return CallbackReturn::SUCCESS;
}

void KidRlController::policy_enable_callback(std_msgs::msg::Bool::SharedPtr msg)
{
  // This callback runs on a non-RT executor thread while update() reads on the
  // controller thread. A single bool needs no double buffer: the atomic store
  // publishes one complete true/false value without a data race or allocation.
  // The flag is latched, so a dropped message leaves the previous state intact.
  policy_enabled_.store(msg->data, std::memory_order_relaxed);
  RCLCPP_INFO(
    get_node()->get_logger(), "enable_policy received: %s", msg->data ? "true" : "false");
}

bool KidRlController::read_joint_state()
{
  // Read into staging buffers first. If even one interface is unavailable or
  // non-finite, the published snapshot remains the last complete, valid one;
  // update() can therefore keep the last valid command and retry next tick
  // without combining new values from early joints with stale values from the
  // remaining joints.
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto position = state_interfaces_[position_state_index_[i]].get_optional();
    const auto velocity = state_interfaces_[velocity_state_index_[i]].get_optional();
    if (!position || !velocity || !std::isfinite(*position) || !std::isfinite(*velocity)) {
      return false;
    }
    next_positions_[i] = *position;
    next_velocities_[i] = *velocity;
  }

  // No allocation occurs: both vectors were sized in on_configure(). After the
  // swap, positions_ and velocities_ represent one complete controller tick.
  positions_.swap(next_positions_);
  velocities_.swap(next_velocities_);
  return true;
}

void KidRlController::log_tick(
  double dt, const CommandSample & command, const ImuSample & imu,
  bool command_fresh, bool imu_fresh)
{
  // 제어 스레드에서 불린다. 여기서 하는 일은 미리 할당된 구조체를 채워
  // 링 버퍼에 복사하는 것뿐이다. 파일 접근도 할당도 없다.
  if (!logger_.running()) {return;}
  log_time_s_ += dt;
  if (log_decimation_ > 1 && (log_tick_++ % static_cast<std::uint64_t>(log_decimation_)) != 0) {
    return;
  }

  log_sample_.t_s = log_time_s_;
  log_sample_.dt_s = dt;
  log_sample_.mode = static_cast<int>(mode_);
  log_sample_.imu_valid = imu_fresh ? 1 : 0;
  // 샘플이 몇 ms 묵었는지. imu_valid 는 imu_timeout_s 를 넘겨야 0 이 되므로,
  // 그보다 짧은 끊김은 이 값으로만 드러난다. IMU 가 끊겨도 제어 틱은 계속 돌아
  // 같은 샘플이 반복 기록되므로, 나이가 자라는 모습이 그대로 남는다.
  // IMU 나이와 관절값 나이를 같은 '지금'으로 재야 둘을 나란히 비교할 수 있다.
  const std::int64_t log_now_ns = steady_time_nanoseconds();
  log_sample_.imu_age_ms = imu.valid ?
    static_cast<double>(log_now_ns - imu.stamp_ns) / 1.0e6 : -1.0;
  bus_timing_monitor_.fill(
    state_interfaces_, static_cast<double>(log_now_ns) * 1.0e-9, log_sample_);
  log_sample_.command_fresh = command_fresh ? 1 : 0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    log_sample_.gravity[axis] = imu.projected_gravity[axis];
    log_sample_.gyro[axis] = imu.angular_velocity[axis];
    log_sample_.gyro_raw[axis] = imu_gyro_raw_[axis];
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    log_sample_.command[axis] = command.velocity[axis];
  }
  // 이번 틱 관측에 들어간 위상 [0, 1). 위상을 쓰지 않거나 정책이 돌지 않을 때는 -1.
  // 진입 틱은 아직 추론 전(policy_first_setpoint_ 가 참)이라 -1. 첫 추론 틱이 0 이고,
  // 이후는 GaitClock 과 같은 실제 경과 시간이다. sim 로그와 정렬하는 기준.
  const bool policy_started = mode_ == Mode::POLICY_RUNNING && !policy_first_setpoint_;
  log_sample_.policy_t_s = policy_started ? gait_clock_.elapsed_s() : -1.0;
  log_sample_.gait_phase = (uses_gait_phase_ && policy_started) ?
    gait_phase(gait_clock_.elapsed_s(), gait_cycle_time_s_) : -1.0;

  const std::size_t count = std::min<std::size_t>(joints_.size(), CsvLogger::kMaxJoints);
  log_sample_.joint_count = count;
  rx_missing_monitor_.fill(state_interfaces_, log_sample_);
  motor_telemetry_monitor_.fill(state_interfaces_, log_sample_);
  for (std::size_t i = 0; i < count; ++i) {
    log_sample_.position[i] = positions_[i];
    log_sample_.velocity[i] = velocities_[i];
    log_sample_.target[i] = targets_[i];
    log_sample_.action[i] = actions_[i];
    const auto state = state_interfaces_[hardware_state_index_[i]].get_optional();
    log_sample_.hardware_state[i] = state ? *state : -1.0;
  }

  logger_.log(log_sample_);
}

bool KidRlController::hardware_state_ok()
{
  // Reads the joint "hardware_state" interface.  Despite the per-joint name,
  // DynamixelHardware writes the SAME bus-wide aggregate error_state into all
  // of them (dynamixel_hardware_interface.cpp:889-895), not each servo's
  // Hardware Error Status byte -- so this detects "some servo faulted", never
  // which one.  Input Voltage Error is masked out upstream and never appears.
  // A missing/non-finite health value is also unsafe.  Communication errors are
  // not visible here because the driver returns before updating this interface
  // on that path.
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto state = state_interfaces_[hardware_state_index_[i]].get_optional();
    if (!state || !std::isfinite(*state) || *state != 0.0) {return false;}
  }
  return true;
}

void KidRlController::set_mode(Mode mode, const char * reason)
{
  // The single place mode_ changes, so every transition is logged with the
  // reason that caused it. Self-transitions are dropped to keep the log
  // readable when a condition holds across many ticks.
  if (mode_ == mode) {return;}
  RCLCPP_INFO(
    get_node()->get_logger(), "Mode %s -> %s (%s)", mode_name(mode_), mode_name(mode), reason);
  mode_ = mode;
}

void KidRlController::hold_measured_pose()
{
  // Re-anchor the command on where the joints actually are. The tracking error
  // becomes zero, so the servos see no step -- that is what makes this safe to
  // call from a fault or a loop overrun, whatever the previous target was.
  targets_ = positions_;
}

void KidRlController::approach(const std::vector<double> & goal, double dt)
{
  // Rate-limited move towards `goal`, for the modes where the controller --
  // not the policy -- generates the motion: SAFE_HOLD's return to the default
  // pose, and the path out of POLICY_RUNNING when the operator disables it.
  // The policy path is deliberately NOT rate limited; see
  // update_policy_setpoint() for why.
  const double max_delta = target_rate_limit_ * dt;
  for (std::size_t i = 0; i < targets_.size(); ++i) {
    targets_[i] += clamp(goal[i] - targets_[i], -max_delta, max_delta);
  }
}

bool KidRlController::ready_to_start_policy(const ImuSample & imu) const
{
  // Entry gate for POLICY_RUNNING. The policy was trained starting from an
  // upright robot near its default pose, so handing it control from an
  // arbitrary pose asks it to extrapolate outside its training distribution.
  // Both conditions are checked against measured state, never against the
  // targets, which may be anywhere the previous mode left them.
  if (imu.projected_gravity[2] > fall_gravity_z_threshold_) {return false;}
  if (entry_position_tolerance_ <= 0.0) {return true;}
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (entry_check_skip_[i]) {continue;}
    if (std::abs(positions_[i] - default_positions_[i]) > entry_position_tolerance_) {return false;}
  }
  return true;
}

void KidRlController::build_observation(const CommandSample & command, const ImuSample & imu)
{
  // Packs the observation in the exact order the policy was trained on. This
  // layout is a contract with the ONNX file -- its `observation_names` metadata
  // reads base_ang_vel, projected_gravity, joint_pos, joint_vel, actions,
  // command -- and a mismatch produces a policy that runs happily and walks
  // into the floor. The order is fixed; do not tidy it.
  //
  // last_actions_ carries the policy output after the action clip, which is what
  // the policy fed back to itself during training: mjlab's last_action term reads
  // env.action_manager.action, and the RSL-RL wrapper clamps to agent.clip_actions
  // before env.step() (mjlab/rl/vecenv_wrapper.py:75-76).
  std::size_t k = 0;
  for (double v : imu.angular_velocity) {
    observation_[k++] = v;
  }
  for (double v : imu.projected_gravity) {
    observation_[k++] = v;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    observation_[k++] = positions_[i] - default_positions_[i];
  }
  for (double v : velocities_) {
    observation_[k++] = v;
  }
  for (double v : last_actions_) {
    observation_[k++] = v;
  }
  for (double v : command.velocity) {
    observation_[k++] = v;
  }
  // gait_phase: [sin(2*pi*phase), cos(2*pi*phase)], mjlab_kid_RL tasks/mdp.py 와 같은 식.
  // 지연·노이즈·스케일 없이 들어간다(학습 설정 그대로).
  if (uses_gait_phase_) {
    gait_phase_observation(
      gait_clock_.elapsed_s(), gait_cycle_time_s_, observation_[k], observation_[k + 1]);
    k += 2;
  }
}

bool KidRlController::run_policy()
{
  if (dry_run_policy_) {
    std::fill(actions_.begin(), actions_.end(), 0.0);
    return true;
  }
  // The exported model contains its observation normalizer, so the raw
  // observation is passed directly. PolicyRunner reuses preallocated float
  // tensors and writes one action per configured joint.
  return policy_runner_.run(observation_, actions_);
}

void KidRlController::update_policy_setpoint()
{
  // Runs at policy_rate_hz and writes the joint targets directly. Between
  // policy ticks the targets are simply held (zero-order hold): training fed
  // the policy target straight to the joints -- no low-pass filter, no rate
  // limit, latency modelled as a 15-30 ms pure delay in the actuator -- so any
  // smoothing here would command a trajectory the policy never produced.
  // Smooth motion comes from the joint-level PD in the servo firmware.
  //
  // Action clip = 학습 계약의 일부다 (2026-09-23 수정).
  // 예전 주석은 "학습에 clip 이 없다"고 했지만, 2026-09-21 부터 학습 설정에
  // agent.clip_actions = 5.0 이 들어갔고 현재 policy.onnx(run 2026-09-23_14-03-13)도
  // 그 값으로 학습됐다. 래퍼가 env.step() 전에 ±5 로 자르므로 목표각도, 다음
  // 관측의 last_action 도 자른 값이다. 실기에서 자르지 않으면 정책은 한 번도 본 적
  // 없는 |last_action| > 5 를 입력받는다.
  // 2026-09-23 22:06 실기: 몸이 약 40도 기울며 |action| 이 9.75 가 됐고, 그 값이
  // 그대로 되먹임되며 20 ms 마다 6~17 배씩 커져 0.8 초 뒤 NaN 으로 FAULT 났다.
  // 그동안 관절 22개가 한계까지 명령받았다. 그래서 목표 계산 전에 자르고,
  // 자른 값을 last_actions_ 에도 넣는다(아래 루프).
  if (action_clip_ > 0.0) {
    bool clipped = false;
    for (double & a : actions_) {
      if (a > action_clip_ || a < -action_clip_) {
        a = clamp(a, -action_clip_, action_clip_);
        clipped = true;
      }
    }
    if (clipped && !action_clip_warned_) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Policy action hit the action_clip (%.1f); the policy is outside its trained "
        "range. Logged once per activation.", action_clip_);
      action_clip_warned_ = true;
    }
  }
  // The joint-limit clamp below is the remaining bound, and it is a physical
  // limit rather than a reshaping of the policy output.
  // 발산 감지. 명령을 다듬지 않고(그러면 정책이 내지 않은 궤적이 된다) 대신
  // "정상 보행이라면 나올 수 없는 변화율"이 보이면 폴트로 멈춘다.
  //
  // 2026-09-21 실기에서 이것 없이 3.8초 동안 발산했다. 당시 기록:
  // 목표값이 초당 8~9회 부호를 뒤집으며 진폭 2.4~4.4 rad, |action| 최대 9.35.
  // 무릎 스윙의 정상 최대가 5~8 rad/s 이므로 그 두 배를 기준으로 잡는다.
  const double policy_period = 1.0 / policy_rate_hz_;
  double worst_slew = 0.0;
  std::size_t worst_joint = 0;
  std::vector<double> candidate(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    // fixed_target_joints: 학습에서 목표각을 고정한 관절(joint_pos_action.clip)은
    // 정책 출력이 목표에 아무 영향을 주지 못했으므로 그 출력은 의미 없는 값이다.
    // 목표만 default 로 고정하고, actions_ / last_actions_ 는 건드리지 않는다
    // (학습의 last_action 관측은 자르기 전의 정책 출력이다).
    const double action = fixed_target_[i] ? 0.0 : actions_[i];
    candidate[i] = clamp(
      default_positions_[i] + action_scale_ * action,
      joint_position_min_[i], joint_position_max_[i]);
    const double slew = std::abs(candidate[i] - targets_[i]) / policy_period;
    if (slew > worst_slew) {
      worst_slew = slew;
      worst_joint = i;
    }
  }
  const bool over_limit = policy_slew_limit_ > 0.0 && worst_slew > policy_slew_limit_;
  if (policy_first_setpoint_) {
    // 진입 계단은 정상이므로 통과시키되, 얼마나 컸는지는 남긴다. 이 값이
    // 관절 가동범위에 육박하면 진입 자세가 정책의 평형과 너무 멀다는 뜻이다.
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Policy engaged: first setpoint steps %.3f rad on '%s' (%.1f rad/s over one policy tick)",
      worst_slew * policy_period, joints_[worst_joint].c_str(), worst_slew);
    policy_first_setpoint_ = false;
    policy_slew_strikes_ = 0;
  } else if (over_limit) {
    // 한 틱 튀는 것과 계속 발산하는 것은 다르다. 기동 직후에는 관측에 들어가는
    // 이전 액션이 0 -> 큰 값으로 바뀌면서 두세 틱 과도현상이 생기는데, 학습에서도
    // 에피소드 시작 때 같은 일이 일어나므로 정책이 겪어 본 상황이다.
    // 반면 진짜 발산은 매 틱 이어진다. 그래서 연속 횟수로 가른다.
    ++policy_slew_strikes_;
    if (policy_slew_strikes_ >= policy_slew_debounce_ticks_) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Policy target slew %.1f rad/s on '%s' exceeds %.1f rad/s for %d consecutive policy "
        "ticks (action %.3f); the policy is diverging",
        worst_slew, joints_[worst_joint].c_str(), policy_slew_limit_,
        policy_slew_strikes_, actions_[worst_joint]);
      // 목표는 건드리지 않는다. enter_fault() 가 측정 자세로 고정한다.
      enter_fault("policy target slew exceeded", false);
      return;
    }
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Policy target slew %.1f rad/s on '%s' exceeds %.1f rad/s (%d/%d); "
      "allowing it as a transient",
      worst_slew, joints_[worst_joint].c_str(), policy_slew_limit_,
      policy_slew_strikes_, policy_slew_debounce_ticks_);
  } else {
    policy_slew_strikes_ = 0;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    last_actions_[i] = actions_[i];
    targets_[i] = candidate[i];
  }
}

void KidRlController::enter_fault(const char * reason, bool recoverable)
{
  // Failure state. Freezing at the measured pose rather than driving to the
  // default pose is deliberate: a fault means the robot is already in a
  // situation the controller does not understand, and moving it anywhere --
  // even somewhere nominally safe -- may be the wrong call.
  //
  // `recoverable` says whether the cause is the kind that can pass on its own.
  // A dropped servo reply or a stale IMU can; a policy that emitted a
  // non-finite action cannot -- retrying it would just fault again a second
  // later, with the robot briefly walking each time.
  if (mode_ != Mode::FAULT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Fault: %s. Freezing the joint targets at the measured pose; %s", reason,
      (recoverable && fault_recovery_s_ >= 0.0) ?
          "will return to SAFE_HOLD once the condition clears." :
          "deactivate and reactivate the controller to recover.");
    hold_measured_pose();
    fault_recoverable_ = recoverable;
    fault_clear_s_ = 0.0;
    mode_ = Mode::FAULT;
  }
}

bool KidRlController::write_commands()
{
  // Single choke point: nothing reaches the hardware without a finiteness check
  // and a joint-limit clamp, whatever mode produced the target.
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (!std::isfinite(targets_[i])) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Non-finite target for joint '%s'", joints_[i].c_str());
      return false;
    }
    targets_[i] = clamp(targets_[i], joint_position_min_[i], joint_position_max_[i]);
    if (!command_interfaces_[position_command_index_[i]].set_value(targets_[i])) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not write command for joint '%s'", joints_[i].c_str());
      return false;
    }
  }
  return true;
}

controller_interface::return_type KidRlController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // The RT tick, at the controller_manager rate (target 200 Hz), and the only
  // place the mode machine advances. Shape:
  //
  //   read state -> overrun guard -> snapshot inputs -> mode machine -> write
  //
  // Every path out of here, including each early return and each mode, exits
  // through write_commands(), so the hardware always receives a fresh, checked
  // command; there is no tick that leaves the command interface stale.
  //
  // The policy itself runs at policy_rate_hz via the accumulator in
  // POLICY_RUNNING. The remaining ticks are not idle: they refresh the joint
  // state, run the watchdogs and the fall debounce at full resolution, and
  // hold the standing target (zero-order hold).
  if (!read_joint_state()) {
    // A missing sample is not a controller fault. Keep the last finite target,
    // leave Dynamixel torque enabled, and retry the complete snapshot on the
    // next controller tick. write_commands() still rejects an invalid command
    // or a failed command-interface write.
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Joint state unavailable or non-finite; keeping the last valid command");
    return write_commands() ?
           controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  if (!hardware_state_ok()) {
    // A servo-side over-temperature/overload/encoder fault can disable torque
    // and may require a reboot after the physical cause is removed.  It must
    // never auto-resume the policy.
    // NOT voltage: the input-voltage bit is masked out of the aggregate
    // error_state that feeds this interface (dynamixel.hpp:89
    // kInputVoltageErrorMask), so an Input Voltage Error can never reach here.
    // That is deliberate for this robot -- it stays diagnostic-only.
    enter_fault("DYNAMIXEL hardware error", false);
    return write_commands() ?
           controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  // Loop overrun. `period` is measured wall-clock time, so a starved RT thread
  // can report half a second. Nothing here can meaningfully integrate a gap
  // that long -- approach() would get a max_delta of target_rate_limit * 0.5 =
  // 2 rad, which is no limit at all and slams the joints -- and the robot was
  // genuinely uncommanded for that whole window. Re-anchor the targets on the
  // measured pose (zero tracking error, no step) and hand control back to
  // SAFE_HOLD, which walks them to the default pose under the rate limit. The
  // policy can re-enter from there once the entry checks pass again.
  if (period.seconds() > kMaxIntegrationPeriodS) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Control loop overrun: %.1f ms (limit %.1f ms); holding the measured pose",
      period.seconds() * 1000.0, kMaxIntegrationPeriodS * 1000.0);
    if (mode_ != Mode::FAULT && mode_ != Mode::WAITING_FOR_STATE) {
      hold_measured_pose();
      set_mode(Mode::SAFE_HOLD, "control loop overrun");
      policy_accumulator_s_ = 0.0;
      fall_timer_s_ = 0.0;
    }
    return write_commands() ?
           controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  // The upper bound is unreachable after the overrun guard above; the lower one
  // still matters, since a backwards clock step yields a negative period and a
  // negative max_delta would invert every rate limit.
  const double dt = clamp(period.seconds(), 0.0, kMaxIntegrationPeriodS);

  // Gamepad and IMU snapshots come from their RealtimeBuffers. The enable flag
  // is a single independent bool, so an atomic load is sufficient.
  CommandSample command = gamepad_.read_from_rt();
  ImuSample imu = imu_.read_from_rt();
  const bool policy_requested = policy_enabled_.load(std::memory_order_relaxed);

  // 반드시 샘플을 읽은 '뒤에' 시각을 잡는다. 예전에는 now_ns 를 먼저 잡았는데,
  // 그 사이(수 us)에 IMU 스레드가 새 샘플을 찍어 넣으면 stamp_ns > now_ns 가 되어
  // 아래 ">= 0" 검사가 방금 도착한 샘플을 '미래에서 온 것'으로 보고 무효 처리했다.
  // 그 한 틱이 POLICY_RUNNING 에서 "IMU watchdog expired" FAULT 를 걸었고
  // fault_recovery_s = -1 이라 풀리지 않았다. 2026-09-23 CSV 에서 FAULT 로 끝난
  // 4회 모두 해당 틱의 imu_age_ms 가 0.21~0.33 ms(= 멀쩡한 샘플)였다.
  // 읽은 뒤에 잡으면 스탬프는 항상 now_ns 이전이므로 이 경합이 사라진다.
  const std::int64_t now_ns = steady_time_nanoseconds();

  const bool command_fresh = command.valid &&
    now_ns - command.stamp_ns >= 0 &&
    now_ns - command.stamp_ns <= to_nanoseconds(command_timeout_s_);
  const bool imu_fresh = imu.valid &&
    now_ns - imu.stamp_ns >= 0 &&
    now_ns - imu.stamp_ns <= to_nanoseconds(imu_timeout_s_);

  // gyro 저역통과 (imu_gyro_filter_hz). 관측에 들어가는 값만 거른다 -- 넘어짐 판정은
  // projected_gravity 를 쓰므로 영향이 없다. 모드와 무관하게 매 틱 갱신해 두어서
  // 정책이 시작될 때 필터가 이미 최근 움직임에 맞춰져 있다. IMU 가 끊기면 다시 시작한다.
  imu_gyro_raw_ = imu.angular_velocity;
  if (imu_fresh) {
    imu.angular_velocity = gyro_filter_.update(imu.angular_velocity, dt);
  } else {
    gyro_filter_.reset();
  }

  // A stale velocity command degrades to "stand still", which is a command the
  // policy was trained on. A stale IMU has no safe substitute and trips the
  // watchdog instead.
  if (!command_fresh) {command.velocity = {0.0, 0.0, 0.0};}

  switch (mode_) {
    case Mode::WAITING_FOR_STATE:
      activation_positions_ = positions_;
      hold_measured_pose();
      ready_pose_elapsed_s_ = 0.0;
      set_mode(Mode::READY_POSE, "valid joint state available");
      break;

    case Mode::READY_POSE: {
        ready_pose_elapsed_s_ += dt;
        const double ratio = clamp(ready_pose_elapsed_s_ / ready_pose_duration_s_, 0.0, 1.0);
        // Smoothstep: zero velocity at both ends, so no torque step at the
        // start and no overshoot at the end.
        const double smooth = ratio * ratio * (3.0 - 2.0 * ratio);
        for (std::size_t i = 0; i < joints_.size(); ++i) {
          targets_[i] = activation_positions_[i] +
            smooth * (default_positions_[i] - activation_positions_[i]);
        }
        if (ratio >= 1.0) {set_mode(Mode::SAFE_HOLD, "ready pose reached");}
        break;
      }

    case Mode::SAFE_HOLD:
      approach(default_positions_, dt);
      if (policy_requested) {
        if (!imu_fresh) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "Policy requested but the IMU is stale or invalid; staying in SAFE_HOLD");
        } else if (!ready_to_start_policy(imu)) {
          // 두 조건을 뭉뚱그려 말하면 현장에서 무엇을 고쳐야 할지 알 수 없다.
          // 어느 쪽에 걸렸는지, 관절이면 어느 관절이 얼마나 벗어났는지 남긴다.
          std::size_t worst = 0;
          double worst_error = 0.0;
          std::size_t beyond = 0;
          for (std::size_t i = 0; i < joints_.size(); ++i) {
            if (entry_check_skip_[i]) {continue;}
            const double error = std::abs(positions_[i] - default_positions_[i]);
            if (error > entry_position_tolerance_) {++beyond;}
            if (error > worst_error) {
              worst_error = error;
              worst = i;
            }
          }
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "Policy requested but entry checks failed; staying in SAFE_HOLD. "
            "upright: gravity_z=%+.3f (needs <= %+.3f) -> %s | "
            "pose: %zu/%zu joints beyond %.3f rad, worst '%s' off by %.3f rad "
            "(measured %+.3f, default %+.3f)",
            imu.projected_gravity[2], fall_gravity_z_threshold_,
            imu.projected_gravity[2] <= fall_gravity_z_threshold_ ? "OK" : "FAIL",
            beyond, joints_.size(), entry_position_tolerance_,
            joints_[worst].c_str(), worst_error, positions_[worst], default_positions_[worst]);
        } else {
          std::fill(actions_.begin(), actions_.end(), 0.0);
          std::fill(last_actions_.begin(), last_actions_.end(), 0.0);
          // targets_ is left where SAFE_HOLD put it (at or near the default
          // pose). The first inference overwrites it on the very next tick --
          // the entry checks above already guarantee the joints are within
          // entry_position_tolerance of default, so that step is small.
          fall_timer_s_ = 0.0;
          // Prime the accumulator so the first inference happens immediately.
          policy_accumulator_s_ = 1.0 / policy_rate_hz_;
          // 진입 후 첫 추론은 변화율 검사에서 뺀다. 정책의 평형 자세는
          // default_positions 와 다르기 때문에(관측이 0 이어도 출력은 0 이 아니다)
          // 첫 틱에는 항상 계단이 생긴다. 학습에서도 마찬가지이므로 발산이 아니다.
          policy_first_setpoint_ = true;
          // 학습의 에피소드 시작과 같은 시점. 첫 관측은 위상 0 을 본다.
          gait_clock_.restart();
          set_mode(Mode::POLICY_RUNNING, "enable_policy=true and entry checks passed");
        }
      }
      break;

    case Mode::POLICY_RUNNING: {
        if (!policy_requested) {
          set_mode(Mode::SAFE_HOLD, "enable_policy=false");
          approach(default_positions_, dt);
          break;
        }
        // 위상 시계는 정책이 도는지와 상관없이 POLICY_RUNNING 인 매 틱 실제 시간을
        // 따라간다(낙상 debounce 로 추론을 건너뛰는 틱에도 몸의 시간은 흐른다).
        // 진입 후 첫 틱은 더하지 않아 첫 관측이 위상 0 이 된다 -- GaitClock 참고.
        gait_clock_.advance(dt);
        if (!imu_fresh) {
          enter_fault("IMU watchdog expired");
          break;
        }
        if (imu.projected_gravity[2] > fall_gravity_z_threshold_) {
          // Freeze the targets while the fall condition is debounced: a single
          // noisy sample must not stop a walk, a sustained one must.
          fall_timer_s_ += dt;
          if (fall_timer_s_ >= fall_debounce_s_) {enter_fault("fall detected");}
          break;
        }
        fall_timer_s_ = 0.0;
        if (!command_fresh) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "gamepad command is stale; commanding zero velocity");
        }

        policy_accumulator_s_ += dt;
        const double policy_period = 1.0 / policy_rate_hz_;
        // 반 틱의 여유를 둔다(2026-09-27). 예전 조건(accumulator >= period, 이후 fmod)은
        // 제어 주기와 정책 주기가 같을 때(50 Hz / 50 Hz), 주기가 19.9 ms 처럼 조금만 짧게
        // 잡혀도 그 틱의 추론을 건너뛰었다. mock 종단 테스트에서 16.8 %, 실기 22:06 에서
        // 2.2 % 의 틱이 그렇게 빠졌고, 그 틱에는 action 이 40 ms 유지되며 다음 관측의
        // last_action·gait_phase 간격이 학습(매 스텝 추론)과 달라졌다.
        // 지금 조건은 흔들림이 반 틱 안이면 50/50 에서는 매 틱, 200/50 처럼 정수배면 정확히
        // N 틱마다 돈다. 크게 밀린 뒤에는 한 번만 돌고 나머지는 버린다(예전과 같음).
        if (policy_accumulator_s_ >= policy_period - 0.5 * dt) {
          policy_accumulator_s_ =
            std::clamp(policy_accumulator_s_ - policy_period, -0.5 * policy_period,
            0.5 * policy_period);
          build_observation(command, imu);
          if (!run_policy()) {
            enter_fault("policy inference failed", false);
            break;
          }
          if (!is_finite_vector(actions_)) {
            enter_fault("policy produced a non-finite action", false);
            break;
          }
          update_policy_setpoint();
        }
        break;
      }

    case Mode::FAULT:
      // Targets stay frozen at the pose captured when the fault latched. If the
      // cause can pass and it has stayed clear for fault_recovery_s_, hand
      // control back to SAFE_HOLD rather than making an operator power-cycle
      // the controller for a dropped packet.
      //
      // Reaching this point already proves read_joint_state() succeeded. Both
      // remaining causes are read through the IMU, so when one is fitted the
      // recovery gate is "IMU talking again and the robot upright" -- that also
      // stops a fallen robot from posing itself while lying on its side.
      // POLICY_RUNNING is not re-entered from here: SAFE_HOLD eases back to the
      // default pose under the rate limit, and ready_to_start_policy() decides
      // when the policy resumes, exactly as it does after a fresh activation.
      if (fault_recoverable_ && fault_recovery_s_ >= 0.0) {
        const bool clear = !imu_enabled_ ||
          (imu_fresh && imu.projected_gravity[2] <= fall_gravity_z_threshold_);
        fault_clear_s_ = clear ? fault_clear_s_ + dt : 0.0;
        if (fault_clear_s_ >= fault_recovery_s_) {
          // Re-anchor: the joints may have moved while the fault was latched,
          // so the frozen target is stale and resuming from it would step.
          hold_measured_pose();
          fault_clear_s_ = 0.0;
          fall_timer_s_ = 0.0;
          policy_accumulator_s_ = 0.0;
          set_mode(Mode::SAFE_HOLD, "fault condition cleared");
        }
      }
      break;  
  }

  // 모드 기계가 끝난 뒤에 기록한다. 그래야 이번 틱에 실제로 나간 target 과
  // 그때의 모드가 같은 줄에 남는다.
  log_tick(dt, command, imu, command_fresh, imu_fresh);

  return write_commands() ?
         controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

}  // namespace kid_rl

PLUGINLIB_EXPORT_CLASS(kid_rl::KidRlController, controller_interface::ControllerInterface)
