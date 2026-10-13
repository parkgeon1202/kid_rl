// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__KID_RL_CONTROLLER_HPP_
#define KID_RL__KID_RL_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kid_rl/csv_logger.hpp"
#include "kid_rl/gamepad.hpp"
#include "kid_rl/imu.hpp"
#include "kid_rl/input_types.hpp"
#include "kid_rl/policy_runner.hpp"
#include "kid_rl/motor_telemetry_monitor.hpp"
#include "kid_rl/rx_missing_monitor.hpp"
#include "kid_rl/bus_timing_monitor.hpp"
#include "kid_rl/gait_phase.hpp"
#include "kid_rl/lowpass.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/bool.hpp"

namespace kid_rl
{

/// Position controller that drives a locomotion RL policy inside ros2_control.
///
/// The controller runs at the controller_manager update rate (target: 200 Hz,
/// the same loop the DYNAMIXEL bus is read and written on) and decimates the
/// policy inference down to `policy_rate_hz` (target: 50 Hz). Two rates coexist:
///
///   * policy tick (50 Hz) -> writes `targets_` from the policy output
///   * every tick (200 Hz) -> reads the joints, runs the watchdogs, writes the
///                            standing target to hardware
///
/// Between policy ticks the target is held, not smoothed or interpolated
/// (zero-order hold): training fed the policy target straight to the joints --
/// no filter, no rate limit, latency modelled as a 15-30 ms pure delay in the
/// actuator -- so shaping it here would command a trajectory the policy never
/// produced. Smooth motion comes from the joint-level PD in the servo firmware.
/// The fast rate earns its keep on the read path instead: fresher joint
/// velocities for the observation, and watchdogs, fall detection and the
/// overrun guard reacting in 5 ms instead of 20 ms. Do NOT set this
/// controller's own `update_rate` parameter to 50: that decimates `update()`
/// itself and throws the fast path away.
///
/// `target_rate_limit` still bounds SAFE_HOLD and READY_POSE, where the
/// controller -- not the policy -- generates the motion.
///
/// The ONNX policy is loaded and validated during configure. Inference reuses
/// preallocated input/output tensors on the 50 Hz policy ticks.
///
/// Observation layout (must match the trained policy exactly):
///   [0..2]              body angular velocity (gyro), body frame, rad/s
///   [3..5]              projected gravity, body frame, unit vector
///   [6      .. 6+N-1]   joint position - default position, rad
///   [6+N    .. 6+2N-1]  joint velocity, rad/s
///   [6+2N   .. 6+3N-1]  previous raw action
///   [6+3N   .. 8+3N]    velocity command vx, vy, yaw_rate
class KidRlController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  /// Internal operating mode, independent of the lifecycle state.
  enum class Mode
  {
    WAITING_FOR_STATE,  ///< configured/activated but no valid joint state yet
    READY_POSE,         ///< interpolating from the activation pose to the default pose
    SAFE_HOLD,          ///< rate-limited hold at the default pose, policy disabled
    POLICY_RUNNING,     ///< policy owns the joint targets
    FAULT               ///< targets frozen; config decides whether recovery is latched
  };

  // Gamepad and IMU use direct device I/O, not ROS topics.
  void policy_enable_callback(std_msgs::msg::Bool::SharedPtr msg);

  // Real-time helpers.
  bool map_interfaces();
  bool read_joint_state();
  bool hardware_state_ok();
  void set_mode(Mode mode, const char * reason);
  void hold_measured_pose();
  void approach(const std::vector<double> & goal, double dt);
  bool ready_to_start_policy(const ImuSample & imu) const;
  void build_observation(const CommandSample & command, const ImuSample & imu);
  bool run_policy();
  void update_policy_setpoint();
  void enter_fault(const char * reason, bool recoverable = true);
  bool write_commands();
  static const char * mode_name(Mode mode);

  // Configuration.
  std::vector<std::string> joints_;
  std::vector<double> default_positions_, joint_position_min_, joint_position_max_;
  double policy_rate_hz_{50.0};
  double ready_pose_duration_s_{2.0};
  double command_timeout_s_{0.25}, imu_timeout_s_{0.10};
  double action_scale_{1.0};
  /// 학습의 agent.clip_actions 와 같은 값. 0 이면 끈다.
  double action_clip_{5.0};
  /// 학습의 _GAIT_SWING_TIME, _GAIT_DOUBLE_SUPPORT_BAND 와 같은 값. 주기는 여기서 계산한다.
  double gait_swing_time_s_{0.0};
  double gait_double_support_band_{0.1};
  /// 위 두 값으로 계산한 한 주기 [s]. 모델이 gait_phase 를 쓸 때만 쓰인다(uses_gait_phase_).
  double gait_cycle_time_s_{0.0};
  bool uses_gait_phase_{false};
  GaitClock gait_clock_;
  bool action_clip_warned_{false};
  /// gyro 관측 1차 저역통과 컷오프 [Hz]. 0 이면 끈다. 학습의 gyro 필터와 같은 값이어야 한다.
  double imu_gyro_filter_hz_{0.0};
  LowPass3 gyro_filter_;
  std::array<double, 3> imu_gyro_raw_{};  ///< 필터 전 gyro (로그용)
  /// Slew limit [rad/s] for the controller-generated moves (READY_POSE's tail
  /// and SAFE_HOLD's return to the default pose). The policy path is not rate
  /// limited -- see the class comment.
  double target_rate_limit_{4.0};
  double fall_gravity_z_threshold_{-0.5}, fall_debounce_s_{0.10};
  /// How long every fault condition must stay clear before FAULT hands control
  /// back to SAFE_HOLD. Negative latches the fault until an operator cycles the
  /// controller, which is the older behaviour.
  double fault_recovery_s_{1.0};
  double entry_position_tolerance_{0.35};
  /// 진입 자세 검사에서 빼는 관절. 팔은 MX-28 kp 가 낮아 기본 자세에 못 미친다.
  std::vector<std::string> entry_check_skip_joints_;
  std::vector<bool> entry_check_skip_;
  /// 정책 출력과 무관하게 목표를 default_positions 로 고정하는 관절. 학습의
  /// joint_pos_action.clip (예: torso_yaw (0, 0)) 과 같은 관절이어야 한다.
  std::vector<std::string> fixed_target_joints_;
  std::vector<bool> fixed_target_;
  /// POLICY_RUNNING 진입 후 첫 추론인지. 첫 틱은 변화율 검사에서 제외한다.
  bool policy_first_setpoint_{true};
  /// 정책이 만든 목표의 변화율 상한 [rad/s]. 넘으면 폴트로 멈춘다.
  /// 명령을 자르는 게 아니라 발산을 감지하는 용도다. 0이면 검사하지 않는다.
  double policy_slew_limit_{15.0};
  /// 변화율 상한을 연속으로 몇 번 넘겨야 폴트로 보는지. 기동 과도현상(두세 틱)과
  /// 진짜 발산(계속 이어짐)을 가르는 값이다.
  int policy_slew_debounce_ticks_{3};
  int policy_slew_strikes_{0};
  bool dry_run_policy_{false};
  std::string policy_path_;
  bool gamepad_enabled_{true};
  Gamepad::Config gamepad_config_;
  bool imu_enabled_{false};
  Imu::Config imu_config_;

  // CSV 기록. 실제 파일 쓰기는 CsvLogger 의 별도 스레드가 한다.
  bool log_enabled_{true};
  std::string log_path_;
  int log_decimation_{1};
  int log_buffer_samples_{4000};
  CsvLogger logger_;
  CsvLogger::Sample log_sample_;
  std::uint64_t log_tick_{0};
  double log_time_s_{0.0};
  void log_tick(double dt, const CommandSample & command, const ImuSample & imu,
    bool command_fresh, bool imu_fresh);

  // Interface index maps, resolved by name in on_activate().
  std::vector<std::size_t> position_state_index_, velocity_state_index_;
  std::vector<std::size_t> hardware_state_index_, position_command_index_;
  /// xacro track_rx=true 일 때만 켜지는 진단: CSV 의 rx_missing_count/rx_missing_ids.
  RxMissingMonitor rx_missing_monitor_;
  BusTimingMonitor bus_timing_monitor_;
  /// xacro 가 Present Input Voltage / Present PWM 을 선언한 관절만 켜지는 진단: CSV 의 vin_/duty_.
  MotorTelemetryMonitor motor_telemetry_monitor_;

  // Runtime state.
  std::vector<double> positions_, velocities_, activation_positions_;
  /// Staging buffers make a joint-state read transactional: positions_ and
  /// velocities_ change only after every requested interface returned a
  /// finite value in the same controller tick.
  std::vector<double> next_positions_, next_velocities_;
  /// Joint position command written to hardware every tick. Set by the policy
  /// on its own tick and held in between; SAFE_HOLD/READY_POSE drive it too.
  std::vector<double> targets_;
  std::vector<double> actions_, last_actions_, observation_;
  double policy_accumulator_s_{0.0}, ready_pose_elapsed_s_{0.0}, fall_timer_s_{0.0};
  /// Time the fault conditions have been continuously clear, and whether the
  /// latched fault is the kind that may clear at all (a policy that emitted a
  /// non-finite action will just do it again, so that one stays latched).
  double fault_clear_s_{0.0};
  bool fault_recoverable_{true};
  Mode mode_{Mode::WAITING_FOR_STATE};

  Gamepad gamepad_;
  Imu imu_;
  PolicyRunner policy_runner_;
  std::atomic<bool> policy_enabled_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr policy_enable_subscription_;
};

}  // namespace kid_rl

#endif  // KID_RL__KID_RL_CONTROLLER_HPP_
