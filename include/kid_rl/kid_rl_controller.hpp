// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__KID_RL_CONTROLLER_HPP_
#define KID_RL__KID_RL_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kid_rl/csv_logger.hpp"
#include "kid_rl/external_commands.hpp"
#include "kid_rl/imu.hpp"
#include "kid_rl/input_types.hpp"
#include "kid_rl/policy_runner.hpp"
#include "kid_rl/motor_telemetry_monitor.hpp"
#include "kid_rl/rx_missing_monitor.hpp"
#include "kid_rl/bus_timing_monitor.hpp"
#include "kid_rl/gait_phase.hpp"
#include "kid_rl/hurdle_vision.hpp"
#include "kid_rl/lowpass.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
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

  // IMU uses direct device I/O, not ROS topics.
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
  // 자동 정지: walk_command.topic 이 (0, 0, 0) 이고 자이로 크기가 auto_hold.gyro_threshold
  // [rad/s] 미만으로 auto_hold.settle_s 동안 유지되면 enable_policy 를 false 로 내린다
  // (-> SAFE_HOLD, motion_operator 다리 명령을 받음). 그 뒤 walk_command.topic 에 0 이 아닌
  // 속도가 오면 enable_policy 를 다시 true 로 올린다. 사람이 ~/enable_policy 로 직접 바꾸면
  // auto_disabled_ 가 지워져서, 사람이 끈 정책은 토픽 명령으로 켜지지 않는다.
  bool auto_hold_enabled_{true};
  double auto_hold_gyro_threshold_{0.1}, auto_hold_settle_s_{0.5};
  double auto_hold_timer_s_{0.0};
  // 비실시간 enable_policy 콜백이 지우고 update() 가 세우고 읽는다.
  std::atomic<bool> auto_disabled_{false};
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
  /// 모델 메타데이터 observation_names 끝에 hurdle_vision 이 있으면 참. 관측 끝에 44 칸이 붙는다.
  bool uses_hurdle_vision_{false};
  /// 정책 스텝마다 이번 /hurdle/pose 프레임을 넣는다. POLICY_RUNNING 진입(= 학습의 리셋) 때 비운다.
  HurdleVisionHistory hurdle_vision_;
  /// 쌓아 넣는 관측(base_ang_vel, projected_gravity, joint_pos, joint_vel, actions)의 프레임 수.
  /// 모델 입력 폭에서 구한다(허들 태스크 5, 평지 1). 프레임은 정책 스텝(20 ms)마다 하나다.
  std::size_t observation_history_{1};
  /// 한 프레임 = 위 다섯 항목 (6 + 3n 칸). observation_history_ 개를 오래된 것부터 둔다.
  std::vector<double> stacked_frames_;
  /// 참이면 다음 프레임으로 모든 칸을 채운다(mjlab CircularBuffer 가 리셋 뒤 첫 값으로 채우는 것과 같다).
  bool stacked_frames_empty_{true};
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
  /// 정책과 무관한 관절(관측·action 에 없음). 활성화 때 측정 자세에서 hold_positions 까지
  /// hold_rate_limit 으로 천천히 옮긴 뒤 모든 모드에서 그 자리에 붙잡는다.
  /// IRC 로봇처럼 학습 모델에서는 고정된 상체(팔, 허리)를 실기에서 잡을 때 쓴다.
  std::vector<std::string> hold_joints_;
  std::vector<double> hold_positions_, hold_targets_;
  std::vector<std::size_t> hold_position_state_index_, hold_command_index_;
  double hold_rate_limit_{0.5};
  /// 정책이 요청되면(enable_policy=true) 외부 명령을 놓고 hold_positions 로 돌아가는 고정 관절.
  /// 팔·허리처럼 정책이 학습한 자세가 있는 관절이다. 그리퍼(공을 쥔 채 걷는다)와
  /// tilt(pan_tilt 전용)는 false 라 계속 외부 명령을 따른다.
  std::vector<bool> hold_release_;
  double last_dt_s_{0.0};
  bool read_hold_positions(std::vector<double> & out);
  bool write_hold_commands();
  /// 고정 관절의 이번 틱 측정값 (/motor_status 용). 읽기 실패 시 마지막 값을 유지한다.
  std::vector<double> hold_measured_, hold_velocities_;
  std::vector<std::size_t> hold_velocity_state_index_;
  void read_hold_state();

  /// 외부 노드(motion_operator, humanoid_manipulation, 비전 pan_tilt)의 모터 명령.
  /// 슬롯 k: k < joints_.size() 이면 정책 관절, 아니면 hold_joints_[k - joints_.size()].
  ExternalCommands external_;
  ExternalCommands::Config external_config_;
  bool external_enabled_{false};
  /// 정책 관절(다리)에 온 외부 명령을 받을지. 받더라도 SAFE_HOLD 이고 enable_policy=false
  /// 일 때만 따르고, 정책이 요청되는 순간 놓아서 다리가 default 자세로 돌아간다.
  bool external_accept_policy_joints_{true};
  /// 슬롯별: 마지막으로 처리한 seq, 따르는 중인지, 목표, 프로파일, 현재 프로파일 속도.
  std::vector<std::uint64_t> external_applied_seq_;
  std::vector<bool> external_latched_;
  /// tilt 모터의 hold_joints 인덱스(-1 = 없음). 이 관절은 hold_positions 로 가지 않고
  /// pan_tilt 명령만 따른다(명령 전에는 활성화 때 측정 자세를 유지).
  int tilt_hold_index_{-1};

  // 모션 P gain (motion_gain.*): SAFE_HOLD 이고 enable_policy=false 면 motion_gain.p_gains 를,
  // 그 밖의 모드에서는 robot_description 의 <관절>_dxl <param name="Position P Gain"> (학습 값)을
  // <관절>_dxl/Position P Gain command 에 쓴다. 순서는 정책 관절 다음 고정 관절.
  // 그 command_interface 가 robot_description 에 하나도 없으면(예: mock) 스스로 끈다.
  static constexpr char kGainInterface[] = "Position P Gain";
  bool configure_gains();
  bool write_gain_commands();
  bool motion_gain_enabled_{true};
  bool gain_interfaces_{false};
  std::vector<double> motion_gains_, trained_gains_;
  std::vector<std::size_t> gain_command_index_;
  /// 학습 gain 을 연속으로 쓴 틱 수. POLICY_RUNNING 진입은 이 값이 2 이상이어야 한다
  /// (이번 틱에 쓴 값은 이 틱의 write() 에서야 서보로 나간다).
  int trained_gain_ticks_{0};
  std::vector<double> external_goal_, external_velocity_, external_acceleration_;
  std::vector<double> external_speed_;
  /// 새 외부 명령을 받아 슬롯에 걸어 둔다. 모드에 따라 받지 않을 명령은 버린다.
  void process_external_commands(bool policy_requested);
  /// Dynamixel 프로파일(속도·가속 상한, 0 = 제한 없음)을 흉내 내 target 을 goal 쪽으로 옮긴다.
  static double profile_step(
    double target, double goal, double velocity, double acceleration, double dt, double & speed);
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
    bool command_fresh, bool imu_fresh, const HurdlePoseSample & hurdle_pose, bool hurdle_fresh);

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

  Imu imu_;
  PolicyRunner policy_runner_;
  std::atomic<bool> policy_enabled_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr policy_enable_subscription_;

  // 보행 속도 명령(geometry_msgs/TwistStamped):
  // twist.linear.x / linear.y [m/s], twist.angular.z [rad/s]. 비실시간 콜백이
  // CommandSample(정상 시계 stamp)로 넣고 update() 가 읽는다.
  void walk_command_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg);
  std::string walk_command_topic_{"/cmd_vel"};
  /// 보행 명령 크기 상한 vx, vy [m/s], wz [rad/s]. 학습 범위 밖으로 나가지 않게 자른다.
  std::array<double, 3> walk_command_max_{0.4, 0.3, 0.7};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr walk_command_subscription_;
  realtime_tools::RealtimeBuffer<CommandSample> walk_command_buffer_;

  // 허들 자세: ExternalCommands 가 /hurdle/pose 를 받아 두고 update() 가 읽는다.
  // 지난 정책 스텝 뒤에 새로 받았으면 이번 프레임은 [1, x, y, z], 아니면 [0, 0, 0, 0].
  // 이미 버퍼에 넣은 위치의 수신 시각(steady). 이보다 늦게 받은 것만 새 프레임이다.
  std::int64_t hurdle_pose_consumed_ns_{0};

  // 보행 정지 상태(std_msgs/Bool, walk_state.topic): true = SAFE_HOLD 이고 enable_policy=false 라
  // 다리가 외부 명령(/dynamixel_control)을 따른다. task planner 는 이게 true 가 된 뒤에 모션이나
  // 매니를 시작한다. 값이 바뀌면 바로, 아니면 walk_state.rate_hz 로 다시 낸다.
  void publish_walk_state(double dt);
  std::string walk_state_topic_{"/walk_stopped"};
  double walk_state_rate_hz_{10.0};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr walk_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>
  walk_state_rt_publisher_;
  std_msgs::msg::Bool walk_state_msg_;
  double walk_state_accumulator_s_{0.0};
  int walk_state_last_{-1};  ///< 마지막으로 낸 값. -1 = 아직 안 냄.
};

}  // namespace kid_rl

#endif  // KID_RL__KID_RL_CONTROLLER_HPP_
