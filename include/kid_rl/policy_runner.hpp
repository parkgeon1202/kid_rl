// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__POLICY_RUNNER_HPP_
#define KID_RL__POLICY_RUNNER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace kid_rl
{

/// Owns the ONNX Runtime session and its preallocated input/output tensors.
/// Model creation and metadata validation happen outside the real-time loop in
/// load(); run() only converts values, executes the session, and copies output.
class PolicyRunner
{
public:
  PolicyRunner();
  ~PolicyRunner();

  PolicyRunner(const PolicyRunner &) = delete;
  PolicyRunner & operator=(const PolicyRunner &) = delete;

  /// 모델을 열고, 모델이 스스로 들고 있는 메타데이터와 설정값이 맞는지 검사한다.
  /// 관절 이름/순서뿐 아니라 default_joint_pos 와 action_scale 까지 대조한다.
  /// 이 셋 중 하나라도 어긋나면 정책은 "잘 돌면서 조용히 틀린 자세"를 명령하게
  /// 되므로, 기동 시점에 실패시키는 편이 낫다.
  ///
  /// 관측: 모델 메타데이터 observation_names 가 base_observation_names 와 같으면 위상 없는
  /// 모델(입력 크기 base_observation_size), 그 뒤에 "gait_phase" 가 하나 더 붙어 있으면
  /// 위상(sin, cos)을 쓰는 모델(입력 크기 base + 2)로 보고 uses_gait_phase 에 알린다.
  /// 맨 끝에 "hurdle_vision" 이 붙어 있으면 허들 관측(kHurdleVisionSize 칸)을 쓰는 모델로
  /// 보고 uses_hurdle_vision 에 알린다(gait_phase 가 있으면 그 뒤).
  /// 프레임을 쌓은 모델(mjlab ObservationTermCfg.history_length): 메타데이터에는 이름만 있으므로
  /// 입력 폭에서 거꾸로 구한다. 쌓이는 앞부분(stacked_observation_size 칸, base 의 앞쪽)이
  /// history_length 번 들어 있으면 그 수를 알린다. 안 쌓은 모델은 1.
  /// 그 밖의 순서·이름은 컨트롤러가 조립하는 관측과 다르므로 거부한다.
  bool load(
    const std::string & model_path,
    const std::vector<std::string> & base_observation_names,
    std::size_t base_observation_size,
    bool & uses_gait_phase,
    bool & uses_hurdle_vision,
    std::size_t stacked_observation_size,
    std::size_t & history_length,
    std::size_t expected_action_size,
    const std::vector<std::string> & expected_joint_names,
    const std::vector<double> & expected_default_positions,
    double expected_action_scale,
    std::string & error);

  void reset() noexcept;
  bool run(const std::vector<double> & observation, std::vector<double> & actions) noexcept;
  bool is_loaded() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace kid_rl

#endif  // KID_RL__POLICY_RUNNER_HPP_
