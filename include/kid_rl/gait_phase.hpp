// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__GAIT_PHASE_HPP_
#define KID_RL__GAIT_PHASE_HPP_

#include <cmath>

namespace kid_rl
{

/// 학습과 같은 방식으로 주기를 계산한다 (mjlab_kid_RL kid_RL_env_cfg.py):
///   _GAIT_SWING_FRACTION = 0.5 - asin(_GAIT_DOUBLE_SUPPORT_BAND) / pi
///   _GAIT_CYCLE_TIME     = _GAIT_SWING_TIME / _GAIT_SWING_FRACTION
/// 한 주기는 두 걸음이고, 한 발이 떠 있는 비율이 swing fraction 이다.
inline double gait_cycle_time(double swing_time_s, double double_support_band)
{
  constexpr double kPi = 3.141592653589793;
  return swing_time_s / (0.5 - std::asin(double_support_band) / kPi);
}

/// 보행 위상 관측. mjlab_kid_RL tasks/mdp.py 의 gait_phase 와 같은 식이다:
///
///   phase = (episode_length_buf * step_dt / cycle_time) mod 1
///   observation = [sin(2*pi*phase), cos(2*pi*phase)]
///
/// 학습에서 episode_length_buf * step_dt 는 '에피소드가 시작된 뒤 흐른 시뮬레이션
/// 시간'이다. 리셋 직후 관측은 episode_length_buf = 0 이라 (sin, cos) = (0, 1) 이고,
/// k 번째 action 을 정하는 관측의 위상은 k * step_dt / cycle_time 이다
/// (test/data/gait_phase_mjlab.csv 가 실제 env 에서 기록한 값).
/// 실기에서는 elapsed_s 로 '정책이 처음 돈 뒤 흐른 실제 시간'을 넘긴다 (GaitClock).
inline double gait_phase(double elapsed_s, double cycle_time_s)
{
  double phase = std::fmod(elapsed_s / cycle_time_s, 1.0);
  if (phase < 0.0) {phase += 1.0;}   // torch.remainder 와 같게 [0, 1)
  return phase;
}

inline void gait_phase_observation(
  double elapsed_s, double cycle_time_s, double & sin_out, double & cos_out)
{
  constexpr double kTwoPi = 6.283185307179586;
  const double angle = kTwoPi * gait_phase(elapsed_s, cycle_time_s);
  sin_out = std::sin(angle);
  cos_out = std::cos(angle);
}

/// 정책 한 번의 '에피소드' 동안의 위상 시계.
///
/// 학습의 에피소드 시작 = 실기에서 POLICY_RUNNING 에 들어가는 순간이다. 두 곳 모두
/// 그 시점의 첫 관측은 이전 action 0 과 함께 위상 0 을 본다.
///
/// 사용법 (컨트롤러 update 안):
///   - POLICY_RUNNING 에 들어갈 때 restart()
///   - POLICY_RUNNING 인 매 제어 틱 처음에 advance(dt)  (dt = 이번 틱의 실측 주기)
///   - 관측을 만들 때 elapsed_s()
///
/// 진입 후 첫 advance() 는 시간을 더하지 않는다. 그 틱이 에피소드의 k = 0 관측이기
/// 때문이다. 이후 틱마다 실측 주기를 더하므로 elapsed_s() 는 '첫 관측 이후 흐른
/// 실제 시간'이 된다. 주기가 정확히 20 ms 면 학습의 k * 0.02 와 같고, 주기가
/// 흔들리거나 사이클을 건너뛰면 실제 시간을 따라간다 -- 로봇의 몸은 실제 시간으로
/// 움직이므로 위상도 그쪽에 맞춘다.
class GaitClock
{
public:
  void restart() noexcept
  {
    elapsed_s_ = 0.0;
    started_ = false;
  }

  void advance(double dt_s) noexcept
  {
    if (!started_) {
      started_ = true;
      return;
    }
    elapsed_s_ += dt_s;
  }

  double elapsed_s() const noexcept {return elapsed_s_;}

private:
  double elapsed_s_{0.0};
  bool started_{false};
};

}  // namespace kid_rl

#endif  // KID_RL__GAIT_PHASE_HPP_
