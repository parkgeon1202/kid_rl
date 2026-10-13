// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__LOWPASS_HPP_
#define KID_RL__LOWPASS_HPP_

#include <array>
#include <cmath>
#include <cstddef>

namespace kid_rl
{

/// 1차 저역통과 필터 (3축). 학습 쪽 mjlab_kid_RL tasks/mdp.py 의 gyro 필터와 같은 식이다:
///
///   alpha = dt / (dt + 1 / (2 * pi * cutoff_hz))
///   y[k]  = y[k-1] + alpha * (x[k] - y[k-1])
///
/// 과거 샘플의 지수 가중 평균이라 출력의 실질 나이는 느린 신호 기준 약 1 / (2 pi fc) 다
/// (6 Hz 에서 약 27 ms). 첫 샘플은 그대로 통과시켜 상태를 초기화한다.
/// cutoff_hz <= 0 이면 꺼져서 입력을 그대로 돌려준다.
class LowPass3
{
public:
  void configure(double cutoff_hz) noexcept
  {
    cutoff_hz_ = cutoff_hz;
    reset();
  }

  bool enabled() const noexcept {return cutoff_hz_ > 0.0;}

  void reset() noexcept {initialized_ = false;}

  static double alpha(double dt_s, double cutoff_hz) noexcept
  {
    constexpr double kTwoPi = 6.283185307179586;
    return dt_s / (dt_s + 1.0 / (kTwoPi * cutoff_hz));
  }

  std::array<double, 3> update(const std::array<double, 3> & x, double dt_s) noexcept
  {
    if (!enabled()) {return x;}
    if (!initialized_ || !(dt_s > 0.0)) {
      if (!initialized_) {
        y_ = x;
        initialized_ = true;
      }
      return y_;
    }
    const double a = alpha(dt_s, cutoff_hz_);
    for (std::size_t i = 0; i < 3; ++i) {
      y_[i] += a * (x[i] - y_[i]);
    }
    return y_;
  }

private:
  double cutoff_hz_{0.0};
  bool initialized_{false};
  std::array<double, 3> y_{};
};

}  // namespace kid_rl

#endif  // KID_RL__LOWPASS_HPP_
