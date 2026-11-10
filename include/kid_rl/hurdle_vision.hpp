// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__HURDLE_VISION_HPP_
#define KID_RL__HURDLE_VISION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace kid_rl
{

/// hurdle_vision 관측. mjlab_kid_RL tasks/hurdle.py 의 hurdle_vision 과 같은 모양이다:
///   한 프레임 = (visible, x, y, z), /hurdle/pose 의 base_link 좌표 [m]. 못 보면 전부 0.
///   정책 스텝(50 Hz)마다 한 프레임을 넣고, VISION_STRIDE(5 스텝 = 0.1 s) 간격으로
///   VISION_FRAMES(11) 개를 오래된 것부터 이어 44 칸을 만든다(최신과 1.0 s 전까지).
/// 카메라 tilt 는 여기서 다루지 않는다. task planner 가 450 mm 규칙으로 따로 바꾼다.
constexpr std::size_t kHurdleVisionFrames = 11;
constexpr std::size_t kHurdleVisionStride = 5;
constexpr std::size_t kHurdleVisionFrameDim = 4;
constexpr std::size_t kHurdleVisionSize = kHurdleVisionFrames * kHurdleVisionFrameDim;
/// 학습 스텝 주기 [Hz]. kHurdleVisionStride 는 이 주기 기준 스텝 수다.
constexpr double kHurdleVisionStepRateHz = 50.0;

/// /hurdle/pose 콜백이 RealtimeBuffer 로 넘기는 값. stamp 는 받은 순간의 정상 시계.
struct HurdlePoseSample
{
  std::array<double, 3> position{};  // x, y, z [m], base_link
  std::int64_t stamp_ns{0};
  /// 직전 메시지와의 수신 간격 [ms]. 두 번째 메시지부터 채워지고, 그전에는 -1.
  double period_ms{-1.0};
  bool valid{false};
};

/// 최근 (kHurdleVisionFrames - 1) * kHurdleVisionStride + 1 스텝의 프레임을 들고 있는 링 버퍼.
/// 고정 크기라 실시간 스레드에서 할당 없이 쓴다.
class HurdleVisionHistory
{
public:
  using Frame = std::array<double, kHurdleVisionFrameDim>;

  /// 학습의 에피소드 리셋과 같다: 모든 프레임을 0(못 봄)으로.
  void reset()
  {
    frames_.fill(Frame{});
    newest_ = 0;
  }

  /// 정책 스텝마다 한 번 부른다.
  void push(const Frame & frame)
  {
    newest_ = (newest_ + 1) % kDepth;
    frames_[newest_] = frame;
  }

  /// kHurdleVisionSize 칸을 오래된 프레임부터 out 에 쓴다.
  void write(double * out) const
  {
    for (std::size_t i = 0; i < kHurdleVisionFrames; ++i) {
      const std::size_t steps_back = (kHurdleVisionFrames - 1 - i) * kHurdleVisionStride;
      const Frame & frame = frames_[(newest_ + kDepth - steps_back) % kDepth];
      for (std::size_t d = 0; d < kHurdleVisionFrameDim; ++d) {
        out[i * kHurdleVisionFrameDim + d] = frame[d];
      }
    }
  }

private:
  static constexpr std::size_t kDepth = (kHurdleVisionFrames - 1) * kHurdleVisionStride + 1;
  std::array<Frame, kDepth> frames_{};
  std::size_t newest_{0};
};

}  // namespace kid_rl

#endif  // KID_RL__HURDLE_VISION_HPP_
