// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0
//
// hurdle_vision 링 버퍼가 mjlab_kid_RL tasks/hurdle.py 의 hurdle_vision 과 같은 순서로 44 칸을
// 만드는지 검사한다. 학습 코드는 history[0] 이 최신이고 offsets = [50, 45, ..., 0] 스텝 전을
// 골라 이어 붙인다(지연 0 기준): 오래된 것부터, 5 스텝(0.1 s) 간격, 11 프레임.

#include <cstdlib>
#include <iostream>

#include "kid_rl/hurdle_vision.hpp"

namespace
{

int failures = 0;

void expect(bool ok, const char * what)
{
  if (!ok) {
    std::cerr << "FAIL: " << what << '\n';
    ++failures;
  }
}

}  // namespace

int main()
{
  using kid_rl::HurdleVisionHistory;
  using kid_rl::kHurdleVisionFrameDim;
  using kid_rl::kHurdleVisionFrames;
  using kid_rl::kHurdleVisionSize;
  using kid_rl::kHurdleVisionStride;

  expect(kHurdleVisionSize == 44, "44 values");

  HurdleVisionHistory history;
  history.reset();
  double out[kHurdleVisionSize];

  // 리셋 직후는 전부 0(못 봄).
  history.write(out);
  for (double v : out) {expect(v == 0.0, "reset is all zero");}

  // 스텝 k 에 (1, k, -k, 0.01k) 를 넣는다. 70 스텝이면 버퍼(51)를 한 바퀴 넘게 돈다.
  constexpr int kSteps = 70;
  for (int k = 1; k <= kSteps; ++k) {
    history.push({1.0, static_cast<double>(k), -static_cast<double>(k), 0.01 * k});
  }
  history.write(out);
  for (std::size_t i = 0; i < kHurdleVisionFrames; ++i) {
    const int expected_step =
      kSteps - static_cast<int>((kHurdleVisionFrames - 1 - i) * kHurdleVisionStride);
    const double * frame = &out[i * kHurdleVisionFrameDim];
    expect(frame[0] == 1.0, "visible flag");
    expect(frame[1] == expected_step, "x is the step 0.1 s apart, oldest first");
    expect(frame[2] == -expected_step, "y follows x");
    expect(frame[3] == 0.01 * expected_step, "z follows x");
  }

  // 못 본 프레임 [0, 0, 0, 0] 이 최신 칸에 그대로 들어간다.
  history.push({});
  history.write(out);
  for (std::size_t d = 0; d < kHurdleVisionFrameDim; ++d) {
    expect(out[kHurdleVisionSize - kHurdleVisionFrameDim + d] == 0.0, "newest frame is missed");
  }

  // 아직 1.0 s 가 안 찼으면 오래된 칸은 0 이다.
  history.reset();
  history.push({1.0, 2.0, 3.0, 4.0});
  history.write(out);
  expect(out[0] == 0.0 && out[1] == 0.0, "oldest frame is zero before 1.0 s of history");
  expect(out[kHurdleVisionSize - 4] == 1.0 && out[kHurdleVisionSize - 3] == 2.0, "newest frame");

  if (failures != 0) {return EXIT_FAILURE;}
  std::cout << "hurdle_vision_test OK\n";
  return EXIT_SUCCESS;
}
