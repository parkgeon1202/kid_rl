// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0
//
// 보행 위상(gait_phase) 관측이 mjlab 학습 때와 '같은 시각에 같은 값'으로 들어가는지 검사한다.
//
// 기준 데이터 test/data/gait_phase_mjlab.csv 는 mjlab 학습 환경(Mjlab-Velocity-KidRL)을 실제로
// 띄워 env.reset()/env.step() 이 정책에 넘긴 actor 관측의 gait_phase 칸을 그대로 기록한 것이다
// (test/gen_gait_phase_golden.py). 공식을 다시 쓴 게 아니라 학습 코드의 출력이다.
//
// 사용법: gait_phase_test GOLDEN_CSV

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "kid_rl/gait_phase.hpp"

namespace
{

struct Row
{
  int k{0};
  double sin{0.0};
  double cos{0.0};
  std::string note;
};

int g_failures = 0;

void check(bool ok, const std::string & what)
{
  if (!ok) {
    ++g_failures;
    std::cerr << "FAIL: " << what << '\n';
  }
}

bool load_golden(const std::string & path, std::vector<Row> & rows, double & cycle, double & dt)
{
  std::ifstream in(path);
  if (!in) {return false;}
  std::string line;
  bool header_seen = false;
  while (std::getline(in, line)) {
    if (line.rfind("#", 0) == 0) {
      // "# task=... cycle_time=0.85... step_dt=0.02"
      const auto c = line.find("cycle_time=");
      const auto d = line.find("step_dt=");
      if (c != std::string::npos) {cycle = std::stod(line.substr(c + 11));}
      if (d != std::string::npos) {dt = std::stod(line.substr(d + 8));}
      continue;
    }
    if (!header_seen) {header_seen = true; continue;}
    std::stringstream ss(line);
    std::string k, buf, s, co, note;
    std::getline(ss, k, ',');
    std::getline(ss, buf, ',');
    std::getline(ss, s, ',');
    std::getline(ss, co, ',');
    std::getline(ss, note, ',');
    Row r;
    r.k = std::stoi(k);
    r.sin = std::stod(s);
    r.cos = std::stod(co);
    r.note = note;
    rows.push_back(r);
  }
  return !rows.empty() && cycle > 0.0 && dt > 0.0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: gait_phase_test GOLDEN_CSV\n";
    return 2;
  }
  std::vector<Row> rows;
  double cycle = 0.0, step_dt = 0.0;
  if (!load_golden(argv[1], rows, cycle, step_dt)) {
    std::cerr << "could not read golden data " << argv[1] << '\n';
    return 2;
  }
  // 학습 설정과 컨트롤러 설정(config/controllers.yaml gait_swing_time_s, gait_double_support_band)이 같은지.
  // controllers.yaml 의 gait_swing_time_s = 0.4, gait_double_support_band = 0.1 로 컨트롤러가 계산하는
  // 주기가 mjlab 이 계산해 env 에 넣은 cycle_time 과 같아야 한다.
  check(std::abs(kid_rl::gait_cycle_time(0.4, 0.1) - cycle) < 1e-12,
    "gait_cycle_time(0.4, 0.1) differs from the cycle_time mjlab used");
  check(std::abs(step_dt - 0.02) < 1e-12, "golden step_dt is not 0.02 (50 Hz policy)");

  // 1. 식: k 번째 관측 = gait_phase_observation(k * step_dt). mjlab 은 float32 라 1e-5 허용.
  double worst = 0.0;
  for (const auto & r : rows) {
    double s = 0.0, c = 0.0;
    kid_rl::gait_phase_observation(r.k * step_dt, cycle, s, c);
    worst = std::max(worst, std::max(std::abs(s - r.sin), std::abs(c - r.cos)));
  }
  check(worst < 1e-5, "formula differs from mjlab env output (worst " + std::to_string(worst) + ")");

  // 2. 컨트롤러가 쓰는 순서 그대로 재현: 에피소드 시작(리셋)에서 restart(), 매 틱 처음에
  //    advance(dt), 그 틱의 관측은 elapsed_s(). 학습 env 의 리셋 지점까지 포함해 기준 데이터
  //    전 구간(리셋 2번, 한 에피소드 최대 300 스텝 = 약 7 주기)과 한 줄씩 맞아야 한다.
  kid_rl::GaitClock clock;
  double worst_seq = 0.0;
  int resets = 0;
  for (const auto & r : rows) {
    if (r.note.find("reset") != std::string::npos) {
      clock.restart();
      ++resets;
      check(r.k == 0, "golden reset row does not have k = 0");
    }
    clock.advance(step_dt);
    double s = 0.0, c = 0.0;
    kid_rl::gait_phase_observation(clock.elapsed_s(), cycle, s, c);
    worst_seq = std::max(worst_seq, std::max(std::abs(s - r.sin), std::abs(c - r.cos)));
    if (r.k == 0) {
      check(clock.elapsed_s() == 0.0, "first observation of an episode is not at phase 0");
    }
  }
  check(resets >= 2, "golden data should contain the initial and a forced reset");
  check(worst_seq < 1e-5,
    "controller clock sequence differs from mjlab env (worst " + std::to_string(worst_seq) + ")");

  // 3. 첫 관측은 정확히 (0, 1): 학습의 리셋 직후 관측과 같다.
  {
    kid_rl::GaitClock c;
    c.restart();
    c.advance(0.0213);  // 진입 틱의 주기가 얼마든 첫 관측은 0
    double s = 1.0, co = 0.0;
    kid_rl::gait_phase_observation(c.elapsed_s(), cycle, s, co);
    check(s == 0.0 && co == 1.0, "first observation after restart is not (0, 1)");
  }

  // 4. 주기 흔들림: 위상은 실제 경과 시간을 따라간다.
  //    합성 dt 시퀀스 -- 2026-09-23~24 실기 CSV 의 분포를 흉내 냈다(실측 파일이 아님):
  //    대부분 20 ms 근처, 읽기 실패 후 14~16 ms 따라잡기, 칸 건너뛰기 40 ms, 드물게 60 ms.
  {
    std::vector<double> dts;
    std::srand(7);
    for (int i = 0; i < 20000; ++i) {
      const int r = std::rand() % 1000;
      double dt = 0.0200 + ((std::rand() % 1001) - 500) * 1e-6;  // ±0.5 ms
      if (r < 12) {dt = 0.0142 + (std::rand() % 20) * 1e-4;}      // 1.2 %: 14~16 ms
      else if (r < 16) {dt = 0.040;}                              // 0.4 %: 40 ms
      else if (r < 17) {dt = 0.060;}                              // 0.1 %: 60 ms
      dts.push_back(dt);
    }
    kid_rl::GaitClock c;
    c.restart();
    double expected = 0.0;
    double worst_t = 0.0;
    for (std::size_t i = 0; i < dts.size(); ++i) {
      c.advance(dts[i]);
      if (i > 0) {expected += dts[i];}  // 진입 틱의 dt 는 에피소드 시간에 들어가지 않는다
      worst_t = std::max(worst_t, std::abs(c.elapsed_s() - expected));
      const double p = kid_rl::gait_phase(c.elapsed_s(), cycle);
      check(p >= 0.0 && p < 1.0, "phase left [0, 1)");
    }
    check(worst_t < 1e-9, "clock does not follow real elapsed time");
  }

  // 5. 긴 실행: 10 분(30,000 틱)을 20 ms 씩 더해도 k * 0.02 로 계산한 값과 같아야 한다.
  {
    kid_rl::GaitClock c;
    c.restart();
    double worst_long = 0.0;
    for (int k = 0; k <= 30000; ++k) {
      c.advance(step_dt);
      const double a = kid_rl::gait_phase(c.elapsed_s(), cycle);
      const double b = kid_rl::gait_phase(k * step_dt, cycle);
      double d = std::abs(a - b);
      d = std::min(d, 1.0 - d);  // 0/1 경계를 넘는 경우
      worst_long = std::max(worst_long, d);
    }
    check(worst_long < 1e-9, "phase drifts over a 10 minute run (worst " +
      std::to_string(worst_long) + ")");
  }

  // 6. 재진입: SAFE_HOLD 로 내려갔다가 다시 들어오면 새 에피소드처럼 0 부터.
  {
    kid_rl::GaitClock c;
    c.restart();
    for (int i = 0; i < 37; ++i) {c.advance(step_dt);}
    check(c.elapsed_s() > 0.7, "clock did not advance");
    c.restart();
    c.advance(step_dt);
    check(c.elapsed_s() == 0.0, "re-entry does not restart the phase at 0");
    c.advance(step_dt);
    check(std::abs(c.elapsed_s() - step_dt) < 1e-15, "second tick after re-entry is not step_dt");
  }

  if (g_failures == 0) {
    std::cout << "gait_phase_test: all checks passed (" << rows.size()
              << " mjlab rows, formula worst " << worst << ", sequence worst " << worst_seq
              << ")\n";
    return 0;
  }
  std::cerr << g_failures << " check(s) failed\n";
  return 1;
}
