// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "kid_rl/policy_runner.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: policy_runner_test MODEL_PATH GOLDEN_PATH\n";
    return 2;
  }

  // 기댓값 파일(test/gen_policy_golden.py 가 만든다):
  //   # 주석
  //   <관측 크기> <gait_phase 사용 여부 0/1>
  //   <관측 값들>      에피소드 첫 관측 모양: 전부 0, 위상 모델이면 (sin, cos) = (0, 1)
  //   <기대 action 들>
  // 모델을 새로 내보내면 이 파일을 다시 만든다. 예전처럼 기댓값을 소스에 박아 두면 모델이
  // 바뀔 때마다 테스트가 조용히 낡는다(2026-09-21, 2026-09-23 두 번 그랬다).
  std::ifstream golden(argv[2]);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(golden, line)) {
    if (!line.empty() && line[0] != '#') {lines.push_back(line);}
  }
  if (lines.size() != 3) {
    std::cerr << "golden file " << argv[2] << " is malformed\n";
    return 2;
  }
  std::size_t golden_size = 0;
  int golden_gait = 0;
  std::stringstream(lines[0]) >> golden_size >> golden_gait;
  auto parse = [](const std::string & text) {
      std::vector<double> v;
      std::stringstream ss(text);
      for (double x; ss >> x;) {v.push_back(x);}
      return v;
    };
  const std::vector<double> observation = parse(lines[1]);
  const std::vector<double> expected = parse(lines[2]);

  const std::vector<std::string> joints{
    "torso_yaw", "neck_yaw", "head_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow_pitch", "left_wrist_pitch",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow_pitch", "right_wrist_pitch",
    "left_hip_yaw", "left_hip_roll_crank", "left_hip_pitch",
    "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll_crank",
    "right_hip_yaw", "right_hip_roll_crank", "right_hip_pitch",
    "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll_crank"};

  // config/controllers.yaml 의 default_positions 와 같아야 한다. load() 가 모델
  // 메타데이터와 이 값을 대조하므로, 이 테스트는 골든 출력뿐 아니라
  // "설정과 모델이 같은 계약을 쓰는가"까지 확인한다.
  const double kHip = 0.2617993877991494;    // 15 deg
  const double kKnee = 0.5235987755982988;   // 30 deg
  // 팔 접은 자세(130 deg). 위상 fixture 모델은 팔을 편 예전 계약으로 내보낸 것이라 0 이다.
  const double kElbow = golden_gait ? 0.0 : 2.2689280275926285;
  const std::vector<double> defaults{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -kElbow, 0.0, 0.0, 0.0, 0.0, kElbow, 0.0, 0.0, 0.0,
    -kHip, kKnee, -kHip, 0.0,
    0.0, 0.0, kHip, -kKnee, kHip, 0.0};
  // 학습값(mjlab joint_pos_action.scale). 1.0 으로 돌렸다가 실기에서 정책이
  // 발산했다. load() 가 메타데이터와 다르면 실패하므로 여기서도 지킨다.
  const double action_scale = 0.5;

  // 컨트롤러 build_observation() 의 순서 (kid_rl_controller.cpp kBaseObservationNames).
  const std::vector<std::string> base_names{
    "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions", "command"};
  const std::size_t base_size = 3 + 3 + 3 * joints.size() + 3;

  kid_rl::PolicyRunner runner;
  std::string error;
  bool uses_gait_phase = false;
  if (!runner.load(
      argv[1], base_names, base_size, uses_gait_phase, 25, joints, defaults, action_scale, error))
  {
    std::cerr << "load failed: " << error << '\n';
    return 1;
  }
  if (uses_gait_phase != (golden_gait != 0)) {
    std::cerr << "gait_phase detection " << uses_gait_phase << " != golden " << golden_gait << '\n';
    return 1;
  }
  const std::size_t observation_size = base_size + (uses_gait_phase ? 2 : 0);
  if (observation_size != golden_size || observation.size() != golden_size) {
    std::cerr << "observation size " << observation_size << " != golden " << golden_size << '\n';
    return 1;
  }

  // 관측 순서가 다른 컨트롤러는 거부해야 한다: 앞 두 항목을 바꿔 넣으면 load 실패.
  {
    kid_rl::PolicyRunner wrong;
    std::string wrong_error;
    bool wrong_gait = false;
    std::vector<std::string> swapped = base_names;
    std::swap(swapped[0], swapped[1]);
    if (wrong.load(
        argv[1], swapped, base_size, wrong_gait, 25, joints, defaults, action_scale, wrong_error))
    {
      std::cerr << "load accepted a controller with a different observation order\n";
      return 1;
    }
  }

  std::vector<double> actions(25, 0.0);
  for (int iteration = 0; iteration < 100; ++iteration) {
    if (!runner.run(observation, actions)) {
      std::cerr << "inference failed at iteration " << iteration << '\n';
      return 1;
    }
  }
  for (std::size_t i = 0; i < actions.size(); ++i) {
    if (!std::isfinite(actions[i]) || std::abs(actions[i] - expected[i]) > 1e-5) {
      std::cerr << "action mismatch at " << i << ": " << actions[i]
                << " != " << expected[i] << '\n';
      return 1;
    }
  }
  std::cout << "policy_runner_test: " << argv[1] << " observation " << observation_size
            << (uses_gait_phase ? " (with gait_phase)" : "") << " OK\n";
  return 0;
}
