#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""mjlab 학습 환경이 정책에 실제로 넣는 gait_phase 관측을 기록해 기준 데이터로 만든다.

컨트롤러의 위상 구현(include/kid_rl/gait_phase.hpp)이 학습과 '같은 시각에 같은 값'을
넣는지 test/gait_phase_test.cpp 가 이 파일과 대조한다. 여기서는 추정한 공식이 아니라
mjlab 의 ManagerBasedRlEnv 를 실제로 띄워, env.reset()/env.step() 이 돌려주는 actor
관측의 마지막 두 칸(= gait_phase, 관측 순서는 ONNX 메타데이터와 같음)을 그대로 적는다.

기록하는 것:
  - 에피소드 시작 후 k 번째 관측(= k 번째 action 을 정하는 관측)의 sin, cos
  - 같은 시점의 env.episode_length_buf (학습 코드가 위상을 계산하는 근거)
  - 중간에 강제 리셋을 넣어, 리셋 직후 관측이 다시 k = 0 인지

실행 (mjlab_kid_RL venv):
  cd ~/mjlab_ws/mjlab_kid_RL
  .venv/bin/python ~/deploy_ws/src/kid_rl/test/gen_gait_phase_golden.py \
      ~/deploy_ws/src/kid_rl/test/data/gait_phase_mjlab.csv
"""

import csv
import sys

import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

TASK = "Mjlab-Velocity-KidRL"
STEPS = 600          # 12 초 = 약 14 주기
RESET_AT = 300       # 이 스텝에서 env 0 을 강제 리셋


def main(out_path: str) -> None:
    cfg = load_env_cfg(TASK)
    cfg.scene.num_envs = 2
    # action 을 0 으로 넣으면 로봇이 금방 넘어져 에피소드가 한 주기(약 43 스텝)도 안 된다.
    # 위상이 한 바퀴 돌아 0 으로 넘어가는 구간까지 기록하려고 시간 초과 외의 종료 조건을 뺀다.
    # 위상은 episode_length_buf 만의 함수라 로봇이 넘어져 있어도 값은 같다.
    cfg.terminations = {k: v for k, v in cfg.terminations.items() if k == "time_out"}
    term = cfg.observations["actor"].terms["gait_phase"]
    cycle_time = float(term.params["cycle_time"])
    step_dt = cfg.sim.mujoco.timestep * cfg.decimation
    names = list(cfg.observations["actor"].terms.keys())
    assert names[-1] == "gait_phase", names
    assert term.delay_max_lag == 0 and term.noise is None, "gait_phase 는 지연/노이즈가 없어야 한다"

    env = ManagerBasedRlEnv(cfg, device="cuda:0" if torch.cuda.is_available() else "cpu")
    obs, _ = env.reset()
    zero = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)

    rows = []
    k = 0             # env 0 의 에피소드 시작 후 관측 번호
    def record(o, note):
        actor = o["actor"][0]
        rows.append({
            "k": k,
            "episode_length_buf": int(env.episode_length_buf[0]),
            "sin": float(actor[-2]),
            "cos": float(actor[-1]),
            "note": note,
        })
    record(obs, "reset")
    for step in range(1, STEPS + 1):
        obs, _, terminated, truncated, _ = env.step(zero)
        if bool(terminated[0]) or bool(truncated[0]):
            k = 0
            record(obs, "auto_reset")
        else:
            k += 1
            record(obs, "")
        if step == RESET_AT:
            obs, _ = env.reset(env_ids=torch.tensor([0], device=env.device))
            k = 0
            record(obs, "forced_reset")

    with open(out_path, "w", newline="") as f:
        f.write(f"# task={TASK} cycle_time={cycle_time!r} step_dt={step_dt!r}\n")
        w = csv.DictWriter(f, fieldnames=["k", "episode_length_buf", "sin", "cos", "note"])
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} rows, cycle_time={cycle_time}, step_dt={step_dt}")


if __name__ == "__main__":
    main(sys.argv[1])
