#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""관절별 target -> pos 응답을 real 과 sim 에서 비교한다.

sim 쪽은 mjlab_sim_csv.py --replay REAL.csv --open-loop-targets 로 만든 CSV (정책 없이 real 의
target 을 그대로 모터에 넣은 것). 두 CSV 의 같은 policy_t_s 구간에서:

  track_rms  : target - pos 의 RMS (rad)
  gain@f     : 주파수 f 에서 |pos| / |target| (1 이면 완전히 따라감)
  lag@f      : 주파수 f 에서 pos 가 target 보다 늦는 시간 (ms, 위상차로 계산)

f 는 기본으로 real target 의 주 주파수(떨림 주파수)를 쓴다.
행 k 의 pos 는 tick k 에 읽은 값, target 은 tick k 에 보낸 값이라 lag 에는 한 틱(20 ms)
만큼의 '보내고 다음 틱에 읽기' 지연이 real/sim 모두 똑같이 들어 있다.

사용: python3 motor_response.py REAL.csv SIM_OPENLOOP.csv [--t0 0.2] [--t1 2.1] [--freq 12.5]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load(path: Path, t0: float, t1: float) -> dict[str, np.ndarray]:
    rows = [r for r in csv.DictReader(open(path)) if t0 <= float(r["policy_t_s"]) <= t1]
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]
            if k.startswith(("pos_", "target_", "policy_t_s"))}


def transfer(target: np.ndarray, pos: np.ndarray, f: float, dt: float) -> tuple[float, float]:
    """주파수 f 에서 pos/target 의 이득과 지연(ms). Hann 창 DFT."""
    w = np.hanning(target.size)
    tt = np.arange(target.size) * dt
    e = np.exp(-2j * np.pi * f * tt) * w
    T = np.sum((target - target.mean()) * e)
    P = np.sum((pos - pos.mean()) * e)
    H = P / T
    phase = np.angle(H)                    # 음수 = pos 가 늦음
    return float(abs(H)), float(-phase / (2 * np.pi * f) * 1000)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("real", type=Path)
    ap.add_argument("sim", type=Path)
    ap.add_argument("--t0", type=float, default=0.2)
    ap.add_argument("--t1", type=float, default=2.1)
    ap.add_argument("--freq", type=float, default=None)
    args = ap.parse_args()

    R, S = load(args.real, args.t0, args.t1), load(args.sim, args.t0, args.t1)
    t = R["policy_t_s"]
    S = {k: np.interp(t, S["policy_t_s"], v) for k, v in S.items()}
    dt = 0.02
    joints = [k[4:] for k in R if k.startswith("pos_")]

    f = args.freq
    if f is None:
        spec = sum(np.abs(np.fft.rfft(R[f"target_{j}"] - R[f"target_{j}"].mean())) for j in joints)
        fr = np.fft.rfftfreq(t.size, dt)
        spec[fr < 2] = 0
        f = float(fr[np.argmax(spec)])
    print(f"구간 {t[0]:.2f}~{t[-1]:.2f} s ({t.size} 틱), 비교 주파수 {f:.2f} Hz")
    print(f"{'joint':26s} {'targ amp':>8s} | {'track_rms':>9s} {'gain':>5s} {'lag ms':>6s} (real)"
          f" | {'track_rms':>9s} {'gain':>5s} {'lag ms':>6s} (sim) | gain r/s")
    rows = []
    for j in joints:
        tr, ts = R[f"target_{j}"], S[f"target_{j}"]
        amp = float(np.std(tr))
        gr, lr = transfer(tr, R[f"pos_{j}"], f, dt)
        gs, ls = transfer(ts, S[f"pos_{j}"], f, dt)
        er = float(np.sqrt(np.mean((tr - R[f"pos_{j}"]) ** 2)))
        es = float(np.sqrt(np.mean((ts - S[f"pos_{j}"]) ** 2)))
        rows.append((amp, j, er, gr, lr, es, gs, ls))
    for amp, j, er, gr, lr, es, gs, ls in sorted(rows, reverse=True):
        print(f"{j:26s} {amp:8.3f} | {er:9.3f} {gr:5.2f} {lr:6.1f}       "
              f" | {es:9.3f} {gs:5.2f} {ls:6.1f}      | {gr / gs if gs > 0 else float('nan'):5.2f}")
    print("targ amp = real target 표준편차(rad). 진폭이 작은 관절은 gain/lag 가 부정확하다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
