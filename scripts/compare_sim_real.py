#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""real CSV(Jetson 컨트롤러)와 sim CSV(mjlab_sim_csv.py)를 policy_t_s 로 맞춰 비교한다.

정렬: 두 파일 모두 policy_t_s = 첫 추론 이후 시간. sim 값을 real 의 policy_t_s 에 선형 보간한다.
real 끝에서 서보 25개가 모두 무응답인 구간(전원 스위치)은 잘라낸다.

출력
  - 관절별 RMS 차이 (pos, vel, action), IMU(gravity, gyro) RMS 차이
  - 떨림 지표: action 변화율 |Δa| 평균, 5 Hz 이상 관절 속도 에너지 (sim/real 각각)
  - 상호상관으로 추정한 시간 어긋남 (다리 관절 pos, ±200 ms 안에서)
  - PNG 그래프 (--plot DIR)

사용: python3 compare_sim_real.py REAL.csv SIM.csv [--plot DIR]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load(path: Path) -> dict[str, np.ndarray]:
    rows = [r for r in csv.DictReader(open(path)) if float(r.get("policy_t_s", -1)) >= 0]
    if not rows:
        raise SystemExit(f"{path}: policy_t_s >= 0 인 행이 없다")
    t = np.array([float(r["policy_t_s"]) for r in rows])
    if np.any(np.diff(t) < 0):
        # 정책을 껐다 켠 실행: 첫 구간만 쓴다.
        cut = int(np.argmax(np.diff(t) < 0)) + 1
        print(f"{path.name}: policy_t_s 가 {t[cut - 1]:.2f}s 뒤 0 으로 돌아간다 -> 첫 구간만 비교")
        rows = rows[:cut]
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows], dtype=object)
    if "rx_missing_count" in out:
        # 끝의 전체 무응답(전원 차단) 잘라내기: 25개(=관절 수) 무응답이 처음 나온 곳 앞까지.
        n = sum(1 for k in out if k.startswith("pos_"))
        dead = np.nonzero(out["rx_missing_count"] >= n)[0]
        if dead.size:
            # 무응답 직전 몇 틱은 이미 부분 실패일 수 있어 3 틱 더 뺀다
            end = max(int(dead[0]) - 3, 1)
            print(f"{path.name}: {out['policy_t_s'][dead[0]]:.2f}s 부터 전체 무응답(전원) -> 그 앞까지만")
            out = {k: v[:end] for k, v in out.items()}
    return out


def hf_energy(vel: np.ndarray, dt: float, f_cut: float = 5.0) -> float:
    """f_cut 이상 주파수 성분의 RMS (떨림)."""
    v = vel - vel.mean()
    spec = np.fft.rfft(v)
    f = np.fft.rfftfreq(v.size, dt)
    spec[f < f_cut] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(spec, v.size) ** 2)))


def lag_ms(a: np.ndarray, b: np.ndarray, dt: float, max_ms: float = 200.0) -> tuple[float, float]:
    """b 가 a 보다 몇 ms 늦는지(양수 = sim 이 real 보다 늦음)와 그때의 상관계수."""
    a = (a - a.mean()) / (a.std() + 1e-12)
    b = (b - b.mean()) / (b.std() + 1e-12)
    m = int(max_ms / 1000 / dt)
    best = (0, -2.0)
    for s in range(-m, m + 1):
        x, y = (a[:len(a) - s], b[s:]) if s >= 0 else (a[-s:], b[:len(b) + s])
        c = float(np.mean(x * y))
        if c > best[1]:
            best = (s, c)
    return best[0] * dt * 1000, best[1]


def plot_all(outdir: Path, t, R, S, joints, window) -> None:
    """관측 4종(gyro, gravity, joint pos, joint vel)과 action 을 real/sim 겹쳐 그린다.

    joint pos 는 정책이 보는 값과 같은 모양(실측 위치)에 target 을 겹친다.
    joint vel 은 real 속도 레지스터(정책 입력), real 위치 차분 Δpos/Δt(실제 움직임), sim 속도를 같이 그린다.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    outdir.mkdir(parents=True, exist_ok=True)
    if window is None:
        m, tag = np.ones_like(t, dtype=bool), ""
    else:
        m, tag = (t >= window[0]) & (t <= window[1]), f"zoom_{window[0]:g}-{window[1]:g}_"
    tt = t[m]
    mk = dict(marker=".", ms=3) if window is not None else {}
    legs = [j for j in joints if any(s in j for s in ("hip", "knee", "ankle"))]
    upper = [j for j in joints if j not in legs]

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(outdir / f"{tag}{name}.png", dpi=100)
        plt.close(fig)

    fig, ax = plt.subplots(6, 1, figsize=(13, 10), sharex=True)
    for a, k in zip(ax, ("gyro_x", "gyro_y", "gyro_z", "gravity_x", "gravity_y", "gravity_z")):
        a.plot(tt, R[k][m], label="real", lw=1, **mk)
        a.plot(tt, S[k][m], label="sim", lw=1, **mk)
        a.set_ylabel(k)
        a.grid(alpha=0.3)
    ax[0].legend(ncol=2, fontsize=8)
    ax[-1].set_xlabel("policy_t_s [s]")
    save(fig, "imu")

    # 위치 차분: 실제 읽은 시각 간격(dt_s)으로 나눈다
    def fd(pos):
        d = np.gradient(pos, t)
        return d[m]

    for group, keys in (("legs", legs), ("upper", upper)):
        fig, ax = plt.subplots(len(keys), 1, figsize=(13, 1.5 * len(keys)), sharex=True)
        for a, j in zip(ax, keys):
            a.plot(tt, R[f"pos_{j}"][m], label="real pos", lw=1, **mk)
            a.plot(tt, S[f"pos_{j}"][m], label="sim pos", lw=1, **mk)
            a.plot(tt, R[f"target_{j}"][m], "--", label="real target", lw=0.6)
            a.plot(tt, S[f"target_{j}"][m], ":", label="sim target", lw=0.6)
            a.set_ylabel(j, rotation=0, ha="right", fontsize=7)
            a.grid(alpha=0.3)
        ax[0].legend(ncol=4, fontsize=7)
        ax[-1].set_xlabel("policy_t_s [s]")
        save(fig, f"{group}_pos")

        fig, ax = plt.subplots(len(keys), 1, figsize=(13, 1.5 * len(keys)), sharex=True)
        for a, j in zip(ax, keys):
            a.plot(tt, R[f"vel_{j}"][m], label="real vel (register = policy input)", lw=1, **mk)
            a.plot(tt, fd(R[f"pos_{j}"]), label="real Δpos/Δt", lw=0.7, alpha=0.7)
            a.plot(tt, S[f"vel_{j}"][m], label="sim vel", lw=1, **mk)
            a.set_ylabel(j, rotation=0, ha="right", fontsize=7)
            a.grid(alpha=0.3)
        ax[0].legend(ncol=3, fontsize=7)
        ax[-1].set_xlabel("policy_t_s [s]")
        save(fig, f"{group}_vel")

    fig, ax = plt.subplots(len(legs), 1, figsize=(13, 1.5 * len(legs)), sharex=True)
    for a, j in zip(ax, legs):
        a.plot(tt, R[f"action_{j}"][m], label="real", lw=1, **mk)
        a.plot(tt, S[f"action_{j}"][m], label="sim", lw=1, **mk)
        a.set_ylabel(j, rotation=0, ha="right", fontsize=7)
        a.grid(alpha=0.3)
    ax[0].legend(ncol=2, fontsize=7)
    ax[-1].set_xlabel("policy_t_s [s]")
    save(fig, "legs_action")


def plot_spectra(outdir: Path, t, R, S, joints) -> None:
    """관측별 주파수 성분 크기 (진폭 스펙트럼, 0~25 Hz). 관절은 다리 pitch 6개 평균."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    dt = 0.02
    f = np.fft.rfftfreq(t.size, dt)
    w = np.hanning(t.size)

    def spec(x):
        return 2 * np.abs(np.fft.rfft((x - x.mean()) * w)) / w.sum()

    legs = [j for j in joints if any(s in j for s in ("hip_pitch", "knee_pitch", "ankle_pitch"))]
    items = [(k, k) for k in ("gyro_x", "gyro_y", "gyro_z", "gravity_x", "gravity_y", "gravity_z")]
    items += [("joint_pos (mean of 6 leg pitch)", "pos_"), ("joint_vel (mean of 6 leg pitch)", "vel_"),
              ("action (mean of 6 leg pitch)", "action_")]
    fig, ax = plt.subplots(3, 3, figsize=(15, 10))
    for a, (title, key) in zip(ax.flat, items):
        if key.endswith("_"):
            r = np.mean([spec(R[key + j]) for j in legs], axis=0)
            s_ = np.mean([spec(S[key + j]) for j in legs], axis=0)
        else:
            r, s_ = spec(R[key]), spec(S[key])
        a.semilogy(f, r + 1e-6, label="real", lw=1)
        a.semilogy(f, s_ + 1e-6, label="sim", lw=1)
        if key == "vel_":
            fdr = np.mean([spec(np.gradient(R["pos_" + j], t)) for j in legs], axis=0)
            a.semilogy(f, fdr + 1e-6, label="real Δpos/Δt", lw=0.8, alpha=0.8)
        a.axvspan(10, 15, color="red", alpha=0.08)
        a.set_title(title, fontsize=9)
        a.set_xlabel("Hz")
        a.grid(alpha=0.3, which="both")
        a.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(outdir / "spectra.png", dpi=100)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("real", type=Path)
    ap.add_argument("sim", type=Path)
    ap.add_argument("--plot", type=Path, default=None, help="PNG 를 저장할 폴더")
    ap.add_argument("--zoom", type=float, nargs=2, action="append", default=[], metavar=("T0", "T1"),
                    help="이 구간(policy_t_s)만 확대한 그래프도 그린다. 여러 번 줄 수 있다")
    args = ap.parse_args()

    real, sim = load(args.real), load(args.sim)
    t = real["policy_t_s"]
    t = t[t <= sim["policy_t_s"][-1]]
    n = t.size
    joints = [k[4:] for k in real if k.startswith("pos_")]
    S = {k: np.interp(t, sim["policy_t_s"], v) for k, v in sim.items() if v.dtype.kind == "f"}
    R = {k: v[:n] for k, v in real.items()}
    dt = 0.02
    print(f"\n비교 구간: 0 ~ {t[-1]:.2f} s ({n} 틱), real dt 중앙값 {np.median(R['dt_s'][1:]) * 1000:.2f} ms")

    # 위상은 1 -> 0 으로 되돌아가 선형 보간하면 안 된다. sin/cos 로 보간해 되돌린다.
    ang = 2 * np.pi * sim["gait_phase"]
    s_phase = np.arctan2(np.interp(t, sim["policy_t_s"], np.sin(ang)),
                         np.interp(t, sim["policy_t_s"], np.cos(ang))) / (2 * np.pi) % 1.0
    phase_err = np.abs(((R["gait_phase"] - s_phase) + 0.5) % 1.0 - 0.5)
    print(f"gait_phase 차이 최대 {phase_err.max():.2e} (시간 정렬 확인)")

    def rms(x):
        return float(np.sqrt(np.mean(x ** 2)))

    print("\nIMU  (RMS 차이 / real 평균 / sim 평균)")
    for k in ("gravity_x", "gravity_y", "gravity_z", "gyro_x", "gyro_y", "gyro_z"):
        print(f"  {k:10s} {rms(R[k] - S[k]):7.3f}   {R[k].mean():+7.3f}   {S[k].mean():+7.3f}")

    print("\n관절 (pos: rad, vel: rad/s) -- pos RMS 차이 큰 순")
    print(f"  {'joint':26s} {'posΔrms':>8s} {'posΔmean':>8s} {'velΔrms':>8s} {'actΔrms':>8s}"
          f" {'|Δa| real':>9s} {'|Δa| sim':>9s} {'hfvel real':>10s} {'hfvel sim':>9s}")
    stats = []
    for j in joints:
        dp = R[f"pos_{j}"] - S[f"pos_{j}"]
        stats.append((rms(dp), j, dp.mean(),
                      rms(R[f"vel_{j}"] - S[f"vel_{j}"]),
                      rms(R[f"action_{j}"] - S[f"action_{j}"]),
                      np.mean(np.abs(np.diff(R[f"action_{j}"]))),
                      np.mean(np.abs(np.diff(sim[f"action_{j}"][:n]))),
                      hf_energy(R[f"vel_{j}"], dt), hf_energy(sim[f"vel_{j}"][:n], dt)))
    for s in sorted(stats, reverse=True):
        print(f"  {s[1]:26s} {s[0]:8.4f} {s[2]:+8.4f} {s[3]:8.3f} {s[4]:8.3f}"
              f" {s[5]:9.3f} {s[6]:9.3f} {s[7]:10.3f} {s[8]:9.3f}")
    tot = np.array([s[5:] for s in stats])
    print(f"  {'(평균)':26s} {'':8s} {'':8s} {'':8s} {'':8s}"
          f" {tot[:, 0].mean():9.3f} {tot[:, 1].mean():9.3f} {tot[:, 2].mean():10.3f} {tot[:, 3].mean():9.3f}")
    print("  |Δa| = 한 스텝 사이 action 변화 평균, hfvel = 5 Hz 이상 관절속도 RMS (떨림)")

    print("\n시간 어긋남 (상호상관, 다리 pos; + 면 sim 이 늦음)")
    for j in [j for j in joints if "hip_pitch" in j or "knee" in j or "ankle_pitch" in j]:
        ms, c = lag_ms(R[f"pos_{j}"], S[f"pos_{j}"], dt)
        print(f"  {j:26s} {ms:+6.0f} ms  (상관 {c:.2f})")

    if args.plot:
        plot_all(args.plot, t, R, S, joints, None)
        for t0, t1 in args.zoom:
            plot_all(args.plot, t, R, S, joints, (t0, t1))
        plot_spectra(args.plot, t, R, S, joints)
        print(f"\n그래프: {args.plot}/ (imu, legs/upper _pos/_vel, legs_action, spectra"
              + (", zoom_*" if args.zoom else "") + ")")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
