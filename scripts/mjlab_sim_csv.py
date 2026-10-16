#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""배포 ONNX 정책을 mjlab 에서 돌리며 실기 컨트롤러와 같은 형식의 CSV 를 남긴다.

sim 과 real 을 나란히 비교하기 위한 도구다 (비교는 scripts/compare_sim_real.py).

시간 정렬
  두 CSV 모두 `policy_t_s` 열을 가진다: 첫 추론(= 학습의 에피소드 시작, 위상 0) 이후 흐른 시간.
  real 은 컨트롤러가 실측 주기를 더해 기록하고, sim 은 k 번째 스텝에서 k * 0.02 이다.
  real 의 `t_s` 는 컨트롤러 활성화 기준이라 실행마다 정책 시작 시점이 다르므로 쓰지 않는다.

명령 재생 (--replay REAL.csv)
  real 실행의 cmd_vx/vy/wz 를 policy_t_s 에 맞춰 그대로 넣는다(영차 유지). 같은 입력에 대한 sim 과
  real 의 반응을 비교하려면 이 옵션을 쓴다. 없으면 --command 로 준 고정 명령을 쓴다.

행 하나의 의미 (real CSV 와 같음)
  그 틱에 읽은 상태(관절·IMU) -> 그 상태로 만든 관측 -> 추론한 action(±clip) -> 목표각.
  pos_/vel_/gravity/gyro 는 sim 의 참값(센서 잡음·지연 없는 상태)이다. real 의 값은 센서로 읽은
  값이다. sim 은 관측 노이즈·지연이 없으므로 이 참값이 곧 정책 입력이다.

sim 설정: 항상 명목 모델(enable_randomization=False). real 과의 차이를 보는 도구라 sim 이 실행마다
  달라지면 안 된다 -- 도메인 무작위화 이벤트(마찰·질량·armature·엔코더 바이어스·밀기), 초기 자세
  무작위, 관측 노이즈, 관측 지연, BAM 명령 지연, 전압 무작위화를 모두 끈다(BAM vin = 16.3 V 고정).
  target_ 은 실기 컨트롤러와 같은 식: clamp(default + action_scale * action, 관절 한계(yaml)).

실행 (mjlab_kid_RL venv):
  cd ~/mjlab_ws/mjlab_kid_RL
  .venv/bin/python ~/deploy_ws/src/kid_rl/scripts/mjlab_sim_csv.py \\
      --replay ~/kid_rl_logs/kid_rl_YYYYMMDD_HHMMSS.csv --out sim.csv
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import csv
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mjlab_onnx_joystick import OnnxPolicy  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
OBS_LAYOUT = (("base_ang_vel", 3), ("projected_gravity", 3), ("joint_pos", None),
              ("joint_vel", None), ("actions", None), ("command", 3), ("gait_phase", 2))


class CommandSchedule:
    """policy_t_s -> (vx, vy, wz). real CSV 를 영차 유지로 재생하거나 고정값을 준다."""

    def __init__(self, replay: Path | None, fixed: tuple[float, float, float]):
        self.fixed = np.asarray(fixed, dtype=np.float32)
        self.t = None
        if replay is not None:
            rows = [r for r in csv.DictReader(open(replay)) if float(r.get("policy_t_s", -1)) >= 0]
            if not rows:
                raise SystemExit(f"{replay}: policy_t_s >= 0 인 행이 없다 (정책이 돈 실행인지, "
                                 "policy_t_s 열이 있는 새 로그인지 확인)")
            self.t = np.array([float(r["policy_t_s"]) for r in rows])
            self.cmd = np.array([[float(r["cmd_vx"]), float(r["cmd_vy"]), float(r["cmd_wz"])]
                                 for r in rows], dtype=np.float32)
            self.targets = {k[7:]: np.array([float(r[k]) for r in rows])
                            for k in rows[0] if k.startswith("target_")}
            # --ground-assist 용: real 몸통 자세(중력 방향)와 각속도(필터 전 값이 있으면 그것).
            self.grav = np.array([[float(r["gravity_x"]), float(r["gravity_y"]),
                                   float(r["gravity_z"])] for r in rows])
            gk = "gyro_raw_" if "gyro_raw_x" in rows[0] else "gyro_"
            self.gyro = np.array([[float(r[gk + "x"]), float(r[gk + "y"]), float(r[gk + "z"])]
                                  for r in rows])

    @property
    def duration(self) -> float | None:
        return None if self.t is None else float(self.t[-1])

    def at(self, t: float) -> np.ndarray:
        if self.t is None:
            return self.fixed
        i = int(np.searchsorted(self.t, t + 1e-9, side="right")) - 1
        return self.cmd[max(i, 0)]

    def attitude_at(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """real 의 그 시각 몸통 중력 방향(몸통 좌표)과 각속도 (영차 유지)."""
        i = max(int(np.searchsorted(self.t, t + 1e-9, side="right")) - 1, 0)
        return self.grav[i], self.gyro[i]

    def target_at(self, t: float, joints: list[str]) -> np.ndarray:
        """real 이 그 시각에 모터로 보낸 목표각 (영차 유지)."""
        i = max(int(np.searchsorted(self.t, t + 1e-9, side="right")) - 1, 0)
        return np.array([self.targets[j][i] for j in joints])


def _bam_actuators(robot) -> list:
    """robot.actuators 안의 BamActuator 인스턴스 (지연 래퍼가 있으면 벗겨서)."""
    found = []
    for act in robot.actuators:
        seen, stack = set(), [act]
        while stack:
            a = stack.pop()
            if id(a) in seen:
                continue
            seen.add(id(a))
            if type(a).__name__ == "BamActuator":
                found.append(a)
                break
            stack.extend(v for k, v in vars(a).items()
                         if not k.startswith("__") and hasattr(v, "__dict__")
                         and type(v).__module__.startswith(("bam", "mjlab")))
    return found


def apply_actuator_scales(env, robot, joints, armature_scale, friction_scale, kp_scale) -> None:
    """실험용 동역학 배율. 모두 1 이면 아무것도 하지 않는다."""
    import mujoco
    import torch
    if armature_scale != 1.0:
        m = env.sim.mj_model
        dofs = []
        for j in joints:
            jid = next(i for i in range(m.njnt)
                       if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i).split("/")[-1] == j)
            dofs.append(int(m.jnt_dofadr[jid]))
        env.sim.expand_model_fields(("dof_armature",))
        arm = env.sim.model.dof_armature
        arm[:, dofs] = arm[:, dofs] * armature_scale
        print(f"[scale] armature x{armature_scale} on {len(dofs)} dofs -> "
              f"e.g. {float(arm[0, dofs[-1]]):.4f}")
    bams = _bam_actuators(robot)
    if friction_scale != 1.0:
        for b in bams:
            orig = b._compute_friction_budget

            def scaled(*a, _orig=orig, **k):
                return _orig(*a, **k) * friction_scale
            b._compute_friction_budget = scaled
        print(f"[scale] friction x{friction_scale} on {len(bams)} BAM actuators")
    if kp_scale != 1.0:
        for b in bams:
            b.kp_scale[:] = kp_scale
            if b.default_kp_scale is not None:
                b.default_kp_scale[:] = kp_scale
        print(f"[scale] kp x{kp_scale} on {len(bams)} BAM actuators")
    if (friction_scale != 1.0 or kp_scale != 1.0) and not bams:
        raise SystemExit("BamActuator 를 찾지 못했다")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--task", default="Mjlab-Velocity-Flat-KidRL-IRC",
                    choices=("Mjlab-Velocity-Flat-KidRL-IRC", "Mjlab-Velocity-Flat-KidRL",
                             "Mjlab-Velocity-KidRL"))
    ap.add_argument("--onnx", type=Path, default=PKG / "models/policy.onnx")
    ap.add_argument("--controllers", type=Path, default=PKG / "config/controllers.yaml",
                    help="관절 순서·기본 자세·관절 한계·clip 을 읽는다 (실기 컨트롤러와 같은 파일)")
    ap.add_argument("--replay", type=Path, default=None, help="명령을 재생할 real CSV")
    ap.add_argument("--open-loop-targets", action="store_true",
                    help="정책 대신 --replay CSV 의 target_ 을 그대로 모터에 넣는다 (모터 응답 비교용). "
                         "action = (target - default) / action_scale")
    ap.add_argument("--command", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                    metavar=("VX", "VY", "WZ"), help="--replay 가 없을 때 고정 명령")
    ap.add_argument("--duration", type=float, default=None,
                    help="초. 기본은 재생하는 real 실행 길이, 없으면 20")
    ap.add_argument("--bam", nargs="*", default=[], metavar="KEY=VALUE",
                    help="실험용: 모든 BAM 액추에이터 cfg 필드를 덮어쓴다 (예: stiff_frictionloss=False)")
    ap.add_argument("--physics-dt", type=float, default=None,
                    help="실험용: 물리 간격(s). 정책 주기 20 ms 는 유지하고 decimation = 0.02 / dt")
    ap.add_argument("--hang", type=float, default=None, metavar="LIFT_M",
                    help="몸통을 서 있는 높이보다 LIFT_M 위 공중에 똑바로(gravity z=-1) 매단다. "
                         "몸통 자유도는 남기고 weld 구속(손으로 잡은 것 같은 스프링)으로 잡는다")
    ap.add_argument("--hang-timeconst", type=float, default=0.02,
                    help="매다는 구속의 시간상수(s). 작을수록 단단하게 잡음 (물리 간격의 2배 이상)")
    ap.add_argument("--hang-torquescale", type=float, default=1.0,
                    help="매다는 구속의 회전 강도 배율 (1 = 위치와 같은 강도, 작을수록 몸통이 잘 기울고 떨림)")
    ap.add_argument("--armature-scale", type=float, default=1.0,
                    help="실험용: 구동 관절 armature(모터 회전자 관성) 배율")
    ap.add_argument("--friction-scale", type=float, default=1.0,
                    help="실험용: BAM 마찰(쿨롱+부하, 점성 제외) 배율")
    ap.add_argument("--kp-scale", type=float, default=1.0,
                    help="실험용: BAM 펌웨어 kp 배율 (모든 모터 공통)")
    ap.add_argument("--gyro-filter-hz", type=float, default=None,
                    help="actor gyro 저역통과 컷오프. 기본은 controllers.yaml imu_gyro_filter_hz "
                         "(실기와 같게). 0 이면 필터 없음")
    ap.add_argument("--foot-friction", type=float, default=None,
                    help="실험용: 발 충돌 박스(*_foot_collision_*)의 미끄럼 마찰계수 (기본 모델 1.0)")
    ap.add_argument("--kp-scale-motor", nargs="*", default=[], metavar="MOTOR=SCALE",
                    help="실험용: 특정 BAM 모터 그룹만 kp 배율 (예: mx106v2=2.0). 이름은 BAM 액추에이터 클래스 기준")
    ap.add_argument("--linkage-friction", type=float, default=None,
                    help="실험용: hip/ankle roll 4절 링크 접촉 geom(*_cap, *_groove_wall*)의 미끄럼 마찰 (기본 0.2)")
    ap.add_argument("--roll-joint-scale", type=float, default=1.0,
                    help="실험용: 4절 링크 수동 관절(*_actual, *_cap)의 frictionloss/damping 배율")
    ap.add_argument("--foot-stud-contact", action="store_true",
                    help="실험용: 발 충돌 상자를 CAD 발바닥의 네 돌기 사각형(145.5 x 91 mm)으로 줄인다")
    ap.add_argument("--ground-assist", type=float, nargs=2, default=None, metavar=("KP", "KD"),
                    help="지면 하중 open-loop 용: 몸통에 보조 토크만 걸어 real 의 몸통 자세(roll/pitch)와 "
                         "각속도를 따라가게 한다. 힘은 주지 않으므로 몸무게는 다리가 받는다. "
                         "예: 200 16 (N*m/rad, N*m*s/rad). 종료 조건은 끈다")
    ap.add_argument("--random-torque", type=float, default=None, metavar="NM",
                    help="실험용: 몸통(base_link)에 축마다 균등분포 [-NM, NM] 의 외부 토크를 걸고 "
                         "0.3~1.0 s 마다 새로 뽑는다 (학습의 torso_torque_disturbance 이벤트와 같은 교란)")
    ap.add_argument("--stud-solref", type=float, nargs=2, default=None, metavar=("TIMECONST", "DAMPRATIO"),
                    help="실험용: 스터드 접촉 solref 를 고정값으로 (기본 0.02 1). 학습의 stud_contact_solref 랜덤화 확인용")
    ap.add_argument("--imu-mirror-y", action="store_true",
                    help="실험용: 정책에 들어가는 IMU 를 몸통 y 축 거울상으로 바꾼다 "
                         "(gravity_y, gyro_x, gyro_z 부호 반전). CSV 에는 정책이 본 값이 기록된다")
    ap.add_argument("--motor-kt", nargs="*", default=[], metavar="MOTOR=KT",
                    help="실험용: BAM 모터 그룹의 kt(토크 상수 = 역기전력 상수, V/(rad/s))를 덮어쓴다 "
                         "(예: xh=3.8). 클수록 같은 전압에서 최고 속도가 낮아진다")
    ap.add_argument("--backlash", type=float, default=0.0, metavar="RAD",
                    help="실험용: 관절 유격 반폭(rad). 목표-현재 오차가 이 안이면 모터가 힘을 내지 않는다 "
                         "(학습의 randomize_joint_backlash 와 같은 데드존 모델, 전체 유격은 2배)")
    ap.add_argument("--backlash-motor", nargs="*", default=[], metavar="MOTOR",
                    help="--backlash 를 걸 BAM 모터 그룹(예: mx106v2 xh). 비우면 전부")
    ap.add_argument("--torque-stats", action="store_true",
                    help="끝에 관절별 액추에이터 토크 통계(최대, rms, 토크 상한에 닿은 시간 비율)를 출력한다")
    ap.add_argument("--frc-scale", type=float, default=1.0,
                    help="관절 actuatorfrcrange(토크 상한) 배율. 1.43 이면 stall torque 100%% 상당")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default=None)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    if args.open_loop_targets and args.replay is None:
        ap.error("--open-loop-targets 에는 --replay 가 필요하다")

    import torch
    import mjlab.tasks  # noqa: F401  registers KidRL
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.utils.lab_api.math import quat_apply
    from mjlab.tasks.registry import load_rl_cfg
    from mjlab_kid_rl.tasks.kid_RL_env_cfg import make_kid_rl_velocity_env_cfg

    ctl = yaml.safe_load(open(args.controllers))["kid_rl"]["ros__parameters"]
    joints = list(ctl["joints"])
    default = np.asarray(ctl["default_positions"], dtype=np.float64)
    jmin = np.asarray(ctl["joint_position_min"], dtype=np.float64)
    jmax = np.asarray(ctl["joint_position_max"], dtype=np.float64)
    action_clip = float(ctl.get("action_clip", 0.0))

    flat = "Flat" in args.task
    robot_kind = "irc" if args.task.endswith("-IRC") else "dance"
    env_cfg = make_kid_rl_velocity_env_cfg(play=True, flat=flat, enable_randomization=False,
                                           robot=robot_kind)
    # actor gyro 필터를 실기 컨트롤러 설정과 같게 맞춘다 (학습 기본값이 아니라 배포 설정 기준).
    gyro_fc = args.gyro_filter_hz if args.gyro_filter_hz is not None else \
        float(ctl.get("imu_gyro_filter_hz", 0.0))
    gyro_term = env_cfg.observations["actor"].terms["base_ang_vel"]
    if gyro_fc > 0:
        from mjlab_kid_rl.tasks.mdp import imu_gyro_lowpass
        gyro_term.func = imu_gyro_lowpass
        gyro_term.params = {"sensor_name": "robot/imu_ang_vel", "cutoff_hz": gyro_fc}
    else:
        from mjlab.envs.mdp import builtin_sensor
        gyro_term.func = builtin_sensor
        gyro_term.params = {"sensor_name": "robot/imu_ang_vel"}
    print(f"[gyro] actor gyro low-pass: {gyro_fc if gyro_fc > 0 else 'off'}")
    env_cfg.scene.num_envs = 1
    if args.physics_dt:
        policy_dt = env_cfg.sim.mujoco.timestep * env_cfg.decimation
        env_cfg.sim.mujoco.timestep = args.physics_dt
        env_cfg.decimation = int(round(policy_dt / args.physics_dt))
        print(f"[physics] timestep {args.physics_dt} s, decimation {env_cfg.decimation}")
    if args.hang is not None:
        import mujoco
        robot_cfg = env_cfg.scene.entities["robot"]
        base_z = float(robot_cfg.init_state.pos[2]) + args.hang
        robot_cfg.init_state = replace(robot_cfg.init_state, pos=(0.0, 0.0, base_z),
                                       rot=(1.0, 0.0, 0.0, 0.0))
        orig_spec_fn = robot_cfg.spec_fn

        def hung_spec_fn():
            spec = orig_spec_fn()
            root = next(b for b in spec.bodies
                        if any(j.type == mujoco.mjtJoint.mjJNT_FREE for j in b.joints))
            eq = spec.add_equality(name="hang_weld")
            eq.type = mujoco.mjtEq.mjEQ_WELD
            eq.objtype = mujoco.mjtObj.mjOBJ_BODY
            eq.name1 = root.name          # name2 비움 = world
            # data = anchor(3), relpos(3), relquat(4), torquescale.
            # world 가 root 기준으로 (0, 0, -base_z), 회전 없음 = root 가 (0, 0, base_z) 에 똑바로.
            eq.data[:11] = [0, 0, 0, 0, 0, -base_z, 1, 0, 0, 0, args.hang_torquescale]
            eq.solref = [args.hang_timeconst, 1.0]
            return spec
        robot_cfg.spec_fn = hung_spec_fn
        # 공중이라 발 접촉 기반 종료 조건은 의미가 없다. 넘어짐도 없으므로 종료는 전부 끈다.
        env_cfg.terminations.clear()
        print(f"[hang] base z {base_z:.3f} m, weld timeconst {args.hang_timeconst} s, "
              f"torquescale {args.hang_torquescale}")
    if args.frc_scale != 1.0:
        robot_cfg = env_cfg.scene.entities["robot"]
        frc_spec_fn = robot_cfg.spec_fn

        def frc_scaled_spec_fn():
            spec = frc_spec_fn()
            n = 0
            for j in spec.joints:
                lo, hi = j.actfrcrange
                if hi > lo:
                    j.actfrcrange = [lo * args.frc_scale, hi * args.frc_scale]
                    n += 1
            print(f"[frc] actuatorfrcrange x{args.frc_scale} on {n} joints")
            return spec
        robot_cfg.spec_fn = frc_scaled_spec_fn
    if args.foot_stud_contact:
        robot_cfg = env_cfg.scene.entities["robot"]
        stud_spec_fn = robot_cfg.spec_fn

        def stud_spec_fn_wrapped():
            spec = stud_spec_fn()
            for g in spec.geoms:
                if g.name in ("left_foot_collision_1", "right_foot_collision_1"):
                    # CAD 메쉬 최저점(돌기) 범위: x [-0.0628, 0.0827], |y| 폭 0.091
                    g.pos[0] = 0.00995
                    g.size[0] = 0.07275
                    g.size[1] = 0.0455
                    print(f"[foot stud] {g.name}: pos {list(g.pos)} size {list(g.size)}")
            return spec
        robot_cfg.spec_fn = stud_spec_fn_wrapped
    if args.ground_assist is not None:
        if args.replay is None:
            ap.error("--ground-assist 에는 --replay 가 필요하다")
        env_cfg.terminations.clear()
    for kv in args.bam:
        key, val = kv.split("=", 1)
        val = {"True": True, "False": False}.get(val, val)
        for act in env_cfg.scene.entities["robot"].articulation.actuators:
            if not hasattr(act, key):
                raise SystemExit(f"{type(act).__name__} 에 {key} 필드가 없다")
            cur = getattr(act, key)
            setattr(act, key, type(cur)(val) if isinstance(cur, (int, float)) and not isinstance(cur, bool) else val)
        print(f"[bam override] {key} = {val}")
    env_cfg.seed = args.seed
    # real 은 몇 분씩 돌리므로 20 초 에피소드 제한을 뺀다(그대로면 20 초마다 리셋되어 위상이 0 으로
    # 돌아간다). 넘어짐 같은 종료 조건은 남기고, 걸리면 그 행을 FAULT 로 쓰고 멈춘다.
    env_cfg.terminations.pop("time_out", None)
    agent_cfg = load_rl_cfg(args.task)
    if action_clip > 0 and abs(float(agent_cfg.clip_actions or 0) - action_clip) > 1e-9:
        raise SystemExit(f"학습 clip_actions={agent_cfg.clip_actions} 와 yaml action_clip="
                         f"{action_clip} 이 다르다")

    names = list(env_cfg.observations["actor"].terms.keys())
    sched = CommandSchedule(args.replay, tuple(args.command))
    duration = args.duration or sched.duration or 20.0

    device = args.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
    env = ManagerBasedRlEnv(cfg=env_cfg, device=device)
    try:
        # 명령 덮어쓰기: command_manager.compute() 직후 real 명령으로 바꾼다. step() 안에서 compute 는
        # 관측 계산 직전에 불리므로, 그 스텝이 끝난 뒤의 관측(다음 action 을 정하는 관측)에 들어간다.
        term = env.command_manager.get_term("twist")
        original_compute = term.compute
        clock = {"t": 0.0}

        def compute(dt: float) -> None:
            original_compute(dt)
            term.vel_command_b[0] = torch.as_tensor(
                sched.at(clock["t"]), device=term.vel_command_b.device,
                dtype=term.vel_command_b.dtype)
        term.compute = compute

        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        policy = OnnxPolicy(args.onnx, wrapped.num_actions, device)
        if list(policy.action_names) != joints:
            raise SystemExit("ONNX 관절 순서가 controllers.yaml joints 와 다르다")

        robot = env.scene["robot"]
        action_term = env.action_manager.get_term("joint_pos")
        ids = action_term.target_ids
        sim_names = [robot.joint_names[int(i)] for i in ids]
        if sim_names != joints:
            raise SystemExit(f"sim action 관절 순서가 다르다: {sim_names[:3]}...")
        apply_actuator_scales(env, robot, joints, args.armature_scale, args.friction_scale,
                              args.kp_scale)
        for kv in args.motor_kt:
            name, val = kv.split("=")
            hit = 0
            for bact in _bam_actuators(robot):
                cls = type(bact._bam_model.actuator).__name__.lower()
                if cls.startswith(name.lower().replace("_", "")):
                    old = float(bact._bam_model.kt.value)
                    bact._bam_model.kt.value = float(val)
                    hit += 1
                    print(f"[kt] {cls}: {old:.3f} -> {float(val):.3f}")
            if not hit:
                raise SystemExit(f"--motor-kt {name}: 해당 BAM 액추에이터 그룹 없음")
        if args.backlash > 0:
            import dataclasses
            hit = 0
            for bact in _bam_actuators(robot):
                cls = type(bact._bam_model.actuator).__name__.lower()
                if args.backlash_motor and not any(cls.startswith(m.lower().replace("_", "")) for m in args.backlash_motor):
                    continue
                inner = bact.compute

                def compute(cmd, _inner=inner):
                    err = cmd.position_target - cmd.pos
                    dead = torch.sign(err) * torch.clamp(err.abs() - args.backlash, min=0.0)
                    return _inner(dataclasses.replace(cmd, position_target=cmd.pos + dead))
                bact.compute = compute
                hit += 1
            print(f"[backlash] half width {args.backlash} rad on {hit} actuator group(s)")
            if not hit:
                raise SystemExit("--backlash-motor: 해당 BAM 액추에이터 그룹 없음")
        for kv in args.kp_scale_motor:
            name, val = kv.split("=")
            hit = 0
            for b in _bam_actuators(robot):
                cls = type(b._bam_model.actuator).__name__.lower()
                if cls.startswith(name.lower().replace("_", "")):
                    b.kp_scale[:] = float(val)
                    b.default_kp_scale[:] = float(val)
                    hit += 1
            print(f"[kp] {name} x{val} on {hit} actuator group(s)")
            if not hit:
                raise SystemExit(f"{name}: 해당 BAM 액추에이터 그룹 없음")
        if args.linkage_friction is not None:
            import mujoco
            m = env.sim.mj_model
            gids = [g for g in range(m.ngeom)
                    if any(k in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")
                           for k in ("_hip_cap", "_ankle_cap", "_groove_wall"))]
            env.sim.expand_model_fields(("geom_friction",))
            env.sim.model.geom_friction[:, gids, 0] = args.linkage_friction
            print(f"[linkage friction] {args.linkage_friction} on {len(gids)} geoms")
        if args.roll_joint_scale != 1.0:
            import mujoco
            m = env.sim.mj_model
            dofs = [int(m.jnt_dofadr[j]) for j in range(m.njnt)
                    if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "").endswith(("_actual", "_cap"))]
            env.sim.expand_model_fields(("dof_frictionloss", "dof_damping"))
            env.sim.model.dof_frictionloss[:, dofs] *= args.roll_joint_scale
            env.sim.model.dof_damping[:, dofs] *= args.roll_joint_scale
            print(f"[roll joints] frictionloss/damping x{args.roll_joint_scale} on {len(dofs)} dofs")
        if args.stud_solref is not None:
            import mujoco
            m = env.sim.mj_model
            gids = [g for g in range(m.ngeom)
                    if "_foot_collision_stud_" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
            env.sim.expand_model_fields(("geom_solref",))
            env.sim.model.geom_solref[:, gids, 0] = args.stud_solref[0]
            env.sim.model.geom_solref[:, gids, 1] = args.stud_solref[1]
            print(f"[stud solref] timeconst {args.stud_solref[0]} dampratio {args.stud_solref[1]} on {len(gids)} geoms")
        if args.foot_friction is not None:
            import mujoco
            m = env.sim.mj_model
            gids = [g for g in range(m.ngeom)
                    if "_foot_collision_" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
            env.sim.expand_model_fields(("geom_friction",))
            env.sim.model.geom_friction[:, gids, 0] = args.foot_friction
            print(f"[foot friction] {args.foot_friction} on {len(gids)} foot geoms")
        step_dt = env.step_dt
        sizes = {"joint_pos": len(joints), "joint_vel": len(joints), "actions": len(joints)}
        layout, offset = {}, 0
        for name, n in OBS_LAYOUT:
            if name not in names:
                continue
            n = n or sizes[name]
            layout[name] = (offset, offset + n)
            offset += n
        if names != [n for n, _ in OBS_LAYOUT if n in names]:
            raise SystemExit(f"관측 순서가 예상과 다르다: {names}")

        clock["t"] = 0.0
        obs, _ = wrapped.reset()          # 첫 관측: 에피소드 시작, 명령은 real 의 t=0 값

        header = ("t_s,dt_s,mode,imu_valid,imu_age_ms,command_fresh,gravity_x,gravity_y,gravity_z,"
                  "gyro_x,gyro_y,gyro_z,cmd_vx,cmd_vy,cmd_wz,gait_phase,policy_t_s,"
                  "gyro_raw_x,gyro_raw_y,gyro_raw_z").split(",")
        for j in joints:
            header += [f"pos_{j}", f"vel_{j}", f"target_{j}", f"err_{j}", f"action_{j}", f"hwerr_{j}"]
        # sim 에서만 알 수 있는 참값: 몸통 속도(몸통 좌표), yaw 각속도, 몸통 높이. 실기 CSV 에는 없는 열이다.
        header += ["base_vx", "base_vy", "base_wz", "base_z"]
        args.out.parent.mkdir(parents=True, exist_ok=True)
        steps = int(round(duration / step_dt))
        assist_log: list[float] = []
        rt_rng, rt_next = np.random.default_rng(args.seed), 0.0
        torque_log: list[np.ndarray] = []
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            with torch.inference_mode():
                for k in range(steps + 1):
                    t = k * step_dt
                    if args.imu_mirror_y:
                        g0, _ = layout["base_ang_vel"]
                        p0, _ = layout["projected_gravity"]
                        obs["actor"][:, g0] *= -1.0
                        obs["actor"][:, g0 + 2] *= -1.0
                        obs["actor"][:, p0 + 1] *= -1.0
                    o = obs["actor"][0].detach().cpu().numpy().astype(np.float64)
                    pos = robot.data.joint_pos[0, ids].detach().cpu().numpy().astype(np.float64)
                    grav = robot.data.projected_gravity_b[0].detach().cpu().numpy()
                    # 실기 CSV 와 같게: gyro/vel 은 정책 입력(필터·60 ms 평균 후), gyro_raw 는 참값.
                    gyro_raw = robot.data.root_link_ang_vel_b[0].detach().cpu().numpy()
                    gyro = o[slice(*layout["base_ang_vel"])]
                    vel = o[slice(*layout["joint_vel"])]
                    a0, a1 = layout["command"]
                    cmd = o[a0:a1]
                    if "gait_phase" in layout:
                        g0, _ = layout["gait_phase"]
                        phase = (math.atan2(o[g0], o[g0 + 1]) / (2 * math.pi)) % 1.0
                    else:
                        phase = -1.0
                    if args.open_loop_targets:
                        a = (sched.target_at(t, joints) - default) / policy.action_scale
                        raw = torch.as_tensor(a, dtype=torch.float32, device=device)[None]
                    else:
                        raw = policy(obs)
                    action = raw.detach().cpu().numpy()[0].astype(np.float64)
                    if action_clip > 0:
                        action = np.clip(action, -action_clip, action_clip)
                    target = np.clip(default + policy.action_scale * action, jmin, jmax)
                    row = [f"{t:.6g}", f"{step_dt:.6g}", "POLICY_RUNNING", 1, 0, 1,
                           *(f"{v:.6g}" for v in grav), *(f"{v:.6g}" for v in gyro),
                           *(f"{v:.6g}" for v in cmd), f"{phase:.6g}", f"{t:.6g}",
                           *(f"{v:.6g}" for v in gyro_raw)]
                    for i in range(len(joints)):
                        row += [f"{pos[i]:.6g}", f"{vel[i]:.6g}", f"{target[i]:.6g}",
                                f"{target[i] - pos[i]:.6g}", f"{action[i]:.6g}", 0]
                    lin_b = robot.data.root_link_lin_vel_b[0].detach().cpu().numpy()
                    row += [f"{lin_b[0]:.5g}", f"{lin_b[1]:.5g}", f"{gyro_raw[2]:.5g}",
                            f"{float(robot.data.root_link_pos_w[0, 2]):.5g}"]
                    if k == steps:
                        w.writerow(row)
                        break
                    if args.torque_stats:
                        torque_log.append(
                            robot.data.qfrc_actuator[0, ids].detach().cpu().numpy().astype(np.float64))
                    if args.random_torque is not None and t >= rt_next:
                        tau = rt_rng.uniform(-args.random_torque, args.random_torque, 3)
                        rt_next = t + rt_rng.uniform(0.3, 1.0)
                        dev = robot.data.root_link_quat_w.device
                        robot.write_external_wrench_to_sim(
                            torch.zeros(1, 1, 3, device=dev),
                            torch.as_tensor(tau, dtype=torch.float32, device=dev).reshape(1, 1, 3),
                            body_ids=[0])
                    if args.ground_assist is not None:
                        # 몸통 좌표에서: 현재 중력 방향 s, real 중력 방향 u. s x u 가 "real 보다 더 돈 각"의
                        # 중력 수직 성분이다. 그만큼 되돌리는 토크 + real 각속도와의 차이에 대한 감쇠.
                        kp_a, kd_a = args.ground_assist
                        u, w_real = sched.attitude_at(t)
                        u = u / np.linalg.norm(u)
                        s_b = grav / np.linalg.norm(grav)
                        tau_b = -kp_a * np.cross(s_b, u) - kd_a * (gyro_raw - w_real)
                        q = robot.data.root_link_quat_w[0]
                        tau_w = quat_apply(q, torch.as_tensor(tau_b, dtype=q.dtype, device=q.device))
                        robot.write_external_wrench_to_sim(
                            torch.zeros(1, 1, 3, device=q.device), tau_w.reshape(1, 1, 3),
                            body_ids=[0])
                        assist_log.append(float(torch.linalg.norm(tau_w)))
                    clock["t"] = (k + 1) * step_dt   # 이번 step 이 끝난 뒤 관측의 시각
                    obs, _, dones, _ = wrapped.step(raw)
                    if bool(dones.any()):
                        row[2] = "FAULT"
                        w.writerow(row)
                        print(f"terminated at policy_t_s={t + step_dt:.2f} s")
                        break
                    w.writerow(row)
        if args.torque_stats and torque_log:
            import mujoco
            m = env.sim.mj_model
            tq = np.abs(np.array(torque_log))
            print("[torque] joint, limit N*m, max, rms, fraction of time above 95% of limit")
            for i, j in enumerate(joints):
                jid = next(k for k in range(m.njnt)
                           if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, k).split("/")[-1] == j)
                lim = float(m.jnt_actfrcrange[jid, 1])
                print(f"[torque] {j} {lim:.2f} {tq[:, i].max():.2f} "
                      f"{np.sqrt((tq[:, i] ** 2).mean()):.2f} {(tq[:, i] > 0.95 * lim).mean():.3f}")
        if args.ground_assist is not None and assist_log:
            a = np.array(assist_log)
            print(f"[ground-assist] root body '{robot.body_names[0]}', assist torque rms "
                  f"{np.sqrt((a ** 2).mean()):.2f} N*m, p95 {np.percentile(a, 95):.2f}, max {a.max():.2f}; "
                  f"final base z {float(robot.data.root_link_pos_w[0, 2]):.3f} m")
        if args.hang is not None:
            z = float(robot.data.root_link_pos_w[0, 2])
            print(f"[hang] final base z {z:.3f} m (target {base_z:.3f})")
        print(f"wrote {args.out} ({k + 1} rows, {min(k + 1, steps + 1) * step_dt:.1f} s, "
              f"replay={'yes' if args.replay else 'no'})")
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
