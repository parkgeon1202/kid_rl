#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""Drive the exported ONNX policy in a MuJoCo viewer from a gamepad.

This is a bench for the *deployment* contract, not a training tool. It rebuilds
the observation the same way kid_rl_controller.cpp does and reads the same
config the robot will read, so a mismatch between controllers.yaml, the ONNX
file and the MJCF shows up here instead of on hardware:

  * joint names and their order come from config/controllers.yaml, resolved
    against the MJCF by name -- a typo fails at startup
  * default_positions is cross-checked against the ONNX `default_joint_pos`
    metadata (which is rounded to 1e-3, hence the tolerance)
  * the observation is gyro(3) + projected_gravity(3) + (q - q_default)(N) +
    qd(N) + last_raw_action(N) + cmd(3), matching build_observation()
  * the action becomes default + action_scale * action, matching
    update_policy_setpoint() -- no filter, no rate limit, zero-order held
    between policy ticks
  * the gamepad axes map exactly as gamepad.cpp maps them

What it is NOT: mjlab's BAM actuator model is not here. The MJCF's own
<position> actuators are used instead, so torque/latency behaviour is only a
first-order match. Treat divergence in *gait quality* as expected and
divergence in *observation layout* as a bug.

Usage::

    python3 policy_viewer.py                      # keyboard (WASD/JK)
    python3 policy_viewer.py --joystick /dev/input/event20

Keyboard: W/S = vx, A/D = vy, J/K = wz, SPACE = stop, R = reset.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import re
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_PKG = _HERE.parent

# Linux input_event: struct timeval (2x long), __u16 type, __u16 code, __s32 value
_EVENT_FMT = "llHHi"
_EVENT_SIZE = struct.calcsize(_EVENT_FMT)
_EV_ABS = 0x03
_ABS_X, _ABS_Y, _ABS_RX = 0x00, 0x01, 0x03

# The 8 passive followers of the hip/ankle roll four-bar linkages carry no
# encoder and are set by the tendon equality, so the policy neither sees nor
# drives them. Same filter mjlab applies (DOFS_FILTER in kid_RL_env_cfg.py).
_PASSIVE = re.compile(r"_(cap|actual)$")


class Gamepad:
    """Raw /dev/input reader mirroring gamepad.cpp.

    evdev is deliberately not used: the C++ side reads the same struct off the
    same device, so keeping the mapping literal makes the two easy to compare.
    Axes are normalised to [-1, 1] from the device's own range, then mapped
    straight to a velocity command exactly as the controller does:
    cmd = axis * max, and 0 inside the deadband.
    """

    def __init__(self, path: str, config: dict):
        self._limit = np.array([float(config["walk_command.max_vx"]),
                                float(config["walk_command.max_vy"]),
                                float(config["walk_command.max_wz"])])
        # 실기 컨트롤러에서 게임패드를 뺄 때(2026-10-30) 마지막으로 쓰던 값.
        self._deadband = np.full(3, 0.1)
        self._axes = {_ABS_X: 0.0, _ABS_Y: 0.0, _ABS_RX: 0.0}
        self._range = self._absinfo_ranges(path)
        self._lock = threading.Lock()
        self._fd = os.open(path, os.O_RDONLY)
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _absinfo_ranges(path: str) -> dict[int, tuple[int, int]]:
        """EVIOCGABS via ioctl; falls back to int16 range if it fails."""

        class _AbsInfo(ctypes.Structure):
            _fields_ = [(n, ctypes.c_int32) for n in
                        ("value", "minimum", "maximum", "fuzz", "flat", "resolution")]

        out: dict[int, tuple[int, int]] = {}
        fd = os.open(path, os.O_RDONLY)
        try:
            import fcntl
            for code in (_ABS_X, _ABS_Y, _ABS_RX):
                info = _AbsInfo()
                # _IOR('E', 0x40 + code, struct input_absinfo)
                req = (2 << 30) | (ctypes.sizeof(_AbsInfo) << 16) | (ord("E") << 8) | (0x40 + code)
                try:
                    fcntl.ioctl(fd, req, info)
                    if info.maximum > info.minimum:
                        out[code] = (info.minimum, info.maximum)
                except OSError:
                    pass
        finally:
            os.close(fd)
        for code in (_ABS_X, _ABS_Y, _ABS_RX):
            out.setdefault(code, (-32768, 32767))
        return out

    def _loop(self) -> None:
        while True:
            data = os.read(self._fd, _EVENT_SIZE)
            if len(data) < _EVENT_SIZE:
                continue
            _, _, etype, code, value = struct.unpack(_EVENT_FMT, data)
            if etype != _EV_ABS or code not in self._axes:
                continue
            lo, hi = self._range[code]
            norm = 2.0 * (value - lo) / (hi - lo) - 1.0
            with self._lock:
                self._axes[code] = float(np.clip(norm, -1.0, 1.0))

    def command(self) -> np.ndarray:
        with self._lock:
            lx, ly, rx = self._axes[_ABS_X], self._axes[_ABS_Y], self._axes[_ABS_RX]
        raw = np.clip(np.array([-ly, -lx, -rx]), -1.0, 1.0)
        raw[np.abs(raw) < self._deadband] = 0.0
        return raw * self._limit


def build_scene(mujoco, spec) -> None:
    """Add what mjlab's SceneCfg adds and the robot MJCF leaves out.

    kid_RL_dance.xml is the robot alone: no ground geom (worldbody has zero
    geoms) and no lights, because mjlab composes the scene at load time. Without
    this the robot falls forever.

    This reproduces the `flat=True` variant of the task --
    TerrainEntityCfg(terrain_type="plane") from kid_RL_env_cfg.py:511-515 --
    rather than the procedural generator SCENE_CFG uses by default. A bench for
    the observation contract does not need stairs, and the flat task
    (Mjlab-Velocity-Flat-KidRL) is trained from the same checkpoints.
    """
    # terrain_entity.py:_import_ground_plane -- a body and a geom both named
    # "terrain", size (0,0,0.01), everything else left at MuJoCo defaults
    # (friction 1 0.005 0.0001, condim 3). The foot geoms carry their own
    # friction from the robot MJCF; mjlab randomises those per env at startup
    # over (0.6, 1.2), which is not reproduced here.
    body = spec.worldbody.add_body(name="terrain")
    body.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_PLANE,
                  size=(0.0, 0.0, 0.01), material="groundplane")

    # terrain_entity.py:_DEFAULT_PLANE_TEXTURE, with kid_RL's own material
    # override (_GROUNDPLANE_MATERIAL: texrepeat 5x5, not mjlab's 4x4).
    spec.add_texture(name="groundplane", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     mark=mujoco.mjtMark.mjMARK_EDGE,
                     rgb1=(0.2, 0.3, 0.4), rgb2=(0.1, 0.2, 0.3),
                     markrgb=(0.8, 0.8, 0.8), width=300, height=300)
    mat = spec.add_material(name="groundplane", texuniform=True,
                            texrepeat=(5.0, 5.0), reflectance=0.2)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "groundplane"

    # _apply_scene_xml_visuals() -- skybox gradient and camera framing.
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=(0.3, 0.5, 0.7), rgb2=(0.0, 0.0, 0.0),
                     width=512, height=3072)
    spec.visual.global_.azimuth = 160
    spec.visual.global_.elevation = -20

    # mjlab/scene/scene.xml, which every mjlab scene starts from.
    spec.visual.headlight.diffuse = (0.6, 0.6, 0.6)
    spec.visual.headlight.ambient = (0.3, 0.3, 0.3)
    spec.visual.headlight.specular = (0.0, 0.0, 0.0)
    spec.visual.rgba.haze = (0.15, 0.25, 0.35, 1.0)
    spec.visual.map.force = 0.005
    spec.visual.quality.shadowsize = 8192
    spec.stat.meansize = 0.03


def load_config(path: Path) -> dict:
    import yaml
    return yaml.safe_load(path.read_text())["kid_rl"]["ros__parameters"]


def check_against_onnx(session, joints, defaults, obs_dim, act_dim) -> None:
    """Compare the deployment config with the metadata baked into the policy."""
    meta = session.get_modelmeta().custom_metadata_map
    if "joint_names" in meta:
        names = [n for n in meta["joint_names"].split(",") if not _PASSIVE.search(n)]
        if names != list(joints):
            raise SystemExit(
                "joint order mismatch between controllers.yaml and the ONNX metadata\n"
                f"  yaml : {list(joints)}\n  onnx : {names}")
        print(f"  joint order      : matches ONNX metadata ({len(names)})")
    if "default_joint_pos" in meta and "joint_names" in meta:
        full = dict(zip(meta["joint_names"].split(","),
                        (float(v) for v in meta["default_joint_pos"].split(","))))
        worst = max(abs(full[n] - d) for n, d in zip(joints, defaults))
        # list_to_csv_str() writes 3 decimals, so anything under 1e-3 is the
        # exporter's rounding rather than a real disagreement.
        status = "ok (exporter rounding)" if worst <= 1.1e-3 else "MISMATCH"
        print(f"  default_positions: max deviation {worst:.2e} rad -- {status}")
        if worst > 1.1e-3:
            raise SystemExit("default_positions disagrees with the trained pose")
    if "action_scale" in meta:
        print(f"  action_scale     : onnx={meta['action_scale']}")
    print(f"  dimensions       : obs {obs_dim}, action {act_dim}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mjcf", type=Path,
                    default=Path("/home/geonhu/mjlab_ws/mjlab_kid_RL/src/mjlab_kid_rl/"
                                 "robot/kid_rl_dance/kid_RL_dance.xml"))
    ap.add_argument("--onnx", type=Path, default=_PKG / "models" / "policy.onnx")
    ap.add_argument("--config", type=Path, default=_PKG / "config" / "controllers.yaml")
    ap.add_argument("--joystick", type=str, default=None,
                    help="/dev/input/eventN. Omitted: keyboard control.")
    ap.add_argument("--policy-rate", type=float, default=None,
                    help="Override policy_rate_hz from the config.")
    ap.add_argument("--timestep", type=float, default=0.005,
                    help="Physics timestep. Defaults to mjlab SIM_CFG's 0.005 (200 Hz), "
                         "which is what the policy was trained against; the MJCF's own "
                         "<option timestep> is 0.001. Pass 0 to keep the file's value.")
    ap.add_argument("--solver-iterations", type=int, default=50,
                    help="mjlab SIM_CFG iterations. 0 keeps the MJCF value.")
    args = ap.parse_args()

    try:
        import mujoco
        import mujoco.viewer
        import onnxruntime as ort
    except ImportError as exc:
        raise SystemExit(
            f"missing dependency: {exc.name}\n"
            "  mujoco/numpy live in the mjlab venv; onnxruntime and pyyaml do not.\n"
            "  e.g. /home/geonhu/mjlab_ws/mjlab_kid_RL/.venv/bin/pip install onnxruntime pyyaml")

    cfg = load_config(args.config)
    joints = list(cfg["joints"])
    defaults = np.asarray(cfg["default_positions"], dtype=np.float64)
    action_scale = float(cfg["action_scale"])
    policy_hz = float(args.policy_rate or cfg["policy_rate_hz"])
    n = len(joints)

    spec = mujoco.MjSpec.from_file(str(args.mjcf))
    build_scene(mujoco, spec)
    model = spec.compile()
    # The MJCF ships timestep=0.001 but mjlab overrides it to 0.005 in SIM_CFG,
    # and 0.005 x decimation 4 is the 50 Hz the policy was trained at. Matching
    # it here keeps the contact and solver behaviour in the same regime; leaving
    # the file's 0.001 would give a decimation of 20 and a stiffer contact
    # solution than training ever saw.
    if args.timestep > 0:
        model.opt.timestep = args.timestep
    if args.solver_iterations > 0:
        model.opt.iterations = args.solver_iterations
    data = mujoco.MjData(model)

    # Resolve every name up front; the controller does the same in
    # map_interfaces() and refuses to activate when one is missing.
    def jid(name):
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if i < 0:
            raise SystemExit(f"joint '{name}' from controllers.yaml is not in the MJCF")
        return i

    # Actuators are resolved through their transmission target, not by name:
    # the four linkage cranks are driven by actuators called *_hip_roll /
    # *_ankle_roll while the joints they move are *_hip_roll_crank /
    # *_ankle_roll_crank, so a name lookup misses exactly those four.
    joint_to_actuator = {
        int(model.actuator_trnid[a, 0]): a
        for a in range(model.nu)
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
    }

    def aid(name):
        i = joint_to_actuator.get(jid(name), -1)
        if i < 0:
            raise SystemExit(f"no actuator drives joint '{name}' in the MJCF")
        return i

    qpos_adr = np.array([model.jnt_qposadr[jid(j)] for j in joints])
    qvel_adr = np.array([model.jnt_dofadr[jid(j)] for j in joints])
    ctrl_adr = np.array([aid(j) for j in joints])
    gyro_adr = model.sensor_adr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")]
    base_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    session = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    in_name = session.get_inputs()[0].name
    obs_dim = session.get_inputs()[0].shape[1]
    act_dim = session.get_outputs()[0].shape[1]

    print(f"MJCF   : {args.mjcf}")
    print(f"ONNX   : {args.onnx}")
    print(f"config : {args.config}")
    check_against_onnx(session, joints, defaults, obs_dim, act_dim)
    expected = 9 + 3 * n
    if obs_dim != expected or act_dim != n:
        raise SystemExit(f"dimension mismatch: onnx {obs_dim}/{act_dim}, config {expected}/{n}")

    sim_hz = 1.0 / model.opt.timestep
    decimation = max(1, round(sim_hz / policy_hz))
    print(f"  rates            : sim {sim_hz:.0f} Hz, policy {policy_hz:.0f} Hz "
          f"(decimation {decimation})")

    pad = None
    if args.joystick:
        pad = Gamepad(args.joystick, cfg)
        print(f"  gamepad          : {args.joystick}")
    else:
        print("  gamepad          : none -- W/S vx, A/D vy, J/K wz, SPACE stop, R reset")

    key_cmd = np.zeros(3)
    reset_flag = threading.Event()
    step = np.array([float(cfg["walk_command.max_vx"]), float(cfg["walk_command.max_vy"]),
                     float(cfg["walk_command.max_wz"])])

    def on_key(keycode):
        c = chr(keycode) if 0 < keycode < 0x110000 else ""
        table = {"W": (0, +1), "S": (0, -1), "A": (1, +1),
                 "D": (1, -1), "J": (2, +1), "K": (2, -1)}
        if c.upper() in table:
            i, s = table[c.upper()]
            key_cmd[i] = np.clip(key_cmd[i] + s * step[i] * 0.5, -step[i], step[i])
        elif c == " ":
            key_cmd[:] = 0.0
        elif c.upper() == "R":
            reset_flag.set()

    def reset():
        # The MJCF's "home" keyframe holds the base pose but leaves every joint
        # at zero; mjlab overrides those from init_state at load time and the
        # raw file does not carry that. Take the base from the keyframe and the
        # joints from controllers.yaml, so what is being exercised is the pose
        # the robot will actually start from.
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        data.qpos[qpos_adr] = defaults
        data.qvel[:] = 0.0
        data.ctrl[ctrl_adr] = defaults
        mujoco.mj_forward(model, data)

    reset()
    last_action = np.zeros(n, dtype=np.float64)
    obs = np.zeros(obs_dim, dtype=np.float32)
    targets = defaults.copy()

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        tick = 0
        wall = time.perf_counter()
        while viewer.is_running():
            if reset_flag.is_set():
                reset_flag.clear()
                last_action[:] = 0.0
                targets = defaults.copy()
                tick = 0

            if tick % decimation == 0:
                cmd = pad.command() if pad else key_cmd

                # projected gravity in the base frame. mjlab computes it as
                # quat_apply_inverse(root_link_quat_w, (0,0,-1)); xmat is that
                # rotation as a matrix, so its transpose times -z is the same.
                rot = data.xmat[base_bid].reshape(3, 3)
                proj_grav = rot.T @ np.array([0.0, 0.0, -1.0])

                k = 0
                obs[k:k + 3] = data.sensordata[gyro_adr:gyro_adr + 3]; k += 3
                obs[k:k + 3] = proj_grav; k += 3
                obs[k:k + n] = data.qpos[qpos_adr] - defaults; k += n
                obs[k:k + n] = data.qvel[qvel_adr]; k += n
                obs[k:k + n] = last_action; k += n
                obs[k:k + 3] = cmd

                action = session.run(None, {in_name: obs[None, :]})[0][0]
                last_action[:] = action           # raw, exactly as the obs feeds back
                targets = defaults + action_scale * action

            data.ctrl[ctrl_adr] = targets         # zero-order hold between policy ticks
            mujoco.mj_step(model, data)
            tick += 1

            viewer.sync()
            wall += model.opt.timestep
            sleep = wall - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                wall = time.perf_counter()        # fell behind; stop accumulating debt
    return 0


if __name__ == "__main__":
    sys.exit(main())
