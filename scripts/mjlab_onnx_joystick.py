#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""Run the deployment ONNX policy inside the original one-env MJLab task.

MJLab remains responsible for observations, actions, BAM actuation, resets and
physics decimation.  This script only replaces the PyTorch policy with the
exported ONNX policy and overwrites the velocity command from the gamepad
(default) or the keyboard.  No ROS topic is involved.

Gamepad : same mapping as the real controller (src/gamepad.cpp) -- left stick
          up/down = vx, left stick left/right = vy, right stick left/right = wz.
          Stick position maps directly to speed (full deflection = max), with
          the same deadband; releasing the stick gives zero.
Keyboard: --input keyboard.  W/S = vx, A/D = vy, J/K = wz, SPACE = stop.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import select
import struct
import threading
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np


class KeyboardCommand:
    """Increment and retain vx, vy and wz from MuJoCo key presses."""

    _KEY_COMMANDS = {
        ord("W"): (0, +1.0),
        ord("S"): (0, -1.0),
        ord("A"): (1, +1.0),
        ord("D"): (1, -1.0),
        ord("J"): (2, +1.0),
        ord("K"): (2, -1.0),
    }
    _KEY_SPACE = 32

    def __init__(self, max_vx: float, max_vy: float, max_wz: float, step_ratio: float):
        self._limit = np.asarray((max_vx, max_vy, max_wz), dtype=np.float32)
        self._step = self._limit * np.float32(step_ratio)
        self._command = np.zeros(3, dtype=np.float32)
        self._lock = threading.Lock()

    def on_key(self, key: int) -> None:
        with self._lock:
            if key == self._KEY_SPACE:
                self._command[:] = 0.0
                return
            axis_and_sign = self._KEY_COMMANDS.get(key)
            if axis_and_sign is None:
                return
            axis, sign = axis_and_sign
            self._command[axis] = np.clip(
                self._command[axis] + sign * self._step[axis],
                -self._limit[axis],
                self._limit[axis],
            )

    def handles_key(self, key: int) -> bool:
        return key == self._KEY_SPACE or key in self._KEY_COMMANDS

    def command(self) -> np.ndarray:
        with self._lock:
            return self._command.copy()

    @staticmethod
    def close() -> None:
        pass


class GamepadCommand:
    """Read a gamepad the way the real controller does (src/gamepad.cpp).

    Device choice, axis choice, normalisation, deadband and signs follow the C++
    code line for line, so a stick position gives the same (vx, vy, wz) here as
    on the robot: the first /dev/input/event* whose USB vendor/product matches,
    else the first whose name contains ``name_hint``; ABS_X / ABS_Y for the left
    stick and the first of ABS_RX, ABS_Z, ABS_RZ for the right stick.
    """

    _EV_ABS = 0x03
    _ABS_X, _ABS_Y, _ABS_Z, _ABS_RX, _ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x05
    _EVENT = struct.Struct("llHHi")  # struct input_event on 64-bit Linux
    _ABSINFO = struct.Struct("6i")  # struct input_absinfo
    _INPUT_ID = struct.Struct("4H")  # struct input_id: bustype, vendor, product, version

    @staticmethod
    def _ioc_read(number: int, size: int) -> int:
        # _IOC(_IOC_READ, 'E', number, size)
        return (2 << 30) | (size << 16) | (ord("E") << 8) | number

    def __init__(
        self,
        limits: tuple[float, float, float],
        deadband: tuple[float, float, float],
        device_path: str = "",
        name_hint: str = "Gulikit X",
        vendor_id: int = 0x045E,
        product_id: int = 0x028E,
    ) -> None:
        self._limit = np.asarray(limits, dtype=np.float64)
        self._deadband = np.asarray(deadband, dtype=np.float64)
        self._lock = threading.Lock()
        self._axes = {"left_x": 0.0, "left_y": 0.0, "right_x": 0.0}
        self._fd, self.device_path, self.device_name = self._open(
            device_path, name_hint, vendor_id, product_id
        )
        self._ranges: dict[int, tuple[float, float]] = {}
        self._codes = self._configure_axes()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _name(self, fd: int) -> str:
        try:
            raw = fcntl.ioctl(fd, self._ioc_read(0x06, 256), bytes(256))
        except OSError:
            return ""
        return raw.split(b"\0", 1)[0].decode(errors="replace")

    def _id(self, fd: int) -> tuple[int, int] | None:
        try:
            raw = fcntl.ioctl(fd, self._ioc_read(0x02, self._INPUT_ID.size), bytes(self._INPUT_ID.size))
        except OSError:
            return None
        _, vendor, product, _ = self._INPUT_ID.unpack(raw)
        return vendor, product

    def _absinfo(self, fd: int, code: int) -> tuple[int, int] | None:
        """(minimum, maximum) of an absolute axis, or None if the device has no such axis."""
        try:
            raw = fcntl.ioctl(
                fd, self._ioc_read(0x40 + code, self._ABSINFO.size), bytes(self._ABSINFO.size)
            )
        except OSError:
            return None
        _, minimum, maximum, _, _, _ = self._ABSINFO.unpack(raw)
        return minimum, maximum

    def _open(
        self, device_path: str, name_hint: str, vendor_id: int, product_id: int
    ) -> tuple[int, str, str]:
        flags = os.O_RDONLY | os.O_NONBLOCK
        if device_path:
            try:
                fd = os.open(device_path, flags)
            except OSError as error:
                raise RuntimeError(f"could not open {device_path}: {error.strerror}") from error
            return fd, device_path, self._name(fd)

        name_match: tuple[int, str] | None = None
        denied = 0
        for index in range(64):
            path = f"/dev/input/event{index}"
            try:
                candidate = os.open(path, flags)
            except PermissionError:
                denied += 1
                continue
            except OSError:
                continue
            if self._id(candidate) == (vendor_id, product_id):
                if name_match is not None:
                    os.close(name_match[0])
                return candidate, path, self._name(candidate)
            if name_match is None and name_hint and name_hint in self._name(candidate):
                name_match = (candidate, path)
            else:
                os.close(candidate)
        if name_match is not None:
            return name_match[0], name_match[1], self._name(name_match[0])
        hint = (
            f" ({denied} event devices were not readable by this user; a plugged-in gamepad "
            "is normally readable, so check that it is connected)" if denied else ""
        )
        raise RuntimeError(
            f"no gamepad found: no /dev/input/event* with USB id {vendor_id:04x}:{product_id:04x} "
            f"or a name containing '{name_hint}'{hint}. Use --gamepad-device to name one, "
            "or --input keyboard."
        )

    def _configure_axes(self) -> dict[int, str]:
        for code in (self._ABS_X, self._ABS_Y):
            info = self._absinfo(self._fd, code)
            if info is None:
                raise RuntimeError("gamepad does not expose ABS_X and ABS_Y")
            self._ranges[code] = info
        codes = {self._ABS_X: "left_x", self._ABS_Y: "left_y"}
        for code in (self._ABS_RX, self._ABS_Z, self._ABS_RZ):
            info = self._absinfo(self._fd, code)
            if info is not None:
                self._ranges[code] = info
                codes[code] = "right_x"
                return codes
        raise RuntimeError("gamepad does not expose ABS_RX, ABS_Z, or ABS_RZ")

    def _normalize(self, code: int, value: int) -> float:
        minimum, maximum = self._ranges[code]
        if maximum == minimum:
            return 0.0
        center = 0.5 * (maximum + minimum)
        half_range = 0.5 * (maximum - minimum)
        return float(np.clip((value - center) / half_range, -1.0, 1.0))

    def _handle_event(self, event_type: int, code: int, value: int) -> None:
        if event_type != self._EV_ABS or code not in self._codes:
            return
        with self._lock:
            self._axes[self._codes[code]] = self._normalize(code, value)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue
                data = os.read(self._fd, self._EVENT.size * 64)
            except OSError:
                # Unplugged: stop commanding motion, like the robot's stale-command hold.
                with self._lock:
                    self._axes = dict.fromkeys(self._axes, 0.0)
                return
            for offset in range(0, len(data) - self._EVENT.size + 1, self._EVENT.size):
                _, _, event_type, code, value = self._EVENT.unpack_from(data, offset)
                self._handle_event(event_type, code, value)

    def command(self) -> np.ndarray:
        with self._lock:
            # Same signs as gamepad.cpp: left Y -> vx, left X -> vy, right X -> yaw rate.
            stick = np.array(
                [-self._axes["left_y"], -self._axes["left_x"], -self._axes["right_x"]]
            )
        mapped = np.where(
            np.abs(stick) < self._deadband, 0.0, np.clip(stick, -1.0, 1.0) * self._limit
        )
        return mapped.astype(np.float32)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=0.5)
        try:
            os.close(self._fd)
        except OSError:
            pass


class ZeroCommand:
    """Keyboard-compatible source that always returns zero."""

    @staticmethod
    def command() -> np.ndarray:
        return np.zeros(3, dtype=np.float32)

    @staticmethod
    def close() -> None:
        pass


class OnnxPolicy:
    """Adapt ONNX Runtime to MJLab's viewer policy callback."""

    def __init__(self, path: Path, action_size: int, device: str) -> None:
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self._session = ort.InferenceSession(
            str(path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise RuntimeError("ONNX policy must have exactly one input and one output")
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        self._observation_size = int(inputs[0].shape[1])
        if list(outputs[0].shape) != [1, action_size]:
            raise RuntimeError(
                f"ONNX action shape {outputs[0].shape} does not match [1, {action_size}]"
            )
        self._device = device
        metadata = self._session.get_modelmeta().custom_metadata_map
        self._action_scale = float(metadata.get("action_scale", "0.5"))
        model_joints = metadata.get("joint_names", "").split(",")
        self._action_names = tuple(
            name for name in model_joints
            if name and not name.endswith(("_cap", "_actual"))
        )
        if len(self._action_names) != action_size:
            self._action_names = tuple(f"action_{i}" for i in range(action_size))

    @property
    def observation_size(self) -> int:
        return self._observation_size

    @property
    def action_scale(self) -> float:
        return self._action_scale

    @property
    def action_names(self) -> tuple[str, ...]:
        return self._action_names

    def __call__(self, observations: Any):
        import torch

        try:
            actor_observation = observations["actor"]
        except (KeyError, TypeError):
            actor_observation = observations
        array = np.asarray(
            actor_observation.detach().cpu().numpy(), dtype=np.float32
        )
        if list(array.shape) != [1, self._observation_size]:
            raise RuntimeError(
                f"MJLab actor observation shape {list(array.shape)} does not match "
                f"ONNX input [1, {self._observation_size}]"
            )
        action = self._session.run(
            [self._output_name], {self._input_name: array}
        )[0]
        # Copy because ONNX Runtime owns the returned NumPy storage.
        return torch.from_numpy(action.copy()).to(self._device)


def _install_command_override(
    env: Any,
    command_source: KeyboardCommand | GamepadCommand | ZeroCommand,
    print_period: float,
) -> None:
    """Overwrite MJLab's sampled twist after every command-manager update."""
    import torch

    term = env.command_manager.get_term("twist")
    original_compute: Callable[[float], None] = term.compute
    last_print = 0.0

    # The environment samples its first command during construction.  Replace
    # it before the first observation/policy call as well as after later
    # command-manager updates.
    initial_command = command_source.command()
    term.vel_command_b[0] = torch.as_tensor(
        initial_command, device=term.vel_command_b.device, dtype=term.vel_command_b.dtype
    )

    def compute(dt: float) -> None:
        nonlocal last_print
        original_compute(dt)
        command = command_source.command()
        term.vel_command_b[0] = torch.as_tensor(
            command, device=term.vel_command_b.device, dtype=term.vel_command_b.dtype
        )
        now = time.monotonic()
        if print_period > 0.0 and now - last_print >= print_period:
            print(
                f"\rcommand vx={command[0]:+6.3f}  vy={command[1]:+6.3f}  "
                f"wz={command[2]:+6.3f}",
                end="",
                flush=True,
            )
            last_print = now

    term.compute = compute


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--task", default="Mjlab-Velocity-Flat-KidRL-IRC",
        choices=("Mjlab-Velocity-Flat-KidRL-IRC", "Mjlab-Velocity-Flat-KidRL", "Mjlab-Velocity-KidRL"),
    )
    parser.add_argument(
        "--onnx", type=Path, default=Path(__file__).resolve().parents[1] / "models/policy.onnx"
    )
    parser.add_argument(
        "--zero-command",
        action="store_true",
        help="Disable keyboard input and hold vx, vy and wz at zero.",
    )
    parser.add_argument(
        "--input", choices=("gamepad", "keyboard"), default="gamepad",
        help="Command source (default: gamepad, mapped like the real controller).",
    )
    parser.add_argument(
        "--gamepad-device", default="",
        help="Event device to use, e.g. /dev/input/event17 (default: search like the robot does).",
    )
    parser.add_argument("--gamepad-name-hint", default="Gulikit X")
    parser.add_argument(
        "--deadband", type=float, default=0.1,
        help="Stick deadband for all three axes (controllers.yaml gamepad.deadband_*).",
    )
    parser.add_argument("--max-vx", type=float, default=0.4)
    parser.add_argument("--max-vy", type=float, default=0.3)
    parser.add_argument("--max-wz", type=float, default=0.7)
    parser.add_argument(
        "--key-step-ratio", type=float, default=0.5,
        help="Fraction of each maximum added per key press (default: 0.5).",
    )
    parser.add_argument("--device", default=None, help="MJLab device, e.g. cuda:0 or cpu")
    parser.add_argument("--viewer", choices=("native", "viser"), default="native")
    parser.add_argument(
        "--print-period", type=float, default=0.5,
        help="Seconds between command prints; set 0 to disable.",
    )
    parser.add_argument(
        "--diagnostic-steps", type=int, default=0,
        help=(
            "Run this many policy steps without a viewer and print initial-stability "
            "metrics (0 keeps the interactive viewer)."
        ),
    )
    args = parser.parse_args()
    limits = np.asarray((args.max_vx, args.max_vy, args.max_wz), dtype=float)
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
        parser.error("--max-vx, --max-vy and --max-wz must be finite and positive")
    if not np.isfinite(args.key_step_ratio) or not 0.0 < args.key_step_ratio <= 1.0:
        parser.error("--key-step-ratio must be in (0, 1]")
    if args.viewer != "native" and not args.zero_command and args.input == "keyboard":
        parser.error("keyboard control requires --viewer native")
    if not np.isfinite(args.deadband) or not 0.0 <= args.deadband < 1.0:
        parser.error("--deadband must be in [0, 1)")
    if args.diagnostic_steps < 0:
        parser.error("--diagnostic-steps must be >= 0")

    import torch

    # Importing tasks loads this project's mjlab.tasks entry point and registers KidRL.
    import mjlab.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
    from mjlab.utils.torch import configure_torch_backends
    from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer

    configure_torch_backends()
    device = args.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
    env_cfg = load_env_cfg(args.task, play=True)
    agent_cfg = load_rl_cfg(args.task)
    env_cfg.scene.num_envs = 1

    source: KeyboardCommand | GamepadCommand | ZeroCommand
    if args.zero_command:
        source = ZeroCommand()
    elif args.input == "gamepad":
        try:
            source = GamepadCommand(
                (args.max_vx, args.max_vy, args.max_wz),
                (args.deadband,) * 3,
                device_path=args.gamepad_device,
                name_hint=args.gamepad_name_hint,
            )
        except RuntimeError as error:
            raise SystemExit(f"gamepad: {error}") from error
    else:
        source = KeyboardCommand(
            args.max_vx, args.max_vy, args.max_wz, args.key_step_ratio
        )

    env = None
    try:
        env = ManagerBasedRlEnv(cfg=env_cfg, device=device)
        _install_command_override(env, source, args.print_period)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        policy = OnnxPolicy(args.onnx, wrapped.num_actions, device)

        actor_shape = tuple(wrapped.get_observations()["actor"].shape)
        expected_shape = (1, policy.observation_size)
        if actor_shape != expected_shape:
            raise RuntimeError(
                f"MJLab actor observation shape {actor_shape} does not match "
                f"ONNX input {expected_shape}"
            )

        print(f"ONNX     : {args.onnx}")
        print(f"task     : {args.task} (1 environment, original BAM pipeline)")
        print(f"device   : {device}")
        if isinstance(source, GamepadCommand):
            print(f"gamepad  : {source.device_name or '?'} ({source.device_path})")
            print(
                f"           left stick = vx/vy, right stick X = wz; max "
                f"{args.max_vx}/{args.max_vy}/{args.max_wz}, deadband {args.deadband}"
            )
        elif isinstance(source, KeyboardCommand):
            print("keyboard : W/S vx, A/D vy, J/K wz, SPACE stop")
        print(f"contract : observation {actor_shape}, action (1, {wrapped.num_actions})")

        if args.diagnostic_steps > 0:
            import torch

            observations = wrapped.get_observations()
            previous_target = None
            max_action = 0.0
            max_target_slew = 0.0
            max_action_by_joint = np.zeros(wrapped.num_actions, dtype=np.float32)
            max_slew_by_joint = np.zeros(wrapped.num_actions, dtype=np.float32)
            done_count = 0
            hip_roll_index = policy.action_names.index("right_hip_roll_crank")
            action_term = env.action_manager.get_term("joint_pos")
            joint_ids = action_term.target_ids
            joint_limits = (
                env.scene["robot"].data.joint_pos_limits[0, joint_ids]
                .detach().cpu().numpy()
            )
            default_positions = action_term.offset[0].detach().cpu().numpy()
            initial_height = float(env.scene["robot"].data.root_link_pos_w[0, 2])
            print(f"diagnostic: {args.diagnostic_steps} zero-command policy steps")
            print(f"step 0: root_z={initial_height:.4f}")
            with torch.inference_mode():
                for step in range(1, args.diagnostic_steps + 1):
                    action = policy(observations)
                    action_absmax = float(action.abs().max())
                    max_action = max(max_action, action_absmax)
                    action_array = action.detach().cpu().numpy()[0]
                    max_action_by_joint = np.maximum(max_action_by_joint, np.abs(action_array))
                    # Mirror the deployment controller's target construction,
                    # including its physical joint-limit clamp.
                    target_array = np.clip(
                        default_positions + policy.action_scale * action_array,
                        joint_limits[:, 0], joint_limits[:, 1],
                    )
                    if previous_target is not None:
                        slew_array = np.abs(target_array - previous_target) / env.step_dt
                        target_slew = float(slew_array.max())
                        max_slew_by_joint = np.maximum(max_slew_by_joint, slew_array)
                        max_target_slew = max(max_target_slew, target_slew)
                    else:
                        target_slew = 0.0
                    observations, _, dones, _ = wrapped.step(action)
                    done_count += int(dones.sum())
                    root_z = float(env.scene["robot"].data.root_link_pos_w[0, 2])
                    if step <= 10 or step % 25 == 0 or bool(dones.any()):
                        print(
                            f"step {step:4d}: root_z={root_z:.4f} "
                            f"action_absmax={action_absmax:.3f} "
                            f"target_slew_max={target_slew:.1f} rad/s "
                            f"right_hip_roll_action={action_array[hip_roll_index]:+.3f} "
                            f"done={int(dones.sum())}"
                        )
                    previous_target = target_array.copy()
            print(
                f"summary: max_action={max_action:.3f} "
                f"max_target_slew={max_target_slew:.1f} rad/s "
                f"terminations={done_count}"
            )
            action_rank = np.argsort(max_action_by_joint)[::-1][:5]
            slew_rank = np.argsort(max_slew_by_joint)[::-1][:5]
            print("largest raw actions:")
            for index in action_rank:
                print(f"  {policy.action_names[index]}: {max_action_by_joint[index]:.3f}")
            print("largest target slews:")
            for index in slew_rank:
                print(f"  {policy.action_names[index]}: {max_slew_by_joint[index]:.1f} rad/s")
        elif args.viewer == "native":
            class KeyboardMujocoViewer(NativeMujocoViewer):
                def _safe_key_callback(self, key: int) -> None:
                    # A and SPACE are native viewer shortcuts. Consume command
                    # keys first so lateral motion does not toggle the scene and
                    # stopping does not pause the simulation.
                    if isinstance(source, KeyboardCommand) and source.handles_key(key):
                        source.on_key(key)
                        return
                    super()._safe_key_callback(key)

            KeyboardMujocoViewer(wrapped, policy).run()
        else:
            ViserPlayViewer(wrapped, policy).run()
    finally:
        print()
        source.close()
        if env is not None:
            env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
