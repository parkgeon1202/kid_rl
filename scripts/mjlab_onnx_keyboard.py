#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""Run the deployment ONNX policy inside MJLab and drive it from the keyboard.

Same as mjlab_onnx_joystick.py with --input keyboard as the default, so the two
command sources can be started separately. Every option of that script works here.

Keys (in the MuJoCo viewer window): W/S = vx, A/D = vy, J/K = wz, SPACE = stop.
Each press adds --key-step-ratio of the maximum (default half) and the value is held.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mjlab_onnx_joystick import main  # noqa: E402

if __name__ == "__main__":
    if not any(arg == "--input" or arg.startswith("--input=") for arg in sys.argv[1:]):
        sys.argv[1:1] = ["--input", "keyboard"]
    raise SystemExit(main())
