#!/usr/bin/env python3
"""Map Linux gamepad A/B presses to the KidRL lifecycle and enable topic."""

from __future__ import annotations

import argparse
import os
import select
import struct
import subprocess
import sys
import time
from pathlib import Path


EV_KEY = 0x01

# Physical labels reported by the controller currently used on the robot.
# They do not match Linux's geometric BTN_* names, so keep this measured map:
#   physical A (east)  -> code 305 / 0x131 / Linux BTN_EAST
#   physical B (south) -> code 304 / 0x130 / Linux BTN_SOUTH
#   physical Y (west)  -> code 307 / 0x133 / Linux BTN_NORTH
#   physical X (north) -> code 308 / 0x134 / Linux BTN_WEST
BUTTON_A = 0x131
BUTTON_B = 0x130
BUTTON_Y = 0x133
BUTTON_X = 0x134
INPUT_EVENT = struct.Struct("llHHi")


def key_name(code: int) -> str:
    if code == BUTTON_A:
        return "physical-A/BTN_EAST"
    if code == BUTTON_B:
        return "physical-B/BTN_SOUTH"
    if code == BUTTON_Y:
        return "physical-Y/BTN_NORTH"
    if code == BUTTON_X:
        return "physical-X/BTN_WEST"
    return "unmapped"


def _read_hex(path: Path) -> int | None:
    try:
        return int(path.read_text().strip(), 16)
    except (OSError, ValueError):
        return None


def find_device(device: str, name_hint: str, vendor: int, product: int) -> Path | None:
    if device:
        path = Path(device)
        return path if path.exists() else None

    name_fallback = None
    for event in sorted(Path("/sys/class/input").glob("event*")):
        dev = event / "device"
        try:
            name = (dev / "name").read_text().strip()
        except OSError:
            name = ""
        candidate = Path("/dev/input") / event.name
        if _read_hex(dev / "id/vendor") == vendor and _read_hex(dev / "id/product") == product:
            return candidate
        if name_fallback is None and name_hint and name_hint in name:
            name_fallback = candidate
    return name_fallback


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="")
    parser.add_argument("--name-hint", default="Gulikit X")
    parser.add_argument("--vendor-id", type=lambda value: int(value, 0), default=0x045E)
    parser.add_argument("--product-id", type=lambda value: int(value, 0), default=0x028E)
    parser.add_argument(
        "--log-only",
        action="store_true",
        help="print EV_KEY code/value events without changing controller or policy state",
    )
    args, _ = parser.parse_known_args()

    helper = Path(__file__).resolve().with_name("kid_rl_policy.sh")
    if not helper.exists():
        print(f"gamepad supervisor: helper not found: {helper}", file=sys.stderr, flush=True)
        return 1

    while True:
        device = find_device(args.device, args.name_hint, args.vendor_id, args.product_id)
        if device is None:
            print("gamepad supervisor: waiting for gamepad", flush=True)
            time.sleep(1.0)
            continue
        try:
            fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        except OSError as exc:
            print(f"gamepad supervisor: cannot open {device}: {exc}", file=sys.stderr, flush=True)
            time.sleep(1.0)
            continue

        print(f"gamepad supervisor: reading A/B from {device}", flush=True)
        try:
            while True:
                readable, _, _ = select.select([fd], [], [], 1.0)
                if not readable:
                    continue
                data = os.read(fd, INPUT_EVENT.size)
                if len(data) != INPUT_EVENT.size:
                    break
                _, _, event_type, code, value = INPUT_EVENT.unpack(data)
                if event_type != EV_KEY:
                    continue

                # Print every key transition so the controller's actual A/B
                # codes can be read directly. value: 0=release, 1=press,
                # 2=autorepeat.
                print(
                    "gamepad key event: "
                    f"code={code} (0x{code:03x}, {key_name(code)}) value={value}",
                    flush=True,
                )
                if args.log_only or value != 1:
                    continue
                if code == BUTTON_A:
                    print("gamepad A: activating/resetting kid_rl, then enabling policy", flush=True)
                    subprocess.run([str(helper), "enable"], check=False)
                elif code == BUTTON_B:
                    print("gamepad B: disabling policy", flush=True)
                    subprocess.run([str(helper), "disable"], check=False)
        except OSError as exc:
            print(f"gamepad supervisor: {device} disconnected: {exc}", file=sys.stderr, flush=True)
        finally:
            os.close(fd)
        time.sleep(1.0)


if __name__ == "__main__":
    raise SystemExit(main())
