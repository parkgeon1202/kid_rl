#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""서보 한 개의 명령 반응 지연을 계단 입력(step)으로 잰다.

평상시 제어 틱에서는 '서보가 새 Goal Position 을 받아 실제로 반응하기까지'를 볼 수
없다. 이 스크립트는 관절 하나에 작은 계단 목표를 주고, 그 뒤 서보의 Present PWM
(모터에 거는 전압 비율)과 Present Position 을 최대한 빠르게 다시 읽어서 언제
반응이 시작되는지 잰다.

  ack_ms       Goal Position 쓰기 → 서보의 상태 패킷(수신 확인) 도착. 왕복 통신 시간.
  pwm_react_ms 수신 확인 → PWM 이 움직이기 시작. 서보 내부 제어기가 새 목표를
               집어 들기까지의 시간(= 학습 BAM 명령 지연에 대응하는 값).
  pos_react_ms 수신 확인 → 엔코더가 실제로 움직임. 모터·기어의 기계적 반응까지 포함.
  poll_rt_ms   한 번 읽는 왕복 시간 = 위 두 반응 시간의 측정 분해능.

주의
- 관절이 실제로 움직인다(기본 0.05 rad ≈ 2.9도). 로봇을 받쳐 두고, 가볍고 걸리는 데
  없는 관절(손목, 머리 등)로 한다.
- 컨트롤 노드(ros2_control_node)를 끄고 실행한다. 포트를 쥐고 있으면 열리지 않는다.
- 끝나면 목표를 원래 위치로 돌리고 토크 상태도 원래대로 되돌린다.

예) python3 servo_step_latency.py --id 8 --trials 10
"""

import argparse
import statistics
import sys
import time

import serial

_T = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else (_c << 1) & 0xFFFF
    _T.append(_c)


def crc16(data):
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _T[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_PWM = 124       # 124 PWM(2) 126 Load/Current(2) 128 Velocity(4) 132 Position(4)
STATUS_BLOCK_LEN = 12


class Bus:
    def __init__(self, port, baud):
        self.port = serial.Serial(port, baud, timeout=0.0005)

    def _txrx(self, dxl_id, inst, params, timeout=0.02):
        body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id,
                      (len(params) + 3) & 0xFF, (len(params) + 3) >> 8, inst]) + params
        self.port.reset_input_buffer()
        t_tx = time.perf_counter()
        self.port.write(body + crc16(body).to_bytes(2, "little"))
        buf = b""
        deadline = t_tx + timeout
        while time.perf_counter() < deadline:
            buf += self.port.read(64)
            j = buf.find(b"\xff\xff\xfd\x00")
            if j >= 0 and len(buf) >= j + 7:
                ln = buf[j + 5] | (buf[j + 6] << 8)
                if len(buf) >= j + 7 + ln:
                    pkt = buf[j:j + 7 + ln]
                    t_rx = time.perf_counter()
                    if crc16(pkt[:-2]) != int.from_bytes(pkt[-2:], "little"):
                        raise IOError(f"ID {dxl_id}: CRC error")
                    # pkt[8] 은 에러 바이트. 0x80(alert) 은 하드웨어 에러가 걸려 있다는
                    # 표시일 뿐 이번 명령의 실패가 아니므로 하위 7비트만 본다.
                    if pkt[8] & 0x7F:
                        raise IOError(f"ID {dxl_id}: status error 0x{pkt[8]:02x}")
                    return pkt[9:-2], t_tx, t_rx
        raise IOError(f"ID {dxl_id}: no status packet")

    def read(self, dxl_id, addr, length):
        data, t_tx, t_rx = self._txrx(
            dxl_id, 0x02, bytes([addr & 0xFF, addr >> 8, length & 0xFF, length >> 8]))
        return data, t_tx, t_rx

    def write(self, dxl_id, addr, value, size):
        params = bytes([addr & 0xFF, addr >> 8]) + int(value).to_bytes(size, "little", signed=True)
        _, t_tx, t_rx = self._txrx(dxl_id, 0x03, params)
        return t_tx, t_rx

    def status(self, dxl_id):
        data, t_tx, t_rx = self.read(dxl_id, ADDR_PRESENT_PWM, STATUS_BLOCK_LEN)
        pwm = int.from_bytes(data[0:2], "little", signed=True)
        pos = int.from_bytes(data[8:12], "little", signed=True)
        return pwm, pos, t_tx, t_rx


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--port", default="/dev/ttyUSB-U2D2")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--step-rad", type=float, default=0.05)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--pwm-threshold", type=int, default=15,
                    help="기준 PWM 에서 이만큼 벗어나면 반응 시작으로 본다")
    ap.add_argument("--pos-threshold", type=int, default=2, help="엔코더 틱")
    args = ap.parse_args()

    if not 0.005 <= args.step_rad <= 0.2:
        sys.exit("--step-rad 는 0.005~0.2 rad 사이로 준다")
    step_ticks = max(1, round(args.step_rad / 0.0015339807878856412))

    bus = Bus(args.port, args.baud)
    dxl = args.id
    mode = bus.read(dxl, ADDR_OPERATING_MODE, 1)[0][0]
    if mode != 3:
        sys.exit(f"ID {dxl}: Operating Mode 가 {mode} 이다. 3(위치 제어)일 때만 실행한다.")
    torque_was_on = bus.read(dxl, ADDR_TORQUE_ENABLE, 1)[0][0] == 1
    _, home, _, _ = bus.status(dxl)

    results = []
    try:
        bus.write(dxl, ADDR_GOAL_POSITION, home, 4)   # 토크를 켜기 전에 목표 = 현재
        if not torque_was_on:
            bus.write(dxl, ADDR_TORQUE_ENABLE, 1, 1)
        time.sleep(0.5)

        for trial in range(args.trials):
            sign = 1 if trial % 2 == 0 else -1
            time.sleep(0.4)                           # 이전 움직임이 가라앉을 때까지
            base = [bus.status(dxl) for _ in range(5)]
            pwm0 = statistics.median(b[0] for b in base)
            pos0 = base[-1][1]
            target = home + sign * step_ticks

            t_cmd, t_ack = bus.write(dxl, ADDR_GOAL_POSITION, target, 4)
            t_pwm = t_pos = None
            rts = []
            deadline = t_ack + 0.3
            while time.perf_counter() < deadline and (t_pwm is None or t_pos is None):
                pwm, pos, t_tx, t_rx = bus.status(dxl)
                rts.append((t_rx - t_tx) * 1e3)
                # 값은 요청을 보낸 시각과 응답을 받은 시각 사이 어딘가에서 찍힌 것이다.
                # 가운데로 잡고, 오차는 poll_rt_ms 의 절반이다.
                t_sample = 0.5 * (t_tx + t_rx)
                if t_pwm is None and abs(pwm - pwm0) >= args.pwm_threshold:
                    t_pwm = t_sample
                if t_pos is None and abs(pos - pos0) >= args.pos_threshold:
                    t_pos = t_sample
            row = {
                "ack_ms": (t_ack - t_cmd) * 1e3,
                "pwm_react_ms": (t_pwm - t_ack) * 1e3 if t_pwm else float("nan"),
                "pos_react_ms": (t_pos - t_ack) * 1e3 if t_pos else float("nan"),
                "poll_rt_ms": statistics.median(rts) if rts else float("nan"),
            }
            results.append(row)
            print(f"trial {trial + 1:2d} ({'+' if sign > 0 else '-'}{step_ticks} ticks): " +
                  "  ".join(f"{k}={v:6.2f}" for k, v in row.items()))
    finally:
        try:
            bus.write(dxl, ADDR_GOAL_POSITION, home, 4)
            time.sleep(0.5)
            if not torque_was_on:
                bus.write(dxl, ADDR_TORQUE_ENABLE, 0, 1)
        except IOError as error:
            print(f"WARNING: could not restore ID {dxl}: {error}", file=sys.stderr)

    print("\n중앙값 (NaN 은 0.3 s 안에 반응이 안 보인 시행)")
    for key in ("ack_ms", "pwm_react_ms", "pos_react_ms", "poll_rt_ms"):
        vals = [r[key] for r in results if r[key] == r[key]]
        if vals:
            print(f"  {key:13s} median {statistics.median(vals):6.2f}  "
                  f"min {min(vals):6.2f}  max {max(vals):6.2f}  (n={len(vals)})")


if __name__ == "__main__":
    main()
