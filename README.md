# kid_rl

시뮬레이션에서 학습한 보행 정책을 ROS 2 `ros2_control` 위에서 실행하는 컨트롤러.

기본 모델은 `models/policy.onnx`이며 C++ ONNX Runtime으로 실행한다. configure 때
입력 `[1,84]`, 출력 `[1,25]`, float 타입, 관절 순서를 검증하고 워밍업 추론까지
성공해야 컨트롤러가 활성화될 수 있다.

목표 주기는 **하드웨어 200 Hz / 정책 50 Hz**다. 컨트롤러는 controller_manager
주기(200 Hz)로 돌면서 정책만 50 Hz로 decimation하고, 그 사이 틱에서는 마지막
정책 목표를 그대로 유지한다. 200 Hz는 아직 **목표이지 실측값이 아니며**, 시스템
설정(rtprio, FTDI latency timer, baud ≥ 3 Mbps)이 선행되어야 한다 —
[`docs/THREADING.md`](docs/THREADING.md).

## 문서

| 문서 | 내용 |
| --- | --- |
| [`docs/STUDY.md`](docs/STUDY.md) | **처음이면 여기부터.** 배경지식(정책·관측·주기·실시간·ONNX)과 microban / ai_sapiens / roa_controller 세 구현의 해부·비교 |
| [`docs/CONTROLLER_GUIDE.md`](docs/CONTROLLER_GUIDE.md) | 이 패키지의 코드 설명. ros2_control, 정책 계약, 주기 설계, 안전 설계, 파라미터 레퍼런스, 브링업 절차, 디버깅 |
| [`docs/THREADING.md`](docs/THREADING.md) | 스레딩·실시간·400 Hz. RT 설정, ONNX 스레드, DYNAMIXEL 버스 예산, 브링업 체크리스트 |
| [`docs/WORKLOG.md`](docs/WORKLOG.md) | 작업 이력, 리뷰에서 고친 것, 남은 할 일 |
| [`docs/DEPLOYMENT_AUDIT_2026-09-18.md`](docs/DEPLOYMENT_AUDIT_2026-09-18.md) | **실물 배포 전 필독.** 모델·모터·launch·FAULT 감사 결과와 실측 승인표 |
| [`CLAUDE_COLLABORATION_AND_DOCUMENTATION.md`](CLAUDE_COLLABORATION_AND_DOCUMENTATION.md) | Codex가 작성한 인계 문서. 위 둘과 내용이 겹친다 (통합 미결정) |

## 입력과 ROS 인터페이스

| 종류 | 이름 | 타입 |
| --- | --- | --- |
| 직접 입력 | `/dev/input/event*` | 게임패드 `vx`, `vy`, `wz` |
| 직접 입력 | `/dev/tty*` | EBIMU ASCII 프레임 |
| 구독 | `~/enable_policy` | `std_msgs/msg/Bool` |
| state interface | `<joint>/position`, `<joint>/velocity`, `<joint>/hardware_state` | 25관절 |
| command interface | `<joint>/position` | 25관절 |

## 운전 상태

```
WAITING_FOR_STATE → READY_POSE → SAFE_HOLD ⇄ POLICY_RUNNING
                                     ↑              │
                                     └─── FAULT ◄───┘
                                    (latched, 재활성화로만 탈출)
```

`POLICY_RUNNING` 진입 조건: `enable_policy=true`, IMU 신선, 직립,
모든 관절이 `default_positions`에서 `entry_position_tolerance` 이내.
IMU stale / 낙상(debounce 후) / 비유한 action → `FAULT`.
게임패드 명령 stale은 fault가 아니라 **속도 명령 0**으로 처리한다
([이유](docs/CONTROLLER_GUIDE.md#62-watchdog--imu와-cmd_vel을-다르게-다루는-이유)).

## 빌드

```bash
cd ~/deploy_ws
colcon build --packages-select kid_rl --symlink-install \
  --cmake-args -DONNXRUNTIME_ROOT=$HOME/opt/onnxruntime/current
source install/setup.bash
```

`ONNXRUNTIME_ROOT` 아래에는 `include/onnxruntime_cxx_api.h`와
`lib/libonnxruntime.so`가 있어야 한다. 다른 위치에 설치했다면 그 경로를 넘긴다.

## 하드웨어 없는 시험의 현재 상태

기존 `test/smoke_test.sh`와 mock URDF는 구형 18축·ROS topic IMU 기반이라 현재 25축·직접
serial IMU 제어기를 검증하지 못한다. 새 mock을 만들기 전에는 이 시험을 성공 근거로 사용하지
않는다. 자세한 내용은 배포 감사 보고서를 본다.

## 실기에 올리기 전에

`description/kid_dynamixel.ros2_control.xacro`는 include용 매크로다. 로봇 URDF에서
호출해야 하며, **포트 / baud rate / Drive Mode 부호 / 관절 한계는 하드웨어에서
하나씩 검증한 뒤** 사용해야 한다. 절차는
[`docs/CONTROLLER_GUIDE.md` 10절](docs/CONTROLLER_GUIDE.md#10-브링업-절차).

현재 `description/kid_rl.urdf.xacro`는 ros2_control 파싱용 제어 스키마이며 실제 TF 모델이
아니다. launch는 이 더미에서 동적 TF가 발행되지 않도록 막는다.
