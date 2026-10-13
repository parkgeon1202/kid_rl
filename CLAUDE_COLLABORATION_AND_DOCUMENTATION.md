# `kid_rl` 개발·협업·기술 문서

> **주의: 이 문서는 과거 18축 skeleton(기능 뼈대) 단계의 작업 기록이며 현재 실행 계약이
> 아니다.** ONNX backend, 25축 관절, 직접 장치 IMU, FAULT 복구 등 일부 설명이 현재 코드와
> 다르다. 실물 배포 판단에는
> [`docs/DEPLOYMENT_AUDIT_2026-09-18.md`](docs/DEPLOYMENT_AUDIT_2026-09-18.md)를 사용한다.

이 문서는 Claude와 Codex가 같은 패키지를 이어서 개선할 때 사용하는 인계 문서다.
현재 구현, 검증된 사실, 남은 일, 설계 배경을 한곳에 기록한다. 코드를 바꾼 사람은 작업을
끝내기 전에 아래 작업 기록과 검증 결과를 함께 갱신한다.

> **문서 지도.** 이 패키지에는 문서가 셋 있고 겹치는 부분이 있다. 통합 여부는 아직
> 결정되지 않았다.
>
> | 문서 | 성격 |
> | --- | --- |
> | `CLAUDE_COLLABORATION_AND_DOCUMENTATION.md` (이 문서) | Codex 작성. 협업 규약 + 배경지식 + 할 일 |
> | [`docs/CONTROLLER_GUIDE.md`](docs/CONTROLLER_GUIDE.md) | Claude 작성. 배경지식 심화 + 파라미터 레퍼런스 + 브링업/디버깅 절차 |
> | [`docs/WORKLOG.md`](docs/WORKLOG.md) | Claude 작성. 리뷰에서 고친 버그의 증상/원인/수정과 우선순위별 할 일 |

> 안전 상태: 이 패키지는 아직 실제 보행 정책을 실행하지 않는다. `dry_run_policy=true`에서
> action을 모두 0으로 만드는 골격이다. 실제 로봇에서 보행 가능한 상태로 오해하면 안 된다.

## 1. 현재 목표와 범위

`kid_rl`은 ROS 2 Jazzy의 `ros2_control` controller plugin으로 동작하며, 궁극적으로는 학습된
ONNX 보행 정책을 18개 DYNAMIXEL 관절의 position command로 변환한다.

현재 구현된 범위는 다음과 같다.

- lifecycle 기반 configure/activate/deactivate
- activation 자세에서 기본 자세까지 부드러운 보간
- 관절 position/velocity와 IMU, 속도 명령으로 observation 구성
- controller update 주기와 50 Hz policy 주기 분리
- action clip, scale, 저역통과필터, 위치 변화율 제한, 관절 위치 제한
- 명령/IMU watchdog, 자세 기반 낙상 감지, `SAFE_HOLD`/`FAULT`
- 실제 추론 대신 zero-action dry run

아직 구현되지 않은 범위는 ONNX Runtime, 모델 metadata 계약 검사, 실제 모델별 observation
정규화, IMU frame의 로봇별 검증, fault reset 절차, 자동 시험이다.

## 2. Claude와 Codex의 작업 방식

### 작업을 시작할 때

1. 이 문서의 **현재 상태**, **결정 기록**, **남은 작업**을 읽는다.
2. `git diff` 또는 파일 변경 시각으로 다른 작업자의 미완료 변경을 확인한다.
3. 수정하려는 정책 계약을 먼저 적는다. 특히 joint 순서, 단위, frame, tensor shape는 추측하지 않는다.
4. 실제 하드웨어가 필요한 사실과 코드만으로 검증 가능한 사실을 구분한다.

### 작업을 끝낼 때

1. 변경 이유와 안전 영향이 드러나는 작은 단위로 코드를 수정한다.
2. 최소한 `colcon build --packages-select kid_rl`을 실행한다.
3. 설정/URDF 변경이면 xacro 확장과 controller interface 이름도 검사한다.
4. 아래 작업 기록에 날짜, 작업자, 변경, 검증, 미해결 위험을 남긴다.
5. 실제 로봇에서 확인하지 않은 항목을 “완료”로 표시하지 않는다.

### 역할을 나눌 때 권장 경계

- 한 작업자는 `PolicyContract`와 ONNX 입출력 검증을 맡는다.
- 다른 작업자는 controller state machine과 안전 시험을 맡는다.
- 하드웨어 mapping 작업은 motor ID, 방향, zero offset을 실물에서 확인할 수 있는 사람이 맡는다.
- 같은 파일을 동시에 고치기보다는 먼저 이 문서에 담당 범위를 기록한다.

## 3. 작업 기록

### 2026-08-31 — Claude가 작성한 초기 구현 검토

확인된 초기 구현:

- 18관절 position controller plugin 및 pluginlib 등록
- READY_POSE → SAFE_HOLD → POLICY_RUNNING/FAULT 상태 흐름
- 50 Hz observation과 zero-action dry policy
- IMU/`cmd_vel` freshness, 낙상 판정, action 안전 처리
- DYNAMIXEL용 ros2_control xacro와 controller YAML

검토 중 발견한 위험:

- `cmd_vel`에 `NaN` 또는 `Inf`가 있어도 freshness만 만족하면 정책 입력으로 들어갈 수 있었다.
- callback 시각이 update 시각보다 미래인 경우 음수 age가 timeout을 통과할 수 있었다.
- 중복/빈 관절명, 비유한 매개변수, 잘못된 filter 계수와 관절 한계를 configure에서 거부하지 않았다.
- ONNX backend가 없으므로 `dry_run_policy=false`도 실제 추론을 수행하지 않는다.
- 자동 시험이 없어 observation 순서와 상태 전이를 회귀 검증하지 못한다.

### 2026-08-31 — Codex 안전성 보강

변경:

- `CommandSample.valid`를 추가하고 `vx`, `vy`, `yaw_rate`가 모두 유한할 때만 유효하게 처리했다.
- watchdog age가 `0 <= age <= timeout`일 때만 fresh가 되게 했다.
- 모든 수치 설정의 유한성 및 유효 범위를 configure 단계에서 검사한다.
- 관절 이름의 빈 문자열/중복, `min >= max`, 기본 자세의 limit 이탈을 거부한다.
- Claude의 후속 변경과 통합해 interface를 이름으로 매핑하고, 낙상 debounce와 policy 진입
  자세 검사를 적용했다. `dry_run_policy=false`인데 backend가 없으면 configure를 거부한다.

검증 결과는 이 문서의 **검증 이력**에 기록한다.

## 4. 코드 구조와 데이터 흐름

```text
controller_manager
  ├─ hardware.read() -> joint position/velocity
  ├─ KidRlController.update()
  │    ├─ 입력·관절 상태 검증
  │    ├─ observation 구성 (policy 주기에만)
  │    ├─ policy 실행 (현재는 zero action)
  │    ├─ action 안전 처리
  │    └─ position command 기록
  └─ hardware.write() -> DYNAMIXEL
```

파일별 책임:

- `include/kid_rl/kid_rl_controller.hpp`: controller 상태와 인터페이스 선언
- `src/kid_rl_controller.cpp`: lifecycle, 입력 callback, state machine, policy loop, command 출력
- `config/controllers.yaml`: controller manager 주기와 정책/안전 매개변수
- `description/kid_dynamixel.ros2_control.xacro`: joint–motor ID–transmission 연결
- `kid_rl_plugin.xml`: pluginlib에서 C++ class를 찾기 위한 등록 정보
- `CMakeLists.txt`, `package.xml`: 빌드, 의존성, 설치/export 계약

## 5. 꼭 알아야 할 배경지식

### 5.1 `ros2_control`의 한 주기

`controller_manager`는 보통 다음 세 단계를 반복한다.

1. hardware `read`: 모터 값을 ROS joint state 단위로 읽는다.
2. controller `update`: state를 읽고 다음 command를 계산한다.
3. hardware `write`: command를 모터 레지스터 값으로 변환해 보낸다.

이 패키지의 manager 주기는 200 Hz, 즉 이상적으로 5 ms마다 한 번이다. 정책은 50 Hz, 즉
20 ms마다 한 번만 실행한다. 정책 사이의 세 번의 update에서는 마지막 target을 계속 쓴다.
이런 **다중 주기 구조**는 느리고 계산량이 큰 신경망을 매 hardware 주기마다 실행하지 않으면서
모터 command 갱신은 빠르게 유지하기 위해 필요하다.

실제 주기는 항상 정확히 5 ms가 아니다. 그래서 “update 네 번마다 추론”처럼 횟수만 세지 않고
`period.seconds()`를 누적한다. 긴 지연 뒤 밀린 추론을 여러 번 연속 실행하지 않고 한 번 실행한 뒤
나머지만 보존한다. 실시간 loop의 연쇄 지연을 막기 위한 선택이다.

### 5.2 lifecycle과 내부 운전 상태는 다르다

ROS lifecycle은 controller 자원의 수명이다.

- `on_init`: parameter를 선언한다.
- `on_configure`: 설정을 검증하고 subscription과 버퍼를 준비한다.
- `on_activate`: hardware interface가 대여된 뒤 현재 관절 자세를 읽는다.
- `update`: 활성 상태에서 매 주기 실행된다.
- `on_deactivate`: command 생성을 멈출 준비를 한다.

내부 `Mode`는 활성화된 controller 안에서 로봇이 무엇을 하고 있는지를 뜻한다.

```text
activate
  -> READY_POSE
  -> SAFE_HOLD  <──── enable=false ────┐
       └─ 진입 게이트 통과 -> POLICY_RUNNING
                                 └─ IMU stale / 낙상(debounce 후) /
                                    비유한 action / 잘못된 joint state -> FAULT

  * stale `cmd_vel`은 FAULT가 아니다. 속도 명령을 0으로 대체한다.
  * FAULT는 latched. deactivate -> activate 로만 빠져나온다.
```

`SAFE_HOLD`는 기본 자세 command를 유지하는 운전 상태다. `FAULT`는 오류가 발생한 순간의 측정
자세를 target으로 잡는다. 하지만 position hold가 모든 고장에서 안전한 것은 아니다. 넘어진
로봇에 강한 position gain을 유지하면 모터나 링크에 큰 힘이 걸릴 수 있으므로, 최종 시스템에서는
하드웨어가 지원하는 damping 또는 torque-off 정책을 별도로 설계해야 한다.

### 5.3 command/state interface

controller가 요청하는 interface는 관절마다 다음과 같다.

```text
입력 state shape:  joint 수 N × (position 1 + velocity 1) = 2N
연산 대상:         각 관절의 현재 위치와 속도
출력 command shape: joint 수 N × position 1 = N
```

현재 N=18이므로 state interface는 36개, command interface는 18개다. 이름은 예를 들어
`right_knee/position`, `right_knee/velocity` 형식이다. YAML의 joint 순서는 모델 학습 순서와
정확히 같아야 한다. 값의 개수가 같더라도 순서가 다르면 정책은 다른 관절을 제어한다.

### 5.4 pluginlib가 필요한 이유

controller manager는 실행 시점에 문자열 `kid_rl/KidRlController`를 보고 shared library에서
C++ class를 동적으로 불러온다. 이 연결에는 세 부분이 모두 일치해야 한다.

- C++의 `PLUGINLIB_EXPORT_CLASS`
- `kid_rl_plugin.xml`의 class/type/base class
- `CMakeLists.txt`의 plugin description export와 library 설치

하나라도 틀리면 컴파일은 성공해도 controller load 단계에서 실패할 수 있다.

### 5.5 realtime buffer

ROS subscription callback과 controller update는 서로 다른 thread에서 실행될 수 있다. callback이
쓰는 도중 update가 같은 객체를 읽으면 데이터 경합이 생긴다. `RealtimeBuffer`는 callback에서
완성된 sample을 교체하고 update에서는 일관된 sample을 읽게 해 이 경계를 단순화한다.

여기서 “realtime”은 모든 코드가 자동으로 deadline을 보장한다는 뜻이 아니다. update 경로에서
동적 메모리 할당, 파일 I/O, 긴 logging, mutex 대기, 모델 session 생성이 없어야 한다. ONNX
session과 tensor memory는 configure 때 만들고 update에서는 이미 할당된 buffer만 재사용해야 한다.

### 5.6 observation 계약과 shape

현재 한 시점의 observation 순서는 고정되어 있다.

```text
입력 shape
  body angular velocity:        3
  projected gravity:            3
  joint position error:         N
  joint velocity:               N
  previous action:              N
  velocity command:             3
  optional gait phase:          2

연산 대상 축
  위 term들을 feature 축 하나로 이어 붙임

출력 shape
  gait phase 없음: 9 + 3N
  gait phase 있음: 11 + 3N
  N=18이면 각각 63 또는 65
```

모델이 batch 축을 요구하면 ONNX 입력은 보통 `[1, 63]` 또는 `[1, 65]`가 된다. 현재 vector는
feature만 저장하므로 backend에서 batch 축을 추가해야 한다.

shape만 같다고 계약이 맞는 것은 아니다. term 순서, 관절 순서, 단위, scale, clip, frame,
history 길이가 전부 학습 환경과 같아야 한다. 예를 들어 학습 때 joint velocity에 0.05를 곱했는데
배포에서 원값을 넣으면 shape는 같지만 입력 분포가 완전히 달라진다.

### 5.7 좌표계, quaternion, projected gravity

IMU orientation은 quaternion `(x, y, z, w)`로 들어온다. quaternion은 회전을 표현하지만 센서나
수치 오차 때문에 길이가 정확히 1이 아닐 수 있어 먼저 정규화한다. 현재 코드는 그 회전으로 세계의
중력 방향을 body frame에 투영한다.

정상적으로 선 로봇에서 현재 계약의 gravity z는 약 -1이다. 몸체가 90도 기울면 z는 약 0이 된다.
따라서 `projected_gravity.z > -0.5`이면 약 60도보다 크게 기운 상태로 보아 fault로 전환한다.

이 해석은 IMU가 반환하는 orientation의 frame convention과 장착 방향이 기대와 같을 때만 맞다.
실물에서 반드시 다음을 측정해야 한다.

- 똑바로 섰을 때 `(gx, gy, gz)`가 대략 `(0, 0, -1)`인가
- 앞/옆으로 기울일 때 어느 축의 부호가 바뀌는가
- angular velocity 단위가 rad/s인가
- IMU frame과 학습에 사용한 body frame 사이에 고정 회전이 필요한가

### 5.8 watchdog과 시간

watchdog은 마지막 유효 sample이 너무 오래되었는지 검사한다.

```text
age = 현재 controller 시각 - callback 수신 시각
fresh 조건 = 0 <= age <= timeout
```

`cmd_vel` timeout은 0.25 s, IMU timeout은 0.10 s다. 단순히 `age <= timeout`만 검사하면 clock
점프나 서로 다른 clock 때문에 미래 시각이 들어왔을 때 음수 age도 통과한다. 현재 구현은 음수
age를 거부한다. 실행 중 stale `cmd_vel`은 정지 명령으로 대체하고, stale IMU는 대체할 수 없으므로
FAULT로 전환한다. ROS simulation time을 쓸 때도 publisher/controller의 `/clock` 설정을 일치시켜야 한다.

현재 timestamp는 메시지 header가 아니라 callback 수신 시각이다. 이는 통신 후 로컬 freshness는
측정하지만 센서 내부에서 이미 지연된 데이터인지는 알 수 없다. 향후 IMU header stamp를 사용할
경우 clock domain과 transport delay 정책을 명시해야 한다.

### 5.9 action에서 관절 target까지

정책 출력 `a`는 관절별로 다음 단계를 지난다.

1. action clip: `a = clamp(a, -action_clip, +action_clip)`
2. position 변환: `raw = default_position + action_scale × a`
3. joint limit: `raw`를 `[position_min, position_max]`로 제한
4. 저역통과필터: `filtered = old + alpha × (raw - old)`
5. 변화율 제한: 한 policy step의 변화가 `target_rate_limit × 0.02 s`를 넘지 않게 제한

작은 예시: 기존 target이 0.0 rad, raw가 1.0 rad, alpha가 0.2이면 filtered는 0.2 rad다.
변화율 한계가 4 rad/s이고 policy 주기가 0.02 s이면 이번 step 최대 변화는 0.08 rad이므로 최종
target은 0.08 rad가 된다.

filter와 rate limit은 급격한 command를 줄일 뿐 안정성을 증명하지 않는다. motor gain, 링크 관성,
통신 지연, 접촉 충격까지 포함한 안정성은 simulation과 제한된 실기 시험으로 확인해야 한다.

### 5.10 DYNAMIXEL mapping과 단위

xacro는 policy joint와 DYNAMIXEL ID를 transmission으로 연결한다. 이때 서로 다른 세 종류의
“방향”을 혼동하면 안 된다.

- URDF joint의 양의 회전 방향
- motor encoder 값이 증가하는 방향
- policy가 학습한 joint 축 방향

`Drive Mode`만 뒤집는 것으로 항상 세 좌표계가 맞는 것은 아니다. zero offset과 mechanical
assembly 방향도 확인해야 한다. 각 관절을 torque 제한 상태에서 작은 양의 command로 움직여 실제
방향과 joint state 부호를 기록해야 한다.

모든 controller 값은 기본적으로 SI 단위를 기대한다. 각도 rad, 각속도 rad/s, 시간 s다.
10도는 약 0.1745 rad다. 설정의 기본 자세 값들이 degree가 아니라 rad인지 계속 유지해야 한다.

### 5.11 ONNX backend를 붙일 때의 필수 계약

ONNX는 신경망 계산 그래프를 저장하는 형식이고 ONNX Runtime은 이를 실행하는 library다. 모델을
연결할 때 파일 경로만 parameter로 추가해서는 부족하다. configure에서 최소한 다음을 검사한다.

- 입력/출력 tensor 이름
- 입력 rank와 각 dimension (`[1, observation_size]` 등)
- 출력 dimension이 joint 수 N과 같은지
- tensor type이 float32인지
- metadata의 joint 이름/순서와 YAML이 같은지
- default position, action scale, observation normalization 버전

update에서 예외나 잘못된 출력이 생기면 `NaN/Inf`, shape 오류, 추론 시간 초과를 구분해 세고,
정해진 정책에 따라 SAFE_HOLD 또는 FAULT로 이동해야 한다. 첫 실기 전에 기록된 입력에 대한
Python 결과와 C++ 결과가 허용 오차 안에서 같은 golden test가 필요하다.

## 6. 상태별 안전 의미

| 상태 | command | 진입 조건 | 빠져나가는 조건 |
|---|---|---|---|
| `READY_POSE` | activation 자세→기본 자세 보간 | controller activate | 보간 완료 |
| `SAFE_HOLD` | 기본 자세로 rate-limited 접근 | 보간 완료 또는 policy disable | enable + IMU fresh + 직립 + 자세오차 ≤ `entry_position_tolerance` |
| `POLICY_RUNNING` | 정책 target, 현재는 기본 자세 수렴 | 위 진입 게이트 4개 전부 통과 | disable(→SAFE_HOLD) 또는 fault |
| `FAULT` | fault 순간 측정 자세 hold | stale/낙상/잘못된 state | 현재 자동 복구 없음 |

`WAITING_FOR_STATE`는 deactivate 시 설정되는 상태이고, `update()`의 `switch`에서도 명시적으로
처리된다(측정 자세를 잡고 `READY_POSE`로 넘어간다). `switch`에 `default:`를 두지 않았으므로
새 `Mode`를 추가하면 컴파일러가 `-Wswitch`로 누락을 잡는다.

fault reset을 구현할 때 lifecycle 재활성화만 허용할지, 별도 reset service를 둘지 결정해야 한다.
현재는 재활성화만 가능하다.

## 7. 설정 항목 문서

| parameter | 의미 | 현재값 | 필수 조건 |
|---|---|---:|---|
| `joints` | 정책 관절 순서 | 18개 | 비어 있지 않고 중복 없음 |
| `default_positions` | action 0의 자세 | 관절별 | N개, 유한, joint limit 내부 |
| `joint_position_min/max` | software 위치 한계 | 미지정 시 ±π | N개, min < max |
| `policy_rate_hz` | 정책 실행 주파수 | 50 | 유한, 0보다 큼 |
| `ready_pose_duration_s` | 초기 자세 보간 시간 | 2.0 | 유한, 0보다 큼 |
| `command_timeout_s` | 속도 명령 watchdog | 0.25 | 유한, 0보다 큼 |
| `imu_timeout_s` | IMU watchdog | 0.10 | 유한, 0보다 큼 |
| `action_scale` | action→rad 배율 | 1.0 | 유한, 0 이상 |
| `action_clip` | action 절댓값 한계 | 1.0 | 유한, 0보다 큼 |
| `target_rate_limit` | target 변화율 rad/s | 4.0 | 유한, 0보다 큼 |
| `target_filter_alpha` | 새 target 반영 비율 | 1.0 (LPF 비활성화) | 0 < alpha <= 1 |
| `fall_gravity_z_threshold` | 낙상 z 경계 | -0.5 | -1 이상 1 이하 |
| `fall_debounce_s` | 낙상 조건 지속 시간 | 0.10 | 유한, 0 이상 |
| `entry_position_tolerance` | 정책 진입 시 기본 자세 오차 rad | 0.35 | 유한, 0 이상; 0은 검사 비활성화 |
| `use_gait_phase` | observation phase 2개 추가 | false | 모델 계약과 동일 |
| `phase_total_steps` | gait phase 한 주기의 policy step 수 | 20 | 1 이상 |
| `dry_run_policy` | zero-action backend 사용 | true | 실제 backend 전 true 유지 |

`joint_position_min/max`는 현재 YAML에 명시되지 않아 기본값 ±π를 쓴다. 이는 실제 기구 한계가
아니다. 실기 전 URDF 및 제조사 한계를 근거로 관절별 값을 반드시 입력해야 한다.

## 8. 문서화해야 할 것과 필요한 배경지식

### 모델 문서

작성할 내용:

- 모델 파일의 출처, commit/hash, 학습 환경 버전
- observation/action의 이름, 순서, shape, dtype, 단위, scale, clip
- joint 순서와 default pose
- policy 주기, history, gait phase 정의
- 검증용 golden input/output

필요한 배경지식: tensor shape, feature 정규화, ONNX graph와 metadata, 부동소수점 허용 오차.

### 로봇·하드웨어 문서

작성할 내용:

- joint 이름↔motor ID↔모델↔장착 위치 표
- 축 방향, Drive Mode, zero offset, gear ratio
- position/velocity/current 단위와 변환식
- 기구 한계, software limit, motor gain, current/temperature limit
- 포트, baud rate, 전원, emergency stop 절차

필요한 배경지식: URDF joint axis, transmission, encoder, DYNAMIXEL control table, SI 단위.

### 좌표계·센서 문서

작성할 내용:

- world/base/IMU frame 관계와 축 그림
- quaternion 의미와 IMU mounting transform
- angular velocity와 projected gravity의 부호
- timestamp 출처, 주기, 지연, timeout
- 정지·기울임 실측값

필요한 배경지식: 오른손 좌표계, 회전과 quaternion, frame transform, sensor timestamp.

### controller 문서

작성할 내용:

- lifecycle 및 내부 상태 전이와 각 상태 command
- update/policy/sensor 주기와 deadline
- topic, QoS, parameter, interface 이름
- 각 fault 조건, fault 후 동작, reset 방법
- filter/rate limit 공식과 선정 근거

필요한 배경지식: ROS 2 executor와 QoS, `ros2_control`, 실시간 programming, watchdog.

### 시험·운영 문서

작성할 내용:

- unit/integration/simulation/HIL/실기 시험의 입력과 합격 기준
- preflight와 startup/shutdown 순서
- 로그에 남길 상태, 추론 시간, sensor age, saturation 횟수
- rollback 가능한 모델/설정 버전
- 안전 담당자와 시험 중 중단 기준

필요한 배경지식: 회귀 시험, hardware-in-the-loop, deadline/latency 통계, 위험 분석.

## 9. 남은 작업

우선순위 0 — 실기 전에 반드시:

- 실제 joint limit을 YAML에 명시하고 URDF/기구 한계와 대조
- 18개 motor ID, 방향, zero offset, motor model을 실물에서 검증
- IMU 장착 frame과 projected gravity 부호 검증
- position hold 대신 낙상 시 damping/torque 정책 결정
- 물리 emergency stop과 낮은 gain/current 제한을 포함한 시험 절차 작성

우선순위 1 — 실제 정책 연결:

- `PolicyContract`, `ObservationBuilder`, `PolicyRunner`, `ActionPipeline`을 분리
- ONNX Runtime을 configure에서 로드하고 모든 tensor 계약 검사
- 학습과 동일한 observation scale/clip/history 구현
- inference `NaN/Inf`, 예외, 반복 실패, overrun 처리
- Python/C++ golden I/O test 추가

우선순위 2 — 회귀 방지:

- parameter validation unit test
- observation 순서/shape test (N=18에서 63/65)
- READY_POSE 보간과 상태 전이 test
- stale/future/비유한 입력 및 낙상 test
- action clip/filter/rate/joint-limit test
- plugin load integration test와 simulation test

## 10. 단계별 검증 절차

1. 빌드: `colcon build --symlink-install --packages-select kid_rl`
2. plugin: controller manager가 `kid_rl/KidRlController`를 load하는지 확인
3. 무출력 preflight: torque-off 또는 mock hardware에서 joint/interface 순서 확인
4. dry run: `dry_run_policy=true`, 로봇 고정 상태에서 topic freshness와 상태 전이 확인
5. 제한 실기: 낮은 gain/current, 지지대, e-stop 담당자와 READY_POSE만 확인
6. 기록 재생: 실제 sensor log를 policy에 넣고 target/limit/fault 분석
7. 정책 실기: 작은 command 범위부터 단계적으로 확대

각 단계가 통과하기 전 다음 단계로 넘어가지 않는다. 빌드 성공은 동작 안전성의 증거가 아니며,
simulation 성공도 motor 방향과 실물 기구 한계를 보장하지 않는다.

## 11. 검증 이력

| 날짜 | 대상 | 명령/방법 | 결과 |
|---|---|---|---|
| 2026-08-31 | Claude 초기 구현 | 기존 colcon 로그 확인 | 최종 빌드 로그에는 오류 없음. 과거 const 관련 오류는 수정된 상태 |
| 2026-08-31 | Claude/Codex 통합 안전성 보강 | `colcon build --symlink-install --packages-select kid_rl` | 성공 |
| 2026-08-31 | xacro/XML/YAML 구문 | xacro 확장, Python XML/YAML parse | 성공 |
| 2026-08-31 | 패키지 시험 경로 | `colcon test --packages-select kid_rl` | 명령 성공, 등록된 test는 0개 |

## 12. 결정 기록

- controller manager 200 Hz와 policy 50 Hz를 분리한다.
- 실제 backend가 들어오기 전에는 zero-action dry run임을 명시한다.
- 잘못된 수치를 조용히 보정하지 않고 configure/input 경계에서 거부한다.
- joint 순서와 모델 계약은 하나의 versioned contract로 통합할 예정이다.
- 실물에서 검증되지 않은 Drive Mode, offset, limit은 확정 사실로 취급하지 않는다.
