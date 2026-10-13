# kid_rl 컨트롤러 완전 가이드

이 문서는 `kid_rl` 패키지가 **무엇을 하는지**, 그리고 그 코드를 이해하는 데 필요한
**배경지식 전부**를 담는다. ros2_control을 처음 보는 사람, 강화학습 정책을 실기에
올려본 적 없는 사람이 이 문서만 읽고 코드를 읽을 수 있게 하는 것이 목표다.

읽는 순서 추천:

관련 문서: 배경지식·레퍼런스 구현 비교는 [`STUDY.md`](STUDY.md),
스레드·400 Hz는 [`THREADING.md`](THREADING.md), 이력과 할 일은 [`WORKLOG.md`](WORKLOG.md).

| 목적 | 읽을 절 |
| --- | --- |
| "이게 대체 뭐 하는 물건인가" | 1, 2 |
| ros2_control을 모른다 | 3 |
| 관측 벡터/정책 계약이 궁금하다 | 4 |
| 주기(50 Hz vs 200 Hz)가 헷갈린다 | 5 |
| 안전장치를 손보려 한다 | 6 |
| DYNAMIXEL 쪽을 만진다 | 7 |
| 코드를 직접 고친다 | 8, 9 |
| 로봇을 켜려 한다 | 10, 11 |

---

## 1. 한 문단 요약

`kid_rl`은 시뮬레이션에서 학습한 **보행 정책(neural network)** 을 실제 로봇 위에서
돌리기 위한 `ros2_control` 컨트롤러다. 매 제어 주기마다 관절 상태와 IMU를 읽어
정책이 기대하는 **관측 벡터(observation)** 로 포장하고, 정책이 뱉은 **행동(action)** 을
관절 목표 위치로 변환해 하드웨어에 쓴다. 그 사이에 "언제 정책을 켜도 되는가",
"언제 꺼야 하는가"를 판단하는 **상태 기계와 안전장치**가 들어 있다.

정책 백엔드는 `models/policy.onnx`를 C++ ONNX Runtime으로 실행한다. 모델은 configure
단계에서 입력·출력 shape와 관절 순서를 검사하고 워밍업 추론을 마친다. 이후 정책
주기에는 미리 할당한 입력·출력 텐서를 재사용한다. `dry_run_policy=true`는 하드웨어
통신 시험을 위해 action을 0으로 만드는 명시적인 시험 모드다.

---

## 2. 큰 그림: sim2real 배포가 무엇인가

### 2.1 학습 쪽에서 벌어지는 일

Isaac Lab / MuJoCo(mjlab) 같은 시뮬레이터에서 수천 개의 로봇을 병렬로 굴리면서
PPO 같은 알고리즘으로 정책을 학습한다. 정책은 결국 이런 함수다.

```
action = π(observation)
```

- `observation`: 로봇의 현재 상태를 **고정 길이 실수 벡터**로 납작하게 편 것
- `action`: 관절 개수만큼의 실수. 보통 **토크가 아니라 목표 위치의 오프셋**이다.

학습이 끝나면 `walk.onnx` 같은 파일로 내보낸다. ONNX는 신경망 구조와 가중치를
프레임워크 독립적으로 담는 포맷이고, ONNX Runtime이 그걸 CPU에서 돌려준다.

### 2.2 실기 쪽에서 벌어져야 하는 일

실기 코드가 할 일은 딱 두 가지다.

1. **시뮬레이터가 정책에게 주던 것과 비트 단위로 같은 관측을 만든다.**
2. **정책의 출력을 시뮬레이터가 하던 것과 같은 방식으로 관절 명령으로 바꾼다.**

이게 전부다. 그리고 이게 sim2real에서 실패하는 이유의 90%다. 순서가 하나 틀리거나,
단위가 rad/s가 아니라 deg/s거나, 부호가 뒤집혔거나, 기준 자세가 다르면 정책은
"학습 때 본 적 없는 입력"을 받고 아무 의미 없는 출력을 낸다. 로봇은 즉시 넘어진다.
**부드럽게 실패하지 않는다.** 그래서 이 코드는 계약(contract) 검증에 집착한다.

### 2.3 sim2real gap

시뮬레이터와 실기는 절대 같지 않다.

| 항목 | 시뮬 | 실기 |
| --- | --- | --- |
| 관절 지연 | 0 | 통신 + 펌웨어 = 수 ms |
| 관절 속도 | 해석적으로 정확 | 엔코더 차분, 노이즈 큼 |
| IMU | 완벽 | bias, 노이즈, 마운트 회전 |
| 접지 | 이상적 마찰 | 미끄러짐, 발바닥 변형 |
| 액추에이터 | 이상적 PD | 전류 한계, 전압 강하, 백래시 |

학습 쪽에서는 domain randomization으로 이 간극을 덮고, 배포 쪽에서는 **필터·제한·
watchdog** 으로 덮는다. 6절의 안전장치는 전부 이 간극에 대한 보험이다.

---

## 3. 배경지식: ros2_control

### 3.1 왜 그냥 ROS 노드로 안 하는가

"IMU 토픽 구독하고 관절 명령 토픽 발행하는 노드 하나 짜면 되지 않나?"

안 된다. 이유는 **지연과 지터(jitter)** 다. 보행 제어는 200 Hz면 5 ms 주기다.
DDS 토픽 왕복은 좋은 날에도 1~3 ms가 걸리고, 나쁜 날에는 수십 ms가 튄다. 게다가
ROS executor는 실시간 스케줄링이 아니라서 언제 깨어날지 보장이 없다.

`ros2_control`은 이 문제를 **하나의 프로세스, 하나의 실시간 스레드** 안에
하드웨어 접근과 제어 연산을 밀어넣어서 푼다.

### 3.2 구성 요소

```
                    ┌───────────────────────────────────────┐
                    │        controller_manager             │
                    │  (하나의 프로세스, 하나의 RT 스레드)   │
                    │                                       │
   URDF ──────────▶ │  ┌─────────────────────────────────┐  │
   <ros2_control>   │  │      Resource Manager           │  │
                    │  │  Hardware Component 인스턴스들   │  │
                    │  │  ex) DynamixelHardware          │  │
                    │  └───────┬─────────────────▲───────┘  │
                    │   state  │                 │ command  │
                    │ interface│                 │interface │
                    │  ┌───────▼─────────────────┴───────┐  │
                    │  │   Controller (플러그인)          │  │
                    │  │   ex) kid_rl/KidRlController    │  │
                    │  └─────────────────────────────────┘  │
                    └───────────────────────────────────────┘
```

- **Hardware Component**: 실제 장치와 말하는 쪽. `read()`로 센서를 읽고 `write()`로
  액추에이터에 쓴다. 우리 경우 `dynamixel_hardware_interface/DynamixelHardware`가
  USB2Dynamixel 버스에서 sync read/write를 한다.
- **Resource Manager**: 하드웨어가 노출하는 "인터페이스"들을 이름으로 관리하고,
  어느 컨트롤러가 어느 인터페이스를 점유(claim)했는지 추적한다.
- **Controller**: 순수 계산. 하드웨어를 직접 만지지 않고, 빌려온 인터페이스만 읽고 쓴다.
  pluginlib으로 동적 로딩되는 공유 라이브러리다 (`kid_rl_plugin.xml`이 그 선언).

### 3.3 인터페이스: state와 command

인터페이스 이름은 항상 `<prefix>/<interface>` 꼴이다.

- **state interface** — 읽기 전용. `left_knee/position`, `left_knee/velocity`, `left_knee/effort`
- **command interface** — 쓰기 전용. `left_knee/position`

컨트롤러는 두 함수로 "뭘 쓰겠다"를 선언한다.

```cpp
// src/kid_rl_controller.cpp
controller_interface::InterfaceConfiguration
KidRlController::command_interface_configuration() const   // 18개 position
KidRlController::state_interface_configuration() const     // 18개 position + 18개 velocity
```

`INDIVIDUAL` 타입은 "이름을 하나하나 지정하겠다"는 뜻이다 (`ALL`은 전부, `NONE`은 없음).
**한 command interface는 한 컨트롤러만 점유할 수 있다.** 그래서 `kid_rl`이 활성화된
동안 `joint_trajectory_controller` 같은 다른 위치 컨트롤러를 동시에 켤 수 없다.

> **주의 — 순서를 믿지 말 것.**
> `state_interfaces_` 벡터가 우리가 요청한 순서대로 채워진다는 보장에 기대면 안 된다.
> 예전 코드는 `state_interfaces_[2*i]`가 i번 관절의 position이라고 가정했는데,
> 그 가정이 깨지면 **왼쪽 무릎 목표가 오른쪽 발목으로 간다.** 조용히, 로그 한 줄 없이.
> 지금 코드는 `on_activate()`의 `map_interfaces()`에서 `get_prefix_name()` /
> `get_interface_name()`으로 인덱스를 만들어 둔다.

### 3.4 lifecycle: 언제 무엇을 하는가

컨트롤러는 ROS 2 managed node의 lifecycle을 따른다.

| 콜백 | 시점 | 여기서 할 일 | 여기서 하면 안 되는 일 |
| --- | --- | --- | --- |
| `on_init()` | 로드 직후 | 파라미터 **선언** | 하드웨어 접근 |
| `on_configure()` | `configure` 요청 | 파라미터 **검증**, 메모리 **전부 할당**, 구독 생성, ONNX 세션 로드 | 명령 쓰기 |
| `on_activate()` | `activate` 요청 | 인터페이스 인덱스 매핑, 측정 자세 스냅샷, 상태 초기화 | 큰 할당 |
| `update()` | 매 주기 | 계산만 | **할당, 락, 파일 I/O, 예외, 로그 폭탄** |
| `on_deactivate()` | `deactivate` 요청 | 내부 상태 리셋 | |
| `on_cleanup()` | `cleanup` 요청 | 구독 해제 | |

**핵심 원칙: `update()`에서는 힙 할당을 하지 않는다.** `malloc`은 락을 잡을 수 있고,
락은 우선순위 역전을 만들고, 우선순위 역전은 제어 주기를 놓치게 하고, 주기를 놓치면
로봇이 넘어진다. 그래서 `on_configure()`에서 `positions_`, `targets_`, `observation_`
같은 모든 벡터를 최종 크기로 `assign()` 해 두고, `update()`에서는 인덱스로만 접근한다.

### 3.5 read → update → write

`controller_manager`의 실시간 루프는 매 주기 이 순서로 돈다.

```
for each hardware:   read()      # 버스에서 센서값을 읽어 state interface에 채움
for each controller: update()    # state interface -> 계산 -> command interface
for each hardware:   write()     # command interface를 버스로 내보냄
```

그래서 `update()` 안에서 보이는 관절 상태는 **이번 주기에 방금 읽은 값**이고,
`command_interfaces_[i].set_value()` 로 쓴 값은 **이번 주기 끝에 하드웨어로 나간다.**

### 3.6 RealtimeBuffer: 두 스레드를 잇는 다리

문제: `~/imu` 토픽은 ROS executor 스레드가 콜백으로 받는다. `update()`는 실시간
스레드가 돈다. 두 스레드가 같은 데이터를 만지는데 **뮤텍스를 쓸 수 없다**
(RT 스레드가 블록되면 안 되니까).

해답이 `realtime_tools::RealtimeBuffer<T>`다. 내부에 버퍼 두 개를 두고,

- non-RT 쪽(`writeFromNonRT`)은 놀고 있는 버퍼에 쓰고 "새 데이터 있음" 플래그를 세운다
- RT 쪽(`readFromRT`)은 플래그가 서 있으면 포인터를 **swap** 하고 새 버퍼를 반환한다

RT 쪽은 절대 대기하지 않는다. 대신 **오래된 데이터를 읽을 수는 있다.** 그래서
타임스탬프와 watchdog이 필요하다 (6.2절).

> **함정 — `readFromRT()`가 반환한 포인터의 수명.**
> `readFromRT()`는 포인터를 swap할 수 있다. 즉 **첫 호출로 받은 포인터는 다음 호출
> 이후에는 유효하지 않을 수 있다.** (그 버퍼가 이제 writer 차지가 되니까.)
> 예전 코드는 포인터를 잡아둔 채 `inputs_are_fresh()`가 내부에서 다시 `readFromRT()`를
> 부르고, 그 후에 원래 포인터를 역참조했다. 지금 코드는 `update()` 맨 앞에서
> **버퍼마다 딱 한 번 읽고 값으로 복사**한다.
> ```cpp
> const auto * command_ptr = command_buffer_.readFromRT();
> const auto * imu_ptr = imu_buffer_.readFromRT();
> CommandSample command = command_ptr ? *command_ptr : CommandSample{};
> const ImuSample imu = imu_ptr ? *imu_ptr : ImuSample{};
> ```
> `CommandSample`/`ImuSample`은 POD라서 복사에 할당이 없다.

### 3.7 시간을 다루는 법

`rclcpp::Time`의 뺄셈은 **두 시각의 clock type이 다르면 예외를 던진다.**
`update()`에서 예외가 나가면 그대로 컨트롤러가 죽는다. 그래서 이 코드는 타임스탬프를
`rclcpp::Time`이 아니라 `std::int64_t` 나노초로 저장하고 정수 비교만 한다.

```cpp
struct ImuSample { ...; std::int64_t stamp_ns{0}; bool valid{false}; };
const bool imu_fresh = imu.valid &&
  now_ns - imu.stamp_ns >= 0 &&
  now_ns - imu.stamp_ns <= to_nanoseconds(imu_timeout_s_);
```

`>= 0` 검사는 미래에서 온 타임스탬프(시뮬 시간 점프, 시계 재동기화)를 "무한히 신선함"
으로 오해하지 않기 위한 것이다.

---

## 4. 정책 계약 (Policy Contract)

이 절이 이 문서에서 제일 중요하다. **여기 적힌 것 중 하나라도 틀리면 로봇이 넘어진다.**

### 4.1 관절 순서

`config/controllers.yaml`의 `joints:` 리스트는 microban의
`src/constants.py: OBSERVATION_DOF_ORDER`와 **정확히 같은 순서**여야 한다.

```
right_shoulder_pitch, right_shoulder_roll, right_elbow,
right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee,
right_ankle_pitch, right_ankle_roll,
left_shoulder_pitch, left_shoulder_roll, left_elbow,
left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee,
left_ankle_pitch, left_ankle_roll
```

URDF의 순서도, 알파벳 순서도, DYNAMIXEL ID 순서도 아니다. **학습 때 쓴 순서**다.
ONNX 모델에는 `joint_names` 메타데이터가 들어 있으니, PolicyRunner를 붙일 때
이 리스트와 대조해서 다르면 configure를 실패시켜야 한다 (9절 TODO).

### 4.2 관측 벡터 레이아웃

N = 관절 수 = 18. 총 길이 = 3 + 3 + 3N + 3 (+ 2) = **63** (gait phase 쓰면 65).

| 인덱스 | 내용 | 단위 | 프레임 |
| --- | --- | --- | --- |
| `0..2` | 각속도 (gyro) | rad/s | body |
| `3..5` | projected gravity | 단위벡터 | body |
| `6 .. 6+N-1` | `q - q_default` | rad | 관절 |
| `6+N .. 6+2N-1` | `q̇` | rad/s | 관절 |
| `6+2N .. 6+3N-1` | 직전 action | (무차원) | — |
| `6+3N .. 8+3N` | 명령 속도 `vx, vy, ωz` | m/s, m/s, rad/s | body |
| `9+3N, 10+3N` | gait phase `cos, sin` | — | — (선택) |

구현: `KidRlController::build_observation()`.

### 4.3 왜 각 항목이 그 형태인가

**각속도(gyro)** — 자이로 원시값을 그대로 쓴다. 적분하지 않는다. 적분하면 드리프트가
쌓이고, 정책은 어차피 순간 각속도만 필요하다 (넘어지는 속도를 알면 된다).

**projected gravity** — 이게 왜 quaternion보다 나은지가 중요하다.

몸통 자세를 나타내는 방법은 여럿이다: 쿼터니언 4개, 오일러각 3개, 회전행렬 9개.
그런데 보행 정책은 **로봇이 어느 방향을 보고 있든 똑같이 행동해야 한다.** 북쪽을
보든 남쪽을 보든 앞으로 걷는 법은 같다. 즉 yaw는 관측에 **없어야** 한다.

projected gravity는 세계 좌표계의 중력 방향 `(0, 0, -1)`을 몸통 좌표계에서 본
3개 값이다.

```text
입력: 세계 좌표계의 중력 방향 (0, 0, -1), 몸통 자세 q
변환: 중력 방향을 몸통 좌표계로 회전
출력: 몸통 좌표계의 중력 방향 (gx, gy, gz)
```

몸통을 세계 z축 둘레로 돌리는 yaw는 중력 방향 `(0, 0, -1)`을 바꾸지 않는다.
따라서 yaw만 바뀌면 projected gravity는 그대로다. 반면 몸통이 앞뒤나 좌우로
기울면 중력이 몸통 좌표계에서 보이는 방향이 바뀌므로 pitch와 roll은 값에 남는다.
즉 **yaw에는 완전히 불변**이면서 roll과 pitch는 그대로 담는다.

구현에서는 쿼터니언 `q = (qw, qx, qy, qz)`에서 다음 세 값을 바로 계산한다.

```cpp
sample.projected_gravity = {
  -2.0 * (qx * qz - qw * qy),
  -2.0 * (qy * qz + qw * qx),
  -(1.0 - 2.0 * (qx * qx + qy * qy))};
```

직립일 때는 `(gx, gy, gz) = (0, 0, -1)`이다. 완전히 옆으로 누우면 `gz`가
0에 가까워지고, 뒤집히면 `gz`가 1에 가까워진다. 그래서 낙상 판정이
`gz > -0.5`다 (6.3절).

**`q - q_default`** — 절대 관절각이 아니라 기준 자세로부터의 편차를 준다. 이유:
(1) 대부분의 값이 0 근처라 신경망 입력 정규화가 쉽고, (2) 기준 자세를 바꿔도
정책이 "중립"의 의미를 다시 배울 필요가 없다. `q_default`는 학습 때 쓴 값과
**정확히** 같아야 한다. ONNX 메타데이터의 `default_joint_pos`가 정본이다.

**직전 action** — 정책에게 "네가 방금 뭘 시켰는지"를 알려준다. 없으면 정책은
자기 명령과 실제 관절 상태의 지연 관계를 추론할 수 없어서 진동하기 쉽다. 사실상
정책에 1스텝 메모리를 주는 값싼 방법이다.

> **여기 저장하는 값은 clip 전 원본이어야 한다.** 학습 때 정책이 관측한 것은
> 네트워크 raw 출력이다. 우리가 안전을 위해 clip한 값을 되먹이면 관측 분포가
> 학습과 어긋난다. 그래서 `update_target_from_action()`은
> `last_actions_[i] = actions_[i]` (raw)를 저장하고, clip은 목표 위치 계산에만 쓴다.

**명령 속도** — 사용자가 원하는 몸통 속도. `~/cmd_vel`의 `linear.x`, `linear.y`,
`angular.z`. 학습 때 쓴 범위를 넘기면 안 된다 (microban은 vx ≤ 0.7 m/s 등).
아직 이 컨트롤러는 스케일링/제한을 하지 않는다 (9절 TODO).

**gait phase** — 정책에 "현재 한 걸음 주기의 어느 지점인가"를 알려주는 선택적 관측이다.
예를 들어 한 주기를 20번의 정책 실행으로 나누면 `phase_step`은 0부터 19까지 진행한다.
이 번호를 그대로 넣지 않고 다음 두 값으로 바꿔 observation 맨 뒤에 붙인다.

```text
phase_step 0  → (cos, sin) = ( 1,  0)  주기 시작
phase_step 5  → (cos, sin) = ( 0,  1)  주기의 1/4
phase_step 10 → (cos, sin) = (-1,  0)  주기의 절반
phase_step 15 → (cos, sin) = ( 0, -1)  주기의 3/4
```

두 값으로 표현하면 19에서 0으로 돌아갈 때도 원 위에서 가까운 값끼리 이어진다. 정책이
주기 경계에서 갑작스러운 숫자 점프를 보지 않는 이유다. 속도 명령이 거의 0이면
`phase_step`을 0으로 되돌리므로 관측되는 쌍은 `(1, 0)`이 된다.

"관측 길이가 2 늘어난다"는 말은 정책 입력 벡터 끝에 `cos`와 `sin` 두 칸이 실제로
추가된다는 뜻이다.

```text
18관절 배포 형식: gait phase 없음 → observation shape [batch, 63]
18관절 배포 형식: gait phase 사용 → observation shape [batch, 65]

현재 mjlab 25관절 형식:
3 gyro + 3 gravity + 25 joint_pos + 25 joint_vel + 25 last_action + 3 command = 84
gait phase까지 학습했다면 84 + 2 = 86
```

**현재 `mjlab_kid_RL` 학습에는 gait phase를 observation으로 넣은 코드가 없다.** 최신
dance 모델의 입력도 84개이므로 위 기본 항목만 사용하며, 배포 설정은
`use_gait_phase: false`여야 한다. 설정을 잘못 `true`로 바꾸면 C++ 컨트롤러가 86개를
만들어 84개 입력 모델과 맞지 않게 된다. `delay_per_env_phase`라는 설정은 센서 지연의
갱신 시점을 환경별로 분산하는 옵션일 뿐, 여기서 말하는 보행 위상 관측이 아니다.

나중에 gait phase를 쓰려면 학습 환경에도 동일한 두 값을 observation term으로 넣고
그 상태로 모델을 새로 학습해야 한다. 이미 학습된 84입력 모델에 배포 단계에서만 두 값을
추가해서는 안 된다. 현재 컨트롤러에는 입력 shape 자동 검증이 없으므로 PolicyRunner를
붙일 때 모델 입력 크기와 `use_gait_phase` 설정이 일치하는지 검사해야 한다.

<!-- gait phase는 센서로 측정한 로봇 상태가 아니라 정책에 제공하는 주기적인 시계 신호입니다.
붙이는 이유는 비슷한 로봇 상태에서도 보행 주기의 위치에 따라 다음 행동이 달라질 수 있기 때문입니다.
최소 예시:
현재 자세: 몸통 직립, 양쪽 무릎 각도도 비슷함
속도 명령: 앞으로 걷기

phase = 주기 시작   → 왼발을 내보내기
phase = 주기 절반   → 오른발을 내보내기
phase가 없으면 정책은 관절 상태, 속도, 직전 action만 보고 현재가 왼발 차례인지 오른발 차례인지 추론해야 합니다. phase를 넣으면 정책에 그 순서를 직접 알려줘 일정한 교대 보행을 배우기 쉬워집니다.
다만 지적한 것처럼 단점도 있습니다. 이 값은 실제 발 상태와 무관하게 증가하는 open-loop 시계이므로:
phase 신호: 왼발을 내디딜 차례
실제 로봇: 미끄러져서 아직 오른발 지지가 끝나지 않음
처럼 실제 움직임과 어긋날 수 있습니다. 이 경우 정책이 회복보다 예정된 동작을 고집할 수 있습니다.
따라서 gait phase는 필수 관측이 아닙니다.
- 정해진 박자와 좌우 교대 동작을 강하게 학습할 때 유용
- 지형 변화나 미끄러짐에 대한 반응성이 중요하면 불리할 수 있음
- 관절 위치, 속도, 직전 action만으로도 정책이 보행 단계를 추론할 수 있음
현재 mjlab_kid_RL 모델은 gait phase 없이 학습됐습니다. 따라서 이 프로젝트에서는 해당 두 값을 붙이지 않고 use_gait_phase: false로 두는 것이 맞습니다. -->



### 4.4 action → 관절 목표

```
q_target[i] = q_default[i] + action_scale * clip(action[i], ±action_clip)
```

`action_scale`은 정책 출력의 단위를 rad로 바꾸는 계수다. microban은 1.0을 쓴다.
이 값도 학습 설정에서 가져와야 하며, 추측하면 안 된다.

그다음 세 겹의 안전 필터를 통과한다 (6.4절).

### 4.5 IMU 프레임 — 지금은 가정, 나중에 검증

이 컨트롤러는 `~/imu`가 **몸통(trunk) 좌표계** 기준이라고 가정한다. microban은
IMU가 몸통에 비틀려 붙어 있어서 `IMU_MOUNT_QUAT = (0.5, -0.5, -0.5, 0.5)`로
센서 프레임 → 몸통 프레임 회전을 적용한다. 우리 쪽에서는 그 보정을 **IMU 드라이버가
이미 했다고 가정**한다. 실기에서 제일 먼저 확인할 것: 로봇을 똑바로 세우고
`ros2 topic echo /kid_rl/imu` 했을 때 orientation이 단위 쿼터니언에 가까운가.
아니면 마운트 회전 파라미터를 컨트롤러에 추가해야 한다 (9절 TODO).

---

## 5. 주기 설계 (multi-rate)

### 5.1 왜 주기가 두 개인가

- **controller_manager update_rate = 400 Hz (목표)** — 하드웨어 버스를 얼마나 자주 읽고
  쓸지. 높을수록 관절 추종이 부드럽고 지연이 작다. 상한은 DYNAMIXEL 버스 대역폭이 정한다.
  하드웨어의 `rw_rate`는 이 값을 **넘을 수 없다**(감속만 가능). 자세한 것은
  [`THREADING.md`](THREADING.md).
- **policy_rate_hz = 50 Hz** — 정책을 얼마나 자주 추론할지.
  **이건 성능 문제가 아니라 계약 문제다.** 학습 때 정책은 `dt = 0.02 s` 간격으로
  행동했다. 실기에서 200 Hz로 돌리면 정책이 보는 "한 스텝"의 물리적 의미가 4배
  달라지고, 직전 action 되먹임의 시간 상수도 어긋나서 정책이 발산한다.
  **학습 dt와 반드시 일치시킬 것.**

### 5.2 decimation 구현

```cpp
policy_accumulator_s_ += dt;
const double policy_period = 1.0 / policy_rate_hz_;
if (policy_accumulator_s_ >= policy_period) {
  policy_accumulator_s_ = std::fmod(policy_accumulator_s_, policy_period);
  build_observation(command, imu);
  run_policy();
  update_target_from_action(policy_period);
}
```

카운터(`if (++tick % 4 == 0)`)가 아니라 **시간 누적**을 쓰는 이유: 제어 주기가
정확히 200 Hz가 아닐 수 있고(지터), `update_rate`를 바꿔도 정책 주기는 그대로
50 Hz로 유지되어야 하기 때문이다. `fmod`로 잔여 시간을 남겨서 장기적으로 평균
50 Hz가 유지된다.

`update_target_from_action`에 넘기는 dt가 `policy_period`인 것도 의도적이다.
목표 변화율 제한은 "정책 한 스텝 동안 얼마나 움직일 수 있는가"이지, 컨트롤러
한 틱 기준이 아니다.




### 5.4 dt 폭주 방어

```cpp
const double dt = clamp(period.seconds(), 0.0, kMaxIntegrationPeriodS);  // 0.1 s
```

RT 스레드가 굶어서 `period`가 0.5초로 들어오면, 그걸 그대로 적분하면 rate limit이
`4.0 * 0.5 = 2 rad` 를 허용해 버린다. 사실상 제한이 없어지는 것이다. 100 ms로 자른다.

---

## 6. 안전 설계

### 6.1 상태 기계

lifecycle(`inactive`/`active`)과 **별개로** 내부 운전 상태를 둔다.

```
        on_activate
             │
             ▼
   ┌──────────────────┐   유효한 joint state
   │ WAITING_FOR_STATE│──────────────┐
   └──────────────────┘              ▼
                            ┌──────────────────┐
                            │    READY_POSE    │  측정 자세 → default 자세
                            │  (smoothstep 보간)│  ready_pose_duration_s 동안
                            └────────┬─────────┘
                                     │ 보간 완료
                                     ▼
                            ┌──────────────────┐
              enable=false  │    SAFE_HOLD     │  default 자세 유지
            ┌──────────────▶│ (rate-limited)   │  (rate limit 적용)
            │               └────────┬─────────┘
            │                        │ enable=true
            │                        │ AND IMU 신선
            │                        │ AND 직립
            │                        │ AND 자세오차 ≤ tol
            │                        ▼
            │               ┌──────────────────┐
            └───────────────│  POLICY_RUNNING  │
                            └────────┬─────────┘
                                     │ IMU stale / 낙상 / NaN action
                                     ▼
                            ┌──────────────────┐
                            │      FAULT       │  목표 동결 (latched)
                            └──────────────────┘
                                     │
                            deactivate → activate 만이 탈출구
```

각 상태의 의도:

- **READY_POSE** — 활성화 순간의 **측정된** 관절 위치에서 출발한다. 이게 중요하다.
  바로 `default_positions`를 명령하면 관절이 순간적으로 최대 토크로 튄다.
  smoothstep `3r² - 2r³`은 양 끝에서 속도가 0이라 시작할 때 토크 계단이 없고
  끝날 때 오버슈트가 없다.
- **SAFE_HOLD** — "정책은 꺼져 있지만 로봇은 서 있어야 한다". default 자세로
  rate limit을 걸어 접근한다. POLICY_RUNNING에서 여기로 돌아올 때 목표가
  튀지 않게 하기 위한 것이다 (예전 코드는 한 틱에 스냅해서 점프가 생겼다).
- **FAULT** — **latched**. 자동 복구하지 않는다. 사람이 로봇을 확인하고
  `ros2 control set_controller_state kid_rl inactive` → `active` 를 해야 나온다.
  안전 사고 후 자동 재기동은 사고를 두 번 낸다.

### 6.2 watchdog — IMU와 cmd_vel을 다르게 다루는 이유

이건 의도적인 비대칭이다.

| 입력 | timeout | 만료 시 동작 | 왜 |
| --- | --- | --- | --- |
| `~/imu` | `imu_timeout_s` (0.10) | **FAULT** | 자세를 모르면 어떤 명령도 안전하지 않다. 대체값이 없다. |
| `~/cmd_vel` | `command_timeout_s` (0.25) | **속도 명령 0** + throttled warn | 정책은 `(0,0,0)` 명령으로 제자리에 서도록 학습되어 있다. 안전한 대체값이 있다. |

예전 코드는 둘 다 FAULT였다. 조이스틱 퍼블리셔가 250 ms 딸꾹질하면 휴머노이드가
걷다 말고 FAULT로 굳는다는 뜻이다. 나쁘다. 게다가 위험하다 — 걷는 중에 목표가
동결되면 그대로 넘어진다. 지금은 명령만 0이 되고 정책은 계속 돌면서 로봇을 세운다.

### 6.3 낙상 감지와 debounce

```cpp
if (imu.projected_gravity[2] > fall_gravity_z_threshold_) {   // 기본 -0.5
  fall_timer_s_ += dt;
  if (fall_timer_s_ >= fall_debounce_s_) { enter_fault("fall detected"); }
  break;   // debounce 동안 목표 동결
}
fall_timer_s_ = 0.0;
```

`gz = -1`이면 완전 직립이고, `gz = -0.5`이면 몸통이 약 60° 기운 상태다.
60°의 코사인이 0.5이기 때문이다. 그보다 더 기울면 낙상 후보로 본다.

**debounce가 필요한 이유**: IMU 한 샘플이 튀거나(충격, I2C 글리치) 착지 충격으로
순간적으로 임계를 넘을 수 있다. 그걸로 정책을 죽이면 정상 보행 중에 nuisance
fault가 난다. 0.1초(= 정책 5스텝) 연속으로 유지되어야 진짜 낙상으로 본다.
그동안에는 정책 추론을 멈추고 목표를 동결한다 — microban의 `step()`이
낙상 조건에서 그냥 `return` 하는 것과 같은 동작이다.

### 6.4 4중 방어: action에서 하드웨어까지

정책 출력이 관절에 도달하기까지 네 개의 관문을 지난다.

```
action (정책 raw 출력)
  │
  ├─ (1) 유한성 검사 ──────────── NaN/Inf → FAULT
  │
  ├─ (2) clip(±action_clip) ───── 정책이 미쳐도 진폭 제한
  │
  │      q_raw = q_default + action_scale * action
  │
  ├─ (3) clamp(joint_min, joint_max) ── 기구 한계
  │
  │      policy_targets[i] = q_raw            ← 여기까지가 50 Hz (정책 틱)
  │      ─────────────────────────────────────────────────────────────
  │      step = alpha_tick * (policy_targets[i] - targets[i])   ← 1차 LPF
  ├─ (4) clamp(step, ±rate_limit*dt) ─ 변화율 제한   ← 여기부터 400 Hz (매 틱)
  │
  │      targets[i] += step
  ▼
write_commands(): 유한성 재검사 + joint limit 최종 clamp → 하드웨어
```

각각이 막는 것이 다르다.

- **(2) action clip** — 정책 자체의 이상 출력. 학습 분포 밖 관측이 들어오면
  신경망은 임의로 큰 값을 뱉을 수 있다.
- **(3) joint limit** — 기구적 파손. 링크 간섭, 케이블 끊김.
- **(4) rate limit** — 액추에이터 보호. 기본값에서는 LPF가 꺼져 있다.
  `rate_limit = 4 rad/s`는 정책 한 스텝(20 ms)에 최대 0.08 rad(4.6°).

  > **트레이드오프 주의.** 필터와 rate limit은 안전하지만 **지연을 만든다.**
  > 보행은 지연에 민감해서 너무 세게 걸면 정책이 불안정해진다. 브링업 초기에는
  > 보수적인 rate limit으로 시작하고, 안정적으로 걷기 시작하면
  > 서서히 풀어서 지연을 줄이는 것이 정석이다. 학습 쪽에서 필터를 모델링하지
  > 않았다면 결국 alpha는 1.0에 가까워야 sim과 맞는다.

- **`write_commands()`의 최종 clamp** — 모든 모드가 지나가는 **단일 관문**이다.
  READY_POSE든 FAULT든 정책이든, 어떤 경로로 만들어진 목표든 여기서 유한성과
  관절 한계가 보장된다. 새 모드를 추가해도 이 보장은 자동으로 상속된다.

### 6.5 FAULT에서 목표를 동결하는 것에 대해

지금 FAULT는 `targets_ = positions_`(사고 순간의 측정 자세)로 고정하고 그대로 둔다.
이게 최선은 아니다.

- **동결의 장점**: 로봇이 무너지지 않는다. 위치 인터페이스만 있는 지금 할 수 있는 최선.
- **동결의 단점**: 넘어진 로봇이 이상한 자세로 계속 최대 토크를 낸다. 모터가 타거나
  BMS가 트립될 수 있다.
- **이상적인 것**: **damping** — 토크를 0에 가깝게 낮추고 관절이 자유롭게 접히게
  한다. ai_sapiens와 roa_controller 둘 다 fault 시 Damping 상태로 간다.

우리는 DYNAMIXEL의 `Position P Gain`을 낮게 쓰거나 torque를 끄면 damping을 흉내낼 수
있다. 이건 hardware component에 gpio command interface가 필요하다 (9절 TODO).
**그때까지는 실기 테스트에 반드시 사람이 붙어서 전원을 끊을 수 있어야 한다.**

### 6.6 POLICY_RUNNING 진입 게이트

`enable_policy=true` 하나로 정책이 켜지지 않는다. 네 조건 전부:

1. READY_POSE가 끝나 SAFE_HOLD에 있을 것
2. IMU가 신선하고 유효할 것
3. 직립일 것 (`g_z ≤ fall_gravity_z_threshold`)
4. 모든 관절이 default에서 `entry_position_tolerance`(0.35 rad ≈ 20°) 이내일 것

(4)가 잡는 것: 관절 하나가 걸렸거나, 케이블이 빠졌거나, 로봇이 아직 스탠드에
매달린 채 다리가 늘어져 있는 상태. 그 상태로 정책을 켜면 첫 스텝에 큰 오차를
내면서 폭발한다. 조건을 못 맞추면 2초마다 warn을 찍으며 SAFE_HOLD에 머문다.

---

## 7. 하드웨어 계층: DYNAMIXEL

`description/kid_dynamixel.ros2_control.xacro`가 18개 관절을
`dynamixel_hardware_interface/DynamixelHardware`에 연결하는 xacro 매크로다.
이건 include용이라 로봇 URDF에서 호출해야 한다.

### 7.1 이 컨트롤러가 위치 명령만 쓰는 이유

DYNAMIXEL XL330은 내부에 위치 P 제어기를 갖고 있다. 우리는 목표 위치만 보내고
전류 제어는 모터 펌웨어가 한다. 즉 **강성(stiffness)은 `Position P Gain` 레지스터가
결정한다.**

microban 값:
- `KP_DEFAULT = 400` → 약 0.886 Nm/rad (단단함, 일반 자세 제어용)
- `KP_RL = 125` → 약 0.277 Nm/rad (부드러움, 보행용)
- 변환: `KP_GAIN_PRM = 0.0022 Nm/rad per register unit`

**보행 정책은 학습 때의 강성으로 돌려야 한다.** 시뮬레이터의 PD 게인이
0.277 Nm/rad였는데 실기에서 0.886으로 돌리면 같은 목표 위치가 3배 토크를 만든다.
정책은 자기가 만든 힘을 예측하지 못하고 진동한다.

**현재 이 컨트롤러는 P gain을 바꾸지 않는다.** xacro에 `Position P Gain = 125`를
정적으로 박아 두었다. 즉 이 로봇은 항상 RL 강성으로 동작한다. 모드에 따라 바꾸려면
gpio command interface가 필요하다 (9절 TODO).

### 7.2 Drive Mode = 부호

xacro의 `sign` 인자가 `Drive Mode` 레지스터의 bit 0(Normal/Reverse)을 설정한다.
모터가 물리적으로 반대로 붙어 있는 관절을 하드웨어 레벨에서 뒤집는 것이다.
microban의 `MOTOR_SIGN` 딕셔너리와 대응한다.

> **이 부호는 반드시 실물에서 하나씩 검증해야 한다.** 부호가 뒤집힌 관절 하나가
> 보행 정책 전체를 망친다. 검증법: torque를 끈 상태에서 관절을 사람이 +방향으로
> 돌리고 `/joint_states`가 증가하는지 본다. URDF의 회전축 정의와 일치해야 한다.

### 7.3 update_rate는 버스가 정한다

`update_rate: 200`은 **아직 측정하지 않은 추정값이다.**

18개 모터에 sync read(Present Position + Velocity + Current)를 하고, Goal Position이
바뀐 틱에는 sync write도 한다. 걸리는 시간은 baud rate와 패킷 크기가 정한다.
1 Mbps에서 대략:

- sync read 응답: 모터당 상태 패킷, 18개면 수백 바이트
- 왕복 + 처리 시간이 5 ms를 넘으면 200 Hz는 불가능

**측정 방법**: `ros2 control list_hardware_components`로 컴포넌트를 확인하고,
`~/controller_manager` 노드의 실행 통계(pal_statistics 토픽 또는
`ros2 topic echo /controller_manager/introspection_data/...`)에서 실제 주기와
overrun 횟수를 본다. overrun이 나면 `update_rate`를 낮추거나 baud를 올린다.
`policy_rate_hz`는 `update_rate` 이하여야 하고, 지금 코드는 그걸 configure에서 검사한다.

---

## 8. 코드 지도

### 8.1 파일

| 파일 | 역할 |
| --- | --- |
| `include/kid_rl/kid_rl_controller.hpp` | 클래스 선언, 관측 레이아웃 주석, Mode enum |
| `src/kid_rl_controller.cpp` | 전체 구현 |
| `config/controllers.yaml` | controller_manager + 컨트롤러 파라미터 |
| `description/kid_dynamixel.ros2_control.xacro` | 하드웨어 매크로 (include용) |
| `kid_rl_plugin.xml` | pluginlib 선언 |
| `test/mock/kid_mock.urdf` | 하드웨어 없는 18-DoF mock 로봇 |
| `test/smoke_test.sh` | 상태 기계 전체를 도는 스모크 테스트 |

### 8.2 함수 흐름

```
on_init()            파라미터 선언만
  ↓
on_configure()       파라미터 검증 → 벡터 전부 할당 → 구독 생성
  │                  · dry_run_policy=false면 ERROR (백엔드 없음)
  │                  · policy_rate_hz > update_rate면 ERROR
  │                  · default_positions가 joint limit 밖이면 ERROR
  ↓
on_activate()        map_interfaces() 이름으로 인덱스 매핑
  │                  read_joint_state() 측정 자세 스냅샷
  │                  set_mode(READY_POSE)
  ↓
update(time, period) ┌ read_joint_state()          실패 → FAULT
     매 틱           ├ 버퍼 3개 스냅샷 (1회씩)
                     ├ freshness 계산 (정수 ns 비교)
                     ├ switch(mode_) { ... }
                     └ write_commands()            유한성 + limit clamp
```

`update()`의 `switch`는 **모든 Mode를 명시적으로 처리한다.** `default:`가 없어서
새 상태를 추가하면 컴파일러가 `-Wswitch`로 잡아 준다.

---

## 9. 파라미터 레퍼런스

`config/controllers.yaml`의 `kid_rl: ros__parameters:` 아래.

| 파라미터 | 타입 | 기본 | 의미 / 주의 |
| --- | --- | --- | --- |
| `joints` | string[] | (필수) | **학습 순서 그대로.** 4.1절 |
| `default_positions` | double[] | (필수) | 기준 자세 [rad]. ONNX `default_joint_pos`와 일치해야 함 |
| `joint_position_min` | double[] | 전부 −π | 생략하면 ±π. **실기에서는 반드시 실측값을 채울 것** |
| `joint_position_max` | double[] | 전부 +π | 위와 동일 |
| `policy_rate_hz` | double | 50.0 | **학습 dt와 일치.** controller_manager update_rate 이하 |
| `ready_pose_duration_s` | double | 2.0 | 활성화 자세 → default 보간 시간 |
| `command_timeout_s` | double | 0.25 | 초과 시 속도 명령 0 (fault 아님) |
| `imu_timeout_s` | double | 0.10 | 초과 시 FAULT |
| `action_scale` | double | 1.0 | 학습 설정에서 가져올 것 |
| `action_clip` | double | 1.0 | 정책 출력 진폭 제한 |
| `target_rate_limit` | double | 4.0 | 목표 변화율 [rad/s]. 매 컨트롤러 틱에 적용 |
| `target_filter_alpha` | double | 1.0 | **정책 주기 1회 기준** LPF 계수 (0, 1]. 1.0 = LPF 비활성화(기본값) |
| `fall_gravity_z_threshold` | double | −0.5 | `g_z`가 이보다 크면 낙상 후보 |
| `fall_debounce_s` | double | 0.10 | 낙상 확정까지 연속 유지 시간 |
| `entry_position_tolerance` | double | 0.35 | 정책 진입 허용 자세오차 [rad]. ≤0이면 검사 생략 |
| `use_gait_phase` | bool | false | 모델이 phase 관측을 쓸 때만 |
| `phase_total_steps` | int | 20 | 참조 보행 한 주기의 policy step 수 |
| `dry_run_policy` | bool | true | **백엔드 링크 전에는 true만 허용.** false면 configure 실패 |

> ROS 2 파라미터는 **빈 리스트 `[]`의 타입을 추론하지 못한다.**
> `joint_position_min: []` 라고 쓰면 `No parameter value set` 예외로 컨트롤러 로드가
> 실패한다. 값을 안 넣을 거면 **키 자체를 생략**해야 한다.

---

## 10. 브링업 절차

> 실시간 설정(rtprio, CPU 격리, FTDI latency timer, baud rate)은
> [`THREADING.md` 5절 체크리스트](THREADING.md)를 함께 볼 것. 400 Hz는 코드만으로
> 되지 않는다.

### 10-0. 빌드

```bash
cd ~/deploy_ws
colcon build --packages-select kid_rl --symlink-install
source install/setup.bash
```

### 10-1. mock 하드웨어 (지금 가능)

```bash
bash src/kid_rl/test/smoke_test.sh
```

기대 출력:

```
[WARN]  [kid_rl]: Configured 18 joints, observation size 63, policy 50.0 Hz. DRY RUN: ...
[INFO]  [kid_rl]: Mode WAITING_FOR_STATE -> READY_POSE (activated)
[INFO]  [kid_rl]: Mode READY_POSE -> SAFE_HOLD (ready pose reached)
[INFO]  [kid_rl]: Mode SAFE_HOLD -> POLICY_RUNNING (enable_policy=true and entry checks passed)
[WARN]  [kid_rl]: cmd_vel is stale; commanding zero velocity
[ERROR] [kid_rl]: Fault: IMU watchdog expired. Freezing the joint targets ...
```

그리고 `/joint_states`가 `default_positions`와 일치해야 한다.

### 10-2. 실기 preflight (모터 토크 OFF)

정책을 켜지 않고 배선/부호/스케일부터 검증한다.

1. `ros2 control list_hardware_interfaces` — 18×(position, velocity) state와
   18×position command가 전부 `available`인가
2. 관절을 손으로 움직이며 `/joint_states` 확인 — **관절 이름과 실제 관절의 대응**,
   **부호**, **단위(rad)**, **영점**
3. `ros2 topic echo /kid_rl/imu` — 로봇을 똑바로 세웠을 때 orientation ≈ (1,0,0,0)인가.
   앞으로 기울이면 어느 축이 변하는가 (4.5절)
4. `joint_position_min/max`를 실측해서 yaml에 채운다

### 10-3. 스탠드 위 (로봇 매달고, 발 안 닿게)

1. `activate` → READY_POSE가 도는지, 자세가 default로 부드럽게 가는지
2. `enable_policy: true` → POLICY_RUNNING 진입 로그. dry run이라 움직이지 않아야 정상
3. IMU 퍼블리셔를 끊어서 FAULT가 걸리는지
4. 로봇을 기울여 낙상 감지가 걸리는지

### 10-4. ONNX 백엔드 붙인 뒤

1. **golden I/O 테스트 먼저.** 학습 쪽에서 (관측, 행동) 쌍을 몇 개 덤프해서
   C++ 추론이 같은 값을 내는지 비교한다. 여기서 안 맞으면 실기에 올리지 않는다.
2. 스탠드 위에서 정책 켜고 다리가 보행 패턴을 그리는지 육안 확인
3. 바닥에 내리되 **사람이 잡고**, `cmd_vel = 0`으로 제자리 서기부터
4. 그다음 아주 작은 `vx`

---

## 11. 자주 하는 실수 / 디버깅

| 증상 | 흔한 원인 |
| --- | --- |
| 컨트롤러 로드 실패, `No parameter value set` | yaml에 빈 리스트 `[]` (9절 마지막 note) |
| `Interface count mismatch` | URDF의 `<ros2_control>`에 관절이 빠졌거나 이름이 다름 |
| `Missing state interface X/velocity` | URDF에 `<state_interface name="velocity"/>`가 없음 |
| configure에서 `dry_run_policy=false but no policy backend` | 의도된 동작. 백엔드 링크 전까지 true |
| SAFE_HOLD에서 안 넘어감, 2초마다 warn | 진입 게이트 실패. IMU 없음 / 안 직립 / 자세오차 큼 (6.6절) |
| 정책 켜자마자 로봇이 폭발 | 관절 순서, 부호, `action_scale`, `default_positions`, KP 중 하나 |
| 걷다가 진동 | `policy_rate_hz`가 학습 dt와 다름, 또는 필터 지연 과다 |
| 정상 보행 중 nuisance FAULT | `fall_gravity_z_threshold` 너무 보수적, `fall_debounce_s` 너무 짧음 |
| 목표가 계단처럼 움직임 | 정상 (zero-order hold, 5.3절). 문제면 보간 추가 |

디버깅에 쓸 것:

```bash
ros2 control list_controllers                 # 상태 확인
ros2 control list_hardware_interfaces         # claim 상태
ros2 topic echo /joint_states                 # 실제 관절
ros2 param get /kid_rl policy_rate_hz         # 파라미터가 실제로 먹었는지
ros2 control set_controller_state kid_rl inactive   # FAULT 탈출
ros2 control set_controller_state kid_rl active
```

> 현재 컨트롤러는 내부 상태(mode, observation, action, freshness)를 발행하지 않는다.
> 로그로만 볼 수 있다. `~/status` 퍼블리셔가 우선순위 높은 TODO다.

---

## 12. 참조 구현에서 가져올 것

### ROBOTIS `ai_sapiens_sim2real`

정책 실행을 거대한 컨트롤러 하나가 아니라 역할별로 쪼갠다:
sensor handle(freshness 검사 포함), observation manager(YAML로 term 순서/scale/clip/history
정의), policy runtime(ONNX 사전 로드, 추론 실패 카운트), action pipeline(항목별
scale+offset+clip), mode controller(Damping/ReadyPose/Velocity/Mimic), command publisher.
`ros2_control`은 1 kHz joint impedance controller와 broadcaster에만 쓰고 정책은
별도 루프에서 돈다. 센서 stale / 잘못된 자세 / action limit / 반복 추론 실패는
전부 **Damping** 으로 간다.

### `roa_controller`

패키지 경계로 나눈다: `roa_policy_driver`(typed observation/action, ONNX dimension 검사,
golden I/O + stress test), `roa_main_controller`(관측 구성, 50 Hz 추론, target LPF),
hardware command loop(300 Hz), `roa_system_manager`(BOOT/RUN/SAFE_HOLD/ESTOP).
입력별로 별도 timeout(IMU, gravity, motor state, velocity command)을 둔다.
ONNX는 configure에서 한 번만 로드하고 RT 경로에서는 고정 버퍼로 `run()`만 호출한다.

우리가 채택한 것: 다중 주기, 상태 기계, 입력별 timeout, action 파이프라인, 진입 게이트.
아직 안 한 것: typed PolicyContract, observation term YAML 구성, damping 상태,
관측 history, 상태 발행, 테스트. 상세는 `WORKLOG.md`.

---

## 부록: microban 대조표

| 항목 | microban (`src/`) | kid_rl |
| --- | --- | --- |
| 관절 순서 | `constants.py: OBSERVATION_DOF_ORDER` | `controllers.yaml: joints` |
| 기준 자세 | ONNX metadata `default_joint_pos` (fallback `NEUTRAL_POSE`) | `controllers.yaml: default_positions` |
| 관측 구성 | `moves/walk.py: build_observation()` | `build_observation()` |
| projected gravity | `observer.py` + `imu_reader.py: quat_apply_inverse` | `imu_callback()` |
| 낙상 판정 | `walk.py: _projected_gravity_z_threshold = -0.5`, debounce 없음 | 같은 임계 + `fall_debounce_s` |
| gait phase 게이트 | `mean(|vx|,|vy|,|vθ|) > 0.01` | 동일 (mean) |
| action → 명령 | `default_pose + action * action_scale` | 동일 + clip/LPF/rate limit/joint limit |
| 제어 주기 | 50 Hz, IMU 200 Hz 별도 스레드 | update 200 Hz, 정책 50 Hz decimation |
| 과전류 보호 | `OVERCURRENT_CUTOFF_A` + debounce | **없음 (TODO)** |
| KP 전환 | `KP_DEFAULT 400 ↔ KP_RL 125` | **없음 (xacro에 125 고정)** |
| IMU 마운트 회전 | `IMU_MOUNT_QUAT` | **없음 (드라이버가 했다고 가정)** |
