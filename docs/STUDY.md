# RL 보행 배포 스터디 노트

> 이 문서는 코드 문서가 아니라 **읽는 사람을 위한 공부 자료**다.
> 1부는 배경지식, 2부는 microban / ai_sapiens / roa_controller 세 구현의 해부,
> 3부는 셋을 나란히 놓고 비교하며 `kid_rl`이 어디에 서 있는지 정리한다.
>
> 패키지 코드 자체의 설명은 [`CONTROLLER_GUIDE.md`](CONTROLLER_GUIDE.md),
> 스레드·실시간은 [`THREADING.md`](THREADING.md), 작업 이력은
> [`WORKLOG.md`](WORKLOG.md)에 있다.
>
> 참조 저장소
> - microban — https://github.com/Rhoban/microban
> - ai_sapiens — https://github.com/ROBOTIS-GIT/ai_sapiens
> - roa_controller — https://github.com/WJJJ2004/roa_controller
>
> 아래 서술은 2026-08-31 기준 각 저장소의 기본 브랜치를 직접 읽고 정리한 것이다.

---

# 1부 — 기본 배경지식

## 1.1 "강화학습 보행 정책"은 결국 무엇인가

거창하게 들리지만, 배포하는 입장에서 정책은 **하나의 순수 함수**다.

```
action = π(observation)
```

- `observation`: 로봇의 현재 상태를 납작하게 편 고정 길이 float 벡터
- `action`: 관절 개수만큼의 float
- `π`: 보통 2~3층짜리 MLP. K1급이라도 수십만 파라미터, CPU에서 수십 µs면 돈다.

기억할 것: **정책에는 상태가 거의 없다.** LSTM을 쓰지 않는 한 내부 메모리가 없고,
"직전 action"과 "관측 history"를 입력으로 넣어 주는 방식으로 기억을 흉내낸다.
그래서 배포 코드가 그 기억을 대신 관리해 줘야 한다.

### 왜 토크가 아니라 위치를 출력하나

거의 모든 보행 정책은 **관절 목표 위치**를 출력한다.

```
q_target = q_default + action_scale × action
```

그리고 그 목표를 PD 제어기가 토크로 바꾼다.

```
τ = kp × (q_target − q) − kd × q̇
```

토크를 직접 출력하지 않는 이유:

1. **PD가 고주파를 담당한다.** 정책은 50 Hz로 느리게 돌아도, PD는 1 kHz 이상(모터
   펌웨어 안이면 수 kHz)으로 돌면서 그 사이를 메꾼다. 정책이 모든 것을 할 필요가 없다.
2. **탐색이 쉽다.** 위치 공간은 액션이 조금 틀려도 로봇이 부드럽게 움직인다.
   토크 공간은 조금만 틀려도 발산한다.
3. **안전하다.** 위치 명령은 clamp하면 물리적 한계 안에 갇힌다. 토크는 clamp해도
   적분되어 어디로든 간다.

> **여기서 나오는 함정**: `kp`, `kd`는 **정책 계약의 일부다.** 학습 때 `kp=100`이었는데
> 실기에서 `kp=400`이면 같은 `q_target`이 4배 토크를 만든다. 정책은 자기가 만든 힘을
> 예측하지 못하고 진동한다. 세 구현 모두 이걸 명시적으로 다룬다 (microban은 보행 시작 시
> KP를 400→125로 낮추고, ai_sapiens는 YAML에 관절별 stiffness/damping을 박아 둔다).

## 1.2 관측 벡터의 각 항목이 왜 거기 있는가

전형적인 locomotion 관측은 이 여섯 덩어리로 이루어진다.

| 덩어리 | 크기 | 의미 |
| --- | --- | --- |
| base angular velocity | 3 | 몸통 각속도 (자이로 원시값) |
| projected gravity | 3 | 몸통 좌표계에서 본 중력 방향 |
| velocity command | 3 | 사용자가 원하는 `vx, vy, ωz` |
| joint position (relative) | N | `q − q_default` |
| joint velocity | N | `q̇` |
| last action | N | 직전 정책 출력 |

### projected gravity — 이 문서에서 제일 중요한 개념

몸통 자세를 나타내는 방법은 여럿이다: 쿼터니언 4개, 오일러각 3개, 회전행렬 9개.
그런데 보행 정책은 **로봇이 어느 방향을 보고 있든 똑같이 행동해야 한다.**
북쪽을 보든 남쪽을 보든 앞으로 걷는 법은 같다. 즉 yaw는 관측에 **없어야** 한다.

projected gravity는 세계 좌표계의 중력 방향 `(0,0,−1)`을 몸통 좌표계로 옮긴 것이다.

```
g_body = R(q)ᵀ · (0, 0, −1)
```

몸통을 세계 z축으로 ψ만큼 돌리면 `R' = Rz(ψ)·R` 이고,

```
g'_body = R'ᵀ·g_world = Rᵀ·Rz(ψ)ᵀ·(0,0,−1) = Rᵀ·(0,0,−1) = g_body
```

`Rz`는 z축 벡터를 바꾸지 않으므로 **yaw에 완전히 불변**이면서 roll/pitch는 그대로 담는다.
딱 필요한 정보만 3개 숫자로. 오일러각처럼 짐벌락도 없고, 쿼터니언처럼 부호 모호성
(`q`와 `−q`가 같은 회전)도 없다.

쿼터니언에서 바로 계산하려면, `R(q)`의 3행이

```
[ 2(xz − wy),  2(yz + wx),  1 − 2(x² + y²) ]
```

이므로 `Rᵀ·e_z`가 그 3행이고, −1을 곱하면 끝이다.

```cpp
g_body = { -2*(x*z - w*y), -2*(y*z + w*x), -(1 - 2*(x*x + y*y)) };
```

직립일 때 `(0, 0, −1)`. 옆으로 누우면 `g_z → 0`. 뒤집히면 `g_z → +1`.
**그래서 세 구현이 전부 `g_z > −0.5`를 낙상 판정으로 쓴다** (`cos 60° = 0.5`, 즉 60° 이상 기울어짐).

### q − q_default (절대각이 아니라 편차)

두 가지 이득이 있다. (1) 대부분의 값이 0 근처라 신경망 입력 정규화가 쉽다.
(2) 기준 자세를 바꿔도 정책이 "중립"의 의미를 다시 배울 필요가 없다.

`q_default`는 학습 때 쓴 값과 **비트 단위로** 같아야 한다. microban은 이걸 아예
ONNX 메타데이터(`default_joint_pos`)에서 읽어서 사람이 옮겨 적을 여지를 없앤다.
좋은 습관이다.

### last action — 신경망에 주는 값싼 메모리

정책에게 "네가 방금 뭘 시켰는지"를 알려준다. 없으면 정책은 자기 명령과 실제 관절
상수 태의 지연 관계를 추론할 없어서 진동하기 쉽다.

> **미묘한 함정**: 여기 넣는 값은 **clip 전 raw 출력**이어야 한다. 학습 때 정책이
> 관측한 것은 네트워크 원본 출력이다. 안전을 위해 clip한 값을 되먹이면 관측 분포가
> 학습과 어긋난다. 세 구현 모두 raw를 저장한다 (roa의 주석에 아예
> `last_action{}; // raw prev action`이라고 적혀 있다).

### observation scale — 정규화는 학습 설정의 일부다

ai_sapiens의 YAML을 보면 항목마다 scale이 다르다.

```yaml
base_ang_vel:   scale: [0.2, 0.2, 0.2]
projected_gravity: scale: [1.0, 1.0, 1.0]
joint_pos_rel:  scale: [1.0, ...]
joint_vel_rel:  scale: [0.05, 0.05, ...]
last_action:    scale: [1.0, ...]
```

이건 임의의 숫자가 아니라 IsaacLab locomotion 예제의 표준값이다. 각속도는 rad/s로
수 rad/s까지 나오고 관절 속도는 수십 rad/s까지 나오므로, 신경망 입력을 대략 O(1)로
맞추려고 각각 0.2, 0.05를 곱한다. **배포 코드가 이 곱셈을 빼먹으면 정책은
10~20배 큰 입력을 받고 즉시 폭주한다.**

microban과 roa는 scale이 전부 1.0이라 이 단계가 없는 것처럼 보이는데, 사실은
**학습 쪽에서 이미 1.0으로 설정한 것**이지 "scale이란 개념이 없는" 게 아니다.

### observation history

ai_sapiens는 모든 항목에 `history_length: 5`를 준다. 즉 현재 + 과거 4스텝을
쌓아서 넣는다. K1(23 DoF)의 한 스텝 관측이 `3+3+3+23+23+23 = 78`이니
실제 입력은 `78 × 5 = 390`차원이다.

왜? 로봇 상태는 부분관측(POMDP)이다. 한 순간의 자세와 속도만으로는 접촉 상태나
지면 특성을 알 수 없다. history를 주면 정책이 그걸 암묵적으로 추정한다.
LSTM보다 구현이 단순하고 추론 시간이 일정하다는 게 장점이다.

**배포 관점에서 history는 링버퍼 하나가 늘어나는 것이고, 리셋 정책이 중요하다.**
정책을 껐다 켤 때 history를 지우지 않으면 옛날 데이터가 섞여 들어간다.

### gait phase (선택)

일부 모델은 참조 보행 주기의 위상을 `cos, sin` 쌍으로 받는다. 각도 대신 삼각함수
쌍을 쓰는 이유는 주기의 끝과 시작(2π ↔ 0)에서 점프가 없기 때문이다.
명령 속도가 거의 0이면 위상을 0으로 리셋해서 제자리에 서게 한다.

## 1.3 주기: policy dt와 control dt

여기가 초보가 제일 많이 틀리는 곳이다.

**정책 주기는 성능 문제가 아니라 계약 문제다.**

학습 때 정책은 `step_dt = 0.02 s`(50 Hz) 간격으로 행동했다. 실기에서 200 Hz로 돌리면
정책이 보는 "한 스텝"의 물리적 의미가 4배 달라지고, last action 되먹임의 시간 상수도
어긋나서 발산한다. **학습 dt와 반드시 일치시켜야 한다.**

반면 **제어 주기(하드웨어 read/write)는 높을수록 좋다.** 관절 추종이 부드럽고
지연이 작다. 상한은 버스 대역폭이 정한다.

그래서 거의 모든 실기 스택이 **두 주기를 분리한다.**

| 구현 | 빠른 루프 | 정책 |
| --- | --- | --- |
| microban | 50 Hz (분리 안 함) + IMU 200 Hz 스레드 | 50 Hz |
| roa_controller | 300 Hz | 50 Hz |
| ai_sapiens | 1000 Hz | 50 Hz (`step_dt: 0.02`) |
| **kid_rl (목표)** | **400 Hz** | **50 Hz** |

### decimation 구현 — 카운터가 아니라 시간 누적

세 구현 중 두 개(ai_sapiens, kid_rl)가 **같은 패턴**을 쓴다.

```cpp
accumulated_period_ += period.seconds();
if (accumulated_period_ < step_dt_) { return; }   // 이번 틱은 정책 안 돌림
accumulated_period_ = std::fmod(accumulated_period_, step_dt_);
// ... 여기서 추론 ...
```

`if (++tick % 8 == 0)` 같은 카운터를 안 쓰는 이유:

- 제어 주기가 정확히 400 Hz가 아닐 수 있다 (지터)
- `update_rate`를 바꿔도 정책 주기는 50 Hz로 유지되어야 한다
- 큰 시간 점프가 있어도 `fmod`가 "밀린 스텝"을 몰아서 돌지 않게 잘라 준다
  (ai_sapiens 주석: *"Drop whole elapsed steps without looping on large time jumps"*)

### 그 사이 틱에서는 뭘 하나

여기서 설계가 갈린다.

- **아무것도 안 함 (zero-order hold)**: 마지막 정책 목표를 그대로 반복해서 쓴다.
  구현이 제일 쉽다. 관절이 계단 응답을 하지만 PD가 저역통과 역할을 한다.
- **목표를 slew한다**: 정책 목표를 *setpoint*로 두고, 매 빠른 틱마다 저역통과 필터와
  변화율 제한을 걸어 그쪽으로 조금씩 접근한다. 계단이 매끄러운 램프가 된다.

`kid_rl`은 후자를 택했다 (`update_policy_setpoint()` 50 Hz + `slew_targets()` 400 Hz).
roa도 `policy_target_lpf: cutoff_hz: 1.5`로 비슷한 것을 한다.

> **필터를 rate-independent하게 만드는 법.** `y += α(u − y)`에서 α는 호출 주기에
> 종속이다. 50 Hz에서 α=0.2였던 걸 400 Hz에서 그대로 쓰면 필터가 8배 빨라진다.
> 시간 상수로 바꿔서 저장해야 한다.
>
> ```
> τ = −T / ln(1 − α)        (T = 원래 주기)
> α_tick = 1 − exp(−dt / τ)  (dt = 실제 틱 주기)
> ```
>
> `kid_rl`의 `filter_time_constant_s_`가 이것이다.

## 1.4 실시간이란 무엇인가

**"빠르다"가 아니라 "늦지 않는다"이다.** 평균 0.5 ms에 도는 루프라도 1000번에 한 번
5 ms가 걸리면, 400 Hz(2.5 ms 예산) 시스템에서는 실패다. 관심사는 평균이 아니라
**최악값(worst case)과 지터**다.

리눅스에서 이걸 망치는 것들:

| 원인 | 증상 | 대책 |
| --- | --- | --- |
| 일반 스케줄링(CFS) | 다른 프로세스에 선점당함 | `SCHED_FIFO` + 우선순위 |
| page fault | 스왑/지연 할당으로 수 ms 정지 | `mlockall()` |
| 힙 할당 | `malloc`이 내부 락을 잡음 | RT 경로에서 할당 금지 |
| 우선순위 역전 | 저순위가 잡은 락을 고순위가 기다림 | RT 경로에서 뮤텍스 금지 |
| CPU 경합 | 다른 코어 작업, 인터럽트 | `isolcpus` + affinity |
| 로깅 | `stringstream`, 파일 I/O | throttle, RT-safe 로거 |

세 구현이 이걸 얼마나 신경 쓰는지가 성숙도의 척도다.

- **microban**: 파이썬. GIL, GC, `time.sleep()` 기반 페이싱. 실시간 보장 없음.
  대신 오버런을 감지해서 경고만 찍는다. 50 Hz라서 이걸로 충분하다.
- **roa_controller**: C++이지만 `SingleThreadedExecutor` + wall timer.
  RT 스케줄링/메모리 락 없음. 300 Hz라 아슬아슬하게 성립한다.
- **ai_sapiens**: 전용 RT 스레드 + `SCHED_FIFO` + `lock_memory` +
  `realtime_tools::realtime_helpers`. 1 kHz. **제대로 한 것.**

## 1.5 스레드 간 데이터 전달

문제는 항상 같다. 센서 콜백은 executor 스레드가 받고, 제어 루프는 RT 스레드가 돈다.
**RT 스레드는 절대 대기하면 안 된다.**

세 가지 접근:

1. **뮤텍스 latch** (roa의 `Latch<T>`) — `std::mutex`로 감싼 `shared_ptr` + 수신시각.
   간단하고 짧게 잡지만, 원리적으로 RT 스레드가 블록될 수 있다.
   ```cpp
   void set(SharedPtr msg, Time rx) { lock_guard lk(m_); msg_=move(msg); rx_time_=rx; }
   pair<SharedPtr,Time> get() const { lock_guard lk(m_); return {msg_, rx_time_}; }
   ```
2. **뮤텍스 snapshot** (microban의 `ThreadedIMUReader`) — 같은 아이디어의 파이썬판.
   `threading.Lock`으로 감싼 불변 `IMUSnapshot` 통째 교환. GIL이 있으니 어차피 논점 밖.
3. **lock-free double buffer** (`realtime_tools::RealtimeBuffer`, kid_rl) — 버퍼 두 개.
   writer는 노는 쪽에 쓰고 플래그를 세우고, reader는 플래그가 서 있으면 포인터를 swap한다.
   RT 쪽이 **절대 대기하지 않는다.** 대신 오래된 데이터를 읽을 수 있으니
   타임스탬프와 watchdog이 필수다.

> **RealtimeBuffer의 함정**: `readFromRT()`가 포인터를 swap하므로, 첫 호출로 받은
> 포인터는 다음 호출 이후 유효하지 않을 수 있다. **한 틱에 버퍼당 한 번만 읽고
> 값으로 복사할 것.**

## 1.6 안전장치의 계층

실기에서 정책 출력이 관절에 닿기까지 몇 겹을 통과해야 하는가. 각 층이 막는 것이 다르다.

```
정책 raw 출력
  │
  ├─ (1) 유한성 검사 ──────── NaN/Inf → 고장 처리
  ├─ (2) raw clip ─────────── 학습 분포 밖 입력에 대한 이상 출력 (sapiens: ±5.0)
  │
  │   q_target = q_default + scale × action
  │
  ├─ (3) joint limit clamp ── 기구 파손, 링크 간섭
  ├─ (4) 저역통과 필터 ────── 고주파 진동, 액추에이터 보호
  ├─ (5) 변화율 제한 ──────── 급격한 목표 점프
  ▼
하드웨어
```

그리고 **입력 쪽 watchdog**:

| 입력 | 없으면 | 왜 |
| --- | --- | --- |
| IMU | 치명적 | 자세를 모르면 어떤 명령도 안전하지 않다. 대체값 없음 |
| 관절 상태 | 치명적 | 관측 자체가 성립 안 함 |
| 속도 명령 | 치명적이지 않음 | `(0,0,0)`으로 대체하면 정책이 제자리에 선다 |

이 비대칭이 중요하다. 조이스틱 250 ms 딸꾹질에 걷던 휴머노이드를 정지시키면
오히려 넘어진다.

그리고 **고장 시 무엇으로 갈 것인가**. 위치를 동결하면 넘어진 로봇이 계속 최대
토크를 낸다. 정답은 **damping** — 강성을 0으로, 감쇠만 남겨서 관절이 자유롭게
접히게 한다. ai_sapiens와 roa 둘 다 이 상태를 명시적으로 갖고 있다.
microban은 아예 토크를 끈다.

## 1.7 ONNX Runtime을 실시간 루프에서 쓰기

ONNX는 신경망 그래프와 가중치를 프레임워크 독립적으로 담는 포맷이고,
ONNX Runtime(ORT)이 그걸 CPU에서 실행한다. 실시간 관점에서 알아야 할 것:

1. **세션 로드는 configure에서 한 번만.** `Ort::Session` 생성은 파일 I/O, 파싱,
   그래프 최적화, 메모리 할당을 전부 한다. 수십~수백 ms. RT 경로 근처에도 두면 안 된다.
2. **스레드풀을 반드시 1로 묶어라.** ORT는 기본적으로 코어 수만큼 intra-op 스레드를
   띄우고, 그 스레드들이 **작업이 없어도 spin**한다. RT 루프가 도는 코어를 훔쳐 간다.
   ```cpp
   session_options.SetIntraOpNumThreads(1);
   session_options.SetInterOpNumThreads(1);
   session_options.SetExecutionMode(ORT_SEQUENTIAL);
   ```
   ai_sapiens 코드에 그대로 있다 — 주석까지 달려 있다:
   *"Keep ONNX Runtime single-threaded so policy latency stays predictable."*
   roa도 `Options{ intra_op_threads = 1, inter_op_threads = 1 }`이 기본값이다.
   MLP 하나 돌리는 데 멀티스레드는 오버헤드만 늘린다.
3. **입출력 버퍼를 미리 잡아라.** `Run()`에 매번 `std::vector`를 넘기면 할당이 생긴다.
   고정 크기 배열 + `Ort::Value::CreateTensor`로 미리 만들어 두고 값만 갱신한다.
   roa의 `TypedPolicyDriver`가 `std::array<float, kObsDim> obs_buf_`를 멤버로 들고 있는 게 이것이다.
4. **추론 시간을 반드시 측정하라.** 400 Hz면 한 틱이 2.5 ms다. 직렬 통신 1.2 ms +
   추론 1.5 ms = 2.7 ms면 정책이 도는 8틱마다 한 번씩 오버런한다.

## 1.8 정책 계약을 코드로 강제하기

이 절이 실전에서 제일 값어치가 있다.

관절 순서 하나, 부호 하나, scale 하나가 틀리면 로봇은 **부드럽게 실패하지 않는다.**
그냥 넘어진다. 그리고 원인을 찾기가 극도로 어렵다. 그래서 성숙한 구현은
**설정 단계에서 계약을 강제로 검증한다.**

roa_controller가 이 방향으로 가장 멀리 갔다.

```cpp
// 컴파일 타임에 차원을 박는다
struct Policy12DofV2 {
  static constexpr int kObsDim = 45;   // 3 + 3 + 12 + 12 + 3 + 12
  static constexpr int kActDim = 12;
  static constexpr float kActionScale = 0.5f;
  struct Obs { array<float,3> gravity; array<float,3> cmd; ... };
  static void pack_obs(const Obs&, float* out);   // 순서를 한 곳에 고정
};

// 로드 시 모델과 대조
bool load(...) {
  if (core_.input_dim()  != Spec::kObsDim) return false;
  if (core_.output_dim() != Spec::kActDim) return false;
}
```

여기에 **golden I/O 테스트**를 붙인다. 학습 쪽에서 `(관측, 행동)` 쌍을 덤프해
`golden_obs.bin` / `golden_action.bin`으로 저장하고, C++ 추론이 같은 값을 내는지
CTest로 검증한다. **여기서 안 맞으면 실기에 올리지 않는다.**

microban은 다른 방향으로 같은 목적을 이룬다. 계약을 코드에 박는 대신
**ONNX 메타데이터에서 읽는다.**

```python
meta = self._ort_session.get_modelmeta().custom_metadata_map
names = meta["joint_names"].split(",")
positions = [float(v) for v in meta["default_joint_pos"].split(",")]
self._default_pose = dict(zip(names, positions))

# 입력 크기로 gait phase 사용 여부를 자동 판별
base_obs_size = 3 + 3 + 3 * len(OBSERVATION_DOF_ORDER) + 3
self._use_reference_phase = session.get_inputs()[0].shape[1] > base_obs_size
```

사람이 옮겨 적을 여지가 없다는 점에서 이쪽이 더 안전하다. **둘 다 하는 게 정답이다.**

---

# 2부 — 세 구현 해부

## 2.1 microban (Rhoban)

> https://github.com/Rhoban/microban — 소형 휴머노이드, XL330 × 19, Raspberry Pi, Python

### 한 줄 요약

**가장 단순하고 가장 읽기 쉬운 참조 구현.** 전부 파이썬, ROS 없음, 단일 50 Hz 루프.
"보행 정책 배포에 최소한 무엇이 필요한가"를 배우기에 최적.

### 구조

```
main.py
  └─ Scheduler(frequency_hz=50)         ← 단일 while 루프
       ├─ Observer ──────────► Controller ──► DYNAMIXEL (sync read/write)
       │     └─ ThreadedIMUReader (200 Hz, 별도 데몬 스레드)
       ├─ InputSource (게임패드/키보드)
       └─ registered_moves: { head, squat, walk }
              └─ WalkMove ──► onnxruntime (Python)
```

### 주기와 스레딩

- 메인 루프 **50 Hz**, `time.perf_counter()` + `time.sleep(dt − elapsed)`로 페이싱.
  늦으면 오버런 경고만 찍고 계속 간다.
- IMU만 **200 Hz 데몬 스레드**(`ThreadedIMUReader`). BMI088을 I2C로 읽고 Madgwick
  필터로 쿼터니언을 만든다. 스냅샷을 `threading.Lock`으로 교환하고, 메인 루프는
  `get_status()["age_s"]`로 신선도를 확인한다.
- 실시간 스케줄링, 메모리 락 없음. **파이썬이라 애초에 보장할 수 없다.**

> IMU만 별도 스레드로 뺀 이유가 흥미롭다. I2C 읽기는 느리고 블로킹인데, Madgwick
> 필터는 적분기라 **일정한 dt로 자주 돌아야** 자세가 안 튄다. 50 Hz 루프 안에서
> 같이 돌리면 필터 품질이 떨어진다. "센서 융합은 빠르게, 정책은 느리게"의 전형.

### 관측

`moves/walk.py: build_observation()`. 순서:

```
gyro(3) → projected_gravity(3) → q_rel(N) → q̇(N) → last_action(N) → cmd(3) → [phase(2)]
```

- 관절 순서는 `constants.py: OBSERVATION_DOF_ORDER` (팔 3 + 다리 6, 오른쪽 먼저).
- scale 없음 (전부 1.0).
- history 없음.
- `default_pose`는 **ONNX 메타데이터에서 읽는다** (fallback으로 `NEUTRAL_POSE` 상수).
- `use_reference_phase`는 **모델 입력 크기로 자동 판별**한다.

### 액션

```python
command.target_angles[name] = self._default_pose[name] + action[i] * self.action_scale
```

`action_scale = 1.0`. clip 없음, 필터 없음, 변화율 제한 없음. **날것 그대로 나간다.**

### 안전

- **낙상**: `projected_gravity[2] > −0.5`면 `step()`에서 그냥 `return`.
  즉 정책을 건너뛰고 **직전 목표를 유지**한다. fault 상태로 가지 않는다.
- **과전류**: 여기가 microban의 백미다.
  ```
  total_current ≥ OVERCURRENT_CUTOFF_A (15 A)가
  OVERCURRENT_DEBOUNCE_TICKS (2틱) 연속 → 루프 break → 전 모터 토크 OFF
  ```
  목적: 낙상 시 전 모터가 동시에 버티면서 나는 전류 스파이크로 **BMS가 트립되기 전에**
  로봇을 끊는 것.
  더 영리한 건 **전류를 읽지 않고도 추정한다**는 점이다. `Present Current`를 읽으면
  버스 트랜잭션이 늘어나므로, bam(XL330 m6) 모터 모델로 프록시를 만든다.
  ```
  duty = clip(KP × ERROR_GAIN × (target − q), ±1)
  I    = (VIN × duty − KT × q̇) / R,   |I| ≤ 1.75 A
  ```
  그리고 **지연 정렬**까지 한다. 피드백은 3틱 늦게 오므로, 지금 목표가 아니라
  `_cmd_history[0]`(3틱 전 명령)과 짝지어 계산한다. 안 그러면 보행 시작 시
  오차 항이 부풀어서 오검출이 난다. — 실기에서 데인 사람이 쓴 코드다.
- **KP 전환**: `on_start`에서 전 모터 P gain을 `KP_DEFAULT 400` → `KP_RL 125`,
  `on_stop`에서 되돌린다. 1.1절에서 말한 "게인은 계약의 일부"를 실행한 것.
- **통신 오류**: 3연속 실패면 루프 종료.
- **종료**: `finally`에서 무조건 전 모터 토크 OFF. `/tmp/microban_scheduler.stop`
  파일로 외부에서 정지 요청 가능.

### 하드웨어

DYNAMIXEL XL330, Protocol 2.0. `Observer.read_state()`가 `sync_read_present_position`과
`sync_read_present_velocity`를 **각각** 부른다 (라운드트립 2회). 50 Hz라 문제없지만
고주기로는 못 올라간다.

명령 속도 제한이 중앙집중되어 있는 것도 좋다: 입력 소스는 `[−1,1]` 정규화 값을 내고
`scale_velocity()`가 `VX_MAX 0.7 / VX_MAX_BACKWARD 0.5 / VY_MAX 0.3 /
VTHETA_MAX_STATIONARY 3.0 / VTHETA_MAX_MOVING 1.5`로 매핑한다.
제자리 회전일 때 회전 상한을 키우는 디테일까지 있다.

### 배울 것 / 안 따라할 것

**배울 것**
- ONNX 메타데이터에서 계약을 읽는 습관
- 모델 입력 크기로 관측 변형 자동 판별
- 과전류 프록시 + 지연 정렬 + debounce
- 모드별 KP 전환
- 명령 속도 스케일링을 한 곳에 모으기
- `finally`에서 무조건 토크 OFF

**안 따라할 것**
- 파이썬 단일 루프 (실시간 보장 없음, 고주기 불가)
- action clip / 필터 / 변화율 제한 부재
- 낙상 시 목표 동결만 하고 상태 전이가 없음

---

## 2.2 ai_sapiens (ROBOTIS)

> https://github.com/ROBOTIS-GIT/ai_sapiens — K1 휴머노이드, 23 DoF, ROS 2, C++

### 한 줄 요약

**셋 중 가장 정교하고 가장 배울 게 많은 구현.** 관측·액션·상태기계를 전부 YAML로
빼서 재컴파일 없이 모델을 바꿀 수 있게 만들었고, 실시간 처리를 제대로 한다.

### 구조 — 두 개의 프로세스, 두 개의 RT 루프

여기가 핵심이다. **ros2_control과 정책 런타임을 분리했다.**

```
┌──────────────────────────────────────────┐   ┌────────────────────────────────┐
│  ros2_control (controller_manager)       │   │  ai_sapiens_sim2real (별도 노드) │
│  update_rate: 1000, thread_priority: 80  │   │  자체 RT 스레드, control_rate    │
│                                          │   │  기본 1000 Hz, priority 50,     │
│  • joint_group_impedance_controller      │   │  lock_memory: true             │
│      position + velocity + kp + kd       │   │                                │
│  • imu_sensor_broadcaster                │◄──┤  sense → decide → act → command│
│  • rc_broadcaster (RC 조종기 16채널)      │   │                                │
│  • mujoco_hardware_interface 또는 실기    ├──►│  PolicyRuntime (step_dt 0.02)  │
└──────────────────────────────────────────┘   └────────────────────────────────┘
        1 kHz 임피던스 제어                          1 kHz 루프 안에서 50 Hz 추론
```

- `ros2_control` 쪽은 **저수준 임피던스 제어와 센서 브로드캐스트만** 한다.
- 정책은 `ai_sapiens_sim2real`이라는 **별도 노드**가 돌린다. 이 노드는 사실상
  자기만의 미니 controller_manager(`ControlLoop` 클래스)를 갖고 있다.
  주석에 대놓고 그렇게 써 있다: *"Similar to ros2_control's ControllerManager..."*

### ControlLoop — 자체 RT 루프의 구조

```cpp
void rt_thread_func();          // 전용 std::thread
void prepare_realtime_thread(); // SCHED_FIFO + mlockall
bool wait_for_sensors_ready();  // 필수 센서가 다 올 때까지 대기 (timeout 30 s)

// 매 틱:
void sense(t);              // 센서 핸들들이 SharedControlData를 채운다
const ModeDecision& decide(t, dt);  // ModeController: 모드 선택 + 속도 소스 선택
void act(decision, t, dt);          // PolicyController: 관절 명령 계산
void command(t);                    // 퍼블리셔가 하드웨어로 전송
void sleep_until_next_tick(next, period);  // steady_clock 기준 절대시각 페이싱
```

배울 점 몇 가지:

1. **`sense → decide → act → command` 단계 분리.** 각 단계가 `SharedControlData`의
   어느 부분을 건드리는지가 주석에 명시되어 있다. `act`는 `ModeDecision`을
   **const 참조**로 받아서 결정을 못 바꾸게 막는다.
2. **`wait_for_sensors_ready()`** — 필수 센서가 다 도착하기 전에는 추론을 시작하지
   않는다. 타임아웃 30초. 우리가 `entry_position_tolerance`로 하는 것의 일반화.
3. **`sleep_until_next_tick`이 절대 시각 기준**이다. `sleep(period)`가 아니라
   `next += period; sleep_until(next)`. 드리프트가 누적되지 않는다.
4. **`faulted_` 플래그** — RT 스레드가 예외로 죽으면 프로세스가 0이 아닌 코드로 종료한다.
   systemd가 재시작을 판단할 수 있다.
5. **`publish_emergency_damping()`** — 비상시 damping 명령을 직접 쏜다.

### 관측 — YAML로 정의되는 ObservationManager

이게 ai_sapiens의 최대 차별점이다. `assets/k1/locomotion/velocity/walk_default/params/sim2real.yaml`:

```yaml
step_dt: 0.02
observations:
  base_ang_vel:       { scale: [0.2, 0.2, 0.2],   history_length: 5 }
  projected_gravity:  { scale: [1.0, 1.0, 1.0],   history_length: 5 }
  velocity_commands:  { scale: [1.0, 1.0, 1.0],   history_length: 5,
                        params: { command_name: base_velocity } }
  joint_pos_rel:      { scale: [1.0 × 23],        history_length: 5 }
  joint_vel_rel:      { scale: [0.05 × 23],       history_length: 5 }
  last_action:        { scale: [1.0 × 23],        history_length: 5 }
```

- 항목이 **이름으로 등록**되고 YAML 순서대로 조합된다.
- 항목마다 `scale`, `clip`, `history_length`가 따로.
- 관절 속성도 같은 파일에: 관절별 `default_position`, `stiffness`, `damping`,
  `position_limit`.
- 모델을 바꾸면 **YAML만 바꾸면 된다.** 재컴파일 없음.

관측 차원: `(3+3+3+23+23+23) × 5 = 390`.

### 액션 — ActionPipeline

```yaml
actions:
  joint_pos:
    scale:    [0.25 × 23]
    offset:   [-0.205, -0.205, 0.0, ...]   # = default_position
    raw_clip: [[-5.0, 5.0] × 23]
    clip:     null
```

`raw_clip`(정책 출력 자체를 자름)과 `clip`(변환 후 결과를 자름)이 **분리되어 있다.**
그리고 전부 **관절별 배열**이다 — 우리처럼 전 관절 공통 스칼라가 아니다.

### 상태 기계 — YAML로 정의되는 계층적 FSM

```yaml
state_machine:
  initial: Damping            # ← 부팅 시 기본이 damping이다
  states:
    Damping:   { run: damping,          transitions: [{when: ReadyPoseRequested, to: ReadyPose}] }
    ReadyPose: { run: ready_pose,       transitions: [ Damping, Velocity, mimic_selector ] }
    ZeroPose:  { run: posture_zero_pose, ... }
    Velocity:  { run: velocity_policy,  transitions: [ Damping, ReadyPose, mimic ] }
    Mimic:     { abstract: true, transitions: [ Damping, ReadyPose, Velocity ] }
    MimicSquat:  { parent: Mimic, run: mimic_squat }
    MimicDance1: { parent: Mimic, run: mimic_dance1 }
```

**부모 상태(`abstract: true`)를 두고 공통 전이를 상속**시킨다. Mimic 계열 어디에
있든 Damping 요청은 항상 먹는다. 상태가 늘어나도 전이 표가 폭발하지 않는다.

각 상태의 행동도 YAML:

```yaml
state_behaviors:
  damping:
    kind: damping
    damping: { values: [[3.0 × 6], [3.0 × 6], [3.0], [3.0 × 5], [3.0 × 5]] }
    # stiffness 없음 = kp 0. 감쇠만 남기고 관절을 풀어 준다.
  ready_pose:
    kind: posture
    duration: 3.0
    stiffness: {...}   damping: {...}   target_position: {...}
```

**초기 상태가 Damping이라는 게 중요하다.** 켜면 로봇이 축 늘어져 있고, 사람이
명시적으로 ReadyPose를 요청해야 자세를 잡는다. 안전 설계의 기본.

### 권한(authority)과 게이트

- `authority`: API(원격)와 teleop(RC 조종기) 중 누가 명령권을 갖는지 관리.
  API 진입에 `warmup_duration: 3.0`, `velocity_neutral_threshold: 0.05`,
  `allowed_from_states: [Damping, ReadyPose, Velocity]` 조건을 건다.
- `startup_gate`, `realtime_request_gate`: 기동 조건과 RT 경로에서의 요청 처리.
- `teleop_input`은 **pluginlib 플러그인**이다 (Radiomaster, DualSense, 키보드).
  조종기를 바꿔도 코어를 안 건드린다.

### 정책 런타임

```cpp
accumulated_period_ += period.seconds();
if (accumulated_period_ < step_dt_) { return; }
accumulated_period_ = std::fmod(accumulated_period_, step_dt_);
// 추론
...
// 실패 시
++consecutive_inference_failures_;
if (consecutive_inference_failures_ >= kMaxConsecutiveInferenceFailures) {
  // Damping 요청
}
```

ONNX 설정:
```cpp
session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
session_options_.SetIntraOpNumThreads(1);
session_options_.SetInterOpNumThreads(1);
session_options_.SetExecutionMode(ORT_SEQUENTIAL);
```

### 테스트

`test_gait_clock`, `test_mode_controller`, `test_request_gate`, `test_startup_gate`,
`test_state_machine_config`, `test_motion_playback`, teleop 플러그인 테스트...
**상태 기계와 게이트에 단위 테스트가 붙어 있다.** 그리고
`scripts/run_mode_smoke_tests.py`로 모드 전환 스모크 테스트를 돌린다.

### 배울 것 / 안 따라할 것

**배울 것**
- 관측/액션/상태기계를 YAML로 빼기 (모델 교체에 재컴파일 불필요)
- 항목별 scale/clip/history
- 계층적 상태 기계 + abstract 부모 상태
- 초기 상태 = Damping
- `sense/decide/act/command` 단계 분리와 const 참조로 권한 고정
- 절대 시각 기반 페이싱
- 필수 센서 준비 대기
- 연속 추론 실패 카운터 → Damping
- ONNX 단일 스레드 + `ORT_SEQUENTIAL`
- 상태 기계 단위 테스트

**안 따라할 것 (우리 규모에서는)**
- 프로세스를 둘로 나누는 것 — 토픽 왕복 지연이 생긴다. 18 DoF 소형 로봇에
  이 정도 분리는 과하다. ai_sapiens는 mimic/teleop/API 등 요구사항이 훨씬 크다.
- YAML 설정 시스템 전체 (config 파서만 파일 10개 이상)

---

## 2.3 roa_controller (WJJJ2004)

> https://github.com/WJJJ2004/roa_controller — 12 DoF 휴머노이드, RobStride 모터 + SocketCAN,
> ROS 2 Humble, ONNX Runtime 1.23.2

### 한 줄 요약

**정책 드라이버를 라이브러리로 분리하고 golden I/O로 검증하는 접근이 백미.**
루프 구조 자체는 셋 중 가장 단순하다.

### 구조 — 패키지 경계로 나누기

```
lib/
  roa_policy_driver     ONNX 로더 + typed 정책 인터페이스 (ROS 의존 없음!)
  roa_packet_manager    모터 명령 패킷 빌더 / 상태 디코더
  roa_common            초기 자세, 관절 한계 상수
msgs/roa_interfaces     Motor/RSU/SystemStatus 메시지
ros2_pkg/
  roa_main_controller       관측 구성 + 추론 + 액추에이터 명령
  roa_system_manager        lifecycle과 RUN/SAFE_HOLD/ESTOP
  robstride_hardware_interface  SocketCAN 모터 인터페이스 (300 Hz)
  rsu_manager               RSU 발목 역기구학 / 임피던스 매핑
  imu_pkg                   EBIMU 시리얼 드라이버
```

**`roa_policy_driver`가 ROS에 의존하지 않는 순수 CMake 라이브러리**라는 게 핵심이다.
독립적으로 빌드·테스트·설치되고, 소비자는 `find_package(roa_policy_driver)`로 쓴다.
덕분에 ROS 없이 CTest만으로 정책 검증이 가능하다.

### 주기와 스레딩

```yaml
hw_rate_hz: 300.0
policy_rate_hz: 50.0
status_rate_hz: 10.0
rsu_timeout_ms / cmd_timeout_ms / imu_timeout_ms / gravity_timeout_ms
  / motor_state_timeout_ms / policy_cmd_timeout_ms: 50.0  # 전부 개별 타임아웃
```

구현은 **wall timer 세 개**다.

```cpp
hw_timer_     = create_wall_timer(1/300 s, ControlLoop);
policy_timer_ = create_wall_timer(1/50 s,  InferenceLoop);
status_timer_ = create_wall_timer(1/10 s,  publish_controller_status);
```

그리고 `SingleThreadedExecutor`로 돈다.

> **여기가 이 구현의 약점이다.** 세 타이머와 모든 구독 콜백이 **한 스레드에서
> 직렬화**된다. 즉 `InferenceLoop`가 도는 동안 `ControlLoop`는 못 돈다.
> 추론이 1 ms 걸리면 300 Hz 루프(3.3 ms 예산)에서 그 틱은 30%를 잡아먹는다.
> SCHED_FIFO도 mlockall도 없다.
>
> 대신 구현이 매우 단순하고, 300 Hz면 예산이 넉넉해서 실제로 동작한다.
> **"되는 것"과 "보장되는 것"의 차이**를 보여주는 좋은 예다.

센서 공유는 `Latch<MsgT>` — `std::mutex` + `shared_ptr` + 수신 시각.

### 관측 — 컴파일 타임 typed 계약

```cpp
struct Policy12DofV2 {
  static constexpr int kDof = 12;
  static constexpr int kObsDim = 45;        // 3 + 3 + 12 + 12 + 3 + 12
  static constexpr int kActDim = 12;
  static constexpr float kActionScale = 0.5f;

  struct Obs {
    array<float,3>    gravity;         // body frame 중력 단위벡터
    array<float,3>    cmd;             // vx, vy, yaw_rate
    array<float,kDof> q_rel;           // q - default
    array<float,kDof> qd_rel;          // q̇
    array<float,3>    imu_omega_body;  // gyro
    array<float,kDof> last_action;     // raw
  };
  static void pack_obs(const Obs& o, float* out) { /* memcpy 6번 */ }
  static void unpack_obs(const float* in, Obs& o) { /* 역변환 */ }
};
```

주목:

- **관측 순서가 microban과 다르다.** roa는 `gravity, cmd, q_rel, qd, gyro, last_action`,
  microban은 `gyro, gravity, q_rel, qd, last_action, cmd`.
  **모델마다 계약이 다르다는 산 증거다. 절대 추측하지 말 것.**
- `action_scale = 0.5` (microban 1.0, sapiens 0.25). 역시 모델마다 다르다.
- `unpack_obs`가 있는 이유는 **round-trip 테스트** 때문이다
  (`test_iface_roundtrip_12dof.cpp`). pack → unpack → 원본과 같은가.
- `action_to_q_target` / `q_target_to_action` 양방향 변환이 있어서
  디버깅 시 "이 관절 각도가 어떤 action에 해당하는가"를 역산할 수 있다.

### TypedPolicyDriver — RT 안전 래퍼

```cpp
template <class Spec>
class TypedPolicyDriver {
  static_assert(is_valid_spec<Spec>::value, "Spec must define kObsDim/kActDim > 0");
public:
  bool load(const string& path, const Options& opt) {
    if (!core_.load(path, opt)) return false;
    if (core_.input_dim()  != Spec::kObsDim) return false;  // 모델과 대조
    if (core_.output_dim() != Spec::kActDim) return false;
    return true;
  }
  bool run(const Obs& obs, Act& act) noexcept {   // "RT loop용: 성공 경로에서 동적할당 없음"
    Spec::pack_obs(obs, obs_buf_.data());
    if (!core_.run(obs_buf_.data(), Spec::kObsDim, act_buf_.data(), Spec::kActDim)) return false;
    Spec::unpack_act(act_buf_.data(), act);
    return true;
  }
private:
  array<float, Spec::kObsDim> obs_buf_{};   // 고정 버퍼, 멤버라 크기 고정
  array<float, Spec::kActDim> act_buf_{};
};
```

- `static_assert`로 잘못된 Spec을 **컴파일 시점에** 거른다
- 로드 시 모델 차원과 Spec 차원을 대조 — 불일치면 로드 실패
- 버퍼가 멤버 배열이라 `run()` 성공 경로에 할당이 없다
- `noexcept`

### 테스트 — golden I/O

```
lib/roa_policy_driver/
  onnx/12dof_guv_v2/data/golden_obs.bin
  onnx/12dof_guv_v2/data/golden_action.bin
  tests/test_iface_roundtrip_12dof.cpp
  tests/bench_inference.cpp
```

README가 밝히는 테스트 구성:
1. `test_dimensions` — 모델 로드 및 입출력 차원 검증
2. `test_golden_io` — `golden_obs.bin` 입력 시 `golden_action.bin`과 같은 출력인지
3. `test_stress` — 반복 실행 안정성(메모리/세션)

**이게 sim2real에서 제일 값어치 있는 테스트다.** 학습 쪽과 배포 쪽이 같은 숫자를
내는지 확인하는 유일한 방법.

> 다만 README에 *"Some golden and stress tests described in the package README are
> disabled in CMake"*라고 솔직히 적혀 있다. 현재 CTest는 주로 round-trip만 돈다.

### 안전과 상태 관리

- `roa_system_manager`가 **별도 노드**로 main controller의 lifecycle을 제어하고
  `RUN / SAFE_HOLD / ESTOP` 상태를 관리한다. 기동 중에는 초기 모터 자세를 유지한다.
- 입력마다 **개별 타임아웃** (IMU, gravity, RSU, motor state, velocity command,
  policy command — 전부 50 ms).
- `is_infer_first_run_done`이 false인 동안 `ControlLoop`는 초기 자세를 계속 발행한다.
  추론이 준비될 때까지 로봇이 안전 자세를 유지하는 것.
- `policy_target_lpf: { enabled: true, cutoff_hz: 1.5 }` — 정책 목표 저역통과.
- `REALTIME_CONTROL_MODE` 파라미터로 실기/디버그 모드를 분리. 디버그에서는
  lifecycle 전이를 수동으로 해야 한다.
- README 최상단에 `> [!CAUTION]` 경고: 실기 전 지지대 고정, CAN 인터페이스,
  모터 ID, 초기 자세, RSU seed, 비상정지 절차 확인.

### 특이 사항 — RSU 발목

RSU(Revolute–Spherical–Universal) 병렬 발목을 쓴다. 정책은 발목을 pitch/roll
**관절 공간**으로 생각하는데, 실제 액추에이터는 두 개의 병렬 링크다.
그래서 `rsu_manager`가 정책 목표 ↔ 액추에이터 명령 사이에서 역기구학과
임피던스 매핑을 한다.

**교훈: 비직결 관절이 있으면 정책 공간과 액추에이터 공간을 분리해야 한다.**
`feasible` 플래그와 신선도까지 확인하고 쓴다.

### 배울 것 / 안 따라할 것

**배울 것**
- 정책 드라이버를 ROS 비의존 라이브러리로 분리 (독립 테스트 가능)
- 컴파일 타임 typed 계약 + 로드 시 모델 차원 대조
- golden I/O 테스트
- pack/unpack round-trip 테스트
- `action_to_q_target` / `q_target_to_action` 양방향 변환
- 입력별 개별 타임아웃
- 추론 준비 전 초기 자세 유지
- 실기/디버그 모드 분리
- README의 명시적 안전 경고

**안 따라할 것**
- `SingleThreadedExecutor` + wall timer로 300 Hz — RT 보장 없음.
  400 Hz를 노린다면 이 구조로는 안 된다.
- 뮤텍스 기반 Latch — RT 경로에서는 lock-free가 낫다

---

# 3부 — 비교와 kid_rl의 위치

## 3.1 정면 비교

| 항목 | microban | ai_sapiens | roa_controller | **kid_rl (현재/목표)** |
| --- | --- | --- | --- | --- |
| 언어 | Python | C++ | C++ | C++ |
| 프레임워크 | 없음 (독립 스크립트) | ROS 2 + ros2_control | ROS 2 (일반 노드) | **ros2_control 컨트롤러** |
| DoF | 19 (정책 18) | 23 | 12 | 18 |
| 액추에이터 | DYNAMIXEL XL330 | 미상 (임피던스) | RobStride / CAN | DYNAMIXEL XL330 |
| 빠른 루프 | 50 Hz | 1000 Hz | 300 Hz | **400 Hz (목표)** |
| 정책 주기 | 50 Hz | 50 Hz (`step_dt` 0.02) | 50 Hz | 50 Hz |
| 루프 구현 | `time.sleep` | 전용 RT 스레드 | wall timer ×3 | controller_manager RT 스레드 |
| SCHED_FIFO | ✗ | ✓ (prio 50/80) | ✗ | ✓ (prio 80, 설정함) |
| mlockall | ✗ | ✓ | ✗ | ✓ (`lock_memory`) |
| 스레드 간 전달 | `threading.Lock` | SharedControlData | `std::mutex` Latch | **lock-free RealtimeBuffer** |
| 관측 정의 | 코드 | **YAML** | 코드 (typed struct) | 코드 |
| 관측 순서 | gyro,grav,q,q̇,a,cmd | ang_vel,grav,cmd,q,q̇,a | grav,cmd,q,q̇,gyro,a | gyro,grav,q,q̇,a,cmd |
| obs scale | 1.0 | 항목별 (0.2/0.05/1.0) | 1.0 | ✗ (없음) |
| obs history | ✗ | **5** | ✗ | ✗ |
| action_scale | 1.0 | 0.25 (관절별) | 0.5 | 파라미터 (스칼라) |
| action clip | ✗ | raw ±5.0 (관절별) | ✗ | ✓ (스칼라) |
| 목표 필터 | ✗ | — | LPF 1.5 Hz | ✓ LPF + 변화율 제한 |
| 계약 검증 | ONNX 메타데이터 | YAML + 로드 검사 | **컴파일 타임 + 차원 대조** | ✗ (TODO) |
| golden I/O | ✗ | ✗ | **✓** | ✗ (TODO) |
| 상태 기계 | Move 상태 | **YAML 계층 FSM** | RUN/SAFE_HOLD/ESTOP | 5-상태 enum |
| 초기 상태 | — | **Damping** | 초기 자세 유지 | READY_POSE |
| 고장 시 | 토크 OFF | **Damping** | SAFE_HOLD | 목표 동결 (개선 필요) |
| 낙상 감지 | `g_z > −0.5`, 스텝 skip | — | — | `g_z > −0.5` + debounce |
| 과전류 | **✓ 프록시 + debounce** | ✗ | ✗ | ✗ (TODO) |
| KP 전환 | **✓ 400↔125** | YAML 관절별 stiffness | 임피던스 게인 | ✗ (TODO) |
| ONNX 스레드 | 기본값 (Python) | **1/1/SEQUENTIAL** | **1/1** | 미구현 |
| 추론 실패 처리 | — | **연속 N회 → Damping** | 경고만 | ✗ (TODO) |
| 단위 테스트 | ✗ | **✓ (FSM, gate, teleop)** | **✓ (round-trip, bench)** | ✗ (TODO) |

## 3.2 세 구현이 공통으로 동의하는 것

이견이 없다는 건 그만큼 확실하다는 뜻이다.

1. **정책은 50 Hz.** 셋 다. 학습 `step_dt`가 0.02 s이기 때문.
2. **`q_target = q_default + scale × action`.** 셋 다 정확히 같은 식.
3. **projected gravity를 쓴다.** 쿼터니언이나 오일러각이 아니라.
4. **last action은 raw(clip 전)를 관측에 넣는다.**
5. **하드웨어 루프는 정책보다 훨씬 빠르다** (300~1000 Hz).
6. **ONNX는 단일 스레드로 묶는다** (C++ 구현 둘 다).
7. **입력 신선도를 시각 기반으로 검사한다.**
8. **정책을 켜기 전에 명시적인 준비 자세 단계가 있다.**
9. **정책 계약(관절 순서, 기본 자세, scale)은 모델마다 다르고 절대 추측하면 안 된다.**

## 3.3 셋이 갈리는 곳

| 쟁점 | 선택지 | 누가 |
| --- | --- | --- |
| 정책을 ros2_control 안에 넣나 | 컨트롤러로 | **kid_rl** |
| | 별도 프로세스 | ai_sapiens |
| | 일반 노드 (ros2_control 미사용) | roa, microban |
| 계약을 어디에 두나 | 코드 (컴파일 타임) | roa |
| | YAML | ai_sapiens |
| | ONNX 메타데이터 | microban |
| 고장 시 | damping | ai_sapiens |
| | 토크 OFF | microban |
| | 위치 유지 | roa, kid_rl |
| 실시간을 진지하게 | 예 | ai_sapiens, kid_rl |
| | 아니오 (되면 됨) | microban, roa |

## 3.4 kid_rl은 어디에 서 있나

**구조적으로는 ai_sapiens와 microban의 중간이다.**

- ai_sapiens처럼 `ros2_control` + RT 스레드 + lock-free 버퍼 + 다중 주기를 쓴다.
- 하지만 프로세스를 나누지 않고 **컨트롤러 하나 안에 다 넣었다.**
  18 DoF 소형 로봇에는 이게 맞다. 토픽 왕복이 없으니 지연이 최소고,
  `read() → update() → write()`가 한 스레드에서 원자적으로 돈다.
- microban에서 정책 계약(관절 순서, 기본 자세, 관측 순서, 낙상 임계)을 그대로 가져왔다.
  같은 로봇 계열이므로 당연하다.

**우리가 이미 잘한 것**
- lock-free RealtimeBuffer + 한 틱 1회 스냅샷
- 정수 나노초 기반 watchdog (예외 없음)
- 시간 누적 decimation (sapiens와 동일 패턴)
- 400 Hz target slew + rate-independent 필터
- IMU/cmd_vel 비대칭 watchdog (셋 중 아무도 명시적으로 안 하지만 논리적으로 옳다)
- 낙상 debounce (microban은 debounce 없이 skip만)
- 이름 기반 인터페이스 매핑
- 모든 모드가 통과하는 단일 write 관문
- 정책 진입 게이트 4중

**가장 시급하게 가져와야 할 것 (우선순위 순)**

1. **golden I/O 테스트** (roa) — 이게 없으면 ONNX를 붙여도 맞는지 알 수 없다
2. **typed PolicyContract + 로드 시 차원/이름 대조** (roa) +
   **ONNX 메타데이터 읽기** (microban)
3. **ONNX 단일 스레드 설정** (sapiens, roa) — 안 하면 400 Hz RT 루프가 망가진다
4. **FAULT → damping** (sapiens, microban) — 지금 목표 동결은 넘어진 로봇을 태운다
5. **KP 전환** (microban) — 학습 강성으로 정책을 돌려야 한다
6. **연속 추론 실패 카운터** (sapiens)
7. **과전류 감시** (microban) — XL330 + BMS 조합에서 실제로 겪는 문제
8. **observation scale / history 지원** (sapiens) — 모델이 요구하면 필수
9. **관절별 scale/clip** (sapiens) — 지금은 전 관절 공통 스칼라
10. **상태 발행** (`~/status`) — 셋 다 있다. 우리만 없다

## 3.5 읽는 순서 추천

직접 소스를 읽는다면:

1. **microban `src/moves/walk.py`** (170줄) — 정책 배포의 최소 형태를 30분에 이해할 수 있다
2. **microban `src/scheduler.py`** — 실기 안전 처리의 현실 (과전류 프록시 부분)
3. **roa `lib/roa_policy_driver/`** (파일 4개) — 계약을 코드로 강제하는 법
4. **ai_sapiens `control_loop.hpp`** — RT 루프를 제대로 만드는 법
5. **ai_sapiens `assets/k1/.../sim2real.yaml`** — 관측/액션 계약을 데이터로 빼는 법
6. **ai_sapiens `config/k1_config.yaml`의 `state_machine`** — 계층적 FSM
