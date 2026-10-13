# kid_rl 작업 기록

`codex`와 `Claude`가 번갈아 이 패키지를 만들고 있다. 이 파일이 **무엇을 했고,
왜 그렇게 했고, 다음에 무엇을 할지**의 단일 기록이다. 새 작업 세션을 시작할 때
이 파일부터 읽고, 끝낼 때 이 파일을 갱신한다.

배경지식과 설계 설명은 [`CONTROLLER_GUIDE.md`](CONTROLLER_GUIDE.md)에 있다.
이 파일은 **이력과 할 일**만 다룬다.

> Codex가 같은 날 별도로 작성한 인계 문서
> [`../CLAUDE_COLLABORATION_AND_DOCUMENTATION.md`](../CLAUDE_COLLABORATION_AND_DOCUMENTATION.md)이
> 패키지 루트에 있고, 협업 규약·배경지식·할 일이 이 두 파일과 겹친다.
> **통합할지 역할을 나눌지는 아직 결정되지 않았다.** 셋 중 하나를 고칠 때 나머지도 확인할 것.

---

## 작업 규칙

에이전트가 바뀌어도 깨지면 안 되는 것들.

1. **관측 계약을 건드리면 문서를 같이 고친다.** 관절 순서, 관측 항목 순서/스케일,
   `default_positions`, `action_scale` 중 하나라도 바꾸면
   `CONTROLLER_GUIDE.md` 4절과 `controllers.yaml` 주석을 반드시 함께 갱신한다.
   여기가 어긋나면 로봇이 넘어지고, 넘어지는 이유를 아무도 못 찾는다.
2. **`update()`에 힙 할당을 넣지 않는다.** `push_back`, `resize`, `std::string` 생성,
   `new` 금지. 모든 버퍼는 `on_configure()`에서 최종 크기로 잡는다.
3. **안전 게이트를 완화할 때는 근거를 이 파일에 적는다.** watchdog timeout,
   `action_clip`, `target_rate_limit`, `fall_*`, `entry_position_tolerance`를
   느슨하게 바꾸는 커밋은 왜 그래도 되는지가 로그에 남아야 한다.
4. **실기 동작을 바꾸는 변경은 `test/smoke_test.sh`를 돌려서 확인한다.**
   상태 기계 전이 로그가 기대대로 나오는지, `/joint_states`가
   `default_positions`와 일치하는지.
5. **추측한 값은 추측이라고 적는다.** `update_rate: 200`, `target_rate_limit: 4.0`
   같은 값은 아직 측정하지 않았다. 주석에 그렇게 써 둔다.

---

## 2026-08-31 — codex: 초기 골격

`codex`가 패키지 전체를 만들었다.

- `KidRlController`: ros2_control 컨트롤러 플러그인, position 명령
- 초기 자세 보간(READY_POSE), 50 Hz decimation, 입력 watchdog, 낙상 감지,
  action clip/filter/rate limit, SAFE_HOLD/FAULT 상태
- microban `src/moves/walk.py` / `src/constants.py`를 읽고 관측 순서와
  `default_positions`를 그대로 옮김
- `description/kid_dynamixel.ros2_control.xacro`: 18관절 DYNAMIXEL 매크로
- ai_sapiens / roa_controller 구조 조사를 README에 정리
- 정책 백엔드는 zero-action dry run (C++ ONNX Runtime 미설치)

**관측 계약은 정확했다.** microban과 대조한 결과 관절 순서 18개, `default_positions`
18개, projected gravity 수식, 관측 항목 순서가 전부 일치한다.

---

## 2026-08-31 — Claude: 리뷰 및 수정

codex의 코드를 리뷰하고 아래를 고쳤다. 빌드는 `-Wall -Wextra -Wpedantic`에서
경고 0, mock 하드웨어 스모크 테스트 통과.

### 정확성 버그

**1. `dry_run_policy`가 정책 진입 조건에 들어가 있었다 (논리 역전)**

```cpp
// before
if (policy_requested && inputs_are_fresh(time) && dry_run_policy_) { ... }
```

ONNX를 붙이고 `dry_run_policy: false`로 두는 순간 **영원히 POLICY_RUNNING에 못 들어간다.**
지금은 이 조건을 진입 게이트에서 빼고, 대신 `on_configure()`에서 백엔드가 없는데
`dry_run_policy: false`면 ERROR를 반환한다. 실패해야 할 곳에서 실패하게 한 것.

**2. RealtimeBuffer 포인터 aliasing**

```cpp
// before
const auto * command = command_buffer_.readFromRT();
const auto * imu = imu_buffer_.readFromRT();
...
else if (!inputs_are_fresh(time)) {...}          // 내부에서 readFromRT()를 또 호출
else if (imu->projected_gravity[2] > ...) {...}  // 위에서 잡은 포인터를 그 후에 역참조
```

`readFromRT()`는 내부 포인터를 swap할 수 있다. 먼저 받은 포인터가 가리키는 버퍼가
그 후 non-RT writer의 차지가 될 수 있으므로, 두 번째 `readFromRT()` 이후의 역참조는
찢어진 데이터를 읽을 수 있다. 지금은 `update()` 맨 앞에서 **버퍼마다 정확히 한 번씩**
읽어 값으로 복사한다. 동시에 `imu`가 nullptr일 때의 역참조 가능성도 사라졌다.

**3. state/command interface를 위치 인덱스로 가정**

```cpp
// before
state_interfaces_[2 * i].get_optional();      // i번 관절의 position이라고 가정
```

`controller_manager`가 요청 순서를 보존한다는 보장에 기댄 코드였다. 어긋나면
**왼쪽 무릎 목표가 오른쪽 발목으로 간다.** 로그 한 줄 없이. 지금은
`on_activate()`의 `map_interfaces()`가 `get_prefix_name()` / `get_interface_name()`으로
인덱스를 만들고, 못 찾으면 어느 인터페이스가 없는지 찍고 실패한다.

**4. `rclcpp::Time` 뺄셈이 RT 경로에서 예외를 던질 수 있었다**

clock type이 다르면 `operator-`가 `std::runtime_error`를 던진다. `update()`에서
예외가 나가면 컨트롤러가 죽는다. 타임스탬프를 `std::int64_t` 나노초로 바꾸고
정수 비교만 한다. 덤으로 미래 타임스탬프(`age < 0`)도 stale로 취급한다.

**5. gait phase 게이트가 microban과 달랐다**

microban은 `mean(|vx|,|vy|,|vθ|) > 0.01`, codex는 `sum > 0.01`. 3배 차이.
`/3.0`을 넣어 맞췄다.

**6. `last_action`에 clip된 값을 저장했다**

관측에 들어가는 `last_action`은 학습 때 정책이 본 것 — 즉 네트워크 **raw 출력** —
이어야 한다. clip된 값을 되먹이면 관측 분포가 학습과 어긋난다. 지금은 raw를 저장하고
clip은 목표 위치 계산에만 적용한다. (`action_clip`이 안 걸리는 정상 구간에서는
차이가 없지만, 걸리는 순간부터 조용히 어긋난다.)

### 안전 문제

**7. cmd_vel timeout이 FAULT를 유발했다**

조이스틱 퍼블리셔가 250 ms 딸꾹질하면 걷던 휴머노이드가 FAULT로 굳는다. 목표가
동결되면 그대로 넘어진다. 정책은 `(0,0,0)` 명령으로 제자리에 서도록 학습되어 있으므로
**안전한 대체값이 있다.** 지금은 cmd_vel stale → 속도 명령 0 + 2초 throttled warn,
IMU stale → FAULT. 의도적 비대칭이며 `CONTROLLER_GUIDE.md` 6.2절에 근거를 적었다.

**8. SAFE_HOLD 진입 시 목표가 한 틱에 스냅했다**

```cpp
// before
targets_ = default_positions_;   // POLICY_RUNNING -> SAFE_HOLD 전환 순간 점프
```

걷는 도중 `enable_policy: false`를 주면 목표가 순간 이동한다. 지금은
`approach(default_positions_, dt)`로 `target_rate_limit`을 적용해 접근한다.

**9. 낙상 감지에 debounce가 없었다**

IMU 한 샘플이 튀거나 착지 충격으로 임계를 순간 넘으면 즉시 FAULT였다. 정상 보행 중
nuisance fault가 난다. `fall_debounce_s`(기본 0.1 s = 정책 5스텝) 동안 조건이 연속
유지되어야 확정한다. 그동안에는 목표를 동결한다 — microban `walk.py: step()`이
낙상 조건에서 `return` 하는 것과 같은 동작.

**10. write 경로에 joint limit clamp가 없었다**

`update_target_from_action()` 안에만 clamp가 있어서 READY_POSE와 FAULT 경로는
한계 검사 없이 하드웨어로 나갔다. `write_commands()`를 **단일 관문**으로 만들어
유한성 검사와 `[joint_position_min, joint_position_max]` clamp를 모든 모드에 적용한다.
새 모드를 추가해도 이 보장이 자동으로 상속된다.

**11. `WAITING_FOR_STATE` / `FAULT`에 분기가 없었다**

if-else 체인이라 두 상태에서 `targets_`가 손대지 않은 채 write되었다. `on_activate()`가
`targets_`를 채우므로 실제로 0이 나가지는 않았지만, 방어가 없었다.
`switch (mode_)` + `default:` 없음으로 바꿔서 **새 Mode를 추가하면 컴파일러가
`-Wswitch`로 잡는다.**

**12. `period` 폭주 방어가 없었다**

RT 스레드가 굶어서 `period`가 0.5 s로 들어오면 rate limit이 `4.0 × 0.5 = 2 rad`를
허용한다. 사실상 제한이 사라진다. `dt`를 100 ms로 clamp.

**13. 비유한 action 검사가 없었다**

지금은 zero action이라 무해하지만, ONNX가 붙으면 NaN이 그대로 목표가 된다.
`run_policy()` 직후 `is_finite_vector(actions_)` 검사 → 실패 시 FAULT.

**14. 정책 진입 자세 검증이 없었다**

README에 요구사항으로 적혀 있었으나 미구현이었다. `entry_position_tolerance`
(기본 0.35 rad ≈ 20°)를 추가. 관절이 걸렸거나 스탠드에 매달려 다리가 늘어진 상태로
정책을 켜는 것을 막는다. 직립 여부(`g_z`)도 함께 검사한다.

**15. `policy_rate_hz > update_rate` 검사가 없었다**

설정하면 정책이 매 틱 돌면서 학습 dt와 어긋난다. configure에서 거절.

### 관측성 / 유지보수

**16. 모드 전환이 로그를 남기지 않았다** → `set_mode(mode, reason)` 헬퍼로 전이마다
`Mode A -> B (reason)` INFO. 이게 없으면 실기에서 무슨 일이 있었는지 알 수 없다.

**17. configure 로그가 오해를 유발했다** — `dry_run_policy`가 false여도
"zero action only"라고 찍었다. 관절 수 / 관측 크기 / 정책 주기를 포함한 정확한
메시지로 교체.

**18. `on_cleanup()`이 없었다** → 구독 해제 추가.

**19. 잡다한 것** — `hardware_interface::HW_IF_POSITION` / `HW_IF_VELOCITY` 상수 사용
(include는 있었는데 안 쓰고 있었다), `phase_total_steps`를 하드코딩에서 파라미터로,
관측 크기 계산을 `6 + 3n + 3`에서 `3 + 3 + 3n + 3`으로 (수치는 같지만 레이아웃이
읽힌다), 관측 레이아웃을 헤더 주석에 표로 명시.

### 함정 하나 (스모크 테스트로 발견)

`joint_position_min: []` 처럼 **빈 리스트를 yaml에 쓰면 컨트롤러 로드가 실패한다.**

```
InvalidParameterValueException: parameter_value_from failed for
  parameter 'joint_position_min': No parameter value set
```

ROS 2 파라미터는 빈 배열의 타입을 추론하지 못한다. 값을 안 넣을 거면 **키 자체를
생략**해야 한다. yaml에는 주석 처리된 예시만 남겼다.

### 추가한 것

- `test/mock/kid_mock.urdf` — `mock_components/GenericSystem` 기반 18-DoF 가짜 로봇
- `test/smoke_test.sh` — 상태 기계 전체를 도는 하드웨어 없는 스모크 테스트
- `docs/CONTROLLER_GUIDE.md` — 배경지식 포함 전체 가이드
- `docs/WORKLOG.md` — 이 파일

### 검증 결과

```
$ colcon build --packages-select kid_rl --symlink-install
Finished <<< kid_rl [8.93s]          # -Wall -Wextra -Wpedantic, 경고 0

$ bash src/kid_rl/test/smoke_test.sh
[WARN]  [kid_rl]: Configured 18 joints, observation size 63, policy 50.0 Hz. DRY RUN: ...
[INFO]  [kid_rl]: Mode WAITING_FOR_STATE -> READY_POSE (activated)
[INFO]  [kid_rl]: Mode READY_POSE -> SAFE_HOLD (ready pose reached)
[INFO]  [kid_rl]: Mode SAFE_HOLD -> POLICY_RUNNING (enable_policy=true and entry checks passed)
[WARN]  [kid_rl]: cmd_vel is stale; commanding zero velocity     # fault 아님 (의도)
[ERROR] [kid_rl]: Fault: IMU watchdog expired. Freezing the joint targets ...
```

`/joint_states` 18개 관절 전부 `default_positions`와 1e-4 이내로 일치.
관측 크기 63 = 3 + 3 + 3×18 + 3, gait phase 미사용 시 기대값과 일치.

**검증되지 않은 것**: 실제 DYNAMIXEL 하드웨어, 실제 IMU, 실제 정책 추론.
전부 dry run과 mock 위에서만 확인했다.

---

## 2026-08-31 — Claude: 400 Hz / 50 Hz 다중 주기 구조

목표가 "DYNAMIXEL 400 Hz, 정책 50 Hz"로 정해져서 구조를 거기에 맞췄다.

### 먼저 확인한 제약

`hardware_interface/src/resource_manager.cpp`:

```cpp
hw.rw_rate = (hw.rw_rate == 0 || hw.rw_rate > update_rate) ? update_rate : hw.rw_rate;
```

**하드웨어 `rw_rate`는 controller_manager `update_rate`를 넘을 수 없다.** 감속만
가능하다. 따라서 "CM 50 Hz + 하드웨어 400 Hz"는 불가능하고, `update_rate: 400`이
전제가 된다. 정책 50 Hz는 컨트롤러 내부 decimation으로 만든다.

### 코드 변경

**목표 계산을 두 주기로 분리했다.**

- `update_policy_setpoint()` — 50 Hz. `policy_targets_`(raw setpoint)만 만든다.
- `slew_targets(dt)` — **매 틱(400 Hz).** LPF + 변화율 제한으로 `targets_`를
  setpoint 쪽으로 접근시킨다.

이전에는 필터와 rate limit이 정책 틱에서만 돌아서 하드웨어에 50 Hz 계단이
그대로 나갔다. 이제 400 Hz 램프가 된다. 그리고 watchdog·낙상 판정의 반응
해상도가 20 ms → 2.5 ms가 된다.

**필터 계수를 주기 독립으로 만들었다.**

`y += α(u − y)`의 α는 호출 주기에 종속이다. 50 Hz 기준 α=0.2를 400 Hz에서 그대로
쓰면 필터가 8배 빨라져 사실상 사라진다. configure에서 시간 상수로 변환한다.

```
filter_time_constant_s_ = −(1/policy_rate_hz) / ln(1 − target_filter_alpha)
α_tick = 1 − exp(−dt / filter_time_constant_s_)
```

`target_filter_alpha`는 여전히 "정책 주기 1회에 오차의 20% 제거"라는 의미를 유지한다.

**POLICY_RUNNING 진입 시 `policy_targets_ = targets_`** — 첫 추론이 무엇을 내든
첫 slew 스텝이 0이 되도록.

### 설정 변경

`config/controllers.yaml`
- `update_rate: 200` → `400`
- `lock_memory: true`, `thread_priority: 80` 추가
- `cpu_affinity` 주석으로 준비
- `joint_state_broadcaster`에 `update_rate: 50` (400 Hz로 뿌릴 이유가 없다)
- `kid_rl`의 `update_rate`는 **일부러 비워 둠**. 여기에 50을 적으면 `update()`
  자체가 감속되어 400 Hz slew가 사라진다. yaml 주석에 경고를 적었다.

`description/kid_dynamixel.ros2_control.xacro`
- `baud` 기본값 `1000000` → `3000000`. **1 Mbps로는 400 Hz가 물리적으로 불가능하다**
  (한 사이클 ~400바이트 = 4.0 ms).
- `rw_rate` 인자 추가, `<ros2_control rw_rate="...">`로 전달
- `Return Delay Time: 0` 추가 (기본 250 = 500 µs = 예산의 20%)

### 문서

- `docs/THREADING.md` 신규 — 스레드 목록, RT 설정, ONNX 스레드 봉인, 버스 예산 계산,
  예산 초과 시 탈출 순서, 브링업 체크리스트
- `docs/STUDY.md` 신규 — 배경지식 + microban/ai_sapiens/roa_controller 해부와 비교
- `CONTROLLER_GUIDE.md` 5.1/5.3/6.4/9/10절 갱신

### mock에서 측정한 것

`update_rate: 400`으로 스모크 테스트를 돌린 결과:

```
kid_rl                  active     (Mode 전이 전부 정상, 63차원 관측, 정책 50 Hz)
/joint_states           49.98 Hz   (joint_state_broadcaster update_rate: 50 적용 확인)
Overrun detected!       1회        loop took 3.386 ms (missed cycles: 2)
Could not enable FIFO RT scheduling policy: Operation not permitted
```

**mock 하드웨어(직렬 통신 0 ms)인데도 오버런이 났다.** 원인은 RT 스케줄링이
거부된 것이다. `THREADING.md` 3.2a의 rtprio 설정이 왜 선택이 아니라 전제인지를
그대로 보여준다. 실기에서는 여기에 1 ms 이상의 직렬 트랜잭션이 더해진다.

### 함정: 컨트롤러 파라미터의 위치

`update_rate`를 `controller_manager:` 하위에 적으면 **조용히 무시된다.**

```yaml
# 안 먹음
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      update_rate: 50        # ← 무시됨. /joint_states 가 400 Hz 로 나온다
```

`controller_manager:` 하위 블록은 **`type`만** 읽는다. `update_rate` / `is_async` /
`thread_priority`는 `controller_interface_base.cpp`가 **컨트롤러 자신의 노드**에
선언하는 파라미터라 최상위 블록에 적어야 한다.

```yaml
# 먹음
joint_state_broadcaster:
  ros__parameters:
    update_rate: 50
```

에러도 경고도 안 난다. `ros2 topic hz`로 실제 주기를 확인하는 것 외에는
알 방법이 없다. 실제로 첫 시도에서 이 함정에 빠졌다.

### 아직 검증 안 된 것

**400 Hz는 목표이지 실기 측정값이 아니다.** 버스 예산은 계산이고, 실측하지 않았다.
그리고 `update_rate: 400`은 mock 하드웨어에서만 돌려 봤다 — 직렬 통신이 없으니
당연히 여유가 있다. 실기에서는 [`THREADING.md` 5절 체크리스트](THREADING.md)를
순서대로 밟아야 한다. 특히 rtprio 한도와 FTDI latency timer가 안 되어 있으면
400 Hz는 시작도 못 한다.

---

## 2026-09-27 — Claude: 보행 위상(gait_phase) 관측 구현 + 추론 건너뜀 수정

### 학습 쪽 계약 (mjlab_kid_RL tasks/mdp.py gait_phase, kid_RL_env_cfg.py)
- 관측 = [sin(2π·phase), cos(2π·phase)], phase = (episode_length_buf × 0.02 / cycle_time) mod 1,
  cycle_time = 0.4 / (0.5 − asin(0.1)/π) = 0.8544895701575631 s. 지연·노이즈·스케일 없음.
- 관측 순서의 맨 끝: base_ang_vel, projected_gravity, joint_pos, joint_vel, actions, command, gait_phase (86).
- step() 순서상 k 번째 action 을 정하는 관측의 episode_length_buf = k, 리셋 직후 관측은 (0, 1).

### 구현
- `include/kid_rl/gait_phase.hpp`: 식과 GaitClock. POLICY_RUNNING 진입 때 restart, 매 틱 advance(dt)
  (진입 후 첫 advance 는 더하지 않음 → 첫 추론 관측 위상 0), 이후 실측 주기를 더해 실제 경과 시간을 따른다.
- PolicyRunner::load 가 메타데이터 observation_names 를 읽어 84(위상 없음)/86(위상 있음)을 자동 판단하고,
  순서가 컨트롤러 조립 순서와 다르면 거부한다. 위상 모델인데 gait_cycle_time_s 가 0 이면 기동 거부.
- controllers.yaml `gait_cycle_time_s: 0.8544895701575631`. CSV 에 `gait_phase` 열(안 쓰면 −1).

### 함께 고친 버그: 추론 건너뜀
50 Hz 루프 / 50 Hz 정책에서 `accumulator >= period` 조건 때문에 주기가 19.9 ms 처럼 조금 짧은 틱은
추론을 건너뛰었다(mock 16.8 %, 실기 22:06 2.2 %). 그 틱에는 action 이 40 ms 유지되고 다음 관측의
last_action·위상 간격이 학습과 달라진다. 반 틱 여유(`>= period − dt/2`) + 잔여 clamp 로 바꿨다.

### 검증
- `test/gait_phase_test`: mjlab 학습 env 를 실제로 띄워 기록한 602 행(`test/gen_gait_phase_golden.py`,
  리셋 2회, 에피소드 최대 300 스텝)과 식·GaitClock 순서 대조(최대 4.6e-6, float32 수준), 첫 관측 (0,1),
  합성 흔들림 dt 에서 실제 경과 시간 추종, 10 분 누적 오차, 재진입.
- `policy_runner_test`: 배포 모델(84)과 위상 모델(86, 2026-09-27_19-43-46) 각각 로드·출력 대조, 관측 순서가
  다르면 거부되는지. 기댓값은 `test/gen_policy_golden.py` 로 모델별 파일에서 읽는다.
- `test/e2e/run_e2e.sh` + `check_e2e.py`: mock 하드웨어 + socat 가상 IMU 로 컨트롤러를 통째로 돌림.
  첫 추론 위상 0, 추론 건너뜀 0/450~711, 위상 = 실제 경과 시간(1.8e-6), CSV 관측으로 ONNX 재생 시 기록 action
  재현(오차 중앙 5e-4), 위상을 한 틱 밀면 44~61 배 어긋남. CPU 부하로 dt 14~26 ms 흔들어도 같음.

### KP (xacro vs 학습)
9/26 이후 학습 kp_fw = MX-28 35 / MX-64 60 / MX-106 134 / XH540 165, xacro 도 같음(head_pitch 만 의도적으로 120).
단 지금 `models/policy.onnx`(9/21 체크포인트)는 27/46/103/150 으로 학습된 모델이라 xacro 와 맞지 않는다.

### 미완료
- 로봇(Jetson) 미반영: 작업 시점에 192.168.100.2 접속 불가.
- `models/policy.onnx` 교체는 하지 않았다(어느 체크포인트를 쓸지는 사람이 정함).

## 2026-09-23 — Claude: 23:48 실기 결과, SyncRead ping 진단 제거

23:48 실행(64 초)에서 A·B·action_clip·track_rx 동작 확인: 실패 read 중앙 14.2 ms(이전 20.1),
실패 44회 모두 명령 전송(Write Fail 0), clip 발동 0, FAULT 0, IMU 무효 0.

빠진 서보: 실패 44회 중 43회가 `2 6 12 10 14 16 18 20` — SyncRead 순서에서 ID 2(left_shoulder_roll)
부터 끝까지. ID 2 가 응답하지 못하면 뒤 서보들이 차례를 못 받는다. ID 2 배선 점검, 또는 순서에서 ID 2 를
맨 뒤로 옮겨 확인할 것.

제거: read 실패 시 25개 서보에 개별 ping 을 돌리던 DiagnoseSyncReadResponders (5초에 한 번). 제어
스레드에서 돌아 응답이 있을 때 약 25 ms(실패 9회가 약 39 ms), 버스가 죽었을 때 약 870 ms 를 먹어 사이클을
건너뛰게 했다. 빠진 ID 는 track_rx 의 missing IDs 가 대신 알려 준다.

64 초의 버스 전체 무응답 → 드라이버 ERROR 는 사용자가 모터 전원 스위치를 끈 것이다(2026-09-24 사용자 확인).
04:49 등 이전 실행 끝의 전체 무응답도 모두 같다. 앞서 적은 '전원 전압 강하 의심'은 틀린 추정이었다.

## 2026-09-23 — Claude: read 실패 시 사이클 손실 줄이기 (드라이버 A + B)

문제: read 응답 대기 제한이 제어 주기 전체(20 ms)라, 응답 하나만 빠져도 read 가 20 ms 를 쓰고
사이클이 주기를 넘겼다. ros2_control 은 지나간 칸을 버리고 다음 칸(40 ms)으로 건너뛴다
(controller_manager sleeping_policies.cpp). 게다가 write() 는 dxl_status_ == COMM_ERROR 면 전송을
건너뛰어서, 정책은 돌았는데 새 명령이 약 60 ms 동안 나가지 않았다(22:06 CSV: 실패 틱 다음 dt 중앙 30 ms).

수정 (xacro 파라미터, 0/false 면 예전 동작):
- A. `read_timeout_ms = 14`: 성공한 read 최대 12.25 ms(p99.9 11.1)보다 여유. 실패해도 14 + update·write
  최대 3.98 ms ≈ 18 ms 로 20 ms 안에 끝난다.
- B. `write_on_read_failure = true`: read 실패여도 SyncWrite 를 보낸다. 명령은 직전 관절값으로 계산되며,
  학습 관측 지연 0~1 스텝이 이 경우를 덮는다.
- 계측 버그 수정: write_sent 를 분기 전에 초기화(예전엔 전송 분기 안에서만 해서 read 실패 틱에 직전 값 1 이 남음).

검증: 로컬·Jetson 빌드, xacro 확장 확인. **실기 미검증** — 다음 CSV 에서 read 실패 틱의 read_ms ≈ 14,
다음 틱 dt ≈ 20, prev_write_sent = 1, 관절값 나이 상한 ≈ 1 스텝인지 볼 것.

## 2026-09-23 — Claude: action clip 누락으로 인한 정책 발산 수정

증상: 22:06 실기 128.8 초에 `Fault: policy produced a non-finite action`. 직전 0.8 초 동안
|action| 이 9.75 → 84 → 880 → … 20 ms 마다 6~17 배씩 커져 NaN 이 됐고, 그동안 관절 22개가
관절 한계까지 명령받았다. 관측(중력·각속도·관절 속도)은 정상 범위였다.

원인: 학습 설정에 2026-09-21 부터 `agent.clip_actions = 5.0` 이 있고, 배포 모델
(run 2026-09-23_14-03-13)도 그 값으로 학습됐다. RSL-RL 래퍼가 env.step() 전에 ±5 로 자르므로
목표각도 last_action 관측도 자른 값이다. 컨트롤러는 "학습에 clip 없음"이라는 예전 전제로
자르지 않았고, 몸이 약 40도 기울며(gravity_z −0.75, 들어 올림 추정) |action| 이 5 를 넘자
정책이 본 적 없는 |last_action| > 5 가 되먹임돼 폭주했다.

수정: `action_clip` 파라미터(기본 5.0, yaml 에 명시). 목표 계산 전에 자르고, 자른 값을
last_actions_ 에 넣는다. 처음 걸릴 때 활성화당 한 번 경고.

검증: 22:06 CSV 의 관측으로 정책을 재생(재구성 오차 중앙값 3e-4, CSV 자릿수 한계). 발산 구간
61틱에서 clip 없음 → inf, clip ±5 → 최대 5 로 유지. 단, 61틱 중 42틱이 clip 에 걸렸다 —
넘어지거나 들린 상황에서는 여전히 끝까지 치우친 명령이 나간다.

미해결: `policy_slew_limit` 이 0.0(비활성, '임시 실기 진단')으로 남아 있다. 서보별 응답
추적(`track_rx`)을 켰다 — 다음 CSV 에서 rx_missing_ids 로 빠지는 서보 ID 를 본다.

## 2026-09-23 — Claude: 버스 타이밍 계측 (CSV) + 서보 반응 지연 측정 스크립트

목적: 학습 지연 랜덤화(obs delay, BAM 명령 지연)의 근거를 계산이 아니라 측정으로 잡기.

### 무엇을 재는가

| CSV 열 | 뜻 | 측정 방식 |
| --- | --- | --- |
| `read_ms` | 이번 틱 SyncRead 트랜잭션 시간 | 드라이버 `read()` 에서 steady_clock 두 번 |
| `read_ok` | 이번 틱 SyncRead 성공 여부 | `ReadMultiDxlData()` 결과 |
| `joint_age_min_ms` / `joint_age_max_ms` | 정책이 쓰는 관절값의 나이 범위 | now − 마지막 **성공** 읽기의 끝/시작. 서보는 그 사이에 값을 찍는다. 읽기 실패 틱엔 자연히 한 틱만큼 자란다 |
| `prev_write_sent` / `prev_write_ms` | 직전 틱 SyncWrite 여부와 호출 시간(커널에 넘기기까지) | `write()` 에서 steady_clock. `write()` 가 `update()` 뒤라 '직전 틱' |
| `prev_write_drain_ms` | 직전 틱 송신 큐가 빌 때까지 | `tcdrain()`. xacro `measure_write_drain=true` 일 때만, 아니면 −1 |

- 드라이버: `type=bus_timing` 가상 gpio(`dxl_bus`)를 새로 받아 위 값을 state interface 로
  내보낸다. 이 gpio 는 ping/InitItem(레지스터 쓰기) 루프에서 건너뛴다. `ID=255` 는 공통
  gpio 검사를 통과하기 위한 자리표시자.
- 컨트롤러: `BusTimingMonitor` 가 robot_description 에 그 gpio 가 있으면 스스로 켜진다
  (`RxMissingMonitor` 와 같은 방식). 없으면 인터페이스도 열도 없다.
- xacro 스위치: `bus_timing`(기본 true), `measure_write_drain`(기본 false — 제어 스레드를
  막고 커널 틱 단위로 값이 부풀 수 있어서).

### 매 틱으로는 못 재는 것

- **서보가 새 목표를 집어 드는 시간**: `scripts/servo_step_latency.py`. 관절 하나에 작은
  계단 목표(기본 0.05 rad)를 주고 Present PWM/Position 을 빠르게 폴링해 반응 시작을 잰다.
  관절이 움직이므로 로봇을 받치고 컨트롤 노드를 끈 상태에서 사람이 실행한다.
- **선 위 전송 시간**: 1 Mbps 에서 139 바이트 = 1.39 ms. 물리량이라 계산이 곧 값이다.
  끝나는 시점은 `prev_write_drain_ms` 로 상한만 보인다.
- 앞선 보고의 "FTDI latency_timer 최대 1 ms" 는 **명령(쓰기) 경로가 아니다.** 그 타이머는
  칩이 받은 바이트를 호스트로 올려 보낼 때(수신) 기다리는 시간이라 `read_ms` 안에 포함된다.

### 검증

- 로컬, Jetson 모두 `colcon build --packages-select dynamixel_hardware_interface kid_rl` 통과.
  xacro 확장에서 `dxl_bus` gpio 확인.
- **실기 미검증**: 새 CSV 열이 실제로 채워지는지는 다음 실행에서 확인해야 한다.
- `policy_runner_test` 실패는 이번 변경과 무관: `models/policy.onnx` 가 12:00 에 새 모델로
  바뀌었고 테스트의 기댓값(golden value)은 이전 모델 기준이다(`0.517 != 0.213`).
- 같은 날 15:45 에 들어온 SyncRead 실패 ID 추적(`track_rx`) 작업은 로봇에 없었다. 이번에
  두 패키지를 통째로 올리면서 함께 반영됐다(로봇 쪽에 따로 고친 줄이 없는 것 확인).

## 문서화 계획 — 무엇을 쓸 것이고 각각 어떤 배경지식이 필요한가

현재 문서 다섯 개와, 앞으로 필요한 문서들. 각 항목에 **그 문서를 쓰거나 읽는 데
필요한 배경지식**을 같이 적었다.

### 이미 있는 것

| 문서 | 대상 독자 | 다루는 것 | 필요한 배경지식 |
| --- | --- | --- | --- |
| `README.md` | 처음 오는 사람 | 이게 뭐고 어떻게 빌드/실행하나 | ROS 2 워크스페이스, colcon |
| `docs/STUDY.md` | 공부하는 사람 | 배경지식 + 세 레퍼런스 구현 해부 | 없음 (이 문서가 배경지식 자체) |
| `docs/CONTROLLER_GUIDE.md` | 코드를 고치는 사람 | ros2_control, 정책 계약, 안전, 파라미터, 브링업 | C++, ROS 2 기초 |
| `docs/THREADING.md` | 400 Hz를 맞추는 사람 | 스레드, RT 설정, 버스 예산 | 리눅스 스케줄링, 시리얼 통신 |
| `docs/WORKLOG.md` | 이어받는 사람 | 이력, 결정, 할 일 | 없음 |
| `CLAUDE_COLLABORATION_AND_DOCUMENTATION.md` | 〃 (codex 작성) | 협업 규약 + 배경지식 + 할 일 | 없음 |

> 마지막 둘은 내용이 겹친다. 통합 여부 미결정.

### 앞으로 써야 하는 것

**1. `docs/POLICY_CONTRACT.md` — 모델 계약서** `[P0]`

지금 쓰는 ONNX 모델 **하나에 대한** 사실만 적는 문서. 관절 순서, 기본 자세,
관측 항목·순서·scale·clip·history, action scale/offset/clip, 학습 시 PD 게인,
학습 dt, 명령 속도 범위, gait phase 사용 여부, 모델 파일 해시.

- 왜 필요한가: 지금 이 정보가 `controllers.yaml`, 코드 주석, microban 소스에
  흩어져 있다. 모델을 바꾸면 무엇을 같이 바꿔야 하는지 한 곳에 있어야 한다.
- 필요한 배경지식: [`STUDY.md` 1.2](STUDY.md) (관측 항목의 의미),
  [1.8](STUDY.md) (계약을 코드로 강제하기)
- 참고: roa의 `lib/roa_policy_driver/onnx/12dof_guv_v2/model_info.md`,
  ai_sapiens의 `assets/.../params/sim2real.yaml`

**2. `docs/HARDWARE.md` — 로봇 실측 문서** `[P0]`

모터 ID ↔ 관절 이름, Drive Mode 부호와 **검증 방법/결과**, 영점 오프셋,
관절 가동범위 실측값, 배선/데이지체인 순서, 포트·baud, 전원·BMS 사양,
과전류 임계값 근거.

- 왜 필요한가: 이게 없으면 로봇을 분해했다 조립할 때마다 처음부터 다시 한다.
  그리고 `joint_position_min/max`를 채우려면 어차피 측정해야 한다.
- 필요한 배경지식: DYNAMIXEL Protocol 2.0 컨트롤 테이블,
  [`CONTROLLER_GUIDE.md` 7절](CONTROLLER_GUIDE.md), URDF 관절 축 규약

**3. `docs/FRAMES.md` — 좌표계·센서 문서** `[P0]`

몸통(trunk) 좌표계 정의, IMU 물리 장착 방향과 센서→몸통 회전, `~/imu`가 어느
프레임인지, 타임스탬프 출처(헤더 vs 수신 시각), 각속도·가속도 단위와 부호,
자세 추정 방식(Madgwick? 하드웨어 융합?)과 수렴 시간.

- 왜 필요한가: 부호 하나가 뒤집히면 로봇이 넘어지는데, 그걸 코드만 봐서는 알 수 없다.
  microban은 `IMU_MOUNT_QUAT`을 상수로 갖고 있는데 우리는 "드라이버가 했겠지"라고
  가정하고 있다. 그 가정을 문서로 못 박거나 깨야 한다.
- 필요한 배경지식: [`STUDY.md` 1.2 projected gravity](STUDY.md),
  쿼터니언, 회전 합성 순서

**4. `docs/TESTING.md` — 시험 절차서** `[P1]`

단계별 시험(mock → preflight → 스탠드 → 바닥)의 **합격 기준**, golden I/O 절차,
회귀 시험 목록, 실패 시 롤백 절차, 시험 로그 보관 위치.

- 왜 필요한가: "돌려 봤는데 되던데요"는 검증이 아니다. 무엇을 어떤 조건에서
  확인했고 무엇이 통과 기준이었는지가 남아야 한다.
- 필요한 배경지식: [`CONTROLLER_GUIDE.md` 10절](CONTROLLER_GUIDE.md),
  [`THREADING.md` 5절](THREADING.md), ctest/launch_testing

**5. `docs/OPERATION.md` — 운용 절차서** `[P1]`

기동/종료 순서, 비상정지 방법(물리 + 소프트), FAULT 복구 절차, 배터리 관리,
정상 로그와 비정상 로그의 구분, 자주 나오는 경고의 의미, 누구에게 연락하나.

- 왜 필요한가: 로봇을 만든 사람이 아닌 사람이 켤 때 필요하다.
  그리고 사고 직후에는 문서를 읽을 시간이 없으므로 한 장으로 압축되어야 한다.
- 필요한 배경지식: 없음 (이 문서만으로 운용 가능해야 함)

**6. `docs/TUNING.md` — 튜닝 가이드** `[P2]`

`target_filter_alpha` / `target_rate_limit` / `action_clip` /
`fall_gravity_z_threshold` / `entry_position_tolerance`를 어떤 증상에서
어느 방향으로 움직이는가. 각 파라미터의 트레이드오프.

- 왜 필요한가: 지금은 값의 의미만 문서화되어 있고, **어떤 증상에 무엇을 만질지**가 없다.
- 필요한 배경지식: 1차 저역통과 필터, 위상 지연과 안정성,
  [`STUDY.md` 1.3](STUDY.md)

**7. `docs/ONNX_INTEGRATION.md` — 백엔드 통합 문서** `[P1, PolicyRunner와 함께]`

ONNX Runtime 설치 방법과 버전, CMake 연동, 세션 옵션과 그 이유, 텐서 사전 할당,
메타데이터 규약, 추론 시간 측정 결과, golden I/O 파일 형식.

- 왜 필요한가: 설치가 시스템 경로를 건드리므로(roa는 `/opt/onnxruntime/current`를
  쓰고 `ld.so.conf.d`를 수정한다) 재현 가능하게 적어야 한다.
- 필요한 배경지식: [`STUDY.md` 1.7](STUDY.md),
  [`THREADING.md` 3.2c](THREADING.md), CMake `find_package`/`imported target`

### 문서를 쓸 때의 규칙

1. **추측한 값은 추측이라고 적는다.** `update_rate: 400`, `target_rate_limit: 4.0`,
   버스 예산 계산 — 전부 아직 실측이 아니다.
2. **검증된 것과 검증 안 된 것을 분리한다.** "mock에서 확인"과 "실기에서 확인"은
   다른 문장이다.
3. **왜를 적는다.** "IMU stale은 FAULT"만 적으면 다음 사람이 cmd_vel도 FAULT로
   바꾼다. "대체값이 없으므로"를 같이 적어야 한다.
4. **숫자에는 출처를 단다.** `KP_RL = 125`는 microban `constants.py`,
   `g_z > −0.5`는 microban `walk.py`.

---

## 할 일

우선순위 순. `[codex]`는 codex에게 맡기기 좋은(사양이 명확한) 작업,
`[사람]`은 실물 측정이나 결정이 필요한 작업.

### P0 — 정책이 돌기 위해 반드시 필요

- [x] **`PolicyRunner` (ONNX Runtime C++)** `[codex]`
      `on_configure()`에서 세션 로드, RT 경로에서는 고정 크기 버퍼로 `run()`만.
      추론 실패 카운터 → 연속 N회 실패 시 FAULT.
      먼저 `onnxruntime` C++ 라이브러리를 설치하고 CMake에서 찾을 수 있게 해야 한다.
- [ ] **`PolicyContract` 검증** `[codex]`
      configure 단계에서 ONNX 메타데이터와 대조:
      입력 shape == `observation_.size()`, 출력 shape == 관절 수,
      `joint_names` == `joints` 파라미터(순서까지), `default_joint_pos` ==
      `default_positions`(허용오차 이내). 하나라도 다르면 configure 실패.
      입력 shape으로 `use_gait_phase`를 자동 판별하는 것도 여기서
      (microban `walk.py`의 `_use_reference_phase` 로직과 동일).
- [x] **golden I/O 테스트** `[codex]`
      학습 쪽에서 덤프한 (관측, 행동) 쌍 몇 개를 파일로 받아 C++ 추론 결과와 비교.
      **여기서 안 맞으면 실기에 올리지 않는다.**
- [ ] **`joint_position_min/max` 실측** `[사람]`
      URDF와 실물 양쪽에서 확인해 yaml에 채운다. 지금 ±π는 사실상 제한이 없다.
- [ ] **DYNAMIXEL 버스 대역폭 측정 후 `update_rate` 확정** `[사람]`
      `update_rate: 200`은 추측값이다. overrun 통계를 보고 정한다.
      `CONTROLLER_GUIDE.md` 7.3절.
- [ ] **관절 부호 / 영점 / 이름 대응 검증** `[사람]`
      토크 OFF 상태에서 하나씩. `CONTROLLER_GUIDE.md` 10-2절.
- [ ] **IMU 프레임 확인** `[사람]`
      `~/imu`가 몸통 좌표계인가. 아니면 마운트 회전 파라미터를 추가해야 한다
      (microban `IMU_MOUNT_QUAT`). `CONTROLLER_GUIDE.md` 4.5절.

### P1 — 실기 안전에 필요

- [ ] **DYNAMIXEL 통신 자동 복구** `[codex]`
      버스 통신 상태를 state interface로 제어기에 전달하고, 하드웨어 인터페이스의
      비실시간 작업 스레드가 포트를 다시 열고 Sync Read/Write 핸들러를 재구성한다.
      통신 복구 중에는 마지막 정상 명령을 유지하고 Torque Enable을 건드리지 않는다.
      현재 `CommReset()`은 `stop()`에서 토크를 끄고 모터까지 reboot하므로 사용하지
      않는다. 25축 controller 설정과 18축 ros2_control xacro의 이름/개수 불일치도
      이 작업에서 함께 바로잡는다.

- [ ] **FAULT 시 damping** `[codex]` + `[사람]`
      지금은 목표 동결이라 넘어진 로봇이 계속 토크를 낸다. DYNAMIXEL
      `Position P Gain`을 낮추거나 torque를 끄려면 hardware component에
      gpio command interface가 필요하다. ai_sapiens와 roa 둘 다 Damping 상태를 둔다.
      `CONTROLLER_GUIDE.md` 6.5절.
- [ ] **과전류 감시** `[codex]`
      `Present Current` state interface를 읽어 합산, 임계 초과가 debounce 틱 이상
      지속되면 FAULT. microban `constants.py: OVERCURRENT_CUTOFF_A`,
      `OVERCURRENT_DEBOUNCE_TICKS`. 임계값은 실측 필요 `[사람]`.
- [ ] **`~/status` 발행** `[codex]`
      mode, observation, action, 입력 freshness, 정책 추론 시간, overrun 카운트.
      `realtime_tools::RealtimePublisher` 사용. **지금은 실기에서 무슨 일이 있었는지
      로그 말고는 볼 방법이 없다.**
- [ ] **`cmd_vel` 스케일링/제한** `[codex]`
      학습 때 쓴 명령 범위를 벗어나면 정책이 학습 분포 밖으로 나간다.
      microban은 `VX_MAX 0.7`, `VX_MAX_BACKWARD 0.5`, `VY_MAX 0.3`,
      `VTHETA_MAX_STATIONARY 3.0`, `VTHETA_MAX_MOVING 1.5`를 쓰고 제자리 회전일 때
      회전 상한을 키운다.
- [ ] **KP 전환** `[codex]`
      SAFE_HOLD는 `KP_DEFAULT 400`, POLICY_RUNNING은 `KP_RL 125`.
      **학습 때의 강성으로 정책을 돌려야 한다.** `CONTROLLER_GUIDE.md` 7.1절.
      gpio command interface 필요.

### P2 — 구조 개선

- [ ] **단위 테스트** `[codex]`
      projected gravity 수식(알려진 쿼터니언 → 기대 벡터), 관측 packing 순서와 크기,
      상태 전이(각 게이트가 실제로 막는지), watchdog 경계, rate limit / clip 동작.
      순수 함수를 헤더로 빼면 `ament_cmake_gmock`으로 컨트롤러 없이 테스트 가능.
      (지금 `package.xml`에 `ament_cmake_gmock` / `ament_lint_auto` 의존이 선언되어
      있는데 `CMakeLists.txt`에 `BUILD_TESTING` 블록이 없다. 같이 정리할 것.)
- [ ] **`ObservationBuilder` 분리** `[codex]`
      지금은 관측 순서가 C++ 코드에 하드코딩되어 있다. ai_sapiens처럼 YAML로
      term 순서 / scale / clip / history를 정의하면 모델이 바뀌어도 재컴파일이 없다.
      **history 지원이 특히 중요하다** — 관측 history를 쓰는 정책이 많다.
- [ ] **`ActionPipeline` 분리** `[codex]`
      항목별 scale / offset / clip. 지금은 전 관절 공통 스칼라다.
- [ ] **IMU를 토픽이 아니라 state interface로** `[codex]` + `[사람]`
      hardware component가 IMU를 노출하면 토픽 왕복 지연과 지터가 사라지고
      `read()` 시점에 정확히 동기화된 값을 얻는다. microban은 200 Hz 전용 스레드를 쓴다.
- [ ] **launch 파일** `[codex]`
      `robot_state_publisher` + `ros2_control_node` + controller spawner.
      실기용과 mock용 둘 다.
- [ ] **FAULT 복구 서비스** `[codex]`
      지금은 deactivate → activate 뿐. 명시적인 `~/reset_fault` 서비스가 있으면
      운용이 쉽다. 단, 자동 복구는 하지 말 것.
- [ ] **비직결 관절 기구학** `[codex]`
      RSU ankle 같은 링크 구조가 있으면 policy target(관절 공간)과 actuator
      command(모터 공간) 사이에 변환이 필요하다. microban 로봇에 해당하는지 확인 필요.
- [ ] **README 정리** `[codex]`
      지금 README에 설계 논의가 섞여 있다. README는 "이게 뭐고 어떻게 빌드/실행하나"만
      남기고 나머지는 `docs/`로 옮긴다.

### 열린 질문 `[사람]`

- ONNX 모델은 `2026-09-08_22-48-01.onnx`로 확정했다. 입력 `[1,84]`, 출력
  `[1,25]`이며 gait phase 입력은 없다.
- 학습 쪽 `action_scale`, `policy dt`, PD 게인의 정확한 값은?
  (지금 yaml 값은 microban `walk.py`에서 가져온 것이고 모델이 다르면 틀리다.)
- 이 로봇이 microban과 정확히 같은 기구인가, 아니면 KID 쪽 변형인가?
  `default_positions`와 모터 ID / 부호가 그대로 유효한가?
- `dynamixel_hardware_interface`가 `Position P Gain`을 런타임에 쓸 수 있는가?
  (P1의 damping과 KP 전환이 여기에 달려 있다.)
