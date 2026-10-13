# 스레딩·실시간·400 Hz 설계

목표: **DYNAMIXEL read/write 400 Hz, 정책 추론 50 Hz.**

이 문서는 (1) 그게 ros2_control에서 어떻게 구성되는지, (2) 지금 여기서 처리해야 할
것이 무엇인지, (3) 아직 안 된 것이 무엇인지를 다룬다.
배경지식은 [`STUDY.md` 1.4~1.7절](STUDY.md), 코드 설명은
[`CONTROLLER_GUIDE.md`](CONTROLLER_GUIDE.md)에 있다.

---

## 1. 먼저 알아야 할 제약: rw_rate는 update_rate를 넘을 수 없다

"controller_manager는 50 Hz로 두고 하드웨어만 400 Hz로 돌리면 되지 않나?"

**안 된다.** `ros2_control`의 `ResourceManager`가 URDF를 읽을 때 이렇게 자른다.

```cpp
// hardware_interface/src/resource_manager.cpp
hw.rw_rate = (hw.rw_rate == 0 || hw.rw_rate > update_rate) ? update_rate : hw.rw_rate;
```

`<ros2_control rw_rate="...">`는 **controller_manager 주기의 감속(decimation)에만**
쓸 수 있다. 400 Hz로 버스를 돌리려면 `update_rate: 400`이어야 한다.

같은 규칙이 컨트롤러에도 적용된다. 컨트롤러의 `update_rate` 파라미터는
CM 주기 이하여야 하고, CM 주기의 약수가 아니면 경고가 뜬다
(`controller_interface_base.cpp`, `controller_manager.cpp:1668`).

### 그래서 나오는 구조

```
controller_manager RT 스레드 @ 400 Hz  (2.5 ms 예산)
  ├─ DynamixelHardware::read()      직렬 트랜잭션
  ├─ kid_rl::update()               ← 400 Hz로 호출됨
  │    ├─ 매 틱:   watchdog, 낙상 판정, 목표 변화율 제한
  │    └─ 8틱마다: build_observation() → 추론 → update_policy_setpoint()
  ├─ joint_state_broadcaster::update()   (update_rate: 50 으로 감속)
  └─ DynamixelHardware::write()     명령이 바뀐 틱에만 직렬 전송
```

정책 50 Hz는 **컨트롤러 내부 시간 누적 decimation**으로 만든다.

### 왜 컨트롤러 `update_rate: 50`을 쓰지 않는가

쓸 수는 있다. CM이 8틱에 한 번만 `update()`를 부르고, 그 사이 command interface는
마지막 값을 유지한다(zero-order hold). 하지만 다음 두 가지를 잃는다.

1. **watchdog과 낙상 판정이 20 ms 해상도가 된다.** 400 Hz에서는 2.5 ms마다
   검사할 수 있다. 다만 실제 반응 시간은 IMU 발행 주기와 버스 지연에도 좌우된다.
2. **SAFE_HOLD의 목표 변화율 제한이 20 ms 간격으로 동작한다.** 400 Hz에서는
   안전 자세로 돌아가는 목표를 2.5 ms 간격으로 갱신한다.

그래서 `config/controllers.yaml`에서 `kid_rl`의 `update_rate`를 **명시하지 않았다**
(= CM 주기 400을 그대로 상속). 이건 의도적이다. 바꾸지 말 것.

### 코드에서의 대응

```cpp
// 50 Hz: 정책이 setpoint 를 만든다
void update_policy_setpoint() {
  policy_targets_[i] = clamp(default + scale*clip(action), joint limits);
}

// 400 Hz: 기본 설정에서는 LPF 없이 변화율 제한만 적용한다
void slew_targets(double dt) {
  const double alpha = 1.0 - std::exp(-dt / filter_time_constant_s_);
  const double max_delta = target_rate_limit_ * dt;
  targets_[i] += clamp(alpha * (policy_targets_[i] - targets_[i]), ±max_delta);
}
```

> **필터 계수는 주기 독립이어야 한다.** LPF를 켠 경우 `target_filter_alpha`는
> "정책 주기 1회에 제거할 오차의 비율"이라는 뜻이며, 내부적으로는 configure에서
> 시간 상수로 변환해 둔다.
> ```
> τ = −(1/policy_rate) / ln(1 − α)
> α_tick = 1 − exp(−dt/τ)
> ```
> 이렇게 안 하면 400 Hz에서 필터가 8배 빨라져 사실상 없는 것과 같아진다.

관절 속도 관측은 위치를 차분해서 만들지 않는다. 하드웨어가 제공하는
`Present Velocity`를 그대로 읽는다. 위치 차분은 양자화 잡음을 키우므로 400 Hz
자체만으로 더 좋은 속도 추정치가 보장되지 않는다.

DYNAMIXEL의 Goal Position 레지스터는 새 값이 올 때까지 유지된다. 따라서
`write()`는 마지막 성공 전송 이후 명령값이 바뀐 경우에만 Sync/Bulk Write를
보낸다. 이 동작은 DYNAMIXEL의 Bus Watchdog이 비활성화되어 있다는 전제다.

---

## 2. 이 시스템에 존재하는 스레드 전부

| # | 스레드 | 누가 만드나 | 주기 | 우선순위 |
| --- | --- | --- | --- | --- |
| 1 | **CM 실시간 루프** | `ros2_control_node`가 `std::thread`로 | 400 Hz | SCHED_FIFO 80 |
| 2 | rclcpp executor (MultiThreaded) | `ros2_control_node` 메인 스레드 | 이벤트 구동 | 일반 |
| 3 | DXL 상태 퍼블리셔 | `realtime_tools::RealtimePublisher` | read()가 깨움 | 일반 |
| 4 | joint_state_broadcaster 퍼블리셔 | 〃 | 50 Hz | 일반 |
| 5 | **ONNX Runtime intra-op 풀** | ORT (미구현) | — | 기본값 = 위험 |
| 6 | DDS 미들웨어 스레드 | rmw 구현 | — | 일반 |
| 7 | IMU 드라이버 | 별도 프로세스/노드 | 200~400 Hz | 일반 |
| 8 | (선택) 하드웨어 async 스레드 | `is_async="true"` | rw_rate | 설정값 |
| 9 | (선택) 컨트롤러 async 스레드 | `is_async: true` | update_rate | 기본 50 |

`update()`가 도는 것은 **1번뿐이다.** 나머지는 전부 그 주변이다.
2번(executor)이 `~/imu`, `~/cmd_vel`, `~/enable_policy` 콜백을 실행하고,
그 결과가 `RealtimeBuffer`를 통해 1번에게 전달된다.

---

## 3. 지금 여기서 처리해야 할 것

### 3.1 ✅ 이미 되어 있는 것

**스레드 간 전달**
여러 필드가 한 샘플을 이루는 게임패드와 IMU는 각각 `RealtimeBuffer<T>`를 사용한다.
단일 `bool`인 정책 활성화 값은 `std::atomic<bool>`로 전달한다.

```cpp
CommandSample command = gamepad_.read_from_rt();
const ImuSample imu = imu_.read_from_rt();
const bool policy_requested = policy_enabled_.load(std::memory_order_relaxed);
```

`RealtimeBuffer::readFromRT()`가 반환한 포인터는 각 입력 모듈 안에서 즉시 값으로
복사하므로, 컨트롤러는 버퍼 내부 포인터를 보관하지 않는다.

**RT 경로 무할당**
모든 벡터를 `on_configure()`에서 최종 크기로 `assign()`한다. `update()`에는
`push_back`도 `resize`도 `std::string` 생성도 없다.

**예외 없는 시간 비교**
타임스탬프를 `rclcpp::Time`이 아니라 `std::int64_t` 나노초로 저장한다.
`rclcpp::Time::operator-`는 clock type이 다르면 `std::runtime_error`를 던지는데,
`update()`에서 예외가 나가면 컨트롤러가 죽는다.

**RT 스케줄링 설정**
`config/controllers.yaml`에 `lock_memory: true`, `thread_priority: 80`.

**dt 폭주 방어**
RT 스레드가 굶어서 `period`가 크게 들어와도 100 ms로 자른다. 안 그러면
변화율 제한이 무력화된다.

### 3.2 ⚠️ 실기 전에 반드시 해야 하는 것

#### (a) rtprio 한도 열기 — 안 하면 RT 우선순위가 안 먹는다

스모크 테스트에서 실제로 나온 경고:

```
[WARN] [controller_manager]: Could not enable FIFO RT scheduling policy:
       with error number <1>(Operation not permitted)
```

`thread_priority: 80`을 적어도 커널이 거부하면 그냥 일반 스케줄링으로 돈다.
**400 Hz에서는 이 상태로 못 버틴다.**

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<'EOF'
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
@realtime   -   nice       -20
EOF
# 재로그인 후 확인
ulimit -r        # 99 가 나와야 함
ulimit -l        # unlimited
```

기동 후 실제로 적용됐는지:

```bash
ps -eLo pid,tid,cls,rtprio,comm | grep ros2_control_node
# cls 가 FF, rtprio 가 80 인 스레드가 하나 있어야 한다
```

> **실측 근거.** mock 하드웨어(직렬 통신 0 ms)로 `update_rate: 400`을 돌렸을 때,
> RT 스케줄링이 거부된 상태에서 이미 오버런이 났다.
> ```
> [WARN] Could not enable FIFO RT scheduling policy: Operation not permitted
> [WARN] Overrun detected! ... desired rate of 400 Hz. The loop took 3.386 ms (missed cycles : 2)
> ```
> 계산상 여유가 아무리 많아도 스케줄링이 안 되면 소용없다.

#### (b) CPU 격리

400 Hz RT 루프가 다른 작업과 코어를 나눠 쓰면 지터가 생긴다.

```bash
# /etc/default/grub  →  GRUB_CMDLINE_LINUX_DEFAULT 에 추가
isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0,1
sudo update-grub && sudo reboot
```

그다음 `controllers.yaml`에서:

```yaml
cpu_affinity: [2]
```

USB 인터럽트도 격리 코어에서 빼야 한다 (`irqaffinity=0,1`이 그 역할).
Raspberry Pi 같은 4코어에서는 2코어를 격리하면 나머지가 부족할 수 있으니
`isolcpus=3` 하나만으로 시작하는 것도 방법이다.

#### (c) ONNX Runtime 스레드 봉인 — **이게 제일 위험하다**

ORT는 기본적으로 **코어 수만큼 intra-op 스레드를 띄우고, 작업이 없어도 spin한다.**
그 스레드들이 격리 코어까지 침범하면 RT 루프가 선점당한다. 세션을 만들 때 반드시:

```cpp
Ort::SessionOptions opts;
opts.SetIntraOpNumThreads(1);
opts.SetInterOpNumThreads(1);
opts.SetExecutionMode(ORT_SEQUENTIAL);
opts.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
// spin 억제 (ORT 1.14+)
opts.AddConfigEntry("session.intra_op.allow_spinning", "0");
opts.AddConfigEntry("session.inter_op.allow_spinning", "0");
```

ai_sapiens와 roa 둘 다 정확히 이 설정을 쓴다. MLP 하나 돌리는 데 멀티스레드는
오버헤드만 늘린다.

그리고 **세션 생성은 `on_configure()`에서 한 번만.** `Ort::Session` 생성은
파일 I/O + 파싱 + 그래프 최적화 + 할당으로 수십~수백 ms가 걸린다.

`Run()`도 할당 없이 하려면 입출력 텐서를 미리 만들어 둔다.

```cpp
// on_configure()
memory_info_ = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
input_tensor_ = Ort::Value::CreateTensor<float>(
  memory_info_, obs_buf_.data(), obs_buf_.size(), input_shape_.data(), input_shape_.size());
// update() 에서는 obs_buf_ 내용만 갱신하고 Run() 호출
```

#### (d) 추론 시간을 측정하고 예산에 맞는지 확인

**400 Hz의 한 틱 예산은 2.5 ms다.** 그 안에 전부 들어가야 한다.

| 항목 | 예상 | 비고 |
| --- | --- | --- |
| DXL read (Fast Sync Read, 18축, 10바이트) | 0.8~1.3 ms | baud에 종속, 4절 |
| DXL write (Sync Write, 18축, 4바이트) | 0.3~0.4 ms | 〃 |
| `kid_rl::update()` — 정책 없는 틱 | < 20 µs | slew + watchdog뿐 |
| `kid_rl::update()` — 정책 틱 | ? | **측정 필요** |
| joint_state_broadcaster (50 Hz) | 수 µs | |

`0.8 + 0.3 = 1.1 ms`가 통신에 나가면 추론에 쓸 수 있는 건 **1.4 ms 정도**다.
18 DoF MLP는 보통 50~200 µs라 충분하지만, 관측 history가 5스텝이거나 네트워크가
크면 달라진다. **반드시 실측할 것.**

```bash
# controller_manager 실행 통계
ros2 topic echo /controller_manager/introspection_data/values --once
# 또는
ros2 topic hz /joint_states     # 감속 후에도 안정적인지
```

`controller_manager`는 컨트롤러별 실행시간과 주기 통계를 diagnostics로 낸다.
`periodicity`가 400에서 벗어나거나 `execution_time`이 예산을 넘으면 로그에 경고가 나온다.

#### (e) 예산을 못 지킬 때의 탈출구 — 순서대로 시도

1. **baud rate를 올린다** (4절). 제일 효과가 크고 부작용이 없다.
2. **읽는 항목을 줄인다.** `Present Current`가 필요 없으면 xacro의
   `<state_interface name="Present Current"/>`를 뺀다.
3. **`update_rate`를 낮춘다.** 400 → 250 Hz. 정책이 50 Hz이므로
   실제 손해는 slew 해상도뿐이다. **가장 현실적인 타협.**
4. **컨트롤러를 async로 돌린다.**
   ```yaml
   kid_rl:
     is_async: true
     thread_priority: 60      # CM(80)보다 낮게
   ```
   `update()`가 별도 스레드로 빠지고 CM 루프는 그 결과를 가져다 쓴다.
   추론이 길어도 RT 루프가 안 막힌다. **대신 한 주기의 지연이 추가되고**,
   command interface 접근이 두 스레드로 나뉜다. 보행은 지연에 민감하니 마지막 수단.
5. **하드웨어를 async로 돌린다.**
   ```xml
   <ros2_control name="KidDynamixel" type="system" is_async="true" rw_rate="400">
   ```
   직렬 트랜잭션이 별도 스레드로 빠진다. 역시 한 주기 지연이 추가된다.

> **권장: 4·5번은 3번을 시도한 뒤에.** 동기 구조가 지연이 가장 작고 추론하기도 쉽다.
> "일단 async로 해두면 안전하겠지"는 틀렸다 — async는 지터를 지연으로 바꿀 뿐이다.

### 3.3 🔍 알고는 있어야 할 것 (지금 당장 문제는 아님)

**executor 기아(starvation)**
RT 스레드가 SCHED_FIFO 80으로 격리 코어를 점유하면, executor 스레드가 그 코어에서
밀릴 수 있다. IMU 콜백이 100 ms 밀리면 `imu_timeout_s: 0.10`이 만료되어 FAULT가 난다.
→ `cpu_affinity`로 RT를 격리 코어에, executor를 나머지 코어에 두면 해결된다.

**`RCLCPP_WARN_THROTTLE`의 비용**
throttle 매크로는 내부적으로 `get_clock()->now()`를 부른다. 대부분 vDSO라 저렴하지만
공짜는 아니고, 조건이 참일 때 문자열 포매팅이 일어난다. 지금은 예외 경로에만 있어서
괜찮지만, 정상 경로에 로그를 추가하지 말 것.

**`dynamixel_hardware_interface`의 RT 위생**
`read()` 오류 경로에 `RCLCPP_ERROR_STREAM`이 있다. `std::stringstream` 생성 =
힙 할당이다. 버스가 불안정하면 매 사이클 할당이 일어나 지터를 만든다.
우리 패키지가 아니라 손대지 않았지만, 400 Hz에서 통신 오류가 잦으면 이게 원인일 수 있다.

**`joint_state_broadcaster`를 400 Hz로 돌리지 말 것**
`/joint_states`를 400 Hz로 뿌리면 DDS와 executor에 부담이 크다.
`update_rate: 50`으로 감속해 두었다.

### 3.4 조용히 무시되는 설정 함정

컨트롤러의 `update_rate` / `is_async` / `thread_priority`를 `controller_manager:`
하위에 적으면 **에러도 경고도 없이 무시된다.**

```yaml
# ✗ 무시됨 — controller_manager 하위 블록은 `type` 만 읽는다
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      update_rate: 50

# ✓ 먹음 — 컨트롤러 자신의 노드 파라미터
joint_state_broadcaster:
  ros__parameters:
    update_rate: 50
```

이 값들은 `controller_interface_base.cpp`가 컨트롤러 노드에 `auto_declare` 하는
파라미터다. 확인 방법은 `ros2 topic hz`로 실제 주기를 보거나
`ros2 param get /<controller> update_rate`뿐이다.

---

## 4. DYNAMIXEL 버스 예산 — 400 Hz가 가능한가

### 계산

`dynamixel_hardware_interface`는 indirect address로 읽을 항목을 한 블록으로 묶고
**Fast Sync Read**(단일 상태 패킷)를 쓴다. 이게 400 Hz를 가능하게 하는 전제다.

XL330 컨트롤 테이블에서 우리가 읽는 세 항목은 연속이다:
`Present Current(126, 2B)`, `Present Velocity(128, 4B)`, `Present Position(132, 4B)`
= **10바이트/서보**.

18축 기준 한 사이클의 바이트 수:

| 패킷 | 계산 | 바이트 |
| --- | --- | --- |
| Fast Sync Read 명령 | 12 + 18(ID) + 2 | 32 |
| Fast Sync Read 응답 | 9 + 18 × (1 err + 1 ID + 10 data + 2 crc) + 2 | 263 |
| Sync Write Goal Position | 12 + 18 × (1 + 4) + 2 | 104 |
| **합계** | | **≈ 400 B** |

8N1이면 1바이트 = 10비트다.

| baud | 바이트 시간 | 사이클 시간 | 400 Hz(2.5 ms) 여유 |
| --- | --- | --- | --- |
| 1 Mbps | 10 µs | **4.0 ms** | ❌ 불가 (250 Hz도 빠듯) |
| 2 Mbps | 5 µs | 2.0 ms | ⚠️ 80% 점유, 여유 없음 |
| 3 Mbps | 3.33 µs | 1.33 ms | ✅ 53% |
| 4 Mbps | 2.5 µs | 1.00 ms | ✅ 40% |

**현재 xacro의 `baud:=1000000`으로는 400 Hz가 물리적으로 불가능하다.**
최소 3 Mbps, 가능하면 4 Mbps로 올려야 한다. XL330은 4.5 Mbps까지 지원한다.

> 배선이 길거나 데이지체인이 많으면 고속에서 CRC 오류가 는다.
> 4 Mbps로 올린 뒤 반드시 오류율을 확인할 것.

### 반드시 같이 해야 하는 두 가지

#### Return Delay Time = 0

기본값 250 = **500 µs**다 (단위 2 µs). 2.5 ms 예산의 20%를 그냥 날린다.

```xml
<param name="Return Delay Time">0</param>
```

#### FTDI latency timer = 1 ms

**이게 제일 자주 사람을 잡는다.** U2D2는 FT232H를 쓰는데, 리눅스 ftdi_sio 드라이버의
기본 latency timer가 **16 ms**다. 짧은 패킷이 커널 버퍼에서 최대 16 ms 대기한다.
400 Hz(2.5 ms)는커녕 60 Hz도 안 나온다.

```bash
# 확인
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# 임시 변경
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# 영구 적용 (udev)
sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules > /dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`dynamixel_hardware_interface/scripts/create_udev_rules`에 관련 스크립트가 있으니
같이 확인할 것.

### 400 Hz가 정말 필요한가 — 한 번은 물어볼 것

XL330의 내부 위치 제어 루프는 펌웨어에서 이미 훨씬 빠르게 돈다. 우리가 400 Hz로
목표를 갱신해서 얻는 것은 **목표 궤적의 매끄러움**이지 제어 대역폭 자체가 아니다.

200 Hz(5 ms 예산, 1 Mbps로도 가능)에서 이미 정책 스텝 사이를 4번 보간한다.
400 Hz는 8번이다. 체감 차이가 있을 수도, 없을 수도 있다.

**권장 순서**: 200 Hz + 1 Mbps로 먼저 걷게 만들고, 그다음 baud를 올려
400 Hz를 시도해 실제로 나아지는지 측정한다. 두 변수를 동시에 바꾸면
문제가 생겼을 때 원인을 못 찾는다.

---

## 5. 브링업 체크리스트

```
□ rtprio / memlock 한도 열기 (3.2a)          → ulimit -r 이 99
□ ros2_control_node RT 스레드 확인            → ps -eLo cls,rtprio | grep FF
□ FTDI latency_timer = 1 (4절)               → cat /sys/.../latency_timer
□ Return Delay Time = 0 (4절)
□ baud rate 결정 및 CRC 오류율 확인
□ update_rate 200 으로 먼저 동작 확인
□ overrun / periodicity 통계 확인
□ update_rate 400 으로 올리고 재확인
□ (ONNX 붙인 뒤) 추론 시간 측정
□ (ONNX 붙인 뒤) ORT 스레드 1/1/SEQUENTIAL 확인
□ CPU 격리 및 cpu_affinity 적용
□ 장시간(30분+) 무오버런 확인
```

---

## 6. 요약: 여기서 처리해야 할 것

**코드에서 (이미 함)**
- lock-free 버퍼, 틱당 1회 스냅샷
- RT 경로 무할당
- 정수 나노초 watchdog
- 400 Hz slew + 주기 독립 필터
- dt clamp

**설정에서 (이미 함, 검증 필요)**
- `update_rate: 400`, `lock_memory: true`, `thread_priority: 80`
- `joint_state_broadcaster` 50 Hz 감속
- `kid_rl`의 `update_rate` 미지정 (CM 주기 상속)

**시스템에서 (아직 안 함)**
- rtprio/memlock 한도
- CPU 격리 + affinity
- FTDI latency timer
- Return Delay Time
- baud rate 상향 (1 Mbps → 3~4 Mbps)

**ONNX를 붙일 때 (아직 안 함)**
- 세션은 configure에서 한 번만
- `SetIntraOpNumThreads(1)`, `SetInterOpNumThreads(1)`, `ORT_SEQUENTIAL`, spin 비활성
- 입출력 텐서 사전 할당
- 추론 시간 실측 후 예산 판단
- 초과 시: baud↑ → 읽을 항목↓ → update_rate↓ → async
