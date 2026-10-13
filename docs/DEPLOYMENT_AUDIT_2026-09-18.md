# KID RL 실물 배포 감사 보고서

작성일: 2026-09-18  
감사 대상: `deploy_ws/src/kid_rl`, `deploy_ws/src/dynamixel_hardware_interface`  
학습 모델 비교 기준: `mjlab_kid_RL/robot/kid_rl/kid_RL.xml`과 지정된 `policy.onnx`

## 1. 최종 판정

현재 소프트웨어는 **구성과 빌드 검증 단계까지는 진행할 수 있지만, 아직 실물 보행을 바로
시작하면 안 된다.** 다음 세 항목은 코드만 보고 확정할 수 없으며, 지지대 위에서 확인하기
전까지 실물 구동 차단 조건이다.

1. 25개 모터의 `Drive Mode`(모터 양의 회전 방향을 정하는 설정)와 영점이 모두 실측되지 않았다.
2. IMU(관성 측정 장치)가 기본 설정에서 꺼져 있으며, 실제 EBIMU 패킷 형식과 장착 축도 확인되지 않았다.
3. 현재 `robot_description`은 제어 인터페이스 파싱용 더미이고 실제 기구학 모델이 아니다.

이번 감사에서 즉시 고친 사항은 다음과 같다.

- `controller_manager`가 `/robot_description`을 실제로 받도록 topic remap(토픽 이름 연결)을 추가했다.
- 가짜 관절 트리에서 거짓 TF(좌표계 사이의 변환)가 발행되지 않도록 `/joint_states` 입력을 분리했다.
- 더미 모델 사용 중임을 launch 화면에 경고하도록 했다.
- ONNX Runtime SDK(ONNX 모델 실행용 C++ 라이브러리)의 실제 설치 위치를 자동 탐색하도록 했다.
- Dynamixel 인터페이스 개수 검사의 논리식 `&&`를 `||`로 고쳤다.
- 컨트롤러가 `<joint>/hardware_state`를 실제로 확보하고, 서보 하드웨어 오류가 있으면 잠금
  FAULT(사람이 재시작하기 전까지 풀리지 않는 오류 상태)로 들어가게 했다.

## 2. 사용자가 지적한 `kid_dummy_joint` 판정

사용자의 판단이 맞다. 기존 매크로는 MJLab의 XML을 URDF(로봇 링크와 관절을 기술하는 XML
형식)로 옮긴 것이 아니다.

```text
현재 제어 스키마
base_link
 ├─ torso_yaw_link
 ├─ neck_yaw_link
 ├─ head_pitch_link
 ├─ ...
 └─ right_ankle_roll_crank_link
```

모든 관절이 `base_link`의 원점에 있고 축도 `0 0 1`이다. 이 구조가 `xacro`(반복 가능한
URDF 매크로 문법) 및 `check_urdf` 검사를 통과하는 이유는 문법적으로 유효한 트리이기 때문이다.
검사 통과가 실제 로봇과 일치한다는 뜻은 아니다.

이 더미가 모터 I/O(입출력)를 가능하게 하는 이유는 ros2_control 파서가 `<ros2_control>`에
나열된 관절 이름을 URDF에서도 찾기 때문이다. Dynamixel 위치 변환은 현재 URDF의 축이나 링크
길이가 아니라 hardware plugin(하드웨어 드라이버를 실행 중 불러오는 모듈)의 transmission
(관절과 모터 값의 변환 규칙)을 사용한다.

반대로 다음 용도에는 사용할 수 없다.

- RViz(ROS 로봇 시각화 도구)
- TF, 순기구학(관절각으로 링크 위치를 계산하는 방법)
- 충돌 검사
- IMU 장착 프레임 변환
- 발 위치나 몸체 자세 계산

이를 숨기지 않도록 launch에서 더미 모델로부터 동적 TF가 발행되지 않게 수정했다. 모터 제어에
필요한 `/robot_description` 발행은 유지한다.

## 3. MJLab 모델과의 정확한 비교

지정된 ONNX 파일은 메타데이터(모델 안에 저장한 설명 정보)에 다음 33개 관절 이름을 가진다.
그중 이름이 `_roll_cap`, `_roll_actual`로 끝나는 8개는 passive joint(모터가 직접 구동하지
않는 종동 관절)이며 정책 입력과 출력에서는 제외된다. 결과적으로 ONNX 입출력과 배포 제어기는
25축이다.

```text
33 movable joints
 = 25 actuated joints(모터가 직접 구동하는 관절)
 + 8 passive linkage joints(링크 기구에 의해 따라 움직이는 관절)
```

MJLab의 실제 트리는 다음 구조다.

```text
base_link
├─ torso → neck → head
│  ├─ left shoulder → upper arm → shoulder yaw → lower arm → hand
│  └─ right shoulder → upper arm → shoulder yaw → lower arm → hand
├─ left hip bracket
│  ├─ hip crank → hip cap
│  └─ hip actual → thigh → knee → ankle → foot → ankle crank → cap
└─ right hip bracket
   └─ 좌측과 대칭
```

축도 더미와 다르다.

| 관절군 | MJLab 축 |
|---|---|
| torso, neck | `+Z` |
| head | `+Y` |
| 왼팔 pitch | `+Y` |
| 오른팔 pitch | `-Y` |
| shoulder roll, roll linkage | `+X` |
| shoulder yaw, hip yaw | `-Z` |
| 왼다리 pitch | `+Y` |
| 오른다리 pitch | `-Y` |

정식 URDF 교체는 단순 복사가 아니다. MJCF(MuJoCo 모델 파일)의 tendon(여러 관절의 운동을
묶는 제약)은 트리만 표현하는 URDF에 그대로 들어가지 않는다. 따라서 다음 둘을 분리해야 한다.

- TF와 시각화를 위한 33관절 링크 트리
- 실제 crank encoder(크랭크 모터의 각도 센서)와 정책의 25개 제어 좌표 사이 변환

구형 `kid_RL_v3.xacro`는 19축 계열이므로 복사하면 안 된다.

### 3.1 MJLab 저장소 자체의 버전 모순

현재 `kid_rl_constants.py`는 `kid_RL.xml`이 아니라 구형 `robot_draft.xml`을 가리킨다.
`robot_draft.xml`은 19개 active joint(정책이 직접 제어하는 관절)를 갖지만, 환경 설정 주석과
지정된 ONNX는 25개를 전제로 한다. 따라서 “현재 MJLab 폴더”만 기준으로 삼으면 안 된다.

이번 배포의 source of truth(최종 판단 기준)는 다음 세 자료의 조합이다.

1. 지정된 2026-09-08 ONNX의 `joint_names` 메타데이터
2. 그 25축 이름과 구조가 일치하는 `kid_RL.xml`
3. 정책을 export(학습 결과를 ONNX로 변환)한 실행 기록의 관측·액션 설정

## 4. ros2_control 연결 감사

### 4.1 정상 연결된 부분

- `kid_rl_plugin.xml`의 class 이름과 C++ `PLUGINLIB_EXPORT_CLASS`가 일치한다.
- `controllers.yaml`은 `kid_rl/KidRlController`를 요청한다.
- YAML과 Dynamixel xacro 모두 25개 관절을 가진다.
- 25개 관절 이름 집합이 서로 같다.
- Dynamixel ID는 0부터 24까지 중복 없이 한 번씩 사용한다.
- YAML 순서와 xacro 선언 순서는 다르지만, 컨트롤러는 이름을 찾아 index map(이름을 배열
  위치에 연결한 표)을 만들므로 정상이다.
- command interface(컨트롤러가 쓰는 값)는 관절별 position 한 개다.
- state interface(컨트롤러가 읽는 값)는 관절별 position, velocity, hardware_state 세 개다.

### 4.2 launch에서 고친 치명적 연결

`controller_manager`는 자신의 `~/robot_description`을 구독한다. 반면
`robot_state_publisher`는 `/robot_description`을 발행한다. 기존 launch에는 두 이름을 잇는
remap이 없었다. 그 상태에서는 ResourceManager(하드웨어와 인터페이스를 보관하는 ros2_control
객체)가 URDF를 받지 못해 controller spawner(컨트롤러를 로드하는 보조 프로세스)가 실패할 수
있다.

수정 후 연결은 다음과 같다.

```text
robot_state_publisher:/robot_description
                │ remap
                ▼
controller_manager:~/robot_description
                │
                ▼
ResourceManager → DynamixelHardware → KidRlController
```

공식 controller_manager 문서도 robot description 구독과 remap 방식을 명시한다.

- <https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html>
- <https://github.com/ros-controls/ros2_control_demos>

### 4.3 controller inactive와 hardware inactive는 다르다

현재 `kid_rl` controller는 기본적으로 inactive(업데이트를 실행하지 않는 lifecycle 상태)로
spawn된다. 그러나 Dynamixel hardware component(하드웨어 플러그인의 lifecycle 객체)는 별도
지정이 없으면 자동 activate된다.

`DynamixelHardware::start()`는 현재 위치를 목표 위치로 한 번 쓰고 Torque Enable(모터가 힘을
내도록 하는 설정)을 켠다. 따라서 “controller가 inactive이므로 모터에 아무 일도 일어나지
않는다”는 설명은 틀리다. 더구나 `disable_torque_at_init=true`는 초기화 도중 기존 토크를 먼저
끄고 다시 켤 수 있다.

결론: 첫 launch부터 로봇을 지지대에 올리고 비상 정지 수단을 준비해야 한다. 향후에는
`hardware_components_initial_state.inactive`를 사용하고, 운영자가 명시적으로 하드웨어를
activate한 뒤 controller를 activate하는 2단계 절차가 더 안전하다. 이 변경은 현재 spawner
순서도 함께 바꿔야 하므로 자동 적용하지 않았다.

## 5. 정책과 관측 계약

ONNX shape(텐서의 각 축 크기)는 다음과 같다.

```text
입력 shape:  [1, 84]
연산 대상:   feature 축 84개
출력 shape:  [1, 25]
```

84개 관측 순서는 다음과 같다.

```text
body angular velocity  3
projected gravity      3
joint position error  25
joint velocity        25
previous action       25
velocity command       3
합계                  84
```

`PolicyRunner`는 configure 시점에 다음을 검사한다.

- 입력과 출력이 각각 하나인지
- 두 tensor(같은 타입 값을 다차원으로 배열한 객체)가 `float`인지
- shape가 `[1,84]`, `[1,25]`인지
- ONNX `joint_names` 순서에서 passive 8축을 뺀 결과가 YAML 순서와 같은지
- warm-up inference(실제 운전 전에 한 번 미리 실행하는 추론)가 성공하는지

입출력 vector와 tensor memory(텐서가 참조하는 실제 메모리)는 configure에서 미리 할당된다.
정책 update에서는 새 tensor를 만들지 않아 allocation(동적 메모리 할당)으로 인한 지연을 줄였다.

## 6. 주기와 실시간 경로

목표 주기는 다음과 같다.

```text
controller_manager / hardware read-write: 200 Hz = 5 ms
ONNX policy inference:                     50 Hz = 20 ms
```

한 controller tick(제어 루프 한 번)은 다음 순서다.

```text
Dynamixel read
 → KidRlController update
   → joint/IMU/gamepad snapshot
   → 4틱마다 ONNX Run
   → position command 갱신
 → Dynamixel write
```

`lock_memory=true`는 mlockall(프로세스 메모리를 RAM에 고정하는 운영체제 호출)을 사용해 page
fault(필요한 메모리 페이지가 RAM에 없어 커널이 불러오는 지연)를 줄인다. `thread_priority=80`은
가능한 경우 controller manager RT thread(정해진 마감시간을 우선하는 제어 스레드)를
`SCHED_FIFO`(높은 우선순위 작업이 양보할 때까지 실행되는 Linux 실시간 스케줄 방식) 우선순위
80으로 요청한다. 권한과 RT kernel(실시간 지연을 줄인 Linux 커널)이 없으면 요청은 실패하며
로그를 확인해야 한다.

주의할 문제도 남아 있다.

- hardware `read()`가 내부에서 `spin_some()`을 실행한다. service callback(서비스 요청을 처리하는
  함수)이 busy-wait(조건이 될 때까지 반복 확인)하거나 reboot를 수행하면 RT loop가 막힐 수 있다.
- `error_timeout_ms=400` 동안 통신 실패를 OK로 반환하여 마지막 state가 최대 400 ms 동안 새 값처럼
  보일 수 있다. 50 Hz 정책은 그동안 오래된 값으로 약 20회 추론할 수 있다.
- startup read timeout(시작할 때 첫 응답을 기다리는 한도)과 runtime timeout(운전 중 통신 실패를
  허용하는 한도)이 같은 파라미터다. 둘을 분리해야 한다.
- “값이 바뀔 때만 SyncWrite(여러 모터에 한 패킷으로 쓰는 명령)” 방식은 정적 목표 패킷이 유실될
  때 재전송하지 않는다. 낮은 주기의 keepalive(같은 명령을 주기적으로 다시 보내는 동작)가 필요하다.

## 7. 게임패드와 IMU

게임패드와 IMU는 ROS topic을 거치지 않고 각각의 입력 스레드에서 장치를 직접 읽는다.
RealtimeBuffer(비실시간 입력 스레드가 쓴 완성 샘플을 실시간 스레드가 일관되게 읽게 하는 버퍼)가
스레드 경계를 보호한다.

게임패드 설정은 현재 다음 값이다.

```text
VID/PID:       0x045e / 0x028e
name hint:     Gulikit X
vx range:      -0.8 .. 0.8
vy range:      -0.6 .. 0.6
wz range:      -0.7 .. 0.7
dead band:     없음
```

IMU는 기본 `enabled=false`다. 그러나 POLICY_RUNNING 진입에는 valid IMU와 projected gravity가
필수다. 따라서 현재 기본 설정으로는 정책이 실행되지 않는 것이 정상이다. 실제 패킷이
`euler_gyro`인지 `quaternion_gyro`인지, 각도와 각속도 단위가 degree인지, 장착 축 변환이 다음과
같은지 실측한 뒤 켜야 한다.

```text
사용자가 설명한 장착:
IMU +Z → ROS -Z
IMU +X → ROS +Y
IMU +Y → ROS +X
```

이 축 변환과 quaternion convention(쿼터니언 성분 순서와 회전 방향 규칙)이 틀리면 standing인데
낙상으로 판단하거나, 반대로 넘어져도 정상으로 판단할 수 있다.

## 8. 모터 설정 감사

학습 모델의 actuator class(모터 동특성 묶음)와 firmware P gain(서보 내부 위치 비례 게인)은
현재 설정과 일치한다.

| 용도 | 모델 계열 | P gain |
|---|---:|---:|
| 목·머리·팔 | MX-28 | 27 |
| torso yaw, hip yaw | MX-64 | 46 |
| hip/ankle roll crank | MX-106 | 103 |
| hip/knee/ankle pitch | XH540 | 150 |

`Operating Mode=3`은 position control(위치 제어), profile velocity/acceleration 0은 servo 내부
trajectory profile(목표를 부드럽게 바꾸는 내장 궤적)을 끄는 설정이다. I/D gain도 학습 모델과
맞추기 위해 0으로 둔다.

하지만 모든 `sign`이 1이고 `Drive Mode`가 0인 것은 실물 근거가 없다. MJCF의 `axis=-Y`와
Dynamixel Drive Mode는 같은 개념이 아니다. 전자는 모델 좌표축이고 후자는 조립된 모터 encoder
부호다. 다음 검사를 25축 각각 수행해야 한다.

1. Torque Off 상태에서 관절을 양의 물리 방향으로 조금 움직인다.
2. ROS position이 MJLab의 해당 관절 좌표에서 양수로 증가하는지 확인한다.
3. 0 rad에서 실제 자세가 ONNX의 default pose 기준과 같은지 확인한다.
4. 어긋나면 Drive Mode와 homing offset(엔코더 영점을 옮기는 값)을 수정한다.
5. 작은 목표 변화로 다시 검증한 뒤에만 READY_POSE를 허용한다.

## 9. FAULT 설계 조사와 권고

핵심 원칙은 controller-local fault(정책과 센서 판단 오류)와 hardware-local fault(버스와 서보
오류)를 분리하는 것이다. 포트 재연결과 reboot는 blocking operation(호출한 스레드를 오랫동안
멈추는 작업)이므로 `KidRlController::update()`에서 실행하면 안 된다.

### 9.1 컨트롤러 상태 권고

```text
POLICY_RUNNING
 ├─ gamepad stale → INPUT_DEGRADED: command=0, 정책은 계속 가능
 ├─ IMU stale / loop overrun → SAFE_HOLD: 정책 중단
 ├─ ONNX 실패 / NaN action → INFERENCE_FAULT: 수동 해제
 ├─ sustained fall → FALL_FAULT: 수동 해제 권장
 └─ servo hardware error → SERVO_FAULT: 수동 점검 후 해제
```

현재 추가한 `hardware_state` 검사는 servo hardware error를 잡아 자동 복구하지 않는 FAULT로
보낸다. 다만 현재 driver가 전체 오류를 모든 관절에 같은 값으로 복사하므로 어느 ID가 고장인지
컨트롤러에서는 알 수 없다. 향후 per-servo bitmask(서보별 오류 비트를 담는 정수)로 고쳐야 한다.

### 9.2 하드웨어 상태 권고

```text
BUS_OK
 → BUS_DEGRADED: 첫 read/write 실패, 정책 즉시 중단, 제한된 횟수만 재시도
 → RECONNECTING: non-RT worker에서 포트 재개방과 sync handler 재구성
 → RECOVERED_PENDING: 25개 ID 응답·유한 state·hardware error=0 확인
 → 현재 position을 goal로 먼저 동기화
 → BUS_OK

기한 또는 횟수 초과 → BUS_FATAL → ros2_control lifecycle 정지
서보 Shutdown       → SERVO_SHUTDOWN → 원인 제거와 사람 확인 뒤 reboot
```

추가할 state interface 권고:

- `bus/comm_ok`
- `bus/state_age_ms`
- `bus/read_fail_count`
- `bus/write_fail_count`
- `<joint>/hardware_error_bits`
- 가능하면 `<joint>/torque_enable`

reconnect/reboot command interface(실시간 명령 값)는 추가하지 않는 편이 맞다. 별도 non-RT
supervisor(상태를 감시하고 lifecycle과 복구 작업을 수행하는 비실시간 관리 객체) 또는 hardware
내부 worker가 처리해야 한다.

### 9.3 Torque와 오류 종류

“오류면 항상 토크 유지”와 “오류면 항상 토크 해제”는 둘 다 안전하지 않다.

- 짧은 통신 끊김: 임의로 Torque Enable을 끄지 말고 제한된 재연결을 시도한다.
- 넘어짐: 강한 position hold가 링크와 모터에 큰 힘을 만들 수 있다. 실기에서 별도 정책을 정한다.
- 과열·과부하 Shutdown: servo가 스스로 torque off할 수 있으며 원인 제거와 reboot가 필요하다.
- 정상 종료: 운영 절차에 맞춰 torque off한다.

DYNAMIXEL Bus Watchdog(호스트 명령이 끊기면 서보 자체가 동작을 제한하는 타이머)를 지원하는
모델에서는 사용을 검토할 수 있다. timeout 뒤 Goal register(목표값 레지스터)가 읽기 전용이 될
수 있으므로 복구 순서는 watchdog clear → 현재 위치 읽기 → goal=current position → watchdog
재설정이어야 한다.

공식 근거:

- ros2_control read/write 오류 처리: <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/doc/handling_errors_during_read_write.rst>
- controller update 오류와 fallback: <https://github.com/ros-controls/ros2_control/blob/master/controller_manager/doc/userdoc.rst>
- 비동기 하드웨어 실행: <https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/asynchronous_components.html>
- 하드웨어 진단 예제: <https://control.ros.org/master/doc/ros2_control_demos/example_17/doc/userdoc.html>
- DYNAMIXEL Bus Watchdog: <https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/#bus-watchdog98>
- DYNAMIXEL Shutdown: <https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/#shutdown63>
- ROBOTIS hardware interface: <https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface>

## 10. 테스트와 환경 감사

확인된 결과:

- xacro 전개 성공
- `check_urdf` 문법 검사 성공
- 출력이 `base_link has 25 children`인 것도 확인되어 더미 구조임을 재확인
- YAML의 joint/default/min/max vector가 모두 25개
- 관절 이름 25개와 DXL ID 25개가 각각 중복 없음
- 설치된 `libkid_rl.so`가 ONNX Runtime 1.29.0을 해석함
- 임시 경로의 clean build(이전 산출물을 재사용하지 않는 빌드)에서 세 패키지 모두 컴파일 성공
- `policy_runner_test` 통과, ONNX `[1,84] → [1,25]` 실행 성공
- launch 파일 로드와 네 개 launch argument 파싱 성공

전체 lint test(코드 형식과 정적 규칙 검사)는 58개 중 실제 코드 관련 검사를 통과했다. 유일한
실패는 sandbox(외부 접근을 제한한 실행 환경)가 `package_format3.xsd` 온라인 XML schema(XML
문서 구조 규칙)를 내려받지 못한 `xmllint` 한 건이다. 로컬 XML 문법과 plugin XML 검사는
통과했으므로 코드 결함으로 분류하지 않는다.

빌드 환경에는 별도 주의가 필요하다. `ROS_DISTRO=jazzy`이지만
`~/ros2_ws/install`의 ros2_control 4.47.0과 `/opt/ros/rolling` 경로가 함께 존재한다. ABI
(컴파일된 라이브러리 사이의 이진 호출 규약)가 다른 배포판이 섞이면 빌드는 되더라도 plugin load
중 충돌할 수 있다. 한 터미널에서 사용할 ROS 배포판과 ros2_control overlay(기본 ROS 위에 덮어
쓴 작업공간)를 고정해야 한다.

기존 `test/smoke_test.sh`와 mock URDF는 18축이며 ROS IMU topic을 발행한다. 현재 제어기는 25축,
직접 serial IMU 구조이므로 이 테스트 결과를 신뢰하면 안 된다. 25축 mock과 가짜 IMU sample을
주입할 seam(시험 코드가 입력을 끼워 넣는 연결점)을 새로 만들기 전까지 폐기 대상으로 본다.

이전 `rl_deploy_controller`의 install 흔적도 `AMENT_PREFIX_PATH`에 남아 있다. 삭제된 plugin XML을
가리키는 dangling symlink(대상이 사라진 심볼릭 링크)가 plugin 검색을 오염시킬 수 있으므로,
새 터미널에서 깨끗한 workspace만 source한 뒤 확인해야 한다.

## 11. 실물 구동 단계별 승인표

### 단계 A: 전원과 토크 없이

- [ ] 깨끗한 Jazzy 환경 하나만 source
- [ ] source에서 전체 빌드 성공
- [ ] xacro와 plugin 조회 성공
- [ ] `/dev/ttyUSB*`, 게임패드 event 장치, EBIMU 권한 확인
- [ ] DXL ID 0..24, baud 3 Mbps, 모델 번호 확인
- [ ] 모터별 영점과 부호 기록

### 단계 B: 지지대 위, 정책 비활성

- [ ] 비상 정지 가능
- [ ] launch 시 torque 상태 변화 관찰
- [ ] controller는 inactive 유지
- [ ] joint state 25개가 유한하고 방향이 맞음
- [ ] 온도와 Hardware Error Status가 읽힘
- [ ] 각 관절에 아주 작은 단일 목표를 보내 방향 확인
- [ ] READY_POSE까지 저속 이동 검증

### 단계 C: IMU와 게임패드

- [ ] upright에서 projected gravity가 대략 `[0, 0, -1]`
- [ ] 각 축 회전 시 angular velocity 부호 확인
- [ ] 100 ms 이상 끊었을 때 IMU watchdog 동작 확인
- [ ] 게임패드 중앙과 최대값 확인
- [ ] 게임패드 분리 시 command가 0으로 가는지 확인

### 단계 D: 정책

- [ ] 로봇을 계속 지지한 상태
- [ ] 50 Hz inference 시간과 200 Hz loop jitter(주기 흔들림) 측정
- [ ] action과 target이 joint limit 내부인지 기록
- [ ] 먼저 `vx=vy=wz=0`
- [ ] 작은 `vx`, `vy`, `wz`를 하나씩 시험
- [ ] 낙상/IMU stale/servo error에서 정책이 즉시 중단되는지 확인

## 12. 남은 구현 작업 우선순위

### P0: 실물 보행 전 필수

1. 25축 방향·영점 calibration(실측값으로 좌표계를 맞추는 작업)
2. EBIMU 패킷·단위·축 검증 후 `imu.enabled=true`
3. hardware inactive 기동과 명시적 activation 절차
4. 33관절 정식 description 또는 별도 TF 모델
5. 오래된 joint state를 즉시 구분할 bus freshness interface

### P1: 첫 통신 시험 직후

1. startup/runtime timeout 분리
2. read/write service 작업을 RT loop 밖으로 이동
3. start 실패와 Torque Enable 실패를 정확히 반환
4. 정적 goal keepalive
5. per-servo 오류와 torque 상태 export
6. 원인별 `FaultCause` enum(허용된 오류 원인을 이름으로 구분하는 열거형)

### P2: 회귀 시험

1. 25축 GenericSystem mock
2. fake gamepad/IMU backend
3. 상태 전이 자동 시험
4. 통신 실패, NaN, ONNX 실패 fault injection(일부러 오류를 넣는 시험)
5. 실제 loop latency와 inference latency 기록

## 13. 이전 요청과 이번 작업의 연결

- 게임패드와 IMU를 별도 C++ 파일과 입력 스레드로 분리했다.
- 두 입력은 ROS topic 대신 직접 장치 I/O와 RealtimeBuffer를 사용한다.
- 게임패드는 `vx`, `vy`, `wz`만 만들며 dead band를 두지 않는다.
- IMU는 각속도와 projected gravity를 observation에 제공한다.
- IMU 장착 축 변환과 gyro scale은 실제 센서 단위를 맞추기 위한 설정이다.
- policy enable 하나는 `std::atomic<bool>`로 공유한다.
- controller interface는 관절 이름으로 매핑하여 manager가 배열 순서를 바꿔도 안전하게 읽는다.
- joint state는 staging vector(완성되기 전 임시 배열)에 먼저 읽고, 전부 정상이면 swap(두 vector의
  내부 메모리 소유권만 맞바꾸는 연산)하여 한 tick의 완전한 snapshot을 만든다.
- ONNX session과 tensor는 configure에서 만들고 update에서는 같은 메모리를 재사용한다.
- 정책 50 Hz와 hardware 200 Hz를 분리하고, 정책 사이에는 마지막 목표를 유지한다.
- 이번 감사에서는 이 구조 위에 launch 연결, hardware error 소비, 모델 진실성 경고를 추가했다.

이 문서는 실제로 확인된 것, 코드상 맞지만 실측이 필요한 것, 아직 구현되지 않은 것을 구분한다.
특히 “컴파일됨”과 “실물에 안전함”을 같은 뜻으로 사용하지 않는다.
