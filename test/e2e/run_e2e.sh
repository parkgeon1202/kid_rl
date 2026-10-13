#!/usr/bin/env bash
# 컨트롤러 종단 테스트 (하드웨어 없음).
#   mock_components/GenericSystem + socat 가상 시리얼로 흘리는 EBIMU 형식 IMU + 위상 모델로
#   kid_rl 을 POLICY_RUNNING 까지 돌려 CSV 를 남기고, check_e2e.py 가 검사한다:
#     - 첫 추론 틱의 위상이 0 인지 (학습의 리셋 직후 관측)
#     - 위상이 첫 추론 이후 실제 경과 시간을 따라가는지
#     - 추론을 건너뛴 틱이 없는지
#     - CSV 관측(위상 포함)으로 ONNX 를 다시 돌리면 기록된 action 이 나오는지, 위상을 한 틱
#       밀면 어긋나는지
# 사용: RUN_S=10 test/e2e/run_e2e.sh && <mjlab venv python> test/e2e/check_e2e.py test/e2e/out/run.csv
# 필요: socat, deploy_ws 빌드, onnxruntime/onnx 가 있는 파이썬(check_e2e.py).
# 컨트롤러 종단 테스트: mock 하드웨어 + 가상 IMU + 위상 모델로 POLICY_RUNNING 을 돌리고 CSV 를 남긴다.

H="$(cd "$(dirname "$0")" && pwd)"
PKG="$(cd "$H/../.." && pwd)"
D="$H/out"; mkdir -p "$D"
source /opt/ros/jazzy/setup.bash
source "$PKG/../../install/setup.bash"
export ROS_DOMAIN_ID=87 RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}]: {message}'
# 이름으로 pkill 하면 이 스크립트를 부른 셸까지 맞을 수 있다. 띄운 PID 와 그 자손만 정리한다.
PIDS=""
killtree() { local c; for c in $(pgrep -P "$1"); do killtree "$c"; done; kill "$1" 2>/dev/null; }
cleanup() { for p in $PIDS; do killtree "$p"; done; }
trap cleanup EXIT
rm -f "$D"/imu_a "$D"/imu_b "$D"/run.csv
socat pty,raw,echo=0,link="$D/imu_a" pty,raw,echo=0,link="$D/imu_b" >/dev/null 2>&1 &
PIDS="$PIDS $!"
sleep 1
python3 "$H/imu_feed.py" "$D/imu_a" &
PIDS="$PIDS $!"
xacro "$PKG/description/kid_rl.urdf.xacro" > "$D/real.urdf"
python3 "$H/make_mock.py" "$D/real.urdf" "$PKG/config/controllers.yaml" "$D" \
  "$PKG/test/data/policy_gait_phase.onnx" "$D/imu_b" "$D/run.csv"
ros2 run robot_state_publisher robot_state_publisher "$D/mock.urdf" --ros-args -r __node:=rsp_e2e > "$D/rsp.log" 2>&1 &
PIDS="$PIDS $!"
ros2 run controller_manager ros2_control_node --ros-args --params-file "$D/params.yaml" \
  -r ~/robot_description:=/robot_description -r __ns:=/ > "$D/cm.log" 2>&1 &
PIDS="$PIDS $!"
sleep 6
ros2 run controller_manager spawner kid_rl --param-file "$D/params.yaml" -c /controller_manager > "$D/spawn.log" 2>&1
sleep 3   # READY_POSE (2 s) -> SAFE_HOLD
ros2 topic pub --times 3 /kid_rl/enable_policy std_msgs/msg/Bool "data: true" > /dev/null 2>&1
sleep ${RUN_S:-8}
ros2 control set_controller_state kid_rl inactive -c /controller_manager > /dev/null 2>&1
sleep 2
grep -E "gait_phase|Loaded ONNX|Mode |Fault|ERROR|Could not" "$D/cm.log" | head -20
