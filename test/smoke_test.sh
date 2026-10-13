#!/usr/bin/env bash
# Hardware-free smoke test for kid_rl/KidRlController.
#
# Brings up a controller_manager on mock_components/GenericSystem, activates the
# controller, and walks it through the whole mode machine:
#
#   WAITING_FOR_STATE -> READY_POSE -> SAFE_HOLD -> POLICY_RUNNING -> FAULT
#
# It also checks the two behaviours that are easy to regress:
#   * after READY_POSE the commanded pose equals default_positions exactly
#   * a missing /kid_rl/cmd_vel degrades to a zero command, it does NOT fault
#   * a missing /kid_rl/imu trips the watchdog and latches FAULT
#
# Usage:  bash test/smoke_test.sh
set -u

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$(mktemp -d)"
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}]: {message}'

cleanup() {
  pkill -P $$ >/dev/null 2>&1
  pkill -f ros2_control_node >/dev/null 2>&1
  pkill -f robot_state_publisher >/dev/null 2>&1
  pkill -f "topic pub" >/dev/null 2>&1
}
trap cleanup EXIT

echo "logs: ${LOG_DIR}"

ros2 run robot_state_publisher robot_state_publisher \
  "${PKG_DIR}/test/mock/kid_mock.urdf" > "${LOG_DIR}/rsp.log" 2>&1 &
ros2 run controller_manager ros2_control_node --ros-args \
  --params-file "${PKG_DIR}/config/controllers.yaml" > "${LOG_DIR}/cm.log" 2>&1 &
sleep 8

ros2 control load_controller --set-state active joint_state_broadcaster >/dev/null 2>&1
ros2 control load_controller --set-state active kid_rl >/dev/null 2>&1
sleep 4

echo "--- pose after READY_POSE (expect default_positions) ---"
ros2 topic echo --once /joint_states 2>/dev/null | sed -n '/name:/,/effort:/p'

echo "--- enabling the policy with an IMU but no cmd_vel ---"
ros2 topic pub -r 100 /kid_rl/imu sensor_msgs/msg/Imu \
  "{orientation: {w: 1.0}, angular_velocity: {x: 0.0}}" >/dev/null 2>&1 &
IMU_PID=$!
sleep 2
ros2 topic pub -1 /kid_rl/enable_policy std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
sleep 4

echo "--- stopping the IMU: the watchdog must latch FAULT ---"
kill "${IMU_PID}" >/dev/null 2>&1
sleep 3

echo
echo "=================== controller log ==================="
grep -aE "\[kid_rl\]" "${LOG_DIR}/cm.log" | grep -av "node arguments"
