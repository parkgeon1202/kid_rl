#!/usr/bin/env bash
set -u

action="${1:-}"

publish_enable() {
  local value="$1"
  echo "[kid_rl_policy] publishing enable_policy=${value} once"
  ros2 topic pub --once \
    --wait-matching-subscriptions 1 \
    --max-wait-time-secs 3 \
    /kid_rl/enable_policy std_msgs/msg/Bool "{data: ${value}}"
}

controller_state() {
  ros2 control list_controllers 2>/dev/null |
    # Jazzy output is: <name> <controller-type> <lifecycle-state>.
    # Use the last field so this also survives controller types containing no
    # assumptions about the number of preceding columns.
    awk '$1 == "kid_rl" {print $NF; exit}'
}

wait_until_active() {
  local deadline=$((SECONDS + 10))
  local state
  while ((SECONDS < deadline)); do
    state="$(controller_state)"
    if [[ "${state}" == "active" ]]; then
      return 0
    fi
    sleep 0.1
  done
  state="$(controller_state)"
  echo "kid_rl did not reach active state within 10 seconds (state='${state:-unknown}'); policy remains disabled" >&2
  return 1
}

case "${action}" in
  enable)
    # Clear a stale enable request before cycling an active controller.  A
    # lifecycle cycle is required to clear enter_fault(..., false); asking an
    # already-active controller to become active again does not reset it.
    publish_enable false

    state="$(controller_state)"
    if [[ "${state}" == "active" ]]; then
      ros2 control set_controller_state kid_rl inactive || exit $?
    fi
    ros2 control set_controller_state kid_rl active || exit $?
    wait_until_active || exit $?
    echo "[kid_rl_policy] verified controller state: kid_rl active"
    publish_enable true
    ;;
  disable)
    publish_enable false
    ;;
  *)
    echo "usage: $0 {enable|disable}" >&2
    exit 2
    ;;
esac
