# kid_rl

`kid_rl` is a `ros2_control` controller that runs a reinforcement-learning walking policy, trained in simulation with [mjlab](https://github.com/mujocolab/mjlab), on a real kid-size humanoid.
The policy is an ONNX model executed with the ONNX Runtime C++ API inside the controller's real-time `update()`.

The servos are driven through a modified fork of the ROBOTIS hardware interface:
**[parkgeon1202/dynamixel_hardware_interface](https://github.com/parkgeon1202/dynamixel_hardware_interface)** (branch `kid-rl`).
That repository lists every change made to the driver. This page explains how kid_rl uses them.

---

## Contents

1. [Branches](#1-branches)
2. [The robot and the control loop](#2-the-robot-and-the-control-loop)
3. [Installation](#3-installation)
4. [Running](#4-running)
5. [Operating modes](#5-operating-modes)
6. [Inputs and ROS interfaces](#6-inputs-and-ros-interfaces)
7. [Policy contract](#7-policy-contract)
8. [Hardware configuration](#8-hardware-configuration)
9. [How kid_rl uses the hardware interface fork](#9-how-kid_rl-uses-the-hardware-interface-fork)
10. [CSV log](#10-csv-log)
11. [Further documentation](#11-further-documentation)

---

## 1. Branches

| Branch | Robot | Policy (`models/policy.onnx`) |
|---|---|---|
| `main` | Dance robot, 25 policy joints (whole body), gamepad | Walking policy deployed on 2026-10-14 |
| `irc` | IRC robot, 12 policy joints (legs only), gamepad | Flat-ground walking policy for the IRC robot |
| `irc_hurdle` | IRC robot. Walking commands, motions and manipulation come from other ROS nodes instead of a gamepad | Hurdle policy that also observes the hurdle position from vision |

This README describes `irc`: the **IRC robot**, whose policy drives only the 12 leg joints. The code is the same as `main`; only `config/controllers.yaml`, the hardware xacro and the policy differ. Each branch carries the `policy.onnx` that was deployed with it.

---

## 2. The robot and the control loop

| Item | Value |
|---|---|
| Servos | 12 DYNAMIXEL leg servos (MX-64 / MX-106 (Protocol 2.0), XH540), IDs 10–21, on one U2D2 bus at 1 Mbps |
| Policy joints | All 12 leg joints. The IRC robot's upper body has no joints in the training model, and its servos are not part of this ros2_control system |
| IMU | EBIMU on a serial port (`/dev/ttyUSB-EBIMU`), read directly by the controller in its own thread |
| Command input | USB gamepad, read directly from `/dev/input/event*` by the controller |
| Control mode | Position control (Operating Mode 3), P gain only (I = D = 0, no profile). This matches the actuator model used in training |
| Rates | `controller_manager` 50 Hz, policy 50 Hz (one inference per tick). Training also ran the policy at 50 Hz (0.005 s physics step × decimation 4) |

The policy target for each joint is `default_position + action_scale × action` (`action_scale = 0.5`, checked against the model metadata at startup).

A walking policy depends much more on timing than a manipulator does:

- **Stale or missing joint readings change the policy's observation.**
- **A skipped write means the servos keep the previous target for another 20 ms or more.**
- **A spurious fault, such as an input-voltage alert from battery sag, drops torque in the middle of a step.**

The hardware interface fork exists to fix these three problems. It also exports diagnostics that kid_rl writes to its CSV log, so problems on the real robot can be traced to a specific servo or tick (§9).

---

## 3. Installation

Tested on Ubuntu 24.04 with ROS 2 Jazzy.

```bash
cd ~/${WORKSPACE}/src
git clone -b jazzy  https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy  https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone -b kid-rl https://github.com/parkgeon1202/dynamixel_hardware_interface.git
git clone -b irc https://github.com/parkgeon1202/kid_rl.git
cd ~/${WORKSPACE}
colcon build --packages-up-to kid_rl \
  --cmake-args -DONNXRUNTIME_ROOT=$HOME/opt/onnxruntime/current
source install/setup.bash
```

`ONNXRUNTIME_ROOT` must contain `include/onnxruntime_cxx_api.h` and `lib/libonnxruntime.so` from the ONNX Runtime C++ release. It can also be set as an environment variable.

---

## 4. Running

```bash
ros2 launch kid_rl kid_rl.launch.py            # port:=/dev/ttyUSB-U2D2 baud:=1000000 autostart:=false
```

The launch file starts `robot_state_publisher`, `ros2_control_node` (with `DynamixelHardware` and `KidRlController`) and `joint_state_broadcaster`.

1. **Hardware active, controller inactive.** The hardware interface turns torque on and holds the current pose. The robot must already be supported.
2. **Activate the controller.** The robot moves from its current pose to the default pose in 2 s (READY_POSE), then holds it (SAFE_HOLD).
   ```bash
   ros2 control set_controller_state kid_rl active
   ```
3. **Enable the policy.**
   ```bash
   ros2 topic pub --once /kid_rl/enable_policy std_msgs/msg/Bool "{data: true}"
   ```
   The policy starts only when the IMU is fresh, the robot is upright, and every joint is within `entry_position_tolerance` (0.35 rad) of the default pose.
4. **Walk** with the gamepad sticks. The stick position maps directly to the command, up to `max_vx 0.4`, `max_vy 0.3`, `max_wz 0.7`, with a 0.1 deadband.

`scripts/kid_rl_policy.sh enable|disable` wraps steps 2–3 (and cycles the controller to clear a latched fault).
`scripts/gamepad_policy_supervisor.py` runs it from the gamepad buttons: **A** enables, **B** disables.

`autostart:=true` activates the controller at launch. Leave it off unless the robot is supported and someone is watching.

---

## 5. Operating modes

```
WAITING_FOR_STATE → READY_POSE → SAFE_HOLD ⇄ POLICY_RUNNING
                                     ↑              │
                                     └─── FAULT ◄───┘
                           (latched: deactivate and reactivate the controller)
```

| Mode | Joint targets | Notes |
|---|---|---|
| `WAITING_FOR_STATE` | — | Waiting for valid joint states after activation |
| `READY_POSE` | Smoothstep from the measured pose to the default pose | `ready_pose_duration_s`, rate-limited by `target_rate_limit` |
| `SAFE_HOLD` | Default pose | Policy disabled or not yet allowed to start |
| `POLICY_RUNNING` | Policy output | Joint limits from `joint_position_min/max` are applied |
| `FAULT` | Frozen | Entered on IMU timeout, fall (projected gravity z > −0.5 for 0.1 s), a non-finite action, or a nonzero servo `hardware_state`. Latched (`fault_recovery_s: -1`) |

A stale gamepad command is not a fault. It is treated as zero velocity, so the robot keeps balancing in place.

---

## 6. Inputs and ROS interfaces

| Kind | Name | Type |
|---|---|---|
| Device | `/dev/input/event*` | Gamepad (`vx`, `vy`, `wz`), found by VID/PID or `gamepad.name_hint` |
| Device | `/dev/ttyUSB-EBIMU` | EBIMU `quaternion_gyro` frames. Axis maps are in `controllers.yaml` |
| Subscription | `/kid_rl/enable_policy` | `std_msgs/Bool` |
| Publication | `/joint_states` | `sensor_msgs/JointState` (joint_state_broadcaster) |
| ros2_control | `<joint>/position`, `velocity`, `hardware_state` state; `<joint>/position` command | 12 joints |

---

## 7. Policy contract

`PolicyRunner::load()` checks the model metadata at configure time and refuses to start on any mismatch:

- input and output tensor shapes (`[1,45]` → `[1,12]`), float type,
- `joint_names` (order matters), `default_joint_pos`, `action_scale`,
- `observation_names`, which must match what the controller assembles.

| Term | Size | Notes |
|---|---|---|
| `base_ang_vel` | 3 | IMU gyro, low-pass filtered at `imu_gyro_filter_hz` (6 Hz) |
| `projected_gravity` | 3 | From the IMU quaternion |
| `joint_pos` | 12 | Relative to the default pose |
| `joint_vel` | 12 | |
| `actions` | 12 | Last policy output |
| `command` | 3 | vx, vy, wz |
| `gait_phase` | 2 | Only if the model lists it: `sin`, `cos` of the gait clock |

---

## 8. Hardware configuration

`description/kid_dynamixel.ros2_control.xacro` defines the hardware. Below is a shortened version with one joint. Parameters and interfaces that exist only in the hardware interface fork are marked `FORK`.

```xml
<ros2_control name="KidDynamixel" type="system" rw_rate="50">
  <hardware>
    <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
    <param name="port_name">/dev/ttyUSB-U2D2</param>
    <param name="baud_rate">1000000</param>
    <param name="number_of_joints">12</param>
    <param name="number_of_transmissions">12</param>
    <param name="error_timeout_ms">400</param>
    <param name="disable_torque_at_init">true</param>
    <param name="use_fast_read_protocol">false</param>   <!-- FORK (§9.2) -->
    <param name="track_sync_read_rx">true</param>        <!-- FORK (§9.3) -->
    <param name="measure_write_drain">false</param>      <!-- FORK (§9.4) -->
    <param name="read_timeout_ms">14</param>             <!-- FORK (§9.5) -->
    <param name="write_on_read_failure">true</param>     <!-- FORK (§9.6) -->
  </hardware>

  <joint name="left_knee_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="hardware_state"/>
    <state_interface name="rx_missing_id"/>              <!-- FORK (§9.3) -->
  </joint>

  <gpio name="left_knee_pitch_dxl">
    <param name="type">dxl</param>
    <param name="ID">16</param>
    <command_interface name="Goal Position"/>
    <state_interface name="Present Position"/>
    <state_interface name="Present Velocity"/>
    <state_interface name="Hardware Error Status"/>
    <state_interface name="Present Temperature"/>
    <state_interface name="Present Input Voltage"/>
    <state_interface name="Present PWM"/>
    <param name="Position P Gain">149</param>
    <param name="Position I Gain">0</param>
    <param name="Position D Gain">0</param>
    <param name="Profile Velocity">0</param>
    <param name="Profile Acceleration">0</param>
    <param name="Operating Mode">3</param>
    <param name="Return Delay Time">10</param>
  </gpio>
  <!-- transmission omitted: SimpleTransmission, reduction 1.0 -->

  <!-- FORK (§9.4): a virtual gpio that exports the driver's own bus timing -->
  <gpio name="dxl_bus">
    <param name="type">bus_timing</param>
    <param name="ID">255</param>                         <!-- placeholder, never used -->
    <state_interface name="read_ms"/>
    <state_interface name="read_ok"/>
    <state_interface name="read_ok_start_s"/>
    <state_interface name="read_ok_end_s"/>
    <state_interface name="write_sent"/>
    <state_interface name="write_ms"/>
    <state_interface name="write_drain_ms"/>
  </gpio>
</ros2_control>
```

kid_rl checks `robot_description` when it is configured. The optional diagnostics (`rx_missing_id`, `dxl_bus`, `Present Input Voltage` / `Present PWM`) turn on kid_rl's matching CSV logging only if **every** joint declares the interface. If none or only some declare it, kid_rl leaves that logging off and does not request the interfaces. So you can remove any of them from the xacro, and the controller still runs. It just logs less.

---

## 9. How kid_rl uses the hardware interface fork

Each item matches the numbered change in the [fork's README](https://github.com/parkgeon1202/dynamixel_hardware_interface#2-changes-from-upstream).

### 9.1 Input Voltage Error is diagnostic-only

- When the legs push off, battery voltage dips for a moment and several servos raise Hardware Error bit 0. Upstream treats it as a fault, which dropped torque or rebooted servos mid-step. The fork logs it and ignores it when bit 0 is the only bit set.
- The controller's `hardware_state_ok()` check reads the joints' `hardware_state` (the driver writes the same bus-wide error state into every joint) and enters FAULT on any nonzero value. Because bit 0 is masked in the driver, a voltage dip no longer stops the policy. Real faults (overheating, overload, ...) still do.
- Supply voltage and PWM are still recorded through `Present Input Voltage` / `Present PWM` as the `vin_<joint>` / `duty_<joint>` CSV columns, so dips stay visible in the log.

### 9.2 Fast Sync Read turned off (`use_fast_read_protocol: false`)

- With Fast Sync Read, all 12 servos answer in one combined packet, so a single late or corrupted reply loses the whole tick's readings.
- kid_rl uses plain Sync Read. Each servo answers in its own packet, so a bad reply costs at most that servo's reading. It is also needed for the per-servo tracking in §9.3.

### 9.3 Per-servo reception tracking (`rx_missing_id`)

- Each joint exports `rx_missing_id`: the servo's ID if its reply was missing this tick, otherwise −1.
- `RxMissingMonitor` (`src/rx_missing_monitor.cpp`) reads it for every joint each tick and writes the `rx_missing_count` and `rx_missing_ids` CSV columns.
- One missing ID means one lost packet. A run of missing IDs up to the end of the bus points to a break in the daisy chain at that servo. These columns can be lined up with falls, voltage dips or specific motions.

### 9.4 Bus timing (`dxl_bus` gpio)

- The policy was trained with a known observation delay. To compare the robot with simulation, kid_rl needs to know how old each joint reading is when the policy uses it.
- `BusTimingMonitor` (`src/bus_timing_monitor.cpp`) reads the `dxl_bus` values every tick and writes:
  - `read_ms`, `read_ok`: duration and result of this tick's Sync Read.
  - `joint_age_min_ms = now − read_ok_end_s` and `joint_age_max_ms = now − read_ok_start_s`: the lower and upper bound on the age of the joint readings in the observation. After a failed read they keep growing, as they should.
  - `prev_write_sent`, `prev_write_ms`, `prev_write_drain_ms`: the previous tick's Sync Write (`write()` runs after `update()`).
- The driver's times use `steady_clock`, so the controller subtracts them directly from its own clock.

### 9.5 Read timeout (`read_timeout_ms: 14`)

- Upstream waits up to the whole 20 ms period for replies. A single missing reply then made the tick overrun, `ros2_control` skipped the next tick, and the policy lost one action.
- 14 ms was chosen from the `read_ms` column over 6,452 successful reads: median 8.4 ms, p99.9 11.1 ms, max 12.25 ms.
- Worst case, a failed read (14 ms) plus `update()` and `write()` (about 4 ms at most) is about 18 ms, inside the 20 ms tick.
- If you change the baud rate, the number of servos or the items read, check `read_ms` again.

### 9.6 Write even when the read failed (`write_on_read_failure: true`)

- Upstream skipped the write after a read communication error, so a new policy target did not reach the servos for about 60 ms.
- With the fork, the policy's target goes out every tick. On a failed-read tick the action is computed from the previous tick's joint readings. The policy was trained with a 0–1 step observation delay, which covers this case.

### 9.7 Sync Write only when commands changed, transmit failures detected

- While the robot holds a pose (SAFE_HOLD with unchanged targets), no Sync Write is sent, leaving more bus time for reads. `prev_write_sent` shows 0 on those ticks.
- A transmit failure that lasts longer than `error_timeout_ms` now makes `write()` return ERROR (§9.8), instead of commands silently stopping.

### 9.8 Torque off on `on_error` and `on_shutdown`

- Upstream never calls `on_deactivate()` on the ERROR transition, so the servos kept torque and their last goal position, freezing a mid-step pose.
- With the fork, if the bus fails for longer than `error_timeout_ms` (400 ms) or the controller manager shuts down, the robot goes limp instead of locking up. The hardware can be brought back with `set_hardware_component_state` without restarting the process.

### 9.9 Gpio command initial value

Not used on this branch. The P gains are written once at startup from the xacro `<param>` values and never changed. Runtime P gain switching through a `Position P Gain` command interface was added later, on `irc_hurdle`.

---

## 10. CSV log

While the controller is active, it writes `~/kid_rl_logs/kid_rl_YYYYmmdd_HHMMSS.csv` (`log.path` to override). The control loop only copies into a preallocated ring buffer; a separate thread writes the file.

| Columns | Content |
|---|---|
| `t_s`, `dt_s`, `mode`, `policy_t_s` | Time, tick period, operating mode, time since the policy started |
| `imu_valid`, `imu_age_ms`, `gravity_*`, `gyro_*`, `gyro_raw_*` | IMU; `gyro_*` is the filtered value the policy sees |
| `command_fresh`, `cmd_vx`, `cmd_vy`, `cmd_wz`, `gait_phase` | Gamepad command |
| `rx_missing_count`, `rx_missing_ids` | §9.3 |
| `read_ms`, `read_ok`, `joint_age_min_ms`, `joint_age_max_ms`, `prev_write_*` | §9.4 |
| `pos_`, `vel_`, `target_`, `err_`, `action_`, `hwerr_<joint>` | Per joint |
| `vin_`, `duty_<joint>` | Supply voltage and PWM (§9.1) |

`scripts/compare_sim_real.py` and `scripts/mjlab_sim_csv.py` compare these logs with simulation rollouts.

---

## 11. Further documentation

The parameters in `config/controllers.yaml` and `description/kid_dynamixel.ros2_control.xacro` are commented with the reason and the measurement behind each value.

The documents in `docs/` are written in Korean and partly describe earlier versions:

| Document | Content |
|---|---|
| [`docs/CONTROLLER_GUIDE.md`](docs/CONTROLLER_GUIDE.md) | Code walkthrough, policy contract, timing, safety design, parameters, bring-up, debugging |
| [`docs/THREADING.md`](docs/THREADING.md) | Threads, real-time settings, ONNX threads, DYNAMIXEL bus budget |
| [`docs/STUDY.md`](docs/STUDY.md) | Background and comparison with other deployment stacks |
| [`docs/SIM2REAL_OSCILLATION_2026-09-28.md`](docs/SIM2REAL_OSCILLATION_2026-09-28.md) | 10–15 Hz oscillation on the real robot and the gyro filter |
| [`docs/DEPLOYMENT_AUDIT_2026-09-18.md`](docs/DEPLOYMENT_AUDIT_2026-09-18.md) | Pre-deployment audit |
| [`docs/WORKLOG.md`](docs/WORKLOG.md) | Work history |
