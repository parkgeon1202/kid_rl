# kid_rl

`kid_rl` is a `ros2_control` controller that runs a reinforcement-learning walking policy, trained in simulation with [mjlab](https://github.com/mujocolab/mjlab), on a real kid-size humanoid.
The policy is an ONNX model executed with the ONNX Runtime C++ API inside the controller's real-time `update()`.

The servos are driven through a modified fork of the ROBOTIS hardware interface:
**[parkgeon1202/dynamixel_hardware_interface](https://github.com/parkgeon1202/dynamixel_hardware_interface)** (branch `kid-rl`).
That repository lists every change made to the driver. This page explains how kid_rl uses them.

---

## Contents

1. [Branches](#1-branches)
2. [Packages used together](#2-packages-used-together)
3. [What this branch adds](#3-what-this-branch-adds)
4. [The robot and the control loop](#4-the-robot-and-the-control-loop)
5. [Installation](#5-installation)
6. [Running](#6-running)
7. [Operating modes](#7-operating-modes)
8. [ROS interfaces](#8-ros-interfaces)
9. [Policy contract](#9-policy-contract)
10. [Hardware configuration](#10-hardware-configuration)
11. [How kid_rl uses the hardware interface fork](#11-how-kid_rl-uses-the-hardware-interface-fork)
12. [CSV log](#12-csv-log)
13. [Further documentation](#13-further-documentation)

---

## 1. Branches

| Branch | Robot | Policy (`models/policy.onnx`) |
|---|---|---|
| `main` | Dance robot, 25 policy joints (whole body), gamepad | Walking policy deployed on 2026-10-14 |
| `irc` | IRC robot, 12 policy joints (legs only), gamepad | Flat-ground walking policy for the IRC robot |
| `for_irc_contest` | IRC robot, 12 policy joints (legs only), upper body held. Walking commands, motions and manipulation come from other ROS nodes instead of a gamepad | Flat-ground policy trained to lift each foot at least 2 cm for at least 0.3 s (run 2026-11-02_02-17-02) |
| `irc_hurdle` | IRC robot, as `for_irc_contest` | Hurdle policy that also observes the hurdle position from vision (§9) |

This README describes `irc_hurdle`. Each branch carries the `policy.onnx` that was deployed with it.

---

## 2. Packages used together

On the competition robot, kid_rl is the only node that talks to the servos. The other packages command the robot through ROS topics:

| Package | Role | Topics with kid_rl |
|---|---|---|
| [dynamixel_hardware_interface (fork)](https://github.com/parkgeon1202/dynamixel_hardware_interface/tree/kid-rl) | `ros2_control` hardware plugin for the 21 servos, loaded in the same process as kid_rl | ros2_control interfaces (§10, §11) |
| [irc_task_planner](https://github.com/ROBIT-KOR-teamHumanoid/irc_task_planner) | Competition state machine: decides where to walk, when to stop, and when to start a motion or manipulation | `/master2ik` → kid_rl (walking command)<br>`/walk_stopped` ← kid_rl (legs released for motions) |
| [motion_operator](https://github.com/ROBIT-KOR-teamHumanoid/motion_operator) | Scripted whole-body motions (keyframes) | `/dynamixel_control` → kid_rl<br>`/motor_status` ← kid_rl |
| [humanoid_manipulation](https://github.com/parkgeon1202/humanoid_manipulation) | Ball pick-up and shooting with the arms and gripper (FSM, IK) | `/dynamixel_control`, `/gripper_control` → kid_rl<br>`/motor_status` ← kid_rl |
| [irc_vision](https://github.com/ROBIT-KOR-teamHumanoid/irc_vision) | RealSense ball / hurdle detection. Its `pan_tilt` node aims the camera; its refiner publishes the hurdle position | `/RealsensePanTilt` → kid_rl (camera tilt, ID 23)<br>`/hurdle/pose` → kid_rl (hurdle position, §9) |
| [dynamixel_interfaces (fork)](https://github.com/parkgeon1202/dynamixel_interfaces/tree/kid-rl) | ROBOTIS messages plus the five used on the topics above (see the note in §5) | — |
| EBIMU | IMU, read by kid_rl directly from the serial port | — |

Before `for_irc_contest`, those packages talked to a separate team DYNAMIXEL node. kid_rl now takes that node's place, so the walking policy, motions and manipulation share one bus and one driver: it accepts the same topics (with a servo `id` added to every element) and publishes the same motor status, so motion_operator and humanoid_manipulation work without a separate driver.

---

## 3. What this branch adds

Compared with `for_irc_contest`, `irc_hurdle` adds hurdle vision to the policy. Everything in `for_irc_contest` (listed in the [for_irc_contest README](https://github.com/parkgeon1202/kid_rl/tree/for_irc_contest#3-what-this-branch-adds)) is included.

| Area | What was added or changed | Code |
|---|---|---|
| Hurdle pose input | Subscribes to `/hurdle/pose` (`geometry_msgs/PoseStamped`, `base_link`, m). The callback stores the position, the receive time (steady clock) and the interval since the previous message, and hands them to the control thread through a `RealtimeBuffer` | `external_commands.cpp/.hpp`, `external.hurdle_pose_topic` |
| `hurdle_vision` observation | A fixed-size ring buffer of `(visible, x, y, z)` frames, one per policy step. A pose counts only on the first step after it arrives. 11 frames 0.1 s apart (44 values) are appended to the observation, in the same order as `tasks/hurdle.py` in training. Cleared when POLICY_RUNNING starts, like an episode reset | `hurdle_vision.hpp` (new), `kid_rl_controller.cpp` |
| Frame stacking | `base_ang_vel`, `projected_gravity`, `joint_pos`, `joint_vel` and `actions` are stacked over `history_length` policy steps, term by term (mjlab layout). After a reset all slots are filled with the first frame | `kid_rl_controller.cpp` |
| Model checks | `PolicyRunner::load()` recognises `hurdle_vision` at the end of `observation_names` and derives `history_length` from the input width (the metadata has only names). A hurdle model is refused unless `policy_rate_hz` is 50 and the hurdle topic is set | `policy_runner.cpp/.hpp` |
| CSV | `hurdle_fresh`, `hurdle_age_ms`, `hurdle_period_ms`, `hurdle_x/y/z` | `csv_logger.cpp/.hpp` |
| Tests | `hurdle_vision_test` checks the 44 values against the training order and spacing; `policy_runner_test` updated for the new `load()` arguments | `test/` |
| Policy | Hurdle policy (run 2026-11-09_17-11-49) | `models/policy.onnx` |

The camera tilt is not part of the observation. The task planner changes it separately.

---

## 4. The robot and the control loop

| Item | Value |
|---|---|
| Servos | 21 DYNAMIXEL servos (MX-28 / MX-64 / MX-106 (Protocol 2.0), XH540) on one U2D2 bus at 1 Mbps |
| Policy joints | 12 leg joints (IDs 10–21), driven by the ONNX policy |
| Held joints | 9 upper-body joints: shoulders, elbows, gripper (IDs 0–6), torso yaw (22), camera tilt (23). Held at a fixed pose, or driven by external motion / manipulation commands |
| IMU | EBIMU on a serial port (`/dev/ttyUSB-EBIMU`), read directly by the controller in its own thread |
| Control mode | Position control (Operating Mode 3), P gain only (I = D = 0, no profile). This matches the actuator model used in training |
| Rates | `controller_manager` 50 Hz, policy 50 Hz (one inference per tick). Training also ran the policy at 50 Hz (0.005 s physics step × decimation 4) |

The policy target for each joint is `default_position + action_scale × action` (`action_scale = 0.5`, checked against the model metadata at startup).

A walking policy depends much more on timing than a manipulator does:

- **Stale or missing joint readings change the policy's observation.**
- **A skipped write means the servos keep the previous target for another 20 ms or more.**
- **A spurious fault, such as an input-voltage alert from battery sag, drops torque in the middle of a step.**

The hardware interface fork exists to fix these three problems. It also exports diagnostics that kid_rl writes to its CSV log, so problems on the real robot can be traced to a specific servo or tick (§11).

---

## 5. Installation

Tested on Ubuntu 24.04 with ROS 2 Jazzy.

```bash
cd ~/${WORKSPACE}/src
git clone -b jazzy  https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b kid-rl https://github.com/parkgeon1202/dynamixel_interfaces.git
git clone -b kid-rl https://github.com/parkgeon1202/dynamixel_hardware_interface.git
git clone -b irc_hurdle https://github.com/parkgeon1202/kid_rl.git
cd ~/${WORKSPACE}
colcon build --packages-up-to kid_rl \
  --cmake-args -DONNXRUNTIME_ROOT=$HOME/opt/onnxruntime/current
source install/setup.bash
```

> **Note on `dynamixel_interfaces`.** The external command topics use five messages that are **not** in the ROBOTIS repository: `DynamixelMsgs`, `DynamixelControlMsgs`, `DynamixelStateMsgs`, `DynamixelMotorState` and `PanTilt` (each motor element carries an `id`). They are in the [dynamixel_interfaces fork](https://github.com/parkgeon1202/dynamixel_interfaces/tree/kid-rl) cloned above. Build motion_operator and humanoid_manipulation against the same package.

`ONNXRUNTIME_ROOT` must contain `include/onnxruntime_cxx_api.h` and `lib/libonnxruntime.so` from the ONNX Runtime C++ release. It can also be set as an environment variable.

---

## 6. Running

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
   The policy starts only when the IMU is fresh, the robot is upright, every leg joint is within `entry_position_tolerance` (0.35 rad) of the default pose, and the trained P gains have been written for at least two ticks.
4. **Walk.** Publish `geometry_msgs/TwistStamped` on `/master2ik`. The command is clipped to `max_vx 0.4`, `max_vy 0.3`, `max_wz 0.7`.

`scripts/kid_rl_policy.sh enable|disable` wraps steps 2–3 (and cycles the controller to clear a latched fault).

`autostart:=true` activates the controller at launch. Leave it off unless the robot is supported and someone is watching.

---

## 7. Operating modes

```
WAITING_FOR_STATE → READY_POSE → SAFE_HOLD ⇄ POLICY_RUNNING
                                     ↑              │
                                     └─── FAULT ◄───┘
                           (latched: deactivate and reactivate the controller)
```

| Mode | Leg targets | Notes |
|---|---|---|
| `WAITING_FOR_STATE` | — | Waiting for valid joint states after activation |
| `READY_POSE` | Smoothstep from the measured pose to the default pose | `ready_pose_duration_s`, rate-limited by `target_rate_limit` |
| `SAFE_HOLD` | Default pose, or external leg commands from `/dynamixel_control` when the policy is disabled | P gains switch to `motion_gain.p_gains` (stiffer) for scripted motions and manipulation |
| `POLICY_RUNNING` | Policy output | Trained P gains. Joint limits from `joint_position_min/max` are applied |
| `FAULT` | Frozen | Entered on IMU timeout, fall (projected gravity z > −0.5 for 0.1 s), a non-finite action, or a nonzero servo `hardware_state`. Latched (`fault_recovery_s: -1`) |

**Automatic hold.** While `/master2ik` sends (0, 0, 0), the policy keeps balancing. Once the filtered gyro stays below 0.1 rad/s for 0.5 s, the controller disables the policy itself and goes to SAFE_HOLD, so that motions and manipulation can take over the legs. A nonzero command re-enables the policy. A policy that an operator disabled through `~/enable_policy` is not re-enabled by walk commands.

`/walk_stopped` (`std_msgs/Bool`, transient local) is `true` while the legs are in SAFE_HOLD with the policy disabled. The task planner waits for it before starting a motion.

---

## 8. ROS interfaces

| Direction | Topic / interface | Type | Purpose |
|---|---|---|---|
| Sub | `/kid_rl/enable_policy` | `std_msgs/Bool` | Operator request to run or stop the policy |
| Sub | `/master2ik` | `geometry_msgs/TwistStamped` | Walking command (vx, vy, wz). Stale (> 0.25 s), future-stamped and non-finite messages are dropped; a stale command means zero velocity, not a fault |
| Sub | `/dynamixel_control` | `dynamixel_interfaces/DynamixelControlMsgs` | Position / profile commands per servo `id` from motion_operator and manipulation |
| Sub | `/gripper_control` | `dynamixel_interfaces/DynamixelMsgs` | Gripper (ID 6) |
| Sub | `/RealsensePanTilt` | `dynamixel_interfaces/PanTilt` | Camera tilt (ID 23), degrees |
| Sub | `/hurdle/pose` | `geometry_msgs/PoseStamped` | Hurdle position in `base_link` [m] from the vision refiner, published only on frames where a hurdle was found (§9) |
| Pub | `/walk_stopped` | `std_msgs/Bool` | See §7 |
| Pub | `/motor_status` | `dynamixel_interfaces/DynamixelStateMsgs` | Position and velocity of every servo, by `id`, at 50 Hz |
| Pub | `/joint_state_broadcaster/joint_states` | `sensor_msgs/JointState` | Kept off `/joint_states` so it does not conflict with the vision TF tree |
| Device | `/dev/ttyUSB-EBIMU` | serial | IMU, `quaternion_gyro` frames, axis maps in `controllers.yaml` |
| ros2_control | `<joint>/position` command, `position` / `velocity` / `hardware_state` state | | Per joint |
| ros2_control | `<joint>_dxl/Position P Gain` command | | Gain switching (§11.9) |

Commands that name an unknown or duplicate `id`, contain non-finite values, or target the camera tilt through `/dynamixel_control` are dropped as a whole message, with an error log.

---

## 9. Policy contract

`PolicyRunner::load()` checks the model metadata at configure time and refuses to start on any mismatch:

- input and output tensor shapes, float type,
- `joint_names` (order matters), `default_joint_pos`, `action_scale`,
- `observation_names`, which must match what the controller assembles.

The observation is assembled in this order:

| Term | Size | Notes |
|---|---|---|
| `base_ang_vel` | 3 | IMU gyro, low-pass filtered at `imu_gyro_filter_hz` (6 Hz), as in training |
| `projected_gravity` | 3 | From the IMU quaternion |
| `joint_pos` | 12 | Relative to the default pose |
| `joint_vel` | 12 | |
| `actions` | 12 | Last policy output |
| `command` | 3 | vx, vy, wz |
| `gait_phase` | 2 | Only if the model lists it: `sin`, `cos` of the gait clock |
| `hurdle_vision` | 44 | Only if the model lists it (hurdle models) |

**Frame stacking.** Hurdle models were trained with `history_length` on the first five terms. The metadata lists only names, so the controller derives the history length from the model's input width (5 for the hurdle model, 1 for flat models). Frames are stacked once per policy step, term by term from oldest to newest, the same layout as mjlab. After a reset every slot is filled with the first frame, as mjlab's `CircularBuffer` does.

**`hurdle_vision`.** One frame is `(visible, x, y, z)`. On each policy step, if a new `/hurdle/pose` arrived since the previous step, the frame is `[1, x, y, z]`; otherwise it is `[0, 0, 0, 0]`. A pose is used on one step only, never held over time. Eleven frames, 0.1 s apart (every 5th step), from oldest to newest, give 44 values, the same as `tasks/hurdle.py` in training. Entering POLICY_RUNNING clears the history, like an episode reset. A hurdle model is refused unless `policy_rate_hz` is 50 and `external.hurdle_pose_topic` is set.

---

## 10. Hardware configuration

`description/kid_dynamixel.ros2_control.xacro` defines the hardware. Below is a shortened version with one joint. Parameters and interfaces that exist only in the hardware interface fork are marked `FORK`.

```xml
<ros2_control name="KidDynamixel" type="system" rw_rate="50">
  <hardware>
    <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
    <param name="port_name">/dev/ttyUSB-U2D2</param>
    <param name="baud_rate">1000000</param>
    <param name="number_of_joints">21</param>
    <param name="number_of_transmissions">21</param>
    <param name="error_timeout_ms">400</param>
    <param name="disable_torque_at_init">true</param>
    <param name="use_fast_read_protocol">false</param>   <!-- FORK (§11.2) -->
    <param name="track_sync_read_rx">true</param>        <!-- FORK (§11.3) -->
    <param name="measure_write_drain">false</param>      <!-- FORK (§11.4) -->
    <param name="read_timeout_ms">14</param>             <!-- FORK (§11.5) -->
    <param name="write_on_read_failure">true</param>     <!-- FORK (§11.6) -->
  </hardware>

  <joint name="left_knee_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="hardware_state"/>
    <state_interface name="rx_missing_id"/>              <!-- FORK (§11.3) -->
  </joint>

  <gpio name="left_knee_pitch_dxl">
    <param name="type">dxl</param>
    <param name="ID">16</param>
    <command_interface name="Goal Position"/>
    <command_interface name="Position P Gain"/>          <!-- switched at runtime by kid_rl (§11.9) -->
    <state_interface name="Present Position"/>
    <state_interface name="Present Velocity"/>
    <state_interface name="Hardware Error Status"/>
    <state_interface name="Present Temperature"/>
    <state_interface name="Present Input Voltage"/>
    <state_interface name="Present PWM"/>
    <param name="Position P Gain">149</param>            <!-- also the initial command value (§11.9) -->
    <param name="Position I Gain">0</param>
    <param name="Position D Gain">0</param>
    <param name="Profile Velocity">0</param>
    <param name="Profile Acceleration">0</param>
    <param name="Operating Mode">3</param>
    <param name="Return Delay Time">10</param>
  </gpio>
  <!-- transmission omitted: SimpleTransmission, reduction 1.0 -->

  <!-- FORK (§11.4): a virtual gpio that exports the driver's own bus timing -->
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

kid_rl checks `robot_description` when it is configured:

- **Optional diagnostics** (`rx_missing_id`, `dxl_bus`, `Present Input Voltage` / `Present PWM`): kid_rl turns its matching CSV logging on only if **every** joint declares the interface. If none or only some declare it, kid_rl leaves that logging off and does not request the interfaces.
- **`Position P Gain` command:** declaring it on only some joints is a configuration error. Declare it on all joints or on none (§11.9).

So you can remove any of these from the xacro, and the controller still runs. It just logs less.

The gripper's Goal PWM (grip force limit) is read from `dynamixel_hardware.gripper_goal_pwm` in `config/controllers.yaml` and passed to the xacro by the launch file.

---

## 11. How kid_rl uses the hardware interface fork

Each item matches the numbered change in the [fork's README](https://github.com/parkgeon1202/dynamixel_hardware_interface#2-changes-from-upstream).

### 11.1 Input Voltage Error is diagnostic-only

- When the legs push off, battery voltage dips for a moment and several servos raise Hardware Error bit 0. Upstream treats it as a fault, which dropped torque or rebooted servos mid-step. The fork logs it and ignores it when bit 0 is the only bit set.
- The controller's `hardware_state_ok()` check reads the joints' `hardware_state` (the driver writes the same bus-wide error state into every joint) and enters FAULT on any nonzero value. Because bit 0 is masked in the driver, a voltage dip no longer stops the policy. Real faults (overheating, overload, ...) still do.
- Supply voltage and PWM are still recorded through `Present Input Voltage` / `Present PWM` as the `vin_<joint>` / `duty_<joint>` CSV columns, so dips stay visible in the log.

### 11.2 Fast Sync Read turned off (`use_fast_read_protocol: false`)

- With Fast Sync Read, all 21 servos answer in one combined packet, so a single late or corrupted reply loses the whole tick's readings.
- kid_rl uses plain Sync Read. Each servo answers in its own packet, so a bad reply costs at most that servo's reading. It is also needed for the per-servo tracking in §11.3.

### 11.3 Per-servo reception tracking (`rx_missing_id`)

- Each joint exports `rx_missing_id`: the servo's ID if its reply was missing this tick, otherwise −1.
- `RxMissingMonitor` (`src/rx_missing_monitor.cpp`) reads it for every joint each tick and writes the `rx_missing_count` and `rx_missing_ids` CSV columns.
- One missing ID means one lost packet. A run of missing IDs up to the end of the bus points to a break in the daisy chain at that servo. These columns can be lined up with falls, voltage dips or specific motions.

### 11.4 Bus timing (`dxl_bus` gpio)

- The policy was trained with a known observation delay. To compare the robot with simulation, kid_rl needs to know how old each joint reading is when the policy uses it.
- `BusTimingMonitor` (`src/bus_timing_monitor.cpp`) reads the `dxl_bus` values every tick and writes:
  - `read_ms`, `read_ok`: duration and result of this tick's Sync Read.
  - `joint_age_min_ms = now − read_ok_end_s` and `joint_age_max_ms = now − read_ok_start_s`: the lower and upper bound on the age of the joint readings in the observation. After a failed read they keep growing, as they should.
  - `prev_write_sent`, `prev_write_ms`, `prev_write_drain_ms`: the previous tick's Sync Write (`write()` runs after `update()`).
- The driver's times use `steady_clock`, so the controller subtracts them directly from its own clock.

### 11.5 Read timeout (`read_timeout_ms: 14`)

- Upstream waits up to the whole 20 ms period for replies. A single missing reply then made the tick overrun, `ros2_control` skipped the next tick, and the policy lost one action.
- 14 ms was chosen from the `read_ms` column over 6,452 successful reads: median 8.4 ms, p99.9 11.1 ms, max 12.25 ms.
- Worst case, a failed read (14 ms) plus `update()` and `write()` (about 4 ms at most) is about 18 ms, inside the 20 ms tick.
- If you change the baud rate, the number of servos or the items read, check `read_ms` again.

### 11.6 Write even when the read failed (`write_on_read_failure: true`)

- Upstream skipped the write after a read communication error, so a new policy target did not reach the servos for about 60 ms.
- With the fork, the policy's target goes out every tick. On a failed-read tick the action is computed from the previous tick's joint readings. The policy was trained with a 0–1 step observation delay, which covers this case.

### 11.7 Sync Write only when commands changed, transmit failures detected

- While the robot holds a pose (SAFE_HOLD with unchanged targets), no Sync Write is sent, leaving more bus time for reads. `prev_write_sent` shows 0 on those ticks.
- A transmit failure that lasts longer than `error_timeout_ms` now makes `write()` return ERROR (§11.8), instead of commands silently stopping.

### 11.8 Torque off on `on_error` and `on_shutdown`

- Upstream never calls `on_deactivate()` on the ERROR transition, so the servos kept torque and their last goal position, freezing a mid-step pose.
- With the fork, if the bus fails for longer than `error_timeout_ms` (400 ms) or the controller manager shuts down, the robot goes limp instead of locking up. The hardware can be brought back with `set_hardware_component_state` without restarting the process.

### 11.9 P gain switching (`Position P Gain` command interface)

- Upstream starts every gpio command at 0, so opening `Position P Gain` as a command would write P gain 0 to every servo on activation. The fork starts the command from the matching `<param>` value.
- `configure_gains()` / `write_gain_commands()` in `src/kid_rl_controller.cpp` switch every servo's P gain at runtime:
  - **Walking:** the P gains from the xacro `<param>`, which match the actuator model used in training.
  - **SAFE_HOLD with the policy disabled** (scripted motions, manipulation): the stiffer `motion_gain.p_gains` from `controllers.yaml` (legs 800, arms and gripper 200, torso 850, camera tilt 600).
- The policy starts only after the trained gains have been written for at least two ticks.
- Declare the interface on **all** servos. If the write items differ between servos, the driver falls back from Sync Write to Bulk Write.
- If the xacro has no `Position P Gain` command (for example a mock system), kid_rl turns gain switching off by itself.

---

## 12. CSV log

While the controller is active, it writes `~/kid_rl_logs/kid_rl_YYYYmmdd_HHMMSS.csv` (`log.path` to override). The control loop only copies into a preallocated ring buffer; a separate thread writes the file.

| Columns | Content |
|---|---|
| `t_s`, `dt_s`, `mode`, `policy_t_s` | Time, tick period, operating mode, time since the policy started |
| `imu_valid`, `imu_age_ms`, `gravity_*`, `gyro_*`, `gyro_raw_*` | IMU; `gyro_*` is the filtered value the policy sees |
| `command_fresh`, `cmd_vx`, `cmd_vy`, `cmd_wz`, `gait_phase` | Walking command |
| `hurdle_fresh`, `hurdle_age_ms`, `hurdle_period_ms`, `hurdle_x/y/z` | `/hurdle/pose`: used this step, age, receive interval, position |
| `rx_missing_count`, `rx_missing_ids` | §11.3 |
| `read_ms`, `read_ok`, `joint_age_min_ms`, `joint_age_max_ms`, `prev_write_*` | §11.4 |
| `pos_`, `vel_`, `target_`, `err_`, `action_`, `hwerr_<joint>` | Per joint |
| `vin_`, `duty_<joint>` | Supply voltage and PWM (§11.1) |

`scripts/compare_sim_real.py` and `scripts/mjlab_sim_csv.py` compare these logs with simulation rollouts.

---

## 13. Further documentation

The parameters in `config/controllers.yaml` and `description/kid_dynamixel.ros2_control.xacro` are commented with the reason and the measurement behind each value.

The documents in `docs/` are written in Korean and partly describe earlier versions (25 joints, gamepad input):

| Document | Content |
|---|---|
| [`docs/CONTROLLER_GUIDE.md`](docs/CONTROLLER_GUIDE.md) | Code walkthrough, policy contract, timing, safety design, parameters, bring-up, debugging |
| [`docs/THREADING.md`](docs/THREADING.md) | Threads, real-time settings, ONNX threads, DYNAMIXEL bus budget |
| [`docs/STUDY.md`](docs/STUDY.md) | Background and comparison with other deployment stacks |
| [`docs/SIM2REAL_OSCILLATION_2026-09-28.md`](docs/SIM2REAL_OSCILLATION_2026-09-28.md) | 10–15 Hz oscillation on the real robot and the gyro filter |
| [`docs/DEPLOYMENT_AUDIT_2026-09-18.md`](docs/DEPLOYMENT_AUDIT_2026-09-18.md) | Pre-deployment audit |
| [`docs/WORKLOG.md`](docs/WORKLOG.md) | Work history |
