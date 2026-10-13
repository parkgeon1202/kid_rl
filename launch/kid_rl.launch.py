# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""Bring up the DYNAMIXEL hardware and the RL locomotion controller.

Node layout::

    robot_state_publisher  --/robot_description-->  ros2_control_node
                                                      +- DynamixelHardware  (plugin)
                                                      +- KidRlController    (plugin)
                                                      +- joint_state_broadcaster
    spawner(joint_state_broadcaster) ---\\
    spawner(kid_rl, --inactive) --------/  service calls into ros2_control_node

`kid_rl` is spawned INACTIVE on purpose. Activating it makes the robot leave its
current pose for the default pose (READY_POSE, a 2 s smoothstep) and then hold
it, so it should happen when someone is watching and the robot is supported::

    ros2 control set_controller_state kid_rl active

The policy still does not run at that point -- POLICY_RUNNING additionally needs
`ros2 topic pub /kid_rl/enable_policy std_msgs/Bool "data: true"` plus a fresh
IMU, an upright robot, and every joint within entry_position_tolerance of the
default pose. Pass `autostart:=true` to activate at launch instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    pkg = FindPackageShare("kid_rl")

    # xacro is expanded at launch time, not build time, so `port:=` and friends
    # can be overridden without rebuilding.
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "description", "kid_rl.urdf.xacro"]),
        " port:=", LaunchConfiguration("port"),
        " baud:=", LaunchConfiguration("baud"),
        " rw_rate:=", LaunchConfiguration("rw_rate"),
    ])

    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    # Publishes /robot_description with transient_local durability, which is how
    # controller_manager receives it (it subscribes; it does not read a file).
    # This package currently has only a control-schema URDF, not the real 33
    # joint MJCF kinematic tree.  Do not let robot_state_publisher turn that
    # placeholder into plausible-looking but false TF transforms.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
        }],
        remappings=[("joint_states", "/kid_control_description/unused_joint_states")],
    )

    # The RT loop lives here: read() -> update() -> write() at update_rate.
    # `emulate_tty` keeps the RCLCPP_* colours; without `output="screen"` the
    # "Could not enable FIFO RT scheduling policy" warning is easy to miss.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        # controller_manager subscribes on its private
        # ``~/robot_description`` topic.  robot_state_publisher publishes the
        # description on ``/robot_description``; without this remap the
        # ResourceManager never receives the URDF and both spawners wait/fail.
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        emulate_tty=True,
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", controllers, *extra],
            output="screen",
        )

    nodes = [
        LogInfo(
            msg="WARNING: kid_rl is using a control-schema-only URDF. TF publication from "
                "/joint_states is disabled until the MJLab 33-joint model is converted."
        ),
        rsp,
        control_node,
        Node(
            package="kid_rl",
            executable="gamepad_policy_supervisor.py",
            name="gamepad_policy_supervisor",
            output="screen",
        ),
        spawner("joint_state_broadcaster"),
    ]

    # --inactive loads and configures the controller without calling its
    # update().  This does NOT keep the hardware inactive: controller_manager
    # activates DynamixelHardware by default, and its start() holds the current
    # position and enables torque.  The robot must already be supported.
    autostart = LaunchConfiguration("autostart").perform(context).lower() in ("true", "1")
    nodes.append(spawner("kid_rl") if autostart else spawner("kid_rl", "--inactive"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB-U2D2",
                              description="DYNAMIXEL serial port"),
        DeclareLaunchArgument("baud", default_value="1000000",
                              description="DYNAMIXEL baud rate. 1 Mbps needs ~7 ms of the "
                                          "20 ms budget at 50 Hz; 3 Mbps was dropped for "
                                          "packet loss, not speed. See the xacro header."),
        DeclareLaunchArgument("rw_rate", default_value="50",
                              description="Hardware read/write rate. Capped at the "
                                          "controller_manager update_rate."),
        DeclareLaunchArgument("autostart", default_value="false",
                              description="Activate kid_rl at launch. Leave false unless "
                                          "the robot is supported and someone is watching."),
        OpaqueFunction(function=_setup),
    ])
