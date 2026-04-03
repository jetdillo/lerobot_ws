"""
bridge.launch.py
================
Launch the SO-ARM10x ↔ ROS2 bridge node together with robot_state_publisher
so that TF2 transforms are available for RViz/Gazebo without needing a
separate joint_state_publisher node.

Usage examples
--------------
# ROS → Robot (drive real arm from sim) — default mode:
ros2 launch so_arm_ros2_bridge bridge.launch.py

# Override port and calibration:
ros2 launch so_arm_ros2_bridge bridge.launch.py \
    port:=/dev/ttyACM1 \
    calibration_path:=/home/user/.cache/huggingface/lerobot/calibration/my_arm.json

# Use mock interface (no hardware — great for testing):
ros2 launch so_arm_ros2_bridge bridge.launch.py use_mock:=true

# Robot → ROS (mirror physical arm into RViz):
ros2 launch so_arm_ros2_bridge bridge.launch.py mode:=robot_to_ros

# Bidirectional:
ros2 launch so_arm_ros2_bridge bridge.launch.py mode:=bidirectional

RViz / Gazebo notes
-------------------
* robot_state_publisher subscribes to /joint_states and publishes TF2.
* joint_state_publisher_gui (optional) provides interactive sliders —
  configure its source_list to publish on /joint_commands so the bridge
  picks them up without looping through /joint_states.
* MoveIt2 publishes on /joint_trajectory_controller/... — see the
  moveit_bridge.launch.py for that integration.
"""

import os
from pathlib import Path

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory("so_arm_ros2_bridge")

    # ── Launch arguments ─────────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyACM0",
        description="Serial port for Feetech bus servo adapter"
    )
    calibration_arg = DeclareLaunchArgument(
        "calibration_path",
        default_value=str(
            Path.home() / ".cache/huggingface/lerobot/calibration/my_arm.json"
        ),
        description="Path to lerobot calibration JSON"
    )
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="ros_to_robot",
        description="Bridge mode: ros_to_robot | robot_to_ros | bidirectional"
    )
    read_rate_arg = DeclareLaunchArgument(
        "read_rate", default_value="50.0",
        description="Servo polling rate in Hz"
    )
    max_delta_arg = DeclareLaunchArgument(
        "max_delta_rad", default_value="0.05",
        description="Max radians per step velocity guard"
    )
    mock_arg = DeclareLaunchArgument(
        "use_mock", default_value="false",
        description="Use mock interface (no hardware)"
    )
    joint_config_arg = DeclareLaunchArgument(
        "joint_config_path",
        default_value=os.path.join(pkg_share, "config", "joint_config.yaml"),
        description="Path to joint_config.yaml"
    )

    # ── URDF / robot_description ──────────────────────────────────────────────
    # Try to load URDF from so101_description (Pavankv92/lerobot_ws).
    # Falls back to a minimal placeholder if that package isn't built yet.
    try:
        lerobot_desc_path = get_package_share_directory("lerobot_description")
        urdf_path = os.path.join(lerobot_desc_path, "urdf", "so101.urdf.xacro")
        robot_description = xacro.process_file(urdf_path).toxml()

    except Exception:
        robot_description = (
            "<robot name='so101'>"
            "  <link name='base_link'/>"
            "  <!-- Full URDF from lerobot_description package not found. "
            "       Build lerobot_ws and source its install/setup.bash. -->"
            "</robot>"
        )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    bridge_node = Node(
        package="so_arm_ros2_bridge",
        executable="bridge_node",
        name="so_arm_bridge",
        output="screen",
        parameters=[{
            "port":               LaunchConfiguration("port"),
            "calibration_path":   LaunchConfiguration("calibration_path"),
            "mode":               LaunchConfiguration("mode"),
            "read_rate":          LaunchConfiguration("read_rate"),
            "max_delta_rad":      LaunchConfiguration("max_delta_rad"),
            "use_mock":           LaunchConfiguration("use_mock"),
            "joint_config_path":  LaunchConfiguration("joint_config_path"),
        }],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ── Optional: joint_state_publisher_gui for manual joint control ──────────
    # Uncomment the block below to add interactive sliders.
    # Note: source_list redirects GUI output to /joint_commands, not
    # /joint_states, so the bridge picks it up without a feedback loop.
    #
    joint_state_publisher_gui_node = Node(
         package="joint_state_publisher_gui",
         executable="joint_state_publisher_gui",
         name="joint_state_publisher_gui",
         parameters=[{
             "robot_description": robot_description,
         }],
         remappings=[
             ("/joint_states", "/joint_commands"),
         ],
     )

    return LaunchDescription([
        port_arg,
        calibration_arg,
        mode_arg,
        read_rate_arg,
        max_delta_arg,
        mock_arg,
        joint_config_arg,
        LogInfo(msg="Starting SO-ARM10x ROS2 bridge…"),
        robot_state_publisher_node,
        bridge_node,
        joint_state_publisher_gui_node,  # uncomment for GUI sliders
    ])
