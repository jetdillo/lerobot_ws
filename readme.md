# LeRobot MotorBus API to ROS2 JointStateMsg Bridge


- Uses LeRobot URDF, Rviz & MoveIt! code forked from [Pavankv92](https://github.com/Pavankv92/lerobot_ws/) for testing/demo
- Install/Usage docs below
- See the [Pavankv92](https://github.com/Pavankv92/lerobot_ws/) repo for detailed docs on installation, usage, etc of `lerobot_ws`

## Features

- ✅ ROS 2 Jazzy, Humble compatibility
- ✅ Provides interface into ROS2 via `/joint_state_publisher`
- 📝 TBD: Generalize to other Feetech-based robot models like Amazing Hand, Open Duck Mini, Reachy Mini, etc.                                                                                                                                                                                                                            
## Installation

Clone this repository and install dependencies using [rosdep](https://docs.ros.org/en/ros2_packages/rosdep.html):


### Clone the repository
`git clone https://github.com/jetdillo/lerobot_ws/`

`cd lerobot_ws`

### Install ROS 2 dependencies
`rosdep update`

`rosdep install --from-paths src --ignore-src -r -y`

### Build
`colcon build`

---
## Exercise the SO101 URDF in [lerobot_ws](https://github.com/Pavankv92/lerobot_ws/) 


### Rviz

Launch LeRobot SO101 in Rviz standalone. This is good as a demo of the URDF. Leave it running to test out the bridge node.

`ros2 launch lerobot_description so101_display.launch.py`

---

### Gazebo and ROS 2 Control

**Summary:** Gazebo and ROS 2 Control: Control the gripper inside Gazebo 

**Commands:**  
`ros2 launch lerobot_description so101_gazebo.launch.py`  
`ros2 launch lerobot_controller so101_controller.launch.py`

[lerobot_ws](https://github.com/Pavankv92/lerobot_ws/) also provides a ROS2 Control interface and interfaces into Gazebo & MoveIt!
Check out that repo for details on working with those tools as well. 

## Physical LeRobot  <-> ROS2 JointState Bridge

**Provides Control Interface for visualizing and/or controlling an IRL LeRobot arm in Gazebo,Rviz, etc.**
- Provides methods to map LeRobot encoder-ticks or percentage actuation values to ROS JointState angular values

## Usage Examples 
**Control SO-ARM10x in Rviz2:**

`ros2 launch so_arm_ros2_bridge bridge.launch.py port:=/dev/ttyACM0 calibration_path:=$HOME/.cache/huggingface/lerobot/calibration/robots/so101_follower/follower-arm.json mode:=ros_to_robot`

**Control URDF SO-ARM10x in Rviz2 or Gazebo:**

 `ros2 launch so_arm_ros2_bridge bridge.launch.py port:=/dev/ttyACM0 calibration_path:=$HOME/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/leader_arm.json mode:=robot_to_ros`



**Example Results:**

```
ros2 launch so_arm_ros2_bridge bridge.launch.py port:=/dev/ttyACM0 calibration_path:=/home/armadilo/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/my_leader.json mode:=robot_to_ros 
[INFO] [launch]: All log files can be found below /home/armadilo/.ros/log/2026-04-01-21-25-37-040201-roadrunner-147625
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Starting SO-ARM10x ROS2 bridge…
[INFO] [robot_state_publisher-1]: process started with pid [147626]
[INFO] [bridge_node-2]: process started with pid [147628]
[robot_state_publisher-1] [INFO] [1775103937.140374077] [robot_state_publisher]: got segment base
[robot_state_publisher-1] [INFO] [1775103937.140464616] [robot_state_publisher]: got segment gripper
[robot_state_publisher-1] [INFO] [1775103937.140469799] [robot_state_publisher]: got segment jaw
[robot_state_publisher-1] [INFO] [1775103937.140472326] [robot_state_publisher]: got segment lower_arm
[robot_state_publisher-1] [INFO] [1775103937.140474708] [robot_state_publisher]: got segment shoulder
[robot_state_publisher-1] [INFO] [1775103937.140477278] [robot_state_publisher]: got segment upper_arm
[robot_state_publisher-1] [INFO] [1775103937.140479731] [robot_state_publisher]: got segment world
[robot_state_publisher-1] [INFO] [1775103937.140482039] [robot_state_publisher]: got segment wrist
[bridge_node-2] [INFO] [1775103938.637899944] [so_arm_bridge]: Loaded 6 joints from /home/armadilo/lerobot_ws/install/so_arm_ros2_bridge/share/so_arm_ros2_bridge/config/joint_config.yaml
[bridge_node-2] [INFO] [1775103938.671702041] [so_arm_bridge]: SO-ARM bridge started — mode=robot_to_ros, port=/dev/ttyACM0, read_rate=50.0 Hz, mock=False
```

**Confirm /joint_state topic shows message flow:**
```
ros2 topic echo /joint_state
header:
  stamp:
    sec: 1775105010
    nanosec: 573142463
  frame_id: ''
name:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_flex_joint
- wrist_flex_joint
- wrist_roll_joint
- gripper_joint
position:
- -0.363642226569368
- -1.7614399835512002
- 1.6709130157554504
- 1.1469306513105593
- 0.1334889186140718
- 0.011302982731554162
```


## License

This project is based on [RobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) and adheres to their license.







