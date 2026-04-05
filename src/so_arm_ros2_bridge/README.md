# SO-ARM10x ROS2 Bridge

Bidirectional bridge between the SO-ARM100/101 physical robot and ROS2
`sensor_msgs/JointState` messages for Gazebo Harmonic / RViz on **ROS2 Jazzy**.
(Also shown to work on **ROS Humble**)

This is a place-holder README.md until things are more stable 

Servo positions on LeRobot joint chains are handled as a dict of "joint_name":position
defined as dict[str,float]. Send one or more joints. 
For joint_names with "gripper" in them, we map % open to servo ticks

# Current model:
We assume an SO-ARM10x model that consists of a series of joints with ticks<->angle
positions ending in a terminal joint that maps ticks<->percent open/closed. 

TBD:
More attention to pure joint models(snakebot or leg-style joint chains) to make sure we get that right. 
Phalangial joint models: Amazing Hand w/ multiple servos providing adduction/abduction, etc. 
Other generalized chains of Feetech servos. 

# AI Disclosure:
This was prototyped by a conversation with Claude Sonnet 4.2, then edited & massaged by me where I thought it would be quicker to just do things myself. Specific refactoring on the ROS side is me.
