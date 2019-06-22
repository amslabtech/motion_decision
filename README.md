# motion_decision

![issue_opened](https://img.shields.io/github/issues/amslabtech/motion_decision.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/motion_decision.svg)

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Published topics
- /cmd_vel (geometry_msgs/Twist)

## Subscribed topics
- /local_path/cmd_vel (geometry_msgs/Twist)
- /task/stop (std_msgs/Bool)
- /emergency_stop (std_msgs/Bool)
- /joy (sensor_msgs/Joy)

## Parameters
- hz
  - loop rate (default: 20)
- max_speed
  - max speed (default: 1.0)
- max_yawrate
  - max_yawrate (default: 1.0)
