# motion_decision

[![ci](https://github.com/amslabtech/motion_decision/workflows/ci/badge.svg)](https://github.com/amslabtech/motion_decision/actions)
![issue_opened](https://img.shields.io/github/issues/amslabtech/motion_decision.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/motion_decision.svg)

## Enviornment
- Ubuntu 20.04
- ROS Noetic

## Published topics
- /cmd_vel (geometry_msgs/Twist)

## Subscribed topics
- /local_path/cmd_vel (geometry_msgs/Twist)
- /emergency_stop (std_msgs/Bool)
- /joy (sensor_msgs/Joy)

### services
- /task/stop (std_srvs/SetBool)

## Parameters
- hz
  - loop rate (default: 20)
- max_speed
  - max speed (default: 1.0)
- max_yawrate
  - max_yawrate (default: 1.0)
