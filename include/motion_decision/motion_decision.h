/**
 * @file motion_decision.h
 * @author amsl
 * @brief C++ implementation of motion decision
 * @copyright Copyright (c) 2024
 */

#ifndef MOTION_DECISION_MOTION_DECISION_H
#define MOTION_DECISION_MOTION_DECISION_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <string>

struct Params
{
  int hz;
  int recovery_mode_threshold;
  int trigger_count_threshold;
  double max_speed;
  double max_yawrate;
  double dt;
  double predict_time;
  double collision_distance;
  double safety_collision_time;
  std::string stop_sound_path;
  std::string recovery_sound_path;
  std::string task_stop_sound_path;
};

struct Flags
{
  bool emergency_stop = false;
  bool task_stop = false;
  bool auto_mode = false;
  bool move_mode = false;
  bool joy = false;
  bool intersection = false;
  bool safety_mode = false;
  bool local_path_received = false;
  bool front_laser_received = false;
  bool rear_laser_received = false;
  bool enable_recovery_mode = true;
};

struct Counters
{
  int stuck = 0;
  int trigger = 0;
};

struct LaserInfo
{
  int front_min_idx = 0;
  int rear_min_idx = 0;
  float front_min_range = -1.0;
  float rear_min_range = -1.0;
};

/**
 * @class MotionDecision
 * @brief Motion Decision Class
 */
class MotionDecision
{
public:
  /**
   * @brief Constructor
   */
  MotionDecision(void);

  /**
   * @brief Process function
   */
  void process(void);

private:
  /**
   * @brief Load parameters function
   */
  void load_params(void);

  /**
   * @brief Emergency stop flag callback function
   * @param [in] msg Msg from emergency_stop_flag_sub_
   */
  void emergency_stop_flag_callback(const std_msgs::BoolConstPtr &msg);

  /**
   * @brief Front laser callback function
   * @details Cache the closest valid sensor data and its index in front_min_range and front_min_idx respectively
   * @param [in] msg Msg from front_laser_sub_
   */
  void front_laser_callback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief Joy callback function
   * @details Set flags by input from joy
   * @param [in] msg Msg from joy_sub_
   */
  void joy_callback(const sensor_msgs::JoyConstPtr &msg);

  /**
   * @brief Local path velocity callback function
   * @param [in] msg Msg from local_path_velocity_sub_
   */
  void local_path_velocity_callback(const geometry_msgs::TwistConstPtr &msg);

  /**
   * @brief Odom callback function
   * @param [in] msg Msg from odom_sub_
   */
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief Rear laser callback function
   * @details Cache the closest valid sensor data and its index in rear_min_range and rear_min_idx respectively
   * @param [in] msg Msg from rear_laser_sub_
   */
  void rear_laser_callback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief Recovery mode flag callback function
   * @param [in] msg Msg from recovery_mode_flag_sub_
   */
  void recovery_mode_flag_callback(const std_msgs::Bool::ConstPtr &msg);

  /**
   * @brief Task stop flag callback function
   * @details Emergency stop when task stop flag is true
   * @param [in] msg Msg from task_stop_flag_sub_
   */
  void task_stop_flag_callback(const std_msgs::BoolConstPtr &msg);

  /**
   * @brief Recovery mode function
   * @param [out] cmd_vel Velocity
   * @param [in] go_back Direction of motion
   */
  void recovery_mode(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief Calculate TTC (Time To Collision) function
   * @param [in] vel Current velocity
   * @param [in] go_back Direction of motion
   * @return float TTC
   */
  float calc_ttc(geometry_msgs::Twist vel);

  Params params_;
  Flags flags_;
  Counters counters_;
  LaserInfo laser_info_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher intersection_flag_pub_;
  ros::Publisher velocity_pub_;
  ros::Subscriber emergency_stop_flag_sub_;
  ros::Subscriber front_laser_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber local_path_velocity_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rear_laser_sub_;
  ros::Subscriber recovery_mode_flag_sub_;
  ros::Subscriber task_stop_flag_sub_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::Twist joy_vel_;
  geometry_msgs::Twist odom_vel_;
  sensor_msgs::LaserScan front_laser_;
  sensor_msgs::LaserScan rear_laser_;
};

#endif  // MOTION_DECISION_MOTION_DECISION_H
