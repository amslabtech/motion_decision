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

struct MotionDecisionParams
{
  int hz;
  int recovery_mode_threshold;
  int trigger_count_threshold;
  float max_speed;
  float max_yawrate;
  float dt;
  float predict_time;
  float collision_distance;
  float safety_collision_time;
  std::string stop_sound_path;
  std::string recovery_sound_path;
  std::string task_stop_sound_path;
};

struct RecoveryParams
{
  float max_velocity;
  float max_yawrate;
  float velocity_resolution;
  float yawrate_resolution;
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
  bool recovery_mode = false;
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
   * @brief Publish velocity function
   * @param [in] laser Laser data
   * @param [in] min_idx Index of the closest valid sensor data
   * @param [in] min_range Closest valid sensor data
   */
  void search_min_range(const sensor_msgs::LaserScan &laser, int &min_idx, float &min_range);

  /**
   * @brief Publish velocity function
   * @param [in] msg Msg of Joy
   * @param [in] mode Current mode
   * @return std::pair<std::string, std::string> New mode
   */
  std::pair<std::string, std::string>
  select_mode(const sensor_msgs::Joy::ConstPtr &msg, const std::pair<std::string, std::string> &mode);

  /**
   * @brief Recovery mode function
   * @details Run when stuck is detected. Move away from the nearest obstacle. Face the direction in which the
   *   LocalPlanner is comfortable moving.
   * @param [out] cmd_vel Velocity
   * @param [in] go_back Direction of motion
   */
  void recovery_mode(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief Calculate TTC (Time To Collision) function
   * @param [in] velocity Velocity
   * @param [in] yawrate Yawrate
   * @return float TTC
   */
  float calc_ttc(const float &velocity, const float &yawrate);

  /**
   * @brief Simulate by uniform circular motion function
   * @param [in] velocity Velocity
   * @param [in] yawrate Yawrate
   * @param [in] sim_time Simulation time
   * @return std::pair<float, float> {predicted_x, predicted_y}
   */
  std::pair<float, float> sim_by_uniform_circluar_motion(const float &velocity, const float &yawrate, const float &sim_time);

  /**
   * @brief Publish velocity function
   * @param [in] cmd_vel Velocity
   */
  void publish_cmd_vel(geometry_msgs::Twist cmd_vel);

  /**
   * @brief Print status function
   * @param [in] cmd_vel Velocity
   */
  void print_status(const geometry_msgs::Twist &cmd_vel);

  MotionDecisionParams params_;
  RecoveryParams params_of_recovery_;
  Flags flags_;
  Counters counters_;
  LaserInfo laser_info_;

  // motiom mode {first: <stop, move>, second: <manual, auto>}
  std::pair<std::string, std::string> mode_ = std::make_pair("stop", "manual");

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
