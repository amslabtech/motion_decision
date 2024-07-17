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
#include <optional>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <string>
#include <utility>

struct MotionDecisionParams
{
  int hz;
  int allowable_num_of_not_received;
  float max_velocity;
  float max_yawrate;
  float dt;
  float predict_time;
  float collision_distance;
  float safety_collision_time;
  float stuck_time_threshold;
  std::string stop_sound_path;
  std::string recovery_sound_path;
  std::string task_stop_sound_path;
};

struct RecoveryParams
{
  bool available;
  bool sim_back;
  float max_velocity;
  float max_yawrate;
  float velocity_resolution;
  float yawrate_resolution;
  float spin_turn_speed;
  float time;
};

struct Flags
{
  bool emergency_stop = false;
  bool task_stop = false;
  bool intersection = false;
  bool local_path_updated = false;
  bool front_laser_updated = false;
  bool rear_laser_updated = false;
};

struct Counters
{
  int stuck = 0;
  int recovery = 0;
  int not_received_front_laser = 0;
  int not_received_rear_laser = 0;
};

struct LaserInfo
{
  int front_index_of_min_range = 0;
  int rear_index_of_min_range = 0;
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
   * @details [Warn] If you stop the robot by emergency stop, you need to republish the flag to false to run the robot.
   *   You can't unlock the robot by Joy.
   * @param [in] msg Msg of emergency stop flag
   */
  void emergency_stop_flag_callback(const std_msgs::BoolConstPtr &msg) { flags_.emergency_stop = msg->data; }

  /**
   * @brief Front laser callback function
   * @param [in] msg Msg of front laser
   */
  void front_laser_callback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief Joy callback function
   * @details Set flags by input from joy
   * @param [in] msg Msg of Joy
   */
  void joy_callback(const sensor_msgs::JoyConstPtr &msg);

  /**
   * @brief Cmd vel callback function
   * @param [in] msg Msg of local path cmd vel
   */
  void local_path_cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg);

  /**
   * @brief Odom callback function
   * @param [in] msg Msg of odom
   */
  void odom_callback(const nav_msgs::OdometryConstPtr &msg) { odom_vel_ = msg->twist.twist; }

  /**
   * @brief Rear laser callback function
   * @param [in] msg Msg of rear laser
   */
  void rear_laser_callback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief Recovery mode flag callback function
   * @details This is not flag to run recovery mode. This is flag to use recovery mode.
   * @param [in] msg Msg of recovery mode flag
   */
  void recovery_mode_flag_callback(const std_msgs::Bool::ConstPtr &msg) { params_of_recovery_.available = msg->data; }

  /**
   * @brief Task stop flag callback function
   * @param [in] msg Msg of task stop flag
   */
  void task_stop_flag_callback(const std_msgs::BoolConstPtr &msg);

  /**
   * @brief Cache the closest sensor data and its index
   * @param [in] laser Laser data
   * @param [out] min_range Closest sensor data
   * @param [out] index_of_min_range Index of the closest sensor data
   */
  void search_min_range(const sensor_msgs::LaserScan &laser, float &min_range, int &index_of_min_range);

  /**
   * @brief Select mode function
   * @param [in] msg Msg of Joy
   * @param [in] mode Current mode
   * @return std::pair<std::string, std::string> New mode
   */
  std::pair<std::string, std::string>
  select_mode(const sensor_msgs::Joy::ConstPtr &msg, const std::pair<std::string, std::string> &mode);

  /**
   * @brief Sound function
   * @param [in] path Path of sound file
   */
  void sound(const std::string &path);

  /**
   * @brief Recovery mode function
   * @details Run when stuck is detected. Move away from the nearest obstacle. Face the direction in which the
   *   LocalPathPlanner is comfortable moving.
   * @param [in] cmd_vel Command velocity
   * @param [in] sim_back Flag to simulate back
   * @return geometry_msgs::Twist Command velocity
   */
  geometry_msgs::Twist recovery_mode(geometry_msgs::Twist cmd_vel, const bool sim_back = true);

  /**
   * @brief Calculate TTC (Time To Collision) function
   * @param [in] velocity Velocity
   * @param [in] yawrate Yawrate
   * @param [in] laser Laser data
   * @return float TTC
   */
  float calc_ttc(const float &velocity, const float &yawrate, const sensor_msgs::LaserScan &laser);

  /**
   * @brief Simulate by uniform circular motion function
   * @param [in] velocity Velocity
   * @param [in] yawrate Yawrate
   * @param [in] sim_time Simulation time
   * @return std::pair<float, float> {predicted_x, predicted_y}
   */
  std::pair<float, float>
  sim_by_uniform_circluar_motion(const float &velocity, const float &yawrate, const float &sim_time);

  /**
   * @brief Publish Cmd Vel function
   * @param [in] cmd_vel Command velocity
   */
  void publish_cmd_vel(geometry_msgs::Twist cmd_vel);

  /**
   * @brief Print status function
   * @param [in] cmd_vel Command velocity
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
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber emergency_stop_flag_sub_;
  ros::Subscriber front_laser_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber local_path_cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rear_laser_sub_;
  ros::Subscriber recovery_mode_flag_sub_;
  ros::Subscriber task_stop_flag_sub_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::Twist odom_vel_;
  std::optional<sensor_msgs::LaserScan> front_laser_;
  std::optional<sensor_msgs::LaserScan> rear_laser_;
};

#endif  // MOTION_DECISION_MOTION_DECISION_H
