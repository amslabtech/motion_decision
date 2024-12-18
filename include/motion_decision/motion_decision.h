/**
 * @file motion_decision.h
 * @author amsl
 * @brief C++ implementation of motion decision
 * @copyright Copyright (c) 2024
 */

#ifndef MOTION_DECISION_MOTION_DECISION_H
#define MOTION_DECISION_MOTION_DECISION_H

#define RESET_COLOR "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define CYAN "\033[36m"
#define YELLOW "\033[33m"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <optional>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <string>
#include <utility>

#include <Eigen/Dense>

struct MotionDecisionParams
{
  bool use_rear_laser;
  bool use_360_laser;
  bool use_local_map;
  bool use_footprint;
  bool enable_turbo_mode;
  int hz;
  int allowable_num_of_not_received;
  int allowable_num_of_not_received_local_path;
  float max_velocity;
  float turbo_max_velocity;
  float max_yawrate;
  float dt;
  float predict_time;
  float collision_distance;
  float safety_collision_time;
  float stuck_time_threshold;
  float angle_increment;
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
  bool move_trigger = false;
  bool turbo_trigger = false;
};

struct Counters
{
  int stuck = 0;
  int recovery = 0;
  int not_received_front_laser = 0;
  int not_received_rear_laser = 0;
  int not_received_local_path = 0;
};

struct LaserInfo
{
  int front_index_of_min_range = 0;
  int rear_index_of_min_range = 0;
  float front_min_range = -1.0;
  float rear_min_range = -1.0;
};

struct BatteryInfo
{
  float full_charge_voltage = 0.0;
  float cutoff_voltage = 0.0;
  float current_percentage = 0.0;
  bool used = false;
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
   * @brief Local map callback function
   * @param [in] msg Msg of local map
   */
  void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);

  /**
   * @brief Battery voltage callback function
   * @param [in] msg Msg of battery voltage
   */
  void battery_voltage_callback(const std_msgs::Float32ConstPtr &msg);

  /**
   * @brief Footprint callback function
   * @param [in] msg Msg of footprint
   */
  void footprint_callback(const geometry_msgs::PolygonStampedPtr &msg);

  /**
   * @brief Recovery mode flag callback function
   * @details This is not flag to run recovery mode. This is flag to use recovery mode.
   * @param [in] req Request of recovery mode flag
   * @param [out] res Response of recovery mode flag
   * @return bool Result of service
   */
  bool recovery_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * @brief Task stop flag service callback function
   * @param [in] req Request of task stop flag
   * @param [out] res Response of task stop flag
   * @return bool Result of service
   */
  bool task_stop_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * @brief Cache the closest sensor data and its index
   * @param [in] laser Laser data
   * @param [out] min_range Closest sensor data
   * @param [out] index_of_min_range Index of the closest sensor data
   */
  void search_min_range(const sensor_msgs::LaserScan &laser, float &min_range, int &index_of_min_range);

  /**
   * @brief Invert footprint function
   * @param [in] footprint Footprint
   * @return geometry_msgs::PolygonStamped Inversed footprint
   */
  geometry_msgs::PolygonStamped invert_footprint(const geometry_msgs::PolygonStamped &footprint);

  /**
   * @brief Create laser from 360 laser function
   * @param [in] msg Msg of 360 laser
   * @param [in] direction Direction of laser
   * @return sensor_msgs::LaserScan Laser data
   */
  sensor_msgs::LaserScan create_laser_from_360_laser(const sensor_msgs::LaserScan &msg, const std::string &direction);

  /**
   * @brief Create laser from local map function
   * @param [in] msg Msg of local map
   * @param [in] direction Direction of laser
   * @return sensor_msgs::LaserScan Laser data
   */
  sensor_msgs::LaserScan create_laser_from_local_map(const nav_msgs::OccupancyGrid &msg, const std::string &direction);

  /**
   * @brief Adjust distance for footprint function
   * @param [in] msg Msg of laser
   * @param [in] direction Direction of laser
   * @return sensor_msgs::LaserScan Adjusted laser data
   */
  sensor_msgs::LaserScan adjust_dist_for_footprint(const sensor_msgs::LaserScan &msg, const std::string &direction);

  /**
   * @brief Calcualte intersection function
   * @param [in] obstacle Obstacle
   * @param [in] footprint Footprint
   * @return geometry_msgs::Point Intersection point
   */
  geometry_msgs::Point calc_intersection(
    const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint);

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
   * @brief Print mode status function
   * @param [in] status Status
   * @param [in] color Color
   */
  void print_mode_status(const std::string &status = "", std::string color = "");

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
  BatteryInfo battery_info_;
  std::optional<geometry_msgs::PolygonStamped> footprint_;
  std::optional<geometry_msgs::PolygonStamped> footprint_inversed_;

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
  ros::Subscriber local_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rear_laser_sub_;
  ros::Subscriber battery_voltage_sub_;
  ros::Subscriber footprint_sub_;
  ros::ServiceServer recovery_mode_flag_server_;
  ros::ServiceServer task_stop_flag_server_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::Twist odom_vel_;
  std::optional<sensor_msgs::LaserScan> front_laser_;
  std::optional<sensor_msgs::LaserScan> rear_laser_;
};

#endif  // MOTION_DECISION_MOTION_DECISION_H
