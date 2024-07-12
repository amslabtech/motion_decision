/**
 * @file motion_decision.h
 * @author amsl
 * @brief C++ implementation of motion decision
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
  void recovery_mode(geometry_msgs::Twist &cmd_vel, bool go_back);

  /**
   * @brief Calculate TTC (Time To Collision) function
   * @param [in] vel Current velocity
   * @param [in] go_back Direction of motion
   * @return float TTC
   */
  float calc_ttc(geometry_msgs::Twist vel, bool go_back);

  int HZ;
  int RECOVERY_MODE_THRESHOLD;
  int TRIGGER_COUNT_THRESHOLD;
  double MAX_SPEED;
  double MAX_YAWRATE;
  double DT;
  double PREDICT_TIME;
  double COLLISION_DISTANCE;
  double SAFETY_COLLISION_TIME;
  std::string STOP_SOUND_PATH;
  std::string RECOVERY_SOUND_PATH;
  std::string TASK_STOP_SOUND_PATH;

  bool emergency_stop_flag_;
  bool task_stop_flag_;
  bool auto_flag_;
  bool move_flag_;
  bool joy_flag_;
  bool intersection_flag_;
  bool safety_mode_flag_;
  bool local_path_received_;
  bool front_laser_received_;
  bool rear_laser_received_;
  bool enable_recovery_mode_;
  int stuck_count_;
  int trigger_count_;
  int front_min_idx_;
  int rear_min_idx_;
  float front_min_range_;
  float rear_min_range_;

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
  sensor_msgs::LaserScan front_laser_;
  sensor_msgs::LaserScan rear_laser_;
  sensor_msgs::Joy joy_;
  nav_msgs::Odometry odom_;
};

#endif  // MOTION_DECISION_MOTION_DECISION_H
