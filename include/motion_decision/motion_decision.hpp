// /**
//  * @file motion_decision.h
//  * @author amsl
//  * @brief C++ implementation of motion decision
//  * @copyright Copyright (c) 2024
//  */

// #ifndef MOTION_DECISION_MOTION_DECISION_HPP
// #define MOTION_DECISION_MOTION_DECISION_HPP

// #define RESET_COLOR "\033[0m"
// #define RED "\033[31m"
// #define GREEN "\033[32m"
// #define CYAN "\033[36m"
// #define YELLOW "\033[33m"

// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <optional>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joy.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <std_srvs/msg/set_bool.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <string>
// #include <utility>

// #include <Eigen/Dense>

// struct MotionDecisionParams
// {
//   bool enable_turbo_mode;
//   int hz;
//   int allowable_num_of_not_received_local_path;
//   float max_velocity;
//   float turbo_max_velocity;
//   float max_yawrate;
// };

// struct Flags
// {
//   bool local_path_updated = false;
//   bool move_trigger = false;
//   bool turbo_trigger = false;
// };

// struct Counters
// {
//   int not_received_local_path = 0;
// };

// /**
//  * @class MotionDecision
//  * @brief Motion Decision Class
//  */
// class MotionDecision
// {
// public:
//   /**
//    * @brief Constructor
//    */
//   MotionDecision(void);

//   /**
//    * @brief Process function
//    */
//   void process(void);

// private:
//   /**
//    * @brief Load parameters function
//    */
//   void load_params(void);


//   /**
//    * @brief Joy callback function
//    * @details Set flags by input from joy
//    * @param [in] msg Msg of Joy
//    */
//   void joy_callback(const sensor_msgs::JoyConstPtr &msg);

//   /**
//    * @brief Cmd vel callback function
//    * @param [in] msg Msg of local path cmd vel
//    */
//   void local_path_cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg);


//   /**
//    * @brief Select mode function
//    * @param [in] msg Msg of Joy
//    * @param [in] mode Current mode
//    * @return std::pair<std::string, std::string> New mode
//    */
//   std::pair<std::string, std::string>
//   select_mode(const sensor_msgs::Joy::ConstPtr &msg, const std::pair<std::string, std::string> &mode);

//   /**
//    * @brief Publish Cmd Vel function
//    * @param [in] cmd_vel Command velocity
//    */
//   void publish_cmd_vel(geometry_msgs::Twist cmd_vel);

//   /**
//    * @brief Print mode status function
//    * @param [in] status Status
//    * @param [in] color Color
//    */
//   void print_mode_status(const std::string &status = "", std::string color = "");

//   /**
//    * @brief Print status function
//    * @param [in] cmd_vel Command velocity
//    */
//   void print_status(const geometry_msgs::Twist &cmd_vel);

//   MotionDecisionParams params_;
//   Flags flags_;
//   Counters counters_;

//   // motiom mode {first: <stop, move>, second: <manual, auto>}
//   std::pair<std::string, std::string> mode_ = std::make_pair("stop", "manual");

//   ros::NodeHandle nh_;
//   ros::NodeHandle private_nh_;
//   ros::Publisher cmd_vel_pub_;
//   ros::Subscriber joy_sub_;
//   ros::Subscriber local_path_cmd_vel_sub_;

//   geometry_msgs::Twist cmd_vel_;
// };

// #endif  // MOTION_DECISION_MOTION_DECISION_HPP



/**
 * @file motion_decision.hpp
 * @author amsl
 * @brief C++ implementation of motion decision (ROS2)
 */

#ifndef MOTION_DECISION_MOTION_DECISION_HPP
#define MOTION_DECISION_MOTION_DECISION_HPP

#define RESET_COLOR "\033[0m"
#define RED         "\033[31m"
#define GREEN       "\033[32m"
#define CYAN        "\033[36m"
#define YELLOW      "\033[33m"

#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

struct MotionDecisionParams
{
  bool  enable_turbo_mode = false;
  int   hz = 20;
  int   allowable_num_of_not_received_local_path = 1;
  double max_velocity = 1.0;
  double turbo_max_velocity = 1.66;
  double max_yawrate = 1.0;
};

struct Flags
{
  bool local_path_updated = false;
  bool move_trigger = false;
  bool turbo_trigger = false;
};

struct Counters
{
  int not_received_local_path = 0;
};

/**
 * @class MotionDecision
 * @brief Motion Decision Class (ROS2 / rclcpp::Node)
 */
class MotionDecision : public rclcpp::Node
{
public:
  /// @brief Constructor
  MotionDecision();

private:
  /// @brief Load parameters from ROS2 parameter server
  void load_params();

  /// @brief Joy callback: mode switching & manual cmd generation
  /// @param msg Joy message
  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  /// @brief Local path cmd_vel callback: adopt cmd_vel in auto mode
  /// @param msg Twist from local path follower
  void local_path_cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void task_type_callback(const std_msgs::msg::String::ConstSharedPtr msg);


  /// @brief Select mode from joy input
  std::pair<std::string, std::string>
  select_mode(const sensor_msgs::msg::Joy::ConstSharedPtr joy,
              const std::pair<std::string, std::string> &prev_mode);

  /// @brief Timer tick: watchdog & publish
  void on_timer();

  /// @brief Final safety clamp & publish
  void publish_cmd_vel(geometry_msgs::msg::Twist cmd_vel);

  /// @brief Print mode status (colored)
  void print_mode_status(const std::string &status = "", std::string color = "");

  /// @brief Print current status and cmd_vel
  void print_status(const geometry_msgs::msg::Twist &cmd_vel);

  // ==== State ====
  MotionDecisionParams params_;
  Flags    flags_;
  Counters counters_;

  // motion mode {first: <stop, move>, second: <manual, auto>}
  std::pair<std::string, std::string> mode_ = std::make_pair("stop", "manual");

  geometry_msgs::msg::Twist cmd_vel_;

  // ==== ROS2 interfaces ====
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr local_path_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_type_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr task1_timer_;
};

#endif  // MOTION_DECISION_MOTION_DECISION_HPP
