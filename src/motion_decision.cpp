// /**
//  * @file motion_decision.cpp
//  * @author amsl
//  * @brief C++ implementation of motion decision
//  * @copyright Copyright (c) 2024
//  */

// #include <algorithm>
// #include <string>
// #include <utility>
// #include <vector>

// #include "motion_decision/motion_decision.h"

// MotionDecision::MotionDecision(void) : private_nh_("~")
// {
//   cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

//   joy_sub_ = nh_.subscribe("joy", 1, &MotionDecision::joy_callback, this);

//   load_params();
//   if (!params_.enable_turbo_mode)
//     params_.turbo_max_velocity = params_.max_velocity;

//   // 初期モード
//   mode_.first  = "stop";    // "stop" / "move"
//   mode_.second = "manual";  // manual固定
// }

// void MotionDecision::load_params(void)
// {
//   // MotionDecisionParams
//   private_nh_.param<bool>("enable_turbo_mode", params_.enable_turbo_mode, false);
//   private_nh_.param<int>("hz", params_.hz, 20);
//   private_nh_.param<int>("allowable_num_of_not_received", params_.allowable_num_of_not_received, 3);

//   private_nh_.param<float>("max_velocity", params_.max_velocity, 1.0);
//   private_nh_.param<float>("turbo_max_velocity", params_.turbo_max_velocity, 1.66);
//   private_nh_.param<float>("max_yawrate", params_.max_yawrate, 1.0);
// }


// void MotionDecision::joy_callback(const sensor_msgs::JoyConstPtr &msg)
// {
//   mode_ = select_mode(msg, mode_);

//   if (mode_.first == "move" && mode_.second == "manual")
//   {
//     flags_.move_trigger = msg->buttons[4];
//     flags_.turbo_trigger = params_.enable_turbo_mode && flags_.move_trigger && msg->axes[2] == -1.0;
//     if (flags_.turbo_trigger)
//     {
//       cmd_vel_.linear.x = flags_.move_trigger ? msg->axes[1] * params_.turbo_max_velocity : 0.0;
//       cmd_vel_.angular.z = flags_.move_trigger ? msg->axes[0] * params_.max_yawrate : 0.0;
//     }
//     else
//     {
//       cmd_vel_.linear.x = flags_.move_trigger ? msg->axes[1] * params_.max_velocity : 0.0;
//       cmd_vel_.angular.z = flags_.move_trigger ? msg->axes[0] * params_.max_yawrate : 0.0;
//     }
//   }
//   else if (
//       mode_.first == "move" && mode_.second == "auto" &&
//       params_.allowable_num_of_not_received_local_path < counters_.not_received_local_path)
//   {
//     cmd_vel_ = geometry_msgs::Twist();
//     ROS_WARN_THROTTLE(1.0, "Local path cmd vel has not been updated");
//   }

//   if (mode_.first == "move")
//     flags_.task_stop = false;
// }

// std::pair<std::string, std::string>
// MotionDecision::select_mode(const sensor_msgs::Joy::ConstPtr &joy, const std::pair<std::string, std::string> &prev_mode)
// {
//   std::string first_mode = prev_mode.first;
//   std::string second_mode = prev_mode.second;

//   if (joy->buttons[0] == 1)  // cross button
//     first_mode = "stop";
//   if (joy->buttons[1] == 1)  // circle button
//     first_mode = "move";
//   if (joy->buttons[2] == 1)  // triangle button
//     second_mode = "auto";
//   if (joy->buttons[3] == 1)  // square button
//     second_mode = "manual";

//   return std::make_pair(first_mode, second_mode);
// }


// void MotionDecision::process(void)
// {
//   ros::Rate loop_rate(params_.hz);
//   while (ros::ok())
//   {


//     publish_cmd_vel(cmd_vel_);


//     loop_rate.sleep();
//     ros::spinOnce();

//   }
// }


// void MotionDecision::print_mode_status(const std::string &status, std::string color)
// {
//   if (color == "")
//     color = RESET_COLOR;
//   const std::string parrentheses = status == "" ? ")" : ") ";
//   std::cout << "=== " << mode_.second << " (" << mode_.first << parrentheses << color << status << RESET_COLOR
//             << " ===" << std::endl;
// }

// void MotionDecision::print_status(const geometry_msgs::Twist &cmd_vel)
// {
//   if (mode_.first == "move" && mode_.second == "manual")
//   {
//     if (flags_.turbo_trigger)
//       print_mode_status("TURBO", CYAN);
//     else if (flags_.move_trigger)
//       print_mode_status("UNLOCKED", GREEN);
//     else
//       print_mode_status("LOCKED", RED);
//   }
//   else
//   {
//     print_mode_status();
//   }



//   if (mode_.first == "move")
//   {
//     std::cout << "cmd vel :" << std::endl;
//     std::cout << "  linear.x  : " << cmd_vel.linear.x << std::endl;
//     std::cout << "  angular.z : " << cmd_vel.angular.z << std::endl;
//   }
//   std::cout << std::endl;
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "motion_decision");
//   MotionDecision motion_decision;
//   motion_decision.process();

//   return 0;
// }



// ##############################################################


// /**
//  * @file motion_decision.cpp
//  * @author amsl
//  * @brief implementation of motion decision
//  */

// #include <algorithm>
// #include <string>
// #include <utility>
// #include <vector>

// #include "motion_decision/motion_decision.hpp"

// MotionDecision::MotionDecision()
// : Node("motion_decision")
// {
//   // 出力：cmd_vel
//   cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

//   // 入力：ジョイ
//   joy_sub_ = nh_.subscribe("joy", 1, &MotionDecision::joy_callback, this);

//   local_path_cmd_vel_sub_ = nh_.subscribe("local_path/cmd_vel", 1, &MotionDecision::local_path_cmd_vel_callback, this);

//   // パラメータ
//   load_params();

//   // ターボ無効なら上限を通常速度に揃える
//   if (!params_.enable_turbo_mode)
//     params_.turbo_max_velocity = params_.max_velocity;

//   // 初期モード（manual固定）
//   mode_.first  = "stop";    // "stop" / "move"
//   mode_.second = "manual";  // "manual"/"auto"
// }

// void MotionDecision::load_params(void)
// {

//   private_nh_.param<bool>("enable_turbo_mode", params_.enable_turbo_mode, false);
//   private_nh_.param<int>("hz", params_.hz, 20);
//   private_nh_.param<int>(
//     "allowable_num_of_not_received_local_path", params_.allowable_num_of_not_received_local_path, 1);

//   private_nh_.param<float>("max_velocity", params_.max_velocity, 1.0);
//   private_nh_.param<float>("turbo_max_velocity", params_.turbo_max_velocity, 1.66);
//   private_nh_.param<float>("max_yawrate", params_.max_yawrate, 1.0);
// }

// void MotionDecision::joy_callback(const sensor_msgs::JoyConstPtr &msg)
// {
//   mode_ = select_mode(msg, mode_);

//   if (mode_.first == "move" && mode_.second == "manual")  // move+manual のときだけ反映
//   {
//     // 例: L1(buttons[4]) を押している間だけ動作
//     flags_.move_trigger  = msg->buttons[4];
//     flags_.turbo_trigger = params_.enable_turbo_mode && flags_.move_trigger && msg->axes[2] == -1.0;

//     if (flags_.turbo_trigger)
//     {
//       cmd_vel_.linear.x = flags_.move_trigger ? msg->axes[1] * params_.turbo_max_velocity : 0.0;
//       cmd_vel_.angular.z = flags_.move_trigger ? msg->axes[0] * params_.max_yawrate : 0.0;
//     }
//     else
//     {
//       cmd_vel_.linear.x = flags_.move_trigger ? msg->axes[1] * params_.max_velocity : 0.0;
//       cmd_vel_.angular.z = flags_.move_trigger ? msg->axes[0] * params_.max_yawrate : 0.0;
//     }
//   }
//   else {
//     // stop のときだけゼロ
//     if (mode_.first == "stop") {
//       cmd_vel_ = geometry_msgs::Twist();
//     }
//   }
//   // manual以外ではターボを確実にオフ（持ち越し防止）
//   if (mode_.second != "manual") {
//     flags_.turbo_trigger = false;
//   }
// }

// void MotionDecision::local_path_cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
// {
//   if (mode_.first == "move" && mode_.second == "auto")
//     cmd_vel_ = *msg;                    // ← ここで /local_path/cmd_vel をそのまま採用
//   flags_.local_path_updated = true;
//   counters_.not_received_local_path = 0;
// }


// std::pair<std::string, std::string>
// MotionDecision::select_mode(const sensor_msgs::Joy::ConstPtr &joy,
//                             const std::pair<std::string, std::string> &prev_mode)
// {
//   std::string first_mode  = prev_mode.first;
//   std::string second_mode = prev_mode.second;

//   if (joy->buttons[0] == 1)  // cross button
//     first_mode = "stop";
//   if (joy->buttons[1] == 1)  // circle button
//     first_mode = "move";
//   if (joy->buttons[2] == 1)  // triangle button
//     second_mode = "auto";
//   if (joy->buttons[3] == 1)  // square button
//     second_mode = "manual";

//   return std::make_pair(first_mode, second_mode);
// }

// void MotionDecision::process(void)
// {
//   ros::Rate loop_rate(params_.hz);
//   while (ros::ok())
//   {
//     // ウォッチドッグ初期化（この周期で更新が無ければ欠測）
//     flags_.local_path_updated = false;

//     ros::spinOnce();  // joy / local_path を処理

//     // /local_path/cmd_vel のウォッチドッグ
//     if (mode_.first == "move" && mode_.second == "auto") {
//       if (!flags_.local_path_updated) {
//         counters_.not_received_local_path++;
//         if (counters_.not_received_local_path >
//             params_.allowable_num_of_not_received_local_path) {
//           cmd_vel_ = geometry_msgs::Twist();  // 許容超え→停止
//         }
//       } else {
//         counters_.not_received_local_path = 0;
//       }
//     } else {
//       counters_.not_received_local_path = 0; // auto以外は監視リセット
//     }

//     publish_cmd_vel(cmd_vel_);
//     loop_rate.sleep();
//   }
// }


// void MotionDecision::publish_cmd_vel(geometry_msgs::Twist cmd_vel)
// {
//   if (flags_.turbo_trigger)
//   {
//     cmd_vel.linear.x = 0.0 < cmd_vel.linear.x
//                            ? std::min(cmd_vel.linear.x, static_cast<double>(params_.turbo_max_velocity))
//                            : std::max(cmd_vel.linear.x, static_cast<double>(-params_.turbo_max_velocity));
//     cmd_vel.angular.z = 0.0 < cmd_vel.angular.z
//                             ? std::min(cmd_vel.angular.z, static_cast<double>(params_.max_yawrate))
//                             : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));
//   }
//   else
//   {
//     cmd_vel.linear.x = 0.0 < cmd_vel.linear.x ? std::min(cmd_vel.linear.x, static_cast<double>(params_.max_velocity))
//                                               : std::max(cmd_vel.linear.x, static_cast<double>(-params_.max_velocity));
//     cmd_vel.angular.z = 0.0 < cmd_vel.angular.z
//                             ? std::min(cmd_vel.angular.z, static_cast<double>(params_.max_yawrate))
//                             : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));
//   }

//   if (mode_.first == "stop")
//     cmd_vel = geometry_msgs::Twist();

//   cmd_vel_pub_.publish(cmd_vel);
//   print_status(cmd_vel);
// }

// void MotionDecision::print_mode_status(const std::string &status, std::string color)
// {
//   if (color == "") color = RESET_COLOR;
//   const std::string parrentheses = status == "" ? ")" : ") ";
//   std::cout << "=== " << mode_.second << " (" << mode_.first << parrentheses
//             << color << status << RESET_COLOR << " ===" << std::endl;
// }

// void MotionDecision::print_status(const geometry_msgs::Twist &cmd_vel)
// {
//   // manual時の簡易表示のみ
//   if (mode_.first == "move" && mode_.second == "manual")
//   {
//     if (flags_.turbo_trigger)
//       print_mode_status("TURBO", CYAN);
//     else if (flags_.move_trigger)
//       print_mode_status("UNLOCKED", GREEN);
//     else
//       print_mode_status("LOCKED", RED);
//   }
//   else
//   {
//     print_mode_status();
//   }

//   if (mode_.first == "move")
//   {
//     std::cout << "cmd vel :" << std::endl;
//     std::cout << "  linear.x  : " << cmd_vel.linear.x << std::endl;
//     std::cout << "  angular.z : " << cmd_vel.angular.z << std::endl;
//   }
//   std::cout << std::endl;
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "motion_decision");
//   MotionDecision motion_decision;
//   motion_decision.process();
//   return 0;
// }



// ###############################################################




// motion_decision.cpp (ROS2 / rclcpp)
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>


#include "motion_decision/motion_decision.hpp"  // 既存ヘッダをそのまま想定（flags_, counters_, params_, print_* など）

using namespace std::chrono_literals;

MotionDecision::MotionDecision()
: rclcpp::Node("motion_decision")
{
  // === パラメータ読み込み ===
  load_params();

  // ターボ無効なら上限を通常速度に揃える
  if (!params_.enable_turbo_mode) {
    params_.turbo_max_velocity = params_.max_velocity;
  }

  // 初期モード
  mode_.first  = "stop";    // "stop" / "move"
  mode_.second = "manual";  // "manual" / "auto"

  // === Pub/Sub ===
  // ROS1のlatched=true相当（transient local）はcmd_velでは通常不要なので付けない
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("whill/controller/cmd_vel", rclcpp::QoS(1));

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::QoS(1),
    std::bind(&MotionDecision::joy_callback, this, std::placeholders::_1));

  local_path_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1),
    std::bind(&MotionDecision::local_path_cmd_vel_callback, this, std::placeholders::_1));

  // /task_type サブスクライブ: "stop" / "none" を受け取る
  task_type_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_type", rclcpp::QoS(10),
    std::bind(&MotionDecision::task_type_callback, this, std::placeholders::_1));


  // === タイマー（ウォッチドッグ含むメインループ） ===
  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, params_.hz)));
  timer_ = this->create_wall_timer(period, std::bind(&MotionDecision::on_timer, this));
}

void MotionDecision::load_params()
{
  // declare_parameter は1回目で宣言＋既定値設定、以降は get_parameter でOK
  this->declare_parameter<bool>("enable_turbo_mode", false);
  this->declare_parameter<int>("hz", 20);
  this->declare_parameter<int>("allowable_num_of_not_received_local_path", 10);
  this->declare_parameter<double>("max_velocity", 1.0);
  this->declare_parameter<double>("turbo_max_velocity", 1.66);
  this->declare_parameter<double>("max_yawrate", 1.0);

  this->get_parameter("enable_turbo_mode", params_.enable_turbo_mode);
  this->get_parameter("hz", params_.hz);
  this->get_parameter("allowable_num_of_not_received_local_path", params_.allowable_num_of_not_received_local_path);
  this->get_parameter("max_velocity", params_.max_velocity);
  this->get_parameter("turbo_max_velocity", params_.turbo_max_velocity);
  this->get_parameter("max_yawrate", params_.max_yawrate);
}

void MotionDecision::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  mode_ = select_mode(msg, mode_);

  if (mode_.first == "move" && mode_.second == "manual")  // move+manual のときだけ反映
  {
    // 例: L1(buttons[4]) を押している間だけ動作
    flags_.move_trigger  = (msg->buttons.size() > 4) ? (msg->buttons[4] != 0) : false;
    const bool turbo_req = (msg->axes.size() > 2) ? (msg->axes[2] == -1.0f) : false;
    flags_.turbo_trigger = params_.enable_turbo_mode && flags_.move_trigger && turbo_req;

    const double vx_scale = flags_.turbo_trigger ? params_.turbo_max_velocity : params_.max_velocity;
    const double wz_scale = params_.max_yawrate;

    const double ax1 = (msg->axes.size() > 1) ? msg->axes[1] : 0.0; // forward/back
    const double ax0 = (msg->axes.size() > 0) ? msg->axes[0] : 0.0; // yaw

    cmd_vel_.linear.x  = flags_.move_trigger ? ax1 * vx_scale : 0.0;
    cmd_vel_.angular.z = flags_.move_trigger ? ax0 * wz_scale : 0.0;
    std::cout << "cmd vel_manual :" << std::endl;
    std::cout << "  linear.x  : " << cmd_vel_.linear.x << std::endl;
    std::cout << "  angular.z : " << cmd_vel_.angular.z << std::endl;
  }
  else
  {
    // auto を使うので、auto時までゼロにしない。stop のときだけゼロ。
    if (mode_.first == "stop") {
      cmd_vel_ = geometry_msgs::msg::Twist();
    }
  }

  // manual以外ではターボを確実にオフ（持ち越し防止）
  if (mode_.second != "manual") {
    flags_.turbo_trigger = false;
  }
}

void MotionDecision::local_path_cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  if (mode_.first == "move" && mode_.second == "auto") {
    cmd_vel_ = *msg;   // /local_path/cmd_vel をそのまま採用
  }
  flags_.local_path_updated = true;
  counters_.not_received_local_path = 0;
  std::cout << "cmd vel_autoflag :" << std::endl;
  std::cout << "  linear.x  : " << cmd_vel_.linear.x << std::endl;
  std::cout << "  angular.z : " << cmd_vel_.angular.z << std::endl;
}

// /task_type 用コールバック
void MotionDecision::task_type_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  // "stop" のときだけ強制的に停止モードにする
  if (msg->data == "stop") {
    mode_.first = "stop";
  }
  // "none" のときは何もしない（現在の mode を維持）
  else if (msg->data == "task1") {
    mode_.first = "stop"; 
    // 既存のタイマーがあればキャンセル
    if (task1_timer_) task1_timer_->cancel(); 
    // 1分後にmoveに戻すタイマーをセット
    task1_timer_ = this->create_wall_timer(
      std::chrono::seconds(40),
      [this]() {
        mode_.first = "move";
        task1_timer_->cancel(); // これで1回だけ実行
      }
    );
  }
  // 他の文字列が来たときに何かしたくなったらここに追記
}

std::pair<std::string, std::string>
MotionDecision::select_mode(const sensor_msgs::msg::Joy::ConstSharedPtr joy,
                            const std::pair<std::string, std::string> &prev_mode)
{
  std::string first_mode  = prev_mode.first;
  std::string second_mode = prev_mode.second;

  if (joy->buttons.size() > 0 && joy->buttons[0] == 1)  // cross
    first_mode = "stop";
  if (joy->buttons.size() > 1 && joy->buttons[1] == 1)  // circle
    first_mode = "move";
  if (joy->buttons.size() > 2 && joy->buttons[2] == 1)  // triangle
    second_mode = "auto";
  if (joy->buttons.size() > 3 && joy->buttons[3] == 1)  // square
    second_mode = "manual";

  return std::make_pair(first_mode, second_mode);
}

// ===== メイン周期処理（ウォッチドッグ＆出力） =====
void MotionDecision::on_timer()
{
  // /local_path/cmd_vel ウォッチドッグ
  if (mode_.first == "move" && mode_.second == "auto") {
    if (!flags_.local_path_updated) {
      counters_.not_received_local_path++;
      if (counters_.not_received_local_path >
          params_.allowable_num_of_not_received_local_path) {
        cmd_vel_ = geometry_msgs::msg::Twist();  // 許容超え→停止
  	std::cout << "cmd vel_zero :" << std::endl;
  	std::cout << "  linear.x  : " << cmd_vel_.linear.x << std::endl;
  	std::cout << "  angular.z : " << cmd_vel_.angular.z << std::endl;
      }
    } else {
      counters_.not_received_local_path = 0;
    }
  } else {
    counters_.not_received_local_path = 0; // auto以外は監視リセット
  }

  std::cout << "cmd vel_ontimer :" << std::endl;
  std::cout << "  linear.x  : " << cmd_vel_.linear.x << std::endl;
  std::cout << "  angular.z : " << cmd_vel_.angular.z << std::endl;
  // ウォッチドッグ初期化（この周期で更新が無ければ欠測）
  flags_.local_path_updated = false;
  publish_cmd_vel(cmd_vel_);
}

void MotionDecision::publish_cmd_vel(geometry_msgs::msg::Twist cmd_vel)
{
  // 速度クリップ（manual/auto 共通）
  if (flags_.turbo_trigger)
  {
    cmd_vel.linear.x = (cmd_vel.linear.x >= 0.0)
                         ? std::min(cmd_vel.linear.x,  static_cast<double>(params_.turbo_max_velocity))
                         : std::max(cmd_vel.linear.x, -static_cast<double>(params_.turbo_max_velocity));
    cmd_vel.angular.z = (cmd_vel.angular.z >= 0.0)
                         ? std::min(cmd_vel.angular.z,  static_cast<double>(params_.max_yawrate))
                         : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));
  }
  else
  {
    cmd_vel.linear.x = (cmd_vel.linear.x >= 0.0)
                         ? std::min(cmd_vel.linear.x,  static_cast<double>(params_.max_velocity))
                         : std::max(cmd_vel.linear.x, -static_cast<double>(params_.max_velocity));
    cmd_vel.angular.z = (cmd_vel.angular.z >= 0.0)
                         ? std::min(cmd_vel.angular.z,  static_cast<double>(params_.max_yawrate))
                         : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));
  }

  if (mode_.first == "stop")
    cmd_vel = geometry_msgs::msg::Twist();

  cmd_vel_pub_->publish(cmd_vel);
  print_status(cmd_vel);  // 既存の表示関数を流用
}

void MotionDecision::print_mode_status(const std::string &status, std::string color)
{
  if (color == "") color = RESET_COLOR;
  const std::string parrentheses = status == "" ? ")" : ") ";
  std::cout << "=== " << mode_.second << " (" << mode_.first << parrentheses
            << color << status << RESET_COLOR << " ===" << std::endl;
}

void MotionDecision::print_status(const geometry_msgs::msg::Twist &cmd_vel)
{
  // manual時の簡易表示のみ
  if (mode_.first == "move" && mode_.second == "manual")
  {
    if (flags_.turbo_trigger)
      print_mode_status("TURBO", CYAN);
    else if (flags_.move_trigger)
      print_mode_status("UNLOCKED", GREEN);
    else
      print_mode_status("LOCKED", RED);
  }
  else
  {
    print_mode_status();
  }

  if (mode_.first == "move")
  {
    std::cout << "cmd vel :" << std::endl;
    std::cout << "  linear.x  : " << cmd_vel.linear.x << std::endl;
    std::cout << "  angular.z : " << cmd_vel.angular.z << std::endl;
  }
  std::cout << std::endl;
}


// ===== main =====
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionDecision>();
  rclcpp::spin(node);  // タイマー＆サブスクを回す
  rclcpp::shutdown();
  return 0;
}
