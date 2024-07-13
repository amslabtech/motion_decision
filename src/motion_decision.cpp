/**
 * @file motion_decision.cpp
 * @author amsl
 * @brief C++ implementation of motion decision
 * @copyright Copyright (c) 2024
 */

#include <string>

#include "motion_decision/motion_decision.h"

MotionDecision::MotionDecision(void) : private_nh_("~")
{
  intersection_flag_pub_ = nh_.advertise<std_msgs::Bool>("/intersection_flag", 1, true);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

  emergency_stop_flag_sub_ = nh_.subscribe("/emergency_stop", 1, &MotionDecision::emergency_stop_flag_callback, this);
  front_laser_sub_ = nh_.subscribe("/front_laser/scan", 1, &MotionDecision::front_laser_callback, this);
  joy_sub_ = nh_.subscribe("/joy", 1, &MotionDecision::joy_callback, this);
  local_path_velocity_sub_ =
      nh_.subscribe("/local_path/cmd_vel", 1, &MotionDecision::local_path_velocity_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &MotionDecision::odom_callback, this);
  rear_laser_sub_ = nh_.subscribe("/rear_laser/scan", 1, &MotionDecision::rear_laser_callback, this);
  recovery_mode_flag_sub_ = nh_.subscribe("/recovery_mode_flag", 1, &MotionDecision::recovery_mode_flag_callback, this);
  task_stop_flag_sub_ = nh_.subscribe("/task/stop", 1, &MotionDecision::task_stop_flag_callback, this);

  load_params();
}

void MotionDecision::load_params(void)
{
  private_nh_.param<int>("hz", params_.hz, 20);
  private_nh_.param<int>("recovery_mode_threshold", params_.recovery_mode_threshold, 60);
  private_nh_.param<int>("trigger_count_threshold", params_.trigger_count_threshold, 2);
  private_nh_.param<double>("max_speed", params_.max_speed, 1.0);
  private_nh_.param<double>("max_yawrate", params_.max_yawrate, 1.0);
  private_nh_.param<double>("dt", params_.dt, 0.1);
  private_nh_.param<double>("predict_time", params_.predict_time, 1.0);
  private_nh_.param<double>("collision_distance", params_.collision_distance, 0.4);
  private_nh_.param<double>("safety_collision_time", params_.safety_collision_time, 0.5);
  private_nh_.param<std::string>("stop_sound_path", params_.stop_sound_path, std::string(""));
  private_nh_.param<std::string>("recovery_sound_path", params_.recovery_sound_path, std::string(""));
  private_nh_.param<std::string>("task_stop_sound_path", params_.task_stop_sound_path, std::string(""));
}

void MotionDecision::emergency_stop_flag_callback(const std_msgs::BoolConstPtr &msg)
{
  flags_.emergency_stop = msg->data;
}

void MotionDecision::front_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  front_laser_ = *msg;
  search_min_range(front_laser_, laser_info_.front_min_idx, laser_info_.front_min_range);
  flags_.front_laser_received = true;
}

void MotionDecision::joy_callback(const sensor_msgs::JoyConstPtr &msg)
{
  mode_ = select_mode(msg, mode_);
  if (mode_.second == "manual")
  {
    counters_.stuck = 0;
    counters_.trigger = 0;
  }
  if (mode_.first == "move" && mode_.second == "manual" && msg->buttons[4])
  {
    cmd_vel_.linear.x = msg->axes[1] * params_.max_speed;
    cmd_vel_.angular.z = msg->axes[0] * params_.max_yawrate;
  }
  if(msg->buttons[11] && msg->buttons[12])
  {
    std_msgs::Bool flag;
    flag.data = true;
    intersection_flag_pub_.publish(flag);
    std::cout << "=========intersection=============" << std::endl;
  }

    // if (vel.linear.x > params_.max_speed)
    // {
    //   vel.linear.x = params_.max_speed;
    // }
    // else if (vel.linear.x < -params_.max_speed)
    // {
    //   vel.linear.x = params_.max_speed;
    // }
    // if (vel.angular.z > params_.max_yawrate)
    // {
    //   vel.angular.z = params_.max_yawrate;
    // }
    // else if (vel.angular.z < -params_.max_yawrate)
    // {
    //   vel.angular.z = -params_.max_yawrate;
    // }

}

void MotionDecision::local_path_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  if (mode_.first == "move" && mode_.second == "auto")
    cmd_vel_ = *msg;
  flags_.local_path_received = true;
}

void MotionDecision::odom_callback(const nav_msgs::OdometryConstPtr &msg) { odom_vel_ = msg->twist.twist; }

void MotionDecision::rear_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  rear_laser_ = *msg;
  search_min_range(rear_laser_, laser_info_.rear_min_idx, laser_info_.rear_min_range);
  flags_.rear_laser_received = true;
}

void MotionDecision::recovery_mode_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
  flags_.enable_recovery_mode = msg->data;
}

void MotionDecision::task_stop_flag_callback(const std_msgs::BoolConstPtr &msg)
{
  std_msgs::Bool flag = *msg;
  if (flag.data)
  {
    std::cout << "========= task stop =========" << std::endl;
    if (params_.task_stop_sound_path != "")
    {
      std::string sound_command = "aplay " + params_.task_stop_sound_path + " &";
      system(sound_command.c_str());
      system(sound_command.c_str());
    }
    flags_.move_mode = false;
  }
  else
  {
    flags_.move_mode = true;
  }
}

void MotionDecision::search_min_range(const sensor_msgs::LaserScan &laser, int &min_idx, float &min_range)
{
  min_range = laser.range_max;
  min_idx = 0;
  for (size_t i = 0; i < laser.ranges.size(); i++)
  {
    if (laser.ranges[i] < min_range)
    {
      min_range = laser.ranges[i];
      min_idx = i;
    }
  }
}

std::pair<std::string, std::string>
MotionDecision::select_mode(const sensor_msgs::Joy::ConstPtr &joy, const std::pair<std::string, std::string> &prev_mode)
{
  std::string first_mode = prev_mode.first;
  std::string second_mode = prev_mode.second;

  if (joy->buttons[0] == 1)  // cross button
    first_mode = "stop";
  if (joy->buttons[1] == 1)  // circle button
    first_mode = "move";
  if (joy->buttons[2] == 1)  // triangle button
    second_mode = "auto";
  if (joy->buttons[3] == 1)  // square button
    second_mode = "manual";

  return std::make_pair(first_mode, second_mode);
}

void MotionDecision::process(void)
{
  ros::Rate loop_rate(params_.hz);
  while (ros::ok())
  {
    // Detect stuck and recover for a certain period of time
    if (mode_.first == "move" && mode_.second == "auto" && flags_.enable_recovery_mode)
    {
      if (0 < counters_.trigger && counters_.trigger < params_.trigger_count_threshold)
      {
        recovery_mode(cmd_vel_);
      }
      else if (((cmd_vel_.linear.x < DBL_EPSILON && fabs(cmd_vel_.angular.z) < DBL_EPSILON) ||
       (odom_vel_.linear.x < 0.01 && fabs(odom_vel_.angular.z) < 0.01)))
      {
        if (counters_.stuck < params_.recovery_mode_threshold)
          counters_.stuck++;
        else
          counters_.trigger++;
      }
      else
      {
        // reset counters when robot moves
        counters_.stuck = 0;
        counters_.trigger = 0;
      }

      if (!flags_.front_laser_received || !flags_.rear_laser_received)
        cmd_vel_ = geometry_msgs::Twist();

      flags_.front_laser_received = false;
      flags_.rear_laser_received = false;
    }

    if (flags_.emergency_stop)
      cmd_vel_ = geometry_msgs::Twist();
    velocity_pub_.publish(cmd_vel_);
    print_status(cmd_vel_);

    laser_info_.front_min_range = -1.0;
    laser_info_.rear_min_range = -1.0;

    loop_rate.sleep();
    ros::spinOnce();
  }
}

void MotionDecision::recovery_mode(geometry_msgs::Twist &cmd_vel)
{
  std::cout << "#####################" << std::endl;
  std::cout << "### recovery mode ###" << std::endl;
  std::cout << "#####################" << std::endl;

  // variables for recovery mode
  double max_velocity_limit = 0.3;
  double max_yawrate_limit = 0.3;
  double velocity_resolution = 0.1;
  double yawrate_resolution = 0.1;
  double max_velocity = 0.0;
  double max_yawrate = 0.0;
  double max_ttc = 0.0;
  bool reverse_flag = false;
  if (0.0 <= cmd_vel.linear.x)
  {
    // when moving forwards
    // calculate ttc when if go backwards
    for (double velocity = -velocity_resolution; velocity >= -max_velocity_limit; velocity -= velocity_resolution)
    {
      for (double yawrate = -max_yawrate_limit; yawrate <= max_yawrate_limit; yawrate += yawrate_resolution)
      {
        geometry_msgs::Twist vel;
        vel.linear.x = velocity;
        vel.angular.z = yawrate;
        // carefull to modify param true
        // go_back variable is false but robot moving backwards virtuary so needed laser data is rear's
        double ttc = calc_ttc(vel);
        if (ttc > max_ttc)
        {
          max_velocity = velocity;
          max_yawrate = yawrate;
          max_ttc = ttc;
        }
        else if (ttc == max_ttc)
        {
          double max_x = max_velocity / max_yawrate * sin(max_yawrate * ttc);
          double max_y = max_velocity / max_y * (1 - cos(max_y * ttc));
          double x = velocity / yawrate * sin(yawrate * ttc);
          double y = velocity / y * (1 - cos(y * ttc));
          double angle = (2.0 * laser_info_.front_min_idx / front_laser_.ranges.size() - 1.0) * (front_laser_.angle_max);
          double angle_diff_a = fabs(angle - atan2(max_y, max_x));
          double angle_diff_b = fabs(angle - atan2(y, x));
          if (M_PI / 2.0 <= fabs(angle) && angle_diff_a < angle_diff_b && calc_ttc(vel) > params_.safety_collision_time)
          {
            max_velocity = velocity;
            max_y = y;
            max_ttc = ttc;
            reverse_flag = true;
          }
          else if (!reverse_flag && angle_diff_a > angle_diff_b)
          {
            max_velocity = velocity;
            max_yawrate = yawrate;
            max_ttc = ttc;
          }
        }
      }
    }
    if (max_ttc > params_.safety_collision_time)
    {
      cmd_vel.linear.x = max_velocity;
      cmd_vel.angular.z = max_yawrate;
    }
    else
    {
      // set vel to move away from obstacles
      if (laser_info_.front_min_idx < front_laser_.ranges.size() * 0.5)
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.2;
      }
      else
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = -0.2;
      }
    }
  }
  else
  {
    // when moving backwards
    // calculate ttc when if go forwards
    for (double velocity = velocity_resolution; velocity <= max_velocity_limit; velocity += velocity_resolution)
    {
      for (double yawrate = -max_yawrate_limit; yawrate <= max_yawrate_limit; yawrate += yawrate_resolution)
      {
        geometry_msgs::Twist vel;
        vel.linear.x = velocity;
        vel.angular.z = yawrate;
        // carefull to modify param true
        // go_back variable is true but robot moving forwards virtuary so needed laser data is front's
        double ttc = calc_ttc(vel);
        if (ttc > max_ttc)
        {
          max_velocity = velocity;
          max_yawrate = yawrate;
          max_ttc = ttc;
        }
        else if (ttc == max_ttc)
        {
          double max_x = max_velocity / max_yawrate * sin(max_yawrate * ttc);
          double max_y = max_velocity / max_y * (1 - cos(max_y * ttc));
          double x = velocity / yawrate * sin(yawrate * ttc);
          double y = velocity / yawrate * (1 - cos(yawrate * ttc));
          double angle = (2.0 * laser_info_.rear_min_idx / rear_laser_.ranges.size() - 1.0) * (rear_laser_.angle_max);
          double angle_diff_a = fabs(angle - atan2(max_y, max_x));
          double angle_diff_b = fabs(angle - atan2(y, x));
          if (angle_diff_a > angle_diff_b)
          {
            max_velocity = velocity;
            max_yawrate = yawrate;
            max_ttc = ttc;
          }
        }
      }
    }
    if (max_ttc > params_.safety_collision_time)
    {
      cmd_vel.linear.x = max_velocity;
      cmd_vel.angular.z = max_yawrate;
    }
    else
    {
      // set vel to move away from obstacles
      if (laser_info_.rear_min_idx > rear_laser_.ranges.size() * 0.5)
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = -0.2;
      }
      else
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.2;
      }
    }
  }

  counters_.trigger++;
}

float MotionDecision::calc_ttc(geometry_msgs::Twist vel)
{
  // select laser data by direction of motion
  sensor_msgs::LaserScan laser;
  if (0.0 <= vel.linear.x)
  {
    laser = front_laser_;
  }
  else
  {
    laser = rear_laser_;
  }

  // calculate TTC
  double ttc = params_.predict_time;
  int i = 0;
  for (auto range : laser.ranges)
  {
    // ignore invalid laser data
    if (range < 0.1)
    {
      continue;
    }
    double x = 0;
    double y = 0;
    double yaw = 0;
    double obs_x, obs_y, angle;
    angle = (2.0 * i / laser.ranges.size() - 1.0) * (laser.angle_max);
    obs_x = range * cos(angle);
    obs_y = range * sin(angle);

    for (double time = params_.dt; time < params_.predict_time; time += params_.dt)
    {
      yaw += vel.angular.z * params_.dt;
      x += fabs(vel.linear.x) * cos(yaw) * params_.dt;

      y += fabs(vel.linear.x) * sin(yaw) * params_.dt;
      double tmp_range = sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
      if (tmp_range < params_.collision_distance)
      {
        if (time < ttc)
        {
          ttc = time;
          break;
        }
      }
    }
    i++;
  }
  return ttc;
}

void MotionDecision::print_status(const geometry_msgs::Twist &cmd_vel)
{
  std::cout << "=== " << mode_.second << " (" << mode_.first << ") ===" << std::endl;
  if (flags_.emergency_stop)
    std::cout << "emergency stop" << std::endl;
  std::cout << "min front laser : " << laser_info_.front_min_range << std::endl;
  std::cout << "min rear laser  : " << laser_info_.rear_min_range << std::endl;
  std::cout << "trigger count   : " << counters_.trigger << std::endl;
  if (0 < counters_.stuck)
    std::cout << "stuck count : " << counters_.stuck << std::endl;
  if (mode_.first == "move")
  {
    std::cout << "cmd vel :" << std::endl;
    std::cout << "  linear.x  : " << cmd_vel.linear.x << std::endl;
    std::cout << "  angular.z : " << cmd_vel.angular.z << std::endl;
  }
  std::cout << std::endl;
}

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_decision");
  MotionDecision motion_decision;
  motion_decision.process();

  return 0;
}
