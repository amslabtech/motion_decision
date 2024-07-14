/**
 * @file motion_decision.cpp
 * @author amsl
 * @brief C++ implementation of motion decision
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <string>
#include <utility>

#include "motion_decision/motion_decision.h"

MotionDecision::MotionDecision(void) : private_nh_("~")
{
  intersection_flag_pub_ = nh_.advertise<std_msgs::Bool>("/intersection_flag", 1, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

  emergency_stop_flag_sub_ = nh_.subscribe("/emergency_stop", 1, &MotionDecision::emergency_stop_flag_callback, this);
  front_laser_sub_ = nh_.subscribe("/front_laser/scan", 1, &MotionDecision::front_laser_callback, this);
  joy_sub_ = nh_.subscribe("/joy", 1, &MotionDecision::joy_callback, this);
  local_path_cmd_vel_sub_ =
      nh_.subscribe("/local_path/cmd_vel", 1, &MotionDecision::local_path_cmd_vel_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &MotionDecision::odom_callback, this);
  rear_laser_sub_ = nh_.subscribe("/rear_laser/scan", 1, &MotionDecision::rear_laser_callback, this);
  recovery_mode_flag_sub_ = nh_.subscribe("/recovery_mode_flag", 1, &MotionDecision::recovery_mode_flag_callback, this);
  task_stop_flag_sub_ = nh_.subscribe("/task/stop", 1, &MotionDecision::task_stop_flag_callback, this);

  load_params();
}

void MotionDecision::load_params(void)
{
  // MotionDecisionParams
  private_nh_.param<int>("hz", params_.hz, 20);
  private_nh_.param<float>("max_velocity", params_.max_velocity, 1.0);
  private_nh_.param<float>("max_yawrate", params_.max_yawrate, 1.0);
  private_nh_.param<float>("dt", params_.dt, 0.1);
  private_nh_.param<float>("predict_time", params_.predict_time, 1.0);
  private_nh_.param<float>("collision_distance", params_.collision_distance, 0.4);
  private_nh_.param<float>("safety_collision_time", params_.safety_collision_time, 0.5);
  private_nh_.param<float>("stuck_time_threshold", params_.stuck_time_threshold, 1.0);
  private_nh_.param<std::string>("stop_sound_path", params_.stop_sound_path, std::string(""));
  private_nh_.param<std::string>("recovery_sound_path", params_.recovery_sound_path, std::string(""));
  private_nh_.param<std::string>("task_stop_sound_path", params_.task_stop_sound_path, std::string(""));

  // RecoveryParams
  private_nh_.param<bool>("recovery/use", params_of_recovery_.use, true);
  private_nh_.param<float>("recovery/max_velocity", params_of_recovery_.max_velocity, 0.3);
  private_nh_.param<float>("recovery/max_yawrate", params_of_recovery_.max_yawrate, 0.3);
  private_nh_.param<float>("recovery/velocity_resolution", params_of_recovery_.velocity_resolution, 0.1);
  private_nh_.param<float>("recovery/yawrate_resolution", params_of_recovery_.yawrate_resolution, 0.1);
  private_nh_.param<float>("recovery/time", params_of_recovery_.time, 3.0);
}

void MotionDecision::front_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  front_laser_ = *msg;
  search_min_range(front_laser_.value(), laser_info_.front_min_range, laser_info_.front_index_of_min_range);
}

void MotionDecision::joy_callback(const sensor_msgs::JoyConstPtr &msg)
{
  mode_ = select_mode(msg, mode_);

  if (mode_.first == "move" && mode_.second == "manual")
  {
    cmd_vel_.linear.x = msg->buttons[4] ? msg->axes[1] * params_.max_velocity : 0.0;
    cmd_vel_.angular.z = msg->buttons[4] ? msg->axes[0] * params_.max_yawrate : 0.0;
  }
  else if (mode_.first == "move" && mode_.second == "auto" && !flags_.local_path_received)
  {
    cmd_vel_ = geometry_msgs::Twist();
  }

  if (msg->buttons[11] && msg->buttons[12])
  {
    std_msgs::Bool flag;
    flag.data = true;
    flags_.intersection = true;
    intersection_flag_pub_.publish(flag);
  }
  else
  {
    flags_.intersection = false;
  }
}

void MotionDecision::local_path_cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
{
  if (mode_.first == "move" && mode_.second == "auto")
    cmd_vel_ = *msg;
  flags_.local_path_received = true;
}

void MotionDecision::rear_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  rear_laser_ = *msg;
  search_min_range(rear_laser_.value(), laser_info_.rear_min_range, laser_info_.rear_index_of_min_range);
}

void MotionDecision::task_stop_flag_callback(const std_msgs::BoolConstPtr &msg)
{
  flags_.task_stop = msg->data;
  if (flags_.task_stop)
  {
    mode_.first = "stop";
    sound(params_.task_stop_sound_path);
  }
  else
  {
    mode_.first = "move";
  }
}

void MotionDecision::search_min_range(const sensor_msgs::LaserScan &laser, float &min_range, int &index_of_min_range)
{
  min_range = laser.range_max;
  index_of_min_range = 0;
  for (size_t i = 0; i < laser.ranges.size(); i++)
  {
    if (laser.ranges[i] < min_range)
    {
      min_range = laser.ranges[i];
      index_of_min_range = i;
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

void MotionDecision::sound(const std::string &path)
{
  if (path == "")
    return;

  const std::string sound_command = "aplay " + params_.task_stop_sound_path + " &";
  if (system(sound_command.c_str()) == -1)
    ROS_WARN("Failed to play sound");
}

void MotionDecision::process(void)
{
  ros::Rate loop_rate(params_.hz);
  const int max_recovery_count = static_cast<int>(params_of_recovery_.time * params_.hz);
  const int max_stuck_count = static_cast<int>(params_.stuck_time_threshold * params_.hz);
  while (ros::ok())
  {
    // Detect stuck and recover for a certain period of time
    if (mode_.first == "move" && mode_.second == "auto" && params_of_recovery_.use)
    {
      if (0 < counters_.recovery && counters_.recovery < max_recovery_count)
      {
        recovery_mode(cmd_vel_);
      }
      else if (max_recovery_count <= counters_.recovery)
      {
        counters_.stuck = 0;
        counters_.recovery = 0;
        if (!flags_.local_path_received)
          cmd_vel_ = geometry_msgs::Twist();
      }
      else if (((cmd_vel_.linear.x < DBL_EPSILON && fabs(cmd_vel_.angular.z) < DBL_EPSILON) ||
                (odom_vel_.linear.x < 0.01 && fabs(odom_vel_.angular.z) < 0.01)))
      {
        if (counters_.stuck < max_stuck_count)
        {
          counters_.stuck++;
        }
        else
        {
          counters_.recovery++;
          sound(params_.recovery_sound_path);
        }
      }
    }
    else if (mode_.second == "manual")
    {
      counters_.stuck = 0;
      counters_.recovery = 0;
    }

    publish_cmd_vel(cmd_vel_);

    front_laser_.reset();
    rear_laser_.reset();
    laser_info_.front_min_range = -1.0;
    laser_info_.rear_min_range = -1.0;
    flags_.local_path_received = false;

    loop_rate.sleep();
    ros::spinOnce();
  }
}

void MotionDecision::recovery_mode(geometry_msgs::Twist &cmd_vel)
{
  counters_.recovery++;
  if (!front_laser_.has_value() || !rear_laser_.has_value())
    return;

  // select data by direction of motion
  const bool sim_back = 0.0 < cmd_vel.linear.x;
  const sensor_msgs::LaserScan laser = sim_back ? front_laser_.value() : rear_laser_.value();
  const int index_of_min_range = sim_back ? laser_info_.front_index_of_min_range : laser_info_.rear_index_of_min_range;
  const float max_velocity = sim_back ? -params_of_recovery_.max_velocity : params_of_recovery_.max_velocity;
  const float velocity_resolution =
      sim_back ? -params_of_recovery_.velocity_resolution : params_of_recovery_.velocity_resolution;

  // set cmd_vel to move away from the nearest obstacle
  float max_ttc = 0.0;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = index_of_min_range < laser.ranges.size() / 2 ? 0.2 : -0.2;
  cmd_vel.linear.z *= sim_back ? 1.0 : -1.0;
  const float obs_angle = laser.angle_min + index_of_min_range * laser.angle_increment;
  const bool obs_found_in_direction_of_travel = obs_angle < M_PI / 2.0;
  for (float velocity = velocity_resolution; fabs(velocity) <= fabs(max_velocity); velocity += velocity_resolution)
  {
    for (float yawrate = -params_of_recovery_.max_yawrate; yawrate <= params_of_recovery_.max_yawrate;
         yawrate += params_of_recovery_.yawrate_resolution)
    {
      const float ttc = calc_ttc(velocity, yawrate, laser);
      if (ttc > max_ttc && ttc > params_.safety_collision_time)
      {
        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = yawrate;
        max_ttc = ttc;
      }
      else if (ttc == max_ttc && ttc > params_.safety_collision_time)
      {
        const std::pair<float, float> best_sim_pos =
            sim_by_uniform_circluar_motion(cmd_vel.linear.x, cmd_vel.angular.z, ttc);
        const std::pair<float, float> tmp_sim_pos = sim_by_uniform_circluar_motion(velocity, yawrate, ttc);
        const float angle_between_best_sim_pos_and_obs =
            fabs(obs_angle - atan2(best_sim_pos.second, best_sim_pos.first));
        const float angle_between_tmp_sim_pos_and_obs = fabs(obs_angle - atan2(tmp_sim_pos.second, tmp_sim_pos.first));

        // if the angle between the obstacle and the robot is greater than 90 degrees, the robot will move backwards
        if (obs_found_in_direction_of_travel &&
                angle_between_best_sim_pos_and_obs > angle_between_tmp_sim_pos_and_obs ||
            !obs_found_in_direction_of_travel && angle_between_best_sim_pos_and_obs < angle_between_tmp_sim_pos_and_obs)
        {
          cmd_vel.linear.x = velocity;
          cmd_vel.angular.z = yawrate;
          max_ttc = ttc;
        }
      }
    }
  }
}

float MotionDecision::calc_ttc(const float &velocity, const float &yawrate, const sensor_msgs::LaserScan &laser)
{
  // calculate TTC (Time To Collision)
  float ttc = params_.predict_time;
  for (size_t i = 0; i < laser.ranges.size(); i++)
  {
    const float angle = (2.0 * i / laser.ranges.size() - 1.0) * laser.angle_max;
    const float obs_x = laser.ranges[i] * cos(angle);
    const float obs_y = laser.ranges[i] * sin(angle);

    // simulate motion by discrete-time model
    float x(0.0), y(0.0), yaw(0.0);
    for (float time = params_.dt; time < params_.predict_time; time += params_.dt)
    {
      yaw += yawrate * params_.dt;
      x += fabs(velocity) * cos(yaw) * params_.dt;
      y += fabs(velocity) * sin(yaw) * params_.dt;
      if (hypot(x - obs_x, y - obs_y) < params_.collision_distance && time < ttc)
      {
        ttc = time;
        break;
      }
    }
  }

  return ttc;
}

std::pair<float, float>
MotionDecision::sim_by_uniform_circluar_motion(const float &velocity, const float &yawrate, const float &sim_time)
{
  const float radius = velocity / yawrate;
  const float theta = yawrate * sim_time;
  const float predict_x = radius * sin(theta);
  const float predict_y = radius - radius * cos(theta);

  return std::make_pair(predict_x, predict_y);
}

void MotionDecision::publish_cmd_vel(geometry_msgs::Twist cmd_vel)
{
  cmd_vel.linear.x = 0.0 < cmd_vel.linear.x ? std::min(cmd_vel.linear.x, static_cast<double>(params_.max_velocity))
                                            : std::max(cmd_vel.linear.x, static_cast<double>(-params_.max_velocity));
  cmd_vel.angular.z = 0.0 < cmd_vel.angular.z ? std::min(cmd_vel.angular.z, static_cast<double>(params_.max_yawrate))
                                              : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));

  if (flags_.emergency_stop || mode_.first == "stop")
    cmd_vel = geometry_msgs::Twist();
  if (mode_.first == "move" && mode_.second == "auto" && (!front_laser_.has_value() || !rear_laser_.has_value()))
    cmd_vel = geometry_msgs::Twist();

  cmd_vel_pub_.publish(cmd_vel);
  print_status(cmd_vel);
}

void MotionDecision::print_status(const geometry_msgs::Twist &cmd_vel)
{
  std::cout << "=== " << mode_.second << " (" << mode_.first << ") ===" << std::endl;
  if (0 < counters_.recovery)
  {
    std::cout << "#####################" << std::endl;
    std::cout << "### recovery mode ###" << std::endl;
    std::cout << "#####################" << std::endl;
  }
  else if (0 < counters_.stuck)
  {
    std::cout << "################" << std::endl;
    std::cout << "### stuck!!! ###" << std::endl;
    std::cout << "################" << std::endl;
  }
  if (flags_.emergency_stop)
  {
    std::cout << "######################" << std::endl;
    std::cout << "### emergency stop ###" << std::endl;
    std::cout << "######################" << std::endl;
  }
  if (flags_.task_stop)
  {
    std::cout << "#################" << std::endl;
    std::cout << "### task stop ###" << std::endl;
    std::cout << "#################" << std::endl;
  }
  if (flags_.intersection)
  {
    std::cout << "####################" << std::endl;
    std::cout << "### intersection ###" << std::endl;
    std::cout << "####################" << std::endl;
  }
  std::cout << "min front laser : " << laser_info_.front_min_range << std::endl;
  std::cout << "min rear laser  : " << laser_info_.rear_min_range << std::endl;
  std::cout << "recovery time   : " << counters_.recovery / static_cast<float>(params_.hz) << std::endl;
  if (0 < counters_.stuck)
    std::cout << "stuck time      : " << counters_.stuck / static_cast<float>(params_.hz) << std::endl;
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
