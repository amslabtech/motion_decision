/**
 * @file motion_decision.cpp
 * @author amsl
 * @brief C++ implementation of motion decision
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "motion_decision/motion_decision.h"

MotionDecision::MotionDecision(void) : private_nh_("~")
{
  intersection_flag_pub_ = nh_.advertise<std_msgs::Bool>("/intersection_flag", 1, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

  emergency_stop_flag_sub_ = nh_.subscribe("/emergency_stop", 1, &MotionDecision::emergency_stop_flag_callback, this);
  front_laser_sub_ = nh_.subscribe("/front_laser/scan", 1, &MotionDecision::front_laser_callback, this);
  joy_sub_ = nh_.subscribe("/joy", 1, &MotionDecision::joy_callback, this);
  local_path_cmd_vel_sub_ = nh_.subscribe("/local_path/cmd_vel", 1, &MotionDecision::local_path_cmd_vel_callback, this);
  local_map_sub_ = nh_.subscribe("/local_map", 1, &MotionDecision::local_map_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &MotionDecision::odom_callback, this);
  rear_laser_sub_ = nh_.subscribe("/rear_laser/scan", 1, &MotionDecision::rear_laser_callback, this);

  recovery_mode_flag_server_ =
      nh_.advertiseService("/recovery/available", &MotionDecision::recovery_mode_flag_callback, this);
  task_stop_flag_server_ = nh_.advertiseService("/task/stop", &MotionDecision::task_stop_flag_callback, this);

  load_params();
  if (params_.use_360_laser || params_.use_local_map)
    params_.use_rear_laser = true;
}

void MotionDecision::load_params(void)
{
  // MotionDecisionParams
  private_nh_.param<bool>("use_rear_laser", params_.use_rear_laser, true);
  private_nh_.param<bool>("use_360_laser", params_.use_360_laser, false);
  private_nh_.param<bool>("use_local_map", params_.use_local_map, false);
  private_nh_.param<int>("hz", params_.hz, 20);
  private_nh_.param<int>("allowable_num_of_not_received", params_.allowable_num_of_not_received, 3);
  private_nh_.param<float>("max_velocity", params_.max_velocity, 1.0);
  private_nh_.param<float>("turbo_max_velocity", params_.turbo_max_velocity, 1.66);
  private_nh_.param<float>("max_yawrate", params_.max_yawrate, 1.0);
  private_nh_.param<float>("dt", params_.dt, 0.1);
  private_nh_.param<float>("predict_time", params_.predict_time, 1.0);
  private_nh_.param<float>("collision_distance", params_.collision_distance, 0.4);
  private_nh_.param<float>("safety_collision_time", params_.safety_collision_time, 0.5);
  private_nh_.param<float>("stuck_time_threshold", params_.stuck_time_threshold, 1.0);
  private_nh_.param<float>("angle_increment", params_.angle_increment, 0.1);
  private_nh_.param<std::string>("stop_sound_path", params_.stop_sound_path, std::string(""));
  private_nh_.param<std::string>("recovery_sound_path", params_.recovery_sound_path, std::string(""));
  private_nh_.param<std::string>("task_stop_sound_path", params_.task_stop_sound_path, std::string(""));

  // RecoveryParams
  private_nh_.param<bool>("recovery/available", params_of_recovery_.available, true);
  private_nh_.param<bool>("recovery/sim_back", params_of_recovery_.sim_back, true);
  private_nh_.param<float>("recovery/max_velocity", params_of_recovery_.max_velocity, 0.3);
  private_nh_.param<float>("recovery/max_yawrate", params_of_recovery_.max_yawrate, 0.3);
  private_nh_.param<float>("recovery/velocity_resolution", params_of_recovery_.velocity_resolution, 0.1);
  private_nh_.param<float>("recovery/yawrate_resolution", params_of_recovery_.yawrate_resolution, 0.1);
  private_nh_.param<float>("recovery/spin_turn_speed", params_of_recovery_.spin_turn_speed, 0.2);
  private_nh_.param<float>("recovery/time", params_of_recovery_.time, 3.0);
}

void MotionDecision::front_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  if (params_.use_local_map)
    return;

  if (!params_.use_360_laser)
    front_laser_ = *msg;
  else
    front_laser_ = create_laser_from_360_laser(*msg, "front");
  search_min_range(front_laser_.value(), laser_info_.front_min_range, laser_info_.front_index_of_min_range);
  flags_.front_laser_updated = true;
  counters_.not_received_front_laser = 0;

  if (!params_.use_360_laser)
    return;
  rear_laser_ = create_laser_from_360_laser(*msg, "rear");
  search_min_range(rear_laser_.value(), laser_info_.rear_min_range, laser_info_.rear_index_of_min_range);
  flags_.rear_laser_updated = true;
  counters_.not_received_rear_laser = 0;
}

void MotionDecision::joy_callback(const sensor_msgs::JoyConstPtr &msg)
{
  mode_ = select_mode(msg, mode_);

  if (mode_.first == "move" && mode_.second == "manual")
  {
    flags_.move_trigger = msg->buttons[4];
    flags_.turbo_trigger = (flags_.move_trigger && msg->axes[2] == -1.0);
    if (flags_.turbo_trigger)
    {
      cmd_vel_.linear.x = flags_.move_trigger ? msg->axes[1] * params_.turbo_max_velocity : 0.0;
      cmd_vel_.angular.z = flags_.move_trigger ? msg->axes[0] * params_.max_yawrate : 0.0;
    }
    else
    {
      cmd_vel_.linear.x = flags_.move_trigger ? msg->axes[1] * params_.max_velocity : 0.0;
      cmd_vel_.angular.z = flags_.move_trigger ? msg->axes[0] * params_.max_yawrate : 0.0;
    }
  }
  else if (mode_.first == "move" && mode_.second == "auto" && !flags_.local_path_updated)
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

  if (mode_.first == "move")
    flags_.task_stop = false;
}

void MotionDecision::local_path_cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
{
  if (mode_.first == "move" && mode_.second == "auto")
    cmd_vel_ = *msg;
  flags_.local_path_updated = true;
}

void MotionDecision::rear_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  if (!params_.use_rear_laser || params_.use_360_laser || params_.use_local_map)
    return;

  rear_laser_ = *msg;
  search_min_range(rear_laser_.value(), laser_info_.rear_min_range, laser_info_.rear_index_of_min_range);
  flags_.rear_laser_updated = true;
  counters_.not_received_rear_laser = 0;
}

void MotionDecision::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  if (!params_.use_local_map)
    return;

  front_laser_ = create_laser_from_local_map(*msg, "front");
  search_min_range(front_laser_.value(), laser_info_.front_min_range, laser_info_.front_index_of_min_range);
  custom_laser_front_pub.publish(front_laser_.value());
  flags_.front_laser_updated = true;
  counters_.not_received_front_laser = 0;

  rear_laser_ = create_laser_from_local_map(*msg, "rear");
  search_min_range(rear_laser_.value(), laser_info_.rear_min_range, laser_info_.rear_index_of_min_range);
  custom_laser_rear_pub.publish(rear_laser_.value());
  flags_.rear_laser_updated = true;
  counters_.not_received_rear_laser = 0;
}

bool MotionDecision::recovery_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  params_of_recovery_.available = req.data;
  res.success = true;
  if (params_of_recovery_.available)
    res.message = "Recovery mode is available..";
  else
    res.message = "Recovery mode is unavailable..";
  return true;
}

bool MotionDecision::task_stop_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  flags_.task_stop = req.data;
  res.success = true;
  if (flags_.task_stop)
  {
    mode_.first = "stop";
    sound(params_.task_stop_sound_path);
    res.message = "Robot has stopped..";
  }
  else
  {
    mode_.first = "move";
    res.message = "Robot has started moving..";
  }
  return true;
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

sensor_msgs::LaserScan
MotionDecision::create_laser_from_360_laser(const sensor_msgs::LaserScan &msg, const std::string &direction)
{
  sensor_msgs::LaserScan laser = msg;
  laser.ranges = std::vector<float>();
  laser.angle_min = -M_PI / 2.0;
  laser.angle_max = M_PI / 2.0;

  if (direction == "front")
  {
    const auto start_it = msg.ranges.begin() + msg.ranges.size() / 4;
    const auto end_it = msg.ranges.begin() + 3 * msg.ranges.size() / 4;
    laser.ranges.insert(laser.ranges.end(), start_it, end_it);
  }
  else if (direction == "rear")
  {
    const auto start_it = msg.ranges.begin() + 3 * msg.ranges.size() / 4;
    const auto end_it = msg.ranges.end();
    const auto start_it2 = msg.ranges.begin();
    const auto end_it2 = msg.ranges.begin() + msg.ranges.size() / 4;
    laser.ranges.insert(laser.ranges.end(), start_it, end_it);
    laser.ranges.insert(laser.ranges.end(), start_it2, end_it2);
  }

  return laser;
}

sensor_msgs::LaserScan
MotionDecision::create_laser_from_local_map(const nav_msgs::OccupancyGrid &msg, const std::string &direction)
{
  sensor_msgs::LaserScan laser;
  laser.header = msg.header;
  laser.angle_min = -M_PI / 2.0;
  laser.angle_max = M_PI / 2.0;
  laser.angle_increment = params_.angle_increment;
  laser.time_increment = 0.0;
  laser.scan_time = 0.0;
  laser.range_min = 0.0;
  laser.range_max = 10.0;

  const float max_search_dist = hypot(msg.info.origin.position.x, msg.info.origin.position.y);
  if (direction == "front")
  {
    for (float angle = laser.angle_min; angle <= laser.angle_max; angle += laser.angle_increment)
    {
      for (float dist = 0.0; dist <= max_search_dist; dist += msg.info.resolution)
      {
        const float pos_x = dist * cos(angle);
        const float pos_y = dist * sin(angle);
        const int index_x = floor((pos_x - msg.info.origin.position.x) / msg.info.resolution);
        const int index_y = floor((pos_y - msg.info.origin.position.y) / msg.info.resolution);

        if ((0 <= index_x && index_x < msg.info.width) && (0 <= index_y && index_y < msg.info.height))
        {
          if (msg.data[index_x + index_y * msg.info.width] == 100)
          {
            laser.ranges.push_back(dist);
            break;
          }
        }
      }

      if (laser.ranges.size() != static_cast<size_t>((angle - laser.angle_min) / laser.angle_increment) + 1)
        laser.ranges.push_back(laser.range_max);
    }
  }
  else
  {
    const std::pair<float, float> angle_range = std::make_pair(M_PI / 2.0, 3 * M_PI / 2.0);
    for (float angle = angle_range.first; angle <= angle_range.second; angle += laser.angle_increment)
    {
      for (float dist = 0.0; dist <= max_search_dist; dist += msg.info.resolution)
      {
        const float pos_x = dist * cos(angle);
        const float pos_y = dist * sin(angle);
        const int index_x = floor((pos_x - msg.info.origin.position.x) / msg.info.resolution);
        const int index_y = floor((pos_y - msg.info.origin.position.y) / msg.info.resolution);

        if ((0 <= index_x && index_x < msg.info.width) && (0 <= index_y && index_y < msg.info.height))
        {
          if (msg.data[index_x + index_y * msg.info.width] == 100)
          {
            laser.ranges.push_back(dist);
            break;
          }
        }
      }

      if (laser.ranges.size() != static_cast<size_t>((angle - angle_range.first) / laser.angle_increment) + 1)
        laser.ranges.push_back(laser.range_max);
    }
  }

  return laser;
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

  const std::string sound_command = "aplay " + path + " &";
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
    // detect stuck and recover for a certain period of time
    if (mode_.first == "move" && mode_.second == "auto" && params_of_recovery_.available)
    {
      if (0 < counters_.recovery && counters_.recovery < max_recovery_count)
      {
        cmd_vel_ = recovery_mode(cmd_vel_, params_of_recovery_.sim_back);
      }
      else if (max_recovery_count <= counters_.recovery)
      {
        counters_.stuck = 0;
        counters_.recovery = 0;
        if (!flags_.local_path_updated)
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
      else
      {
        counters_.stuck = 0;
      }
    }
    else if (mode_.second == "manual" || mode_.first == "stop")
    {
      counters_.stuck = 0;
      counters_.recovery = 0;
    }

    // publish_cmd_vel(cmd_vel_);

    flags_.front_laser_updated = false;
    flags_.rear_laser_updated = false;
    flags_.local_path_updated = false;

    loop_rate.sleep();
    ros::spinOnce();

    if (!flags_.front_laser_updated)
      counters_.not_received_front_laser++;
    if (!flags_.rear_laser_updated)
      counters_.not_received_rear_laser++;
    if (params_.allowable_num_of_not_received < counters_.not_received_front_laser)
    {
      front_laser_.reset();
      laser_info_.front_min_range = -1.0;
    }
    if (params_.allowable_num_of_not_received < counters_.not_received_rear_laser)
    {
      rear_laser_.reset();
      laser_info_.rear_min_range = -1.0;
    }
  }
}

geometry_msgs::Twist MotionDecision::recovery_mode(geometry_msgs::Twist cmd_vel, const bool sim_back)
{
  counters_.recovery++;
  if (!front_laser_.has_value() || !rear_laser_.has_value())
    return geometry_msgs::Twist();

  // select data by direction of motion
  const sensor_msgs::LaserScan laser = sim_back ? front_laser_.value() : rear_laser_.value();
  const int index_of_min_range = sim_back ? laser_info_.front_index_of_min_range : laser_info_.rear_index_of_min_range;
  const float max_velocity = sim_back ? -params_of_recovery_.max_velocity : params_of_recovery_.max_velocity;
  const float velocity_resolution =
      sim_back ? -params_of_recovery_.velocity_resolution : params_of_recovery_.velocity_resolution;

  // set cmd_vel to move away from the nearest obstacle
  float max_ttc = 0.0;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = index_of_min_range < laser.ranges.size() / 2 ? params_of_recovery_.spin_turn_speed
                                                                   : -params_of_recovery_.spin_turn_speed;
  const float obs_angle = laser.angle_min + index_of_min_range * laser.angle_increment;
  const bool obs_found_in_direction_of_travel = fabs(obs_angle) < M_PI / 2.0;
  for (float velocity = velocity_resolution; fabs(velocity) <= fabs(max_velocity); velocity += velocity_resolution)
  {
    for (float yawrate = -params_of_recovery_.max_yawrate; yawrate <= params_of_recovery_.max_yawrate;
         yawrate += params_of_recovery_.yawrate_resolution)
    {
      const float ttc = sim_back ? calc_ttc(velocity, yawrate, rear_laser_.value())
                                 : calc_ttc(velocity, yawrate, front_laser_.value());
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

        // change the turning direction depending on whether there is an obstacle in the direction of travel
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

  return cmd_vel;
}

float MotionDecision::calc_ttc(const float &velocity, const float &yawrate, const sensor_msgs::LaserScan &laser)
{
  // calculate TTC (Time To Collision)
  float ttc = params_.predict_time;
  for (size_t i = 0; i < laser.ranges.size(); i++)
  {
    const float obs_angle = laser.angle_min + i * laser.angle_increment;
    const float obs_x = laser.ranges[i] * cos(obs_angle);
    const float obs_y = laser.ranges[i] * sin(obs_angle);

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
  if (flags_.turbo_trigger)
  {
    cmd_vel.linear.x = 0.0 < cmd_vel.linear.x
                           ? std::min(cmd_vel.linear.x, static_cast<double>(params_.turbo_max_velocity))
                           : std::max(cmd_vel.linear.x, static_cast<double>(-params_.turbo_max_velocity));
    cmd_vel.angular.z = 0.0 < cmd_vel.angular.z
                            ? std::min(cmd_vel.angular.z, static_cast<double>(params_.max_yawrate))
                            : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));
  }
  else
  {
    cmd_vel.linear.x = 0.0 < cmd_vel.linear.x ? std::min(cmd_vel.linear.x, static_cast<double>(params_.max_velocity))
                                              : std::max(cmd_vel.linear.x, static_cast<double>(-params_.max_velocity));
    cmd_vel.angular.z = 0.0 < cmd_vel.angular.z
                            ? std::min(cmd_vel.angular.z, static_cast<double>(params_.max_yawrate))
                            : std::max(cmd_vel.angular.z, -static_cast<double>(params_.max_yawrate));
  }

  if (flags_.emergency_stop || mode_.first == "stop")
    cmd_vel = geometry_msgs::Twist();
  if (mode_.first == "move" && mode_.second == "auto")
  {
    if (!front_laser_.has_value())
    {
      ROS_ERROR("Front laser data is not available..");
      cmd_vel = geometry_msgs::Twist();
    }
    if (!rear_laser_.has_value() && params_.use_rear_laser)
    {
      ROS_ERROR("Rear laser data is not available..");
      cmd_vel = geometry_msgs::Twist();
    }
  }

  cmd_vel_pub_.publish(cmd_vel);
  print_status(cmd_vel);
}

void MotionDecision::print_mode_status(const std::string &status, std::string color)
{
  if (color == "")
    color = RESET_COLOR;
  const std::string parrentheses = status == "" ? ")" : ") ";
  std::cout << "=== " << mode_.second << " (" << mode_.first << parrentheses << color << status << RESET_COLOR
            << " ===" << std::endl;
}

void MotionDecision::print_status(const geometry_msgs::Twist &cmd_vel)
{
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

  if (0 < counters_.recovery && params_of_recovery_.available)
  {
    std::cout << "#####################" << std::endl;
    std::cout << "### recovery mode ###" << std::endl;
    std::cout << "#####################" << std::endl;
  }
  else if (0 < counters_.stuck && params_of_recovery_.available)
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
  if (laser_info_.front_min_range == -1.0)
    std::cout << "min front laser : " << RED << laser_info_.front_min_range << RESET_COLOR << std::endl;
  else
    std::cout << "min front laser : " << laser_info_.front_min_range << std::endl;
  if (params_.use_rear_laser)
  {
    if (laser_info_.front_min_range == -1.0)
      std::cout << "min rear laser : " << RED << laser_info_.rear_min_range << RESET_COLOR << std::endl;
    else
      std::cout << "min rear laser  : " << laser_info_.rear_min_range << std::endl;
  }
  if (params_of_recovery_.available)
  {
    std::cout << "recovery mode   : available" << std::endl;
    std::cout << "recovery time   : " << counters_.recovery / static_cast<float>(params_.hz) << std::endl;
    std::cout << "stuck time      : " << counters_.stuck / static_cast<float>(params_.hz) << std::endl;
  }
  else
  {
    std::cout << "recovery mode   : unavailable" << std::endl;
  }
  if (mode_.first == "move")
  {
    std::cout << "cmd vel :" << std::endl;
    std::cout << "  linear.x  : " << cmd_vel.linear.x << std::endl;
    std::cout << "  angular.z : " << cmd_vel.angular.z << std::endl;
  }
  std::cout << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_decision");
  MotionDecision motion_decision;
  motion_decision.process();

  return 0;
}
