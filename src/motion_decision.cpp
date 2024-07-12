/**
 * @file motion_decision.cpp
 * @author amsl
 * @brief C++ implementation of motion decision
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

  private_nh_.param<int>("HZ", HZ, {20});
  private_nh_.param<int>("RECOVERY_MODE_THRESHOLD", RECOVERY_MODE_THRESHOLD, {60});
  private_nh_.param<int>("TRIGGER_COUNT_THRESHOLD", TRIGGER_COUNT_THRESHOLD, {2});
  private_nh_.param<double>("MAX_SPEED", MAX_SPEED, {1.0});
  private_nh_.param<double>("MAX_YAWRATE", MAX_YAWRATE, {1.0});
  private_nh_.param<double>("DT", DT, {0.1});
  private_nh_.param<double>("PREDICT_TIME", PREDICT_TIME, {1.0});
  private_nh_.param<double>("COLLISION_DISTANCE", COLLISION_DISTANCE, {0.4});
  private_nh_.param<double>("SAFETY_COLLISION_TIME", SAFETY_COLLISION_TIME, {0.5});
  private_nh_.param<std::string>("STOP_SOUND_PATH", STOP_SOUND_PATH, std::string(""));
  private_nh_.param<std::string>("RECOVERY_SOUND_PATH", RECOVERY_SOUND_PATH, std::string(""));
  private_nh_.param<std::string>("TASK_STOP_SOUND_PATH", TASK_STOP_SOUND_PATH, std::string(""));

  emergency_stop_flag_ = false;
  task_stop_flag_ = false;
  safety_mode_flag_ = false;
  auto_flag_ = false;
  move_flag_ = false;
  joy_flag_ = false;
  intersection_flag_ = false;
  enable_recovery_mode_ = true;
  local_path_received_ = false;
  front_laser_received_ = true;
  rear_laser_received_ = true;
  stuck_count_ = 0;
  front_min_range_ = -1.0;
  rear_min_range_ = -1.0;
  trigger_count_ = 0;
}

void MotionDecision::emergency_stop_flag_callback(const std_msgs::BoolConstPtr &msg)
{
  emergency_stop_flag_ = msg->data;
}

void MotionDecision::front_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  front_laser_ = *msg;
  front_min_range_ = front_laser_.range_max;
  front_min_idx_ = 0;
  int count = 0;
  for (auto range : front_laser_.ranges)
  {
    if (range < front_min_range_)
    {
      if (range > 0.1)
      {
        front_min_range_ = range;
        front_min_idx_ = count;
      }
    }
    count++;
  }
  front_laser_received_ = true;
}

void MotionDecision::joy_callback(const sensor_msgs::JoyConstPtr &msg)
{
  joy_ = *msg;
  if (joy_.buttons[3])
  { // square button
    auto_flag_ = false;
  }
  else if (joy_.buttons[2])
  { // triangle button
    auto_flag_ = true;
  }

  if (joy_.buttons[0])
  { // cross button
    move_flag_ = false;
  }
  else if (joy_.buttons[1])
  { // circle button
    move_flag_ = true;
  }
  joy_vel_.linear.x = joy_.axes[1] * MAX_SPEED;
  joy_vel_.angular.z = joy_.axes[0] * MAX_YAWRATE;
  /*
      if(joy.buttons[5]){
          intersection_flag = true;
      }else{
          intersection_flag = false;
      }
  */

  if (joy_.buttons[4])
  {
    joy_flag_ = true;
  }
  else
  {
    joy_flag_ = false;
  }
}

void MotionDecision::local_path_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  cmd_vel_ = *msg;
  local_path_received_ = true;
}

void MotionDecision::odom_callback(const nav_msgs::OdometryConstPtr &msg) { odom_ = *msg; }

void MotionDecision::rear_laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  rear_laser_ = *msg;
  rear_min_range_ = rear_laser_.range_max;
  rear_min_idx_ = 0;
  int count = 0;
  for (auto range : rear_laser_.ranges)
  {
    if (range < rear_min_range_)
    {
      if (range > 0.1)
      {
        rear_min_range_ = range;
        rear_min_idx_ = count;
      }
    }
    count++;
  }
  rear_laser_received_ = true;
}

void MotionDecision::recovery_mode_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
  enable_recovery_mode_ = msg->data;
}

void MotionDecision::task_stop_flag_callback(const std_msgs::BoolConstPtr &msg)
{
  std_msgs::Bool flag = *msg;
  if (flag.data)
  {
    std::cout << "========= task stop =========" << std::endl;
    if (TASK_STOP_SOUND_PATH != "")
    {
      std::string sound_command = "aplay " + TASK_STOP_SOUND_PATH + " &";
      system(sound_command.c_str());
      system(sound_command.c_str());
    }
    move_flag_ = false;
  }
  else
  {
    move_flag_ = true;
  }
}

void MotionDecision::process(void)
{
  ros::Rate loop_rate(HZ);
  while (ros::ok())
  {
    geometry_msgs::Twist vel;
    std::cout << "==== motion decision ====" << std::endl;
    std::cout << "min front laser : " << front_min_range_ << std::endl;
    std::cout << "min rear laser  : " << rear_min_range_ << std::endl;
    std::cout << "trigger count   : " << trigger_count_ << std::endl;
    if (move_flag_)
    {
      std::cout << "move : (";
      if (auto_flag_)
      {
        std::cout << "auto";
        vel = cmd_vel_;
        if (0 < trigger_count_ && trigger_count_ < TRIGGER_COUNT_THRESHOLD)
        {
          recovery_mode(vel, false);
          trigger_count_++;
        }
        else if (
            enable_recovery_mode_ && ((vel.linear.x < DBL_EPSILON && fabs(vel.angular.z) < DBL_EPSILON) ||
                                      (odom_.twist.twist.linear.x < 0.01 && fabs(odom_.twist.twist.angular.z) < 0.01)))
        {
          std::cout << ")" << std::endl;
          std::cout << "stuck_count : " << stuck_count_ << std::endl;
          if (stuck_count_ < RECOVERY_MODE_THRESHOLD)
          {
            stuck_count_++;
          }
          else
          {
            recovery_mode(vel, false);
            trigger_count_++;
          }
        }
        else
        {
          stuck_count_ = 0;
          trigger_count_ = 0;
        }
        front_laser_received_ = false;
        rear_laser_received_ = false;
      }
      else
      {
        std::cout << "manual";
        if (joy_flag_)
        {
          vel = joy_vel_;
        }
        else
        {
          vel.linear.x = 0.0;
          vel.angular.z = 0.0;
        }

        stuck_count_ = 0;
        trigger_count_ = 0;
      }
      std::cout << ")" << std::endl;
      std::cout << vel << std::endl;
    }
    else
    {
      std::cout << "stop : (" << (auto_flag_ ? "auto" : "manual") << ")" << std::endl;
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;

      stuck_count_ = 0;
      trigger_count_ = 0;
    }
    if (emergency_stop_flag_)
    {
      std::cout << "emergency stop" << std::endl;
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;
    }
    if (intersection_flag_)
    {
      std_msgs::Bool flag;
      flag.data = true;
      intersection_flag_pub_.publish(flag);
      std::cout << "=========intersection=============" << std::endl;
    }

    if (vel.linear.x > MAX_SPEED)
    {
      vel.linear.x = MAX_SPEED;
    }
    else if (vel.linear.x < -MAX_SPEED)
    {
      vel.linear.x = MAX_SPEED;
    }
    if (vel.angular.z > MAX_YAWRATE)
    {
      vel.angular.z = MAX_YAWRATE;
    }
    else if (vel.angular.z < -MAX_YAWRATE)
    {
      vel.angular.z = -MAX_YAWRATE;
    }

    velocity_pub_.publish(vel);

    front_min_range_ = -1.0;
    rear_min_range_ = -1.0;

    loop_rate.sleep();
    ros::spinOnce();
  }
}

void MotionDecision::recovery_mode(geometry_msgs::Twist &cmd_vel, bool go_back)
{
  std::cout << "=== recovery mode ===" << std::endl;
  if (!front_laser_received_ || !rear_laser_received_)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return;
  }

  // variables for recovery mode
  double max_velocity_limit = 0.3;
  double max_yawrate_limit = 0.3;
  double velocity_resolution = 0.1;
  double yawrate_resolution = 0.1;
  double max_velocity = 0.0;
  double max_yawrate = 0.0;
  double max_ttc = 0.0;
  bool reverse_flag = false;
  if (!go_back)
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
        double ttc = calc_ttc(vel, true);
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
          double angle = (2.0 * front_min_idx_ / front_laser_.ranges.size() - 1.0) * (front_laser_.angle_max);
          double angle_diff_a = fabs(angle - atan2(max_y, max_x));
          double angle_diff_b = fabs(angle - atan2(y, x));
          if (M_PI / 2.0 <= fabs(angle) && angle_diff_a < angle_diff_b && calc_ttc(vel, false) > SAFETY_COLLISION_TIME)
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
    if (max_ttc > SAFETY_COLLISION_TIME)
    {
      cmd_vel.linear.x = max_velocity;
      cmd_vel.angular.z = max_yawrate;
    }
    else
    {
      // set vel to move away from obstacles
      if (front_min_idx_ < front_laser_.ranges.size() * 0.5)
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
        double ttc = calc_ttc(vel, false);
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
          double angle = (2.0 * rear_min_idx_ / rear_laser_.ranges.size() - 1.0) * (rear_laser_.angle_max);
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
    if (max_ttc > SAFETY_COLLISION_TIME)
    {
      cmd_vel.linear.x = max_velocity;
      cmd_vel.angular.z = max_yawrate;
    }
    else
    {
      // set vel to move away from obstacles
      if (rear_min_idx_ > rear_laser_.ranges.size() * 0.5)
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
}

float MotionDecision::calc_ttc(geometry_msgs::Twist vel, bool go_back)
{
  // select laser data by direction of motion
  sensor_msgs::LaserScan laser;
  if (!go_back)
  {
    laser = front_laser_;
  }
  else
  {
    laser = rear_laser_;
  }

  // calculate TTC
  double ttc = PREDICT_TIME;
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

    for (double time = DT; time < PREDICT_TIME; time += DT)
    {
      yaw += vel.angular.z * DT;
      x += fabs(vel.linear.x) * cos(yaw) * DT;
      y += fabs(vel.linear.x) * sin(yaw) * DT;
      double tmp_range = sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
      if (tmp_range < COLLISION_DISTANCE)
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
