/**
 * @file motion_decision.cpp
 * @author amsl
 * @brief C++ implementation of motion decision
 */

#include "motion_decision/motion_decision.h"

/**
 * @class MotionDecision
 * @brief Motion Decision Class
 */
MotionDecision::MotionDecision()
    :private_nh("~")
{
    local_path_sub = nh.subscribe("/local_path/cmd_vel",1, &MotionDecision::LocalPathCallback, this);
    joy_sub = nh.subscribe("/joy",1, &MotionDecision::JoyCallback, this);
    front_laser_sub = nh.subscribe("/front_laser/scan",1, &MotionDecision::FrontLaserCallback, this);
    rear_laser_sub = nh.subscribe("/rear_laser/scan",1, &MotionDecision::RearLaserCallback, this);
    emergency_stop_flag_sub = nh.subscribe("/emergency_stop",1, &MotionDecision::EmergencyStopFlagCallback, this);
    task_stop_flag_sub = nh.subscribe("/task/stop",1, &MotionDecision::TaskStopFlagCallback, this);
    recovery_mode_flag_sub = nh.subscribe("/recovery_mode_flag", 1, &MotionDecision::RecoveryModeFlagCallback, this);
    odom_sub = nh.subscribe("/odom", 1, &MotionDecision::OdomCallback, this);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    intersection_flag_pub = nh.advertise<std_msgs::Bool>("/intersection_flag",1,true);

    private_nh.param("HZ", HZ, {20});
    private_nh.param("MAX_SPEED", MAX_SPEED, {1.0});
    private_nh.param("MAX_YAWRATE", MAX_YAWRATE, {1.0});
    private_nh.param("COLLISION_DISTANCE", COLLISION_DISTANCE, {0.4});
    private_nh.param("PREDICT_TIME", PREDICT_TIME, {1.0});
    private_nh.param("SAFETY_COLLISION_TIME", SAFETY_COLLISION_TIME, {0.5});
    private_nh.param("DT", DT, {0.1});
    private_nh.param("RECOVERY_MODE_THRESHOLD", RECOVERY_MODE_THRESHOLD, {60});
    private_nh.param("TRIGGER_COUNT_THRESHOLD", TRIGGER_COUNT_THRESHOLD, {2});
    private_nh.param("STOP_SOUND_PATH", STOP_SOUND_PATH, {""});
    private_nh.param("RECOVERY_SOUND_PATH", RECOVERY_SOUND_PATH, {""});
    private_nh.param("TASK_STOP_SOUND_PATH", TASK_STOP_SOUND_PATH, {""});

    emergency_stop_flag = false;
    task_stop_flag = false;
    safety_mode_flag = false;
    auto_flag = false;
    move_flag = false;
    joy_flag = false;
    intersection_flag = false;
    enable_recovery_mode = false;
    local_path_received = false;
    front_laser_received = true;
    rear_laser_received = true;
    stuck_count = 0;
    front_min_range = -1.0;
    rear_min_range = -1.0;
    trigger_count = 0;

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
}

/**
 * @brief Local path callback function
 * @param [in] msg Msg from local_path_sub
 */
void MotionDecision::LocalPathCallback(const geometry_msgs::TwistConstPtr& msg)
{
    cmd_vel = *msg;
    local_path_received = true;
}

/**
 * @brief Front laser callback function
 * @details Cache the closest valid sensor data and its index in front_min_range and front_min_idx respectively
 * @param [in] msg Msg from front_laser_sub
 */
void MotionDecision::FrontLaserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    front_laser = *msg;
    front_min_range = front_laser.range_max;
    front_min_idx = 0;
    int count = 0;
    for(auto range : front_laser.ranges){
        if(range < front_min_range){
            if(range > 0.1){
                front_min_range = range;
                front_min_idx = count;
            }
        }
        count++;
    }
    front_laser_received = true;
}

/**
 * @brief Rear laser callback function
 * @details Cache the closest valid sensor data and its index in rear_min_range and rear_min_idx respectively
 * @param [in] msg Msg from rear_laser_sub
 */
void MotionDecision::RearLaserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    rear_laser = *msg;
    rear_min_range = rear_laser.range_max;
    rear_min_idx = 0;
    int count = 0;
    for(auto range : rear_laser.ranges){
        if(range < rear_min_range){
            if(range > 0.1){
                rear_min_range = range;
                rear_min_idx = count;
            }
        }
        count++;
    }
    rear_laser_received = true;
}

/**
 * @brief Recovery mode flag callback function
 * @param [in] msg Msg from recovery_mode_flag_sub
 */
void MotionDecision::RecoveryModeFlagCallback(const std_msgs::Bool::ConstPtr &msg)
{
    enable_recovery_mode = msg->data;
}

/**
 * @brief Odom callback function
 * @param [in] msg Msg from odom_sub
 */
void MotionDecision::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
}

/**
 * @brief Calculate TTC (Time To Collision) function
 * @param [in] vel Current velocity
 * @param [in] go_back Direction of motion
 * @return float TTC
 */
float MotionDecision::CalcTTC(geometry_msgs::Twist vel, bool go_back)
{
    // select laser data by direction of motion
    sensor_msgs::LaserScan laser;
    if(!go_back){
        laser = front_laser;
    }else{
        laser = rear_laser;
    }

    // calculate TTC
    double ttc = PREDICT_TIME;
    int i = 0;
    for(auto range : laser.ranges){
        // ignore invalid laser data
        if(range < 0.1){
            continue;
        }
        double x=0;
        double y=0;
        double yaw=0;
        double ox,oy,angle;
        angle = (2.0*i/laser.ranges.size()-1.0)*(laser.angle_max);
        ox = range * cos(angle);
        oy = range * sin(angle);

        for(double t=DT; t<PREDICT_TIME; t+=DT){
            yaw+=vel.angular.z*DT;
            x+=fabs(vel.linear.x)*cos(yaw)*DT;
            y+=fabs(vel.linear.x)*sin(yaw)*DT;
            double r = sqrt((x-ox)*(x-ox)+(y-oy)*(y-oy));
            if(r<COLLISION_DISTANCE){
                if(t<ttc){
                    ttc = t;
                    break;
                }
            }
        }
        i++;
    }
    return ttc;
}

/**
 * @brief Rear laser callback function
 * @details Set flags by input from joy
 * @param [in] msg Msg from joy_sub
 */
void MotionDecision::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    joy = *msg;
    if(joy.buttons[3]){ //square button
        auto_flag = false;
    }else if(joy.buttons[2]){ // triangle button
        auto_flag = true;
    }

    if(joy.buttons[0]){ // cross button
        move_flag = false;
    }else if(joy.buttons[1]){ // circle button
        move_flag = true;
    }
    joy_vel.linear.x = joy.axes[1]*MAX_SPEED;
    joy_vel.angular.z = joy.axes[0]*MAX_YAWRATE;
/*
    if(joy.buttons[5]){
        intersection_flag = true;
    }else{
        intersection_flag = false;
    }
*/

    if(joy.buttons[4]){
        joy_flag = true;
    }else{
        joy_flag = false;
    }
}

/**
 * @brief Emergency stop flag callback function
 * @param [in] msg Msg from emergency_stop_flag_sub
 */
void MotionDecision::EmergencyStopFlagCallback(const std_msgs::BoolConstPtr& msg)
{
    emergency_stop_flag = msg->data;
}

/**
 * @brief Task stop flag callback function
 * @details Emergency stop when task stop flag is true
 * @param [in] msg Msg from task_stop_flag_sub
 */
void MotionDecision::TaskStopFlagCallback(const std_msgs::BoolConstPtr& msg)
{
    std_msgs::Bool flag = *msg;
    if(flag.data){
        std::cout << "========= task stop =========" << std::endl;
        if(TASK_STOP_SOUND_PATH != ""){
            std::string sound_command = "aplay " + TASK_STOP_SOUND_PATH + " &";
            system(sound_command.c_str());
            system(sound_command.c_str());
        }
        move_flag = false;
    }else{
        move_flag = true;
    }
}

/**
 * @brief Code for recovery mode
 * @param [out] cmd_vel Velocity
 * @param [in] go_back Direction of motion
 */
void MotionDecision::recovery_mode(geometry_msgs::Twist& cmd_vel, bool go_back)
{
    std::cout << "=== recovery mode ===" << std::endl;
    if(!front_laser_received || !rear_laser_received){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return;
    }

    // variables for recovery mode
    double max_velocity = 0.3;
    double max_yawrate = 0.3;
    double velocity_resolution = 0.1;
    double yawrate_resolution = 0.1;
    double max_v = 0.0;
    double max_w = 0.0;
    double max_ttc = 0.0;
    bool reverse_flag = false;
    if(!go_back){
    // when moving forwards
        // calculate ttc when if go backwards
        for(double v=-velocity_resolution; v>=-max_velocity; v-=velocity_resolution){
            for(double w=-max_yawrate; w<=max_yawrate; w+=yawrate_resolution){
                geometry_msgs::Twist vel;
                vel.linear.x = v;
                vel.angular.z = w;
                // carefull to modify param true
                // go_back variable is false but robot moving backwards virtuary so needed laser data is rear's
                double ttc = CalcTTC(vel, true);
                if(ttc > max_ttc){
                    max_v = v;
                    max_w = w;
                    max_ttc = ttc;
                }else if(ttc == max_ttc){
                    double max_x =  max_v/max_w*sin(max_w*ttc);
                    double max_y =  max_v/max_w*(1-cos(max_w*ttc));
                    double x =  v/w*sin(w*ttc);
                    double y =  v/w*(1-cos(w*ttc));
                    double angle = (2.0*front_min_idx/front_laser.ranges.size()-1.0)*(front_laser.angle_max);
                    double angle_diff_a = fabs(angle - atan2(max_y, max_x));
                    double angle_diff_b = fabs(angle - atan2(y, x));
                    if(M_PI/2.0 <= fabs(angle) && angle_diff_a < angle_diff_b && CalcTTC(vel, false) > SAFETY_COLLISION_TIME){
                        max_v = v;
                        max_w = w;
                        max_ttc = ttc;
                        reverse_flag = true;
                    }else if(!reverse_flag && angle_diff_a > angle_diff_b){
                        max_v = v;
                        max_w = w;
                        max_ttc = ttc;
                    }
                }
            }
        }
        if(max_ttc > SAFETY_COLLISION_TIME){
            cmd_vel.linear.x = max_v;
            cmd_vel.angular.z = max_w;
        }else{
            // set vel to move away from obstacles
            if(front_min_idx < front_laser.ranges.size()*0.5){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.2;
            }else{
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -0.2;
            }
        }
    }else{
    // when moving backwards
        // calculate ttc when if go forwards
        for(double v=velocity_resolution; v<=max_velocity; v+=velocity_resolution){
            for(double w=-max_yawrate; w<=max_yawrate; w+=yawrate_resolution){
                geometry_msgs::Twist vel;
                vel.linear.x = v;
                vel.angular.z = w;
                // carefull to modify param true
                // go_back variable is true but robot moving forwards virtuary so needed laser data is front's
                double ttc = CalcTTC(vel, false);
                if(ttc > max_ttc){
                    max_v = v;
                    max_w = w;
                    max_ttc = ttc;
                }else if(ttc == max_ttc){
                    double max_x =  max_v/max_w*sin(max_w*ttc);
                    double max_y =  max_v/max_w*(1-cos(max_w*ttc));
                    double x =  v/w*sin(w*ttc);
                    double y =  v/w*(1-cos(w*ttc));
                    double angle = (2.0*rear_min_idx/rear_laser.ranges.size()-1.0)*(rear_laser.angle_max);
                    double angle_diff_a = fabs(angle - atan2(max_y, max_x));
                    double angle_diff_b = fabs(angle - atan2(y, x));
                    if(angle_diff_a > angle_diff_b){
                        max_v = v;
                        max_w = w;
                        max_ttc = ttc;
                    }
                }
            }
        }
        if(max_ttc > SAFETY_COLLISION_TIME){
            cmd_vel.linear.x = max_v;
            cmd_vel.angular.z = max_w;
        }else{
            // set vel to move away from obstacles
            if(rear_min_idx > rear_laser.ranges.size()*0.5){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -0.2;
            }else{
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.2;
            }
        }
    }
}

/**
 * @brief Process function
 */
void MotionDecision::process()
{
    ros::Rate loop_rate(HZ);
    while(ros::ok()){
        geometry_msgs::Twist vel;
        std::cout << "==== motion decision ====" << std::endl;
        std::cout << "min front laser : " << front_min_range << std::endl;
        std::cout << "min rear laser  : " << rear_min_range  << std::endl;
        std::cout << "trigger count   : " << trigger_count  << std::endl;
        if(move_flag){
            std::cout << "move : (";
            if(auto_flag){
                std::cout << "auto";
                vel= cmd_vel;
                if(0 < trigger_count && trigger_count < TRIGGER_COUNT_THRESHOLD){
                    recovery_mode(vel, false);
                    trigger_count++;
                }else if(enable_recovery_mode && ((vel.linear.x < DBL_EPSILON && fabs(vel.angular.z) < DBL_EPSILON) || (odom.twist.twist.linear.x < 0.01 && fabs(odom.twist.twist.angular.z) < 0.01))){
                    std::cout << ")" << std::endl;
                    std::cout << "=== stuck recovery mode ===" << std::endl;
                    std::cout << "stuck_count" << stuck_count << std::endl;
                    if(stuck_count < RECOVERY_MODE_THRESHOLD){
                        stuck_count++;
                    }else{
                        recovery_mode(vel, false);
                        trigger_count++;
                    }
                }else{
                    stuck_count = 0;
                    trigger_count = 0;
                }
                front_laser_received = false;
                rear_laser_received = false;
            }else{
                std::cout << "manual";
                if(joy_flag){
                    vel = joy_vel;
                }else{
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                }

                stuck_count = 0;
                trigger_count = 0;
            }
            std::cout << ")" << std::endl;
            std::cout << vel << std::endl;
        }else{
            std::cout << "stop : (" << (auto_flag ? "auto" : "manual") << ")"<< std::endl;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;

            stuck_count = 0;
            trigger_count = 0;
        }
        if(emergency_stop_flag){
            std::cout << "emergency stop" << std::endl;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
        }
        if(intersection_flag){
            std_msgs::Bool flag;
            flag.data=true;
            intersection_flag_pub.publish(flag);
            std::cout << "=========intersection=============" << std::endl;
        }

        if(vel.linear.x > MAX_SPEED){
            vel.linear.x = MAX_SPEED;
        }else if(vel.linear.x < -MAX_SPEED){
            vel.linear.x = MAX_SPEED;
        }
        if(vel.angular.z > MAX_YAWRATE){
            vel.angular.z = MAX_YAWRATE;
        }else if(vel.angular.z < -MAX_YAWRATE){
            vel.angular.z = -MAX_YAWRATE;
        }

        vel_pub.publish(vel);

        front_min_range = -1.0;
        rear_min_range = -1.0;

        loop_rate.sleep();
        ros::spinOnce();
    }
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
