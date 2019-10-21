#include "ros/ros.h"
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>

class MotionDecision{
    public:
        MotionDecision();
        void LocalPathCallback(const geometry_msgs::TwistConstPtr& msg);
        void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
        void EmergencyStopFlagCallback(const std_msgs::BoolConstPtr& msg);
        void TaskStopFlagCallback(const std_msgs::BoolConstPtr& msg);
        void FrontLaserCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void RearLaserCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void LocalGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        void process();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        //subscriber
        ros::Subscriber local_path_sub;
        ros::Subscriber joy_sub;
        ros::Subscriber emergency_stop_flag_sub;
        ros::Subscriber task_stop_flag_sub;
        ros::Subscriber front_laser_sub;
        ros::Subscriber rear_laser_sub;
        ros::Subscriber local_goal_sub;

        //publisher
        ros::Publisher vel_pub;
        ros::Publisher intersection_flag_pub;

        void recovery_mode(geometry_msgs::Twist& cmd_vel);

        bool emergency_stop_flag;
        bool task_stop_flag;
        bool auto_flag;
        bool move_flag;
        bool joy_flag;
        bool intersection_flag;
        bool safety_mode_flag;
        bool front_laser_flag;
        bool rear_laser_flag;
        bool target_arrival;
        bool front_laser_received;
        bool rear_laser_received;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Twist joy_vel;
        sensor_msgs::LaserScan front_laser;
        sensor_msgs::LaserScan rear_laser;
        sensor_msgs::Joy joy;
        float front_min_range;
        float rear_min_range;
        int HZ;
        double MAX_SPEED;
        double MAX_YAWRATE;
        double VEL_RATIO;
        double SAFETY_DISTANCE;
        double GOAL_DISTANCE;
        int RECOVERY_MODE_THRESHOLD;
        int stop_count;
        int stuck_count;
        int front_min_idx;
        int rear_min_idx;
        double target_yaw;
};

MotionDecision::MotionDecision()
    :private_nh("~")
{
    //subscriber
    local_path_sub = nh.subscribe("/local_path/cmd_vel",1, &MotionDecision::LocalPathCallback, this);
    joy_sub = nh.subscribe("/joy",1, &MotionDecision::JoyCallback, this);
    front_laser_sub = nh.subscribe("/front_laser/scan",1, &MotionDecision::FrontLaserCallback, this);
    rear_laser_sub = nh.subscribe("/rear_laser/scan",1, &MotionDecision::RearLaserCallback, this);
    emergency_stop_flag_sub = nh.subscribe("/emergency_stop",1, &MotionDecision::EmergencyStopFlagCallback, this);
    task_stop_flag_sub = nh.subscribe("/task/stop",1, &MotionDecision::TaskStopFlagCallback, this);
    local_goal_sub = nh.subscribe("/local_goal",1, &MotionDecision::LocalGoalCallback, this);

    //publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    intersection_flag_pub = nh.advertise<std_msgs::Bool>("/intersection_flag",1,true);

    private_nh.param("HZ", HZ, {20});
    private_nh.param("MAX_SPEED", MAX_SPEED, {1.0});
    private_nh.param("MAX_YAWRATE", MAX_YAWRATE, {1.0});
    private_nh.param("VEL_RATIO", VEL_RATIO, {0.5});
    private_nh.param("SAFETY_DISTANCE", SAFETY_DISTANCE, {0.6});
    private_nh.param("GOAL_DISTANCE", GOAL_DISTANCE, {0.6});
    private_nh.param("RECOVERY_MODE_THRESHOLD", RECOVERY_MODE_THRESHOLD, {60});

    emergency_stop_flag = false;
    task_stop_flag = false;
    safety_mode_flag = false;
    auto_flag = false;
    move_flag = false;
    joy_flag = false;
    front_laser_flag = false;
    rear_laser_flag = false;
    intersection_flag = false;
    target_arrival = false;
    front_laser_received = false;
    rear_laser_received = false;
    stop_count = 0;
    stuck_count = 0;
    target_yaw = 0.0;

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
}

void MotionDecision::LocalGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    geometry_msgs::PoseStamped local_goal;
    local_goal = *msg;
    target_arrival = false;
    double x = local_goal.pose.position.x;
    double y = local_goal.pose.position.y;
    double dis = sqrt(x*x + y*y);
    if(dis < GOAL_DISTANCE){
        target_arrival = true;
    }
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(local_goal.pose.orientation, quaternion);
    target_yaw = tf::getYaw(quaternion);
}

void MotionDecision::LocalPathCallback(const geometry_msgs::TwistConstPtr& msg)
{
    cmd_vel = *msg;
}

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
        count ++;
    }
    std::cout << "min front laser : " << front_min_range << std::endl;
    if(front_min_range < SAFETY_DISTANCE){
        if(front_min_range > SAFETY_DISTANCE*0.1){
            front_laser_flag = true;
        }else{
            front_laser_flag = false;
        }
    }else{
        front_laser_flag = false;
    }
    front_laser_received = true;
}

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
        count ++;
    }
    std::cout << "min rear laser : " << rear_min_range << std::endl;
    if(rear_min_range < SAFETY_DISTANCE){
        if(rear_min_range > SAFETY_DISTANCE*0.1){
            rear_laser_flag = true;
        }else{
            rear_laser_flag = true;
        }
    }
    rear_laser_received = true;
}

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

    if(joy.buttons[13]){
        joy_vel.linear.x = VEL_RATIO*MAX_SPEED;
        joy_vel.angular.z = 0.0;
    }else if(joy.buttons[14]){
        joy_vel.linear.x = -VEL_RATIO*MAX_SPEED;
        joy_vel.angular.z = 0.0;
    }else if(joy.buttons[15]){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = VEL_RATIO*MAX_YAWRATE;
    }else if(joy.buttons[16]){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = -VEL_RATIO*MAX_YAWRATE;
    }

    if(joy.buttons[5]){
        intersection_flag = true;
    }else{
        intersection_flag = false;
    }

    if(joy.buttons[6]){
        joy_flag = true;
    }else{
        joy_flag = false;
    }
}

void MotionDecision::EmergencyStopFlagCallback(const std_msgs::BoolConstPtr& msg)
{
    emergency_stop_flag = msg->data;
}

void MotionDecision::TaskStopFlagCallback(const std_msgs::BoolConstPtr& msg)
{
    std_msgs::Bool flag = *msg;
    if(flag.data){
        std::cout << "========= task stop =========" << std::endl;
        move_flag = false;
    }
}

void MotionDecision::recovery_mode(geometry_msgs::Twist& cmd_vel)
{
    std::cout << "=== recovery mode ===" << std::endl;
    if(front_min_range < SAFETY_DISTANCE){
        if(rear_min_range > SAFETY_DISTANCE){
            if(front_min_idx > front_laser.ranges.size()*2.0/3.0){
                cmd_vel.linear.x = -0.2;
                cmd_vel.angular.z = 0.15;
            }else if((front_laser.ranges.size()*1.0/3.0 < front_min_idx) && (front_min_idx < front_laser.ranges.size()*2.0/3.0)){
                cmd_vel.linear.x = -0.2;
                cmd_vel.angular.z = 0.0;
            }else{
                cmd_vel.linear.x = -0.2;
                cmd_vel.angular.z = -0.15;
            }
        }else{
            if(front_min_idx > front_laser.ranges.size()*0.5){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.2;
            }else{
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -0.2;
            }
        }
    }else{
        if(front_min_idx > front_laser.ranges.size()*0.5){
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.2;
        }else{
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.2;
        }
    }
    if(front_min_range > SAFETY_DISTANCE*1.2){
        safety_mode_flag = false;
        stop_count = 0;
    }
}

void MotionDecision::process()
{
    ros::Rate loop_rate(HZ);
    while(ros::ok()){
        geometry_msgs::Twist vel;
        std::cout << "==== motion decision ====" << std::endl;
        if(move_flag){
            std::cout << "move : (";
            if(auto_flag){
                std::cout << "auto";
                if(front_laser_received && rear_laser_received){
                    vel = cmd_vel;
                    if(target_arrival){
                        std::cout << "=== target arrival ===" << std::endl;
                        vel.linear.x = 0.0;
                        vel.angular.z = target_yaw/M_PI * MAX_YAWRATE;
                    }
                    if(vel.linear.x == 0.0 && vel.angular.z==0.0){
                        std::cout << ")" << std::endl;
                        std::cout << "=== stuck recovery mode ===" << std::endl;
                        std::cout << "stuck_count" << stuck_count<< std::endl;
                        if(stuck_count < RECOVERY_MODE_THRESHOLD){
                            stuck_count ++;
                        }else{
                            recovery_mode(vel);
                        }
                    }else{
                        stuck_count = 0;
                    }
                    if((vel.linear.x > 0.0 && front_laser_flag) || (vel.linear.x<0.0 && rear_laser_flag)){
                        safety_mode_flag = true;
                    }
                    if(safety_mode_flag){
                        std::cout << ")" << std::endl;
                        std::cout << "=== safety mode ===" << std::endl;
                        std::cout << "stop_count" << stop_count<< std::endl;
                        if(stop_count < RECOVERY_MODE_THRESHOLD){
                            vel.linear.x = 0.0;
                            vel.angular.z = 0.0;
                            stop_count ++;
                        }else{
                            recovery_mode(vel);
                        }
                    }else{
                        stop_count = 0;
                    }
                }else{
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                    std::cout << ")" << std::endl;
                    std::cout << "front_laser: " << front_laser_received << std::endl;
                    std::cout << "rear_laser: " << rear_laser_received << std::endl;
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
            }
            std::cout << ")" << std::endl;
            std::cout << vel << std::endl;
        }else{
            std::cout << "stop : (" << (auto_flag ? "auto" : "manual") << ")"<< std::endl;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
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
        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_decision");

    MotionDecision motion_decision;
    motion_decision.process();

    return 0;
}
