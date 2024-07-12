/**
 * @file motion_decision.cpp
 * @author amsl
 * @brief C++ implementation for Motion Decision
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

/**
 * @brief Motion Decision Class.
 */

class MotionDecision{
    public:
        /**
         * @brief Constructor
         */
        MotionDecision();
        void LocalPathCallback(const geometry_msgs::TwistConstPtr& msg);
        void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
        void EmergencyStopFlagCallback(const std_msgs::BoolConstPtr& msg);
        void TaskStopFlagCallback(const std_msgs::BoolConstPtr& msg);
        void FrontLaserCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void RearLaserCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void RecoveryModeFlagCallback(const std_msgs::Bool::ConstPtr& msg);
        void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

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
        ros::Subscriber recovery_mode_flag_sub;
        ros::Subscriber odom_sub;

        //publisher
        ros::Publisher vel_pub;
        ros::Publisher intersection_flag_pub;

        void recovery_mode(geometry_msgs::Twist& cmd_vel, bool go_back);
        float CalcTTC(geometry_msgs::Twist vel, bool go_back);

        bool emergency_stop_flag;
        bool task_stop_flag;
        bool auto_flag;
        bool move_flag;
        bool joy_flag;
        bool intersection_flag;
        bool safety_mode_flag;
        bool enable_recovery_mode;
        bool local_path_received;
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
        double COLLISION_DISTANCE;
        double DT;
        double PREDICT_TIME;
        double SAFETY_COLLISION_TIME;
        int RECOVERY_MODE_THRESHOLD;
        int TRIGGER_COUNT_THRESHOLD;
        int stuck_count;
        int trigger_count;
        int front_min_idx;
        int rear_min_idx;

        std::string STOP_SOUND_PATH;
        std::string RECOVERY_SOUND_PATH;
        std::string TASK_STOP_SOUND_PATH;

        nav_msgs::Odometry odom;
};
