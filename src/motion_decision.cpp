#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class MotionDecision{
	public:
		MotionDecision();
		void LocalPathCallback(const geometry_msgs::TwistConstPtr& msg);
		void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
		void EmergencyStopFlagCallback(const std_msgs::BoolConstPtr& msg);
		void TaskStopFlagCallback(const std_msgs::BoolConstPtr& msg);

		void process();

	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		//subscriber
		ros::Subscriber local_path_sub;
		ros::Subscriber joy_sub;
		ros::Subscriber emergency_stop_flag_sub;
		ros::Subscriber task_stop_flag_sub;

		//publisher
		ros::Publisher vel_pub;
		ros::Publisher intersection_flag_pub;

		bool emergency_stop_flag;
		bool task_stop_flag;
		bool auto_flag;
		bool move_flag;
		bool joy_flag;
		bool intersection_flag;
		geometry_msgs::Twist cmd_vel;
		geometry_msgs::Twist joy_vel;
		sensor_msgs::Joy joy;
		int HZ;
		double MAX_SPEED;
		double MAX_YAWRATE;
		double VEL_RATIO;
};

MotionDecision::MotionDecision()
	:private_nh("~")
{
	//subscriber
	local_path_sub = nh.subscribe("/local_path/cmd_vel",1, &MotionDecision::LocalPathCallback, this);
	joy_sub = nh.subscribe("/joy",1, &MotionDecision::JoyCallback, this);
	emergency_stop_flag_sub = nh.subscribe("/emergency_stop",1, &MotionDecision::EmergencyStopFlagCallback, this);
	task_stop_flag_sub = nh.subscribe("/task/stop",1, &MotionDecision::TaskStopFlagCallback, this);

	//publisher
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
	intersection_flag_pub = nh.advertise<std_msgs::Bool>("/intersection_flag",1,true);

	private_nh.param("HZ", HZ, {20});
	private_nh.param("MAX_SPEED", MAX_SPEED, {1.0});
	private_nh.param("MAX_YAWRATE", MAX_YAWRATE, {1.0});
	private_nh.param("VEL_RATIO", VEL_RATIO, {0.5});

	emergency_stop_flag = false;
	task_stop_flag = false;
	auto_flag = false;
	move_flag = false;
	joy_flag = false;
	intersection_flag = false;

	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
}

void MotionDecision::LocalPathCallback(const geometry_msgs::TwistConstPtr& msg)
{
	cmd_vel = *msg;
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
	task_stop_flag = msg->data;
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
				vel = cmd_vel;
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
		if(task_stop_flag){
			std::cout << "task stop" << std::endl;
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
