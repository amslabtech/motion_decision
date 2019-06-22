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

		void process();

	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		//subscriber
		ros::Subscriber local_path_sub;
		ros::Subscriber joy_sub;
		ros::Subscriber emergency_stop_flag_sub;

		//publisher
		ros::Publisher vel_pub;

		bool emergency_stop_flag = false;
		bool auto_flag = false;
		bool move_flag = false;
		bool joy_flag = false;
		geometry_msgs::Twist cmd_vel;
		sensor_msgs::Joy joy;
		int HZ = 20;
};

MotionDecision::MotionDecision()
	:private_nh("~")
{
	//subscriber
	local_path_sub = nh.subscribe("/local_path/cmd_vel",1, &MotionDecision::LocalPathCallback, this);
	joy_sub = nh.subscribe("/joy",1, &MotionDecision::JoyCallback, this);
	emergency_stop_flag_sub = nh.subscribe("/emergency_stop",1, &MotionDecision::EmergencyStopFlagCallback, this);

	//publisher
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);

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
	if(joy.buttons[8]){ //square button
		auto_flag = false;
	}else if(joy.buttons[9]){ // triangle button
		auto_flag = true;
	}

	if(joy.buttons[0]){ // cross button
		move_flag = false;
	}else if(joy.buttons[1]){ // circle button
		move_flag = true;
	}
	joy_flag = true;
}

void MotionDecision::EmergencyStopFlagCallback(const std_msgs::BoolConstPtr& msg)
{
	emergency_stop_flag = msg->data;
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
				if(joy_flag && joy.buttons[6]){ // L2 button
					vel.linear.x = joy.axes[1];
					vel.angular.z = joy.axes[0];
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
		joy_flag = false;
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
