#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"

void bumperCallback(const kobuki_msgs::BumperEventConstPtr& event);
void avoidObstacle(ros::Publisher vel_pub);

int bumper_state = 0;
bool rotating = false;
ros::Rate *loop_rate_ptr;
geometry_msgs::Twist twist_msg;

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "navigation_master");

	ros::NodeHandle nh;

	ros::Rate loop_rate(10);
	loop_rate_ptr = &loop_rate;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
	twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 1000, bumperCallback);

	while(ros::ok())
	{
		ros::spinOnce();

		if(bumper_state == 0)
		{
			twist_msg.angular.z = 0.0;
			twist_msg.linear.x = 0.5;
		}
		else
		{
			avoidObstacle(vel_pub);
		}
		
		vel_pub.publish(twistMsg);

		loop_rate.sleep();
	}

	return 0;
}

void bumperCallback(const kobuki_msgs::BumperEventConstPtr& event)
{
	ROS_INFO("Got bumper callback");
	bumper_state = event->state;
}

void avoidObstacle(ros::Publisher vel_pub)
{
	ROS_INFO("Avoiding obstacle");
	int action_countdown = 25;
	while(action_countdown > 0)
	{
		ROS_INFO("Going backwards");
		twist_msg.linear.x = -0.2;
		twist_msg.angular.z = 0.0;
		action_countdown--;
		vel_pub.publish(twistMsg);
		loop_rate_ptr->sleep();
	}
	action_countdown = 50;
	twist_msg.linear.x = 0.0;

	while(action_countdown > 0)
	{
		ROS_INFO("Turning");
		twist_msg.linear.x = 0.0;
		twist_msg.angular.z = 0.4;
		action_countdown--;
		vel_pub.publish(twistMsg);
		loop_rate_ptr->sleep();
	}
}