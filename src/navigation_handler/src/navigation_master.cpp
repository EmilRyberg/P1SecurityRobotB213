#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"

void bumperCallback(const kobuki_msgs::BumperEventConstPtr& event);
void avoidObstacle(ros::Publisher vel_pub);

int bumperState = 0;
bool rotating = false;
ros::Rate *loop_rate_ptr;
geometry_msgs::Twist twistMsg;

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "navigation_master");

	ros::NodeHandle nh;

	ros::Rate loop_rate(10);
	loop_rate_ptr = &loop_rate;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
	twistMsg.linear.x = 0.0;
    twistMsg.angular.z = 0.0;
    
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 1000, bumperCallback);

	while(ros::ok())
	{
		ros::spinOnce();
		if(bumperState == 0)
		{
			twistMsg.angular.z = 0.0;
			twistMsg.linear.x = 0.5;
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
	bumperState = event->state;
}

void avoidObstacle(ros::Publisher vel_pub)
{
	ROS_INFO("Avoiding obstacle");
	int actionCountdown = 25;
	while(actionCountdown > 0)
	{
		ROS_INFO("Going backwards");
		twistMsg.linear.x = -0.2;
		twistMsg.angular.z = 0.0;
		actionCountdown--;
		vel_pub.publish(twistMsg);
		loop_rate_ptr->sleep();
	}
	actionCountdown = 50;
	twistMsg.linear.x = 0.0;

	while(actionCountdown > 0)
	{
		ROS_INFO("Turning");
		twistMsg.linear.x = 0.0;
		twistMsg.angular.z = 0.4;
		actionCountdown--;
		vel_pub.publish(twistMsg);
		loop_rate_ptr->sleep();
	}
}