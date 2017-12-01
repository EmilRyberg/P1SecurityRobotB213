#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "nav_goal/navigation_goal.h"
#include <string>
#include <sstream>

using namespace std;
using namespace ros;

enum class RobotMode : int
{
	PATROL,
	CHASE
};

RobotMode robot_mode = RobotMode::PATROL; //Patrol
bool human_detected_last_frame = false;
bool human_detected_this_frame = false;
bool robot_is_moving = false;
uint8_t human_current_position = 0;
int setup_iterations = 0;

void FoundHumanCallback(const std_msgs::BoolConstPtr &msg);
void HumanPositionCallback(const std_msgs::UInt8ConstPtr &msg);
pid_t StartRecordVideo(string video_directory);
void StopRecordVideo(pid_t PID);

int main(int argc, char *argv[])
{
	init(argc, argv, "robot_master_node");
	NodeHandle nh;
	robot_mode = RobotMode::PATROL;

	Rate loop_rate(100);
	Subscriber found_human_subscriber = nh.subscribe("color_detection/found_human", 1000, FoundHumanCallback);
	Subscriber human_position_subscriber = nh.subscribe("color_detection/human_position", 1000, HumanPositionCallback);
	Publisher navigation_publisher = nh.advertise<nav_goal::navigation_goal>("/navigation_goal", 100);

	while (ros::ok())
	{
		ros::spinOnce();
		if (setup_iterations < 50)
		{
			setup_iterations++;
		}
		else
		{
			switch (robot_mode)
			{
			case RobotMode::PATROL:
				//Do patrol navigation stuff here
				if (!robot_is_moving)
				{
					ROS_INFO("Moving robot");
					robot_is_moving = true;
					nav_goal::navigation_goal first_goal;
					first_goal.goal_x = 1.5;
					first_goal.goal_y = -5.58;
					first_goal.goal_orientation = 1.0;
					navigation_publisher.publish(first_goal);
					ROS_INFO("Published");
				}
				break;
			case RobotMode::CHASE:
				break;
			}
		}
		loop_rate.sleep();
	}

	return 0;
}

void FoundHumanCallback(const std_msgs::BoolConstPtr &msg)
{
}

void HumanPositionCallback(const std_msgs::UInt8ConstPtr &msg)
{
}

pid_t StartRecordVideo(string video_directory)
{
	pid_t PID = fork();
	if (PID == 0)
	{
		chdir(video_directory.c_str());
		execlp("rosrun", "rosrun", "image_view", "video_recorder", "image:=/camera/rgb/image_raw", NULL);
		ROS_INFO("Recording started");
		exit(1);
	}
	return PID;
}

void StopRecordVideo(pid_t PID)
{
	kill(PID, 15);
	ROS_INFO("Recording stopped");

}