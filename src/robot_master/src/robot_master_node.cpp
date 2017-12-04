#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <string>
#include "nav_goal/navigation_goal.h"

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
bool got_to_waypoint = false;
uint8_t human_current_position = 0;
int setup_iterations = 0;
bool qr_code_updated = false;
string qr_code_data = "";

typedef boost::shared_ptr<nav_goal::navigation_goal const> navigation_goalConstPtr;

void FoundHumanCallback(const std_msgs::BoolConstPtr &msg);
void HumanPositionCallback(const std_msgs::UInt8ConstPtr &msg);
void NavigationResultCallback(const navigation_goalConstPtr &msg);
void QRCodeResultCallback(const std_msgs::StringConstPtr &msg);

int main(int argc, char *argv[])
{
	init(argc, argv, "robot_master_node");
	NodeHandle nh;

	Rate loop_rate(100);
	Subscriber found_human_subscriber = nh.subscribe("color_detection/found_human", 1000, FoundHumanCallback);
	Subscriber human_position_subscriber = nh.subscribe("color_detection/human_position", 1000, HumanPositionCallback);
	Subscriber navigation_goal_subscriber = nh.subscribe("navigation_result", 1000, NavigationResultCallback);
	Subscriber qr_code_subscriber = nh.subscribe("qr_reader/qr_code/data", 100, QRCodeResultCallback);
	Publisher navigation_publisher = nh.advertise<nav_goal::navigation_goal>("navigation_goal", 100);

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
					else
					{
						if(human_detected_this_frame && human_detected_last_frame)
						{
							//Code here to stop robot and ask for ID

							if(qr_code_updated)
							{
								if(qr_code_data == "Authorized")
								{
									//Do authorized stuff here
								}
								else
								{
									//Start recording and alarm
								}
							}
						}
					}
					break;
				case RobotMode::CHASE:
					break;
			}

			human_detected_last_frame = human_detected_this_frame;
			human_detected_this_frame = false;
			qr_code_updated = false;
		}
		loop_rate.sleep();
	}

	return 0;
}

void FoundHumanCallback(const std_msgs::BoolConstPtr &msg)
{
	human_detected_this_frame = msg->data;
}

void HumanPositionCallback(const std_msgs::UInt8ConstPtr &msg)
{
	human_current_position = msg->data;
}

void NavigationResultCallback(const navigation_goalConstPtr &msg)
{

}

void QRCodeResultCallback(const std_msgs::StringConstPtr &msg)
{
	qr_code_updated = true;
	qr_code_data = msg->data;
}