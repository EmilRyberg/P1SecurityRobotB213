#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <string>
#include "nav_goal/navigation_goal.h"
#include <string>
#include <sstream>
#include <signal.h>
#include <vector>
#include <memory>
#include <cmath>

using namespace std;
using namespace ros;

enum class RobotMode : int
{
	PATROL,
	CHASE
};

struct NavigationGoal
{
	float x;
	float y;
	float orientation;

	NavigationGoal()
	{
		x = 0.0;
		y = 0.0;
		orientation = 0.0;
	}

	NavigationGoal(float x, float y, float orientation)
	{
		this->x = x;
		this->y = y;
		this->orientation = orientation;
	}
};

const int kWaitForIDSeconds = 30;
const int kUpdateFrequency = 100;

RobotMode robot_mode = RobotMode::PATROL; //Patrol
bool human_detected_last_frame = false;
bool human_detected_this_frame = false;
bool robot_is_moving = false;
bool waiting_for_authorization = false;
bool continue_moving = false;
bool got_to_waypoint = false;
uint8_t human_current_position = 0;
int setup_iterations = 0;
bool qr_code_updated = false;
string qr_code_data = "";
int current_wait_time = 0;
pid_t video_record_process = 0;

vector<NavigationGoal> navigation_waypoints = {
	NavigationGoal(0.71f, 4.34f, 0.1f),
	NavigationGoal(1.5f, -5.58f, 0.1f),
	NavigationGoal(1.44f, -1.61f, 0.1f),
	NavigationGoal(1.5f, -5.58f, 0.1f),
	NavigationGoal(0.71f, 4.34f, 0.1f)};

int current_navigation_waypoint = 0;

typedef boost::shared_ptr<nav_goal::navigation_goal const> navigation_goalConstPtr;

void FoundHumanCallback(const std_msgs::BoolConstPtr &msg);
void HumanPositionCallback(const std_msgs::UInt8ConstPtr &msg);
pid_t StartRecordVideo(string video_directory);
void StopRecordVideo(pid_t PID);
void NavigationResultCallback(const navigation_goalConstPtr &msg);
void QRCodeResultCallback(const std_msgs::StringConstPtr &msg);
void IntruderDetected(const Publisher &sound_play_publisher);

int main(int argc, char *argv[])
{
	init(argc, argv, "robot_master_node");
	NodeHandle nh;

	Rate loop_rate(kUpdateFrequency);
	Subscriber found_human_subscriber = nh.subscribe("color_detection/found_human", 1000, FoundHumanCallback);
	Subscriber human_position_subscriber = nh.subscribe("color_detection/human_position", 1000, HumanPositionCallback);
	Subscriber navigation_goal_subscriber = nh.subscribe("navigation_result", 1000, NavigationResultCallback);
	Subscriber qr_code_subscriber = nh.subscribe("qr_reader/qr_code/data", 100, QRCodeResultCallback);
	Publisher navigation_publisher = nh.advertise<nav_goal::navigation_goal>("navigation_goal", 100);
	Publisher sound_play_publisher = nh.advertise<std_msgs::Byte>("audio/play_sound", 100);

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
				if (!robot_is_moving && !waiting_for_authorization)
				{
					if (got_to_waypoint || continue_moving)
					{
						current_navigation_waypoint++;
						got_to_waypoint = false;

						if (current_navigation_waypoint > navigation_waypoints.size())
						{
							current_navigation_waypoint = 0;
						}

						NavigationGoal new_goal = navigation_waypoints[current_navigation_waypoint];
						ROS_INFO("Moving robot");
						robot_is_moving = true;
						nav_goal::navigation_goal first_goal;
						first_goal.goal_x = new_goal.x;
						first_goal.goal_y = new_goal.y;
						first_goal.goal_orientation = new_goal.orientation;
						first_goal.navigation_abort_override = false;
						navigation_publisher.publish(first_goal);
						ROS_INFO("Published");
					}
					else
					{
						//Initial movement
						NavigationGoal new_goal = navigation_waypoints[0];
						ROS_INFO("Moving robot");
						robot_is_moving = true;
						nav_goal::navigation_goal first_goal;
						first_goal.goal_x = new_goal.x;
						first_goal.goal_y = new_goal.y;
						first_goal.goal_orientation = new_goal.orientation;
						first_goal.navigation_abort_override = false;
						navigation_publisher.publish(first_goal);
						ROS_INFO("Published");
					}
				}

				if (human_detected_this_frame && human_detected_last_frame && !waiting_for_authorization)
				{
					ROS_INFO("Human detected");
					//Code here to stop robot and ask for ID
					nav_goal::navigation_goal navigation_stop;
					navigation_stop.goal_x = 0.0f;
					navigation_stop.goal_y = 0.0f;
					navigation_stop.goal_orientation = 0.0f;
					navigation_stop.navigation_abort_override = true;
					navigation_publisher.publish(navigation_stop);
					robot_is_moving = false;
					waiting_for_authorization = true;

					std_msgs::Byte sound_play_msg;
					sound_play_msg.data = 2;
					sound_play_publisher.publish(sound_play_msg);

					if(video_record_process == 0)
					{
						video_record_process = StartRecordVideo("/tmp");
					}
				}

				if (waiting_for_authorization)
				{
					current_wait_time++;
					int current_wait_time_seconds = (int)floor((float)current_wait_time / (float)kUpdateFrequency);

					if (current_wait_time_seconds >= 30)
					{
						IntruderDetected(sound_play_publisher);
					}
					else
					{
						if (qr_code_updated)
						{
							if (qr_code_data == "Authorize")
							{
								ROS_INFO("Authorized! :)");
								std_msgs::Byte sound_play_msg;
								sound_play_msg.data = 0;
								sound_play_publisher.publish(sound_play_msg);

								waiting_for_authorization = false;
								continue_moving = true;
								current_wait_time = 0;
								StopRecordVideo(video_record_process);
							}
							else if (qr_code_data != "No data")
							{
								IntruderDetected(sound_play_publisher);
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
	kill(PID, 2);
	ROS_INFO("Recording stopped");
	video_record_process = 0;
}

void NavigationResultCallback(const navigation_goalConstPtr &msg)
{
	got_to_waypoint = msg->result_of_navigation;
	robot_is_moving = false;
}

void QRCodeResultCallback(const std_msgs::StringConstPtr &msg)
{
	qr_code_updated = true;
	qr_code_data = msg->data;
}

void IntruderDetected(const Publisher &sound_play_publisher)
{
	ROS_INFO("Intruder detected!!");
	std_msgs::Byte sound_play_msg;
	sound_play_msg.data = 1;
	sound_play_publisher.publish(sound_play_msg);

	waiting_for_authorization = false;
	continue_moving = true;
	StopRecordVideo(video_record_process);
}