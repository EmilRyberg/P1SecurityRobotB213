/* Copyright (C) 2017 Emil Albin Ryberg, Albert Sonne Olesen,
 * Nikolaj Binder Amtoft, Thomas Deuffic, 
 * Benedek Gergály, Jonas Brødholt, Mads Riis Thomsen - All Rights Reserved
 * 
 * You may use, distribute and modify this code under the
 * terms of the MIT license. */

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

//Struct to hold the information for the navigation waypoints
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

//Constants used for waiting for specific things
const int kWaitForIDSeconds = 30;
const int kRecordTimeSeconds = 60;
const int kCooldownSeconds = 30;
const int kUpdateFrequency = 100;

//Global variables to keep track of the robots' different states and data
bool human_detected_last_frame = false;
bool human_detected_this_frame = false;
bool robot_is_moving = false;
bool waiting_for_authorization = false;
bool continue_moving = false;
bool got_to_waypoint = false;
bool recording_video = false;
bool detect_humans = true;
uint8_t human_current_position = 0;
int setup_iterations = 0;
bool qr_code_updated = false;
string qr_code_data = "";
int current_wait_time = 0;
int current_record_time = 0;
int current_cooldown_time = 0;
pid_t video_record_process = 0;
int current_navigation_waypoint = 0;

//The navigation waypoints the robot will move between
vector<NavigationGoal> navigation_waypoints = {
	NavigationGoal(0.71f, 4.34f, 0.1f),
	NavigationGoal(1.5f, -5.58f, 0.1f),
	NavigationGoal(1.44f, -1.61f, 0.1f),
	NavigationGoal(1.5f, -5.58f, 0.1f),
	NavigationGoal(0.71f, 4.34f, 0.1f)
};

//Const pointer typedef for the navigation_goal message
typedef boost::shared_ptr<nav_goal::navigation_goal const> navigation_goalConstPtr;

//Function declarations for later implementation
void FoundHumanCallback(const std_msgs::BoolConstPtr &msg);
void HumanPositionCallback(const std_msgs::UInt8ConstPtr &msg);
pid_t StartRecordVideo(string video_directory);
void StopRecordVideo(pid_t PID);
void NavigationResultCallback(const navigation_goalConstPtr &msg);
void QRCodeResultCallback(const std_msgs::StringConstPtr &msg);
void IntruderDetected(const Publisher &sound_play_publisher, const Publisher &email_publisher);
void HumanAuthorised(const Publisher &sound_play_publisher);
void MoveToWaypoint(const NavigationGoal &goal, const Publisher &navigation_publisher);

int main(int argc, char *argv[])
{
	init(argc, argv, "robot_master_node");
	NodeHandle nh;

	Rate loop_rate(kUpdateFrequency);

	//Setting up subscribers and publishers
	Subscriber found_human_subscriber = nh.subscribe("color_detection/found_human", 1000, FoundHumanCallback);
	Subscriber human_position_subscriber = nh.subscribe("color_detection/human_position", 1000, HumanPositionCallback);
	Subscriber navigation_goal_subscriber = nh.subscribe("navigation_result", 1000, NavigationResultCallback);
	Subscriber qr_code_subscriber = nh.subscribe("qr_reader/qr_code/data", 100, QRCodeResultCallback);
	Publisher navigation_publisher = nh.advertise<nav_goal::navigation_goal>("navigation_goal", 100);
	Publisher sound_play_publisher = nh.advertise<std_msgs::Byte>("audio/play_sound", 100);
	Publisher email_publisher = nh.advertise<std_msgs::Byte>("email/send", 10);

	while (ros::ok())
	{
		ros::spinOnce();
		if (setup_iterations < 50)
		{
			//Wait for 50 iterations to make sure all topics are connected properly
			setup_iterations++;
		}
		else
		{
			//Checks if the robot is not moving and not waiting for authentication
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
					MoveToWaypoint(new_goal, navigation_publisher);
				}
				else
				{
					//Initial movement
					NavigationGoal new_goal = navigation_waypoints[0];
					MoveToWaypoint(new_goal, navigation_publisher);
				}
			}

			//Runs if the robot should detect humans
			if (detect_humans)
			{
				if (human_detected_this_frame && human_detected_last_frame && !waiting_for_authorization)
				{
					ROS_INFO("Human detected");

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

					if (!recording_video)
					{
						video_record_process = StartRecordVideo("/tmp");
					}
				}
			}
			else
			{
				current_cooldown_time++;
				int current_cooldown_seconds = (int)floor((float)current_cooldown_time / (float)kUpdateFrequency);

				if (current_cooldown_seconds >= kCooldownSeconds)
				{
					detect_humans = true;
					current_cooldown_time = 0;
				}
			}

			//If the robot is waiting for authorization
			if (waiting_for_authorization)
			{
				current_wait_time++;
				int current_wait_time_seconds = (int)floor((float)current_wait_time / (float)kUpdateFrequency);

				if (current_wait_time_seconds >= 30)
				{
					IntruderDetected(sound_play_publisher, email_publisher);
				}
				else
				{
					if (qr_code_updated)
					{
						if (qr_code_data == "Authorize")
						{
							HumanAuthorised(sound_play_publisher);
						}
						else if (qr_code_data != "No data")
						{
							IntruderDetected(sound_play_publisher, email_publisher);
						}
					}
				}
			}

			if (recording_video)
			{
				current_record_time++;
				int current_record_seconds = (int)floor((float)current_record_time / (float)kUpdateFrequency);

				if (current_record_seconds >= kRecordTimeSeconds)
				{
					StopRecordVideo(video_record_process);
				}
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

//Function which starts recording video on another thread
pid_t StartRecordVideo(string video_directory)
{
	if (!recording_video)
	{
		recording_video = true;
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
}

//Stops recording video by sending SIGINT (CTRL + C) signal to recording thread
void StopRecordVideo(pid_t PID)
{
	if (recording_video)
	{
		kill(PID, 2);
		ROS_INFO("Recording stopped");
		video_record_process = 0;
		recording_video = false;
	}
}

//Callback from the navigation results
void NavigationResultCallback(const navigation_goalConstPtr &msg)
{
	got_to_waypoint = msg->result_of_navigation;
	robot_is_moving = false;
}

//Callback for the QR data
void QRCodeResultCallback(const std_msgs::StringConstPtr &msg)
{
	qr_code_updated = true;
	qr_code_data = msg->data;
}

void IntruderDetected(const Publisher &sound_play_publisher, const Publisher &email_publisher)
{
	ROS_INFO("Intruder detected!!");
	std_msgs::Byte sound_play_msg;
	sound_play_msg.data = 1;
	sound_play_publisher.publish(sound_play_msg);

	//Send email
	std_msgs::Byte send_email_message;
	send_email_message.data = 1;
	email_publisher.publish(send_email_message);

	waiting_for_authorization = false;
	continue_moving = true;
	current_wait_time = 0;
}

void HumanAuthorised(const Publisher &sound_play_publisher)
{
	ROS_INFO("Authorized! :)");
	std_msgs::Byte sound_play_msg;
	sound_play_msg.data = 0;
	sound_play_publisher.publish(sound_play_msg);

	waiting_for_authorization = false;
	continue_moving = true;
	current_wait_time = 0;
	detect_humans = false;
	StopRecordVideo(video_record_process);
}

void MoveToWaypoint(const NavigationGoal &goal, const Publisher &navigation_publisher)
{
	ROS_INFO("Moving robot");
	robot_is_moving = true;
	nav_goal::navigation_goal navigation_goal;
	navigation_goal.goal_x = goal.x;
	navigation_goal.goal_y = goal.y;
	navigation_goal.goal_orientation = goal.orientation;
	navigation_goal.navigation_abort_override = false;
	navigation_publisher.publish(navigation_goal);
}