#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

using namespace std;

enum class RobotMode : int
{
	PATROL,
	CHASE
};

RobotMode robot_mode = RobotMode::PATROL; //Patrol
bool human_detected_last_frame = false;
bool human_detected_this_frame = false;
uint8_t human_current_position = 0;

void FoundHumanCallback(const std_msgs::BoolConstPtr& msg);
void HumanPositionCallback(const std_msgs::UInt8ConstPtr& msg);

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "robot_master_node");
	ros::NodeHandle nh;
	robot_mode = RobotMode::PATROL;

	ros::Rate loop_rate(100);
	ros::Subscriber found_human_subscriber = nh.subscribe("color_detection/found_human", 1000, FoundHumanCallback);
	ros::Subscriber human_position_subscriber = nh.subscribe("color_detection/human_position", 1000, HumanPositionCallback);
	//ros::Publisher qr_code_publisher = nh.advertise<std_msgs::String>("qr_reader/qr_code/data", 100);

	while(ros::ok())
	{
		switch(robot_mode)
		{
			case RobotMode::PATROL:
				//Do patrol navigation stuff here

				break;
			case RobotMode::CHASE:
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void FoundHumanCallback(const std_msgs::BoolConstPtr& msg)
{

}

void HumanPositionCallback(const std_msgs::UInt8ConstPtr& msg)
{
	
}