#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include "nav_goal/navigation_goal.h"
#include "nav_goal/navigation_goal_srv.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal *goalPtr;
MoveBaseClient *acPtr;

bool navigationCallback(nav_goal::navigation_goal_srv::Request &input, nav_goal::navigation_goal_srv::Response &output){
    ROS_INFO("got msg");
    goalPtr->target_pose.pose.position.x = input.goal_x;
    goalPtr->target_pose.pose.position.y = input.goal_y;
    goalPtr->target_pose.pose.orientation.w = input.goal_orientation;

    ROS_INFO("Sending goal");
    acPtr->sendGoal(*goalPtr);
    acPtr->waitForResult();

    if(acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Goal reached");
      output.result = 1;
    }
    else{
      ROS_INFO("Navigation failed!");
      output.result = 0;
    }

    return 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_translator");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("navigation_goal",navigationCallback);

    ROS_INFO("test");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    acPtr = &ac;

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goalPtr = &goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    ros::spin();

    return 0;
}
