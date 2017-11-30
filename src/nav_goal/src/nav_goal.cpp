#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_goal/navigation_goal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float goal_x = 0.0;
float goal_y = 0.0;
float goal_orientation = 0.0;
bool got_msg = 0;

void navigationCallback(const nav_goal::navigation_goal::ConstPtr& msg){
  ROS_INFO("got msg");

  goal_x = msg->goal_x;
  goal_y = msg->goal_y;
  goal_orientation = msg->goal_orientation;
  got_msg = 1;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_translator");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("navigation_goal",1000,navigationCallback);

  ROS_INFO("test");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();


  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;
  goal.target_pose.pose.orientation.w = goal_orientation;
  got_msg = 0; //clear flag

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached");
  else
    ROS_INFO("Navigation failed!");


  ros::spin();

  return 0;
}
