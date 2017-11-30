#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_goal/navigation_goal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void navigationCallback(const nav_goal::navigation_goal::ConstPtr& msg){
  ROS_INFO("input",msg->goal_x);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_translator");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("navigation_goal",1000,navigationCallback);

  ros::spin();

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
while(1){
  goal.target_pose.pose.position.x = -0.43;
  goal.target_pose.pose.position.y = 5.77;
  goal.target_pose.pose.orientation.w = 3.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  goal.target_pose.pose.position.x = 1.34;
  goal.target_pose.pose.position.y = 5.65;
  goal.target_pose.pose.orientation.w = 3.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  goal.target_pose.pose.position.x = 0.59;
  goal.target_pose.pose.position.y = -5.4;
  goal.target_pose.pose.orientation.w = 3.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  goal.target_pose.pose.position.x = 2.46;
  goal.target_pose.pose.position.y = -7.43;
  goal.target_pose.pose.orientation.w = 3.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
}
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
