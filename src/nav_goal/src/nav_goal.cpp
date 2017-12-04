#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_goal/navigation_goal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal *goalPtr;
MoveBaseClient *action_clientPtr;

bool received_new_goal_flag = 0;

void navigationCallback(const nav_goal::navigation_goal::ConstPtr& input){
    ROS_INFO("got msg");
    goalPtr->target_pose.pose.position.x = input->goal_x;
    goalPtr->target_pose.pose.position.y = input->goal_y;
    goalPtr->target_pose.pose.orientation.w = input->goal_orientation;

    ROS_INFO("Sending goal");
    action_clientPtr->sendGoal(*goalPtr);
    received_new_goal_flag = 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_translator");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("navigation_goal",1,navigationCallback);
    ros::Publisher pub = n.advertise<nav_goal::navigation_goal>("navigation_result",10);
    nav_goal::navigation_goal output;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient action_client("move_base", true);
    action_clientPtr = &action_client;
    //wait for the action server to come up
    while(!action_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goalPtr = &goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    ros::Rate loop_rate(10);

    while(ros::ok()){
          if(action_clientPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && received_new_goal_flag == 1){
                ROS_INFO("Goal reached");
                output.result_of_navigation = 1;
                pub.publish(output);
                received_new_goal_flag = 0;
          }else if((action_clientPtr->getState() == actionlib::SimpleClientGoalState::REJECTED || action_clientPtr->getState() ==
                                                            actionlib::SimpleClientGoalState::ABORTED) && received_new_goal_flag == 1){
                ROS_INFO("Navigation failed!");
                output.result_of_navigation = 0;
                pub.publish(output);
                received_new_goal_flag = 0;
          }
          ros::spinOnce();
          loop_rate.sleep();
    }

    return 0;
}
