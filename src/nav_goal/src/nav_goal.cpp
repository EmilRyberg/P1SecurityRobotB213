/* Copyright (C) 2017 Emil Albin Ryberg, Albert Sonne Olesen,
 * Nikolaj Binder Amtoft, Thomas Deuffic, 
 * Benedek Gergály, Jonas Brødholt, Mads Riis Thomsen - All Rights Reserved
 * 
 * You may use, distribute and modify this code under the
 * terms of the MIT license. */

//The serial port reading part's author is Stackoverflow user Lunatic999
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_goal/navigation_goal.h"

#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdio.h>   // standard input / output functions
#include <string.h>  // string function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal *goalPtr;
MoveBaseClient *action_clientPtr;

geometry_msgs::Twist twist_msg;

ros::Publisher vel_pub; //velocity publisher

bool  received_new_goal_flag = 0;         //used to ensure only 1 response is sent back per goal
bool  bumper_override_flag = 0;           //used for obstacle avoidance in override mode
bool  received_first_goal_flag = 0;       //so that after override it won't try to resume navigation to (0;0)
int   rc_override_flag = 0;               //0=normal, 1=override, 2=finished override resume navigation
float goal_x_buffer, goal_y_buffer, goal_orientation_buffer = 0; //goal buffer for resuming navigation

float range(float x, float in_min, float in_max, float out_min, float out_max) { //scaling RC input to correct values
      return((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void navigationCallback(const nav_goal::navigation_goal::ConstPtr&input) {
      ROS_INFO("got msg");
      received_first_goal_flag = 1;
      goalPtr->target_pose.pose.position.x    = input->goal_x; //tranferring the coordinates from the input message to output message
      goalPtr->target_pose.pose.position.y    = input->goal_y;
      goalPtr->target_pose.pose.orientation.w = input->goal_orientation;

      goal_x_buffer           = input->goal_x;
      goal_y_buffer           = input->goal_y;
      goal_orientation_buffer = input->goal_orientation;

      if (input->navigation_abort_override == 1) {
            ROS_INFO("Aborting");
            action_clientPtr->cancelGoal();
      }
      else if (rc_override_flag == 0) {
            ROS_INFO("Sending goal");
            action_clientPtr->sendGoal(*goalPtr);
            received_new_goal_flag = 1;
      }
}

void bumperCallback(const kobuki_msgs::BumperEventConstPtr&event) { //roll back a bit on bumper hit while in RC override only
      if (rc_override_flag == 1) {
            twist_msg.linear.x  = -0.1;
            twist_msg.angular.z = 0;
            vel_pub.publish(twist_msg);
            ros::Duration(0.1).sleep();
            twist_msg.linear.x = 0;
            vel_pub.publish(twist_msg);
      }
}

int main(int argc, char **argv) {
      ros::init(argc, argv, "navigation_translator");

      //Serial port stuff below for RC control
      int            USB = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
      struct termios tty;
      struct termios tty_old;
      memset(&tty, 0, sizeof tty);
      if (tcgetattr(USB, &tty) != 0) {
            std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
      }
      tty_old = tty;
      cfsetospeed(&tty, (speed_t)B115200);
      cfsetispeed(&tty, (speed_t)B115200);
      tty.c_cflag    &= ~PARENB; // Make 8n1
      tty.c_cflag    &= ~CSTOPB;
      tty.c_cflag    &= ~CSIZE;
      tty.c_cflag    |= CS8;
      tty.c_cflag    &= ~CRTSCTS;       // no flow control
      tty.c_cc[VMIN]  = 1;              // read doesn't block
      tty.c_cc[VTIME] = 5;              // 0.5 seconds read timeout
      tty.c_cflag    |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
      cfmakeraw(&tty);

      ros::NodeHandle n;

      //navigation goal and result communication setup
      ros::Subscriber           navigation_sub = n.subscribe("navigation_goal", 1, navigationCallback);
      ros::Publisher            navigation_pub = n.advertise <nav_goal::navigation_goal>("navigation_result", 10);
      nav_goal::navigation_goal output;

      //RC override setup
      ros::Publisher vel_pub_tmp = n.advertise <geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
      vel_pub             = vel_pub_tmp;
      twist_msg.linear.x  = 0.0;
      twist_msg.angular.z = 0.0;
      ros::Subscriber bumper_sub = n.subscribe("mobile_base/events/bumper", 1000, bumperCallback);

      //tell the action client that we want to spin a thread by default
      MoveBaseClient action_client("move_base", true);
      action_clientPtr = &action_client;
      while (!action_client.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;
      goalPtr = &goal;
      goalPtr->target_pose.header.frame_id = "/map";
      goalPtr->target_pose.header.stamp = ros::Time::now();

      ros::Rate loop_rate(10);

      while (ros::ok()) {
            //
            if (action_clientPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && received_new_goal_flag == 1) {
                  ROS_INFO("Goal reached");
                  output.result_of_navigation = 1;
                  navigation_pub.publish(output);
                  received_new_goal_flag = 0;
            }
            else if ((action_clientPtr->getState() == actionlib::SimpleClientGoalState::REJECTED || action_clientPtr->getState() ==
                                                      actionlib::SimpleClientGoalState::ABORTED) && received_new_goal_flag == 1) {
                  ROS_INFO("Navigation failed");
                  output.result_of_navigation = 0;
                  navigation_pub.publish(output);
                  received_new_goal_flag = 0;
            }

            //RC serial port stuff
            tcflush(USB, TCIFLUSH);
            if (tcsetattr(USB, TCSANOW, &tty) != 0) {
                  //std::cout << "Error " << errno << " from tcsetattr" << std::endl;
            }
            int n    = 0,
                spot = 0;
            char buf = '\0';
            char response[1024];
            memset(response, '\0', sizeof response);
            int tmp = 1;
            while (buf != '\r' && n > 0 || tmp == 1) {
                  n = read(USB, &buf, 1);
                  sprintf(&response[spot], "%c", buf);
                  spot += n;
                  tmp   = 0;
            }
            if (n < 0) {
                  //std::cout << "Error reading: " << strerror(errno) << std::endl;
            }
            else if (n == 0) {
                  //std::cout << "Read nothing!" << std::endl;
            }
            else{
                  //std::cout << "Response: " << response << std::endl;
            }
            std::string       str = response;
            std::vector <int> vect;
            std::stringstream ss(str);
            int i;
            while (ss >> i) {
                  vect.push_back(i);
                  if (ss.peek() == ',') {
                        ss.ignore();
                  }
            }
            if (vect.size() == 0) { //fill with default values if nothing is received
                  vect.push_back(1500);
                  vect.push_back(1500);
                  vect.push_back(1000);
                  vect.push_back(1500);
            }
            //     std::cout<<"CH1: "<<vect.at(0)<<std::endl;
            //     std::cout<<"CH2: "<<vect.at(1)<<std::endl;
            //     std::cout<<"CH3: "<<vect.at(2)<<std::endl;
            //     std::cout<<"CH4: "<<vect.at(3)<<std::endl;
            //     std::cout<<"%%%%%%"<<std::endl;
            //     std::cout<<"speed: "<<range(vect.at(1), 1000,2000,-1.0,1.0)<<std::endl;
            //     std::cout<<"steer: "<<range(vect.at(0), 1000,2000,-1.0,1.0)<<std::endl;
            //     std::cout<<"RC flag"<<rc_override_flag<<std::endl;
            //     std::cout<<"=============="<<std::endl;
            float lin_speed = range(vect.at(1), 1000, 2000, -1.0, 1.0);     //CH2 right stick up-down
            float ang_speed = range(vect.at(0), 1000, 2000, -1.0, 1.0);     //CH1 right stick up-down
            if (lin_speed < 0.05 && lin_speed > -0.05) { //deadband
                  lin_speed = 0;
            }
            if (ang_speed < 0.05 && ang_speed > -0.05) { //deadband
                  ang_speed = 0;
            }
            if (lin_speed != 0 || ang_speed != 0 || vect.at(3) > 1700) { //stop switch on
                  rc_override_flag = 1;
            }
            if (lin_speed == 0 && ang_speed == 0 && rc_override_flag == 1 && vect.at(3) < 1300) { //stop switch off
                  rc_override_flag = 2;
            }

            if (rc_override_flag > 0) {
                  ROS_INFO("Aborting");
                  action_clientPtr->cancelGoal();
                  twist_msg.linear.x  = lin_speed;
                  twist_msg.angular.z = ang_speed;
                  vel_pub.publish(twist_msg);
            }
            if (rc_override_flag == 2) { //resuming navigation, resending goal
                  twist_msg.linear.x  = 0;
                  twist_msg.angular.z = 0;
                  vel_pub.publish(twist_msg);
                  if (received_first_goal_flag == 1) { //dont send (0;0) if no goal has been received
                        goalPtr->target_pose.pose.position.x    = goal_x_buffer;
                        goalPtr->target_pose.pose.position.y    = goal_y_buffer;
                        goalPtr->target_pose.pose.orientation.w = goal_orientation_buffer;
                        action_clientPtr->sendGoal(*goalPtr);
                  }
                  rc_override_flag = 0;
            }

            ros::spinOnce();
            loop_rate.sleep();
      }

      return(0);
}
