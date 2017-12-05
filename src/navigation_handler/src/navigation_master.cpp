#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <stdio.h>      // standard input / output functions
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

void bumperCallback(const kobuki_msgs::BumperEventConstPtr& event);
void avoidObstacle(ros::Publisher vel_pub);

int bumper_state = 0;
bool rotating = false;
ros::Rate *loop_rate_ptr;
geometry_msgs::Twist twist_msg;

float range(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "navigation_master");


	int USB = open( "/dev/ttyUSB1", O_RDWR| O_NOCTTY );
	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);
	/* Error Handling */
	if ( tcgetattr ( USB, &tty ) != 0 ) {
		 std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}
	/* Save old tty parameters */
	tty_old = tty;
	/* Set Baud Rate */
	cfsetospeed (&tty, (speed_t)B115200);
	cfsetispeed (&tty, (speed_t)B115200);
	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;
	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
	/* Make raw */
	cfmakeraw(&tty);


	ros::NodeHandle nh;

	ros::Rate loop_rate(10);
	loop_rate_ptr = &loop_rate;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
	twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 1000, bumperCallback);

	while(ros::ok())
	{
		tcflush( USB, TCIFLUSH );
		if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
			 std::cout << "Error " << errno << " from tcsetattr" << std::endl;
		}
		int n = 0,
		spot = 0;
		char buf = '\0';
        char response[1024];
        memset(response, '\0', sizeof response);

		int tmp = 1;
		while (buf != '\r' && n > 0 || tmp==1) {
                n = read( USB, &buf, 1 );
                sprintf( &response[spot], "%c", buf );
                spot += n;
				tmp = 0;
        }

        if (n < 0) {
            std::cout << "Error reading: " << strerror(errno) << std::endl;
        }
        else if (n == 0) {
            std::cout << "Read nothing!" << std::endl;
        }
        else {
            //std::cout << "Response: " << response << std::endl;
        }


        std::string str = response;
        std::vector<int> vect;
        std::stringstream ss(str);
        int i;
        while (ss >> i){
            vect.push_back(i);
            if (ss.peek() == ',')
                    ss.ignore();
        }
        if (vect.at(0) == 0){
             vect.clear();
             vect.push_back(1500);
             vect.push_back(1500);
             vect.push_back(1000);
             vect.push_back(1500);
       }
        /*for (i=0; i< vect.size(); i++){
                std::cout << vect.at(i)<<std::endl;
        }*/
        std::cout<<"CH1: "<<vect.at(0)<<std::endl;
        std::cout<<"CH2: "<<vect.at(1)<<std::endl;
        std::cout<<"CH3: "<<vect.at(2)<<std::endl;
        std::cout<<"CH4: "<<vect.at(3)<<std::endl;
        std::cout<<"%%%%%%"<<std::endl;
        std::cout<<"speed: "<<range(vect.at(1), 1000,2000,-1.0,1.0)<<std::endl;
        std::cout<<"steer: "<<range(vect.at(0), 1000,2000,-1.0,1.0)<<std::endl;
        std::cout<<"=============="<<std::endl;
        float lin_speed = range(vect.at(1), 1000,2000,-1.0,1.0); //CH2 right stick up-down
        float ang_speed = range(vect.at(0), 1000,2000,-1.0,1.0); //CH1 right stick up-down
        if (lin_speed<0.05 && lin_speed>-0.05){
            lin_speed = 0;
        }
        if (ang_speed<0.05 && ang_speed>-0.05){
            ang_speed = 0;
        }


		ros::spinOnce();

		if(bumper_state == 0 || true)
		{
			twist_msg.angular.z = ang_speed;
			twist_msg.linear.x = lin_speed;
		}
		else
		{
			avoidObstacle(vel_pub);
		}
    if (ang_speed != 0 || lin_speed != 0){
		    vel_pub.publish(twist_msg);
    }
		loop_rate.sleep();
	}

	return 0;
}

void bumperCallback(const kobuki_msgs::BumperEventConstPtr& event)
{
	ROS_INFO("Got bumper callback");
	bumper_state = event->state;
}

void avoidObstacle(ros::Publisher vel_pub)
{
	ROS_INFO("Avoiding obstacle");
	int action_countdown = 25;
	while(action_countdown > 0)
	{
		ROS_INFO("Going backwards");
		twist_msg.linear.x = -0.2;
		twist_msg.angular.z = 0.0;
		action_countdown--;
		vel_pub.publish(twist_msg);
		loop_rate_ptr->sleep();
	}
	action_countdown = 50;
	twist_msg.linear.x = 0.0;

	while(action_countdown > 0)
	{
		ROS_INFO("Turning");
		twist_msg.linear.x = 0.0;
		twist_msg.angular.z = 0.4;
		action_countdown--;
		vel_pub.publish(twist_msg);
		loop_rate_ptr->sleep();
	}
}
