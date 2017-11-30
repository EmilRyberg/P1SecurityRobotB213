#include "/opt/ros/kinetic/include/ros/ros.h"
#include "/opt/ros/kinetic/include/std_msgs/Byte.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Byte>("audio/play_sound", 1000);

  ros::Rate loop_rate(15);

  while (ros::ok())
  {
    std_msgs::Byte msg;
    msg.data = 0;

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
