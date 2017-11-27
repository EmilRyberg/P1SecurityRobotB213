#include </opt/ros/kinetic/include/sound_play/sound_play.h>
#include <unistd.h>
#include "std_msgs/Byte.h"

//Believe this function is to check the amount of time passed
void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
      sleep(t);
}

void chatterCallback(const std_msgs::Byte::ConstPtr& msg){
  if (msg==1) {
    sc.say("You have proper clearance, move along citizen");
  } else {
    sc.say("Intruder Detected, A higher authority will be here soon");
    sleepok(6, nh);
    sc.playWave("Siren_Noise-KevanGC-1337458893.wav");
  }
}

//start of main
int main(int argc, char **argv)
{
 ros::init(argc, argv, "sound_play_test");
 
 //giving handles to specific ros nodes to minize writing time
 ros::-NodeHandle- nh;
 sound_play::SoundClient sc;

 //subscribes to a node
 ros::Subscriber sub = n.subscribe("audio/play_sound", 1000, chatterCallback);
 ros::spin();

return 0;
}
