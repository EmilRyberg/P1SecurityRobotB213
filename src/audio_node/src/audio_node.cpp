#include </opt/ros/kinetic/include/sound_play/sound_play.h>
#include <unistd.h>
#include "std_msgs/Byte.h"

//Believe this function is to check the amount of time passed
void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
      sleep(t);
}

//start of main
int main(int argc, char **argv)
{
 ros::init(argc, argv, "sound_play_test");


 //giving handles to specific ros nodes to minize writing time
 ros::-NodeHandle- nh;
 sound_play::SoundClient sc;

 ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
 std_msgs::Byte message;


// My actual code
sleepok(1, nh);
sc.say("Anomaly detected. Please show your QR code");
sleepok(3, nh);

if (/* QR code is correct */) {
  sc.say("You have proper clearance, move along citizen");
}
else {
sc.say("Intruder Detected, A higher authority will be here soon");
sleepok(6, nh);
sc.playWave("Siren_Noise-KevanGC-1337458893.wav");
}

return 0;
}
