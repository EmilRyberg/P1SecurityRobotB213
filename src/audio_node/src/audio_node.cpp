#include </opt/ros/kinetic/include/sound_play/sound_play.h>
#include <unistd.h>
#include "/opt/ros/kinetic/include/std_msgs/Byte.h"
#include "/opt/ros/kinetic/include/ros/ros.h"

ros::NodeHandle *nh_ptr;
sound_play::SoundClient *sc_ptr;

//Function is to pass an amount of time
void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh_ptr->ok())
      sleep(t);
}

//Function to play sounds according to the message recieved
void chatterCallback(const std_msgs::Byte::ConstPtr& msg){
int my_counter=msg->data;
//Acts according to the msg recieved
  if (my_counter==1) {
    sc_ptr->say("You have proper clearance, move along citizen");
  } else {
    sc_ptr->say("Intruder Detected, A higher authority will be here soon");
    // waits 5 seconds via sleepok function
    sleepok(5, *nh_ptr);
    sc_ptr->playWave("/opt/ros/kinetic/share/sound_play/sounds/Siren_Noise-KevanGC-1337458893.wav");
  }
}

  //start of main
int main(int argc, char **argv)
{
 ros::init(argc, argv, "audio_node");

 //giving handles to specific ros nodes to minize writing time
 ros::NodeHandle nh;
 nh_ptr = &nh;
 sound_play::SoundClient sc;
 sc_ptr = &sc;

 //subscribes to a node (audio_play_sound)
 ros::Subscriber sub = nh.subscribe("audio/play_sound", 1000, chatterCallback);
 ros::spin();

return 0;
}