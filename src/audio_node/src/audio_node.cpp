/* Copyright (C) 2017 Emil Albin Ryberg, Albert Sonne Olesen,
 * Nikolaj Binder Amtoft, Thomas Deuffic, 
 * Benedek Gergály, Jonas Brødholt, Mads Riis Thomsen - All Rights Reserved
 * 
 * You may use, distribute and modify this code under the
 * terms of the MIT license. */

#include <unistd.h>
#include <std_msgs/Byte.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>

ros::NodeHandle *nh_ptr;
sound_play::SoundClient *sc_ptr;

//Function is to pass an amount of time
void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh_ptr->ok())
    sleep(t);
}

//Function to play sounds according to the message recieved
void playSound(const std_msgs::ByteConstPtr &msg)
{
  int sound_play = msg->data;
  //Acts according to the msg recieved
  switch (sound_play)
  {
    case 0:
      sc_ptr->say("You have proper clearance, move along citizen");
    break;

    case 1:
      sc_ptr->say("Intruder Detected, A higher authority will be here soon");
       // waits 5 seconds via sleepok function
      sleepok(5, *nh_ptr);
      sc_ptr->playWave("/opt/ros/kinetic/share/sound_play/sounds/Siren_Noise-KevanGC-1337458893.wav");
    break;

    case 2:
      sc_ptr->say("Human detected. Please show identification");
    break;
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
  ros::Subscriber sub = nh.subscribe("audio/play_sound", 1000, playSound);
  ros::spin();

  return 0;
}
