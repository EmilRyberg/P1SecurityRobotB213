/* Copyright (C) 2017 Emil Albin Ryberg, Albert Sonne Olesen,
 * Nikolaj Binder Amtoft, Thomas Deuffic,
 * Benedek Gergály, Jonas Brødholt, Mads Riis Thomsen - All Rights Reserved
 *
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * playSound function contains code from:
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Code inspired from: http://docs.ros.org/diamondback/api/sound_play/html/test_8cpp_source.html
 */

#include <unistd.h>
#include <std_msgs/Byte.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>

ros::NodeHandle *nh_ptr;
sound_play::SoundClient *sc_ptr;

//Function is to pass an amount of time
void sleepok(int t)
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
      sleepok(5);
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
