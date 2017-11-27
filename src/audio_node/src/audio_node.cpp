#include </opt/ros/kinetic/include/sound_play/sound_play.h>
#include <unistd.h>

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
// My actual code
sleepok(1, nh);
while (nh.ok()) {
  sc.say("Intruder Detected, A higher authority will be here soon");
  sleepok(6, nh);
  const char *siren="Siren_Noise-KevanGC-1337458893.wav";


 sc.say("You have proper clearance, move along citizen")
}









//calls the void, makes the robot be standby for 1 second
 sleepok(1, nh);

 while(nh.ok())
   {
     //Says hello world and then waits 2 seconds
     sc.say("Hello world!");
     sleepok(2, nh);

     //Repeats the string for 4 seconds before it stops
     const char *str1 = "I am annoying.";
     sc.repeat(str1);
     sleepok(4, nh);
     sc.stopSaying(str1);

     //plays a .wav file
     sc.playWave("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
     sleepok(2, nh);

     //play a .wav and stops it after t seconds
     const char *str2 = "/usr/share/xemacs21/xemacs-packages/etc/sounds/piano-beep.wav";
     sc.startWave(str2);
     sleepok(4, nh);
     sc.stopWave(str2);

     //plays sounds from the sound_play package
     sc.play(sound_play::SoundRequest::NEEDS_UNPLUGGING);
     sleepok(2, nh);

     //plays sound from sound_play package for 4 seconds
     sc.start(sound_play::SoundRequest::BACKINGUP);
     sleepok(4, nh);
     sc.stop(sound_play::SoundRequest::BACKINGUP);

                 sleepok(2, nh);
                 sound_play::Sound s1 = sc.waveSound("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
                 s1.repeat();
                 sleepok(1, nh);
                 s1.stop();

                 sleepok(2, nh);
                 sound_play::Sound s2 = sc.voiceSound("This is a really long sentence that will get cut off.");
                 s2.play();
                 sleepok(1, nh);
                 s2.stop();

                 sleepok(2, nh);
                 sound_play::Sound s3 = sc.builtinSound(sound_play::SoundRequest::NEEDS_UNPLUGGING_BADLY);
                 s3.play();
                 sleepok(1, nh);
                 s3.stop();

                 sleepok(2, nh);
                 sound_play::Sound s4 = sc.waveSoundFromPkg("sound_play", "sounds/BACKINGUP.ogg");
                 s4.play();
                 sleepok(1, nh);
                 s4.stop();
   }
}
