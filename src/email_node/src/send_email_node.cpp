#include <std_msgs/Byte.h>
#include "ros/ros.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include </usr/include/Poco/Net/MailMessage.h>
#include </usr/include/Poco/Net/MailRecipient.h>
#include </usr/include/Poco/Net/SMTPClientSession.h>
#include </usr/include/Poco/Net/NetException.h>
#include "/usr/include/Poco/Types.h"
#include "/usr/include/Poco/Net/Net.h"

using namespace std;
using namespace Poco::Net;
using namespace Poco;

ros::NodeHandle *nh_ptr;

void chatterCallback(const std_msgs::Byte::ConstPtr& msg){
  int intruder_detected=msg->data;

  string host = "mail.domain.com";
  string user = "mads";
  string password = "ThomasIsLate";
  UInt16 port = 25;
  string to = "ridep1security@gmail.com";
  string from = "ridep1security@gmail.com";
  string subject = "Intruder detected";
  subject = MailMessage::encodeWord(subject, "UTF-8");
  string content = "An intruder has been detected, the suspect must be apprehended.";
  MailMessage message;
  message.setSender(from);
  message.addRecipient(MailRecipient(MailRecipient::PRIMARY_RECIPIENT, to));
  message.setSubject(subject);
  message.setContentType("text/plain; charset=UTF-8");
  message.setContent(content, MailMessage::ENCODING_8BIT);

    if(intruder_detected==1)
      {
          try
          {
              SMTPClientSession session("localhost", 25);
              session.open();
              try
              {
                  session.login(SMTPClientSession::AUTH_LOGIN, user, password);
                  session.sendMessage(message);
                  cout << "Message successfully sent" << endl;
                  session.close();
              }
              catch (SMTPException &e)
              {
                  cerr << e.displayText() << endl;
                  session.close();
              }
          }
          catch (NetException &e)
          {
              cerr << e.displayText() << endl;
          }
        }
    }

int main(int argc, char **argv)
{
 ros::init(argc, argv, "email_node");

 ros::NodeHandle nh;
 nh_ptr = &nh;

 ros::Subscriber sub = nh.subscribe("email/send", 1000, chatterCallback);
 ros::spin();

return 0;
}
