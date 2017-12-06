#include </opt/ros/kinetic/include/std_msgs/Byte.h>
#include "ros/ros.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <Poco/Net/MailMessage.h>
#include <Poco/Net/MailRecipient.h>
#include <Poco/Net/SMTPClientSession.h>
#include <Poco/Net/NetException.h>

using namespace std;
using namespace Poco::Net;
using namespace Poco;

void chatterCallback(const std_msgs::Byte::ConstPtr& msg)
{
  int intruder_detected=msg->data;

  string host = "localhost";
  UInt16 port = 25;
  string user = "ridep1project@gmail.com";
  string password = "ThomasIsLate";
  string to = "ridep1project@gmail.com";
  string from = "ridep1project@gmail.com";
  string subject = "Intruder detected";
  subject = MailMessage::encodeWord(subject, "UTF-8");
  string content = "An intruder has been detected. Please apprehend the intruder.";
  MailMessage message;
  message.setSender(from);
  message.addRecipient(MailRecipient(MailRecipient::PRIMARY_RECIPIENT, to));
  message.setSubject(subject);
  message.setContentType("text/plain; charset=UTF-8");
  message.setContent(content, MailMessage::ENCODING_8BIT);

  if (intruder_detected==1)
  {
    try
    {
      SMTPClientSession session(host, port);
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

 ros::Subscriber sub = nh.subscribe("email/send", 1000, chatterCallback);
 ros::spin();

return 0;
}
