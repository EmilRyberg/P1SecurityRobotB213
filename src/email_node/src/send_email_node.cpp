#include <iostream>
#include "std_msgs/Byte.h"
#include <Poco/Net/MailMessage.h>
#include <Poco/Net/MailRecipient.h>
#include <Poco/Net/SMTPClientSession.h>
#include <Poco/Net/NetException.h>

using namespace std;
using namespace Poco::Net;
using namespace Poco;

void chatterCallback(const std_msgs::Byte::ConstPtr& msg){
  int my_counter=msg->data;

    string host = "smtp.gmail.com";
    UInt16 port = 25;
    string user = "-username-";
    string password = "-password-";
    string to = "xxx@gmail.com";
    string from = "xxx@domain.com";
    string subject = "Your first e-mail message sent using Poco Libraries";
    subject = MailMessage::encodeWord(subject, "UTF-8");
    string content = "Well done! You've successfully sent your first message using Poco SMTPClientSession";
    MailMessage message;
    message.setSender(from);
    message.addRecipient(MailRecipient(MailRecipient::PRIMARY_RECIPIENT, to));
    message.setSubject(subject);
    message.setContentType("text/plain; charset=UTF-8");
    message.setContent(content, MailMessage::ENCODING_8BIT);

    if(my_counter==1){
      try {
          SMTPClientSession session(host, port);
          session.open();
            try {
              session.login(SMTPClientSession::AUTH_LOGIN, user, password);
              session.sendMessage(message);
              cout << "Message successfully sent" << endl;
              session.close();
            } catch (SMTPException &e) {
              cerr << e.displayText() << endl;
              session.close();
              return 0;
            }
          } catch (NetException &e) {
            cerr << e.displayText() << endl;
            return 0;
          }
          return 0;
        }
    }

    
int main(int argc, char **argv)
{
 ros::init(argc, argv, "audio_node");

 ros::NodeHandle nh;
 nh_ptr = &nh;

 ros::Subscriber sub = nh.subscribe("email/send", 1000, chatterCallback);
 ros::spin();

return 0;
}
