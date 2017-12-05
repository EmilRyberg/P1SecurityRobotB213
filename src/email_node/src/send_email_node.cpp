#include <iostream>
#include <std_msgs/Byte.h>
#include <stdio.h>
#include <string.h>
#include <curl/curl.h>
#include "curlpp/cURLpp.hpp"
#include "curlpp/Options.h"

#define FROM    "<TurtleSecurity@gmail.com>"
#define TO      "<TurtleSecurity@gmail.com>"
#define CC      "<info@example.org>"

ros::NodeHandle *nh_ptr;

static const char *payload_text[] = {
  "Date: Mon, 29 Nov 2010 21:54:29 +1100\r\n",
  "To: " TO "\r\n",
  "From: " FROM " (Example User)\r\n",
  "Cc: " CC " (Another example User)\r\n",
  "Message-ID: <dcd7cb36-11db-487a-9f3a-e652a9458efd@"
  "rfcpedant.example.org>\r\n",
  "Subject: Intruder has been detected\r\n",
  "\r\n", /* empty line to divide headers from body, see RFC5322 */
  "The body of the message starts here.\r\n",
  "\r\n",
  "It could be a lot of lines, could be MIME encoded, whatever.\r\n",
  "Check RFC5322.\r\n",
  NULL
};

struct upload_status {
  int lines_read;
};

static size_t payload_source(void *ptr, size_t size, size_t nmemb, void *userp)
{
  struct upload_status *upload_ctx = (struct upload_status *)userp;
  const char *data;

  if((size == 0) || (nmemb == 0) || ((size*nmemb) < 1)) {
    return 0;
  }

  data = payload_text[upload_ctx->lines_read];

  if(data) {
    size_t len = strlen(data);
    memcpy(ptr, data, len);
    upload_ctx->lines_read++;

    return len;
  }

  return 0;
}

void chatterCallback(const std_msgs::Byte::ConstPtr& msg){
  int my_counter=msg->data;

  CURL *curl;
  CURLcode res = CURLE_OK;
  struct curl_slist *recipients = NULL;
  struct upload_status upload_ctx;

  upload_ctx.lines_read = 0;

  curl = curl_easy_init();

    if(my_counter==1){
      /* This is the URL for your mailserver */
      curl_easy_setopt(curl, CURLOPT_URL, "smtp://mail.example.com");

      /* Note that this option isn't strictly required, omitting it will result
       * in libcurl sending the MAIL FROM command with empty sender data. All
       * autoresponses should have an empty reverse-path, and should be directed
       * to the address in the reverse-path which triggered them. Otherwise,
       * they could cause an endless loop. See RFC 5321 Section 4.5.5 for more
       * details.
       */
      curl_easy_setopt(curl, CURLOPT_MAIL_FROM, FROM);

      /* Add two recipients, in this particular case they correspond to the
       * To: and Cc: addressees in the header, but they could be any kind of
       * recipient. */
      recipients = curl_slist_append(recipients, TO);
      recipients = curl_slist_append(recipients, CC);
      curl_easy_setopt(curl, CURLOPT_MAIL_RCPT, recipients);

      /* We're using a callback function to specify the payload (the headers and
       * body of the message). You could just use the CURLOPT_READDATA option to
       * specify a FILE pointer to read from. */
      curl_easy_setopt(curl, CURLOPT_READFUNCTION, payload_source);
      curl_easy_setopt(curl, CURLOPT_READDATA, &upload_ctx);
      curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

      /* Send the message */
      res = curl_easy_perform(curl);

      /* Check for errors */
      if(res != CURLE_OK)
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));

      /* Free the list of recipients */
      curl_slist_free_all(recipients);

      /* curl won't send the QUIT command until you call cleanup, so you should
       * be able to re-use this connection for additional messages (setting
       * CURLOPT_MAIL_FROM and CURLOPT_MAIL_RCPT as required, and calling
       * curl_easy_perform() again. It may not be a good idea to keep the
       * connection open for a very long time though (more than a few minutes
       * may result in the server timing out the connection), and you do want to
       * clean up in the end.
       */
      curl_easy_cleanup(curl);
    }
          return 0;
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
