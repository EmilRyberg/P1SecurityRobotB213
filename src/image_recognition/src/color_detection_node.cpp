#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include "opencv2/opencv.hpp"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;
using namespace zbar;

sensor_msgs::ImageConstPtr current_frame; 
const ros::Publisher* publisher_ptr; //publisher for the master

void ImageRawCallback(const sensor_msgs::ImageConstPtr& data_ptr); //declaration, used later

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "color_detection_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(2);
	ros::Subscriber camera_subscriber = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);
	ros::Publisher publisher = nh.advertise<std_msgs::String>("color_detection/found_human", 100);
	publisher_ptr = &publisher;

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//Copying the image from the camera so we can edit the image frame
//sensor_msgs::ImageConstPtr& type = constant reference pointer that points to the data from the camera
void ImageRawCallback(const sensor_msgs::ImageConstPtr& data_ptr)  
{
	current_frame = data_ptr; //Making current_frame look into the same location as the data_ptr.
	cv_bridge::CvImagePtr cv_ptr; 

	//Tries to copy the data from the location current_frame is looking into.
	//sensor_msgs::image_encodings::BGR8 Tells the CvImagePtr what format it is working with.
	try
	{
		cv_ptr = cv_bridge::toCvCopy(current_frame, sensor_msgs::image_encodings::BGR8); 
	}
	catch (cv_bridge::Exception& e) //If it cannot get any data it gives an error
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	
	Mat cv_image = cv_ptr->image; //makes a CvImage(Mat) format copy of the picture
	Mat img_hsv; //Declares an empty picturefile of mat format
	cvtColor(cv_image, img_hsv, CV_BGR2HSV); //Converts the image to grayscale and puts it into imgout
	int width = img_hsv.cols;  
	int height = img_hsv.rows;  
	uchar *raw = (uchar *)img_hsv.data; //Takes the raw data from the grayscale picture
	//publisher_ptr->publish(string_msg); //Publish the string to master
}