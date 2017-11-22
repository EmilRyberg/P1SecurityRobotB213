#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

sensor_msgs::Image currentFrame;

void ImageRawCallback(const sensor_msgs::ImageConstPtr dataPtr);

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "image_recognition_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(5);
    ros::Subscriber bumper_sub = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);

	while(ros::ok())
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(currentFrame, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat cv_image = cv_ptr->image;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void ImageRawCallback(const sensor_msgs::ImageConstPtr dataPtr)
{
	currentFrame = *dataPtr;
}