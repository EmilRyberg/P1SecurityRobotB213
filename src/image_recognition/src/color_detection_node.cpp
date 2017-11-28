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
const ros::Publisher *publisher_ptr; //publisher for the master
const ros::Publisher *image_threshold_publisher_ptr; //publisher for the master

void ImageRawCallback(const sensor_msgs::ImageConstPtr &data_ptr); //declaration, used later

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "color_detection_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(2);
	ros::Subscriber camera_subscriber = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);
	ros::Publisher publisher = nh.advertise<std_msgs::String>("color_detection/found_human", 100);
	ros::Publisher image_threshold_publisher = nh.advertise<sensor_msgs::Image>("color_detection/camera_processed/thresholded_image", 100);
	publisher_ptr = &publisher;
	image_threshold_publisher_ptr = &image_threshold_publisher;

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//Copying the image from the camera so we can edit the image frame
//sensor_msgs::ImageConstPtr& type = constant reference pointer that points to the data from the camera
void ImageRawCallback(const sensor_msgs::ImageConstPtr &data_ptr)
{
	current_frame = data_ptr; //Making current_frame look into the same location as the data_ptr.
	cv_bridge::CvImagePtr cv_ptr;

	//Tries to copy the data from the location current_frame is looking into.
	//sensor_msgs::image_encodings::BGR8 Tells the CvImagePtr what format it is working with.
	try
	{
		cv_ptr = cv_bridge::toCvCopy(current_frame, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e) //If it cannot get any data it gives an error
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat cv_image = cv_ptr->image;			 //makes a CvImage(Mat) format copy of the picture
	Mat img_hsv;							 //Declares an empty picturefile of mat format
	cvtColor(cv_image, img_hsv, CV_BGR2HSV); //Converts the image to grayscale and puts it into imgout
	int iLowH = 170;
	int iHighH = 179;
	int iLowS = 150;
	int iHighS = 255;
	int iLowV = 60;
	int iHighV = 255;
	int iLastX = -1;
	int iLastY = -1;

	//Create a black image with the size as the camera output
	Mat img_lines = Mat::zeros(img_hsv.size(), CV_8UC3);
	Mat img_thresholded;

	inRange(img_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), img_thresholded); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//Calculate the moments of the thresholded image
	Moments oMoments = moments(img_thresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
	if (dArea > 10000)
	{
		//calculate the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;

		if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		{
			//Draw a red line from the previous point to the current point
			line(img_lines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
		}

		iLastX = posX;
		iLastY = posY;
	}

	cv_image = cv_image + img_lines;
	Mat threshold_with_lines;
	cvtColor(img_thresholded, threshold_with_lines, CV_GRAY2BGR);
	cv_bridge::CvImage ros_img_thresholded;
	ros_img_thresholded.header = cv_ptr->header;
	ros_img_thresholded.encoding = sensor_msgs::image_encodings::BGR8;
	ros_img_thresholded.image = img_thresholded;

	image_threshold_publisher_ptr->publish(ros_img_thresholded.toImageMsg());

	//publisher_ptr->publish(string_msg); //Publish the string to master
}