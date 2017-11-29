#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;
using namespace zbar;
using namespace ros;

sensor_msgs::ImageConstPtr current_frame;
const Publisher *found_human_publisher_ptr;
const Publisher *image_threshold_publisher_ptr;
const Publisher *image_processed_publisher_ptr;
const Publisher *human_position_publisher_ptr;

int i_last_x = -1;
int i_last_y = -1;
vector<Point> previous_positions;

void ImageRawCallback(const sensor_msgs::ImageConstPtr &data_ptr); //declaration, used later

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "color_detection_node");
	ros::NodeHandle nh;

	Rate loop_rate(15);
	Subscriber camera_subscriber = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);
	Publisher found_human_publisher = nh.advertise<std_msgs::Bool>("color_detection/found_human", 100);
	Publisher human_position_publisher = nh.advertise<std_msgs::UInt8>("color_detection/human_position", 100);
	Publisher image_threshold_publisher = nh.advertise<sensor_msgs::Image>("color_detection/camera_processed/thresholded_image", 100);
	Publisher image_processed_publisher = nh.advertise<sensor_msgs::Image>("color_detection/camera_processed/processed_image", 100);
	found_human_publisher_ptr = &found_human_publisher;
	image_threshold_publisher_ptr = &image_threshold_publisher;
	image_processed_publisher_ptr = &image_processed_publisher;
	human_position_publisher_ptr = &human_position_publisher;

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
	int low_h = 160;
	int high_h = 240;
	int low_s = 40;
	int high_s = 240;
	int low_v = 40;
	int high_v = 240;

	//Create a black image with the size as the camera output
	Mat img_lines = Mat::zeros(img_hsv.size(), CV_8UC3);
	Mat img_thresholded;

	inRange(img_hsv, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), img_thresholded); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//Calculate the moments of the thresholded image
	Moments image_moments = moments(img_thresholded);

	double moment_01 = image_moments.m01;
	double moment_10 = image_moments.m10;
	double area = image_moments.m00;

	bool found_human = false;
	uint8_t human_position = -1;

	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
	if (area > 10000)
	{
		found_human = true;

		//calculate the position of the ball
		int pos_x = moment_10 / area;
		int pos_y = moment_01 / area;

		//ROS_INFO_STREAM("Pos: {" << pos_x << "; " << pos_y << "}");

		if (i_last_x >= 0 && i_last_y >= 0 && pos_x >= 0 && pos_y >= 0)
		{
			//Draw a red line from the previous point to the current point
			line(img_lines, Point(pos_x, pos_y), Point(i_last_x, i_last_y), Scalar(0, 0, 255), 2);
			circle(img_lines, Point(pos_x, pos_y), 20, Scalar(0, 0, 255), CV_FILLED);
		}

		i_last_x = pos_x;
		i_last_y = pos_y;

		previous_positions.push_back(Point(i_last_x, i_last_y));

		int current_x = i_last_x;
		int current_y = i_last_y;
		for(int i = 0; i < previous_positions.size(); i++)
		{
			//Draw a red line from the previous point to the current point
			line(img_lines, Point(current_x, current_y), previous_positions[i], Scalar(0, 0, 255), 10);
		}

		if(previous_positions.size() > 10)
		{
			previous_positions.erase(previous_positions.begin(), previous_positions.begin() + 1);
		}

		human_position = (uint8_t)floor(100.0 * ((float)pos_x / (float)img_lines.size().width));
		ROS_INFO_STREAM("Human position: " << (int)human_position);
	}
	else
	{
		previous_positions.clear();
	}

	cv_image = cv_image + img_lines;
	Mat threshold_as_bgr;
	cvtColor(img_thresholded, threshold_as_bgr, CV_GRAY2BGR);
	Mat threshold_with_lines;
	addWeighted(threshold_as_bgr, 0.75, img_lines, 1, 0.0, threshold_with_lines);
	cv_bridge::CvImage ros_img_thresholded;
	ros_img_thresholded.header = cv_ptr->header;
	ros_img_thresholded.encoding = sensor_msgs::image_encodings::BGR8;
	ros_img_thresholded.image = threshold_with_lines;

    cv_bridge::CvImage ros_img_processed;
	ros_img_processed.header = cv_ptr->header;
	ros_img_processed.encoding = sensor_msgs::image_encodings::BGR8;
	ros_img_processed.image = cv_image;

	image_threshold_publisher_ptr->publish(ros_img_thresholded.toImageMsg());
	image_processed_publisher_ptr->publish(ros_img_processed.toImageMsg());

	std_msgs::Bool found_human_msg;
	found_human_msg.data = found_human;
	std_msgs::UInt8 human_position_msg;
	human_position_msg.data = human_position;
	found_human_publisher_ptr->publish(found_human_msg);
	human_position_publisher_ptr->publish(human_position_msg);
}