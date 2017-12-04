#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;
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
	init(argc, argv, "color_detection_node");
	NodeHandle nh;

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

	while (ok())
	{
		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//Copying the image from the camera so we can edit the image frame
//sensor_msgs::ImageConstPtr& type = constant reference pointer that points to the data from the camera
void ImageRawCallback(const sensor_msgs::ImageConstPtr &data_ptr)
{
	current_frame = data_ptr; //Making current_frame look into the same location as the data_ptr.
	cv_bridge::CvImagePtr cv_ptr; //An image pointer

	//Tries to copy the data from the location current_frame is looking into and then assigns the data to
	//cv_ptr in a format which is compatible with OpenCv.
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
	cvtColor(cv_image, img_hsv, CV_BGR2HSV); //Converts the image to grayscale and puts it into img_hsv
	int low_h = 120; //sets the low hue threshold for pink	
	int high_h = 179; //sets the high hue threshold for pink
	int low_s = 70; //sets the low saturation threshold for pink
	int high_s = 255; //sets the high saturation threshold for pink
	int low_v = 70; //sets the low value threshold for pink
	int high_v = 255; //sets the high value threshold for pink

	//Creates a black image with the size as the camera output
	Mat img_lines = Mat::zeros(img_hsv.size(), CV_8UC3);
	Mat img_thresholded;
 	//Thresholds the image with the upper mentioned thresholds
	inRange(img_hsv, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), img_thresholded); 

	//morphological opening (removes small objects from the foreground)
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); //Expands dark colour
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); //Expands light colour

	//morphological closing (removes small holes from the foreground)
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//Finds the moments of the thresholded image and calculates the area through them
	Moments image_moments = moments(img_thresholded);

	double moment_01 = image_moments.m01;
	double moment_10 = image_moments.m10;
	double area = image_moments.m00;

	bool found_human = false;
	uint8_t human_position = 0; //assigns the human position to be 0 since there is no human yet

	// if the area > some value, it is considered that there is a human in the picture
	if (area > 500000)
	{
		found_human = true;

		//calculates the position of the human by looking at the moments compared to the area
		int pos_x = moment_10 / area;
		int pos_y = moment_01 / area;

		//ROS_INFO_STREAM("Pos: {" << pos_x << "; " << pos_y << "}"); //debugging

		if (i_last_x >= 0 && i_last_y >= 0 && pos_x >= 0 && pos_y >= 0) //If the human has moved draw a line
		{
			//Draw a red line from the previous point to the current point
			line(img_lines, Point(pos_x, pos_y), Point(i_last_x, i_last_y), Scalar(0, 0, 255), 2);
			circle(img_lines, Point(pos_x, pos_y), 20, Scalar(0, 0, 255), CV_FILLED);
		}
		//puts the current point(the last x and y coordinate) into the vector previous_positions
		previous_positions.push_back(Point(i_last_x, i_last_y)); 
		//Assigns the start draw positions to the last position
		int draw_start_x = i_last_x; 
		int draw_start_y = i_last_y;
		for(int i = previous_positions.size() - 1; i > 0; i--)
		{
			//Draw a red line from the previous point to the current point
			line(img_lines, Point(draw_start_x, draw_start_y), previous_positions[i], Scalar(0, 0, 255), 10);
			draw_start_x = previous_positions[i].x;
			draw_start_y = previous_positions[i].y;
			
		}
		//Deletes the first segment of the line when more than x amount of points have been connected
		if(previous_positions.size() > 20) 
		{
			previous_positions.erase(previous_positions.begin(), previous_positions.begin() + 1);
		}

		//after drawing the late the last x and y coordinate is assigned to the current possition
		i_last_x = pos_x; 
		i_last_y = pos_y;
		//Assigns the humans position between 0(left) and 100(right) according to the camera
		human_position = (uint8_t)floor(100.0 * ((float)pos_x / (float)img_hsv.size().width));
		//ROS_INFO_STREAM("Human position: " << (int)human_position);
	}
	else //If there are no object clear the list of previous points
	{
		previous_positions.clear();
	}

	cv_image = cv_image + img_lines; //adds the lines of the previous positions to the image
	Mat threshold_as_bgr; //creates an image file used in next line to create a BGR format of the image with thresholds
	cvtColor(img_thresholded, threshold_as_bgr, CV_GRAY2BGR); 
	Mat threshold_with_lines; //Creates an image to have the thresholded image WITH the lines(done next line)
	addWeighted(threshold_as_bgr, 0.75, img_lines, 1, 0.0, threshold_with_lines);
	cv_bridge::CvImage ros_img_thresholded; //Creates an empty ROS combatible format picture 
	ros_img_thresholded.header = cv_ptr->header; //Assigns the header to the same as the header of the original file
	ros_img_thresholded.encoding = sensor_msgs::image_encodings::BGR8; //Assigns information about the encoding
	ros_img_thresholded.image = threshold_with_lines; //Inserts the image data from the threshold_with_lines

    cv_bridge::CvImage ros_img_processed; //Creates an empty ROS combatible format picture
	ros_img_processed.header = cv_ptr->header; //Assigns the header to the same as the header of the original file
	ros_img_processed.encoding = sensor_msgs::image_encodings::BGR8; //Assigns information about the encoding
	ros_img_processed.image = cv_image; //Inserts the image data from the cv_image(original + lines)
	//Publishers for the thresholded image with lines and the original image with lines
	image_threshold_publisher_ptr->publish(ros_img_thresholded.toImageMsg());
	image_processed_publisher_ptr->publish(ros_img_processed.toImageMsg());

	std_msgs::Bool found_human_msg;//Creates class for information regarding human detection
	found_human_msg.data = found_human; //Sets the data for the message to be equal to found human (true/false)
	std_msgs::UInt8 human_position_msg; //Creates class for the humans position
	human_position_msg.data = human_position; //Sets the data for the mssage to be equal to the humans position(1-100/left-right)
	found_human_publisher_ptr->publish(found_human_msg); 
	human_position_publisher_ptr->publish(human_position_msg);
}