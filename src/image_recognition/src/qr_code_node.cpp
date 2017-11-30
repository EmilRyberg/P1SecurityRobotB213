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
const ros::Publisher* qr_code_publisher_ptr; //publisher for the master

void ImageRawCallback(const sensor_msgs::ImageConstPtr& data_ptr); //declaration, used later

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "qr_code_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(2);
	ros::Subscriber camera_subscriber = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);
	ros::Publisher qr_code_publisher = nh.advertise<std_msgs::String>("qr_reader/qr_code/data", 100);
	qr_code_publisher_ptr = &qr_code_publisher;

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
	ImageScanner scanner;  
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);  
	Mat imgout; //Declares an empty picturefile of mat format
	cvtColor(cv_image, imgout, CV_BGR2GRAY); //Converts the image to grayscale and puts it into imgout
	int width = imgout.cols;  
	int height = imgout.rows;  
	uchar *raw = (uchar *)imgout.data; //Takes the raw data from the grayscale picture

	// wrap image data into Y800 format from the raw data and information on height and width
	Image image(width, height, "Y800", raw, width * height); 
	// scan the image for barcodes with the config settings from scanner.set_config
	int n = scanner.scan(image);  

	string code_data = "No data";
	// extracts results and put the data into code_data
	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
	{   
		//ROS_INFO_STREAM("Decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << "\" "); 
		code_data = symbol->get_data();
		break; //Break instantly since it is assumed that the first symbol is the QR code we want
	}

	std_msgs::String string_msg;
	string_msg.data = code_data;
	qr_code_publisher_ptr->publish(string_msg); //Publish the string to master

	image.set_data(NULL, 0);
}