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
const ros::Publisher* qr_code_publisher_ptr;

void ImageRawCallback(const sensor_msgs::ImageConstPtr& dataPtr);

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "image_recognition_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(2);
	ros::Subscriber bumper_sub = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);
	ros::Publisher qr_code_publisher = nh.advertise<std_msgs::String>("qr_reader/qr_code/data", 100);
	qr_code_publisher_ptr = &qr_code_publisher;

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void ImageRawCallback(const sensor_msgs::ImageConstPtr& dataPtr)
{
	current_frame = dataPtr;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(current_frame, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat cv_image = cv_ptr->image;
	ImageScanner scanner;  
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);  
	Mat imgout;  
	cvtColor(cv_image, imgout, CV_BGR2GRAY);  
	int width = imgout.cols;  
	int height = imgout.rows;  
	uchar *raw = (uchar *)imgout.data;

	// wrap image data  
	Image image(width, height, "Y800", raw, width * height);  
	// scan the image for barcodes  
	int n = scanner.scan(image);  

	string code_data = "No data";
	// extract results  
	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
	{   
		//ROS_INFO_STREAM("Decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << "\" "); 
		code_data = symbol->get_data();
		break;
	}

	std_msgs::String string_msg;
	string_msg.data = code_data;
	qr_code_publisher_ptr->publish(string_msg);

	image.set_data(NULL, 0);
}