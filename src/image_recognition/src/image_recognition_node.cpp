#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
using namespace zbar;

sensor_msgs::ImageConstPtr current_frame;

void ImageRawCallback(const sensor_msgs::ImageConstPtr& dataPtr);

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "image_recognition_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(5);
    ros::Subscriber bumper_sub = nh.subscribe("camera/rgb/image_raw", 1000, ImageRawCallback);

	while(ros::ok())
	{
		ros::spinOnce();
		//waitKey();  

		loop_rate.sleep();
	}

	return 0;
}

void ImageRawCallback(const sensor_msgs::ImageConstPtr& dataPtr)
{
	ROS_INFO("Got callback");
	current_frame = dataPtr;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		ROS_INFO_STREAM("Copying data: " << current_frame->data[0]);
		cv_ptr = cv_bridge::toCvCopy(current_frame, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat cv_image = cv_ptr->image;

	ROS_INFO_STREAM("CV image " << *cv_ptr->image.data);

	ImageScanner scanner;  
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);  
	//Mat imgout;  
	//cvtColor(cv_image, imgout, CV_BGR2GRAY);  
	int width = cv_image.cols;  
	int height = cv_image.rows;  
	uchar *raw = (uchar *)cv_image.data;  
	// wrap image data  
	Image image(width, height, "Y800", raw, width * height);  
	// scan the image for barcodes  
	int n = scanner.scan(image);  
	// extract results  
	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
	{  
		vector<Point> vp;  
		// do something useful with results  
		ROS_INFO_STREAM("Decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << "\" ");

		int n = symbol->get_location_size();  
		for(int i=0; i<n; i++)
		{  
			vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i))); 
		}  
		//RotatedRect r = minAreaRect(vp);  
		//Point2f pts[4];  
		//r.points(pts);  
		//for(int i=0; i<4; i++)
		//{
		//	line(imgout,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);  
		//}  
		//cout<<"Angle: "<<r.angle<<endl;  
	}  

	//imshow("imgout.jpg",imgout);  
	// clean up  
	image.set_data(NULL, 0);  
}