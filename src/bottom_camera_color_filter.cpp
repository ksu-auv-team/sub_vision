#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv-3.2.0/opencv2/imgproc/imgproc.hpp>
#include <opencv-3.2.0/opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
/*
int input = 0;
string input = " ";
*/
class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	
	public:
	ImageConverter()
	: it_(nh_)
	{
	//Image Subscriber
	image_sub_ = it_.subscribe("/image_raw", 1,
		&ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/modified_video", 1);
	
	cv::namedWindow(OPENCV_WINDOW);
}
~ImageConverter()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void imageCb (const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	//Do some shit like draw a big circle
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) 
			cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0)); //success it works
	
	/* keep working on this if statement
	if 
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) 
			cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0)); //success it works
	else
		//do nothing
		break;
	*/
	
	//Update GUI
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);
	
	//Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}
};

int main(int argc, char** argv)
{
	/*
	cout <<"Enter integer for desired video output:\n>";
	getline(cin, input);
	cout <<"Mode: " << input << endl << endl;
	
	while (true) 
	{
		cout <<"Please enter valid number: "<<endl;
		getline 
*/
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
