#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv/cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>

using namespace std;
using namespace cv;

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN 	4	

#define HIGH_PWM	2000
#define MID_PWM 	1500
#define LOW_PWM 	1000

using namespace std;
using namespace cv;

bool debug = true;
bool showVid = true;
bool blob_detected = false;

Scalar gateMinRange = Scalar(100,140,150); //hotel room 7-29
Scalar gateMaxRange = Scalar(150,190,200); //hotel room 7-29
Scalar bouyMinGreen = Scalar(209,239,3);
Scalar bouyMaxGreen = Scalar(151,168,0);
Scalar bouyMinRed = Scalar(148,112,38);
Scalar bouyMaxRed = Scalar(212,164,192);
Scalar bouyMinYellow = Scalar(115,184,33);
Scalar bouyMaxYellow = Scalar(165,255,169);

enum MODE 
{
	DISABLE,
	FLOAT,
	FORWARD,
	TO_GATE,
	TO_RED,
	TO_GREEN,
	ROTATE_R,
	ROTATE_L
};

enum TARGET
{
	GATE,
	RED,
	GREEN
};


int yaw_gain = 200;
Point2f direction_error;
Point2f prev_direction_error;
static const char WINDOW[] = "Image Unprocessed";
static const char WINDOW2[] = "Image Processed";
double start_time;
double current_time = ros::Time::now().toSec();
double time_elapsed = current_time - start_time;

TARGET current_target = GATE;
MODE current_mode = FLOAT;
MODE manual_mode = FLOAT;

namespace enc = sensor_msgs::image_encodings;

void morphOps(Mat& src)
{
	Mat _erode= getStructuringElement(MORPH_RECT, Size(3,3));
	Mat _dilate= getStructuringElement(MORPH_RECT, Size(8,8));
	
	erode(src, src, _erode);
	erode(src, src, _erode);
	
	dilate(src, src, _dilate);
	dilate(src, src, _dilate);
}

void blob_track_CB (Mat src)
{
	morphOps(src);//make the image less noisy
	
	int tooSmallBlobs = 0;
	int tooLargeBlobs = 0;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> centers_of_blobs;
	
	//find blobs (shapes) from the color filtered image
	findContours(src.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//go through the blobs if there is more than 0 blobs and less than 200 blobs
	if (hierarchy.size() > 0 && hierarchy.size() < 50)
	{
		//find the center of the blob and save it to centers
		for (int i = 0; i >= 0 ; i = hierarchy[i][0])
		{
			Moments moment = moments((cv::Mat) contours[i]);
			// moment.m00 is the area
			if ( (moment.m00 > 100) && (moment.m00 < 30000))//if area is > 200 pixels^2 and < 30000 pixels^2
			{				
				blob_detected = true;//we have detected a large blob
				
				//find the midpoint:
				Point p(moment.m10/moment.m00,
						moment.m01/moment.m00);
						
				centers_of_blobs.push_back(p); //save midpoint to centers vector
			}
			else
			{
				if (moment.m00 < 400)
				{
					tooSmallBlobs ++;
				}
				if (moment.m00 > 30000)
				{
					tooLargeBlobs ++;
				}
				blob_detected = false;//we did not detect any large blobs
			}
			if (debug)
			{
				if(tooSmallBlobs > 0)
					ROS_INFO("%i too small blobs detected", tooSmallBlobs);
				if(tooLargeBlobs > 0)
					ROS_INFO("%i too large blobs detected", tooLargeBlobs);
			}
		}
		
	}
	else 
	{
		if (debug)
		{
			if (hierarchy.size() > 0)
				ROS_WARN("Too many blobs detected, bad filter");
			if (hierarchy.size() == 0)
				ROS_WARN("No blobs detected, bad filter");
		}
		blob_detected = false; //we did not detect any blobs at all
	}
	
	//ignore blob centers that are high in the image
	for(int i = 0; i < centers_of_blobs.size(); i++)
	{
		if ( centers_of_blobs[i].y < 128 )
			centers_of_blobs.erase(centers_of_blobs.begin() + i);
	}
	
	if (centers_of_blobs.size() > 2)//if two valid midpoints recorded (pretty sure its the gate)
	{
		yaw_gain = 200;
	}
	else
	{
		yaw_gain = 50;
	}
	
	Point average;
	//find the average point of all the centers:
	if (centers_of_blobs.size() > 0)
	{
		for (int i = 0; i < centers_of_blobs.size(); i++)
		{
			average.x += centers_of_blobs[i].x;
			average.y += centers_of_blobs[i].y;
		}
		average.x /= centers_of_blobs.size();
		average.y /= centers_of_blobs.size();
	}
	
	prev_direction_error = direction_error; //save previous 
	direction_error = average;
	
	if (debug && showVid)
	{
		int alpha = 255;
		cv::line(src, direction_error, Point(src.size().width/2, src.size().height/2), Scalar(alpha, alpha, alpha), 3);
		
		circle(src, average, 3, Scalar(alpha, alpha, alpha), 5);
		//~ cout << "count: " << endl;
		
		
		imshow(WINDOW2, src);
	}
	
	direction_error.x -= src.size().width/2;
	direction_error.y -= src.size().height/2;
	
	if ( ((direction_error.x > 320) && (direction_error.y > 230) ) || 
		 ((direction_error.x < -320) && (direction_error.y < -230)) )
	{
		direction_error.x = 0;
		direction_error.y = 0;
	}
	
	Point2f delta_error;
	delta_error.x = 
	
	waitKey(30);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{		
	//cout << "cam callback" << endl;
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//~ imshow(WINDOW, cv_ptr->image);
    
    cv::Mat current_frame = cv_ptr->image;
    
    cvtColor(current_frame, current_frame, CV_BGR2Lab);
	
	//current_target= GATE;//change it to test different colors
	
	if (current_target == GATE)
		inRange(current_frame, gateMinRange, gateMaxRange, current_frame);
	else if (current_target == RED)
		inRange(current_frame, bouyMinRed, bouyMaxRed, current_frame);
	else
		inRange(current_frame, bouyMinGreen, bouyMaxGreen, current_frame);
	if (debug && showVid)
			imshow(WINDOW, current_frame);
blob_track_CB(current_frame);
}
int main (int argc, char **argv)
{
	usleep(10000);
	if (debug && showVid)
		{
			namedWindow(WINDOW);
			namedWindow(WINDOW2);
		}
		
	ros::init(argc, argv, "object_idenifier");
	ros::NodeHandle n;
	start_time = ros::Time::now().toSec(); 
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	return 0;
}
