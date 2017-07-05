#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "opencv2/imgproc.hpp"
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <fstream>
#include <iostream>
#include <numeric>

using namespace std;
using namespace cv;

//Std Threshold values used, the values are based on RGB from competions rules
//For green ground robot
int lMin =0; 
int aMin =0;	
int bMin =0;	
int lMax =255;
int aMax =255;
int bMax =255;
ros::Publisher msg;
std_msgs::Int32MultiArray pubLocation;

void calibrationBars(int, void*) {}

int main( int argc, char** argv ){
	ros::init(argc, argv,"orangeTest");
	ros::NodeHandle nh;
	msg=nh.advertise<std_msgs::Int32MultiArray>("std_msgs/Int32MultiArray", 1000);
	VideoCapture cap(0);
	Mat color,hsv,Thresh1,Thresh2;
	if (!cap.isOpened()){
		return -1;
	}
	while(1)
	{
		//Passing video footage
		cap>>color;
		//Define CIE Lab img and smooth it
		cvtColor(color, hsv, COLOR_BGR2HSV);
		medianBlur(hsv,hsv,11);
		//Std Altitude Threshold
		//inRange(hsv, Scalar(lMin, aMin, bMin), Scalar(lMax, aMax, bMax), Thresh1);
		//Low Altitude Threshold
		inRange(hsv, Scalar(0, 156, 153), Scalar(255, 124, 255), Thresh1);
		inRange(hsv, Scalar(0, 194, 71), Scalar(12, 255, 171),Thresh2);
		////Noise reduction
		Mat erodeElement2 = getStructuringElement(MORPH_RECT, Size(21, 21));
		//Mat dilateElement2 = getStructuringElement(MORPH_RECT, Size(9, 9));
		erode(Thresh2, Thresh2, erodeElement2);
		//dilate(Thresh2, Thresh2, dilateElement2);
		////Combine the two Thresholds
		hsv = Thresh1|Thresh2;
		//Blob detection and center point generation 
		vector<vector<Point> > contours;
		findContours(hsv, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<float> areas;
		vector<Point> contoursPoints;
		int lrgContour;
		Point center;
		if(contours.size()!=0)
		{
			for(int i=0;i<contours.size();i++)
				areas.push_back(contourArea(contours[i], false));
			int xSum =0;
			int ySum =0;
			lrgContour = distance(areas.begin(),max_element(areas.begin(),areas.end()));
			contoursPoints = contours[lrgContour];
			for(int i =0;i<contoursPoints.size();i++)
			{
				xSum += contoursPoints[i].x;
				ySum += contoursPoints[i].y; 
			}
			center = Point(xSum/contoursPoints.size(),ySum/contoursPoints.size());
			pubLocation.data.push_back(center.x);
			pubLocation.data.push_back(center.y);
			//cout <<"< " <<center.x <<" , " <<center.y <<" >" <<endl;
			drawContours(color, contours, lrgContour, Scalar(0, 0, 255), 3, 8, vector<Vec4i>(), 0, Point());
			circle(color, center, 5, Scalar(255, 0, 0), FILLED, LINE_8);
		}
		imshow("Thresh", hsv);
		//imshow("hsv", hsv);
		msg.publish(pubLocation);
		imshow("Original",color);
		waitKey(10);
		ros::Rate rate(10);
		ros::spinOnce();
		rate.sleep();
		
	}
}
