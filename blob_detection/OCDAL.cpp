#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "opencv2/imgproc.hpp"
#include <std_msgs/Int32.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <fstream>
#include <iostream>
#include <numeric>

using namespace cv;
using namespace std;

int H_MIN = 0;			
int H_MAX = 255;			
int S_MIN = 0;			
int S_MAX = 255;			
int V_MIN = 0;				
int V_MAX = 255;
int E_FAC = 1;
int D_FAC = 1;
Mat color;
ros::Publisher msg;
std_msgs::Int32 pubLocation;

void calibrationBars(int, void*) {};

Mat image_Calib(Mat &img) {
	Mat hsv, thresh;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	inRange(hsv, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), thresh);
	GaussianBlur(thresh, thresh, Size(9, 9), 2, 2);
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(E_FAC, E_FAC));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(D_FAC, D_FAC));
	erode(thresh, thresh, erodeElement);
	dilate(thresh, thresh, dilateElement);
	imshow("Calibrated", thresh);
	return thresh;
}

int calculateDist(Mat &color,Mat &thresh) {
	vector<Vec4i> hierarchy;
	vector<int> points_x,points_y;
	vector<double>areas;
	int center;
	double greatest_x,lowest_x;
	vector<vector<Point> > contours;
	findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point> >hull(contours.size());
	vector<Point> points;
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], true);
	}
	Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
	Mat drawing_g;
	if (hull.size()>0) {
		double temp = 0;
		double contour_index;
		for (int i = 0;i < hull.size();i++)
		{
			areas.push_back(contourArea(hull[i], false));
		}
		contour_index = distance(areas.begin(),max_element(areas.begin(),areas.end()));
		points = hull[contour_index];
		for(int i =0;i<points.size();i++)
		{
			points_x.push_back(points[i].x);
			points_y.push_back(points[i].y);
		} 
		greatest_x = *max_element(points_x.begin(), points_x.end());
		lowest_x = *min_element(points_x.begin(), points_x.end());
		center = accumulate(points_x.begin(), points_x.end(), 0)/points.size();
		pubLocation.data=(center);
		//drawContours(color, hull, contour_index, Scalar(0, 255, 0), 3, 8, vector<Vec4i>(), 0, Point());
		drawContours(color, contours, contour_index, Scalar(0, 255, 255), 2, 8, vector<Vec4i>(), 1, Point());
	}
	return(greatest_x-lowest_x);
}

int main(int argc, char **argv){
	ros::init(argc, argv,"OCDAL");
	ros::NodeHandle nh;
	msg=nh.advertise<std_msgs::Int32>("std_msgs/Int32", 1000);
	double c,r;
	std::ostringstream disTxt;
	int pixelWidth;
	double dist;
	VideoCapture cap(1); //Webcam 0, USB Cam 1
	if (!cap.isOpened()){
		ROS_ERROR("camera not open");
		return -1;
	}
	Mat calibrated; namedWindow("Calibration", WINDOW_NORMAL);
	createTrackbar("H_MIN", "Calibration", &H_MIN, H_MAX, calibrationBars);
	createTrackbar("H_MAX", "Calibration", &H_MAX, H_MAX, calibrationBars);
	createTrackbar("S_MIN", "Calibration", &S_MIN, S_MAX, calibrationBars);
	createTrackbar("S_MAX", "Calibration", &S_MAX, S_MAX, calibrationBars);
	createTrackbar("V_MIN", "Calibration", &V_MIN, V_MAX, calibrationBars);
	createTrackbar("V_MAX", "Calibration", &V_MAX, V_MAX, calibrationBars);
	createTrackbar("E_FAC", "Calibration", &E_FAC, 25, calibrationBars);
	createTrackbar("D_FAC", "Calibration", &D_FAC, 25, calibrationBars);
	while(ros::ok())
	{
		if (!cap.isOpened()){
			break;
			}
		cap>>color;
		calibrated = image_Calib(color);
		pixelWidth = calculateDist(color, calibrated);
		dist = 567*101.6/(10*pixelWidth);
		msg.publish(pubLocation);
		imshow("Original", color);
		//ROS_INFO_STREAM(dist);
		waitKey(10);
		ros::Rate rate(10);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
