#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include "BlockTimer.h"
#include "FastSalience.h"
#include "LQRPointTracker.h"
#include "NMPTUtils.h"

#include "geometry_msgs/Point.h"

using namespace std;
using namespace cv; 

	Size imSize(320,240); 
	BlockTimer bt; 
	FastSalience salTracker;
	LQRPointTracker salientSpot(2);
	vector<double> lqrpt(2,.5); 
	Mat im, im2, viz, sal ;
	ros::Publisher pub;
	geometry_msgs::Point pt;
	
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	double saltime, tottime; 
	//	capture >> im2; 
   	cv_bridge::CvImagePtr cv_ptr;
	//sensor_msgs::Image salmap_;
   //CvBridge bridge;
   try
   {
	 cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	 im2=cv_ptr->image;
	 double ratio = imSize.width * 1. / im2.cols; 
			resize(im2, im, Size(0,0), ratio, ratio, INTER_NEAREST);
     //cvShowImage("view", im);
     //
     	viz.create(im.rows, im.cols*2, CV_32FC3); 
		
		bt.blockRestart(0); 
		vector<KeyPoint> pts; 
		salTracker.detect(im, pts); 
		saltime = bt.getCurrTime(0) ; 
		
		salTracker.getSalImage(sal); 
		
		
		double min, max; 
		Point minloc, maxloc; 
		minMaxLoc(sal, &min, &max, &minloc, &maxloc); 
		
		lqrpt[0] = maxloc.x*1.0 / sal.cols;  
		lqrpt[1] = maxloc.y*1.0 / sal.rows; 
		
		salientSpot.setTrackerTarget(lqrpt); 
		
		Mat vizRect = viz(Rect(im.cols,0,im.cols, im.rows));
		cvtColor(sal, vizRect, CV_GRAY2BGR); 
		
		vizRect = viz(Rect(0, 0, im.cols, im.rows)); 
		im.convertTo(vizRect,CV_32F, 1./256.); 
		/*
		for (size_t i = 0; i < pts.size(); i++) {
			circle(vizRect, pts[i].pt, 2, CV_RGB(0,255,0));
		}
		*/
		
		
		salientSpot.updateTrackerPosition(); 
		lqrpt = salientSpot.getCurrentPosition();
		pt.x=lqrpt[0];
		pt.y=lqrpt[1];
		pub.publish(pt);
		/*
		circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 6, CV_RGB(0,0,255));
		circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 5, CV_RGB(0,0,255));
		circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 4, CV_RGB(255,255,0));
		circle(vizRect, Point(lqrpt[0]*sal.cols, lqrpt[1]*sal.rows), 3, CV_RGB(255,255,0));
		*/
		vizRect = viz(Rect(im.cols,0,im.cols, im.rows));
		cvtColor(sal, vizRect, CV_GRAY2BGR); 
		////if (usingCamera) flip(viz, viz, 1); 
		
		tottime = bt.getCurrTime(1); 
		bt.blockRestart(1); 
		/*
		stringstream text; 
		text << "FastSUN: " << (int)(saltime*1000) << " ms ; Total: " << (int)(tottime*1000) << " ms."; 
		
		putText(viz, text.str(), Point(20,20), FONT_HERSHEY_SIMPLEX, .33, Scalar(255,0,255)); 
		
		
		imshow("FastSUN Salience", viz);
		*/ 

     
   }
   catch (...)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

int main(int argc, char **argv)
   {
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     //cvNamedWindow("view");
     cvStartWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
     
     
     pub = nh.advertise<geometry_msgs::Point>("/nmpt_saliency_point", 50);
     
     salientSpot.setTrackerTarget(lqrpt);
     bt.blockRestart(1);

     ros::spin();
     //cvDestroyWindow("view");
   }
