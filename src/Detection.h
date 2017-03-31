#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

class object_tracker
{
	public:

		 ros::Publisher chatter_pub;

		 int iLastX; 
		 int iLastY;

		 int LowH;
		 int HighH;
		 int  LowS;
		 int  HighS;
		 int  LowV;
		 int  HighV;

		 Scalar line_color;
	
	private:

};

#endif