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

Mat imgOriginal;//create matrix called imgOriginal
int thresh = 100;
int max_thresh = 255;
//RNG rng(12345);


int smallestX = 1000;
int largestX = 0;
int smallestY = 1000;
int largestY = 0;

//added callback function from vtest
//
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {


  
  imgOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;
   
    //imshow("Original", imgOriginal); //show the original image

  Mat imgGRAY;
  /// Convert image to gray and blur it
  cvtColor( imgOriginal, imgGRAY, COLOR_BGR2GRAY );
  blur( imgGRAY, imgGRAY, Size(3,3) );

            Mat canny_output;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

            /// Detect edges using canny
            Canny( imgGRAY, canny_output, 75, 250, 3 );
            /// Find contours
            findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

            /// Draw contours
            Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
            for( int i = 0; i< contours.size(); i++ )
               {
                 //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                 if (contours.at(i).size() > 400 && contours.at(i).at(0).x > 100 && contours.at(i).at(0).x < 800) {
                  drawContours( drawing, contours, i, 255, 1, 8, hierarchy, 0, Point() );
                  /*int smallestX = 1000;
                  int largestX = 0;
                  int smallestY = 1000;
                  int largestY = 0;*/
                  for (int j = 0; j < contours.at(i).size(); j++) {
                    if (contours.at(i).at(j).x < smallestX) {
                      smallestX = contours.at(i).at(j).x;
                    }
                    if (contours.at(i).at(j).y < smallestY) {
                      smallestY = contours.at(i).at(j).y;
                    }
                    if (contours.at(i).at(j).y > largestY) {
                      largestY = contours.at(i).at(j).y;
                    }
                    if (contours.at(i).at(j).x > largestX) {
                      largestX = contours.at(i).at(j).x;
                    }
                  }
                  break;
                }
               }

            /// Show in a window
            namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
            imgOriginal = Mat(imgOriginal, Rect(smallestX, smallestY, largestX - smallestX, largestY - smallestY));
            imshow( "Contours", imgOriginal );

    cv::waitKey(30);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}



 int main( int argc, char** argv )
 {

    ros::init(argc, argv, "contours");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose2D>("chatter",1000);



            /// Create Window
             //char* source_window = "Source";
             //namedWindow( source_window, CV_WINDOW_AUTOSIZE );
             //imshow( source_window, imgOriginal );

             //createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh );

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/usb_cam_away/image_raw", 1, imageCallback);
  

  ros::spin();
  cv::destroyWindow("view");



   return 0;
}



// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"
// #include <iostream>
// #include <stdio.h>
// #include <stdlib.h>

// using namespace cv;
// using namespace std;

// Mat src; Mat src_gray;
// int thresh = 100;
// int max_thresh = 255;
// RNG rng(12345);

// /// Function header
// void thresh_callback(int, void* );

// /** @function main */
// int main( int argc, char** argv )
// {
//   /// Load source image and convert it to gray
//   src = imread( argv[1], 1 );

//   /// Convert image to gray and blur it
//   cvtColor( src, src_gray, CV_BGR2GRAY );
//   blur( src_gray, src_gray, Size(3,3) );

//   /// Create Window
//   char* source_window = "Source";
//   namedWindow( source_window, CV_WINDOW_AUTOSIZE );
//   imshow( source_window, src );

//   createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
//   thresh_callback( 0, 0 );

//   waitKey(0);
//   return(0);
// }

// /** @function thresh_callback */
// void thresh_callback(int, void* )
// {
//   Mat threshold_output;
//   vector<vector<Point> > contours;
//   vector<Vec4i> hierarchy;

//   /// Detect edges using Threshold
//   threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
//   /// Find contours
//   findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//   /// Approximate contours to polygons + get bounding rects and circles
//   vector<vector<Point> > contours_poly( contours.size() );
//   vector<Rect> boundRect( contours.size() );
//   vector<Point2f>center( contours.size() );
//   vector<float>radius( contours.size() );

//   for( int i = 0; i < contours.size(); i++ )
//      { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
//        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
//      }


//   /// Draw polygonal contour + bonding rects + circles
//   Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
//   for( int i = 0; i< contours.size(); i++ )
//      {
//        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
//        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
//        circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
//      }

//   /// Show in a window
//   namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//   imshow( "Contours", drawing );
// }