
#include "Detection.h"


  
  //these HSV values are what the trackbars in the window are initialized to
  int iLowH = 0;//0;
  int iHighH = 0;//90;

  int iLowS = 30;//60; 
  int iHighS = 255;//120;

  int iLowV = 120;//180;
  int iHighV = 255;//210;
  

  //create global matrices so that they dont lost their values in between function callbacks
  Mat imgLines;
  Mat imgBoundary;

  //global because the "greens" cant lost their values while blue is being tracked and vice versa
  int blueMomentX = 0;
  int blueMomentY = 0;
  int greenMomentX = 0;
  int greenMomentY = 0;
  int firsttime = 0;

  //create an object for each color you want to track
  object_tracker red;
  object_tracker yellow;
  object_tracker green;
  object_tracker blue;


void tracker(Mat imgOriginal, object_tracker& color)
{


   //THIS SECTION DEALS WITH IMAGE FILTERING AND THRESHOLDING===================================================
   Mat imgHSV;
   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

   Mat imgThresholded;
   inRange(imgHSV, Scalar(color.LowH, color.LowS, color.LowV), Scalar(color.HighH, color.HighS, color.HighV), imgThresholded); //Threshold the image with values from trackbars and store in imgThresholded matrix
 

   //morphological opening (removes small objects from the foreground)
   erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );
   dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) ); 
   //morphological closing (removes small holes from the foreground)
   dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) ); 
   erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );

   Mat imgMasked;
   imgOriginal.copyTo(imgMasked, imgThresholded);//mask the black and white thresholded image with original to show color of object being tracked

   imshow("Thresholded Image", imgMasked); //show the thresholded image
  //==============================================================================================================

   
                    //THIS SECTION DEALS WITH TRACKING THE OBJECT=================================================================
                    geometry_msgs::Pose2D pub_msg;

                    Moments oMoments = moments(imgThresholded);
                    double dM01 = oMoments.m01;//add up all pixel Y? positions
                    double dM10 = oMoments.m10;//add up all pixel X? positions
                    double dArea = oMoments.m00;//calculate area of color

                    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
                    if (dArea > 10000)
                    {
                        //calculate the position of the ball
                        int posX = dM10 / dArea;
                        int posY = dM01 / dArea;    
                              
                        if (color.iLastX >= 0 && color.iLastY >= 0 && posX >= 0 && posY >= 0)
                        {
                            //line(imgLines, Point(posX, posY), Point(color.iLastX, color.iLastY), color.line_color, 2);//draw a line tracking the object
                        }

                        //save current to last pos for purpose of drawing tracking line above
                        color.iLastX = posX;
                        color.iLastY = posY;

                        int adjPosX = posX;
                        int adjPosY = posY*(-1) + 440;//flip the y axis so its oriented like we are used to

                        //used to calculate the angle of the robot only use blue and green objects on the robot
                        if(color.line_color == Scalar(255,0,0))//MAKE SURE THESE MATCH APPROPRIATE COLOR!!!!!
                        {
                          blueMomentX = adjPosX;
                          blueMomentY = adjPosY;
                        }
                        else if(color.line_color == Scalar(0,255,0))//MAKE SURE THESE MATCH APPROPRIATE COLOR!!!!!
                        {
                          greenMomentX = adjPosX;
                          greenMomentY = adjPosY;
                        }

                        //calculate the angle of the robot
                        int opposite = blueMomentY - greenMomentY;
                        int adjacent = blueMomentX - greenMomentX;
                        int angle = (atan2(opposite, adjacent))*(180/3.141);
                        if(angle < 0)
                        {
                          angle = angle + 360;//this makes it 0-360 rather than 0-180 and 0 to -180
                        }

                         //make a message to publish with object's x-y coordinates
                         pub_msg.x = adjPosX;
                         pub_msg.y = adjPosY;
                         pub_msg.theta = angle;
                    }

                    color.chatter_pub.publish(pub_msg);//publish the msg that you built
                    Mat imgDuplicate = imgOriginal;
                    //imgDuplicate = imgDuplicate + imgLines;// + imgBoundary;//add the imgLines on top of original(green line that track objects)
                    imshow("Duplicate", imgDuplicate); //show the original image (plus overlapped green tracking line)
                    //===============================================================================================================

}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    //this if statement is just a hack cuz i was too lazy to figure out variableS
    //the contents of the if statement should only be called once at the beginning so imgLines doesnt repeatedly get rewritten
    if(firsttime == 0)
    {
          Mat imgTmp;//create a matrix called imgTmp
          imgTmp = cv_bridge::toCvShare(msg, "bgr8")->image; //Capture an image from the camera in bgr format and store in imgTmp   
          imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );//Create a black image with the size as the camera output
          imgLines = Mat(imgLines, Rect(105, 20, 625, 435));
          imgBoundary = Mat::zeros( imgTmp.size(), CV_8UC3 );//Create a black image with the size as the camera output
          imgBoundary = Mat(imgBoundary, Rect(105, 20, 625, 435));
    }
    firsttime = 1;

    Mat imgOriginal;//create matrix called imgOriginal that gets newly created for every frame
    imgOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;//capture image from camera and store in imgOriginal matrix in bgr format
    
    //crop the image 
    imgOriginal = Mat(imgOriginal, Rect(105, 20, 625, 435));
  

    //assign high and low values for thresholds
    //keep in this section because you want these values to change everytime callback function is called so that the 
    //threshold values stay current with how the sliders are changing
    red.LowH = iLowH + 15;
    red.HighH = iHighH + 20;
    red.LowS = iLowS;
    red.HighS = iHighS;
    red.LowV = iLowV;
    red.HighV = iHighV;

    yellow.LowH = iLowH + 25;//25
    yellow.HighH = iHighH + 40;//40
    yellow.LowS = iLowS;
    yellow.HighS = iHighS;
    yellow.LowV = iLowV;
    yellow.HighV = iHighV;

    green.LowH = iLowH + 80;
    green.HighH = iHighH + 100;
    green.LowS = iLowS;
    green.HighS = iHighS;
    green.LowV = iLowV;
    green.HighV = iHighV;
   
    blue.LowH = iLowH + 104;//100
    blue.HighH = iHighH + 113;//120
    blue.LowS = iLowS;
    blue.HighS = iHighS;
    blue.LowV = iLowV;
    blue.HighV = iHighV;

    //TRACK THESE COLORS FOR EVERY FRAME
    //tracker(imgOriginal, red);
    tracker(imgOriginal, yellow);
    tracker(imgOriginal, blue);
    tracker(imgOriginal, green);
    

 
    cv::waitKey(30);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}



 int main( int argc, char** argv )
 {

    ros::init(argc, argv, "Detection");
    ros::NodeHandle n;


    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  	//Create trackbars in "Control" window
  	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  	createTrackbar("HighH", "Control", &iHighH, 179);

  	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  	createTrackbar("HighS", "Control", &iHighS, 255);

  	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
  	createTrackbar("HighV", "Control", &iHighV, 255);

    //initialize the "last" values until they receive info
    red.iLastY = -1;
    red.iLastX = -1;
    yellow.iLastY = -1;
    yellow.iLastX = -1;
    green.iLastY = -1;
    green.iLastX = -1;
    blue.iLastY = -1;
    blue.iLastX = -1;

    //initialize each color to its same color to be drawn as the tracking line
    red.line_color = Scalar(0,0,255);
    yellow.line_color = Scalar(0,120,120);
    green.line_color = Scalar(0,255,0);
    blue.line_color = Scalar(255,0,0);


    
    red.chatter_pub = n.advertise<geometry_msgs::Pose2D>("red_chatter",1000);//publisher is called chatter_pub
    yellow.chatter_pub = n.advertise<geometry_msgs::Pose2D>("ball_state_yellow",1000);//publisher is called chatter_pub
    blue.chatter_pub = n.advertise<geometry_msgs::Pose2D>("blue_chatter",1000);//publisher is called chatter_pub
    green.chatter_pub = n.advertise<geometry_msgs::Pose2D>("robot_state",1000);//publisher is called chatter_pub




    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/usb_cam_away/image_raw", 1, imageCallback);//call the callback function every time recieve a subscription
    
   
    ros::spin();
    cv::destroyWindow("view");


   return 0;
}
