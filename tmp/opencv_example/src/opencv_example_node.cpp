#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;
cv::Mat detectedEdges;
int marker_size=0;
bool GO=0;
ros::Publisher vel_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  //cv::GaussianBlur( image, image, cv::Size( 21, 21 ), 5, 5 );
  
  //cv::Mat gray;
  // Convert the image to grayscale
  //cv::cvtColor( image, gray, CV_BGR2GRAY );

  int lowThreshold=10;
  int ratio = 3;
  int kernel_size = 3;

  //cv::Canny( gray, detectedEdges, lowThreshold, lowThreshold*ratio, kernel_size );
  
  cv::Mat hsv_image;
  cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
  cv::Mat result;
  cv::Mat result2;
  cv::inRange(hsv_image, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), result);


  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( image, image, Size(9, 9), 2, 2 );

  vector<Vec3f> circles;
cv::Mat src_gray;
cvtColor( image, src_gray, CV_BGR2GRAY );
marker_size=0;
  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, 1, 200, 50, 0, 0 );
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
  	marker_size = radius;
      // circle outline
      circle( image, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }

  cv::Mat fin;
fin= cv::Mat::zeros(result.size(), result.type());

for( size_t i = 0; i < circles.size(); i++ )
  {

      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle outline
      if(result.at<uchar>(center)==255)
      {
          circle( fin, center, radius, 255, -1, 8, 0 );
      } 

}
//cout<<marker_size<<endl;
if(marker_size < 35 && marker_size>0)
{
	GO = true;
}
else
{
	GO = false;
}

if(GO)
{
       // vector<float> przod ={-0.5,0.0,0.0};
      //  vector<float> obrot ={0.0,0.0,0.0};
	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());	//przód-tył prawy joy(1;-1) 
  	vel->linear.x=-0.5;
  	vel->angular.z=0.0;	//obrót (1;-1)
  	vel_pub.publish(vel);
}
else
{
	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());	//przód-tył prawy joy(1;-1) 
  	vel->linear.x=-0.0;
  	vel->angular.z=0.0;	//obrót (1;-1)
  	vel_pub.publish(vel);
}

  imshow("Image",image);
  imshow("Image edited", result);
  imshow("Image circle found",fin);
  waitKey(1);

  
}

int main(int argc, char **argv)
{
  cout<<"SIEMANECZKOoooo"<<endl;

  //initialize node
  ros::init(argc, argv, "cv_example");

  // node handler
  ros::NodeHandle n;
  
  // subsribe topic
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1000, imageCallback);
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_joy", 1);

  // publish
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/imageEdges", 1);
  
  sensor_msgs::ImagePtr msg;
  
  ros::Rate loop_rate(5);
  while (n.ok()) {
    // Check if grabbed frame is actually full with some content
    if(!detectedEdges.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detectedEdges).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
