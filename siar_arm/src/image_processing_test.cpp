#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

ros::Publisher pub_image_;
using namespace cv;
using namespace std;
 
void callback(const sensor_msgs::Image::ConstPtr& msg)
{
  
  sensor_msgs::Image image;
  sensor_msgs::Image image_out;
  cv_bridge::CvImagePtr cv_ptr; 
  
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  
  image = *msg;
  image_out = *msg;
  
  int height = image.height;
  int width = image.width;
  
  medianBlur(cv_ptr->image, cv_ptr->image, 5);
  
  vector<Vec3f> circles;
  HoughCircles(cv_ptr->image, circles, CV_HOUGH_GRADIENT, 1, 10, 100, 30, 20, 100 );
    
  for( size_t i = 0; i < circles.size(); i++ )
  {
    Vec3i c = circles[i];
    ROS_INFO("circle: %d %d %d", c[0], c[1], c[2]);
    //image_out.data[c[0]*width+c[1]] = 0;
    
    for( int j=0; j<60; j++)
    for( int k=0; k<60; k++)
      image_out.data[(c[0]+j)*width+(c[1]+k)] = 255;
    sleep(1);
  }
  /*
  float max_lum = 230.0;
  for( int i = 0; i < height; i++)
  {
    for( int j = 0; j < width; j++)
    {
      image_out.data[i*width+j] = (image.data[i*width+j] > (max_lum) )? 255:0;
    }
  }  
  */
  
  pub_image_.publish(image_out);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  
  pub_image_ = nh.advertise< sensor_msgs::Image >("/processed_image/image_raw",  1);
  ros::Subscriber sub = nh.subscribe< sensor_msgs::Image >("/mv_25001872/image_raw", 1, callback);  
  
  ros::Rate loop_rate(20);
  while (nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}