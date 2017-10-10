#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include <aruco_msgs/MarkerArray.h>
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "siar_arm/siar_arm.hpp"

#define MOV 0.03

ros::Publisher pub_image_;
using namespace cv;
using namespace std;

SiarArmControl arm1;

bool receiver_seen = true;

void ReadServosCallback(const siar_driver::SiarStatus::ConstPtr& arm_pose)
{
  for(int i=0; i<5; i++)
    arm1.read_values[i] = arm_pose->herculex_position[i]; 
}

void  Move2MarkerCallback(const aruco_msgs::MarkerArray::ConstPtr& markers_read){
		
  if(!arm1.busy)
  {
    geometry_msgs::Point goal_point;
    for( int i=0; i<markers_read->markers.size();i++) 
      goal_point = markers_read->markers[i].pose.pose.position;
      
    arm1.movelArm2Point(goal_point,0);	
    
    receiver_seen = true;
  }
}


void getReceiverCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if(!arm1.busy && receiver_seen) 
  {
    geometry_msgs::Point goal_point;
    geometry_msgs::Point mov_dir;
    
    for(int iters = 0; iters>20; iters++)
    {
      sensor_msgs::Image image;
      //sensor_msgs::Image image_out;
      cv_bridge::CvImagePtr cv_ptr; 
      cv_bridge::CvImagePtr cv_ptr_color; 
      
      int morph_elem = 2;
      int morph_size = 5;
      int morph_operator = 2;

      Mat canny_output;
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      int thresh = 100;
      int max_thresh = 255;
      RNG rng(12345);
      float max_circle_y = 10000;
      int circle_i = 0;
      geometry_msgs::Point goal_point;
      geometry_msgs::Point mov_dir;
      
      
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv_ptr_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      
      image = *msg;
      
      int height = image.height;
      int width = image.width;
      
      
      
      medianBlur(cv_ptr->image, cv_ptr->image, 10);
      threshold(cv_ptr->image, cv_ptr->image, 250, 255,3 );
      
      Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
      morphologyEx( cv_ptr->image, cv_ptr->image, morph_operator, element );
      

      Canny( cv_ptr->image, canny_output, thresh, thresh*2, 3 );
      /// Find contours
      findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

      vector<Moments> mu(contours.size() );
      for( int i = 0; i < contours.size(); i++ )
	{ mu[i] = moments( contours[i], false ); }

      vector<Point2f> mc( contours.size() );
      for( int i = 0; i < contours.size(); i++ )
	{ mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

	
      for( int i = 0; i< contours.size(); i++ )
      {
	if(mc[i].y<max_circle_y)
	{
	  max_circle_y = mc[i].y;
	  circle_i = i;
	}  
      }

      if( contours.size()>0)
      {
	mov_dir.z = (mc[circle_i].x - width/2)*MOV; 
	mov_dir.x = -(mc[circle_i].y - height/2)*MOV;
	mov_dir.y = MOV;
		
	geometry_msgs::PoseStamped actual_pose, next_pose;

	actual_pose = arm1.forwardKinematics(arm1.read_values);
	
	goal_point = actual_pose.pose.position;
	goal_point.x += mov_dir.x;
	goal_point.z += mov_dir.z;
	
	arm1.movelArm2Point(goal_point,0);	
	goal_point.y += mov_dir.y;
	arm1.movelArm2Point(goal_point,0);
      }
      
      
    }
    
    for(int iters = 0; iters>10; iters++)
    {
    	goal_point.z += MOV;
	arm1.movelArm2Point(goal_point,0);
    }
    arm1.busy = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_a_repeater");
  ros::NodeHandle nh;
  
  siar_driver::SiarArmCommand servo_command;

  //pub_image_ = nh.advertise< sensor_msgs::Image >("/processed_image/image_raw",  1);
  
  ros::Subscriber images_right_sub = nh.subscribe("/aruco_marker_publisher_back_right/markers", 2, Move2MarkerCallback);

  ros::Subscriber images_left_sub = nh.subscribe("/aruco_marker_publisher_back_left/markers", 2, Move2MarkerCallback);

  
  ros::Subscriber sub = nh.subscribe< sensor_msgs::Image >("/mv_25001872/image_raw", 1, getReceiverCallback);  
  
  arm1.arm_pos_pub = nh.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 2);

  
  ros::Rate loop_rate(20);
  while (nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}