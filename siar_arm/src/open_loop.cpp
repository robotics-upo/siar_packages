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
#include "std_msgs/Bool.h" 

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

void DetectMarkerCallback(const aruco_msgs::MarkerArray::ConstPtr& markers_read)
{
  if( 0 < markers_read->markers.size()) 
  {
    arm1.marker_point = markers_read->markers[0].pose.pose.position;
    arm1.marker_detected = true;
  }
  else
  {
    arm1.marker_detected = false;
  }
}


void DetectBallCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    arm1.detectBall( msg );
}
  
  
  
void PickUpCallback(const std_msgs::Bool::ConstPtr& a )
{
   arm1.pick_up = true;
   arm1.ball_aprox = 0;
}

void DeployCallback(const std_msgs::Bool::ConstPtr& a )
{
   arm1.deploy = true;
   arm1.ball_aprox = 0;
   arm1.moveArmHL(5);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_a_repeater");
  ros::NodeHandle nh;
  
  siar_driver::SiarArmCommand servo_command;

  //pub_image_ = nh.advertise< sensor_msgs::Image >("/processed_image/image_raw",  1);
  
  ros::Subscriber images_right_sub = nh.subscribe("/aruco_marker_publisher_back_right/markers", 2, DetectMarkerCallback);

  ros::Subscriber images_left_sub = nh.subscribe("/aruco_marker_publisher_back_left/markers", 2, DetectMarkerCallback);

  
  //ros::Subscriber arm_camera_sub = nh.subscribe< sensor_msgs::Image >("/mv_25001872/image_raw", 1, DetectBallCallback);  

  ros::Subscriber pick_up_sub = nh.subscribe< std_msgs::Bool >("/pick_up_repeater", 1, PickUpCallback);  
  ros::Subscriber deploy_sub = nh.subscribe< std_msgs::Bool >("/deploy_repeater", 1, DeployCallback);  

  ros::Subscriber arm_pos_sub = nh.subscribe("/siar_status", 2, ReadServosCallback);

  
  arm1.arm_pos_pub = nh.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 2);

  
  ros::Rate loop_rate(20);
  
  
  arm1.pick_up = false;
  arm1.deploy = false;
  
  while (nh.ok()) {
    ros::spinOnce();
    
   geometry_msgs::Point goal_point;
   geometry_msgs::PoseStamped actual_pose;
    
    if( arm1.pick_up )
    {
      if( arm1.marker_detected)
      {
	arm1.moveArm2Point(arm1.marker_point);   
	arm1.moveArmHL(1);
	arm1.moveArmHL(2);
	arm1.moveArmHL(3);
	arm1.moveArmHL(4);
	arm1.moveArmHL(5);
	arm1.moveArmHL(3);
	arm1.moveArmHL(2);
	arm1.moveArmHL(1);
	arm1.pick_up = false;
      }
    } 
    
    
    if( arm1.deploy )
    {
      	arm1.moveArmHL(1);
	arm1.moveArmHL(2);
	arm1.moveArmHL(3);
	arm1.moveArmHL(4);
	arm1.moveArmHL(5);
	arm1.moveArmHL(3);
	arm1.moveArmHL(2);
	arm1.moveArmHL(1);
	arm1.deploy = false;  
    }
    loop_rate.sleep();
  }
  
}
