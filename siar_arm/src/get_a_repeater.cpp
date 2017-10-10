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

void  DetectMarkerCallback(const aruco_msgs::MarkerArray::ConstPtr& markers_read)
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
  
  
  
void GetRepeaterCallback(const std_msgs::Bool::ConstPtr& a )
{
   geometry_msgs::Point goal_point;
   geometry_msgs::PoseStamped actual_pose;


  if( arm1.ball_aprox < 1 && arm1.marker_detected)
  {
    arm1.moveArm2Point(arm1.marker_point);
    arm1.ball_aprox++;
  }
  else if(arm1.ball_aprox < 20 && arm1.ball_detected)
  {
    geometry_msgs::Point mov_dir;
    
    mov_dir.z = (arm1.mc_ball.x - arm1.width/2)*MOV; 
    mov_dir.x = -(arm1.mc_ball.y - arm1.height/2)*MOV;
    mov_dir.y = MOV;
    
    actual_pose = arm1.forwardKinematics(arm1.read_values);
    goal_point = actual_pose.pose.position;
    goal_point.x += mov_dir.x;
    goal_point.z += mov_dir.z;
    
    arm1.movelArm2Point(goal_point,0);	
    goal_point.y += mov_dir.y;
    arm1.movelArm2Point(goal_point,0);
    arm1.ball_aprox++;
  }
  else if(arm1.ball_aprox == 20){
    
    actual_pose = arm1.forwardKinematics(arm1.read_values);    
    goal_point = actual_pose.pose.position;
    goal_point.z += 0.2;    
    arm1.movelArm2Point(goal_point,0);
    arm1.ball_aprox = 0;
    moveArmHL(0);

  }
  else 
  {
    arm1.wait++;
    if(arm1.wait>10)
    {
      arm1.wait = 0;
      arm1.ball_aprox = 0;
      moveArmHL(0);
    }
  }
   

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_a_repeater");
  ros::NodeHandle nh;
  
  siar_driver::SiarArmCommand servo_command;

  //pub_image_ = nh.advertise< sensor_msgs::Image >("/processed_image/image_raw",  1);
  
  ros::Subscriber images_right_sub = nh.subscribe("/aruco_marker_publisher_back_right/markers", 2, DetectMarkerCallback);

  ros::Subscriber images_left_sub = nh.subscribe("/aruco_marker_publisher_back_left/markers", 2, DetectMarkerCallback);

  
  ros::Subscriber arm_camera_sub = nh.subscribe< sensor_msgs::Image >("/mv_25001872/image_raw", 1, DetectBallCallback);  

  ros::Subscriber order_sub = nh.subscribe< std_msgs::Bool >("/get_repeater", 1, GetRepeaterCallback);  

  ros::Subscriber arm_pos_sub = nh.subscribe("/siar_status", 2, ReadServosCallback);

  
  arm1.arm_pos_pub = nh.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 2);

  
  ros::Rate loop_rate(20);
  while (nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}