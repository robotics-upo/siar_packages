#include "siar_arm/siar_arm_ros_mbzirc.hpp"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "siar_arm_node");
  
  ROS_INFO("Starting SIAR Arm node");
//   tf::TransformListener tf_l(ros::Duration(10);
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  SiarArmROSMBZirc arm_ros(nh, pnh);
  arm_ros.start();
  arm_ros.publishRadStateArm();
  arm_ros.manageServerArduino();
  

  ros::Rate r(ros::Duration(10));
  
  geometry_msgs::Twist cmd_vel;
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
    
  }
  
  return 0;
  
}

