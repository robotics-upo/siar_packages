#include "siar_planner_action_server.hpp"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "siar_planner_node");
  
  ROS_INFO("Starting SIAR planner");
//   tf::TransformListener tf_l(ros::Duration(10);
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  SiarPlannerActionServer spas(nh, pnh);
  
  ros::Rate r(ros::Duration(10));
  
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("planned_cmd_vel", 2);
  
  geometry_msgs::Twist cmd_vel;
  while (ros::ok()) {
    ros::spinOnce();
    
    cmd_vel.angular.z = 0.0;
    cmd_vel.linear.x = 0.0;
    
    spas.getCurrentVel(cmd_vel);
    
    cmd_vel_pub.publish(cmd_vel);
  }
  
  return 0;
  
}