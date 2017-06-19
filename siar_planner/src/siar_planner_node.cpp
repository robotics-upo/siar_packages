#include "siar_planner_action_server.hpp"
#include "ros/ros.h"
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "Siar planner node");
  
  ROS_INFO("Starting SIAR planner");
//   tf::TransformListener tf_l(ros::Duration(10);
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  SiarPlannerActionServer spas(nh, pnh);
  
  ros::spin();
  
  return 0;
  
}