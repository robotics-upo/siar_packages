#include "siar_planner/Astar.hpp"
#include "ros/ros.h"

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "test_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  AStarModel model(nh, pnh);
  
  ROS_INFO("Waiting for map initialization");
  while (!model.isInit()) {
    ros::spinOnce();
    sleep(1);
  }
  ROS_INFO("Map initialized");
  
  return 0;
}