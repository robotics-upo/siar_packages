#include "ros/ros.h"

#include "siar_arm/Point2Fire.hpp"

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "siar_fire_arm_node");

	ROS_INFO("Starting SIAR FIRE Arm node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	ROS_INFO("TEST");

	SIAR::Point2Fire(nh, pnh);

	return 0;
  
}

