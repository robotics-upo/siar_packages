#include "ros/ros.h"

#include "siar_arm/Point2Fire.hpp"

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "siar_fire_arm_node");

	ROS_INFO("Starting SIAR FIRE Arm node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	auto siar_arm = SIAR::Point2Fire(nh, pnh);
	
	ros::Rate r(ros::Duration(0.1));
	while (ros::ok()) {
		ros::spinOnce();
		siar_arm.ControlLoop();
		r.sleep();
	}	

	return 0;
  
}

