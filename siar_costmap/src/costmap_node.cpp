#include <ros/ros.h>
#include <siar_costmap/siarcostmap.hpp>

int main( int argc, char **argv)
{
	// Setup ROS
	ros::init(argc, argv, "costmap_node");
	ros::NodeHandle nh, lnh("~");
	
	// Create service
	SiarCostmap costmap("costmap_node");
	
	// Spin forever
	ros::spin();
    
	return 0;
}
