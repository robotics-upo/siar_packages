#include <ros/ros.h>
#include "udp_client.hpp"

int main( int argc, char **argv)
{
  // Setup ROS
  ros::init(argc, argv, "udp_client");
  ros::NodeHandle nh;
  
  try
  {
    ROS_INFO("In udp client node");
    // Setup udp client node
    UDPClient client;
    // ROS spin forever
    ros::spin();
  }    
  catch (std::exception& e)
  {
    std::cout << "Exception: " << e.what() << std::endl;
  }
  
  return 0;
}
