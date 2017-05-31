#include <ros/ros.h>
#include "udp_server.hpp"

int main( int argc, char **argv)
{
  // Setup ROS
  ros::init(argc, argv, "udp_server");
  ros::NodeHandle nh;
  try
  {
    UDPServer *server;
    while (ros::ok()) 
    {
      server = new UDPServer;
      ros::Rate r(100.0);
      while (server->isRunning()) 
      {
        ros::spinOnce();
        r.sleep();
      }
      delete server;
      sleep(1);
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Exception catched in UDP server. Content: %s", e.what());
  }
  
    
  return 0;
}


