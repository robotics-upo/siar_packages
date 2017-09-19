#include <ros/ros.h>
#include "joy_translator.hpp"

int main( int argc, char **argv)
{
  // Setup ROS
  ros::init(argc, argv, "test_joy_translator");
  ros::NodeHandle nh, lnh("~");
  JoyTranslator j(nh, lnh);
  
  try
  {
    
    while (ros::ok()) {
      usleep(1000);
      ros::spinOnce();
    }
  }    
  catch (std::exception& e)
  {
    std::cout << "Exception: " << e.what() << std::endl;
  }
  
  return 0;
}
