#include "siar_controller.hpp"

using namespace siar_controller;

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "siar_controller");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  SiarController controller(nh, pnh);
  
  return 0;
}