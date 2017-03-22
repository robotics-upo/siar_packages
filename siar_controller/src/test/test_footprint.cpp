#include "../siar_footprint.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace siar_controller;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_footprint");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("footprint_marker", 10, true);
  
  double cellsize = 0.02;
  // POINTS markers use x and y scale for width/height respectively
  
  SiarFootprint fp(cellsize);
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  
  if (argc > 1) {
    x = atof(argv[1]);
  } else {
    x = 0.0;
  }
  if (argc > 2) {
    y = atof(argv[2]);
  }
  if (argc > 3) {
    theta = atof(argv[3]);
  }
  
  visualization_msgs::Marker m;
  fp.addPoints(x, y, theta, m, 0, true);
  fp.addPoints(x+2, y, theta,m, 0, false);
  
  marker_pub.publish(m);
  sleep(1);
  ros::spinOnce();
  sleep(2);
//   fp.printFootprintCollision(x, y, theta, marker_pub, 1);
  
  ros::spin();
 
  return 0;
}
