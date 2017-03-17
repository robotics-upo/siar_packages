#include "../siar_footprint.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace siar_controller;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_footprint");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("footprint_marker", 10);
  
  visualization_msgs::Marker points;
  
  points.header.frame_id = "/base_link";
  points.header.stamp = ros::Time::now();
  points.ns = "footprint";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  
  
  points.type = visualization_msgs::Marker::POINTS;
  double cellsize = 0.02;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = cellsize;
  points.scale.y = cellsize;
  
  SiarFootprint fp(cellsize);
  
  geometry_msgs::Point p;
  

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
  
  std::vector<geometry_msgs::Point> footprint = fp.getFootprint(x, y, theta);
  for (unsigned int i = 0;i < footprint.size(); i++) {
    p.x = footprint.at(i).x;
    p.y = footprint.at(i).y;
    p.z = 0.0;
    
    points.points.push_back(p);
  }
  // Points are green
  points.color.g = 1.0;
  points.color.a = 1.0;
  
  while (ros::ok()) {
    marker_pub.publish(points);
  
    ros::spinOnce();
    sleep(1);
  }
 
  return 0;
}
