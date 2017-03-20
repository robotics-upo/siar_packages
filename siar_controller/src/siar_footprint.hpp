#ifndef __SIAR_FOOTPRINT_H
#define __SIAR_FOOTPRINT_H

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace siar_controller {

typedef std::vector<geometry_msgs::Point> FootprintType;
  
class SiarFootprint {
public:
  SiarFootprint(double cellsize = 0.02, double length = 0.8, double width = 0.56, double wheel_width = 0.06);
  
  ~SiarFootprint();
  
  FootprintType getFootprint(double x, double y, double yaw);
  
  void printFootprint(double x, double y, double yaw, ros::Publisher pub, int id = 0);
  
//   inline cv::Mat *getOriginalFootprint() {return footprint;}
  
protected:
  double m_length, m_width, m_wheel_width;
  
  double m_cellsize;
  
  // Base footprint image
//   cv::Mat *footprint;
  
  FootprintType footprint_p;
  FootprintType footprint_rot;
};

SiarFootprint::SiarFootprint(double cellsize, double length , double width, double wheel_width):
m_length(length),m_width(width), m_wheel_width(wheel_width), m_cellsize(cellsize)
{
  // Construct original_grid. 1 --> Metadata
  
  int n_width = ceil(width/cellsize);
  int n_cel_wheel = ceil(m_wheel_width/cellsize);
  
  // Add points
  double x = -length/2.0;
  geometry_msgs::Point p;
  for (int i = 0; x < length/2.0; x += cellsize, i++) {
    double y = -width/2.0;
    for (int j = 0; y < width/2.0; y += cellsize, j++) {
      if (j < n_cel_wheel || n_width - j <= n_cel_wheel ) {
        p.x = x;
        p.y = y;
        footprint_p.push_back(p);
      }
    }
  }
  footprint_rot = footprint_p;
}

SiarFootprint::~SiarFootprint()
{
}


//! @NOTE The user should NOT free the pointer
FootprintType SiarFootprint::getFootprint(double x, double y, double yaw)
{
  /// Compute a rotation matrix with respect to the center of the image
  for (unsigned int i = 0; i < footprint_p.size();i++) {
    footprint_rot.at(i).x = x + footprint_p.at(i).x * cos(yaw) - footprint_p.at(i).y * sin(yaw);
    footprint_rot.at(i).y = y + footprint_p.at(i).x * sin(yaw) + footprint_p.at(i).y * cos(yaw);
  }
  
  return footprint_rot;
}

void SiarFootprint::printFootprint(double x, double y, double th, ros::Publisher pub, int id)  {
  std::vector<geometry_msgs::Point> fp = getFootprint(x, y, th);
  
  static visualization_msgs::Marker points;
  
  
  points.header.frame_id = "/base_link";
  points.header.stamp = ros::Time::now();
  points.ns = "footprint";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  
  geometry_msgs::Point p;
  
  
  points.type = visualization_msgs::Marker::POINTS;
  double cellsize = 0.02;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = cellsize;
  points.scale.y = cellsize;
  
  FootprintType footprint = getFootprint(x, y, th);
  
  
  for (unsigned int i = 0;i < footprint.size(); i++) {
    p.x = footprint.at(i).x;
    p.y = footprint.at(i).y;
    p.z = 0.0;
    
    points.points.push_back(p);
  }
  // Points are green
  points.color.g = 1.0;
  points.color.a = 1.0;
  
  pub.publish(points);
  
}

}


#endif //__SIAR_FOOTPRINT_H