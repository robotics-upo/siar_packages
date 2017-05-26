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
  SiarFootprint(double cellsize = 0.02, double length = 0.8, double width = 0.56, double wheel_width = 0.06, bool simplified = false);
  
  SiarFootprint(ros::NodeHandle &pn);
  
  ~SiarFootprint();
  
  FootprintType getFootprint(double x, double y, double yaw);
  FootprintType getFootprintCollision(double x, double y, double yaw);
  
  void addPoints(double x, double y, double yaw, visualization_msgs::Marker &m, int id = 0, bool init = false);
//   void printFootprintCollision(double x, double y, double yaw, ros::Publisher pub, int id = 0);
  
//   inline cv::Mat *getOriginalFootprint() {return footprint;}
  
  double m_length, m_width, m_wheel_width;

protected:  
  double m_cellsize;
  
  void init(bool simplified = true);
  
  // Base footprint image
//   cv::Mat *footprint;

  FootprintType footprint_p;
  FootprintType footprint_p_2;
  FootprintType footprint_rot, footprint_rot_2;
  
};

SiarFootprint::SiarFootprint(ros::NodeHandle& pn)
{
  if (!pn.getParam("cellsize", m_cellsize)) {
    m_cellsize = 0.02;
  }
  if (!pn.getParam("length", m_length)) {
    m_length = 0.8;
  }
  if (!pn.getParam("width", m_width)) {
    m_width = 0.56;
  }
  bool simplified = true;
  if (!pn.getParam("simple_footprint", simplified))
    simplified = true;
  
  init(simplified);
}


SiarFootprint::SiarFootprint(double cellsize, double length , double width, double wheel_width, bool simplified):
m_length(length),m_width(width), m_wheel_width(wheel_width), m_cellsize(cellsize)
{
  init(simplified);
}

void SiarFootprint::init(bool simplified) 
{
  // Construct original_grid. 1 --> Metadata
  
  int n_width = ceil(m_width/m_cellsize);
  int n_cel_wheel = ceil(m_wheel_width/m_cellsize);
  
  // Add points
  double x = -m_length/2.0;
  geometry_msgs::Point p;
  for (int i = 0; x < m_length/2.0; x += m_cellsize, i++) {
    double y = -m_width/2.0;
    p.x = x;
    for (int j = 0; y < m_width/2.0; y += m_cellsize, j++) {
      if (i == 0) {
        p.y = y;
        footprint_p_2.push_back(p);
      }
      if (j < n_cel_wheel || n_width - j <= n_cel_wheel ) {
        p.y = y;
        if (!simplified || i%2 == j%2) {
          footprint_p.push_back(p); // Alternate for make the collision detection simpler
        }
      }
    }
  }
  double y = -m_width/2.0;
  p.x = x - m_cellsize;
  for (int j = 0; y < m_width/2.0; y += m_cellsize, j++) {
    p.y = y;
    footprint_p_2.push_back(p);
  }  
  footprint_rot = footprint_p;
  footprint_rot_2 = footprint_p_2;
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

//! @NOTE The user should NOT free the pointer
FootprintType SiarFootprint::getFootprintCollision(double x, double y, double yaw)
{
  /// Compute a rotation matrix with respect to the center of the image
  for (unsigned int i = 0; i < footprint_p_2.size();i++) {
    footprint_rot_2.at(i).x = x + footprint_p_2.at(i).x * cos(yaw) - footprint_p_2.at(i).y * sin(yaw);
    footprint_rot_2.at(i).y = y + footprint_p_2.at(i).x * sin(yaw) + footprint_p_2.at(i).y * cos(yaw);
  }
  
  return footprint_rot_2;
}

void SiarFootprint::addPoints(double x, double y, double th, visualization_msgs::Marker &m, int id, bool init) {
  std::vector<geometry_msgs::Point> fp = getFootprint(x, y, th);
  if (init) {
    m.header.frame_id = "/base_link";
    m.header.stamp = ros::Time::now();
    m.ns = "footprint";
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.id = id;
    m.points.clear();
    m.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
    m.scale.x = m_cellsize;
    m.scale.y = m_cellsize;
    m.scale.z = 0.1;
    // Points are green
    m.color.g = 1.0;
    m.color.a = 1.0;
  }
  FootprintType footprint = getFootprint(x, y, th);
  geometry_msgs::Point p;
  
  for (unsigned int i = 0;i < footprint.size(); i++) {
    p.x = footprint.at(i).x;
    p.y = footprint.at(i).y;
    p.z = 0.0;
    
    m.points.push_back(p);
  }
}

// void SiarFootprint::addFootprintCollision(double x, double y, double th, ros::Publisher pub, int id)  {
//   std::vector<geometry_msgs::Point> fp = getFootprintCollision(x, y, th);
//   
//   static visualization_msgs::Marker points;
//   
//   
//   points.header.frame_id = "/base_link";
//   points.header.stamp = ros::Time::now();
//   points.ns = "footprint";
//   points.action = visualization_msgs::Marker::ADD;
//   points.pose.orientation.w = 1.0;
//   points.id = id;
//   
//   geometry_msgs::Point p;
//   
//   
//   points.type = visualization_msgs::Marker::POINTS;
//   // POINTS markers use x and y scale for width/height respectively
//   points.scale.x = m_cellsize;
//   points.scale.y = m_cellsize;
//   
//   FootprintType footprint = getFootprintCollision(x, y, th);
//   
//   
//   for (unsigned int i = 0;i < footprint.size(); i++) {
//     p.x = footprint.at(i).x;
//     p.y = footprint.at(i).y;
//     p.z = 0.0;
//     
//     points.points.push_back(p);
//   }
//   // Collision points are red
//   points.color.r = 1.0;
//   points.color.a = 1.0;
//   
//   pub.publish(points);
//   
// }

}


#endif //__SIAR_FOOTPRINT_H