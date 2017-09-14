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
  //! @brief Default constructor
  SiarFootprint(double cellsize = 0.02, double length = 0.8, double width = 0.56, double wheel_width = 0.06, bool simplified = false);
  
  //! @brief Constructor from ROS 
  SiarFootprint(ros::NodeHandle &pn);
  
  ~SiarFootprint();
  
  //! @BRIEF Gets the wheel part of the footprint
  FootprintType getFootprint(double x, double y, double yaw);
  
  //! @brief gets the part for collision detection
  FootprintType getFootprintCollision(double x, double y, double yaw);
  
  //! @brief Adds points to a marker
  void addPoints(double x, double y, double yaw, visualization_msgs::Marker &m, int id = 0, bool init = false, const std::string link = "/base_link");
  
  void setWidth(double new_width) ;
  
  inline double getWidth() const {
    return m_width;
  }
  
  double m_length, m_width, m_wheel_width;
  bool m_simplified;
  
  size_t size, size_2; // Size of matrices

protected:  
  double m_cellsize;
  
  void init();

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
  pn.param("wheel_width", m_wheel_width, 0.075);
  if (!pn.getParam("simple_footprint", m_simplified))
    m_simplified = true;
  
  init();
}


SiarFootprint::SiarFootprint(double cellsize, double length , double width, double wheel_width, bool simplified):
m_length(length),m_width(width), m_wheel_width(wheel_width), m_cellsize(cellsize),m_simplified(simplified)
{
  init();
}

void SiarFootprint::init() 
{
  // Construct original_grid. 1 --> Metadata
  int n_width = ceil(m_width/m_cellsize);
  int n_cel_wheel = ceil(m_wheel_width/m_cellsize);
  
  footprint_p.clear();
  footprint_p_2.clear();
  
  // Add points
  double x = -m_length/2.0;
  geometry_msgs::Point p;
  for (int i = 0; x < m_length/2.0; x += m_cellsize, i++) {
    double y = -m_width/2.0;
    p.x = x;
    for (int j = 0; y < m_width/2.0; y += m_cellsize, j++) {
      if (i <= 2) {
        p.y = y;
        footprint_p_2.push_back(p);
      }
      if (j < n_cel_wheel || n_width - j <= n_cel_wheel ) {
        p.y = y;
        if (!m_simplified || i%2 == j%2) {
          footprint_p.push_back(p); // If simplified, alternate for make the collision detection simpler
        }
      }
    }
  }
  double y = -m_width/2.0;
  
  
  // Then the collision footprint
  for (int j = 0; y < m_width/2.0; y += m_cellsize, j++) {
    p.y = y;
    p.x = x - m_cellsize;
    footprint_p_2.push_back(p);
    p.x = x - 2.0 * m_cellsize;
    footprint_p_2.push_back(p);
    p.x = x - 3.0 * m_cellsize;
    footprint_p_2.push_back(p);
  }  
  footprint_rot = footprint_p;
  footprint_rot_2 = footprint_p_2;
  size = footprint_p.size();
  size_2 = footprint_p_2.size();
}

SiarFootprint::~SiarFootprint()
{
}



//! @NOTE The user should NOT free the pointer
FootprintType SiarFootprint::getFootprint(double x, double y, double yaw)
{
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  for (unsigned int i = 0; i < size;i++) {
    footprint_rot[i].x = x + footprint_p[i].x * cos_yaw - footprint_p[i].y * sin_yaw;
    footprint_rot[i].y = y + footprint_p[i].x * sin_yaw + footprint_p[i].y * cos_yaw;
  }
  
  return footprint_rot;
}

//! @NOTE The user should NOT free the pointer
FootprintType SiarFootprint::getFootprintCollision(double x, double y, double yaw)
{
  /// Compute a rotation matrix with respect to the center of the image
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  for (unsigned int i = 0; i < size_2;i++) {
    footprint_rot_2[i].x = x + footprint_p_2[i].x * cos_yaw - footprint_p_2[i].y * sin_yaw;
    footprint_rot_2[i].y = y + footprint_p_2[i].x * sin_yaw + footprint_p_2[i].y * cos_yaw;
  }
  
  return footprint_rot_2;
}

void SiarFootprint::addPoints(double x, double y, double th, visualization_msgs::Marker &m, int id, bool init, const std::string link) {
  std::vector<geometry_msgs::Point> fp = getFootprint(x, y, th);
  if (init) {
    m.header.frame_id = link;
    m.header.stamp = ros::Time::now();
    m.ns = "footprint";
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.id = id;
//     m.points.clear();
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
  
  for (unsigned int i = 0;i < size; i++) {
    p.x = footprint[i].x;
    p.y = footprint[i].y;
    p.z = 0.0;
    
    m.points.push_back(p);
  }
  
  footprint = getFootprintCollision(x, y, th);
  for (unsigned int i = 0;i < footprint.size(); i++) {
    p.x = footprint.at(i).x;
    p.y = footprint.at(i).y;
    p.z = 0.0;
    
    m.points.push_back(p);
  }
}

void SiarFootprint::setWidth(double new_width)
{
  m_width = new_width;
  init();
}


// void SiarFootprint::addFootprintCollision(double x, double y, double th, ros::Publisher pub, int id)  {
//   std::vector<geometry_msgs::Point> fp = getFootprintCollision(x, y, th);
// //   
//   static visualization_msgs::Marker points;
// //   
// //   
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