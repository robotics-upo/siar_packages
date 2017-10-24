#ifndef __ALERT_HPP__ 
#define __ALERT_HPP__ 

#include "ros/ros.h"
#include "alert_db/GenerateAlert.h"
#include <string>
#include <sewer_graph/earthlocation.h>
#include "std_msgs/Header.h"
#include "kml/dom.h"
#include <visualization_msgs/Marker.h>

namespace alert_db 
{

enum AlertType {
  INIT, REPEATER, STRUCTURAL_DEFECT, SERVICEABILITY, END, ALL = 255
};

class Alert {
public:
  Alert(int id, const sewer_graph::EarthLocation &location, const alert_db::GenerateAlertRequest &alert_req,
  const geometry_msgs::Pose &pose);
  
  kmldom::PlacemarkPtr toKML() const;
  
  std::string toHTML() const;
  
  std::string toString() const;
  
  visualization_msgs::Marker getMarker(const std::string &map_frame) const;
  
  
  inline AlertType getType() const { return m_type; }
  
protected:
  int m_id;
  sewer_graph::EarthLocation m_location;
  alert_db::GenerateAlertRequest m_alert;
  geometry_msgs::Pose m_pose;
  AlertType m_type;
};

Alert::Alert(int id, const sewer_graph::EarthLocation& location, const GenerateAlertRequest& alert_req,
  const geometry_msgs::Pose &pose):m_id(id),m_location(location),m_alert(alert_req), m_pose(pose)
{
  m_type = (AlertType)alert_req.type;
}


string Alert::toString() const
{
  std::ostringstream os;
  
  os << "Alert ID: " << m_id << "\t";
  os << "Type: " << m_type << "\n";
  os << "Stamp: " << m_alert.head.stamp << "\n";
  os << "Location: " << m_location.toString() << std::endl;
  os << "Description: " << m_alert.description << std::endl;
  
  return os.str();
}

kmldom::PlacemarkPtr Alert::toKML() const
{
  std::ostringstream name;
  
  switch(m_type) {
    case INIT:
      name << "Init location";
      break;
      
    case SERVICEABILITY:
      name << "Serviceability defect. Number = " << m_id;
      break;
      
    case STRUCTURAL_DEFECT:
      name << "Structural Defect. ID = " << m_id;
      break;
      
    case REPEATER:
      name << "Repeater. ID = " << m_id;
      break;
  }
  
  kmldom::PlacemarkPtr ret = m_location.getKMLPlacemark(name.str());
  ret->set_description(m_alert.description);
//   kmldom::TimePrimitive::TimeStampPtr t;
//   t->
//   ret->set_timeprimitive(); TODO
  
  return ret;
}     

visualization_msgs::Marker Alert::getMarker(const std::string &map_frame) const
{
  visualization_msgs::Marker m;
  
  m.header.frame_id = map_frame;
  m.header.stamp = ros::Time::now();
  m.pose = m_pose;
  m.lifetime = ros::Duration(0);
  m.type = 0;
  
  m.color.b = m.color.r = m.color.g = 0.0;
  m.color.a = 1.0;
  
  switch(m_type) {
    case INIT:
      m.color.g = 1.0;
      m.color.r = 1.0;
      m.text = "init";
      m.type = visualization_msgs::Marker::CUBE;
      break;
      
    case SERVICEABILITY:
      m.color.g = 0.7;
      m.color.r = 0.7;
      m.color.b = 0.7;
      m.text = "serviceability";
      m.type = visualization_msgs::Marker::CYLINDER;
      break;
      
    case STRUCTURAL_DEFECT:
      m.color.b = 1.0;
      m.text = "structural defect";
      m.type = m.type = visualization_msgs::Marker::SPHERE;
      break;
      
    case REPEATER:
      m.color.g = 0.4;
      m.color.r = 0.4;
      m.color.b = 0.4;
      m.text = "init";
      m.type = visualization_msgs::Marker::CUBE;
      break;
  }
  
  return m;
}



}
#endif