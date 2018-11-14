#ifndef __ALERT_HPP__ 
#define __ALERT_HPP__ 

#include "ros/ros.h"
#include "alert_db/GenerateAlert.h"
#include "alert_db/ServiceabilityAlertRequest.h"
#include <string>
#include <sewer_graph/earthlocation.h>
#include "std_msgs/Header.h"
#include "kml/dom.h"
#include <visualization_msgs/Marker.h>
#include "amcl_sewer/Localization.h"

namespace alert_db 
{

enum AlertType {
  INIT, REPEATER, STRUCTURAL_DEFECT, SERVICEABILITY, INLET, MANHOLE, END=254, ALL = 255
};

class Alert {
public:
  Alert(int id, const sewer_graph::EarthLocation &location, const alert_db::GenerateAlertRequest &alert_req,
  const geometry_msgs::Pose &pose);
  
  Alert(int id, const sewer_graph::EarthLocation &location, 
  const geometry_msgs::Pose &pose, const alert_db::ServiceabilityAlertRequest &serv_req);
  
  kmldom::PlacemarkPtr toKML() const;
  
  std::string toHTML() const;
  
  std::string toString() const;
  
  visualization_msgs::Marker getMarker(const std::string &map_frame) const;
  
  
  inline AlertType getType() const { return m_type; }
  
  void setLocalizationInfo(const amcl_sewer::Localization &info);
  
protected:
  int m_id;
  sewer_graph::EarthLocation m_location;
  alert_db::GenerateAlertRequest m_alert;
  geometry_msgs::Pose m_pose;
  AlertType m_type;
  amcl_sewer::Localization m_loc_info;
  
  // Serviceability info
  alert_db::ServiceabilityAlertRequest m_serv;
// //   float bucket_level, curb_level, still_level; // heights of the waste on areas
//   int quality;

  
};

Alert::Alert(int id, const sewer_graph::EarthLocation& location, const GenerateAlertRequest& alert_req,
  const geometry_msgs::Pose &pose):m_id(id),m_location(location),m_alert(alert_req), m_pose(pose)
{
  m_type = (AlertType)alert_req.type;
}

Alert::Alert(int id, const sewer_graph::EarthLocation& location,	
  const geometry_msgs::Pose &pose, const alert_db::ServiceabilityAlertRequest &serv_req):m_id(id),m_location(location), m_pose(pose), m_serv(serv_req)
{
  m_alert.description = m_serv.description;
  m_alert.type = SERVICEABILITY;
  m_alert.head = m_serv.head;
  m_type = SERVICEABILITY;
}

string Alert::toString() const
{
  std::ostringstream os;
  
  const std::string sep=" ";
  
  os << "Alert ID: " << sep << m_id << "\n";
  os << "  Type: " << sep << m_type << "\n";
  os << "  Stamp: " << sep << m_alert.head.stamp << "\n";
  os << "  Global Location: " << sep << m_location.toString() << std::endl;
  os << "  Local Location: " << sep << "(" << m_pose.position.x << ", " << m_pose.position.y << ")\n";
  os << "  Description: " << sep << m_alert.description << std::endl;
  os << "  Distance to manhole: " << sep << m_loc_info.dist_closest_manhole << std::endl;
  
  if (m_type == SERVICEABILITY) {
    os << "  Bucket level: "  << sep << m_serv.bucket_level << "\n";
    os << "  Curb level: " << sep << m_serv.curb_level << "\n";
    os << "  Still level: " << sep << m_serv.still_level << "\n";
    os << "  Quality(%): " << sep << m_serv.quality << "\n";
  }
  
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
  m.id = m_id;
  
  m.color.b = m.color.r = m.color.g = 0.0;
  m.color.a = 1.0;
  
  m.scale.x = 2.0;
  m.scale.y = 2.0;
  m.scale.z = 2.0;
  
  switch(m_type) {
    case INIT:
      m.color.g = 1.0;
      m.color.r = 0.0;
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
      m.type = visualization_msgs::Marker::SPHERE;
      break;
      
    case REPEATER:
      m.color.g = 0.4;
      m.color.r = 0.4;
      m.color.b = 0.4;
      m.text = "repeater";
      m.type = visualization_msgs::Marker::CUBE;
      break;
      
    case INLET:
      m.color.g = m.color.r = 0.2;
      m.color.b = 1.0;
      m.text = "inlet";
      m.type = visualization_msgs::Marker::CYLINDER;
      
    default:
      m.color.r = 1.0;
      m.text = "End of inspection";
      m.type = visualization_msgs::Marker::CUBE;
  }
  
  return m;
}

void Alert::setLocalizationInfo(const amcl_sewer::Localization& info)
{
  m_alert.dist_manhole = info.dist_closest_manhole;
  m_loc_info = info;
}



}
#endif