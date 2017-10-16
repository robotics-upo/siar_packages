#ifndef _ALERT_DB_HPP__
#define _ALERT_DB_HPP__

#include "alert_db/alert.hpp"
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "alert_db/GenerateAlert.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include "kml/base/file.h"
#include "kml/base/math_util.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/convenience/convenience.h"
#include "kml/base/string_util.h"

using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::LineStringPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;


namespace alert_db 
{

class AlertDB 
{
public:
  AlertDB(ros::NodeHandle &nh, ros::NodeHandle &lnh);
  
  ~AlertDB();
  
  bool exportKMLFile(const std::string &file);
  
  bool exportHTML(const std::string &folder);
  
protected:
  std::vector<Alert> alerts;
  ros::Publisher marker_pub;
  ros::Subscriber gps_fix_sub;
  ros::ServiceServer generate_alert_server;
  sensor_msgs::NavSatFix fix;
  sewer_graph::EarthLocation center;
  bool fix_init, init;
  std::string kml_out_file;
  
  std::string map_tf, base_tf;
  
  kmldom::KmlFactory* factory;
  
  tf::TransformListener tfl;
  
  bool generateAlert(GenerateAlert::Request &req, GenerateAlert::Response &res);
  void gpsFixCb(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void publishMarker();
};

AlertDB::AlertDB(ros::NodeHandle &nh, ros::NodeHandle &lnh):fix_init(false), init(false), factory(NULL)
{
  // Params 
  map_tf = lnh.resolveName("/map");
  base_tf = lnh.resolveName("/base_link");
  if (!lnh.getParam("kml_out_file", kml_out_file)) {
    kml_out_file = "~/test_alert_kml.kml";
  }
  
  // Publishers, Subscribers and Services
  marker_pub = nh.advertise<visualization_msgs::Marker>("/alerts", 10);
  generate_alert_server = nh.advertiseService("/generate_alert", &AlertDB::generateAlert, this);
  gps_fix_sub = nh.subscribe("/gps_fix", 2, &AlertDB::gpsFixCb, this);
  
  factory = kmldom::KmlFactory::GetFactory();
}

AlertDB::~AlertDB()
{
  delete factory;
}


bool AlertDB::generateAlert(GenerateAlert::Request& req, GenerateAlert::Response& res)
{
  ROS_INFO("In generateAlert");
  bool ret_val = false;
  try {
    tf::StampedTransform transf;
    tfl.lookupTransform(map_tf, base_tf, ros::Time(0), transf);
    sewer_graph::EarthLocation loc(center);
    double x = transf.getOrigin().getX();
    double y = transf.getOrigin().getY();
    loc.shift(x, y);
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    tf::Quaternion q = transf.getRotation();
    pose.orientation.w = q.getW();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
    
    
    
    Alert curr_alert(alerts.size(), loc, req, pose);
    alerts.push_back(curr_alert);
    ret_val = true;
    publishMarker();
    
    ROS_INFO("New alert generated. Description = %s", req.description.c_str());
  } catch (std::exception &e) {
    
    ROS_ERROR("Could not generate alert. Exception content: %s", e.what());
  }
  
  res.result = (ret_val)?1:0;
  
  if (ret_val && alerts.size() > 0) {
    if (alerts[alerts.size() - 1].getType() == END) {
      exportKMLFile(kml_out_file);
    }
  }
  
  return ret_val;
}


void AlertDB::gpsFixCb(const sensor_msgs::NavSatFix_< std::allocator< void > >::ConstPtr& msg)
{
  fix = *msg;
  fix_init = true;
  center.setLatitude(fix.latitude);
  center.setLongitude(fix.longitude);
}

void AlertDB::publishMarker()
{
  for (size_t i = 0; i < alerts.size(); i++) {
    marker_pub.publish(alerts[i].getMarker(map_tf));
  }
}

bool AlertDB::exportKMLFile(const string& file)
{
  bool ret = true;
  
  // Create document. This is necessary to handle styles
  kmldom::DocumentPtr doc = factory->CreateDocument();
  
      
  for (unsigned int i = 0; i < alerts.size(); i++) {
    doc->add_feature(alerts[i].toKML());  // kml takes ownership.
  }
  // Create the kml pointer with the document
  KmlPtr kml = factory->CreateKml();
  kml->set_feature(doc);
  
  // Now the file and serialize it to string
  KmlFilePtr kmlfile = KmlFile::CreateFromImport(kml);
  if (!kmlfile) {
    ROS_ERROR("error: could not create kml file");
    return false;
  }
  std::string kml_data;
  kmlfile->SerializeToString(&kml_data);
  
  // Once we get the string --> write it to a file
  if (!kmlbase::File::WriteStringToFile(kml_data, file.c_str())) {
    ROS_ERROR("Write of %s failed", file.c_str());
    ret = false;
  }
  
  return ret;
}


}

#endif
