#ifndef _ALERT_DB_HPP__
#define _ALERT_DB_HPP__

#include "alert_db/alert.hpp"
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "alert_db/GenerateAlert.h"
#include "alert_db/ServiceabilityAlert.h"
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "amcl_sewer/Localization.h"

#include "kml/base/file.h"
#include "kml/base/math_util.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/convenience/convenience.h"
#include "kml/base/string_util.h"

#include "functions/functions.h"

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
  
  std::string toString() const;
  
  bool generateReports();
  
protected:
  std::vector<Alert> alerts;
  ros::Publisher marker_pub, text_pub;
  ros::Subscriber gps_fix_sub, image_sub, loc_info_sub;
  ros::ServiceServer generate_alert_server, serviceability_alert_server;
  sensor_msgs::NavSatFix fix;
  sewer_graph::EarthLocation center;
  bool fix_init, init;
  std::string kml_out_file, text_out_file;
  
  std::string map_tf, base_tf;
  
  kmldom::KmlFactory* factory;
  
  tf::TransformListener tfl;
  
  amcl_sewer::Localization last_loc_info;
  bool has_loc_info;
  
  bool getPose(geometry_msgs::Pose& pose, sewer_graph::EarthLocation& loc);
  
  bool generateAlert(GenerateAlert::Request &req, GenerateAlert::Response &res);
  bool serviceabilityAlert(ServiceabilityAlert::Request &req, ServiceabilityAlert::Response &res);
  void gpsFixCb(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void publishMarker();
  void locInfoCb(const amcl_sewer::Localization::ConstPtr &msg);
};

AlertDB::AlertDB(ros::NodeHandle &nh, ros::NodeHandle &lnh):fix_init(false), init(false), factory(NULL), has_loc_info(false)
{
  // Params 
  map_tf = lnh.resolveName("/map");
  base_tf = lnh.resolveName("/base_link");
  if (!lnh.getParam("kml_out_file", kml_out_file)) {
    std::ostringstream os;
    os << "/home/chur/DemoJul2018/alert_gis" << ros::Time::now() << ".kml";
    kml_out_file = os.str();
  }
  if (!lnh.getParam("text_out_file", text_out_file)) {
    std::ostringstream os;
    os << "/home/chur/DemoJul2018/alerts" << ros::Time::now() << ".txt";
    text_out_file = os.str();
  }
  
  // Publishers, Subscribers and Services
  text_pub = nh.advertise<std_msgs::String>("/alert_text", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("/alerts", 10, true);
  generate_alert_server = nh.advertiseService("/generate_alert", &AlertDB::generateAlert, this);
  serviceability_alert_server = nh.advertiseService("/serviceability_alert", &AlertDB::serviceabilityAlert, this);
  gps_fix_sub = nh.subscribe("/gps/fix", 2, &AlertDB::gpsFixCb, this);
  loc_info_sub = nh.subscribe("/amcl_sewer_node/localization_info", 2, &AlertDB::locInfoCb, this);
//   image_sub = nh.subscribe("/front_web/rgb/image_raw/compressed", 2, &AlertDB::gpsFixCb, this);
  
  factory = kmldom::KmlFactory::GetFactory();
}

AlertDB::~AlertDB()
{
  delete factory;
}


bool AlertDB::generateAlert(GenerateAlert::Request& req, GenerateAlert::Response& res)
{
//   ROS_INFO("In generateAlert");
  bool ret_val = false;
   
  geometry_msgs::Pose pose;
  sewer_graph::EarthLocation loc(center);
  if (getPose(pose,loc)) {
    ROS_INFO("Got Pose: %f, %f, %f \t Location: %s", pose.position.x, pose.position.y, pose.orientation.z, loc.toString().c_str());
    
    Alert curr_alert(alerts.size(), loc, req, pose);
    
    if (has_loc_info)
      curr_alert.setLocalizationInfo(last_loc_info);
    
    alerts.push_back(curr_alert);
    ret_val = true;
    publishMarker();
      
    std_msgs::String text_msg;
    text_msg.data = toString();
    text_pub.publish(text_msg);
      
    ROS_INFO("New alert generated. Type = %d. Description = %s", req.type, req.description.c_str());
    res.result = 1;
  } else {
    ROS_INFO("Could not generate alert.");
    res.result = 0;
    ret_val = false;
  }
  
  if (ret_val && alerts.size() > 0) {
//     if (alerts[alerts.size() - 1].getType() == END) {
    if (req.type > 100) { // end!
      generateReports();
    }
  }
  
  return ret_val;
}

bool AlertDB::serviceabilityAlert(ServiceabilityAlert::Request &req, ServiceabilityAlert::Response &res)
{
  bool ret_val = true;
  
  geometry_msgs::Pose pose;
  sewer_graph::EarthLocation loc(center);
  if (getPose(pose,loc)) {
    Alert curr_alert(alerts.size(), loc,  pose, req);
    
    
    alerts.push_back(curr_alert);
    ret_val = true;
    publishMarker();
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

void AlertDB::locInfoCb(const amcl_sewer::Localization_< std::allocator< void > >::ConstPtr& msg)
{
  last_loc_info = *msg;
  has_loc_info = true;
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

string AlertDB::toString() const
{
  std::ostringstream os;
  
  for (unsigned int i = 0; i < alerts.size(); i++) {
    os << alerts[i].toString() << std::endl;  // kml takes ownership.
  }
  
  return os.str();
}

bool AlertDB::generateReports()
{
  exportKMLFile(kml_out_file);
  ROS_INFO("Alerts content:\n%s", toString().c_str());
  functions::writeStringToFile(text_out_file, toString());
}


bool AlertDB::getPose(geometry_msgs::Pose& pose, sewer_graph::EarthLocation &loc)
{
  bool ret_val = false;
  
  try {
    tf::StampedTransform transf;
    tfl.lookupTransform(map_tf, base_tf, ros::Time(0), transf);
    double x = transf.getOrigin().getX();
    double y = transf.getOrigin().getY();
    loc.shift(y, x);
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    tf::Quaternion q = transf.getRotation();
    pose.orientation.w = q.getW();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
    ret_val = true;
  } catch (std::exception &e) {
    ROS_ERROR("AlertDB::getPose --> Could not get the transform from %s to %s.", map_tf.c_str(), base_tf.c_str());
  }
  
  return ret_val;
}


}

#endif
