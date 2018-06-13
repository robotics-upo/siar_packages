#include "CameraSettings.h"

CameraSettings::CameraSettings(const std::string& frame_, double distance_, double yaw_, double pitch_, double f_x_, double f_y_, double f_z_):
frame(frame_), distance(distance_), yaw(yaw_), pitch(pitch_), f_x(f_x_), f_y(f_y_), f_z(f_z_)
{

}


bool CameraSettings::getSettings(std::string root)
{
  if (!ros::isInitialized())
    return false;
  
  ros::NodeHandle nh("~");
  
  std::string s;
  
  
  s = root; s.append("_f_x");
  nh.getParam(s, f_x);
  s = root; s.append("_f_y");
  nh.getParam(s, f_y);
  s = root; s.append("_f_z");
  nh.getParam(s, f_z);
  s = root; s.append("_distance");
  nh.getParam(s, distance);
  s = root; s.append("_yaw");
  nh.getParam(s, yaw);
  s = root; s.append("_pitch");
  std::cout << "Retrieving pitch: " << s << std::endl,
  nh.getParam(s, pitch);
  s = root; s.append("_frame");
  nh.getParam(s, frame);
  return true;
}

void CameraSettings::setSettings(rviz::ViewController* v)
{
  ROS_INFO("Setting view. Parameters: %s" , toString().c_str());
  v->subProp("Focal Point")->subProp("X")->setValue(f_x);
  v->subProp("Focal Point")->subProp("Y")->setValue(f_y);
  v->subProp("Focal Point")->subProp("Z")->setValue(f_z);
  v->subProp("Pitch")->setValue(pitch);
  v->subProp("Yaw")->setValue(yaw);
  v->subProp("Distance")->setValue(distance);
  v->subProp("Target Frame")-> setValue(frame.c_str());
}

std::string CameraSettings::toString() const
{
  std::ostringstream os;
  
  os << "Frame: " << frame << "\n";
  os << "Distance: " << distance << "\n";
  os << "Yaw: " << yaw << "\n";
  os << "Pitch: " << pitch << "\n";
  os << "Focal Point: (" << f_x << ", " << f_y << ", " << f_z << ")\n";
  
  return os.str();
}

