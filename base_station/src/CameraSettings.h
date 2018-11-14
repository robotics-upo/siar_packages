#ifndef __CAMERA_SETTINGS_HPP___
#define __CAMERA_SETTINGS_HPP___



#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/view_controller.h>
#endif

struct CameraSettings {
  std::string frame;
  double distance, yaw, pitch;
  double f_x, f_y, f_z;
  
  CameraSettings(const std::string &frame_ = "base_link", double distance_ = 2, 
                 double yaw_ = M_PI, double pitch = 0.5, double f_x = -0.2, double f_y = 0, double f_z =0.25);
  
  bool getSettings(std::string root);
  void setSettings(rviz::ViewController *v);
  
  std::string toString() const;
};

#endif // __CAMERA_SETTINGS_HPP___