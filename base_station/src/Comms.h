/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#ifndef COMMS_H
#define COMMS_H

#include <QObject>

#include "functions/Point3D.h"
#include "functions/FormattedTime.h"

// ROS includes
#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <siar_driver/SiarStatus.h>
#include <rssi_get/Nvip_status.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <std_msgs/String.h>
#endif
// #include "CameraSettings.hpp"

class Comms:public QObject
{
  Q_OBJECT
public:
  Comms(int argc, char **argv);
  
  virtual ~Comms();
  
  void startComms();
  
  void shutdownComms();
  
signals:
  void siarStatusChanged(const siar_driver::SiarStatus new_status);
  void newRSSI(const rssi_get::Nvip_status new_status);
  void alertDBReceived(const std::string alert_string);
  
public slots:
  void setEmergencyStop();
  void setElecX(int new_x);
  
private:
  double time_step;
  bool emergency, slow; // Are we in emergency mode?
  int automatic; // Are we in automatic mode?
  
  ros::Subscriber status_sub, rssi_sub, text_sub;
  ros::Publisher emergency_pub, elec_x_pub;
  functions::FormattedTime init_log_time;
  
  //! @brief Gets the state data of an UAV
  void siarStatusCallback(const siar_driver::SiarStatus::ConstPtr& s);
  
  //! @brief Gets the comms status
  void nvipCallback(const rssi_get::Nvip_status::ConstPtr& msg);
  
  //! @brief Gets the comms status
  void alertTextCallback(const std_msgs::String::ConstPtr& msg);
  
  // ROS stuff
  ros::AsyncSpinner *spinner;
  
  // Mutex
  boost::mutex data_mutex;
};

#endif // COMMS_H
