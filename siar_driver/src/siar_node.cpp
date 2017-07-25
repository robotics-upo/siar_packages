 /***********************************************************************/
/**                                                                    */
/** raposa_node.h                                                      */
/**                                                                    */
/** Copyright (c) 2015-2016, Service Robotics Lab.                     */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Ignacio Perez-Hurtado                                              */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <boost/bind.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include "siar_driver/siar_manager.hpp"
#include "siar_driver/siar_manager_width_adjustment.hpp"

#define FREQ		50.0
#define FREQ_IMU	150.0
#define VEL_TIMEOUT 	  0.7

SiarManagerWidthAdjustment *siar = NULL;

ros::Time cmd_vel_time;
double vel_timeout;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  cmd_vel_time = ros::Time::now();
  siar->setVelocity(cmd_vel->linear.x,cmd_vel->angular.z);
}

siar_driver::SiarStatus siar_state;

int main(int argc, char** argv)
{
  double dt;

  // Pointers to be freed, here!
  try
  {
    ros::init(argc, argv, "SiarNode");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    std::string siar_port_1, siar_port_2, joy_port, battery_port;

    // Note: Please check the devices of the SIAR and set the params accordingly
    pn.param<std::string>("siar_device_1", siar_port_1, "/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_board2-if00-port0");
    pn.param<std::string>("battery_device", battery_port, "/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTGT8JO-if00-port0");
    pn.param<std::string>("joy_device", joy_port, "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A602XIGF-if00-port0");

    std::string base_frame_id;
    std::string base_ticks_id;
    std::string odom_frame_id;
    bool use_imu;
    bool two_boards;

    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("base_ticks_id", base_ticks_id, "/base_ticks");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    pn.param<bool>("use_imu", use_imu, true);
    pn.param<double>("vel_timeout", vel_timeout, VEL_TIMEOUT);

    double freq, freq_imu;
    pn.param<double>("freq",freq,FREQ);
    ROS_INFO("Freq = %f", freq);
    pn.param<double>("freq_imu",freq_imu,FREQ_IMU);
    
    bool publish_tf;
    pn.param<bool>("publish_tf", publish_tf, false);
    
    dt = 1.0 / freq; // Do not forget the dt :)
    ros::Publisher odom_pub = pn.advertise<nav_msgs::Odometry>(odom_frame_id, 5);
    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1,cmdVelReceived);
    ros::Time current_time,last_time;
    last_time = ros::Time::now();
    cmd_vel_time = ros::Time::now();
    ros::Rate r(100.0);
    tf::TransformBroadcaster tf_broadcaster;
    
    SiarConfig siar_config;
    try {
      siar = new SiarManagerWidthAdjustment(siar_port_1, joy_port, battery_port, siar_config);
      
      ROS_INFO("Connected to SIAR with battery monitor OK");
      
      if (use_imu) {
	siar->setIMU(freq_imu);
      }
    } catch (std::exception &e) {
      ROS_ERROR("Exception thrown while connecting to Siar. Content: %s", e.what());
      return -1;
    }
    
    siar->setVelocity(0.0 ,0.0);
    cmd_vel_time = ros::Time::now() - ros::Duration(vel_timeout);

    // Save the proper fields
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::Imu imu_msg;
    bool first_msg = true;

    while (n.ok()) {
//       // Sleep and actualize
      r.sleep();
      ros::spinOnce();
     
      current_time = ros::Time::now();
      double cmd_vel_sec = (current_time - cmd_vel_time).toSec();

      if (cmd_vel_sec >= vel_timeout) {
	siar->setVelocity(0.0 ,0.0);
      } 

      // Odometry: call a external function
      
      siar->update();
      last_time = current_time;
      
      siar_state = siar->getState();
      
      // ******************************************************************************************
      // first , we'll publish the odometry message over ROS
      nav_msgs::Odometry odom = siar_state.odom;
      odom.header.frame_id = odom_frame_id;
      odom_pub.publish(odom);
      
      if (publish_tf) {
        // then , we'll publish the transforms over tf
        odom_trans.transform.rotation = siar_state.odom.pose.pose.orientation;
        odom_trans.transform.translation.x = siar_state.odom.pose.pose.position.x;
        odom_trans.transform.translation.y = siar_state.odom.pose.pose.position.y;
        odom_trans.transform.translation.z = siar_state.odom.pose.pose.position.z;
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = base_frame_id;
        odom_trans.header.stamp = ros::Time::now();
      
        // Publish the odometry TF
        tf_broadcaster.sendTransform(odom_trans);
      }
    }
  }
  catch(SiarManagerException e) {
    ROS_WARN("%s",e.what());
//     ROS_BREAK();
  }
  
  // Free memory
  delete siar;
  
  return 0;
}
