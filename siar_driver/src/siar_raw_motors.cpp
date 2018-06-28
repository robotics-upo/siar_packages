/***********************************************************************/
/**                                                                    */
/** siar_calibration_node.h                                            */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Ignacio Perez-Hurtado                                              */
/** Noe Perez                                                          */
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
#include "siar_driver/siar_config.hpp"
#include <siar_driver/siar_manager_width_adjustment.hpp>

///////////////////////////////////////////////////
// Default values for actions and buttons        //

#define CMD_VEL     100
#define TOTAL_TIME     2
#define ANG_RATE         0


//////////////////////////////////
// Status flags

bool cancel = false;

using namespace std;

int main(int argc, char** argv)
{
  // ------------- Init ROS stuff -----------------------
  ros::init(argc, argv, "SiarCalibration");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ROS_INFO("Starting Siar calibration node. ");
  
  // -------------- End of ROS stuff -------------------
  
  std::string siar_port_1("/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_board2-if00-port0");
  std::string joy_port("/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTGT8JO-if00-port0");
  std::string battery_port;
  
  pn.param<std::string>("battery_device", battery_port, "/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_board1-if00-port0");
 
  SiarConfig siar_config;
  siar_driver::SiarStatus st;
  
  SiarManager *siar = NULL;
  
  try {
    siar = new SiarManagerWidthAdjustment(siar_port_1, joy_port, battery_port, siar_config);
    ROS_INFO("Connected to Siar OK");
  } catch (exception &e) {
    ROS_ERROR("Could not connect to Siar. Exiting.\n");
    return -1;
  }
  
  // Before starting --> stop Siar
  siar->setRawVelocity(0, 0);
  
  
  // Frequency deliberations
  double delta_t = 0.1;
  ros::Rate r(1/delta_t);
  double t = 0.0;
  
  // Get simulation parameters. 
  int n_motor = 0;
  if (argc > 1) {
    n_motor = atoi(argv[1]);
  }
  
  // The third parameter, if present, is the commanded velocity
  int16_t command_vel = CMD_VEL;
  if (argc > 2) {
    command_vel = atoi(argv[2]);
  }
  
  
  double total_time = TOTAL_TIME;
 
  // Wait confirmation
  cancel = false;
  
  ROS_INFO("Siar raw motors. Sending command  %d to motor %d during %f seconds. Enter 'exit' to cancel, anything else to proceed.", command_vel, n_motor, total_time);
  string s_;
  cin >> s_;
  cancel = s_ == "exit";
  
  if (cancel || !n.ok()) {
    ROS_ERROR("The procedure has been canceled.");
    return -2;
  }

  sleep(1);
  
  // Start timer and reset odometry
  t = 0.0;
  siar->resetOdometry();
  
  while (n.ok() && t < total_time) {
    siar->setMotorVelocity(n_motor, command_vel);
    r.sleep();
    t += delta_t;
//    siar->update();
    ros::spinOnce();
  }

  // First stop Siar
  siar->setRawVelocity(0, 0);

  // Free memory
  delete siar;
  
  return 0;
}
