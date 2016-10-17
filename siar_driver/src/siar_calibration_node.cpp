/***********************************************************************/
/**                                                                    */
/** siar_calibration_node.h                                                */
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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include "siar_driver/siar_config.hpp"
#include <siar_driver/siar_manager_one_board.hpp>

///////////////////////////////////////////////////
// Default values for actions and buttons        //

#define CMD_VEL     200
#define TOTAL_TIME     5
#define ANG_RATE         0
#define PANIC_BUTTON          2
#define START_BUTTON          9
#define BACK_BUTTON           8


//////////////////////////////////
// Status flags

bool start  = false;
bool cancel = false;

////////////////////////////////////////////////////
// Variables that store the actual values        ///
// The ros-parameters have the same name         ///

// General inputs
int panicButton;
int startButton;
int backButton;

void joyReceived(const sensor_msgs::Joy::ConstPtr& joy)
{
  // First of all, panic mode: if pressed --> the panic mode is activated (TODO: deactivate panic)
  start = joy->buttons[startButton] == 1;
  cancel = joy->buttons[panicButton] == 1 || joy->buttons[backButton] == 1;
}

SiarConfig siar_config;

using namespace std;

int main(int argc, char** argv)
{
  // ------------- Init ROS stuff -----------------------
  ros::init(argc, argv, "SiarCalibration");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ROS_INFO("Starting Siar calibration node. ");
  
  // parameters
  pn.param<int>("panic_button", panicButton, PANIC_BUTTON);
  pn.param<int>("start_button", startButton, START_BUTTON);
  pn.param<int>("back_button", backButton, BACK_BUTTON);
  
  // Joy subscriber
  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 5, joyReceived);	
  
  // -------------- End of ROS stuff -------------------
  
  std::string siar_port_1("/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTHE3VLD-if00-port0");
  std::string joy_port("/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_board2-if00-port0");
  SiarConfig siar_config;
  SiarState st;
  
  SiarManager *siar = NULL;
  
  try {
    siar = new SiarManagerOneBoard(siar_port_1, joy_port, siar_config);
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
  
  // The third parameter, if present, is the commanded velocity
  int16_t command_vel = CMD_VEL;
  if (argc > 3) {
    command_vel = atoi(argv[3]);
  }
  
  
  double total_time = TOTAL_TIME;
  int16_t command_right = command_vel;
  int16_t command_left = -command_vel;
  bool twist = false;
 
  int n_tests = 1;
  // Make the different variants of the test
  if (argc > 1 && strcmp(argv[1], "left") == 0) {
    ROS_INFO("Performing twist left.");
    command_left *= -1;
    twist = true;
  } else if (argc > 1 && strcmp(argv[1], "back") == 0) {
    ROS_INFO("Performing backward test.");
    command_left *= -1;
    command_right *= -1;
  } else if (argc > 1 && strcmp(argv[1], "right") == 0) {
    ROS_INFO("Performing twist right.");
    command_right *= -1;
  } else if (argc > 1 && strcmp(argv[1], "double") == 0) {
    ROS_INFO("Performing double test.");
    n_tests = 2;
  } else if (argc > 1 && strcmp(argv[1], "double_twist") == 0) {
    ROS_INFO("Performing double twist test.");
    command_left *= -1;
    n_tests = 2;
  } else {
    ROS_INFO("Performing forward test.");
  }
  
  // The second parameter is the total time, if present
  if (argc > 2) {
    total_time = atof(argv[2]);
  }
  
  int cont = 0;
  if (!twist) {
    while (cont < n_tests && !cancel && n.ok()) {
      // Wait confirmation
      start = false;
      cancel = false;
      
      if (cont == 1) { 
	// Second test: reverse velocities
	command_left *= -1;
	command_right *= -1;
      }
      
      ROS_INFO("Siar calibration. Sending L = %d and R = %d during %f seconds. Press RED (Panic) button to to cancel or START to proceed in 2 secs.", command_left, command_right, total_time);
    
      while (!start && !cancel && n.ok()) {
	r.sleep();
	ros::spinOnce();
      }
      if (cancel || !n.ok()) {
	ROS_ERROR("The calibration procedure has been canceled.");
	return -2;
      }
    
      sleep(2);
      
      // Start timer and reset odometry
      t = 0.0;
      siar->resetOdometry();
      
      while (n.ok() && t < total_time) {
	siar->setRawVelocity(command_left, command_right);
	r.sleep();
	t += delta_t;
	siar->update();
	ros::spinOnce();
      }
    
      // First stop Siar
      siar->setRawVelocity(0, 0);
    
      // Then display stats
      st = siar->getState();
    
      cout << "Left ticks: " << st.middle_left_ticks << endl;
      cout << "Right ticks: " << st.middle_right_ticks << endl;
      cout << "Estimated distance travelled: " << st.odom.pose.pose.position.x << endl;
      cout << "Estimated delta yaw: " << tf::getYaw(st.odom.pose.pose.orientation) << endl;
      cout << "Total time: " << t << endl;
      
      cont++;
    }
  } else {
    while (cont < n_tests && !cancel && n.ok()) {
      // Wait confirmation
      start = false;
      cancel = false;
      
      if (cont == 1) { 
	// Second test: reverse velocities
	command_left *= -1;
	command_right *= -1;
      }
      
      ROS_INFO("Siar calibration. Twist test. Sending L = %d and R = %d during %f seconds. Press RED (Panic) button to to cancel or START to proceed in 2 secs.", command_left, command_right, total_time);
    
      while (!start && !cancel && n.ok()) {
	r.sleep();
	ros::spinOnce();
      }
      if (cancel || !n.ok()) {
	ROS_ERROR("The calibration procedure has been canceled.");
	return -2;
      }
    
      sleep(2);
      
      // Start timer and reset odometry
      t = 0.0;
      siar->resetOdometry();
      start = false;
      
      while (n.ok() && !start && t < total_time) {
	siar->setRawVelocity(command_left, command_right);
	r.sleep();
	t += delta_t;
	siar->update();
	ros::spinOnce();
      }
    
      // First stop Siar
      siar->setRawVelocity(0, 0);
    
      // Then display stats
      st = siar->getState();
      cout << "Left ticks: " << st.middle_left_ticks << endl;
      cout << "Right ticks: " << st.middle_right_ticks << endl;
      cout << "Estimated distance travelled: " << st.odom.pose.pose.position.x << endl;
      cout << "Estimated delta yaw: " << tf::getYaw(st.odom.pose.pose.orientation) << endl;
      cout << "Total time: " << t << endl;
      
      cont++;
    }
  }

  // Free memory
  delete siar;
  
  return 0;
}
