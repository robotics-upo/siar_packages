/***********************************************************************/
/**                                                                    */
/** siar_manager_battery_monitor.hpp                                                   */
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
/** Carlos Marques                                                     */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#ifndef _SIAR_MANAGER_BATTERY_MONITOR_H_
#define _SIAR_MANAGER_BATTERY_MONITOR_H_

#ifndef PI
#define   PI 3.14159265
#endif

// Activate this to show some debug information
#ifndef _SIAR_MANAGER_DEBUG_
// #define _SIAR_MANAGER_DEBUG_
#endif

#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <exception>
#include "siar_manager.hpp"
#include "siar_driver/SiarBatteryMonitor.h"
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#define CMD_TIME_OUT 5 // In secs

//-- HEADERS ----------------------------------------------------------------------------

//! @class SiarManagerBatteryMonitor                                                          */
//! @brief This manages the Siar controller. Includes velocity and arm commands

class SiarManagerBatteryMonitor:public SiarManager
{
  public:
  //! @brief Recommended constructor
  //! @param device_1 Device name where the first Siar board is linked ex. "/dev/ttyUSB0"
  //! @param device_2 Device name where the joystick is linked ex. "/dev/ttyUSB0"
  //! @param params Configuration parameters of the Siar
  //!  Comments:
  //!     The serial connections will be stablished in the constructor, if some
  //!     error happens, a SiarManagerException will be thrown.
  SiarManagerBatteryMonitor(const std::string& device_1, const std::string &device_2, 
                            const std::string& device_battery, const SiarConfig &config);
  
  //! @brief Destructor. Stops the robot and closes all open connections
  ~SiarManagerBatteryMonitor();
  
  //! @brief Gets the state of siar (cmd_vel, measured arm_pos, odometry position...)
  //! Comment: This function should be called in a loop
  bool update();
  
  //! @brief Sets the linear and angular velocity of the siar
  //! @param linear (IN) -- linear velocity in m/s
  //! @param angular (IN) -- angular velocity in rad/s
  //! @retval true Could establish the velocity
  //! @retval false Errors found while estabilishing the velocity
  bool setVelocity(double linear, double angular);

  //! @brief Gets the incremental move distance (IMDL,IMDR) from enconder increments
  //! @param imdl (OUT) The IMD for left wheel in meters (it is the mean of the three wheels)
  //! @param imdr (OUT) The IMD for right wheel in meters (it is the mean of the three wheels)
  virtual bool getIMD(double& imdl, double& imdr);

  //! --------------------- RAW commands for calibration purposes
  
  //! @brief Sets the raw velocities of each motor
  //! @param left Left velocity
  //! @param right Right velocity
  //! @retval true Success
  //! @retval false Error
  bool setRawVelocity(int16_t left, int16_t right);
  
  bool setRawVelocityCarlos(int16_t left, int16_t right, int16_t left_out, int16_t right_out);
  
  protected:
  SiarSerial siar_serial_1; // 1 board, six motors 
  SiarSerial battery_serial; // Interface for acquiring battery data
  SerialInterface joy_serial; 
  ros::Time last_cmd;
  bool first_odometry;
  bool controlled, has_joystick;
  siar_driver::SiarBatteryMonitor motor_battery_status, elec_battery_status;
  ros::Publisher motor_status_pub, elec_status_pub;
  ros::Subscriber slow_motion_sub, reverse_sub;
  int bat_cont, bat_skip;
  bool slow_motion, reverse;
  
  //! @brief Calculates the increments of the position and orientation variables of the robot by integrating odometry measures
  //! @retval true Got ticks from the encoders and performed the calculations
  //! @retval false Some error while retrieving the ticks
  bool calculateOdometry();
  
  bool getJoystickActuate();
  
  bool sendHardStop();
  
  bool actualizeBatteryStatus();
  void slowReceived(const std_msgs::Bool& cmd_vel);
  void reverseReceived(const std_msgs::Bool& reverse);
};

//-- END OF HEADERS ----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ----------------------------------------------------------------------------
// All the neccesary code is here :-)

/***********************************************/
/** SiarManagerBatteryMonitor implementation   */
/***********************************************/

inline SiarManagerBatteryMonitor::SiarManagerBatteryMonitor(const std::string& device_1, 
                                                            const std::string& device_2,
                                                            const std::string& device_battery,
							    const SiarConfig &config) :
siar_serial_1(device_1), joy_serial(device_2, false), battery_serial(device_battery), 
first_odometry(true), controlled(false), has_joystick(true), bat_cont(0), bat_skip(100), slow_motion(false), reverse(false)
{
  state.is_stopped = true;
  
  // Initialize inherited fields
  _config = config;
  imu_ = NULL;
  first_odometry = true;
  
  #ifdef _SIAR_MANAGER_DEBUG_
  _printTime();
  std::cout << "Connecting to Siar"<<std::endl;
  #endif
  
  if (!siar_serial_1.open()) {
    
#ifdef _SIAR_MANAGER_DEBUG_
    std::cerr << "Could not connect to siar.\n";
#endif
    
    throw SiarManagerException(siar_serial_1.getLastError());
  } else {
#ifdef _SIAR_MANAGER_DEBUG_
    _printTime();
    std::cout << "Connected to Siar board 1"<<std::endl;
#endif
  }
  
  if (!joy_serial.open(B19200)) {
    std::cout << "Warning: Could not connect to joystick.\n";
    has_joystick = false;
  } else {
#ifdef _SIAR_MANAGER_DEBUG_
    _printTime();
    std::cout << "Connected to Joystick"<<std::endl;
    has_joystick = true;
#endif
  }
  if (!battery_serial.open()) {
    
#ifdef _SIAR_MANAGER_DEBUG_
    std::cerr << "Could not connect to the battery monitoring board.\n";
#endif
    
    throw SiarManagerException(joy_serial.getLastError());
  } else {
#ifdef _SIAR_MANAGER_DEBUG_
    _printTime();
    std::cout << "Connected to the battery monitor board"<<std::endl;
#endif
  }
  
  ros::NodeHandle pn("~");
  motor_status_pub = pn.advertise<siar_driver::SiarBatteryMonitor>("/motor_battery_status", 5);
  elec_status_pub= pn.advertise<siar_driver::SiarBatteryMonitor>("/elec_battery_status", 5);
  slow_motion_sub = pn.subscribe("/slow_motion", 1, &SiarManagerBatteryMonitor::slowReceived, this);
  reverse_sub = pn.subscribe("/reverse", 1, &SiarManagerBatteryMonitor::reverseReceived, this);
  
  // Stop robot
  setVelocity(0.0f, 0.0f);
  resetOdometry();
  
  // Last cmd
  last_cmd = ros::Time::now() - ros::Duration(CMD_TIME_OUT);
}

void SiarManagerBatteryMonitor::slowReceived(const std_msgs::Bool& cmd_vel)
{
  slow_motion = cmd_vel.data;
}

void SiarManagerBatteryMonitor::reverseReceived(const std_msgs::Bool& cmd_vel)
{
  reverse = cmd_vel.data;
}

inline SiarManagerBatteryMonitor::~SiarManagerBatteryMonitor() {
  if (siar_serial_1.isOpen() ) {
    // If the connection was open --> stop Siar and... 
    setVelocity(0.0f, 0.0f);
  }
}

// TODO: check it!
inline bool SiarManagerBatteryMonitor::getIMD(double& imdl, double& imdr)
{
  unsigned char command[1];
  command[0] = _config.get_enc;
  
  // Ask for odometry in both ports
  bool ret_val = siar_serial_1.write(command, 1);
  unsigned char buffer[16];
  
#ifdef _SIAR_MANAGER_DEBUG_
//   if (!ret_val) {
//     std::cout << "Error in write" << std::endl;
//   } else {
//     std::cout << "Write OK" << std::endl;
//   }
#endif
  
  int16_t m_left, m_right;

  // Signs change because of IMU ref
  if (ret_val && siar_serial_1.getResponse(buffer, 16) && !first_odometry) {
    m_left = -(int16_t)(((unsigned char)buffer[5] << 8) + buffer[6]);
    m_right =  (int16_t)(((unsigned char)buffer[7] << 8) + buffer[8]);
  }
  
#ifdef _SIAR_MANAGER_DEBUG_
  if  (m_right != 0 || m_left != 0 && !first_odometry) 
  {
    std::cout << "Middle right = " << m_right << "\t left = " << m_left << std::endl;
  }
#endif

// 	
  if (m_left == 0 && m_right == 0) {
    state.is_stopped = true;
    imdl = 0.0;
    imdr = 0.0;
  }
  else
  {
    state.is_stopped = false;
    
    // The odometry will be the mean of all three values
    if (m_left > 0) {
      imdl = (double)m_left * _config.meters_tick_l;
    } else {
      imdl = (double)m_left * _config.meters_tick_l_b;
    }
    if (m_right > 0) {
      imdr = (double)m_right * _config.meters_tick_r;
    } else {
      imdr = (double)m_right * _config.meters_tick_r_b;
    }
    state.middle_left_ticks += m_left;
    state.middle_right_ticks += m_right;
  }
  
  return ret_val;
}

// Updates all but the odometry? 
inline bool SiarManagerBatteryMonitor::update()
{
  bool ret_val = true;
  
  calculateOdometry();
  
  if (has_joystick) 
    getJoystickActuate();
  
  // New: actualize and publish battery status
  bat_cont++;
  if (bat_cont >= bat_skip)
  {
    actualizeBatteryStatus();
    bat_cont = 0;
  }
  
  return ret_val;
}


inline bool SiarManagerBatteryMonitor::setRawVelocity(int16_t left, int16_t right) {
  bool ret_val = true;
  
  if(slow_motion) {
    left /= 4;
    right /= 4;
  }
  
  // get the right command
  unsigned char command[13];
  command[0] = _config.set_vel;
  command[1] = (unsigned char)(left >> 8);
  command[2] = (unsigned char)(left & 0xFF);
  command[3] = (unsigned char)(right >> 8);
  command[4] = (unsigned char)(right & 0xFF);
  command[5] = (unsigned char)(left >> 8);
  command[6] = (unsigned char)(left & 0xFF);
  command[7] = (unsigned char)(right >> 8);
  command[8] = (unsigned char)(right & 0xFF);
  command[9] = (unsigned char)(left >> 8);
  command[10] = (unsigned char)(left & 0xFF);
  command[11] = (unsigned char)(right >> 8);
  command[12] = (unsigned char)(right & 0xFF);
  
  // Send it to front and rear board and wait for response
  ret_val = siar_serial_1.write(command, 13);
  unsigned char response[4];
  ret_val = siar_serial_1.getResponse(response, 4);
  
  return ret_val;
}

inline bool SiarManagerBatteryMonitor::setVelocity(double linear, double angular)
{
  unsigned char command[13];
  
  if ((ros::Time::now() - last_cmd).toSec() < CMD_TIME_OUT && has_joystick) {
    // The joystick commands have priority
    return true;
  }
  
  command[0] = _config.set_vel;
  
  linear = _config.velocity_sat.apply(linear);
  angular = _config.velocity_sat.apply(angular);

  // Following idMind driver --> v_r = (linear - L * angular)    ; v_l = -(linear + L * angular) 
  double right_speed = (linear - _config.half_axis_distance * angular);
  double left_speed = -(linear + _config.half_axis_distance * angular);
  
  left_speed = _config.velocity_sat.apply(left_speed);
  right_speed = _config.velocity_sat.apply(right_speed);
  
  // Convert from left and right speed velocities to [-1100, 1100]
  // TODO: this is deprecated
  int v_left = left_speed * ((double)_config.vel_int_sat._high) / (_config.velocity_sat._high * _config.peri_wheel);
  int v_right = right_speed * ((double)_config.vel_int_sat._high) / (_config.velocity_sat._high * _config.peri_wheel);
  
  // TODO: Use the empirical relationship
//    v_left = -( linear * (linear > 0.0)? _config.v_m_s_to_v_raw : _config.v_m_s_to_v_raw_b + angular * (angular > 0.0)?_config.ang_vel_to_raw_l:_config.ang_vel_to_raw_r);
//    v_right = linear * (linear > 0.0)? _config.v_m_s_to_v_raw : _config.v_m_s_to_v_raw_b - angular * (angular > 0.0)?_config.ang_vel_to_raw_l:_config.ang_vel_to_raw_r;
  
  #ifdef _SIAR_MANAGER_DEBUG_
  if (linear > 0.01 || angular > 0.01) {
    std::cout << "SiarManagerBatteryMonitor::setVelocity --> v = " << linear << "\t w = " << angular << "\t ";
    std::cout << "\t l_speed = " << left_speed << "\t r_speed = " << right_speed << "\t ";
    std::cout << "\t l_int = " << v_left << "\t r_int = " << v_left << std::endl;
  }
#endif
  
  return setRawVelocity(v_left, v_right);
}

bool SiarManagerBatteryMonitor::calculateOdometry()
{
  double dl, dr;
  struct timespec curr_time;
  clock_gettime(CLOCK_REALTIME, &curr_time);
  bool ret_val = getIMD(dl, dr);
  
  if (ret_val) {
    double dist = ((dl + dr) * 0.5);
    dist = _config.encoders_dead.apply(dist);
    
     double d_theta_ticks = (dr - dl) / _config.estimated_diag;	 	//rad
    
    if (first_odometry) {
      first_odometry = false;
      clock_gettime(CLOCK_REALTIME, &state.last_update);
    } else {
      // Actualize the linear and angular speeds
      double d_t = curr_time.tv_sec - state.last_update.tv_sec + (curr_time.tv_nsec - state.last_update.tv_nsec)*1e-9;
      state.linear_velocity = dist / d_t;
      double angular_rate_ticks = d_theta_ticks / d_t;
      double yaw = tf::getYaw(state.odom.pose.pose.orientation);
      
      state.odom.header.stamp = ros::Time(curr_time.tv_sec, curr_time.tv_nsec);
      // Correct the angles with the IMU, if available
      if (imu_) {
	sensor_msgs::Imu angles = imu_->getFilteredAngles(angular_rate_ticks);
	state.odom.pose.pose.orientation = angles.orientation;
	state.odom.twist.twist.angular = angles.angular_velocity;
      } else {
	state.odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
	state.odom.twist.twist.angular.z = angular_rate_ticks;
      }
      double eff_angle = yaw + state.odom.twist.twist.angular.z * d_t * 0.5;
      // We use the odometry model in Thurn, Burgard, Fox; considering that the two turns have equal value (half of the total)
      state.odom.twist.twist.linear.x = state.linear_velocity * cos(eff_angle);
      state.odom.twist.twist.linear.y = state.linear_velocity * sin(eff_angle);
      state.odom.pose.pose.position.x += dist * cos(eff_angle);
      state.odom.pose.pose.position.y += dist * sin(eff_angle);
      state.odom.pose.pose.position.z = 0.0; // TODO: 3D odometry?
      
      state.last_update = curr_time;
    }
  }
  
  return ret_val;
}

bool SiarManagerBatteryMonitor::getJoystickActuate()
{
  int bytes = 0;
  joy_serial.incomingBytes(bytes);
  if ( bytes >= 6)
  {
    unsigned char *buffer = new unsigned char[bytes];
    joy_serial.read_stream(buffer, bytes);
    if (buffer[0] == (unsigned char)0xFF && buffer[5] == (unsigned char)0x00)
    {

      if (buffer[1] == 1) //release control
      {
	  //Sending the velocities
        if (controlled) {
	  controlled = false;
	  setRawVelocity(0.0, 0.0);
	  last_cmd = ros::Time::now();
#ifdef _SIAR_MANAGER_DEBUG_
          ROS_INFO("Removing control");
#endif
        }
      }
      else if (buffer[1] == 2) //hardstop
      {
	sendHardStop();
	ROS_ERROR("Sending hard stop command to the SIAR\n");
      }
      else if (buffer[1] == 3) //Gain control
      {
        if (!controlled) {
          controlled = true;
#ifdef _SIAR_MANAGER_DEBUG_
          ROS_INFO("Getting control");
#endif
        }
        last_cmd = ros::Time::now();
	int16_t vel_Rotation, vel_Rotation_out, vel_Forward;
	int16_t Left_ref, Right_ref;
	int16_t Left_ref_out, Right_ref_out;
	vel_Rotation = -(int16_t) (((buffer[3]-1)-100)*8);
	vel_Rotation_out = -(int16_t) (((buffer[3]-1)-100)*10);
	vel_Forward = -(int16_t) (((buffer[2]-1) -100) *15);
        if (reverse) 
          vel_Forward *= -1;

#ifdef _SIAR_MANAGER_DEBUG_
        ROS_INFO("vel_Rotation = %d\tvel_Rotation_out=%dtvel_forward=%d",
                 vel_Rotation, vel_Rotation_out, vel_Forward);
#endif
	Left_ref = (int16_t)-(vel_Forward + vel_Rotation);
	Right_ref = (int16_t)(vel_Forward - vel_Rotation);
	Left_ref_out = (int16_t)-(vel_Forward + vel_Rotation_out);
	Right_ref_out = (int16_t)(vel_Forward - vel_Rotation_out);

	// Put a dead zone in the joystick controller
	if (Left_ref > 60) Left_ref = Left_ref - 60;
	else if (Left_ref < -60) Left_ref = Left_ref + 60;
	else Left_ref = 0;

	if (Right_ref > 60) Right_ref = Right_ref - 60;
	else if (Right_ref < -60) Right_ref = Right_ref + 60;
	else Right_ref = 0;

	if (Left_ref_out > 60) Left_ref_out = Left_ref_out - 60;
	else if (Left_ref_out < -60) Left_ref_out = Left_ref_out + 60;
	else Left_ref_out = 0;

	if (Right_ref_out > 60) Right_ref_out = Right_ref_out - 60;
	else if (Right_ref_out < -60) Right_ref_out = Right_ref_out + 60;
	else Right_ref_out = 0;
        
#ifdef _SIAR_MANAGER_DEBUG_
        ROS_INFO("Left_ref = %d \t Right_ref = %d",Left_ref, Right_ref);
        ROS_INFO("OUT: Left_ref = %d \t Right_ref = %d",Left_ref_out, Right_ref_out);
#endif
	
	setRawVelocityCarlos(Left_ref, Right_ref, Left_ref_out, Right_ref_out);
      }
    } else {
      joy_serial.flush();
    }
    delete [] buffer;
  } else {
    if ((ros::Time::now() - last_cmd).toSec() < CMD_TIME_OUT && !controlled) {
      setRawVelocity(0.0, 0.0);
    }
  }
}

inline bool SiarManagerBatteryMonitor::sendHardStop() 
{
  bool ret_val = true;
  unsigned char command[1];
  command[0] = _config.hard_stop;
  unsigned char buffer[4];
  // Send Hard stop to boards
  ret_val &= siar_serial_1.write(command, 1);
  siar_serial_1.getResponse(buffer, 4);
  return ret_val;
}

inline bool SiarManagerBatteryMonitor::setRawVelocityCarlos(int16_t left, int16_t right, int16_t left_out, int16_t right_out) {
  bool ret_val = true;
  
  if(slow_motion) {
    left_out /= 4;
    right_out /= 4;
    left /= 4;
    right /= 4;
  }
  
  // get the right command
  unsigned char command[13];
  command[0] = _config.set_vel;
  command[1] = (unsigned char)(left_out >> 8);
  command[2] = (unsigned char)(left_out & 0xFF);
  command[3] = (unsigned char)(right_out >> 8);
  command[4] = (unsigned char)(right_out & 0xFF);
  command[5] = (unsigned char)(left >> 8);
  command[6] = (unsigned char)(left & 0xFF);
  command[7] = (unsigned char)(right >> 8);
  command[8] = (unsigned char)(right & 0xFF);
  command[9] = (unsigned char)( left_out >> 8);
  command[10] = (unsigned char)(left_out & 0xFF);
  command[11] = (unsigned char)(right_out >> 8);
  command[12] = (unsigned char)(right_out & 0xFF);
  
  ret_val = siar_serial_1.write(command, 13);
  unsigned char response[4];
  ret_val = siar_serial_1.getResponse(response, 4);
  
  return ret_val;
}

bool SiarManagerBatteryMonitor::actualizeBatteryStatus()
{
  bool ret_val = true;
  unsigned char buffer[16];
  
  // Ask for batteries voltage
  ret_val = battery_serial.write(&_config.get_voltage, 1);
  if (ret_val && battery_serial.getResponse(buffer, 8)) {
    motor_battery_status.voltage = ((int16_t)(((unsigned char)buffer[1] << 8) + buffer[2]))/10.0;
    elec_battery_status.voltage = ((int16_t)(((unsigned char)buffer[3] << 8) + buffer[4]))/10.0;
  }
  
  // Ask for batteries current
  ret_val &= battery_serial.write(&_config.get_inst_curr, 1);
  if (ret_val && battery_serial.getResponse(buffer, 8)) {
    elec_battery_status.current = ((int16_t)(((unsigned char)buffer[1] << 8) + buffer[2]))/1000.0;
    motor_battery_status.current = ((int16_t)(((unsigned char)buffer[3] << 8) + buffer[4]))/1000.0;
  }
  
  // Ask for integrated current
  ret_val &= battery_serial.write(&_config.get_integ_curr, 1);
  if (ret_val && battery_serial.getResponse(buffer, 8)) {
    elec_battery_status.integrated_current = ((int16_t)(((unsigned char)buffer[1] << 8) + buffer[2]))/1000.0;
    motor_battery_status.integrated_current = ((int16_t)(((unsigned char)buffer[3] << 8) + buffer[4]))/1000.0;
  }
  
  // Ask for battery level and remaining time
  ret_val &= battery_serial.write(&_config.get_battery_level, 1);
  if (ret_val && battery_serial.getResponse(buffer, 10)) {
    elec_battery_status.percentage = buffer[1];
    elec_battery_status.remaining_time = ((int16_t)(((unsigned char)buffer[2] << 8) + buffer[3]));
    motor_battery_status.percentage = buffer[4];
    motor_battery_status.remaining_time = ((int16_t)(((unsigned char)buffer[5] << 8) + buffer[6]));
  }
  
  // Publish the results if there is new data
  if (ret_val) {
    elec_status_pub.publish(elec_battery_status);
    motor_status_pub.publish(motor_battery_status);
  }
  
  return ret_val;
}


//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
