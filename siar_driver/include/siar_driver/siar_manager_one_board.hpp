/***********************************************************************/
/**                                                                    */
/** siar_manager_one_board.hpp                                         */
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


// TODO: All conversion giraff--> siar

#ifndef _SIAR_MANAGER_ONE_BOARD_CARLOS_H_
#define _SIAR_MANAGER_ONE_BOARD_CARLOS_H_

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

#define CMD_TIME_OUT 5 // In secs

//-- HEADERS ----------------------------------------------------------------------------

//! @class SiarManagerOneBoard                                                          */
//! @brief This manages the Siar controller. Includes velocity and arm commands

class SiarManagerOneBoard:public SiarManager
{
  public:
  //! @brief Recommended constructor
  //! @param device_1 Device name where the first Siar board is linked ex. "/dev/ttyUSB0"
  //! @param device_2 Device name where the joystick is linked ex. "/dev/ttyUSB0"
  //! @param params Configuration parameters of the Siar
  //!  Comments:
  //!     The serial connections will be stablished in the constructor, if some
  //!     error happens, a SiarManagerException will be thrown.
  SiarManagerOneBoard(const std::string& device_1, const std::string &device_2, const SiarConfig &config);
  
  //! @brief Destructor. Stops the robot and closes all open connections
  ~SiarManagerOneBoard();
  
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
  SerialInterface joy_serial; 
  ros::Time last_cmd;
  bool first_odometry;
  bool controlled;
  
  //! @brief Calculates the increments of the position and orientation variables of the robot by integrating odometry measures
  //! @retval true Got ticks from the encoders and performed the calculations
  //! @retval false Some error while retrieving the ticks
  bool calculateOdometry();
  
  bool getJoystickActuate();
  
  bool sendHardStop();
};

//-- END OF HEADERS ----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ----------------------------------------------------------------------------
// All the neccesary code is here :-)

/***********************************************/
/** SiarManagerOneBoard implementation   */
/***********************************************/

inline SiarManagerOneBoard::SiarManagerOneBoard(const std::string& device_1, 
                                                            const std::string& device_2,
							    const SiarConfig &config) :
siar_serial_1(device_1), joy_serial(device_2, false), first_odometry(true), controlled(false)
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
    
#ifdef _SIAR_MANAGER_DEBUG_
    std::cerr << "Could not connect to joystick.\n";
#endif
    
    throw SiarManagerException(joy_serial.getLastError());
  } else {
#ifdef _SIAR_MANAGER_DEBUG_
    _printTime();
    std::cout << "Connected to Joystick"<<std::endl;
#endif
  }
  
  // Stop robot
  setVelocity(0.0f, 0.0f);
  resetOdometry();
  
  // Last cmd
  last_cmd = ros::Time::now() - ros::Duration(CMD_TIME_OUT);
}

inline SiarManagerOneBoard::~SiarManagerOneBoard() {
  if (siar_serial_1.isOpen() ) {
    // If the connection was open --> stop Siar and... 
    setVelocity(0.0f, 0.0f);
  }
}

// TODO: check it!
inline bool SiarManagerOneBoard::getIMD(double& imdl, double& imdr)
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

// Updates all but the odometry? --> TODO: integrate IMD, and last velocity
inline bool SiarManagerOneBoard::update()
{
  bool ret_val = true;
  
  calculateOdometry();
  
  getJoystickActuate();
  
  return ret_val;
}


inline bool SiarManagerOneBoard::setRawVelocity(int16_t left, int16_t right) {
  bool ret_val = true;
  
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

inline bool SiarManagerOneBoard::setVelocity(double linear, double angular)
{
  unsigned char command[13];
  
  if ((ros::Time::now() - last_cmd).toSec() < CMD_TIME_OUT) {
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
    std::cout << "SiarManagerOneBoard::setVelocity --> v = " << linear << "\t w = " << angular << "\t ";
    std::cout << "\t l_speed = " << left_speed << "\t r_speed = " << right_speed << "\t ";
    std::cout << "\t l_int = " << v_left << "\t r_int = " << v_left << std::endl;
  }
#endif
  
  return setRawVelocity(v_left, v_right);
}

bool SiarManagerOneBoard::calculateOdometry()
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

bool SiarManagerOneBoard::getJoystickActuate()
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
          ROS_INFO("Removing control");
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
          ROS_INFO("Getting control");
        }
        last_cmd = ros::Time::now();
	int16_t vel_Rotation, vel_Rotation_out, vel_Forward;
	int16_t Left_ref, Right_ref;
	int16_t Left_ref_out, Right_ref_out;
	vel_Rotation = -(int16_t) (((buffer[3]-1)-100)*8);
	vel_Rotation_out = -(int16_t) (((buffer[3]-1)-100)*10);
	vel_Forward = -(int16_t) (((buffer[2]-1) -100) *15);

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
  }
}

inline bool SiarManagerOneBoard::sendHardStop() 
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

inline bool SiarManagerOneBoard::setRawVelocityCarlos(int16_t left, int16_t right, int16_t left_out, int16_t right_out) {
  bool ret_val = true;
  
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
  
  // Send it to front and rear board and wait for response
  ret_val = siar_serial_1.write(command, 13);
  unsigned char response[4];
  ret_val = siar_serial_1.getResponse(response, 4);
  
  return ret_val;
}

//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
