/***********************************************************************/
/**                                                                    */
/** siar_manager_width_adjustment.hpp                                  */
/**                                                                    */
/** Copyright (c) 2015-17, Service Robotics Lab.                       */ 
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


#ifndef _SIAR_MANAGER_WIDTH_ADJUSTMENT_H_
#define _SIAR_MANAGER_WIDTH_ADJUSTMENT_H_

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
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#define CMD_TIME_OUT 5 // In secs
#define N_HERCULEX 5      // Number of herculex motors
#define ___MAX_BUFFER_SIAR___ 256

//-- HEADERS ----------------------------------------------------------------------------

//! @class SiarManagerWidthAdjustment                                                          */
//! @brief This manages the Siar controller. Includes velocity and arm commands

class SiarManagerWidthAdjustment:public SiarManager
{
  public:
  //! @brief Recommended constructor
  //! @param device_1 Device route of the first Siar board. Ex. "/dev/ttyUSB0"
  //! @param device_2 Device route of the joystick. Ex. "/dev/ttyUSB0"
  //! @param device_battery Device route of the battery management board. Ex. "/dev/serial/by-id/..."
  //! @param params Configuration parameters of the Siar
  //!  Comments:
  //!     The serial connections will be stablished in the constructor, if some
  //!     error happens, a SiarManagerException will be thrown.
  SiarManagerWidthAdjustment(const std::string& device_1, const std::string &device_2, 
                            const std::string& device_battery, const SiarConfig &config);
  
  //! @brief Destructor. Stops the robot and closes all open connections
  ~SiarManagerWidthAdjustment();
  
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
  
  /// ------------ New commands Jul 2017 (width adjustment)---------------
  
  // ----- SET COMMANDS -------------------
  //! @brief Sets the linear and angular velocity of the linear motor for width adjustment
  //! @param value (IN) -- linear velocity [0, 1024]
  //! @retval true Could establish the velocity
  //! @retval false Errors found while estabilishing the velocity
  bool setLinearVelocity(u_int16_t value);
  
  //! @brief Sets the position of the linear motor in the SIAR
  //! @param value --> new pos [0,1024]
  //! @retval true Could establish the velocity
  //! @retval false Errors found while estabilishing the velocity
  bool setLinearPosition(u_int16_t value);
  
  //! @brief Sets the linear and angular velocity of the siar
  //! @param value --> new time [0,255]
  //! @retval true Could establish the velocity
  //! @retval false Errors found while estabilishing the velocity
  bool setHardStopTime(u_int8_t value);
  
  //! @brief Sets the linear and angular velocity of the siar
  //! @param on true --> FAN active
  //! @retval true OK
  //! @retval false Errors found 
  bool setFan(bool on);
  
  //// -------------- GET COMMANDS ----------------- 
  
  //! @brief Gets the position of the width adjustment motor
  //! @param pos (OUT) The measured position
  bool getLinearMotorPosition(u_int16_t &pos);
  
  //! @brief Gets the potentiometer of the width adjustment motor
  //! @param pot (OUT) The measured position
  bool getLinearMotorPotentiometer(u_int16_t &pot);
  
  //! @brief Gets the time to perform the hard stop
  //! @param hard_time (OUT) The time of hard stop [0, 255] s
  bool getHardStopStatus(bool &active);
  
  //! @brief Gets the time to perform the hard stop
  //! @param hard_time (OUT) The time of hard stop [0, 255] s
  bool getHardStopTime(uint8_t &hard_time);
  
  //! @brief Gets the FAN activation status
  //! @param active (OUT) 0 --> OFF ; 1 --> ON
  bool getFanControl(bool& active);
  
  //! @brief Gets the installed FW versions of both boards (stores them into the class)
  bool getFWVersions();
  
  
  /// ------------Battery management and arm commands  (new July 2017) -----------------------
  
  // Battery and arm set commands
  //! @brief Turns the light ON and OFF
  //! @param on --> 0 = OFF; 1 = ON
  //! @retval true OK
  //! @retval false Errors found 
  bool setLights(bool front_light, bool rear_light);
  
  //! @brief Sets the direction of each PIN
  //! @param new_val =[XXFEDCBA]          
  //!    where:                                
  //! A to F - Pin F0 to F5                  
  //!    A to F = 0 output                    
  //! A to F = 1   input                    
  //! @retval true OK
  //! @retval false Errors found 
  bool setAuxPinsDirection(uint8_t new_val);
  
  //! @brief Sets the value of each PIN
  //! @param new_val =[XXFEDCBA]          
  //!    where:                                
  //! A to F - Pin F0 to F5                  
  //!    A to F = 0=O --> 0V
  //! A to F = 1=O  --> 5V
  //! @retval true OK
  //! @retval false Errors found 
  bool setAuxPinsValues(uint8_t new_val);
  
  //! @brief Sets the value of the torque of one joint
  //! @param id --> ID of the joint [0, 4]
  //! @param value --> 0 = Off ; 0x40 = brake ; 0x60 = on
  //! @retval true OK
  //! @retval false Errors found 
  bool setHerculexTorque(uint8_t id, uint8_t value);
  
  //! @brief Sets the value of the torque of one joint
  //! @param id --> ID of the joint [0, 4]
  //! @param value --> 0 = Off ; 0x40 = brake ; 0x60 = on
  //! @param time_to_go --> time to reach destination [0, 255] (if not in range --> default value 3s)
  //! @retval true OK
  //! @retval false Errors found 
  bool setHerculexPosition(uint8_t id, uint16_t value, int time_to_go);
  
  //! @brief Clears any errors of a joint
  //! @param id --> ID of the joint [0, 4]
  //! @retval true OK
  //! @retval false Errors found 
  bool setHerculexClearStatus(uint8_t id);
  
  // --------------------- get Herculex status -------------------  
  //! @brief Updates the Herculex status
  bool getHerculexStatus();
  
  //! @brief Updates the Herculex torque
  bool getHerculexTorque();
  
  //! @brief Updates the Herculex position
  bool getHerculexPosition();
  
  //! @brief Updates the Herculex temperature
  bool getHerculexTemperature();
  
  // --------------------- new get Battery management functions -------------------  
  //! @brief Updates power supply status
  bool getPowerSupply();
  
  //! @brief Gets the aux pin values
  bool getAuxPinValues();
  
  //! --------------------- RAW commands for calibration purposes ----------------------------
  
  //! @brief Sets the raw velocities of each motor
  //! @param left Left velocity
  //! @param right Right velocity
  //! @retval true Success
  //! @retval false Error
  bool setRawVelocity(int16_t left, int16_t right);
  
  bool setRawVelocityCarlos(int16_t left, int16_t right, int16_t left_out, int16_t right_out);
  
  protected:
  // Serial Communciations stuff
  SiarSerial siar_serial_1; // 1 board, six motors 
  SiarSerial battery_serial; // Interface for acquiring battery data
  SerialInterface joy_serial; 
  ros::Time last_cmd;
  unsigned char command[___MAX_BUFFER_SIAR___], buffer[___MAX_BUFFER_SIAR___];
  
  
  
  // Status flags
  bool first_odometry;
  bool controlled, has_joystick;
  ros::Publisher state_pub;
  ros::Subscriber slow_motion_sub, reverse_sub;
  int bat_cont, bat_skip;
  
  //! @brief Calculates the increments of the position and orientation variables of the robot by integrating odometry measures
  //! @retval true Got ticks from the encoders and performed the calculations
  //! @retval false Some error while retrieving the ticks
  bool calculateOdometry();
  
  bool getJoystickActuate();
  
  bool sendHardStop();
  
  bool actualizeBatteryStatus();
  void slowReceived(const std_msgs::Bool& cmd_vel);
  void reverseReceived(const std_msgs::Bool& reverse);
  
  inline bool checkSum(unsigned char *buffer, int tam) {
    u_int16_t checksum = (int)buffer[tam - 2] * 0x0100 + (int)buffer[tam - 1] * 0x0001;
    
    u_int16_t bytes_sum = 0;
    for (int i = 0; i < tam - 2;i++) 
    {
      bytes_sum += (u_int16_t) buffer[i];
    }
    return checksum == bytes_sum;
  }
};

//-- END OF HEADERS ----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ----------------------------------------------------------------------------
// All the neccesary code is here :-)

/***********************************************/
/** SiarManagerWidthAdjustment implementation   */
/***********************************************/

inline SiarManagerWidthAdjustment::SiarManagerWidthAdjustment(const std::string& device_1, 
                                                            const std::string& device_2,
                                                            const std::string& device_battery,
							    const SiarConfig &config) :
siar_serial_1(device_1), joy_serial(device_2, false), battery_serial(device_battery), 
first_odometry(true), controlled(false), has_joystick(true), bat_cont(0), bat_skip(10)
{
  state.is_stopped = true;
  state.slow = false;
  state.reverse = false;
  
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
  state_pub = pn.advertise<siar_driver::SiarStatus>("/siar_status", 5);
  slow_motion_sub = pn.subscribe("/slow_motion", 1, &SiarManagerWidthAdjustment::slowReceived, this);
  reverse_sub = pn.subscribe("/reverse", 1, &SiarManagerWidthAdjustment::reverseReceived, this);
  
  // Stop robot
  setVelocity(0.0f, 0.0f);
  resetOdometry();
  
  // Last cmd
  last_cmd = ros::Time::now() - ros::Duration(CMD_TIME_OUT);
  
  // Get FW version
  getFWVersions();
}

void SiarManagerWidthAdjustment::slowReceived(const std_msgs::Bool& cmd_vel)
{
//   slow_motion = cmd_vel.data;
  state.slow = cmd_vel.data;
}

void SiarManagerWidthAdjustment::reverseReceived(const std_msgs::Bool& cmd_vel)
{
//   reverse = cmd_vel.data;
  state.reverse = cmd_vel.data;
}

inline SiarManagerWidthAdjustment::~SiarManagerWidthAdjustment() {
  if (siar_serial_1.isOpen() ) {
    // If the connection was open --> stop Siar and... 
    setVelocity(0.0f, 0.0f);
  }
}

// TODO: check it!
inline bool SiarManagerWidthAdjustment::getIMD(double& imdl, double& imdr)
{
  command[0] = _config.get_enc;
  
  // Ask for odometry in both ports
  bool ret_val = siar_serial_1.write(command, 1);
  
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

inline bool SiarManagerWidthAdjustment::update()
{
  bool ret_val = true;
  
  calculateOdometry();
  
  if (has_joystick) 
    getJoystickActuate();
  
  // New: actualize and publish battery status
  bat_cont++;
  if (bat_cont >= bat_skip)
  {
    // Actualize battery and power supply data
    actualizeBatteryStatus();
    getPowerSupply();
    bat_cont = 0;
    
    // Update arm status
    getHerculexPosition();
    getHerculexStatus();
    getHerculexTemperature();
    getHerculexTorque();
    
    // Update aux pins
    getAuxPinValues();
    
    // Linear motors
    getLinearMotorPosition(state.lin_motor_pos);
    getLinearMotorPotentiometer(state.lin_motor_pot);
    
    // 
    bool active;
    getHardStopStatus(active);
    state.hard_stop = active?1:0;
    getHardStopTime(state.hard_stop_time);
  }
  
  // Publish state
  state_pub.publish(state);
  
  return ret_val;
}


inline bool SiarManagerWidthAdjustment::setRawVelocity(int16_t left, int16_t right) {
  bool ret_val = true;
  
  if(state.slow) {
    left /= 4;
    right /= 4;
  }
  
  // get the right command
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
  ret_val = siar_serial_1.getResponse(buffer, 4);
  
  return ret_val;
}

inline bool SiarManagerWidthAdjustment::setVelocity(double linear, double angular)
{
  if ((ros::Time::now() - last_cmd).toSec() < CMD_TIME_OUT && has_joystick) {
    // The joystick commands have priority
    return true;
  }
  
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
    std::cout << "SiarManagerWidthAdjustment::setVelocity --> v = " << linear << "\t w = " << angular << "\t ";
    std::cout << "\t l_speed = " << left_speed << "\t r_speed = " << right_speed << "\t ";
    std::cout << "\t l_int = " << v_left << "\t r_int = " << v_left << std::endl;
  }
#endif
  
  return setRawVelocity(v_left, v_right);
}

bool SiarManagerWidthAdjustment::calculateOdometry()
{
  double dl, dr;
  bool ret_val = getIMD(dl, dr);
  
  if (ret_val) {
    double dist = ((dl + dr) * 0.5);
    dist = _config.encoders_dead.apply(dist);
    
     double d_theta_ticks = (dr - dl) / _config.estimated_diag;	 	//rad
    
    if (first_odometry) {
      first_odometry = false;
    } else {
      // Actualize the linear and angular speeds
      ros::Time t = ros::Time::now();
      double d_t = (t - state.header.stamp).toSec();
      state.speed = dist / d_t;
      double angular_rate_ticks = d_theta_ticks / d_t;
      double yaw = tf::getYaw(state.odom.pose.pose.orientation);
      
      state.odom.header.stamp = t;
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
      state.odom.twist.twist.linear.x = state.speed * cos(eff_angle);
      state.odom.twist.twist.linear.y = state.speed * sin(eff_angle);
      state.odom.pose.pose.position.x += dist * cos(eff_angle);
      state.odom.pose.pose.position.y += dist * sin(eff_angle);
      state.odom.pose.pose.position.z = 0.0; // TODO: 3D odometry?
      
      state.header.stamp = t;
      state.header.seq++;
    }
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getJoystickActuate()
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
        if (state.reverse) 
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

inline bool SiarManagerWidthAdjustment::sendHardStop() 
{
  bool ret_val = true;
  command[0] = _config.hard_stop;
  // Send Hard stop to boards
  ret_val &= siar_serial_1.write(command, 1);
  siar_serial_1.getResponse(buffer, 4);
  return ret_val;
}

inline bool SiarManagerWidthAdjustment::setRawVelocityCarlos(int16_t left, int16_t right, int16_t left_out, int16_t right_out) {
  bool ret_val = true;
  
  if(state.slow) {
    left_out /= 4;
    right_out /= 4;
    left /= 4;
    right /= 4;
  }
  
  // get the right command
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
  ret_val = siar_serial_1.getResponse(buffer, 4);
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::actualizeBatteryStatus()
{
  bool ret_val = true;
  
  // Ask for batteries voltage
  ret_val = battery_serial.write(&_config.get_voltage, 1);
  if (ret_val && battery_serial.getResponse(buffer, 8)) {
    state.motor_battery.voltage = ((int16_t)(((unsigned char)buffer[1] << 8) + buffer[2]))/10.0;
    state.elec_battery.voltage = ((int16_t)(((unsigned char)buffer[3] << 8) + buffer[4]))/10.0;
  }
  
  // Ask for batteries current
  ret_val &= battery_serial.write(&_config.get_inst_curr, 1);
  if (ret_val && battery_serial.getResponse(buffer, 8)) {
    state.elec_battery.current = ((int16_t)(((unsigned char)buffer[1] << 8) + buffer[2]))/1000.0;
    state.motor_battery.current = ((int16_t)(((unsigned char)buffer[3] << 8) + buffer[4]))/1000.0;
  }
  
  // Ask for integrated current
  ret_val &= battery_serial.write(&_config.get_integ_curr, 1);
  if (ret_val && battery_serial.getResponse(buffer, 8)) {
    state.elec_battery.integrated_current = ((int16_t)(((unsigned char)buffer[1] << 8) + buffer[2]))/1000.0;
    state.motor_battery.integrated_current = ((int16_t)(((unsigned char)buffer[3] << 8) + buffer[4]))/1000.0;
  }
  
  // Ask for battery level and remaining time
  ret_val &= battery_serial.write(&_config.get_battery_level, 1);
  if (ret_val && battery_serial.getResponse(buffer, 10)) {
    state.elec_battery.percentage = buffer[1];
    state.elec_battery.remaining_time = ((int16_t)(((unsigned char)buffer[2] << 8) + buffer[3]));
    state.motor_battery.percentage = buffer[4];
    state.motor_battery.remaining_time = ((int16_t)(((unsigned char)buffer[5] << 8) + buffer[6]));
  }
  
  return ret_val;
}

// ----------- New width related functions

bool SiarManagerWidthAdjustment::setLinearPosition(u_int16_t value)
{
  bool ret_val = true;
  
  command[0] = _config.set_lin_pos;
  command[1] = value >> 8;
  command[2] = value & 0xFF;
  
  ret_val = siar_serial_1.write(command, 3);
  ret_val = siar_serial_1.getResponse(buffer, 4);
  
  // Checksum
  if (ret_val) {
    ret_val = buffer[0] == _config.set_lin_pos && checkSum(buffer, 4);
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setLinearVelocity(u_int16_t value)
{
  bool ret_val = true;
  command[0] = _config.set_lin_vel;
  command[1] = value >> 8;
  command[2] = value & 0xFF;
  
  ret_val = siar_serial_1.write(command, 3);
  ret_val = siar_serial_1.getResponse(buffer, 4);
  
  // Checksum
  if (ret_val) {
    ret_val = checkSum(buffer, 4) && buffer[0] == _config.set_lin_vel;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setFan(bool on)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_fan;
  command[1] = on?1:0;
  
  ret_val = siar_serial_1.write(command, 2);
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_fan;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setHardStopTime(u_int8_t value)
{
  bool ret_val = true;
  
  command[0] = _config.set_hard_time;
  command[1] = value;
  
  ret_val = siar_serial_1.write(command, 2);
  const int tam = 4;
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_hard_time;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getLinearMotorPosition(u_int16_t& pos)
{
  bool ret_val = true;
  
  command[0] = _config.get_lin_pos;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 6;
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_lin_pos;
    pos = (int)buffer[1] << 8 + (int)buffer[2];
    if (pos > 32767) pos -= 0xFFFF;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getLinearMotorPotentiometer(u_int16_t& pot)
{
  bool ret_val = true;
  
  command[0] = _config.get_lin_pot;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 6;
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_lin_pot;
    pot = (int)buffer[1] << 8 + (int)buffer[2];
    if (pot > 32767) pot -= 0xFFFF;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getHardStopStatus(bool &active)
{
  bool ret_val = true;
  
  command[0] = _config.get_hard_stop;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 5;
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_hard_stop;
    active = buffer[1] != 0;
  }
  
  return ret_val;
}


bool SiarManagerWidthAdjustment::getHardStopTime(uint8_t& hard_time)
{
  bool ret_val = true;
  
  command[0] = _config.get_hard_time;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 5;
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_hard_time;
    hard_time = buffer[1];
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getFanControl(bool& active)
{
  bool ret_val = true;
  
  command[0] = _config.get_fan;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 5;
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_fan;
    active = buffer[1] != 0;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getFWVersions()
{
  bool ret_val = true;
  const int tam = 29;
  unsigned char buffer[tam];
  unsigned char command[1];
  command[0] = _config.get_fw;
  
  
  // Get the motor FW version
  ret_val = siar_serial_1.write(command, 1);
  ret_val = siar_serial_1.getResponse(buffer, tam);
  state.motor_fw_version.resize(tam - 4);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_fw;
    for (int i = 0; i < tam - 4;i++) {
      state.motor_fw_version[i] = buffer[i + 1];
    }
  }
  
  // Get the battery FW version
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  state.battery_fw_version.resize(tam - 4);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_fw;
    for (int i = 0; i < tam - 4;i++) {
      state.battery_fw_version[i] = buffer[i + 1];
    }
  }
  
  return ret_val;
}

//-------------------------- Herculex and battery monitor commm ---------
bool SiarManagerWidthAdjustment::setLights(bool front_light, bool rear_light)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_lights;
  command[1] = front_light?1:0 + rear_light?2:0;
  
  ret_val = battery_serial.write(command, 2);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_lights;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setAuxPinsDirection(uint8_t new_val)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_aux_pin_direction;
  command[1] = new_val;
  
  ret_val = battery_serial.write(command, 2);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_aux_pin_direction;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setAuxPinsValues(uint8_t new_val)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_aux_pin_values;
  command[1] = new_val;
  
  ret_val = battery_serial.write(command, 2);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_aux_pin_values;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setHerculexTorque(uint8_t id, uint8_t value)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_herculex_torque;
  command[1] = value;
  
  ret_val = battery_serial.write(command, 2);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_herculex_torque;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setHerculexPosition(uint8_t id, uint16_t value, int time_to_go)
{
  bool ret_val = true;
  const int tam = 4;
  
  // First check time_to_go
  if (time_to_go < 0 || time_to_go > 0xFF) {
    time_to_go = 3; // Default value
  }
  
  command[0] = _config.set_herculex_position;
  command[1] = id;
  command[2] = (uint8_t)(value >> 8);
  command[3] = (uint8_t) value & 0x00FF;
  command[4] = (uint8_t) time_to_go;
  ret_val = battery_serial.write(command, 5);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_herculex_position;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setHerculexClearStatus(uint8_t id)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_herculex_clear_status;
  command[1] = id;
  
  ret_val = battery_serial.write(command, 2);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_herculex_clear_status;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getHerculexStatus()
{
  bool ret_val = true;
  const int tam = 9;
  command[0] = _config.get_herculex_status;
  
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_status;
    
    for (int i = 0; i < N_HERCULEX; i++) {
      state.herculex_status[i] = buffer[i + 1];
    }
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getHerculexTorque()
{
  bool ret_val = true;
  const int tam = 9;
  command[0] = _config.get_herculex_torque;
  
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_torque;
    
    for (int i = 0; i < N_HERCULEX; i++) {
      state.herculex_torque[i] = buffer[i + 1];
    }
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getHerculexPosition()
{
  bool ret_val = true;
  const int tam = 14;
  command[0] = _config.get_herculex_position;
  
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_position;
    
    for (int i = 0; i < N_HERCULEX; i++) {
      state.herculex_position[i] = ((uint16_t)buffer[i * 2 + 1]) << 8 + buffer[i * 2 + 2];
    }
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getHerculexTemperature()
{
  bool ret_val = true;
  const int tam = 9;
  command[0] = _config.get_herculex_temp;
  
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_temp;
    
    for (int i = 0; i < N_HERCULEX; i++) {
      state.herculex_temperature[i] = buffer[i + 1];
    }
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getPowerSupply()
{
  bool ret_val = true;
  const int tam = 14;
  command[0] = _config.get_power_supply;
  
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_power_supply;
    state.power_supply.elec_v = (int)buffer[1] << 8 + buffer[2];
    state.power_supply.elec_i = (int)buffer[3] << 8 + buffer[4];
    state.power_supply.motor_v = (int)buffer[5] << 8 + buffer[6];
    state.power_supply.motor_i = (int)buffer[7] << 8 + buffer[8];
    state.power_supply.cable_v = (int)buffer[9] << 8 + buffer[10];
  }
  
  return ret_val;
}


bool SiarManagerWidthAdjustment::getAuxPinValues()
{
  bool ret_val = true;
  const int tam = 5;
  command[0] = _config.get_aux_pin_values;
  
  ret_val = battery_serial.write(command, 1);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_aux_pin_values;
    state.aux_pins_values = buffer[1];
  }
  
  return ret_val;
}


//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
