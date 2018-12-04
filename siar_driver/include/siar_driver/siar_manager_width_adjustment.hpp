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
#define _SIAR_MANAGER_DEBUG_
#endif

#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <exception>
#include "siar_manager.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include "functions/linear_interpolator.hpp"
#include "siar_driver/arm_firewall.hpp"

#define CMD_TIME_OUT 5 // In secs
#define ___MAX_BUFFER_SIAR___ 256

//-- HEADERS ----------------------------------------------------------------------------

//! @class SiarManagerWidthAdjustment                                                          */
//! @brief This manages the Siar controller. Includes velocity and arm commands

// TODO: Refactor without any kind of ROS stuff

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
  bool setLights(bool front_light, bool rear_light, bool middle_light);
  
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
  
  //! --------------------- RAW commands for motor diagnostics
  //! @brief Sets the raw velocities of one motor
  //! @param motor number of the motor
  //! @param vel The raw velocity command
  //! @retval true Success
  //! @retval false Error
  virtual bool setMotorVelocity(int motor, int16_t vel);
  
  double getWidth() const;
  
  //! @brief Sets the norm width
  double setNormWidth(const double value);
  
  //! @brief gets the electronics position
  double getXElectronics() const;

  //! Manually sets the electronics position  
  bool setXElectronics(const double value);
  
  void setArmPanic() {
    state.arm_panic = true;
    for (int i = 0; i < N_HERCULEX; i++) {
      setHerculexTorque(i, 1);
    }
  }
  
  inline void setReverseRight(bool new_val) {reverse_right = new_val;}
  
  protected:
  // Serial Communciations stuff
  SiarSerial siar_serial_1; // 1 board, six motors 
  SiarSerial battery_serial; // Interface for acquiring battery data
  SerialInterface joy_serial; 
  ros::Time last_cmd;
  unsigned char command[___MAX_BUFFER_SIAR___], buffer[___MAX_BUFFER_SIAR___];
  
  // Options
  bool reverse_right;
  
  // Status flags
  bool first_odometry;
  bool controlled, has_joystick;
  ros::Publisher state_pub;
  ros::Subscriber slow_motion_sub, reverse_sub, op_sub;
  int bat_cont, bat_skip;
  int arm_cont, arm_skip;
  
  // Linear interpolators data
  std::string lin_mot_vec_file, width_file, elec_x_file;
  functions::LinearInterpolator *width_inter, *x_inter, *width_to_lin_pos, *x_elec_to_lin_pos;
  
  u_int16_t linear_velocity;
  
  //! @brief Calculates the increments of the position and orientation variables of the robot by integrating odometry measures
  //! @retval true Got ticks from the encoders and performed the calculations
  //! @retval false Some error while retrieving the ticks
  bool calculateOdometry();
  
  bool getJoystickActuate();
  
  bool sendHardStop();
  
  bool actualizeBatteryStatus();
  void slowReceived(const std_msgs::Bool& cmd_vel);
  void reverseReceived(const std_msgs::Bool& reverse);
  void opReceived(const std_msgs::Int8& mode);
  
  inline bool checkSum(unsigned char *buffer, int tam) {
    u_int16_t checksum = from_two_bytes(buffer[tam - 2], buffer[tam - 1]);
    
    u_int16_t bytes_sum = 0;
    for (int i = 0; i < tam - 2;i++) 
    {
      bytes_sum += (u_int16_t) buffer[i];
    }
    
//     ROS_ERROR("Checksum = %u           Bytes sum = %u", checksum, bytes_sum);
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
first_odometry(true), controlled(false), has_joystick(true), bat_cont(0), bat_skip(10), arm_cont(0), arm_skip(23), width_inter(NULL), x_inter(NULL),
width_to_lin_pos(NULL), x_elec_to_lin_pos(NULL), reverse_right(false)
{
  state.is_stopped = true;
  state.slow = false;
  state.reverse = false;
  state.width = 0.51;
  state.electronics_x = 0.14;
  
  // Initialize inherited fields
  _config = config;
  imu_ = NULL;
  first_odometry = true;
  
  #ifdef _SIAR_MANAGER_DEBUG_
  _printTime();
  std::cout << "Connecting to the Motor Board"<<std::endl;
  #endif
  
  while (!siar_serial_1.open() && ros::ok()) {
    
#ifdef _SIAR_MANAGER_DEBUG_
    std::cerr << "Could not connect to siar. Retrying\n";
#endif
    sleep(1);
  }
  if (!ros::ok) {
    std::cerr << "Could not connect to the motor board. Aborting.\n";
    throw SiarManagerException(siar_serial_1.getLastError());
  }
  _printTime();
  std::cout << "Connected to motor board.\n";
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
  while (!battery_serial.open() && ros::ok()) {
    
#ifdef _SIAR_MANAGER_DEBUG_
    std::cerr << "Could not connect to the battery monitoring board. Retrying. \n";
#endif
    sleep(1);
  } 
  
  if (!ros::ok()) {
    std::cerr << "Could not connect to the battery board. Aborting.\n";
    throw SiarManagerException(battery_serial.getLastError());
  }
  _printTime();
  std::cout << "Connected to the battery board.\n";
  
  ros::NodeHandle pn("~");
  state_pub = pn.advertise<siar_driver::SiarStatus>("/siar_status", 5);
  slow_motion_sub = pn.subscribe("/slow_motion", 1, &SiarManagerWidthAdjustment::slowReceived, this);
  reverse_sub = pn.subscribe("/reverse", 1, &SiarManagerWidthAdjustment::reverseReceived, this);
  op_sub = pn.subscribe("/operation_mode", 1, &SiarManagerWidthAdjustment::opReceived, this);
  
  if (!pn.getParam("lin_mot_file", lin_mot_vec_file)) {
    lin_mot_vec_file = "lin_mot";
  }
  if (!pn.getParam("width_file", width_file)) {
    width_file = "width_file";
  }
  if (!pn.getParam("elec_x_file", elec_x_file)) {
    elec_x_file = "elec_x_file";
  }
  int lv;
  if (!pn.getParam("linear_velocity", lv)) {
    linear_velocity = 190;
  } else {
    linear_velocity = lv;
  }
  
  width_inter = new functions::LinearInterpolator(lin_mot_vec_file, width_file);
  x_inter = new functions::LinearInterpolator(lin_mot_vec_file, elec_x_file);
  width_to_lin_pos = new functions::LinearInterpolator(width_file, lin_mot_vec_file); // TODO: not invertible!!
  x_elec_to_lin_pos = new functions::LinearInterpolator(elec_x_file, lin_mot_vec_file);
  
  // Stop robot
  setVelocity(0.0f, 0.0f);
  resetOdometry();
  
  // Set the linear position to 190 as the default
  setLinearVelocity(linear_velocity);
  
  // Last cmd
  last_cmd = ros::Time::now() - ros::Duration(CMD_TIME_OUT);
  
  // Get FW version
  getFWVersions();
  
  for (int i = 0; i < 5; i++) {
    setHerculexClearStatus(i);
  }
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

void SiarManagerWidthAdjustment::opReceived(const std_msgs::Int8& mode)
{
  state.operation_mode = mode.data;
}


inline SiarManagerWidthAdjustment::~SiarManagerWidthAdjustment() {
  if (siar_serial_1.isOpen() ) {
    // If the connection was open --> stop Siar and... 
    setVelocity(0.0f, 0.0f);
  }
  delete width_inter, width_to_lin_pos, x_elec_to_lin_pos, x_inter;
}

// TODO: check it!
inline bool SiarManagerWidthAdjustment::getIMD(double& imdl, double& imdr)
{
  command[0] = _config.get_enc;
  
  // Ask for odometry in both ports
  siar_serial_1.flush();
  bool ret_val = siar_serial_1.write(command, 1);
  
  int16_t m_left, m_right;
  
  if (ret_val && siar_serial_1.getResponse(buffer, 16)) {
    if (checkSum(buffer, 16) && buffer[0] ==_config.get_enc) {
//       std::cout << "Message Content: ";
//       for (int i = 0; i < 16; i++) {
//      
//      std::cout << (unsigned int)buffer[i] << " ";
//      
//      
//       }
//       std::cout << "\n";
      
      if (!first_odometry) {
        m_left = from_two_bytes_signed(buffer[5], buffer[6]);
        m_right = -from_two_bytes_signed(buffer[7], buffer[8]);
//         std::cout << "Middle left: " << m_left << "\t right: " << m_right << std::endl;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
  
#ifdef _SIAR_MANAGER_DEBUG_
  if  (m_right != 0 || m_left != 0 && !first_odometry) 
  {
//     std::cout << "Middle right = " << m_right << "\t left = " << m_left << std::endl;
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
  
  // New: actualize and publish different status TODO: How to handle errors here? Currently ignoring them
  bat_cont++;
  if (bat_cont >= bat_skip)
  {
//     ROS_INFO("Actualizing battery and arm");
    arm_cont++;
    // Actualize battery and power supply data
    if (!actualizeBatteryStatus()) {
//       ROS_ERROR("Could not retrieve the battery status");
    }
    if (!getPowerSupply()) {
//       ROS_ERROR("Could not retrieve the power supply status");
    }
    bat_cont = 0;
  }
    
  // Update arm status
  arm_cont++;
  if (arm_cont >=arm_skip) {
//     ROS_INFO("Actualizing arm");
    if (!getHerculexPosition()) {
//       ROS_ERROR("Could not retrieve the herculex position");
    }
  
//     if (!getHerculexStatus()) {
//       ROS_ERROR("Could not retrieve the herculex status");
//     }
    if (!getHerculexTemperature()) {
//       ROS_ERROR("Could not retrieve the herculex temperature");
    }
    if (!getHerculexTorque()) {
//       ROS_ERROR("Could not retrieve the herculex torque");
    }
    arm_cont = 0;
    
    // Update aux pins
//     if (!getAuxPinValues()) {
//       ROS_ERROR("Could not retrieve the aux pins values");
//     }
    
    // Linear motors
    if (getLinearMotorPosition(state.lin_motor_pos)) {
      state.electronics_x = getXElectronics();
      state.width = getWidth();
    } else {
//       ROS_ERROR("Could not retrieve the motor position");
    }
    if (!getLinearMotorPotentiometer(state.lin_motor_pot)) {
//       ROS_ERROR("Could not retrieve the motor potentiometer");
    }
    
    // 
    bool active;
    if (!getHardStopStatus(active)) {
//       ROS_ERROR("Could not retrieve the hard stop status");
    } else {
      state.hard_stop = active?1:0;
    }
//     if (!getHardStopTime(state.hard_stop_time)) {
//       ROS_ERROR("Could not retrieve the hard stop time");
//     }
  }
  
  // Check arm integrity
  if (!ArmFirewall::checkTemperatureAndStatus(state.herculex_temperature, state.herculex_status)) {
    // Change to panic mode and brake the joints
    if (!state.arm_panic) {
      ROS_ERROR("Siar manager::update --> Arm failed temperature and status check --> panic");
    }
    setArmPanic();
  }
  
  // Publish state
  state_pub.publish(state);
  
  return ret_val;
}


inline bool SiarManagerWidthAdjustment::setRawVelocity(int16_t left, int16_t right) {
  bool ret_val = true;
  
  if(state.slow) {
    left /= 3;
    right /= 3;
  }
  
  if (reverse_right)
    right *= -1; // The right motors are reversed
  
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
  siar_serial_1.flush();
  ret_val = siar_serial_1.write(command, 13);
  ret_val = siar_serial_1.getResponse(buffer, 4);
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setMotorVelocity(int motor, int16_t vel)
{
  bool ret_val = true;
  // get the right command
  command[0] = _config.set_vel;
  for (int i = 1; i < 12; i++) {
    command[i] = 0;
  }
  if (motor >= 0 && motor < 6) {
    command[motor*2 + 1] = (unsigned char)(vel >> 8);
    command[motor*2 + 2] = (unsigned char)(vel & 0xFF);
  }
  
  // Send it to front and rear board and wait for response
  siar_serial_1.flush();
  ret_val = siar_serial_1.write(command, 13);
  ret_val = siar_serial_1.getResponse(buffer, 4);
  
#ifdef _SIAR_MANAGER_DEBUG_
  if(!ret_val) {
    ROS_INFO("Issues when commanding speeds");
  }
#endif
  
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
  double right_speed = -(linear - state.width * angular * 0.75);
  double left_speed = (linear + state.width * angular * 0.75);
  
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
  if (fabs(linear) > 0.01 || fabs(angular) > 0.01) {
    std::cout << "SiarManagerWidthAdjustment::setVelocity --> v = " << linear << "\t w = " << angular << "\t ";
    std::cout << "\t l_speed = " << left_speed << "\t r_speed = " << right_speed << "\t ";
    std::cout << "\t l_int = " << v_left << "\t r_int = " << v_right << std::endl;
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
    
//      double d_theta_ticks = (dr - dl) / _config.estimated_diag;              //rad
    double d_theta_ticks = (dr - dl) / state.width;          //rad
    
//     ROS_INFO("d_theta_ticks = %f", d_theta_ticks);
    
    if (first_odometry) {
      first_odometry = false;
      state.header.stamp = ros::Time::now();
    } else {
      // Actualize the linear and angular speeds
      ros::Time t = ros::Time::now();
      double d_t = (t - state.header.stamp).toSec();
      state.speed = dist / d_t;
      double angular_rate_ticks = d_theta_ticks / d_t;
      double yaw = tf::getYaw(state.odom.pose.pose.orientation);
      
//       ROS_INFO("D_t = %f\tspeed = %f\tyaw = %f", d_t, state.speed, yaw);
      
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
        
        Right_ref *= -1;
        Right_ref_out *= -1;

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
  siar_serial_1.flush();
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
  
  siar_serial_1.flush();
  
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
  int tam = 8;
  // Ask for batteries voltage
  ret_val = battery_serial.write(&_config.get_voltage, 1);
  if (ret_val && battery_serial.getResponse(buffer, tam)) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_voltage;
    if (ret_val) {
      state.motor_battery.voltage = (from_two_bytes(buffer[1], buffer[2]))/10.0;
      state.elec_battery.voltage = (from_two_bytes(buffer[1], buffer[2]))/10.0;
    }
  }
  
  // Ask for batteries current
  ret_val &= battery_serial.write(&_config.get_inst_curr, 1);
  if (ret_val && battery_serial.getResponse(buffer, tam)) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_inst_curr;
    if (ret_val) {
      state.elec_battery.current = from_two_bytes(buffer[1], buffer[2])/1000.0;
      state.motor_battery.current = from_two_bytes(buffer[3], buffer[4])/1000.0;
    }
  }
  
  // Ask for integrated current
  ret_val &= battery_serial.write(&_config.get_integ_curr, 1);
  if (ret_val && battery_serial.getResponse(buffer, tam)) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_integ_curr;
    if (ret_val) {
      state.elec_battery.integrated_current = from_two_bytes(buffer[1], buffer[2])/1000.0;
      state.motor_battery.integrated_current = from_two_bytes(buffer[3], buffer[4])/1000.0;
    }
  }
  
  // Ask for battery level and remaining time
  ret_val &= battery_serial.write(&_config.get_battery_level, 1);
  tam = 10;
  if (ret_val && battery_serial.getResponse(buffer, tam)) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_battery_level;
    if (ret_val) {
      state.elec_battery.percentage = buffer[1];
      state.elec_battery.remaining_time = from_two_bytes(buffer[2], buffer[3]);
      state.motor_battery.percentage = buffer[4];
      state.motor_battery.remaining_time = from_two_bytes(buffer[5], buffer[6]);
    }
  }
  
  return ret_val;
}

// ----------- New width related functions

bool SiarManagerWidthAdjustment::setLinearPosition(u_int16_t value)
{
  bool ret_val = true;
  int tam = 4;
  
  // Reduce the input to the correct range
  double max = width_inter->lower_bound(10000)->first;
//  if (value > max) {
//     ROS_INFO("SiarManager::setLinearPosition --> Wrong command with value %u received. Bounded to %f", value, max);
//     value = max;
    if (value >120) // TODO: erase it
      value = 120;
//  }
  
  command[0] = _config.set_lin_pos;
  command[1] = value >> 8;
  command[2] = value & 0xFF;
  
  ret_val = siar_serial_1.write(command, 3);
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  // Checksum
  if (ret_val) {
    ret_val = buffer[0] == _config.set_lin_pos && checkSum(buffer, tam);
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setLinearVelocity(u_int16_t value)
{
  bool ret_val = true;
  int tam = 4;
  command[0] = _config.set_lin_vel;
  command[1] = value >> 8;
  command[2] = value & 0xFF;
  siar_serial_1.flush();
  
  ret_val = siar_serial_1.write(command, 3);
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  // Checksum
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.set_lin_vel;
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::setFan(bool on)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_fan;
  command[1] = on?1:0;
  
  siar_serial_1.flush();
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
  siar_serial_1.flush();
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
  siar_serial_1.flush();
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_lin_pos;
    pos = from_two_bytes(buffer[1], buffer[2]);
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getLinearMotorPotentiometer(u_int16_t& pot)
{
  bool ret_val = true;
  
  command[0] = _config.get_lin_pot;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 6;
  siar_serial_1.flush();
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_lin_pot;
    if (ret_val) {
      pot = from_two_bytes(buffer[1], buffer[2]);
    }
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getHardStopStatus(bool &active)
{
  bool ret_val = true;
  
  command[0] = _config.get_hard_stop;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 5;
  siar_serial_1.flush();
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_hard_stop;
    if (ret_val)
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
  siar_serial_1.flush();
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_hard_time;
    if (ret_val) 
      hard_time = buffer[1];
    
    ROS_INFO("Get Hard stop time --> %u", buffer[0]);
  }
  
  return ret_val;
}

bool SiarManagerWidthAdjustment::getFanControl(bool& active)
{
  bool ret_val = true;
  
  command[0] = _config.get_fan;
  
  ret_val = siar_serial_1.write(command, 1);
  const int tam = 5;
  siar_serial_1.flush();
  ret_val = siar_serial_1.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_fan;
    if (ret_val) 
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
  siar_serial_1.flush();
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
  battery_serial.flush();
  ret_val = battery_serial.getResponse(buffer, tam);
  
  state.battery_fw_version.resize(tam - 4);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_fw;
    for (int i = 0; i < tam - 4 && ret_val;i++) {
      state.battery_fw_version[i] = buffer[i + 1];
    }
  }
  
  return ret_val;
}

//-------------------------- Herculex and battery monitor commm ---------
bool SiarManagerWidthAdjustment::setLights(bool front_light, bool rear_light, bool middle_light)
{
  bool ret_val = true;
  const int tam = 4;
  command[0] = _config.set_lights;
  command[1] = (front_light?1:0) + (rear_light?2:0) + (middle_light?8:0);
  
  state.front_light = front_light?1:0;
  state.rear_light = rear_light?1:0;
  state.middle_light = middle_light?1:0;
  
  ret_val = battery_serial.write(command, 2);
  battery_serial.flush();
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
  battery_serial.flush();
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
  battery_serial.flush();
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
  command[1] = id;
  command[2] = value;
  
  ROS_INFO("Setting herculex torque. Id = %u. Value = %u", id, value);
 
  battery_serial.flush();
  ret_val = battery_serial.write(command, 3);
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
  battery_serial.flush();
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
  battery_serial.flush();
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
  battery_serial.flush();
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_status;
    
    for (int i = 0; i < N_HERCULEX && ret_val; i++) {
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
  battery_serial.flush();
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_torque;
    
    for (int i = 0; i < N_HERCULEX && ret_val; i++) {
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
  usleep(10000);
  battery_serial.flush();
  usleep(50000);
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_position;
    
    for (int i = 0; i < N_HERCULEX && ret_val; i++) {
      state.herculex_position[i] = from_two_bytes(buffer[i * 2 + 1] , buffer[i * 2 + 2]);
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
  battery_serial.flush();
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_herculex_temp;
    
    for (int i = 0; i < N_HERCULEX && ret_val; i++) {
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
  battery_serial.flush();
  ret_val = battery_serial.getResponse(buffer, tam);
  
  ret_val &= checkSum(buffer, tam) && buffer[0] == _config.get_power_supply;
  
  if (ret_val) {
    state.power_supply.elec_v =  from_two_bytes(buffer[1], buffer[2]);
    state.power_supply.elec_i = from_two_bytes(buffer[3], buffer[4]);;
    state.power_supply.motor_v = from_two_bytes(buffer[5], buffer[6]);
    state.power_supply.motor_i = from_two_bytes(buffer[7], buffer[8]);
    state.power_supply.cable_v = from_two_bytes(buffer[9], buffer[10]);;
  }
  
  return ret_val;
}


bool SiarManagerWidthAdjustment::getAuxPinValues()
{
  bool ret_val = true;
  const int tam = 5;
  command[0] = _config.get_aux_pin_values;
  
  ret_val = battery_serial.write(command, 1);
  battery_serial.flush();
  ret_val = battery_serial.getResponse(buffer, tam);
  
  if (ret_val) {
    ret_val = checkSum(buffer, tam) && buffer[0] == _config.get_aux_pin_values;
    state.aux_pins_values = buffer[1];
  }
  
  return ret_val;
}

// TODO: Refine it empirically the next get... functions (or with the model)
double SiarManagerWidthAdjustment::getWidth() const
{
  return width_inter->interpolate(state.lin_motor_pos);
}

// Gets the x coordinate of the center of the electronics with respect of the base link
double SiarManagerWidthAdjustment::getXElectronics() const
{
  return x_inter->interpolate(state.lin_motor_pos);
}

bool SiarManagerWidthAdjustment::setXElectronics(const double value)
{
  uint16_t val = x_elec_to_lin_pos->interpolate(value);
  return setLinearPosition(val);
}

double SiarManagerWidthAdjustment::setNormWidth(const double value)
{
  std::map<double, double>::const_iterator it = x_elec_to_lin_pos->lower_bound(10000);
  double up_bound = (*x_inter)[0];
  it = x_elec_to_lin_pos -> upper_bound(-10000);
  double eff_val = value*up_bound;
  double low_bound = fabs(it->first);

  if (value < 0.0)
          eff_val = value*low_bound;
  
  uint16_t val = x_elec_to_lin_pos->interpolate(eff_val);
  ROS_INFO("Linear Pos: %u\tUp=%f\tlow=%fValue=%f", val,up_bound, low_bound, value);
  return setLinearPosition(val);
}


//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
