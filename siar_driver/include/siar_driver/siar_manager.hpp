/***********************************************************************/
/**                                                                    */
/** raposa_manager.h                                                   */
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


// TODO: All conversion giraff--> raposa

#ifndef _SIAR_MANAGER_H_
#define _SIAR_MANAGER_H_

#ifndef PI
#define   PI 3.14159265
#endif

// Activate this to show some debug information
// #define _SIAR_MANAGER_DEBUG_

#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <exception>
#include "siar_serial.hpp"
#include "siar_config.hpp"
#include "arduimu_v3/imu.hpp"
#include <nav_msgs/Odometry.h>	

//-- HEADERS ----------------------------------------------------------------------------

//! @class SiarManagerException
//! @brief An exception to throw in case of fatal error
class SiarManagerException:public std::exception
{
  public:
  SiarManagerException(const std::string& message) : message(message) {}
  ~SiarManagerException() throw () {}
  virtual const char* what() const throw() {return message.c_str();}
	  
  private:
  std::string message;
};

//! \struct SiarState Gather all relevant state variables of the Siar
struct SiarState
{
  bool is_stopped;
  double linear_velocity, x, y;
  nav_msgs::Odometry odom;
  
  int64_t front_left_ticks, front_right_ticks; // Sumatory of ticks of front wheels
  int64_t middle_left_ticks, middle_right_ticks; // Sumatory of ticks of middle wheels
  int64_t back_left_ticks, back_right_ticks; // Sumatory of ticks of back wheels
  
  timespec last_update;
};

//! @class SiarManager                                                          */
//! @brief This manages the Siar controller. Includes velocity and arm commands

class SiarManager
{
  public:
  //! @brief Gets the state of raposa (cmd_vel, measured arm_pos, odometry position...)
  //! Comment: This function should be called in a loop
  virtual bool update() = 0;
  
  //! @brief Sets the linear and angular velocity of the raposa
  //! @param linear (IN) -- linear velocity in m/s
  //! @param angular (IN) -- angular velocity in rad/s
  //! @retval true Could establish the velocity
  //! @retval false Errors found while estabilishing the velocity
  virtual bool setVelocity(double linear, double angular) = 0;

  //! @brief Gets the incremental move distance (IMDL,IMDR) from enconder increments
  //! @param IMDL (OUT) The IMD for left wheel in meters
  //! @param IMDR (OUT) The IMD for right wheel in meters
  virtual bool getIMD(double& imdl, double& imdr) = 0;

  //! @brief Determines if the robot is stopped
  //! @retval true The robot is stopped
  //! @retval false The robot is moving	
  bool isStopped() {return state.is_stopped;}
  
  inline double resetOdometry() 
  {
    // Initialize positions
    state.odom.pose.pose.position.x = state.odom.pose.pose.position.y = state.odom.pose.pose.position.z = 0.0;
    state.front_left_ticks = state.front_right_ticks = 0;
    state.middle_left_ticks = state.middle_right_ticks = 0;
    state.back_left_ticks = state.back_right_ticks = 0;
    
    // Initialize orientations
    state.odom.pose.pose.orientation.x = state.odom.pose.pose.orientation.y = state.odom.pose.pose.orientation.z = 0.0;
    state.odom.pose.pose.orientation.w = 1.0;
    
    // Discard the first measure and raise the flag
    first_odometry = true;
    update();
  }
  

  //! @brief accessor to the state of Siar
  const SiarState &getState() const {return state;}

  //! @brief Sets the IMU with a determinate updating rate
  //! @retval true The IMU was opened successfully
  //! @retval false Some errors while finding the IMU
  bool setIMU(double freq);
  
  //! --------------------- RAW commands for calibration purposes
  //! @brief Sets the raw velocities of each motor
  //! @param left Left velocity
  //! @param right Right velocity
  //! @retval true Success
  //! @retval false Error
  virtual bool setRawVelocity(int16_t left, int16_t right) = 0;
  
  protected:
  SiarConfig _config;
  SiarState state;
  
  bool first_odometry;
  
  IMU *imu_;
  
  //! @brief Calculates the increments of the position and orientation variables of the robot by integrating odometry measures
  //! @retval true Got ticks from the encoders and performed the calculations
  //! @retval false Some error while retrieving the ticks
  virtual bool calculateOdometry() = 0;
};

inline bool SiarManager::setIMU(double freq)
{
  if (imu_) {
    delete imu_;
  }
  imu_ = new IMU(freq);
  
  ros::Rate r(2.0);
  while (!imu_->isInit() && ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  if (ros::ok()) {
    ROS_INFO("IMU initialized ok");
  }
  
  return true;
}



#endif
