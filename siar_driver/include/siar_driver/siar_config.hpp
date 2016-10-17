/***********************************************************************/
/**                                                                    */
/** siar_config.h                                                      */
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

#ifndef __SIAR_CONFIG_H__
#define __SIAR_CONFIG_H__

#include "siar_functions.hpp"
#include <stdint.h>

class SiarConfig {
public:
  // Robot constraints
  Saturate<> velocity_sat; // Saturation in velocity (m/s)
  Saturate<> angular_sat; // Saturation in angular velocity (rad/s)
  DeadZone<> velocity_dead; // Dead zone of the velocity (m/s)
  Saturate<int> vel_int_sat; // Two integer saturators that will help to make the conversion
  DeadZone<> encoders_dead;
  
  // Relationships
  
  double half_axis_distance, radius, meters_tick, peri_wheel, ticks_revolution, diag_wheels; // In meters
  
  // Empirical relationships
  double meters_tick_l, meters_tick_r; // Odometry calibration
  double meters_tick_l_b, meters_tick_r_b; // Odometry calibration
  double v_m_s_to_v_raw, v_m_s_to_v_raw_b; // Linear speed to raw commands
//   double ang_vel_to_raw_l, ang_vel_to_raw_r; // Relates angular speed to raw commands
  double estimated_diag;
  
  // Robot commands
  unsigned char set_vel, get_enc, hard_stop;
  
  // Battery monitor commands
  unsigned char get_voltage, get_inst_curr, get_integ_curr, get_battery_level;
  
  SiarConfig(const std::string &filename = "");
  
  void setDefaultConfig();
  
  bool saveConfig(const std::string &filename);
};

SiarConfig::SiarConfig(const std::string &filename)
{
  if (filename == "") {
    setDefaultConfig();
  } // TODO: load parameters from file (yaml)
}

// TODO: implement
bool SiarConfig::saveConfig(const std::string& filename)
{
  return true;
}

void SiarConfig::setDefaultConfig()
{
  half_axis_distance = 0.2;
  diag_wheels = 0.4;
  radius = 0.085; // 0.085 is the distance from the robot center to the ground
  ticks_revolution = 32250; // See the document API_summary_SiarNG_motors_2015.pdf
  peri_wheel = 2 * M_PI * radius; // Perimeter of the wheel
  meters_tick = peri_wheel / ticks_revolution; // meters / tick radius
  meters_tick_l = meters_tick;
  meters_tick_l_b = meters_tick_l;
  meters_tick_r = meters_tick;
  meters_tick_r_b = meters_tick_r;
  estimated_diag = diag_wheels;
  
  // TODO: From calibration
  meters_tick_l = 1 / 250293.0;
  meters_tick_r = 1 / 250395.0;
  meters_tick_l_b = 1 / 254055.0;
  meters_tick_r_b = 1 / 254140.0;
  v_m_s_to_v_raw = 1671;
  v_m_s_to_v_raw_b = 1671;
//   estimated_diag = 0.55;
  
  // Debug
  std::cerr << "DEBUG: Meters / ticks L = " << meters_tick_l << std::endl;
  std::cerr << "DEBUG: Meters / ticks R = " << meters_tick_r << std::endl;
  std::cerr << "DEBUG: Meters / ticks BL = " << meters_tick_l_b << std::endl;
  std::cerr << "DEBUG: Meters / ticks BR = " << meters_tick_r_b << std::endl;
  
  velocity_dead = DeadZone<>(0.01); // TODO: identify
  encoders_dead = DeadZone<>(0.0003);
  
  velocity_sat = Saturate<>(1.12); // Maximum velocity of the SIAR (TODO)
  angular_sat = Saturate<>(0.3);
  vel_int_sat = Saturate<int>(1100);
  
  set_vel = (unsigned char)0x56; // See Raposa manual (SIAR commands have equal command chars)
  get_enc = (unsigned char)0x4A;
  hard_stop = (unsigned char)0x57; 
  get_voltage = (unsigned char)0x52;
  get_inst_curr = (unsigned char)0x53;
  get_integ_curr = (unsigned char)0x54;
  get_battery_level = (unsigned char)0x55;
}

#endif