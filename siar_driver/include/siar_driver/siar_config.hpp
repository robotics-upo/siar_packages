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

#define N_HERCULEX 5      // Number of herculex motors

class SiarConfig {
public:
  // Robot constraints
  Saturate<> velocity_sat; // Saturation in velocity (m/s)
  Saturate<> angular_sat; // Saturation in angular velocity (rad/s)
  DeadZone<> velocity_dead; // Dead zone of the velocity (m/s)
  Saturate<int> vel_int_sat; // Two integer saturators that will help to make the conversion
  DeadZone<> encoders_dead;
  Saturate<int> arm_pos_sat[N_HERCULEX]; // Saturation for the arm low-level controller
  
  // Relationships
  
  double half_axis_distance, radius, meters_tick, peri_wheel, ticks_revolution, diag_wheels; // In meters
  int  max_width_pos;
  double max_x_electronics;
  
  // Empirical relationships
  double meters_tick_l, meters_tick_r; // Odometry calibration
  double meters_tick_l_b, meters_tick_r_b; // Odometry calibration
  double v_m_s_to_v_raw, v_m_s_to_v_raw_b; // Linear speed to raw commands
//   double ang_vel_to_raw_l, ang_vel_to_raw_r; // Relates angular speed to raw commands
  double estimated_diag;
  
  // Robot commands
  unsigned char set_vel, get_enc; // Velocity motor
  unsigned char hard_stop, get_hard_stop, get_hard_time, set_hard_time; // Hard stop stuff
  unsigned char set_lin_vel, set_lin_pos, get_lin_pos, get_lin_pot; // Linear motor
  unsigned char get_fan, set_fan, get_fw; // Fan & FW
  
  // Battery monitor commands
  unsigned char get_voltage, get_inst_curr, get_integ_curr, get_battery_level;
  unsigned int set_lights;
  unsigned int set_aux_pin_direction;
  unsigned int set_aux_pin_values;
  unsigned char set_herculex_torque;
  
  unsigned char set_herculex_position;
  unsigned char set_herculex_clear_status;
  unsigned char get_herculex_status;
  unsigned char get_herculex_torque;
  unsigned char get_herculex_position;
  unsigned char get_herculex_temp;
  unsigned char get_power_supply;
  unsigned char get_aux_pin_values;
  
  // Flags
  bool reverse_right;
  
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
  reverse_right = false;
  
  half_axis_distance = 0.2;
  diag_wheels = 0.4; // TODO: In the width adjusted the diag wheels will not be constant --> Get the maximum and minimum diagonals and interpolate!
  radius = 0.085; // 0.085 is the distance from the robot center to the ground
  ticks_revolution = 32250; // See the document API_summary_SiarNG_motors_2015.pdf
  peri_wheel = 2 * M_PI * radius; // Perimeter of the wheel
  meters_tick = peri_wheel / ticks_revolution; // meters / tick radius
  meters_tick_l = meters_tick;
  meters_tick_l_b = meters_tick_l;
  meters_tick_r = meters_tick;
  meters_tick_r_b = meters_tick_r;
  estimated_diag = diag_wheels;
  
  // TODO: From calibration --> calibrate new prototype
  meters_tick_l = 1 / 250293.0;
  meters_tick_r = 1 / 250395.0;
  meters_tick_l_b = 1 / 254055.0;
  meters_tick_r_b = 1 / 254140.0;
  v_m_s_to_v_raw = 1671;
  v_m_s_to_v_raw_b = 1671;
//   estimated_diag = 0.55;
  
  // Debug
//   std::cerr << "DEBUG: Meters / ticks L = " << meters_tick_l << std::endl;
//   std::cerr << "DEBUG: Meters / ticks R = " << meters_tick_r << std::endl;
//   std::cerr << "DEBUG: Meters / ticks BL = " << meters_tick_l_b << std::endl;
//   std::cerr << "DEBUG: Meters / ticks BR = " << meters_tick_r_b << std::endl;
  
  velocity_dead = DeadZone<>(0.01); // TODO: identify
  encoders_dead = DeadZone<>(0.0003);
  
  velocity_sat = Saturate<>(1.12); // Maximum velocity of the SIAR (TODO)
  angular_sat = Saturate<>(0.3);
  vel_int_sat = Saturate<int>(1100);
  
  // Motor board
  
  set_vel = (unsigned char)0x56; // Followed the C# code by Carlos Marques (Jul 2017)
  set_lin_vel = (unsigned char)0x31;
  set_lin_pos = (unsigned char)0x30;
  set_hard_time = (unsigned char)0x58;
  hard_stop = (unsigned char)0x57;
  set_fan = (unsigned char)0x53;
  
  get_enc = (unsigned char)0x4A;
  get_lin_pos = (unsigned char)0x32;
  get_lin_pot = (unsigned char)0x33;
  get_hard_stop = (unsigned char)0x59;
  get_hard_time = (unsigned char)0x58;
  get_fan = (unsigned char)0x54;
  
  // Common
  get_fw = (unsigned char)0x20;
  
  // Electronic board TODO: Update with the new version of the width adjustment
  
  set_herculex_torque = (unsigned char)0x30;
  set_herculex_position = (unsigned char)0x31;
  set_herculex_clear_status = (unsigned char)0x32;
  
  get_herculex_status = (unsigned char)0x40;
  get_herculex_torque = (unsigned char)0x41;
  get_herculex_position = (unsigned char)0x42;
  get_herculex_temp = (unsigned char)0x43;
  
  get_power_supply = (unsigned char)0x51;
  get_voltage = (unsigned char)0x52;
  get_inst_curr = (unsigned char)0x53;
  get_integ_curr = (unsigned char)0x54;
  get_battery_level = (unsigned char)0x55;
  get_aux_pin_values = (unsigned char)0x56;
  
  
  set_lights = (unsigned char)0x60;
  set_aux_pin_direction = (unsigned char)0x61;
  set_aux_pin_values = (unsigned char)0x62;
  
  
  // Arm lengths and other stuff TODO: necessary? (gonzalo's module)
//   arm_length[0] = 0.035;
//   arm_length[1] = 0.0475;
//   arm_length[2] = 0.215;
//   arm_length[3] = 0.155;
//   arm_length[4] = 0.08;
}

#endif