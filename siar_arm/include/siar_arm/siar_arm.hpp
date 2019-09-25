#ifndef _SIAR_ARM_HPP_
#define _SIAR_ARM_HPP_


#include "math.h"
#include <functions/linear_interpolator.hpp>
#include <string>
#include <queue>
#include <vector>
#include <iostream>

#define ARM_JOINTS 5

typedef boost::array<double, ARM_JOINTS> angle_type;
typedef boost::array<int16_t, ARM_JOINTS> raw_type;

class SiarArm {
  public:
    // Linear interpolators data
  int n_motors;
  std::vector<functions::LinearInterpolator *> pos_mot_interpol_, mot_pos_interpol_;
  std::vector<double> length;
  
  SiarArm() {}
  
  SiarArm(const std::string &mot_arm_file, const std::string &pos_arm_file) {
    load_data(mot_arm_file, pos_arm_file);
  }
  
  bool load_data(const std::string &mot_arm_file, const std::string &pos_arm_file) {
    n_motors = 5;
    
    for (int i = 0; i < n_motors; i++) {
      std::ostringstream mot_arm, pos_arm;
      mot_arm << mot_arm_file << i;
      pos_arm << pos_arm_file << i;
      mot_pos_interpol_.push_back(new functions::LinearInterpolator(mot_arm.str(), pos_arm.str()));
      pos_mot_interpol_.push_back(new functions::LinearInterpolator(pos_arm.str(), mot_arm.str()));
    }
    length.push_back(0.035);
    length.push_back(0.186);
    length.push_back(0.140);
    length.push_back(0.0651);
    length.push_back(0.04297);
  }
  
  ~SiarArm() {
    for(int i = 0; i < pos_mot_interpol_.size(); i++) {
      delete pos_mot_interpol_[i];
    }
    for(int i = 0; i < mot_pos_interpol_.size(); i++) {
      delete mot_pos_interpol_[i];
    }
  }
    
  bool inverseKinematics(double x, double y, double z, angle_type &result, bool erase_remaining = true)
  {
    bool coordenadas_correctas = true;

    // a1 = atan2(Z,X)

// a3 = acos(Z^2+X^2+(Y-L1)^2-L2^2-L3^2)

// d = sqrt(L2^2 + L3^2 -2*L2*L3*cos(a3))

// a2 = acos((y-L1)/d)

// donde L1 = 35.06; L2 = 186; L3 =140.05

// a1,2,3 son los ángulos de la primera, segunda y tercera articulación.
    result[0] = atan2(z, x);
    result[2] = acos(z*z+ x*x + pow(y-length[0],2.0) - length[2]*length[2] - length[3]*length[3]);
    double d = sqrt(length[1]*length[1] + length[2]*length[2] - 2.0 * length[1] * length[2] * cos(result[2]));
    result[1] = acos((y - length[0])/d);
    
    if (erase_remaining) {
      result[3] = 0; // TODO: Check limits
      result[4] = 0; 
    }
    
    return checkJointLimits(result);
  }
  
  void rad2motor(const angle_type &angles, raw_type &commands) {
    for (int i = 0; i < n_motors; i++) {
      functions::LinearInterpolator &interpol = *pos_mot_interpol_[i];
      commands[i] = interpol.interpolate(angles[i]);
    }
  }

  void motor2rad(const raw_type &commands, angle_type &angles) {
    for (int i = 0; i < n_motors; i++) {
      functions::LinearInterpolator &interpol = *mot_pos_interpol_[i];
      angles[i] = interpol.interpolate(commands[i]);
    }
  }
  
  void forwardKinematics(const raw_type &joint_values, double &x, double &y, double &z)
  {
    
//       X = cos(a1) *(L2*sin(a2)+L3*sin(a2+a3))
// Y = L1 + (L2*cos(a2)+L3*cos(a2+a3))
// Z = -1 * sin(a1) *(L2*sin(a2)+L3*sin(a2+a3))
// Donde Y es positiva desde la base en dirección a la primera articulación, X es ortogonal  a Y en el plano proyectado por el dibujo y Z va desde el plano hacia dentro.

    angle_type angles;
    motor2rad(joint_values, angles);
    forwardKinematics(angles, x, y ,z);
  }
  void forwardKinematics(const angle_type &angles, double &x, double &y, double &z) {
    double a1 = angles[0];
    double a2 = angles[1];
    double a3 = angles[2];
    double L1 = length[0];
    double L2 = length[1];
    double L3 = length[2];
    x = cos(a1) * (L2*sin(a2) + L3*sin(a2 + a3));
    y = L1 + L2*cos(a2) + L3*cos(a2 + a3);
    z = -sin(a1) * (L2*sin(a2) + L3*sin(a2 + a3));
  }

  std::vector<raw_type> straightInterpol(double x, double y, double z, uint8_t n_points, const raw_type &curr_pos)
  {
    bool error = false;
    double a_x, a_y, a_z;
    forwardKinematics(curr_pos, a_x, a_y, a_z);
    double i_x, i_y, i_z;  
    i_x = (x - a_x)/(n_points+1);	  
    i_y = (y - a_y)/(n_points+1);
    i_z = (z - a_z)/(n_points+1);
    
    std::vector<raw_type > ret;
    
    angle_type angles;
    raw_type commands;
    for( int i = 0; i < n_points + 1 && !error; i++)	
    {	
      a_x += i_x;
      a_y += i_y;
      a_z += i_z;
      		      
      if (inverseKinematics(a_x, a_y, a_z, angles)) {
	rad2motor(angles, commands);
	ret.push_back(commands);
      } else {
	ret.clear();
	error = true;
      }
    }
    return ret;
  }
  
  bool checkJointLimits( const angle_type &angles) {
    raw_type commands;
    rad2motor(angles, commands);
    return checkJointLimits(commands);
  }
  
  bool checkJointLimits(const raw_type &joint_values)
  {
    bool ret_val = true;
    
    for (int i = 0; i < n_motors && ret_val; i++) {
      double max, min;
      functions::LinearInterpolator &curr_inter = *(mot_pos_interpol_[i]);
      min = curr_inter.upper_bound(0)->first;
      max = curr_inter.lower_bound(2000)->first; // Usually the commands are in the [0, 2000] range
       
      ret_val &= joint_values[i] > min && joint_values[i] < max;
      
    }
    
    return ret_val;
  }
  
  void correctJointLimits(raw_type &joint_values)
  {
    for (int i = 0; i < n_motors; i++) {
      int16_t max, min;
      functions::LinearInterpolator &curr_inter = *(mot_pos_interpol_[i]);
      min = curr_inter.upper_bound(0)->first;
      auto it = curr_inter.lower_bound(2000) ;
      it--;
      max = it->first; // Usually the commands are in the [0, 2000] range
      
      std::cout << "Trying to saturate between: " << min << " and " << max << std::endl;
      
       
      joint_values[i] = functions::saturate(joint_values[i], min, max); 
      
    }
  }

  bool checkTemperatureAndStatus(const boost::array<uint8_t,5> &herculex_temperature, const boost::array<uint8_t,5> &herculex_status) {
    bool ret_val = true;
    for(int i = 0; i < 5; i++)
    {
      if (herculex_temperature[i]<0 && herculex_temperature[i]>50)
      {
        ROS_ERROR("TEMPERATURE OF THE %d LINK IS OUT OF RANGE: %d", i, herculex_temperature[i]);
        ret_val = false;
      }
//       if (herculex_status[i]!=1)    // TODO: Check this!!
//       {
//         ROS_ERROR("%d LINK STATUS: %d", i, herculex_status[i]);
//         ret_val = false;
//       }
    }
    return ret_val;  
  }
  
  
  
};

#endif /* _SIAR_ARM_H_ */

