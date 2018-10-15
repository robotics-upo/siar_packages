#ifndef _SIAR_ARM_HPP_
#define _SIAR_ARM_HPP_


#include "math.h"
#include <functions/linear_interpolator.hpp>
#include <string>
#include <queue>
#include <vector>
#include <boost/concept_check.hpp>

#define L1 0.0475
#define L2 0.215
#define L3 0.155
#define L4 0.080
#include "siar_functions.hpp"


class SiarArm{

  public:
    
    // Linear interpolators data
  int n_motors;
  std::vector<functions::LinearInterpolator *> pos_mot_interpol_, mot_pos_interpol_;
  std::vector<double> length;
  
  SiarArm(const std::string &mot_arm_file, const std::string &pos_arm_file) {
    load_data(mot_arm_file, pos_arm_file);
  }
  
  bool load_data(const std::string &mot_arm_file, const std::string &pos_arm_file) {
    n_motors = 5;
    for (int i = 0; i < n_motors; i++) {
      std::ostringstream mot_arm, pos_arm;
      mot_arm << mot_arm_file << i;
      pos_arm << pos_arm_file << i;
      pos_mot_interpol_.push_back(new functions::LinearInterpolator(mot_arm.str(), pos_arm.str()));
    }
    length.push_back(0.035); // TODO: read it from file?
    length.push_back(0.186);
    length.push_back(0.140);
    length.push_back(0.0651);
    length.push_back(0.04297);
    
  }
  
  ~SiarArm() {
    for(int i = 0; i < pos_mot_interpol_; i++) {
      delete pos_mot_interpol_[i];
    }
    for(int i = 0; i < mot_pos_interpol_; i++) {
      delete mot_pos_interpol_[i];
    }
  }
    
  bool inverseKinematics(double x, double y, double z, std::vector <double> &result)
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
    double d = sqrt(length[1]*length[1] + length[2]*length[2] - 2.0 * length[1] * length[2] * cos(result[2]);
    result[1] = acos((y - length[0])/d;
    //ROS_INFO("x: %f; y: %f;z: %f",x,y,z);
    //ROS_INFO("q1: %f, q2: %f, q3: %f, q4: %f, q5:%f",q1,q2,q3,q4,q5);
  }
  
  void rad2motor(const std::vector<double> &angles, std::vector<int> &commands) {
    commands.resize(n_motors);
    for (int i = 0; i < n_motors; i++) {
      functions::LinearInterpolator &interpol = *pos_mot_interpol_[i];
      commands[i] = interpol(angles[i]);
    }
  }

  void motor2rad(const std::vector<int> &commands, std::vector<double> &angles) {
    angles.resize(n_motors);
    for (int i = 0; i < n_motors; i++) {
      functions::LinearInterpolator &interpol = *mot_pos_interpol_[i];
      angles[i] = interpol(commands[i]);
    }
  }
  
  void forwardKinematics(const std::vector  <int> &joint_values, double &x, double &y, double &z)
  {
    
//       X = cos(a1) *(L2*sin(a2)+L3*sin(a2+a3))
// Y = L1 + (L2*cos(a2)+L3*cos(a2+a3))
// Z = -1 * sin(a1) *(L2*sin(a2)+L3*sin(a2+a3))
// Donde Y es positiva desde la base en dirección a la primera articulación, X es ortogonal  a Y en el plano proyectado por el dibujo y Z va desde el plano hacia dentro.

    std::vector<double> angles;
    motor2rad(joint_values, angles);
    double a1 = joint_values[0];
    double a2 = joint_values[1];
    double a3 = joint_values[2];
    double L1 = length[0];
    double L2 = length[1];
    double L3 = length[2];
    x = cos(a1) * (L2*sin(a2) + L3*sin(a2 + a3));
    y = L1 + L2*cos(a2) + L3*cos(a2 + a3);
    z = -sin(a1) * (L2*sin(a2) + L3*sin(a2 + a3));
  }

  std::vector<std::vector<int> > straightInterpol(double x, double y, double z, uint8_t n_points, const std::vector<int> &curr_pos)
  {
    doble a_x, a_y, a_z;
    forwardKinematics(curr_pos, a_x, a_y, a_z);
    doble i_x, i_y, i_z;  
    i_x = (x - a_x)/(n_points+1);	  
    i_y = (y - a_y)/(n_points+1);
    i_z = (z - a_z)/(n_points+1);
    
    std::vector<std::vector<int> > ret;
    
    std::vector<double> angles;
    std::vector<int> commands;
    for( int i = 0; i < n_points+1; i++)	
    {	
      a_x += i_x;
      a_y += i_y;
      a_z += i_z;
      		      
      inverseKinematics(a_x, a_y, a_z, angles);
      rad2motor(angles, commands);
      ret.push_back(commands);
    }

  }
  
  bool checkJointLimits(const boost::array<int16_t, 5> joint_values)
  {
    bool ret_val = true;
    
    for (int i = 0; i < n_motors && ret_val; i++) {
      double max, min;
      functions::LinearInterpolator &curr_inter = *mot_pos_interpol_[i];
      min = curr_inter.upper_bound(0);
      max = curr_inter.lower_bound(2000); // Usually the commands are in the [0, 2000] range
       
      ret_val &= joint_values[i] > min && joint_values[i] < max;
      
    }
    
    return ret_val;
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
