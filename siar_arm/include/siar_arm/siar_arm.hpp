#ifndef _SIAR_ARM_HPP_
#define _SIAR_ARM_HPP_


#include <math.h>
#include "functions/linear_interpolator.hpp"
#include <string>
#include <queue>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
#include <boost/array.hpp>

#include "functions/functions.h"
#include <ros/ros.h>


#define ARM_JOINTS 5

using angle_type = boost::array<double, ARM_JOINTS>;
using raw_type = boost::array<int16_t, ARM_JOINTS>;  

class SiarArm {
		
	public:
		// Linear interpolators data
		std::vector<functions::LinearInterpolator *> pos_mot_interpol_, mot_pos_interpol_;
		std::vector<double> length{ 0.035, 0.186, 0.140,  0.0651, 0.04297};
		size_t n_motors{ length.size() };

		SiarArm() {}

		SiarArm(const std::string &mot_arm_file, const std::string &pos_arm_file) {
			load_data(mot_arm_file, pos_arm_file);
		}


		~SiarArm() {
			for(size_t i = 0; i < pos_mot_interpol_.size(); i++) {
				delete pos_mot_interpol_[i];
			}
			for(size_t i = 0; i < mot_pos_interpol_.size(); i++) {
				delete mot_pos_interpol_[i];
			}
		}

		void load_data(const std::string &mot_arm_file, const std::string &pos_arm_file) ;       
		bool inverseKinematics(double x, double y, double z, angle_type &result, bool erase_remaining);

		bool rad2motor(const angle_type &angles, raw_type &commands);

		bool motor2rad(const raw_type &commands, angle_type &angles);


		void forwardKinematics(const raw_type &joint_values, double &x, double &y, double &z);
		void forwardKinematics(const angle_type &angles, double &x, double &y, double &z) ;

		std::vector<raw_type> straightInterpol(double x, double y, double z, uint8_t n_points, const raw_type &curr_pos);

		bool checkJointLimits( const angle_type &angles);

		bool checkJointLimits(const raw_type &joint_values);

		void correctJointLimits(raw_type &joint_values);

		bool checkTemperatureAndStatus(const boost::array<uint8_t,5> &herculex_temperature, const boost::array<uint8_t,5> &herculex_status) ;

  
  
};

#endif /* _SIAR_ARM_HPP_ */

