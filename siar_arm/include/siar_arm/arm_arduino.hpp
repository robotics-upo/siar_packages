#ifndef _ARM_ARDUINO_HPP_
#define _ARM_ARDUINO_HPP_


#include <math.h>
#include "functions/linear_interpolator.hpp"
#include <string>
#include <queue>
#include <vector>
#include <iostream>
#include <array>
#include "functions/functions.h"

#include <visualization_msgs/MarkerArray.h>

#define ARM_JOINTS 2

using angle_type = std::array<double, ARM_JOINTS>;
using raw_type = std::array<int16_t, ARM_JOINTS>;  

class ArmArduino {
		
	public:
		// Linear interpolators data
		std::vector<functions::LinearInterpolator *> pos_mot_interpol_, mot_pos_interpol_;
		std::array<double, 3> length{{0.11, 0.04}};
		const size_t n_motors = ARM_JOINTS;

		ArmArduino() {}

		ArmArduino(const std::string &mot_arm_file, const std::string &pos_arm_file) {
			load_data(mot_arm_file, pos_arm_file);
		}


		~ArmArduino() {
			for(size_t i = 0; i < pos_mot_interpol_.size(); i++) {
				delete pos_mot_interpol_[i];
			}
			for(size_t i = 0; i < mot_pos_interpol_.size(); i++) {
				delete mot_pos_interpol_[i];
			}
		}

		void load_data(const std::string &mot_arm_file, const std::string &pos_arm_file) ;       

		bool rad2motor(const angle_type &angles, raw_type &commands);
		bool rad2motor(const angle_type &angles, raw_type &commands, int &num_motor);


		bool motor2rad(const raw_type &commands, angle_type &angles);

		bool checkJointLimits( const angle_type &angles);
		bool checkJointLimits(const raw_type &joint_values);
		void correctJointLimits(raw_type &joint_values);  
};

void ArmArduino::load_data(const std::string &mot_arm_file, const std::string &pos_arm_file) {
	  
	for (int i = 0; i < n_motors; i++) {
		std::ostringstream mot_arm, pos_arm;
		mot_arm << mot_arm_file << i;
		pos_arm << pos_arm_file << i;
		mot_pos_interpol_.push_back(new functions::LinearInterpolator(mot_arm.str(), pos_arm.str()));
		pos_mot_interpol_.push_back(new functions::LinearInterpolator(pos_arm.str(), mot_arm.str()));
	}
}
    

bool ArmArduino::rad2motor(const angle_type &angles, raw_type &commands) {
	bool ret_val = true;
	for (int i = 0; i < n_motors; i++) {
		functions::LinearInterpolator &interpol = *pos_mot_interpol_[i];
		functions::LinearInterpolator &interpol_2 = *mot_pos_interpol_[i];
		commands[i] = interpol.interpolate(angles[i]);
		ret_val &= interpol.inRange(angles[i]);
	}

	return ret_val;
}


bool ArmArduino::rad2motor(const angle_type &angles, raw_type &commands, int &num_motor) {
	bool ret_val = true;
	
	functions::LinearInterpolator &interpol = *pos_mot_interpol_[num_motor];
	ret_val &= interpol.inRange(angles[num_motor]);
	// ROS_ERROR("value rad2motor: %i", ret_val);

	return ret_val;
	
}


bool ArmArduino::motor2rad(const raw_type &commands, angle_type &angles) {
	bool ret_val = true;
	for (int i = 0; i < n_motors; i++) {
		functions::LinearInterpolator &interpol = *mot_pos_interpol_[i];
		angles[i] = interpol.interpolate(commands[i]);
		ret_val &= interpol.inRange(angles[i]);
	}
	
	return ret_val;
}

bool ArmArduino::checkJointLimits( const angle_type &angles) 
{
	raw_type commands;
	ArmArduino::rad2motor(angles, commands);
	return ArmArduino::checkJointLimits(commands);
}
  
bool ArmArduino::checkJointLimits(const raw_type &joint_values)
{
	bool ret_val = true;

	for (int i = 0; i < n_motors && ret_val; i++) {
		functions::LinearInterpolator &curr_inter = *(mot_pos_interpol_[i]);
		auto min = curr_inter.upper_bound(0)->first ;
		auto max = curr_inter.lower_bound(2000)->first ; // Usually the commands are in the [0, 2000] range

		ret_val &= joint_values[i] > min && joint_values[i] < max;
	}
	return ret_val;
}
  
void ArmArduino::correctJointLimits(raw_type &joint_values)
{
	for (int i = 0; i < n_motors; i++) {
		functions::LinearInterpolator &curr_inter = *(mot_pos_interpol_[i]);
		auto min = static_cast<int16_t>(curr_inter.upper_bound(0)->first);
		auto it = curr_inter.lower_bound(2500) ;
		it--;
		auto max = static_cast<int16_t>(it->first); // Usually the commands are in the [0, 2000] range

		std::cout << "Trying to saturate between: " << min << " and " << max << std::endl;
		
		joint_values[i] = functions::saturate(joint_values[i], min, max); 
	}
}



#endif /* _ARM_ARDUINO_HPP_ */

