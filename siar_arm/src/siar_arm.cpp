
#include "siar_arm/siar_arm.hpp"
  
using angle_type = boost::array<double, ARM_JOINTS>;
using raw_type = boost::array<int16_t, ARM_JOINTS>;  
  
void SiarArm::load_data(const std::string &mot_arm_file, const std::string &pos_arm_file) {
	  
	for (int i = 0; i < n_motors; i++) {
		std::ostringstream mot_arm, pos_arm;
		mot_arm << mot_arm_file << i;
		pos_arm << pos_arm_file << i;
		mot_pos_interpol_.push_back(new functions::LinearInterpolator(mot_arm.str(), pos_arm.str()));
		pos_mot_interpol_.push_back(new functions::LinearInterpolator(pos_arm.str(), mot_arm.str()));
	}
}
    
bool SiarArm::inverseKinematics(double x, double y, double z, angle_type &result, bool erase_remaining = true)
{
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
  
bool SiarArm::rad2motor(const angle_type &angles, raw_type &commands) {
	bool ret_val = true;
	for (int i = 0; i < n_motors; i++) {
		functions::LinearInterpolator &interpol = *pos_mot_interpol_[i];
		commands[i] = interpol.interpolate(angles[i]);
		ret_val &= interpol.inRange(angles[i]);
	}
	return ret_val;
}

bool SiarArm::motor2rad(const raw_type &commands, angle_type &angles) {
	bool ret_val = true;
	for (int i = 0; i < n_motors; i++) {
		functions::LinearInterpolator &interpol = *mot_pos_interpol_[i];
		angles[i] = interpol.interpolate(commands[i]);
		ret_val &= interpol.inRange(angles[i]);
	}
	return ret_val;
}
  
void SiarArm::forwardKinematics(const raw_type &joint_values, double &x, double &y, double &z)
{
	//       X = cos(a1) *(L2*sin(a2)+L3*sin(a2+a3))
	// Y = L1 + (L2*cos(a2)+L3*cos(a2+a3))
	// Z = -1 * sin(a1) *(L2*sin(a2)+L3*sin(a2+a3))
	// Donde Y es positiva desde la base en dirección a la primera articulación, X es ortogonal  a Y en el plano proyectado por el dibujo y Z va desde el plano hacia dentro.

	angle_type angles;
	SiarArm::motor2rad(joint_values, angles);
	SiarArm::forwardKinematics(angles, x, y ,z);
}

	void SiarArm::forwardKinematics(const angle_type &angles, double &x, double &y, double &z) {
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

std::vector<raw_type> SiarArm::straightInterpol(double x, double y, double z, uint8_t n_points, const raw_type &curr_pos)
{
	double a_x {0.0};
	double a_y {0.0};
	double a_z {0.0};
	SiarArm::forwardKinematics(curr_pos, a_x, a_y, a_z);
	double i_x = (x - a_x)/(n_points+1);	  
	double i_y = (y - a_y)/(n_points+1);
	double i_z = (z - a_z)/(n_points+1);

	std::vector<raw_type > ret;

	angle_type angles;
	raw_type commands;
	for( int i = 0; i < n_points + 1; i++)	
	{	
		a_x += i_x;
		a_y += i_y;
		a_z += i_z;

		if (inverseKinematics(a_x, a_y, a_z, angles)) 
		{
			SiarArm::rad2motor(angles, commands);
			ret.push_back(commands);
		} 
		else 
		{
			ret.clear();
			return ret;
		}
	}
	return ret;
}

bool SiarArm::checkJointLimits( const angle_type &angles) 
{
	raw_type commands;
	SiarArm::rad2motor(angles, commands);
	return SiarArm::checkJointLimits(commands);
}
  
bool SiarArm::checkJointLimits(const raw_type &joint_values)
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
  
void SiarArm::correctJointLimits(raw_type &joint_values)
{
	for (int i = 0; i < n_motors; i++) {
		functions::LinearInterpolator &curr_inter = *(mot_pos_interpol_[i]);
		auto min = static_cast<int16_t>(curr_inter.upper_bound(0)->first);
		auto it = curr_inter.lower_bound(2000) ;
		it--;
		auto max = static_cast<int16_t>(it->first); // Usually the commands are in the [0, 2000] range

		std::cout << "Trying to saturate between: " << min << " and " << max << std::endl;
		
		joint_values[i] = functions::saturate(joint_values[i], min, max); 
	}
}

bool SiarArm::checkTemperatureAndStatus(const boost::array<uint8_t,5> &herculex_temperature, const boost::array<uint8_t,5> &herculex_status)
{
	for(int i = 0; i < n_motors; i++)
	{
		if (herculex_temperature[i]<0 && herculex_temperature[i]>50)
		{
			ROS_ERROR("TEMPERATURE OF THE %d LINK IS OUT OF RANGE: %d", i, herculex_temperature[i]);
			return false;
		}
		//~ if (herculex_status[i]!=1)    // TODO: Check this!!
		//~ {
		//~ ROS_ERROR("%d LINK STATUS: %d", i, herculex_status[i]);
		//~ ret_val = false;
		//~ }
	}
	return true;  
}
  


