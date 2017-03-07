#ifndef __IMUFILTER_HPP__
#define __IMUFILTER_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <vector>

// Convenient constants
#define RAD2DEG(X) ( (X) * 57.2957795131)
#define DEG2RAD(X) ( (X) * 0.01745329251)
#define LIN2GRAV(X) ( (X) * 0.10197162)

class ImuFilter
{
public:
	
	// Default constructor
	// Input: 
	// - prediction period in seconds
	// - initial calibration time in seconds
	ImuFilter(double _T, double _calibTime = 5.0)
	{
		init = false;
		T = _T;
		T2 = _T*_T;
		calibTime = _calibTime;
		calibSize = (int)_calibTime/_T;
		calibData.resize(calibSize);
		calibIndex = 0;

		// IMU EKF parameters
		acc_dev = 0.01;//0.001
		gyr_dev = 0.005;//0.01;
		mag_dev = 0.3;
		mag_xs = 1.0;
		mag_ys = 1.0;
		mag_zs = 1.0;
		mag_xo = 0.0;
		mag_yo = 0.0;
		mag_zo = 0.0;
		bia_dev = 0.00001;//0.000001;
		bia_th = 0.01; //0.001;
	}

	
	// Initialize EKF
	// Input: last sensor_msgs::Imu
	// The user must continue calling this function with new gyro data until it returns true
	bool initialize(sensor_msgs::Imu &msg)
	{
		double gx_m, gy_m, gz_m, gx_m2, gy_m2, gz_m2, gx_d, gy_d, gz_d; 
	
		// Do we have enought data for IMU initilaization
		calibData[calibIndex++%calibSize] = msg;
		if(calibIndex < calibSize)
			return false;
		
		// Compute mean value and mean square
		gx_m = gy_m = gz_m = gx_m2 = gy_m2 = gz_m2 = 0.0;
		for(int i = 0; i < (int)calibData.size(); i++)
		{
			gx_m += calibData[i].angular_velocity.x;
			gy_m += -calibData[i].angular_velocity.y;
			gz_m += -calibData[i].angular_velocity.z;
			gx_m2 += calibData[i].angular_velocity.x*calibData[i].angular_velocity.x;
			gy_m2 += calibData[i].angular_velocity.y*calibData[i].angular_velocity.y;
			gz_m2 += calibData[i].angular_velocity.z*calibData[i].angular_velocity.z;
		}
		gx_m = gx_m/(double)calibData.size();
		gy_m = gy_m/(double)calibData.size();
		gz_m = gz_m/(double)calibData.size();
		gx_m2 = gx_m2/(double)calibData.size();
		gy_m2 = gy_m2/(double)calibData.size();
		gz_m2 = gz_m2/(double)calibData.size();
		
		// Compute standar deviation of gyros
		gx_d = sqrt(gx_m2-gx_m*gx_m);
		gy_d = sqrt(gy_m2-gy_m*gy_m);
		gz_d = sqrt(gz_m2-gz_m*gz_m);
		
		// Initalize compass calibration
		mag_cal[0] = mag_xs; mag_cal[1] = mag_ys; mag_cal[2] = mag_zs; mag_cal[3] = mag_xo; mag_cal[4] = mag_yo; mag_cal[5] = mag_zo;
		
		// Initialize sensor variances
		acc_var[0] = acc_dev*acc_dev; 	acc_var[1] = acc_dev*acc_dev; 	acc_var[2] = acc_dev*acc_dev;	// Variance in g
		gyr_var[0] = gyr_dev*gyr_dev; 	gyr_var[1] = gyr_dev*gyr_dev; 	gyr_var[2] = gyr_dev*gyr_dev;	// Variance in rad/s
		mag_var[0] = mag_dev*mag_dev; 	mag_var[1] = mag_dev*mag_dev; 	mag_var[2] = mag_dev*mag_dev;	// Variance in mGaus wth data normalized to 1
		bia_var[0] = bia_dev*bia_dev; 	bia_var[1] = bia_dev*bia_dev; 	bia_var[2] = bia_dev*bia_dev;	// Variance in rad/s 
		
		// Initialize accelerometer threshold
		accTh = sqrt(acc_var[0]+acc_var[1]+acc_var[2]);
		
		// Initialize state vector x = [rx, ry, rz, gbx, gby, gbz]
		rx = ry = rz = 0.0;
		gbx = gx_m;
		gby = gy_m;
		gbz = gz_m;
		
		// Initialize covariance matrix
		P.setIdentity(6, 6);
		P(0,0) = M_PI_2;
		P(1,1) = M_PI_2;
		P(2,2) = M_PI_2;
		P(3,3) = 0.01*0.01;
		P(4,4) = 0.01*0.01;
		P(5,5) = 0.01*0.01;

		// Check if calibration is good enought
		if(gx_d < bia_th && gy_d < bia_th && gz_d < bia_th)
		{
			init = true;
		
			return true;
		}
		else
			return false;
	}	
	
	// EKF prediction stage based on gyro information
	// Input: raw gyro sensors rad/s
	bool predict(double gx, double gy, double gz)
	{
		// Check initialization 
		if(!init)
			return false;
		
		// Compute matrix F 
		Eigen::Matrix<double, 6, 6> F;
		F(0,0) = 1, F(0,1) = 0, F(0,2) = 0, F(0,3) = -T, F(0,4) = 0,   F(0,5) = 0;
		F(1,0) = 0, F(1,1) = 1, F(1,2) = 0, F(1,3) = 0,  F(1,4) = -T,  F(1,5) = 0;
		F(2,0) = 0, F(2,1) = 0, F(2,2) = 1, F(2,3) = 0,  F(2,4) = 0,   F(2,5) = -T;
		F(3,0) = 0, F(3,1) = 0, F(3,2) = 0, F(3,3) = 1,  F(3,4) = 0,   F(3,5) = 0;
		F(4,0) = 0, F(4,1) = 0, F(4,2) = 0, F(4,3) = 0,  F(4,4) = 1,   F(4,5) = 0;
		F(5,0) = 0, F(5,1) = 0, F(5,2) = 0, F(5,3) = 0,  F(5,4) = 0,   F(5,5) = 1;

		// Update covariance matrix
		P = F*P*F.transpose();
		P(0,0) += gyr_var[0]*T2;
		P(1,1) += gyr_var[1]*T2;
		P(2,2) += gyr_var[2]*T2;
		P(3,3) += bia_var[0]*T;
		P(4,4) += bia_var[1]*T;
		P(5,5) += bia_var[2]*T;
		
		// Update state vector
		rx += T*(gx - gbx);  
		ry += T*(gy - gby);  
		rz += T*(gz - gbz);  
														
		return true; 
	}
	
	// EKF update stage based on accelerometer information
	// Input: raw accelerometer sensors in g
	bool update(double ax, double ay, double az)
	{
		double crx, srx, cry, sry, mod, y[3];
		
		// Check initialization 
		if(!init)
			return false;
		
		// Pre-compute constants
		crx = cos(rx); 
		cry = cos(ry); 
		srx = sin(rx); 
		sry = sin(ry); 
		
		// Create measurement jacobian H
		Eigen::Matrix<double, 3, 6> H;
		H(0,0) = 0;			H(0,1) = cry;		H(0,2) = 0; H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
		H(1,0) = -crx*cry; 	H(1,1) = srx*sry; 	H(1,2) = 0; H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
		H(2,0) = cry*srx;	H(2,1) = crx*sry;	H(2,2) = 0; H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
		
		// Compute measurement noise jacoban R
		Eigen::Matrix<double, 3, 3> R;
		mod = fabs(sqrt(ax*ax + ay*ay + az*az)-1);
		R.setZero(3, 3);
		R(0,0) = acc_var[0];
		R(1,1) = acc_var[1];
		R(2,2) = acc_var[2];
		/*if(mod > accTh)
		{
			R(0,0) += 1.5*mod*mod;
			R(1,1) += 1.5*mod*mod;
			R(2,2) += 1.5*mod*mod;
		}*/
		
		// Compute innovation matrix
		Eigen::Matrix<double, 3, 3> S;
		S = H*P*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::Matrix<double, 6, 3> K;
		K = P*H.transpose()*S.inverse();
		
		// Compute mean error
		y[0] = ax-sry;
		y[1] = ay+cry*srx;
		y[2] = az+crx*cry;
		
		// Compute new state vector
		rx += K(0,0)*y[0]+K(0,1)*y[1]+K(0,2)*y[2];
		ry += K(1,0)*y[0]+K(1,1)*y[1]+K(1,2)*y[2];
		rz += K(2,0)*y[0]+K(2,1)*y[1]+K(2,2)*y[2];
		gbx += K(3,0)*y[0]+K(3,1)*y[1]+K(3,2)*y[2];
		gby += K(4,0)*y[0]+K(4,1)*y[1]+K(4,2)*y[2];
		gbz += K(5,0)*y[0]+K(5,1)*y[1]+K(5,2)*y[2];
		
		// Compute new covariance matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
		P = (I-K*H)*P;
		
		return true;
	}

	// EKF update stage based on accelerometer information
	// Input: raw accelerometer sensors and raw magnetometer
	bool update(double ax, double ay, double az, double mx, double my, double mz)
	{
		double crx, srx, cry, sry, crz, srz, mod, y[5], hx, hy;
		
		// Check initialization 
		if(!init)
			return false;
		
		// Remove distortion and normalize magnetometer
		mx = (mx - mag_cal[3])/mag_cal[0];
		my = (my - mag_cal[4])/mag_cal[1];
		mz = (mz - mag_cal[5])/mag_cal[2];
		mod = sqrt(mx*mx + my*my + mz*mz);
		mx /= mod;
		my /= mod;
		mz /= mod;
		
		// Pre-compute constants
		crx = cos(rx); 
		cry = cos(ry); 
		crz = cos(rz); 
		srx = sin(rx); 
		sry = sin(ry); 
		srz = sin(rz); 
		
		// Create measurement jacobian H
		Eigen::Matrix<double, 5, 6> H;
		H(0,0) = 0;			H(0,1) = cry;		H(0,2) = 0; 	H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
		H(1,0) = -crx*cry; 	H(1,1) = srx*sry; 	H(1,2) = 0; 	H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
		H(2,0) = cry*srx;	H(2,1) = crx*sry;	H(2,2) = 0; 	H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
		H(3,0) = 0;			H(3,1) = 0;			H(3,2) = -srz; 	H(3,3) = 0; H(3,4) = 0; H(3,5) = 0;
		H(4,0) = 0;			H(4,1) = 0;			H(4,2) = -crz; 	H(4,3) = 0; H(4,4) = 0; H(4,5) = 0;
		
		// Compute measurement noise jacoban R
		Eigen::Matrix<double, 5, 5> R;
		mod = fabs(sqrt(ax*ax + ay*ay + az*az)-1);
		R.setZero(5, 5);
		R(0,0) = acc_var[0];
		R(1,1) = acc_var[1];
		R(2,2) = acc_var[2];
		R(3,3) = mag_var[0];
		R(4,4) = mag_var[1];
		/*if(mod > accTh)
		{
			R(0,0) += 1.5*mod*mod;
			R(1,1) += 1.5*mod*mod;
			R(2,2) += 1.5*mod*mod;
		}*/
		
		// Compute innovation matrix
		Eigen::Matrix<double, 5, 5> S;
		S = H*P*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::Matrix<double, 6, 5> K; 
		K = P*H.transpose()*S.inverse();
		
		// Compute mean error
		hx = mx*cry + mz*crx*sry + my*srx*sry;
		hy = my*crx - mz*srx;
		mod = sqrt(hx*hx+hy*hy);
		y[0] = ax-sry;
		y[1] = ay+cry*srx;
		y[2] = az+crx*cry;
		y[3] = hx/mod-crz;
		y[4] = hy/mod+srz;
		
		// Compute new state vector
		rx += K(0,0)*y[0]+K(0,1)*y[1]+K(0,2)*y[2]+K(0,3)*y[3]+K(0,4)*y[4];
		ry += K(1,0)*y[0]+K(1,1)*y[1]+K(1,2)*y[2]+K(1,3)*y[3]+K(1,4)*y[4];
		rz += K(2,0)*y[0]+K(2,1)*y[1]+K(2,2)*y[2]+K(2,3)*y[3]+K(2,4)*y[4];
		gbx += K(3,0)*y[0]+K(3,1)*y[1]+K(3,2)*y[2]+K(3,3)*y[3]+K(3,4)*y[4];
		gby += K(4,0)*y[0]+K(4,1)*y[1]+K(4,2)*y[2]+K(4,3)*y[3]+K(4,4)*y[4];
		gbz += K(5,0)*y[0]+K(5,1)*y[1]+K(5,2)*y[2]+K(5,3)*y[3]+K(5,4)*y[4];
		
		// Compute new covariance matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
		P = (I-K*H)*P;
		
		return true;
	}
	
	// Get estimated roll, pith and yaw in radians interval [-PI,PI] rad
	bool getAngles(double &rx_, double &ry_, double &rz_)
	{
		rx_ = Pi2PiRange(rx);
		ry_ = Pi2PiRange(ry);
		rz_ = Pi2PiRange(rz);
		
		return true;
	}

	// Get estimated bx, by and bz gyro BIAS in rad/s
	bool getBIAS(double &gbx_, double &gby_, double &gbz_)
	{
		gbx_ = gbx;
		gby_ = gby;
		gbz_ = gbz;
		
		return true;
	}
	
	// Is the IMU init?
	bool isInit(void)
	{
		return init;
	}
	
protected:

	//! Round down absolute function
	double Floor_absolute( double value )
	{
	  if (value < 0.0)
		return ceil( value );
	  else
		return floor( value );
	}

	//! Convert angles into interval [-PI,0,PI]
	double Pi2PiRange(double cont_angle)
	{
		double bound_angle = 0.0;
		if(fabs(cont_angle)<=M_PI)
			bound_angle= cont_angle;
		else
		{
			if(cont_angle > M_PI)
				bound_angle = (cont_angle-2*M_PI) - 2*M_PI*Floor_absolute((cont_angle-M_PI)/(2*M_PI));
			
			if(cont_angle < - M_PI)
				bound_angle = (cont_angle+2*M_PI) - 2*M_PI*Floor_absolute((cont_angle+M_PI)/(2*M_PI));
		}
		
		return bound_angle;
	}
		
	// IMU Kalman filter prediction period and period^2
	double T, T2;
	
	// IMU Kalman filter matrixes 
	double rx, ry, rz, gbx, gby, gbz;  			// x = [rx, ry, rz, gbx, gby, gbz]
	Eigen::MatrixXd P;	
	
	// Sensor calibration information [gainX, painY, gainZ, offsetX, offsetY, offsetZ]
	double mag_cal[6];
	
	// Sensor variances [varX, varY, varZ]
	double acc_var[3]; 
	double mag_var[3];
	double gyr_var[3];
	double bia_var[3];
	
	// Accelerometer threshold for filter updating
	double accTh;
	
	// Filter initialized
	bool init;
	
	// EKF Parameters
	double acc_dev;
	double gyr_dev;
	double mag_dev;
	double bia_dev;
	double bia_th;
	double mag_xs;
	double mag_ys;
	double mag_zs;
	double mag_xo;
	double mag_yo;
	double mag_zo;
	
	// Input calibration data
	double calibTime;
	int calibIndex, calibSize;
	std::vector<sensor_msgs::Imu> calibData;
};

#endif








