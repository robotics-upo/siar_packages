#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <string>
#include "arduimu_v3.hpp"
#include "imufilter.hpp"

int main( int argc, char **argv)
{
	// Setup ROS
	ros::init(argc, argv, "arduimu_v3");
	ros::NodeHandle nh;
	ros::NodeHandle lnh("~");
	ros::Publisher pubImu, pubImuRaw, pubMagRaw;
	
	// Read parameters
	bool filter_imu, use_mag;
	std::string device, frame_id;
	if(!lnh.getParam("device", device))
	{
		std::cout << "No device specified! Assumming /dev/ttyUSB0 as default" << std::endl; 
		device = "/dev/ttyUSB0";	
	}
	if(!lnh.getParam("frame_id", frame_id))
		frame_id = "/arduimu_v3";	
	if(!lnh.getParam("filter_imu", filter_imu))
		filter_imu = true;	
	if(!lnh.getParam("use_mag", use_mag))
		use_mag = true;	
		
	// Advertise publishers
	pubImuRaw = nh.advertise<sensor_msgs::Imu>("/arduimu_v3/imuRaw", 1);
	pubMagRaw = nh.advertise<sensor_msgs::MagneticField>("/arduimu_v3/magRaw", 1);
	if(filter_imu)
		pubImu = nh.advertise<sensor_msgs::Imu>("/arduimu_v3/imu", 1);
	
	// Setup IMU
	ArduimuDev imuDev(device.c_str());
	ArduImuRawData rawData;
	
	// Setup IMU fltering
	ImuFilter imuFilter(1.0/150.0);		// ArduMU serves data at 150Hz

	// Imu publishing
	int seq = 0;
	sensor_msgs::Imu imuMsg, imuRawMsg;
	sensor_msgs::MagneticField magMsg;
	while(ros::ok())
	{
		// Read raw imu data
		if(imuDev.readData(rawData) != 1)
			continue;
		
		// Fill up imu msg
		imuRawMsg.header.seq = seq;
		imuRawMsg.header.frame_id = frame_id;
		imuRawMsg.header.stamp = ros::Time::now();
		imuRawMsg.orientation.x = 0;
		imuRawMsg.orientation.y = 0;
		imuRawMsg.orientation.z = 0;
		imuRawMsg.orientation.w = 1;
		imuRawMsg.angular_velocity.x = DEG2RAD(rawData.gyr_x);
		imuRawMsg.angular_velocity.y = DEG2RAD(rawData.gyr_y);
		imuRawMsg.angular_velocity.z = DEG2RAD(rawData.gyr_z);
		imuRawMsg.linear_acceleration.x = rawData.acc_x*9.80665;
		imuRawMsg.linear_acceleration.y = rawData.acc_y*9.80665;
		imuRawMsg.linear_acceleration.z = rawData.acc_z*9.80665;
		pubImuRaw.publish(imuRawMsg);
	
		// Fill up mag msg
		if(rawData.mag_update)
		{
			magMsg.header.seq = seq;
			magMsg.header.frame_id = frame_id;
			magMsg.header.stamp = imuRawMsg.header.stamp;
			magMsg.magnetic_field.x = rawData.mag_x;
			magMsg.magnetic_field.y = rawData.mag_y;
			magMsg.magnetic_field.z = rawData.mag_z;
			pubMagRaw.publish(magMsg);
		}
		
		// Filter raw data to get stabilized IMU
		if(filter_imu)
		{
			if(!imuFilter.isInit())
				imuFilter.initialize(imuRawMsg);
			else
			{
				// Filetr IMU data
				imuFilter.predict(imuRawMsg.angular_velocity.x, -imuRawMsg.angular_velocity.y, -imuRawMsg.angular_velocity.z);
				if(use_mag)
					imuFilter.update(rawData.acc_x, -rawData.acc_y, -rawData.acc_z, rawData.mag_x, -rawData.mag_y, -rawData.mag_z);
				else
					imuFilter.update(rawData.acc_x, -rawData.acc_y, -rawData.acc_z);
					
				// Publish estimation
				double rx, ry, rz, bx, by, bz;
				imuFilter.getAngles(rx, ry, rz);
				//std::cout << "Rx: " << 180*rx/M_PI << ", Ry: " << -180*ry/M_PI << ", Rz: " << -180*rz/M_PI << std::endl;
				imuFilter.getBIAS(bx, by, bz);
				if(!use_mag)
					rz = 0.0;
				tf::Quaternion q = tf::createQuaternionFromRPY(rx, -ry, -rz);
				imuMsg.header.seq = seq;
				imuMsg.header.frame_id = frame_id;
				imuMsg.header.stamp = imuRawMsg.header.stamp;
				imuMsg.orientation.x = q.getX();
				imuMsg.orientation.y = q.getY();
				imuMsg.orientation.z = q.getZ();
				imuMsg.orientation.w = q.getW();
				imuMsg.angular_velocity.x = DEG2RAD(rawData.gyr_x-bx);
				imuMsg.angular_velocity.y = DEG2RAD(rawData.gyr_y+by);
				imuMsg.angular_velocity.z = DEG2RAD(rawData.gyr_z+bz);
				imuMsg.linear_acceleration.x = rawData.acc_x*9.80665;
				imuMsg.linear_acceleration.y = rawData.acc_y*9.80665;
				imuMsg.linear_acceleration.z = rawData.acc_z*9.80665;
				pubImu.publish(imuMsg);
			}
		}
		
		// Increase seq counter
		seq++;

		// Ros spin
		ros::spinOnce();
	}
    
	return 0;
}


