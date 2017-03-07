#ifndef __ARDUIMUV3_IMU_HPP
#define __ARDUIMUV3_IMU_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "arduimu_v3/yaw_mixer.hpp"
#include "arduimu_v3/yaw_mixer_chur.hpp"
#include "arduimu_v3/yaw_mixer_caballero.hpp"

class IMU {
public:
  //! @brief Preferred constructor
  //! @param rate Rate of the filter in Hz
  IMU(double rate);
  
  //! @brief Destructor
  ~IMU();
  
  //! @brief Gets the absolute angles and their rates
  //! @param z_rate_ticks The yaw rate estimation based on the measures of the ticks
  sensor_msgs::Imu getFilteredAngles(double z_rate_ticks);
  
  //! @brief Callback function of imu messages
  void imuReceived(const sensor_msgs::Imu::ConstPtr& imu);
  
  inline bool isInit() const {return init_;}
  
protected:
  sensor_msgs::Imu last_message_;
  YawMixer *yaw_mixer_;
  std::string yaw_mixer_type_;
  double rate_;
  ros::NodeHandle n_;
  ros::NodeHandle pn_;
  ros::Subscriber imu_sub_;
  bool init_;
  std::string imu_topic_;
  ros::Time imu_t_;
};

IMU::IMU(double rate):yaw_mixer_(NULL),rate_(rate), n_(), pn_("~"), init_(false)
{
  
  pn_.param<std::string>("imu_topic", imu_topic_, "/arduimu_v3/imu");
  pn_.param<std::string>("yaw_mixer", yaw_mixer_type_, "Bias");
  
  
  ROS_INFO("IMU::IMU() --> Yaw mixer type: %s.", yaw_mixer_type_.c_str());
  if (yaw_mixer_type_ == "K") {
    ROS_INFO("Using yaw mixer for K estimate.");
    yaw_mixer_ = new YawMixerChur(rate_);
  } else {
    yaw_mixer_ = new YawMixerCaballero(rate_);
  }
  
  imu_sub_ = n_.subscribe<sensor_msgs::Imu>(imu_topic_, 1, &IMU::imuReceived, this);
  ROS_INFO("IMU::IMU() --> Subscribed to IMU topic: %s", imu_topic_.c_str());
}

IMU::~IMU()
{
  delete yaw_mixer_;
}


void IMU::imuReceived(const sensor_msgs::Imu::ConstPtr& imu)
{
  // Copy the data and raise the flag!
  last_message_ = *imu;
  ros::Time t_ = ros::Time::now();
//   std::cerr << "dt imuReceived: " << (t_ - imu_t_).toSec() << "\n";
  if (init_) {
    yaw_mixer_->update(last_message_.angular_velocity.z, (t_-imu_t_).toSec());
  } else {
    yaw_mixer_->update(last_message_.angular_velocity.z, 1 / rate_);
    init_ = true;
  }
  ROS_DEBUG("IMU::IMU() --> Received message. dt = %f", (t_ - imu_t_).toSec());
  imu_t_ = t_;
}

sensor_msgs::Imu IMU::getFilteredAngles(double z_rate_ticks) 
{
  // Get yaw from filter and bypass everything else
  sensor_msgs::Imu ret = last_message_;
  tf::Quaternion q(last_message_.orientation.x, last_message_.orientation.y, last_message_.orientation.z, last_message_.orientation.w);
  tf::Matrix3x3 m(q);
  
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = yaw_mixer_->correct( z_rate_ticks);
  // DEBUG
//   std::cerr << "ang_vel = " << last_message_.angular_velocity.z << "\t z_rate_ticks = " << z_rate_ticks << "\t yaw = " << yaw;
//   std::cerr << "\t bias: " << yaw_mixer_->getBIAS() ;
//   YawMixerChur *y_k = dynamic_cast<YawMixerChur *>(yaw_mixer_);
//   if (yaw_mixer_type_ == "K" && y_k != NULL) {
//     std::cerr << "\t k = " << y_k->getK();
//   }
//   std::cerr << std::endl;
  
  m.setRPY(roll, pitch, yaw);
  m.getRotation(q);
  ret.orientation.x = q.x();
  ret.orientation.y = q.y();
  ret.orientation.z = q.z();
  ret.orientation.w = q.w();
  
  return ret;
}

#endif
