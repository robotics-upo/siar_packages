#ifndef __YAW_MIXER_HPP__
#define __YAW_MIXER_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>

//! @class This abstract class is intended to serve as base to estimate the yaw variable
//!                   as a function of the gyro and ticks rate
class YawMixer
{
public:
  
  //! @brief Default initializing function
  //! @param  hz Fitering frequency in Hz
  virtual void init(double hz) = 0;
  
  //! @brief EKF update stage based on gyro rate 
  //! @param gyroRate gyro rates in rad/s
  virtual double update(double gyro_rate, double dt = -1.0) = 0;
  
  //! @brief EKF correction stage based on tick rate 
  //! @param tickRate Tick rates in rad/s
  virtual double correct(double ticks_rate) = 0;
  
  // Get estimated angle rate in rad/s
  double getRate(void)
  {  
    return rate_;
  }

  // Get estimated BIAS in rad/s
  double getBIAS(void)
  {
    return bias_;
  }
  
protected:
  
  // Signals to be estimated
  double yaw_, rate_, bias_; // In the minimum case x = [rate, bias]

  // EKF Parameters
  double tic_dev_;
  double gyr_dev_;
  double bia_dev_;
  double T_; // Time interval from the last call
};

#endif








