#ifndef __YAW_MIXER_CHUR_HPP__
#define __YAW_MIXER_CHUR_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include "yaw_mixer.hpp"

//! @class This This class estimates the yaw angle by adding a k 
//!                   as a function of the gyro and ticks rate
class YawMixerChur:public YawMixer
{
public:
  YawMixerChur(double hz) {
    init(hz);
  }
  
  //! @brief Default initializing function
  //! @param  hz Fitering frequency in Hz
  virtual void init(double hz);
  
  //! @brief EKF correction stage based on gyro rate and update rate
  //! @param gyroRate gyro rates in rad/s
  //! @return The yaw estimation
  virtual double update(double gyro_rate, double dt = -1.0);
  
  //! @brief EKF correction stage based on ticks rate 
  //! @param tickRate Yaw rate of tick estimation in rad/s
  virtual double correct(double ticks_rate);
  
  inline double getK() const {return k_;}
  
protected:
  Eigen::MatrixXd P;        // Cov Matrix
  double k_, k_dev_; // Extra parameter to be estimated --> the gain between estimated tick rate and real yaw rate
  double min_tic_dev_, tic_prop_dev_; // The noise on the tick measure depends on the rate
  
};

void YawMixerChur::init(double hz)
{
  // Initial values
  rate_ = bias_ = yaw_ = 0.0;
  k_ = 1.0; // We suppose that the gain is correctly tuned at first
  
  // Noise related to measures
  tic_prop_dev_ = 1.0; // Instead than a constant noise for tick, it will depend on the yaw rate n=a+b*rate
  min_tic_dev_ = 0.001;
  gyr_dev_ = 0.005;
  bia_dev_ = 0.005 / hz;
  k_dev_ = 0.01;
  T_ = 1 / hz;
  
  // Initialize covariance matrix
  P.setIdentity(4, 4);
  P(0,0) = 0.000001*0.000001;
  P(1,1) = 0.01*0.01;
  P(2,2) = 0.001*0.001;
  P(3,3) = 0.05*0.05;
}

double YawMixerChur::update(double gyro_rate, double dt)
{
  // Prediction stage
  if (dt > 0.0) {
    T_ = dt;
  }

  // Compute matrix F 
  Eigen::Matrix<double, 4, 4> F;
  F(0,0) = 1, F(0,1) = T_, F(0,2) = 0, F(0,3) = 0.0;
  F(1,0) = 0, F(1,1) = 1 , F(1,2) = 0, F(1,3) = 0.0;
  F(2,0) = 0, F(2,1) = 0 , F(2,2) = 1, F(2,3) = 0.0;
  F(3,0) = 0, F(3,1) = 0 , F(3,2) = 0, F(3,3) = 0.0;
  
  // Update covariance matrix
  P = F*P*F.transpose();
  P(1,1) += gyr_dev_*gyr_dev_;
  P(2,2) += bia_dev_*bia_dev_;
  
  // Update state vector
  yaw_ = yaw_ + T_*rate_;
  
  // Create measurement jacobian H
  Eigen::Matrix<double, 1, 4> H;
  H(0,0) = 0.0;  H(0,1) = 1.0;  H(0,2) = 1.0;  H(0,3) = 0.0;
  
  // Compute measurement noise jacoban R
  Eigen::Matrix<double, 1, 1> R;
  tic_dev_ = min_tic_dev_ + tic_prop_dev_ * fabs(rate_);
  R(0,0) = gyr_dev_*gyr_dev_;

  // Compute innovation matrix
  Eigen::Matrix<double, 1, 1> S;
  S = H*P*H.transpose()+R;
  
  // Compute kalman gain
  Eigen::Matrix<double, 4, 1> K;
  K = P*H.transpose()*S.inverse();
  //std::cout << "P: \n" << P(0,0) << ", " << P(0,1) << std::endl;     
  //std::cout << P(1,0) << ", " << P(1,1) << std::endl; 

  // Compute mean error
  double y = gyro_rate - rate_;
  //std::cout << "Y: " << y[0] << ", " << y[1] << std::endl;
  
  // Compute new state vector
  yaw_ += K(0,0)*y;
  rate_  += K(1,0)*y;
  bias_  += K(2,0)*y;
  k_ += K(3,0)*y;
	      
  // Compute new covariance matrix
  Eigen::Matrix<double, 4, 4> I;
  I.setIdentity(4, 4);
  P = (I-K*H)*P;
  
  return yaw_;
}



// EKF correction stage based on ticks rate
double YawMixerChur::correct(double tick_rate) {
  // Create measurement jacobian H
  Eigen::Matrix<double, 1, 4> H;
  H(0,0) = 0.0;  H(0,1) = k_;  H(0,2) = 0.0;  H(0,3) = rate_;
  
  // Compute measurement noise jacoban R
  Eigen::Matrix<double, 1, 1> R;
  tic_dev_ = min_tic_dev_ + tic_prop_dev_ * rate_;
  R(0,0) = tic_dev_*tic_dev_;

  // Compute innovation matrix
  Eigen::Matrix<double, 1, 1> S;
  S = H*P*H.transpose()+R;
  
  // Compute kalman gain
  Eigen::Matrix<double, 4, 1> K;
  K = P*H.transpose()*S.inverse();
  //std::cout << "P: \n" << P(0,0) << ", " << P(0,1) << std::endl;     
  //std::cout << P(1,0) << ", " << P(1,1) << std::endl; 

  // Compute mean error
  double y = tick_rate - rate_;
  //std::cout << "Y: " << y[0] << ", " << y[1] << std::endl;
  
  // Compute new state vector
  yaw_ += K(0,0)*y;
  rate_  += K(1,0)*y;
  bias_  += K(2,0)*y;
  k_ += K(3,0)*y;
	      
  // Compute new covariance matrix
  Eigen::Matrix<double, 4, 4> I;
  I.setIdentity(4, 4);
  P = (I-K*H)*P;
  
  return yaw_;
}

#endif
