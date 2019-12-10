#ifndef _SIAR_ARM_ROS_MBZIRC_HPP_
#define _SIAR_ARM_ROS_MBZIRC_HPP_

#include "siar_arm/siar_arm_ros.hpp"

class SiarArmROSMBZirc:public SiarArmROS {
  public:
  SiarArmROSMBZirc(ros::NodeHandle &nh, ros::NodeHandle &pnh):SiarArmROS(nh, pnh),status_cont(0) {
   
  }

   virtual void start() {
     ros::NodeHandle nh;

     // Configure ROS comms
     if (enable_marker_) {
      arm_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("arm_marker", 1);
    }
    siar_status_sub_ = nh.subscribe<siar_driver::SiarStatus>("siar_node/siar_status", 1, &SiarArmROSMBZirc::statusCb, this);
     if (enable_server_) {

      arm_pan_sub_ = nh.subscribe<std_msgs::Float32>("arm_pan", 1, &SiarArmROS::armPanReceived, dynamic_cast<SiarArmROS *>(this));
      arm_tilt_sub_ = nh.subscribe<std_msgs::Float32>("arm_tilt", 1, &SiarArmROS::armTiltReceived, dynamic_cast<SiarArmROS *>(this));
      set_pan_sub_ = nh.subscribe<std_msgs::Float32>("set_pan", 1, &SiarArmROS::setPanReceived, dynamic_cast<SiarArmROS *>(this));
      set_tilt_sub_ = nh.subscribe<std_msgs::Float32>("set_tilt", 1, &SiarArmROS::setTiltReceived, dynamic_cast<SiarArmROS *>(this));
      arm_cmd_pub_ = nh.advertise<siar_driver::SiarArmCommand>("arm_cmd", 1);
      arm_clear_status_pub_ = nh.advertise<std_msgs::Bool>("arm_clear_status", 1);
      arm_torque_pub_ = nh.advertise<std_msgs::UInt8>("arm_torque", 1);
      clearStatusAndActivateMotors();
    }

    // Clear status and activate the motors of the arm
    SiarArm::load_data(mot_file, ang_file);
    ros::Rate r(loop_rate_);
    
    last_status_ = curr_status_ = NOT_INITIALIZED;
    s_.start();

    s_.registerGoalCallback(boost::bind(&SiarArmROSMBZirc::goalCb, this));
    
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
      
      if (enable_server_) { // Enable server got from parameter in siar_arm_ros.hpp Defaults to true
	      manageServer();
      }
      
      if (enable_marker_) {
	      arm_marker_pub_.publish(getARMMarkerArray());
      }
    }
    s_.shutdown();

  }
  
  virtual void manageServer() {
    if ( (curr_status_ == PAN_AND_TILT || curr_status_ == SiarArmROS::NAVIGATION) &&
         (move_pan_ || move_tilt_)
      ) {
      movePanTilt(pan_rate_/loop_rate_, tilt_rate_/loop_rate_);
    } else {
      pan_rate_ = tilt_rate_ = 0.0;
    }

    if (status_cont > 50) {
      clearStatusAndActivateMotors();
      status_cont = 0;
    } else {
      status_cont++;
    }

    if (curr_status_ == MOVING) {
      curr_status_ = PAN_AND_TILT;
      ROS_INFO("Moving status not considered in MBZIRC");
	    
    }
  }
  
  virtual void goalCb() {
    ROS_INFO("Goal not supported in MBZIRC");
  }
  
  void statusCb(const siar_driver::SiarStatus::ConstPtr& new_status) {
    curr_siar_status_ = *new_status;
    if (curr_status_ == NOT_INITIALIZED) {
      curr_cmd_ = curr_siar_status_.herculex_position;
      last_status_= curr_status_ = PAN_AND_TILT;
      
      std::ostringstream os;

      bool non_zero = true;
      for (auto x:curr_cmd_) {
        os << (int)x << " ";
        non_zero &= x == 0;
      }
      if (!non_zero) {
        ROS_INFO("Received first status. Assuming PAN_AND_TILT. Last command: %s", os.str().c_str());
      } else {
        ROS_INFO("Received first status but is zero. Still in NOT_INITIALIZED");
        last_status_= curr_status_ = NOT_INITIALIZED;
      }
    }
  }

  protected:
  int status_cont;

  
};
#endif /* _SIAR_ARM_H_ */

