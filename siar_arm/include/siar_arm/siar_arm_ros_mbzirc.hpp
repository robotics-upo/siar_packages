#ifndef _SIAR_ARM_ROS_MBZIRC_HPP_
#define _SIAR_ARM_ROS_MBZIRC_HPP_

#include "siar_arm/siar_arm_ros.hpp"

class SiarArmROSMBZirc:public SiarArmROS {
  public:
  SiarArmROSMBZirc(ros::NodeHandle &nh, ros::NodeHandle &pnh):SiarArmROS(nh, pnh),status_cont(0) {
   
  }

   virtual void start() {
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

    if (status_cont > 10) {
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
      
      ROS_INFO("Received first status. Assuming PAN_AND_TILT");
    }
  }

  protected:
  int status_cont;

  
};
#endif /* _SIAR_ARM_H_ */

