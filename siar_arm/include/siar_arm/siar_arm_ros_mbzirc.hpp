#ifndef _SIAR_ARM_ROS_MBZIRC_HPP_
#define _SIAR_ARM_ROS_MBZIRC_HPP_

#include "siar_arm/siar_arm_ros.hpp"

class SiarArmROSMBZirc:public SiarArmROS {
  public:
  SiarArmROSMBZirc(ros::NodeHandle &nh, ros::NodeHandle &pnh):SiarArmROS(nh, pnh) {
   
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
      
      if (enable_server_) {
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

    if (curr_status_ == MOVING) {
      timeout_ -= period_;
      siar_driver::SiarArmCommand curr_com = curr_traj_[curr_feed_.curr_mov];
      auto v = curr_com.joint_values;
      auto v2 = curr_siar_status_.herculex_position;
      bool arrived = true;
      
      for (int i = 0; i < v.size() && arrived; i++) {
	    arrived = fabs(v[i] - v2[i]) < max_joint_dist_;
      }
      
      
      if (arrived) {
	    curr_feed_.curr_mov++;
	    if (curr_feed_.curr_mov == curr_traj_.size()) {
            // Final waypoint has been reached --> send the result
            siar_arm::armServosMoveActionResult::_result_type result;
            result.executed = true;
            result.position_servos_final = curr_siar_status_.herculex_position;
            std::ostringstream message;
            message <<"SiarArm::loop --> Arm reached the final destination. New state: ";
            
            if (curr_traj_name_ == "park") {
                message << "PARKED";
                curr_status_ = PARKED;
            } else {
                message << "PAN_AND_TILT";
                curr_status_ = SiarArmROS::PAN_AND_TILT;
            }
            s_.setSucceeded(result, message.str());
            } else {
                s_.publishFeedback(curr_feed_);
                publishCmd(curr_traj_[curr_feed_.curr_mov]);
                curr_cmd_ = curr_traj_[curr_feed_.curr_mov].joint_values;
            }
      } else if (timeout_ < 0.0) {
	    ROS_ERROR("SiarArm::loop --> Timeout detected while following a trajectory. Returning to state: %d", (int)last_status_);
	    siar_arm::armServosMoveActionResult::_result_type result;
	    result.executed = false;
	    result.position_servos_final = curr_siar_status_.herculex_position;
	    std::ostringstream message;
	    message <<"SiarArm::loop --> Could not final destination. Returning to previous state.";
	    s_.setSucceeded(result, message.str());
	    curr_status_ = last_status_;
	    clearStatusAndActivateMotors();
      }
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
  
};
#endif /* _SIAR_ARM_H_ */

