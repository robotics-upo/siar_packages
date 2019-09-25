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
    auto goal = s_.acceptNewGoal();

    if (curr_status_ == PAN_AND_TILT || curr_status_ == PARKED || curr_status_== NAVIGATION) {
      std::vector< std::vector<double> > mat;
      std::ostringstream os;
      os << resource_folder_ << "/" << goal->mov_name;
      
      last_status_ = curr_status_;
      
      if (curr_status_== PARKED && goal->mov_name != "pan_tilt") { // From PARKED position we can only reach PAN_AND_TILT
        // Arm not ready or already moving to a destination --> cancel
        siar_arm::armServosMoveActionResult::_result_type result;
        result.executed = false;
        result.position_servos_final = curr_siar_status_.herculex_position;
        s_.setAborted(result, "From park status we can only reach pan_tilt state.");
        curr_cmd_ = curr_siar_status_.herculex_position;
        ROS_ERROR("Could not do goal: %s. From park status we can only reach pan_tilt state.", goal->mov_name.c_str());
        return;
      }
      
      if (curr_status_ == PAN_AND_TILT && goal->mov_name != "park") {
        // Arm not ready or already moving to a destination --> cancel
        siar_arm::armServosMoveActionResult::_result_type result;
        result.executed = false;
        result.position_servos_final = curr_siar_status_.herculex_position;
        s_.setAborted(result, "From park status we can only reach pan_tilt state.");
        curr_cmd_ = curr_siar_status_.herculex_position;
        ROS_ERROR("Could not do goal: %s. From park status we can only reach pan_tilt state.", goal->mov_name.c_str());
        return;
      }
      
      os << "_mbzirc.txt";
      curr_traj_.clear();
      curr_traj_name_ =goal->mov_name;
      ROS_INFO("SiarArmROS::goalCb. Command received: %s \t Trying to load file: %s", curr_traj_name_.c_str(), os.str().c_str());
      if (functions::getMatrixFromFile(os.str(), mat)) {
        ROS_INFO("Load succeded.");
	    curr_status_ = MOVING;
        for (size_t i = 0; i < mat.size(); i++) {
        siar_driver::SiarArmCommand curr_cmd;
        if (mat[i].size() < 6)
            continue;
        for (int j = 0; j < 5; j++) {
            curr_cmd.joint_values[j] = mat[i][j];
        }
        curr_cmd.command_time = mat[i][5];
            
        curr_traj_.push_back(curr_cmd);
        }
	
        curr_feed_.n_movs = curr_traj_.size();
        curr_feed_.curr_mov = 0;
      
        publishCmd(curr_traj_[curr_feed_.curr_mov]);
	    curr_cmd_ = curr_traj_[curr_feed_.curr_mov].joint_values;
      
        s_.publishFeedback(curr_feed_);
      } else {
        // Arm not ready or already moving to a destination --> cancel
        siar_arm::armServosMoveActionResult::_result_type result;
        result.executed = false;
        result.position_servos_final = curr_siar_status_.herculex_position;
        s_.setAborted(result, "Trajectory resource not found");
        curr_cmd_ = curr_siar_status_.herculex_position;
      }
      
    } else {
      // Arm not ready or already moving to a destination --> cancel
      siar_arm::armServosMoveActionResult::_result_type result;
      result.executed = false;
      result.position_servos_final = curr_siar_status_.herculex_position;
      s_.setAborted(result, "Arm not ready");
    }
  }
  
  void statusCb(const siar_driver::SiarStatus::ConstPtr& new_status) {
    curr_siar_status_ = *new_status;
    if (curr_status_ == NOT_INITIALIZED) {
      curr_cmd_ = curr_siar_status_.herculex_position;
      last_status_= curr_status_ = initial_status_;
      if (initial_status_ == PARKED) {
        ROS_INFO("Received first status. Assuming park STATE");
      } else if (initial_status_ == PAN_AND_TILT) {
        ROS_INFO("Received first status. Assuming PAN_AND_TILT");
      } 
    }
  }
  
};
#endif /* _SIAR_ARM_H_ */

