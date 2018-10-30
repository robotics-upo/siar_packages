#ifndef _SIAR_ARM_ROS_HPP_
#define _SIAR_ARM_ROS_HPP_


#include "math.h"
#include <functions/linear_interpolator.hpp>
#include <string>
#include <queue>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "siar_driver/siar_functions.hpp"
#include "siar_driver/SiarStatus.h"
#include "siar_arm/siar_arm.hpp"
#include "siar_arm/armServosMoveAction.h"
#include "actionlib/server/simple_action_server.h"
#include "siar_driver/SiarArmCommand.h"

#include <exception>

typedef actionlib::SimpleActionServer<siar_arm::armServosMoveAction> Server;

class SiarArmException:public std::runtime_error {
public:
  SiarArmException(const std::string &s):runtime_error("") {
    desc = s;
  }
  
  const char * what () const throw () {
     std::ostringstream os;
     os << "SiarArm Exception. Content: %s" << desc;
     
     return os.str().c_str();
   }
   
   std::string desc;
};

class SiarArmROS:public SiarArm {
  public:
  Server s_;
  double pan_rate_, tilt_rate_, loop_rate_;
  double max_pan_rate_, max_tilt_rate_;
  siar_driver::SiarStatus curr_siar_status_;
  ros::Subscriber arm_pan_sub_, arm_tilt_sub_;
  ros::Publisher arm_cmd_pub_; // Sends arm commands to SIAR Driver node
  int pan_joint_, tilt_joint_;
  int seq_cmd_;
  std::vector<siar_driver::SiarArmCommand> curr_traj_;
  std::string curr_traj_name_;
  std::string resource_folder_;
  int max_joint_dist_;
  
  enum ArmNodeStatus {
    NOT_INITIALIZED, PARKED, PAN_AND_TILT, INSPECTION, MOVING
  };
  
  ArmNodeStatus curr_status_;
  siar_arm::armServosMoveActionFeedback::_feedback_type curr_feed_;
  
  
  SiarArmROS(ros::NodeHandle &nh, ros::NodeHandle &pnh):SiarArm(),s_(nh, "move_arm", false),seq_cmd_(0){
    std::string mot_file, ang_file;
    
    s_.registerGoalCallback(boost::bind(&SiarArmROS::goalCb, this));
    
    if (!pnh.getParam("motor_file", mot_file) || !pnh.getParam("angular_file", ang_file)) {
      throw SiarArmException("You should give the motor_file and angular_file parameters");
    }
    if (!pnh.getParam("loop_rate", loop_rate_)) {
      loop_rate_ = 10;
    }
    if (!pnh.getParam("max_pan_rate", max_pan_rate_)) {
      max_pan_rate_ = 0.1;
    }
    if (!pnh.getParam("max_tilt_rate", max_tilt_rate_)) {
      max_tilt_rate_ = 0.1;
    }
    if (!pnh.getParam("pan_joint", pan_joint_)) {
      pan_joint_ = 3;
    }
    if (!pnh.getParam("tilt_joint", tilt_joint_)) {
      tilt_joint_ = 3;
    }
    if (!pnh.getParam("resource_folder", resource_folder_)) {
      resource_folder_ = "/home/siar";
    }
    if (!pnh.getParam("max_joint_distance", max_joint_dist_)) {
      
      max_joint_dist_ = 20;
    }
    
    arm_pan_sub_ = nh.subscribe<std_msgs::Float32>("/arm_pan", 1, &SiarArmROS::armPanReceived, this);
    arm_tilt_sub_ = nh.subscribe<std_msgs::Float32>("/arm_tilt", 1, &SiarArmROS::armTiltReceived, this);
    arm_cmd_pub_ = nh.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 1);
    
    SiarArm::load_data(mot_file, ang_file);
    ros::Rate r(loop_rate_);
    
    curr_status_ = NOT_INITIALIZED;
    s_.start();
    
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
      
      if (curr_status_ == PAN_AND_TILT || curr_status_ == SiarArmROS::INSPECTION) {
	movePanTilt(pan_rate_/loop_rate_, tilt_rate_/loop_rate_);
      } else {
	pan_rate_ = tilt_rate_ = 0.0;
      }
      if (curr_status_ == MOVING) {
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
	    message <<"Arm reached the final destination. New state: ";
	    
	    if (curr_traj_name_ == "pan_tilt") {
	      message << "PAN AND TILT";
	      curr_status_ = PAN_AND_TILT;
	    } else if (curr_traj_name_ == "park") {
	      message << "PARKED";
	      curr_status_ = PARKED;
	    } else {
	      message << "INSPECTION";
	      curr_status_ = SiarArmROS::INSPECTION;
	    }
	    s_.setSucceeded(result, message.str());
	  } else {
	    s_.publishFeedback(curr_feed_);
	    publishCmd(curr_traj_[curr_feed_.n_movs]);
	  }
	  
	}
      }
    }
    s_.shutdown();
  }
  
  void armPanReceived(const std_msgs::Float32ConstPtr &data) {
    pan_rate_ = functions::saturate((double)data->data, -1.0, 1.0);
    pan_rate_ *= max_pan_rate_;
  }
  
  void armTiltReceived(const std_msgs::Float32ConstPtr &data) {
    tilt_rate_ = functions::saturate((double)data->data, -1.0, 1.0);
    tilt_rate_ *= max_tilt_rate_;
  }
  
  void goalCb() {
    auto goal = s_.acceptNewGoal();
    // TODO: Implement it!
    if (curr_status_ == PAN_AND_TILT || curr_status_ == PARKED || curr_status_==INSPECTION) {
      std::vector< std::vector<double> > mat;
      std::ostringstream os;
      os << resource_folder_ << "/";
      
      switch (curr_status_) {
	case PARKED: 
	  os << "park2";
	  break;
	  
	case PAN_AND_TILT:
	  os << "pan_tilt2";
	  break;
      }
      bool append = false;
      if (goal->mov_name != "park" && goal->mov_name != "pan_tilt" && curr_status_!=INSPECTION) {
	os << "navigation";
	append = true;
      } else {
	os << goal->mov_name;
      }
      
      os << ".txt";
      curr_traj_.clear();
      curr_traj_name_ =goal->mov_name;
      ROS_INFO("SiarArmROS::goalCb. Command received: %s \t Trying to load file: %s", curr_traj_name_.c_str(), os.str().c_str());
      if (functions::getMatrixFromFile(os.str(), mat)) {
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
	
	if (append && curr_traj_name_ != "navigation") {
	  std::vector< std::vector<double> > mat;
	  std::ostringstream os;
	  
	  os << resource_folder_ << "/" << curr_traj_name_ << ".txt";
	  ROS_INFO("SiarArmROS::goalCb. Appending file: %s", os.str().c_str());
	  if (functions::getMatrixFromFile(os.str(), mat)) {
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
	  } else {
	    ROS_ERROR("Could not load the appended trajectory");
	  }
	}
        curr_feed_.n_movs = curr_traj_.size();
        curr_feed_.curr_mov = 0;
      
        publishCmd(curr_traj_[curr_feed_.curr_mov]);
      
        //TODO: Reverse if necessary
      
        s_.publishFeedback(curr_feed_);
	
      } else {
        // Arm not ready or already moving to a destination --> cancel
	siar_arm::armServosMoveActionResult::_result_type result;
	result.executed = false;
	result.position_servos_final = curr_siar_status_.herculex_position;
	s_.setAborted(result, "Trajectory resource not found");
      }
      
    } else {
      // Arm not ready or already moving to a destination --> cancel
      siar_arm::armServosMoveActionResult::_result_type result;
      result.executed = false;
      result.position_servos_final = curr_siar_status_.herculex_position;
      s_.setAborted(result, "Arm not ready");
//       s_.
    }
  }
  
  void statusCb(const siar_driver::SiarStatus::ConstPtr& new_status) {
    curr_siar_status_ = *new_status;
    if (curr_status_ == NOT_INITIALIZED) {
      curr_status_ = PARKED;
    }
  }
  
  void movePanTilt(double pan_angle, double tilt_angle) {
    auto curr_pos = curr_siar_status_.herculex_position;
    
    
    curr_pos[pan_joint_] += pan_angle;
    curr_pos[tilt_joint_] += tilt_angle;
    correctJointLimits(curr_pos);
    siar_driver::SiarArmCommand cmd;
    cmd.header = getHeader(seq_cmd_++);
    cmd.joint_values = curr_pos;
    cmd.command_time = 100;
      
    arm_cmd_pub_.publish(cmd);
  }
  
  std_msgs::Header getHeader(int seq = 0) {
    std_msgs::Header h;
    h.frame_id = "siar_arm_link";
    h.seq = seq;
    h.stamp = ros::Time::now();
    return h;
  }
  
  void publishCmd(siar_driver::SiarArmCommand &cmd) {
    cmd.header = getHeader(seq_cmd_++);
    arm_cmd_pub_.publish(cmd);
  }
    
};
#endif /* _SIAR_ARM_H_ */

