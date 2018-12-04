#ifndef _SIAR_ARM_ROS_HPP_
#define _SIAR_ARM_ROS_HPP_


#include "math.h"
#include <functions/linear_interpolator.hpp>
#include <functions/functions.h>
#include <string>
#include <queue>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

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
  boost::array<int16_t, 5> curr_cmd_;
  
  ros::Subscriber arm_pan_sub_, arm_tilt_sub_, siar_status_sub_;
  ros::Publisher arm_cmd_pub_, arm_clear_status_pub_, arm_torque_pub_; // Sends arm commands to SIAR Driver node
  int pan_joint_, tilt_joint_;
  bool move_pan_, move_tilt_;
  int seq_cmd_;
  std::vector<siar_driver::SiarArmCommand> curr_traj_;
  std::string curr_traj_name_;
  std::string resource_folder_;
  int max_joint_dist_;
  double timeout_, period_;
  
  enum ArmNodeStatus {
    NOT_INITIALIZED, PARKED, PAN_AND_TILT, INSPECTION, MOVING
  };
  
  ArmNodeStatus curr_status_, last_status_, initial_status_;
  siar_arm::armServosMoveActionFeedback::_feedback_type curr_feed_;
  
  
  SiarArmROS(ros::NodeHandle &nh, ros::NodeHandle &pnh):SiarArm(),s_(nh, "move_arm", false),seq_cmd_(0){
    timeout_ = -1.0;
    std::string mot_file, ang_file;
    move_pan_ = move_tilt_ = false;

    s_.registerGoalCallback(boost::bind(&SiarArmROS::goalCb, this));
    
    if (!pnh.getParam("motor_file", mot_file) || !pnh.getParam("angular_file", ang_file)) {
      throw SiarArmException("You should give the motor_file and angular_file parameters");
    }
    if (!pnh.getParam("loop_rate", loop_rate_)) {
      loop_rate_ = 10;
    }
    period_ = 1/loop_rate_;
    
    if (!pnh.getParam("max_pan_rate", max_pan_rate_)) {
      max_pan_rate_ = 0.1;
    }
    if (!pnh.getParam("max_tilt_rate", max_tilt_rate_)) {
      max_tilt_rate_ = 0.1;
    }
    if (!pnh.getParam("pan_joint", pan_joint_)) {
      pan_joint_ = 4;
    }
    if (!pnh.getParam("tilt_joint", tilt_joint_)) {
      tilt_joint_ = 3;
    }
    if (!pnh.getParam("resource_folder", resource_folder_)) {
      resource_folder_ = "/home/siar";
    }
    if (!pnh.getParam("max_joint_distance", max_joint_dist_)) {
      
      max_joint_dist_ = 100;
    }
    int i;
    if (!pnh.getParam("initial_status", i)) {
      initial_status_ = PARKED;
    } else {
      initial_status_ = (ArmNodeStatus)i;
    }
    
    arm_pan_sub_ = nh.subscribe<std_msgs::Float32>("/arm_pan", 1, &SiarArmROS::armPanReceived, this);
    arm_tilt_sub_ = nh.subscribe<std_msgs::Float32>("/arm_tilt", 1, &SiarArmROS::armTiltReceived, this);
    siar_status_sub_ = nh.subscribe<siar_driver::SiarStatus>("/siar_status", 1, &SiarArmROS::statusCb, this);
    arm_cmd_pub_ = nh.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 1);
    arm_clear_status_pub_ = nh.advertise<std_msgs::Bool>("/arm_clear_status", 1);
    arm_torque_pub_ = nh.advertise<std_msgs::UInt8>("/arm_torque", 1);

    
    // Clear status and activate the motors of the arm
    clearStatusAndActivateMotors();
    
    SiarArm::load_data(mot_file, ang_file);
    ros::Rate r(loop_rate_);
    
    last_status_ = curr_status_ = NOT_INITIALIZED;
    s_.start();
    
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
      
      if ( (curr_status_ == PAN_AND_TILT || curr_status_ == SiarArmROS::INSPECTION) &&
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
	  if (i == 3)
		continue; // Quick fix for the troublesome joint
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
    s_.shutdown();
  }
  
  void armPanReceived(const std_msgs::Float32ConstPtr &data) {
    if (fabs(data->data) > 0.3)
      move_pan_ = true; 
    
    else 
      move_pan_ = false;
    pan_rate_ = functions::saturate((double)data->data, -1.0, 1.0);
    pan_rate_ *= max_pan_rate_;
  }
  
  void armTiltReceived(const std_msgs::Float32ConstPtr &data) {
     if (fabs(data->data) > 0.3)
      move_tilt_ = true; 
    
    else 
      move_tilt_ = false;
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
      
      last_status_ = curr_status_;
      
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
	curr_cmd_ = curr_traj_[curr_feed_.curr_mov].joint_values;
      
        //TODO: Reverse if necessary
      
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
//       s_.
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
  
  void movePanTilt(double pan_angle, double tilt_angle) {
    curr_cmd_[pan_joint_] += pan_angle;
    curr_cmd_[tilt_joint_] += tilt_angle;
    correctJointLimits(curr_cmd_);
    siar_driver::SiarArmCommand cmd;
    cmd.header = getHeader(seq_cmd_++);
    cmd.joint_values = curr_cmd_;
    cmd.command_time = 100;
    
    
    std::cout << "movePanTilt-->Moving: " << pan_angle << " and " << tilt_angle << " Objective: " << commandToString(cmd) << std::endl;
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
    correctJointLimits(cmd.joint_values);
    ROS_INFO("Publishing command: %s", commandToString(cmd).c_str());
    timeout_ = cmd.command_time / 50.0;
    cmd.header = getHeader(seq_cmd_++);
    arm_cmd_pub_.publish(cmd);
  }
  
  bool clearStatusAndActivateMotors() const {
    bool ret_val = true;
    ROS_INFO("Clearing status of the herculex and turning on the torque");
    std_msgs::UInt8 msg2;
    msg2.data = 2; // 2 for turning on the motors
    arm_torque_pub_.publish(msg2);
    usleep(50000);
    std_msgs::Bool msg;
    msg.data = 1;
    arm_clear_status_pub_.publish(msg);
    usleep(50000);
    arm_torque_pub_.publish(msg2);
    
    return ret_val;
  }
  
  std::string commandToString(const siar_driver::SiarArmCommand &msg) {
    std::ostringstream os;
    for (auto i:msg.joint_values) {
      os << i << " ";
    }
    
    os << (int)msg.command_time;
    return os.str();
  }
    
};
#endif /* _SIAR_ARM_H_ */

