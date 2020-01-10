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

#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

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

class SiarArmROS:public SiarArm 
{
  public:
  Server s_;
  double pan_rate_, tilt_rate_, loop_rate_;
  double max_pan_rate_, max_tilt_rate_;
  siar_driver::SiarStatus curr_siar_status_;
  boost::array<int16_t, 5> curr_cmd_;
  
  ros::Subscriber arm_pan_sub_, arm_tilt_sub_, siar_status_sub_;
  ros::Subscriber set_pan_sub_, set_tilt_sub_;
  ros::Publisher arm_cmd_pub_, arm_clear_status_pub_, arm_torque_pub_, arm_marker_pub_; // Sends arm commands to SIAR Driver node
  ros::Publisher arm_ang_rad_pan_pub_, arm_ang_rad_tilt_pub_;
  int pan_joint_, tilt_joint_;
  bool move_pan_, move_tilt_;
  int seq_cmd_;
  std::vector<siar_driver::SiarArmCommand> curr_traj_;
  std::string curr_traj_name_;
  std::string resource_folder_;
  int max_joint_dist_;
  double timeout_, period_;
  bool enable_server_, enable_marker_;
  double threshold_mov_arm {0.01};
  float arm_ang_rad_tilt {0.0};
  float arm_ang_rad_pan {0.0};


  int cmd_time_pan_tilt;
  
  // For marker array
  std::string frame_id;
  tf::TransformBroadcaster tfb;

  std::string mot_file, ang_file;
  
  enum ArmNodeStatus {
    NOT_INITIALIZED, PARKED, PAN_AND_TILT, NAVIGATION, MOVING
  };
  
  ArmNodeStatus curr_status_, last_status_, initial_status_;
  siar_arm::armServosMoveActionFeedback::_feedback_type curr_feed_;
  
  
  SiarArmROS(ros::NodeHandle &nh, ros::NodeHandle &pnh):SiarArm(),s_(nh, "move_arm", false),seq_cmd_(0)
  {
    timeout_ = -1.0;
    cmd_time_pan_tilt = 200;

    move_pan_ = move_tilt_ = false;
    
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
    
    if (!pnh.getParam("frame_id", frame_id)) {
      frame_id = "siar/arm";
    }
    if (!pnh.getParam("enable_server", enable_server_)) {
     enable_server_ = true; 
    }
    if (!pnh.getParam("enable_marker", enable_marker_)) {
      enable_marker_ = true;
    }
    
  }

  virtual void start() {
    ros::NodeHandle nh;
    // Configure communications depending on the flags
    if (enable_marker_) {
      arm_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("arm_marker", 1);
    }

    siar_status_sub_ = nh.subscribe<siar_driver::SiarStatus>("siar_node/siar_status", 1, &SiarArmROS::statusCb, this);
     if (enable_server_) {

      arm_pan_sub_ = nh.subscribe<std_msgs::Float32>("arm_pan", 1, &SiarArmROS::armPanReceived, this);
      arm_tilt_sub_ = nh.subscribe<std_msgs::Float32>("arm_tilt", 1, &SiarArmROS::armTiltReceived, this);
      set_pan_sub_ = nh.subscribe<std_msgs::Float32>("set_pan", 1, &SiarArmROS::setPanReceived, this);
      set_tilt_sub_ = nh.subscribe<std_msgs::Float32>("set_tilt", 1, &SiarArmROS::setTiltReceived, this);

      arm_cmd_pub_ = nh.advertise<siar_driver::SiarArmCommand>("arm_cmd", 1);
      arm_clear_status_pub_ = nh.advertise<std_msgs::Bool>("arm_clear_status", 1);
      arm_torque_pub_ = nh.advertise<std_msgs::UInt8>("arm_torque", 1);
      arm_ang_rad_pan_pub_ = nh.advertise<std_msgs::Float32>("arm_ang_rad_pan", 1);
      arm_ang_rad_tilt_pub_ = nh.advertise<std_msgs::Float32>("arm_ang_rad_tilt", 1);

      clearStatusAndActivateMotors();
    }


    // Clear status and activate the motors of the arm
    SiarArm::load_data(mot_file, ang_file);
    ros::Rate r(loop_rate_);
    
    last_status_ = curr_status_ = NOT_INITIALIZED;
    s_.start();

    s_.registerGoalCallback(boost::bind(&SiarArmROS::goalCb, this));
    
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
    if ( (curr_status_ == PAN_AND_TILT || curr_status_ == SiarArmROS::NAVIGATION) && (move_pan_ || move_tilt_)) 
    {
      movePanTilt(pan_rate_/loop_rate_, tilt_rate_/loop_rate_);
    } 
    else {
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
        if (curr_feed_.curr_mov == curr_traj_.size()) 
        {
          // Final waypoint has been reached --> send the result
          siar_arm::armServosMoveActionResult::_result_type result;
          result.executed = true;
          result.position_servos_final = curr_siar_status_.herculex_position;
          std::ostringstream message;
          message <<"SiarArm::loop --> Arm reached the final destination. New state: ";
          
          if (curr_traj_name_ == "navigation") {
            message << "NAVIGATION";
            curr_status_ = NAVIGATION;
          } 
          else if (curr_traj_name_ == "park") 
          {
            message << "PARKED";
            curr_status_ = PARKED;
          } 
          else 
          {
            message << "PAN_AND_TILT";
            curr_status_ = SiarArmROS::PAN_AND_TILT;
          }
          s_.setSucceeded(result, message.str());
        } 
        else 
        {
          s_.publishFeedback(curr_feed_);
          publishCmd(curr_traj_[curr_feed_.curr_mov]);
          curr_cmd_ = curr_traj_[curr_feed_.curr_mov].joint_values;
        }
      } 
      else if (timeout_ < 0.0) 
      {
        ROS_ERROR("SiarArm::loop --> Timeout detected while following a trajectory. Returning to state: %d", (int)last_status_);
        siar_arm::armServosMoveActionResult::_result_type result;
        result.executed = false;
        result.position_servos_final = curr_siar_status_.herculex_position;
        std::ostringstream message;
        message <<"SiarArm::loop --> Could not reach the final destination. Returning to previous state.";
        s_.setSucceeded(result, message.str());
        curr_status_ = last_status_;
        clearStatusAndActivateMotors();
      }
    }
  }
  
  void armPanReceived(const std_msgs::Float32ConstPtr &data) {
    if (fabs(data->data) > threshold_mov_arm)
      move_pan_ = true; 
    
    else 
      move_pan_ = false;
    
    pan_rate_ = functions::saturate((double)data->data, -1.0, 1.0);
    pan_rate_ *= max_pan_rate_;
    cmd_time_pan_tilt = 100;
  }
  
  void armTiltReceived(const std_msgs::Float32ConstPtr &data) {
     if (fabs(data->data) > threshold_mov_arm)
      move_tilt_ = true; 
    else 
      move_tilt_ = false;
    
    tilt_rate_ = functions::saturate((double)data->data, -1.0, 1.0);
    tilt_rate_ *= max_tilt_rate_;
    cmd_time_pan_tilt = 100;
  }

  void setPanReceived(const std_msgs::Float32ConstPtr &pan) {
    angle_type angles;
    raw_type motors;
	  motor2rad(curr_siar_status_.herculex_position, angles);
    angles[pan_joint_] = pan->data;

    if (rad2motor(angles, motors, pan_joint_)) {
      ROS_INFO("Setting the pan to: %f", pan->data); //, functions::printVector(angles).c_str());
      pan_rate_ = 0.0;
      tilt_rate_ = 0.0;
      curr_cmd_ = motors;
      move_pan_ = true;
      cmd_time_pan_tilt = 100;
    }
  }

  void setTiltReceived(const std_msgs::Float32ConstPtr &tilt) {
    angle_type angles;
    raw_type motors;
	  motor2rad(curr_siar_status_.herculex_position, angles);
    angles[tilt_joint_] = tilt->data;
    if (rad2motor(angles, motors)) {
      ROS_INFO("Setting the tilt to: %f", tilt->data);// functions::printVector(angles).c_str());
      pan_rate_ = 0.0;
      tilt_rate_ = 0.0;
      curr_cmd_ = motors;
      move_tilt_ = true;
      cmd_time_pan_tilt = 100;
    }
  }
  
  virtual void goalCb() {
    auto goal = s_.acceptNewGoal();
    // TODO: Implement it!
    if (curr_status_ == PAN_AND_TILT || curr_status_ == PARKED || curr_status_==NAVIGATION) {
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
      
      if (goal->mov_name == "navigation") {
        // We have to add forwards or backwards depending on the siar status
        if (curr_siar_status_.reverse) {
          os << "_back";
        } else {
          os << "_front";
        }
      }
      
      os << ".txt";
      curr_traj_.clear();
      curr_traj_name_ =goal->mov_name;
      ROS_INFO("SiarArmROS::goalCb. Command received: %s \t Trying to load file: %s", curr_traj_name_.c_str(), os.str().c_str());

      if (functions::getMatrixFromFile(os.str(), mat)) {
        ROS_INFO("Load succeded.");
	      curr_status_ = MOVING;
        for (size_t i = 0; i < mat.size(); i++) 
        {
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
      } 
      else 
      {
        // Arm not ready or already moving to a destination --> cancel
        siar_arm::armServosMoveActionResult::_result_type result;
        result.executed = false;
        result.position_servos_final = curr_siar_status_.herculex_position;
        s_.setAborted(result, "Trajectory resource not found");
        curr_cmd_ = curr_siar_status_.herculex_position;
      }
      
    }
    else 
    {
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

    publishRadStateArm();

  }
  
  std_msgs::Header getHeader(int seq = 0) {
    std_msgs::Header h;
    h.frame_id = "siar/arm_link";
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
    publishRadStateArm();

  }
  
  void publishRadStateArm()
  {
    angle_type angles;
    raw_type motors;
    std_msgs::Float32 arm_ang_rad_tilt_msg , arm_ang_rad_pan_msg;
	  motor2rad(curr_siar_status_.herculex_position, angles);
    arm_ang_rad_tilt_msg.data = angles[3];
    arm_ang_rad_pan_msg.data = angles[4];
    arm_ang_rad_pan_pub_.publish(arm_ang_rad_pan_msg);
    arm_ang_rad_tilt_pub_.publish(arm_ang_rad_tilt_msg);

    // ROS_INFO("pan= %f  ,  tilt= %f",arm_ang_rad_tilt_msg.data,arm_ang_rad_pan_msg.data);

  }

  bool clearStatusAndActivateMotors() const {
    bool ret_val = true;

    std_msgs::UInt8 msg2;
    msg2.data = 0; // 1 for turning off the motors
    arm_torque_pub_.publish(msg2);
    usleep(50000);

    std_msgs::Bool msg;
    msg.data = 1;
    arm_clear_status_pub_.publish(msg);
    usleep(50000);

    ROS_INFO("Clearing status of the herculex and turning on the torque");
    msg2.data = 1; // 1 for turning on the motors
    arm_torque_pub_.publish(msg2);
    usleep(50000);
    
    msg.data = 1;
    arm_clear_status_pub_.publish(msg);
    usleep(50000);
    msg2.data = 2; // 2 for turning on the motors
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
  
  visualization_msgs::MarkerArray getARMMarkerArray() {
    int id = 0;
    visualization_msgs::MarkerArray model;
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;
    angle_type angles;
    motor2rad(curr_siar_status_.herculex_position, angles);
    
    // First and second rotations
    // Emit the first transform: siar_arm_1_2
    tf::Quaternion q;
    q.setRPY(0, angles[1], angles[0]);
    tf::StampedTransform stf;
    stf.stamp_ = ros::Time::now();
    stf.frame_id_ = frame_id;
    stf.child_frame_id_ = "siar/arm_rotation_1_2";
    stf.setRotation(q);
    tfb.sendTransform(stf);
    
    // Add First Link	
    marker.header.frame_id = stf.child_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "siar/arm";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = length[1] * 0.5;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 0.70711;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.70711;
    marker.pose.orientation.z = 0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = length[1];
    marker.color.a = 1.0; 
    marker.color.r = 75.0/255.0;
    marker.color.g = 75.0/255.0;
    marker.color.b = 75.0/255.0;
    marker.points.clear();
    model.markers.push_back(marker);
    
    
    
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_link_1";
    tf::Vector3 v(length[1], 0, 0);
    stf.setIdentity();
    stf.setOrigin(v);
    tfb.sendTransform(stf);
    
    // Rotation 3
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_rotation_3";
    stf.setIdentity();
    q.setRPY(0, angles[2], 0);
    stf.setRotation(q);
    tfb.sendTransform(stf);
    
    // Next link
    marker.scale.z = length[2];
    marker.header.frame_id = stf.child_frame_id_;
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.pose.position.x = length[2] * 0.5;
    marker.id = id++;
    model.markers.push_back(marker);
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_link_2";
    v.setValue(length[2], 0, 0);
    stf.setIdentity();
    stf.setOrigin(v);
    tfb.sendTransform(stf);
    
    // Rotation 4
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_rotation_4";
    stf.setIdentity();
    q.setRPY(0, angles[3], 0);
    stf.setRotation(q);
    tfb.sendTransform(stf);
    
    // Link 4
    marker.scale.z = length[3];
    marker.header.frame_id = stf.child_frame_id_;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    marker.pose.position.x = length[3] * 0.5;
    marker.id = id++;
    model.markers.push_back(marker);
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_link_3";
    v.setValue(length[3], 0, 0);
    stf.setIdentity();
    stf.setOrigin(v);
    tfb.sendTransform(stf);
    
          
    // PreRotation 5
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_prerotation_5";
    stf.setIdentity();
    v.setValue(0.02,0,0);
    q.setRPY(0, M_PI/2, 0);
    stf.setRotation(q);
    stf.setOrigin(v);
    tfb.sendTransform(stf);
          
    // Rotation 5
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_rotation_5";
    stf.setIdentity();
    q.setRPY(0, 0, angles[4]);
    stf.setRotation(q);
    tfb.sendTransform(stf);
    
    // Link 5
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.z = length[4];
    marker.header.frame_id = stf.child_frame_id_;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.8;
    marker.pose.position.x = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.y = 0.0;
    marker.id = id++;
    model.markers.push_back(marker);

    // Final transforms
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_final";
    v.setValue(length[4], 0, 0);
    stf.setIdentity();
    stf.setOrigin(v);
    tfb.sendTransform(stf);

    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_camera";
    stf.setIdentity();
    q.setRPY(3.1415, 0, 1.57);
    stf.setRotation(q);
    tfb.sendTransform(stf);
    
    return model;
  }
    
};
#endif /* _SIAR_ARM_H_ */

