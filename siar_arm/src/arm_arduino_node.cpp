#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <functions/MedianFilter.h>

using namespace std;

#include "siar_arm/arm_arduino.hpp"

// Node variables
int32_t curr_pan = 0;
int32_t  curr_tilt = 0;
bool pan_init = false;  
bool tilt_init = false;  
double curr_tilt_vel = 0.0;
double curr_pan_vel = 0.0;
double curr_tilt_cmd = 0.0;
double curr_pan_cmd = 0.0;
double max_tilt_rate, max_pan_rate;
double rate;
std::unique_ptr<ArmArduino> arm_arduino;
std::string frame_id;
std::unique_ptr<tf::TransformBroadcaster> tfb;

ros::Publisher pan_vel_pub, tilt_vel_pub, marker_pub;

MedianFilter<int,5> tilt_filter, pan_filter;

void panPosCb(const std_msgs::Int32ConstPtr &pan) {
    if (pan->data == 0)
        return;
    if (pan_init) {
        pan_filter.addSample(pan->data);
        curr_pan = pan_filter.getMedian();
    }
        
    if (!pan_init) {
        raw_type motor_pos;
        motor_pos[0] = pan->data;
        motor_pos[1] = curr_tilt;
        angle_type angles;
        arm_arduino->motor2rad(motor_pos, angles);
        curr_pan_cmd = angles[0];
        pan_init = true;
    }
    
}

void tiltPosCb(const std_msgs::Int32ConstPtr &tilt) {
    if (tilt->data == 0) 
        return;
    
    if (tilt_init ) {
        tilt_filter.addSample(tilt->data);
        curr_tilt = tilt_filter.getMedian();
    }
    if (!tilt_init) {
        raw_type motor_pos;
        motor_pos[0] = curr_pan;
        motor_pos[1] = tilt->data;
        angle_type angles;
        arm_arduino->motor2rad(motor_pos, angles);
        curr_tilt_cmd = angles[1];
        tilt_init = true;
    }
}

void panVelCb(const std_msgs::Float32ConstPtr &pan) {
    curr_pan_vel = pan->data;
}

void tiltVelCb(const std_msgs::Float32ConstPtr &tilt) {
    curr_tilt_vel = tilt->data;
}

void managePan() {

    if (!pan_init)
        return;

    curr_pan_cmd += curr_pan_vel * max_pan_rate / rate;

    angle_type angles;
    angles[0] = curr_pan_cmd;
    angles[1] = curr_tilt_cmd;

    raw_type motor_cmd;
    
    if (!arm_arduino->rad2motor(angles, motor_cmd) ) {
        arm_arduino->correctJointLimits(motor_cmd);
        arm_arduino->motor2rad(motor_cmd, angles);
        curr_pan_cmd = angles[0];
        curr_tilt_cmd = angles[1];
    }

    std_msgs::Int32 msg;
    msg.data = static_cast<int32_t>(motor_cmd[0]);
    ROS_INFO("Pan command = %d", msg.data);
    pan_vel_pub.publish(msg);
}


void manageTilt() {
    if (!tilt_init)
        return;

    curr_tilt_cmd += curr_tilt_vel * max_tilt_rate / rate;

    angle_type angles;
    angles[0] = curr_pan_cmd;
    angles[1] = curr_tilt_cmd;

    raw_type motor_cmd;
    if (!arm_arduino->rad2motor(angles, motor_cmd) ) {
        arm_arduino->correctJointLimits(motor_cmd);
    }

    std_msgs::Int32 msg;
    msg.data = static_cast<int32_t>(motor_cmd[1]);

    ROS_INFO("Tilt command = %d", msg.data);
    tilt_vel_pub.publish(msg);
}

visualization_msgs::MarkerArray getARMMarkerArray() {
    int id = 0;
    visualization_msgs::MarkerArray model;
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;
    angle_type angles;

    raw_type herculex_position;
    herculex_position[0] = curr_pan;
    herculex_position[1] = curr_tilt;

    arm_arduino->motor2rad(herculex_position, angles);
    tf::StampedTransform stf;
    stf.stamp_ = ros::Time::now();
    tf::Quaternion q;

    // First and second rotations
    // Emit the first transform: siar_arm_1_2

    // New in MBZIRC: First two joints no longer exist
    
    // Add First Link	
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "siar/arm";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = arm_arduino->length[0] * 0.5;
    marker.pose.position.y = 0;
    marker.pose.position.x = 0;
    marker.pose.orientation.w = 0.70711;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.70711;
    marker.pose.orientation.z = 0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = arm_arduino->length[0];
    marker.color.a = 1.0; 
    marker.color.r = 75.0/255.0;
    marker.color.g = 75.0/255.0;
    marker.header.stamp = ros::Time::now();
    marker.ns = "siar/arm";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = arm_arduino->length[0] * 0.5;
    marker.pose.position.y = 0;
    marker.pose.position.x = 0;
    marker.pose.orientation.w = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = arm_arduino->length[0];
    marker.color.a = 1.0; 
    marker.color.r = 75.0/255.0;
    marker.color.g = 75.0/255.0;
    marker.color.b = 75.0/255.0;
    marker.points.clear();
    model.markers.push_back(marker);
    
    
    stf.frame_id_ = frame_id;
    stf.child_frame_id_ = "siar/arm_link_1";
    tf::Vector3 v(0, 0, arm_arduino->length[0]);
    stf.setIdentity();
    stf.setOrigin(v);
    tfb->sendTransform(stf);
    
    // Rotation 3. Not necessary in MBZIRC
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_pan_tilt";
    stf.setIdentity();
    q.setRPY(0, angles[1], angles[0]);
    stf.setRotation(q);
    tfb->sendTransform(stf);
    
    // Next link
    marker.scale.z = arm_arduino->length[1];
    marker.header.frame_id = stf.child_frame_id_;
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.pose.position.x = arm_arduino->length[1] * 0.5;
    marker.pose.orientation.w = 0.70711;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.70711;
    marker.pose.orientation.z = 0;
    marker.id = id++;
    model.markers.push_back(marker);
    stf.frame_id_ = stf.child_frame_id_;
    stf.child_frame_id_ = "siar/arm_camera";
    v.setValue(arm_arduino->length[1], -0.01, 0);
    stf.setIdentity();
    stf.setOrigin(v);
    tfb->sendTransform(stf);
    
    return model;
  }


int main(int argc, char** argv) {
  ros::init(argc, argv, "arudino_arm_node");
  
  ROS_INFO("Starting Arm Arduino node");
//   tf::TransformListener tf_l(ros::Duration(10);
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tfb.reset(new tf::TransformBroadcaster);
  
  // Definimos pub y sub
  pan_vel_pub = nh.advertise<std_msgs::Int32>("arm_pan_cmd_arduino", 3);
  tilt_vel_pub = nh.advertise<std_msgs::Int32>("arm_tilt_cmd_arduino",3);

  ros::Subscriber pan_vel_sub, tilt_vel_sub;
  ros::Subscriber pan_pos_sub, tilt_pos_sub;

  pan_pos_sub = nh.subscribe("arm_pan_abs_pos_arduino", 1, panPosCb);
  tilt_pos_sub = nh.subscribe("arm_tilt_abs_pos_arduino", 1, tiltPosCb);

  pan_vel_sub = nh.subscribe("arm_pan", 1, panVelCb);
  tilt_vel_sub = nh.subscribe("arm_tilt", 1, tiltVelCb);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("arm_marker", 3);


  // Get parameters from ROS parameter server  
  pnh.param("rate", rate, 20.0);
  pnh.param("max_pan_rate", max_pan_rate, 0.1);
  pnh.param("max_tilt_rate", max_tilt_rate, 0.1);
  pnh.param("frame_id", frame_id, std::string("arduino_arm"));

  // Initialize arm_arduino
  std::string motor_file, angle_file;
  pnh.param("motor_file", motor_file, std::string("cfg/mot_"));
  pnh.param("angle_file", angle_file, std::string("cfg/ang_"));

  arm_arduino.reset(new ArmArduino(motor_file, angle_file));
  ros::Rate r(ros::Duration(1.0/rate));
  
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();  

    if (pan_init) {
        managePan();
    }
    if (tilt_init) {
        manageTilt();
    }
    if (pan_init && tilt_init) {
        marker_pub.publish(getARMMarkerArray());
    }
    ROS_INFO("New command: pan = %f tilt= %f", curr_pan_cmd, curr_tilt_cmd);
  }
  
  return 0;
  
}


