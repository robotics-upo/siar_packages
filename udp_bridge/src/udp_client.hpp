#ifndef __TCPCLIENT_HPP__
#define __TCPCLIENT_HPP__

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <rssi_get/Nvip_status.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <siar_driver/SiarStatus.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include "cycle.hpp"
#include "udp_manager.hpp"
#include "joy_translator.hpp"
#include <boost/thread/scoped_thread.hpp>
#include <geometry_msgs/TransformStamped.h>

class UDPClient : public UDPManager
{
private:

  // ROS params
  double joyRate, joyMinRate;
  std::string odomTopic, odomTopic_2, imageTopic, imageTopic_2, imageTopic_3, imageTopic_4, imageTopic_5;
  std::string depthTopic, depthTopic_2, depthTopic_3, depthTopic_4, depthTopic_5;
  std::string camera_1, camera_2, camera_3, camera_4, camera_5, inspection_camera_1, inspection_camera_2;
  std::string camTopic, camTopic_2, camTopic_3, camTopic_4, camTopic_5;
  std::string depthCamTopic, depthCamTopic_2, depthCamTopic_3, depthCamTopic_4, depthCamTopic_5;
  std::string inspectionTopic_1, inspectionTopic_2, inspectionCam_1, inspectionCam_2, thermal_camera_topic, thermal_camera_info_topic;
  std::string allCamerasTopic, publishDepthTopic, joyTopic, rssi_topic, slowTopic;
  std::string point_topic, siar_status_topic, geo_tf_topic;
  std::string posTopic, widthTopic;
  std::string arm_mode_topic, arm_torque_topic;
  
  // ROS stuff
  ros::Publisher image_pub, odom_pub, odom_pub_2, image_pub_2, image_pub_3, image_pub_4, image_pub_5;
  ros::Publisher depth_pub, depth_pub_2, depth_pub_3, depth_pub_4, depth_pub_5;
  ros::Publisher cam_pub, cam_pub_2, cam_pub_3, cam_pub_4, cam_pub_5;
  ros::Publisher depth_cam_pub, depth_cam_pub_2, depth_cam_pub_3, depth_cam_pub_4, depth_cam_pub_5;
  ros::Publisher rssi_pub, siar_status_pub, point_pub;
  ros::Publisher inspection_pub_1, inspection_pub_2, inspection_cam_pub_1, inspection_cam_pub_2;
  ros::Publisher thermal_pub, thermal_cam_pub;
  ros::Publisher arm_mode_pub, arm_torque_pub;
  
  ros::Subscriber publish_depth_sub, all_cameras_sub, joy_sub, slow_sub;
  ros::Subscriber set_x_pos_sub, width_pos_sub;
  ros::NodeHandle nh;
  tf::TransformBroadcaster tf_broadcaster;
  std::string base_frame_id, odom_frame_id;
  
  // Camera Info stuff
  sensor_msgs::CameraInfo general_info, downsampled_info, inspection_info;
  
  // UDP stuff
  std::string ip_address;
  
  JoyTranslator *joy_trans;
  bool translate_joy;
  
public:
  
  // Default constructor
  UDPClient():UDPManager()
  {
    ROS_INFO("In UDPClient");
    // Read node parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("ip_address", ip_address))
      ip_address = "192.168.168.11";
    if(!lnh.getParam("port", port))
      port = 16000;
    if(!lnh.getParam("odom_topic", odomTopic))
      odomTopic = "odom";
    if(!lnh.getParam("odom_topic_2", odomTopic_2))
      odomTopic = "odom_cam";
    if(!lnh.getParam("camera_1", camera_1))
      camera_1 = "/front";
    if(!lnh.getParam("camera_2", camera_2))
      camera_2 = "/back";
    if(!lnh.getParam("camera_3", camera_3))
      camera_3 = "/front_left";
    if(!lnh.getParam("camera_4", camera_4))
      camera_4 = "/front_right";
    if(!lnh.getParam("camera_5", camera_5))
      camera_5 = "/up_web";
    if (!lnh.getParam("inspection_camera_1", inspection_camera_1))
      inspection_camera_1 = "/inspection1_cam";
    if (!lnh.getParam("inspection_camera_2", inspection_camera_2))
      inspection_camera_2 = "/inspection2_cam";
    if (!lnh.getParam("base_frame_id", base_frame_id))
      base_frame_id = "/base_link";
    if (!lnh.getParam("odom_frame_id", odom_frame_id))  
      odom_frame_id = "/odom";
    if (!lnh.getParam("rssi_topic", rssi_topic))
      rssi_topic = "/rssi_nvip_2400";
    if (!lnh.getParam("siar_status_topic", siar_status_topic))
      siar_status_topic = "/siar_status";
    if (!lnh.getParam("joy_topic", joyTopic))
      joyTopic = "/joy";
    if (!lnh.getParam("point_cloud_topic", point_topic))
      point_topic = "/rgbd_odom_node/point_cloud";
    if(!lnh.getParam("pos_topic", posTopic))
      posTopic = "/set_x_pos";
    if(!lnh.getParam("width_topic", widthTopic))
      widthTopic = "/width_pos";
    // Set the image topics 
    imageTopic = camera_1 + "/rgb/image_raw/compressed";
    imageTopic_2 = camera_2 + "/rgb/image_raw/compressed";
    imageTopic_3 = camera_3 + "/rgb/image_raw/compressed";
    imageTopic_4 = camera_4 + "/rgb/image_raw/compressed";
    imageTopic_5 = camera_5 + "/rgb/image_raw/compressed";
    depthTopic = camera_1 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_2 = camera_2 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_3 = camera_3 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_4 = camera_4 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_5 = camera_5 + "/depth_registered/image_raw/compressedDepth";
    geo_tf_topic = "/rgbd_odom/transform";
    inspectionTopic_1 = inspection_camera_1 + "/image_raw/compressed";
    inspectionTopic_2 = inspection_camera_2 + "/image_raw/compressed";
    arm_mode_topic = "/arm_mode";
    arm_torque_topic = "/arm_torque";
    
    if (!lnh.getParam("thermal_camera_topic", thermal_camera_topic))
      thermal_camera_topic = "/flip_image";
          
    // Create publishers
    image_pub = nh.advertise<sensor_msgs::CompressedImage>(imageTopic, 1);
    image_pub_2 = nh.advertise<sensor_msgs::CompressedImage>(imageTopic_2, 1);
    image_pub_3 = nh.advertise<sensor_msgs::CompressedImage>(imageTopic_3, 1);
    image_pub_4 = nh.advertise<sensor_msgs::CompressedImage>(imageTopic_4, 1);
    image_pub_5 = nh.advertise<sensor_msgs::CompressedImage>(imageTopic_5, 1);
    depth_pub = nh.advertise<sensor_msgs::CompressedImage>(depthTopic, 1);
    depth_pub_2 = nh.advertise<sensor_msgs::CompressedImage>(depthTopic_2, 1);
    depth_pub_3 = nh.advertise<sensor_msgs::CompressedImage>(depthTopic_3, 1);
    depth_pub_4 = nh.advertise<sensor_msgs::CompressedImage>(depthTopic_4, 1);
    depth_pub_5 = nh.advertise<sensor_msgs::CompressedImage>(depthTopic_5, 1);
    rssi_pub = nh.advertise<rssi_get::Nvip_status>(rssi_topic, 1);
    siar_status_pub = nh.advertise<siar_driver::SiarStatus>(siar_status_topic, 1);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>(point_topic, 1);
    inspection_pub_1 = nh.advertise<sensor_msgs::CompressedImage>(inspectionTopic_1, 1);
    inspection_pub_2 = nh.advertise<sensor_msgs::CompressedImage>(inspectionTopic_2, 1);
    thermal_pub = nh.advertise<sensor_msgs::CompressedImage>(thermal_camera_topic, 1);
    arm_mode_pub = nh.advertise<std_msgs::Bool>(arm_mode_topic, 1);
    arm_torque_pub = nh.advertise<std_msgs::UInt8>(arm_torque_topic, 1);
    
    camTopic = camera_1 + "/rgb/camera_info";
    camTopic_2 = camera_2 + "/rgb/camera_info";
    camTopic_3 = camera_3 + "/rgb/camera_info";
    camTopic_4 = camera_4 + "/rgb/camera_info";
    camTopic_5 = camera_5 + "/rgb/camera_info";
    depthCamTopic = camera_1 + "/depth_registered/camera_info";
    depthCamTopic_2 = camera_2 + "/depth_registered/camera_info";
    depthCamTopic_3 = camera_3 + "/depth_registered/camera_info";
    depthCamTopic_4 = camera_4 + "/depth_registered/camera_info";
    depthCamTopic_5 = camera_5 + "/depth_registered/camera_info";
    inspectionCam_1 = inspection_camera_1 + "/camera_info";
    inspectionCam_2 = inspection_camera_2 + "/camera_info";
    
    cam_pub = nh.advertise<sensor_msgs::CameraInfo>(camTopic, 1);
    cam_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(camTopic_2, 1);
    cam_pub_3 = nh.advertise<sensor_msgs::CameraInfo>(camTopic_3, 1);
    cam_pub_4 = nh.advertise<sensor_msgs::CameraInfo>(camTopic_4, 1);
    cam_pub_5 = nh.advertise<sensor_msgs::CameraInfo>(camTopic_5, 1);
    depth_cam_pub = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic, 1);
    depth_cam_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic_2, 1);
    depth_cam_pub_3 = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic_3, 1);
    depth_cam_pub_4 = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic_4, 1);
    depth_cam_pub_5 = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic_5, 1);
    
    odom_pub = nh.advertise<nav_msgs::Odometry>(odomTopic, 1);
    odom_pub_2 = nh.advertise<nav_msgs::Odometry>(odomTopic_2, 1);
    
    inspection_cam_pub_1 = nh.advertise<sensor_msgs::CameraInfo>(inspectionCam_1, 1);
    inspection_cam_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(inspectionCam_2, 1);
    
    thermal_cam_pub = nh.advertise<sensor_msgs::CameraInfo>(thermal_camera_info_topic, 1);
    
    publishDepthTopic = "/publish_depth";
    allCamerasTopic = "/all_cameras";
    
    // Buttons params
    joy_trans = NULL;
    translate_joy = false;
    lnh.getParam("translate_joy", translate_joy);
    if (translate_joy)
      joy_trans = new JoyTranslator(nh, lnh);
    
    // Create subscribers
    publish_depth_sub = nh.subscribe(publishDepthTopic, 1, &UDPClient::publishDepthCallback, this);
    all_cameras_sub = nh.subscribe(allCamerasTopic, 1, &UDPClient::allCamerasCallback, this);
    slow_sub = nh.subscribe(slowTopic, 1, &UDPClient::slowCallback, this);
    joy_sub = nh.subscribe(joyTopic, 1, &UDPClient::joyCallback, this);
    set_x_pos_sub = nh.subscribe(posTopic, 1, &UDPClient::posReceived, this);
    width_pos_sub = nh.subscribe(widthTopic, 1, &UDPClient::widthReceived, this); 
    
    // Set the general camera info
    general_info.D.resize(5);
    int aux;
    if (!lnh.getParam("height", aux)) 
      general_info.height = 480;
    else
      general_info.height = aux;
    if (!lnh.getParam("width", aux))
      general_info.width = 640;
    else 
      general_info.width = aux;
    
    
    for (int i = 0; i < 5; i++) {
      general_info.D[i] = 0.0;
    }
    
    std::vector<double> vec;
    
    if (!lnh.getParam("K", vec)) {
      general_info.K[0] = 570.3422241210938;
      general_info.K[1] = 0.0;
      general_info.K[2] = 319.5;
      general_info.K[3] = 0.0;
      general_info.K[4] = 570.3422241210938;
      general_info.K[5] = 239.5;
      general_info.K[6] = 0.0;
      general_info.K[7] = 0.0;
      general_info.K[8] = 1.0;
    } else {
      for (size_t i = 0;i < vec.size() && i < 9; i++) {
        general_info.K[i] = vec[i];
      }
      
    }
    
    if (!lnh.getParam("P", vec)) {
      general_info.P[0] = 570.3422241210938;
      general_info.P[1] = 0.0;
      general_info.P[2] = 319.5;
      general_info.P[3] = 0.0;
      general_info.P[4] = 0.0;
      general_info.P[5] = 570.3422241210938;
      general_info.P[6] = 239.5;
      general_info.P[7] = 0.0;
      general_info.P[8] = 0.0;
      general_info.P[9] = 0.0;
      general_info.P[10] = 1.0;
      general_info.P[11] = 0.0;
    } else {
      for (size_t i = 0;i < vec.size() && i < 12; i++) {
        general_info.P[i] = vec[i];
      }
      
    }
    
    if (!lnh.getParam("R", vec)) {
      general_info.R[0] = 1.0;
      general_info.R[1] = 0.0;
      general_info.R[2] = 0.0;
      general_info.R[3] = 0.0;
      general_info.R[4] = 1.0;
      general_info.R[5] = 0.0;
      general_info.R[6] = 0.0;
      general_info.R[7] = 0.0;
      general_info.R[8] = 1.0;
    } else {
      for (size_t i = 0;i < vec.size() && i < 9; i++) {
        general_info.R[i] = vec[i];
      }
      
    }
    general_info.roi.do_rectify = false;
    general_info.roi.x_offset = 0;
    general_info.roi.y_offset = 0;
    general_info.roi.height = 0;
    general_info.roi.width = 0;
    
    general_info.distortion_model = "plumb_bob";
    
    // Set the downsampled camera info
    downsampled_info.D.resize(5);
    if (!lnh.getParam("height2", aux)) 
      downsampled_info.height = 120;
    else
      downsampled_info.height = aux;
    if (!lnh.getParam("width2", aux))
      downsampled_info.width = 160;
    else 
      downsampled_info.width = aux;
    for (int i = 0; i < 5; i++) {
      downsampled_info.D[i] = 0.0;
    }
    
    if (!lnh.getParam("K2", vec)) {
      downsampled_info.K[0] = 570.3422241210938;
      downsampled_info.K[1] = 0.0;
      downsampled_info.K[2] = 319.5;
      downsampled_info.K[3] = 0.0;
      downsampled_info.K[4] = 570.3422241210938;
      downsampled_info.K[5] = 239.5;
      downsampled_info.K[6] = 0.0;
      downsampled_info.K[7] = 0.0;
      downsampled_info.K[8] = 1.0;
    } else {
      for (size_t i = 0;i < vec.size() && i < 9; i++) {
        downsampled_info.K[i] = vec[i];
      }
      
    }
    
    if (!lnh.getParam("P2", vec)) {
      downsampled_info.P[0] = 570.3422241210938;
      downsampled_info.P[1] = 0.0;
      downsampled_info.P[2] = 319.5;
      downsampled_info.P[3] = 0.0;
      downsampled_info.P[4] = 0.0;
      downsampled_info.P[5] = 570.3422241210938;
      downsampled_info.P[6] = 239.5;
      downsampled_info.P[7] = 0.0;
      downsampled_info.P[8] = 0.0;
      downsampled_info.P[9] = 0.0;
      downsampled_info.P[10] = 1.0;
      downsampled_info.P[11] = 0.0;
    } else {
      for (size_t i = 0;i < vec.size() && i < 12; i++) {
        downsampled_info.P[i] = vec[i];
      }
      
    }
    
    if (!lnh.getParam("R2", vec)) {
      downsampled_info.R[0] = 1.0;
      downsampled_info.R[1] = 0.0;
      downsampled_info.R[2] = 0.0;
      downsampled_info.R[3] = 0.0;
      downsampled_info.R[4] = 1.0;
      downsampled_info.R[5] = 0.0;
      downsampled_info.R[6] = 0.0;
      downsampled_info.R[7] = 0.0;
      downsampled_info.R[8] = 1.0;
    } else {
      for (size_t i = 0;i < vec.size() && i < 9; i++) {
        downsampled_info.R[i] = vec[i];
      }
      
    }
    
    //TODO: Camera info of the inspection camera?
    inspection_info = general_info;
    if (lnh.getParam("height_inspection", aux)) 
      inspection_info.height = aux;
    if (lnh.getParam("width_inspection", aux))
      inspection_info.width = aux;
    if (lnh.getParam("PI", vec)) {
      for (size_t i = 0;i < vec.size() && i < 9; i++) {
        inspection_info.P[i] = vec[i];
      }
    }
    if (lnh.getParam("RI", vec)) {
      for (size_t i = 0;i < vec.size() && i < 9; i++) {
        inspection_info.R[i] = vec[i];
      }
    }
    std::string dist_model;
    if (lnh.getParam("dist_model_I", dist_model)) {
      inspection_info.distortion_model = dist_model;
    }
    
    init();
  }

  ~UDPClient()
  {
    endSession();
    delete joy_trans;
  }
  
virtual bool startSession()
{
  try
  {
    msg_list.clear();
    msg_sent = 0;
    ROS_INFO("Starting comms. Ip = %s. Port = %d", ip_address.c_str(), port);
    udp::resolver resolver(io_service);
    timer_.reset(new boost::asio::deadline_timer(io_service));
    udp::resolver::query query(udp::v4(), ip_address.c_str(), boost::to_string(port).c_str());
    remote_endpoint = *resolver.resolve(query);

    socket_ptr.reset(new udp::socket(io_service));
    socket_ptr->open(udp::v4());

    // Start comms TODO: Wait for server response?
//     ROS_INFO("Starting comms 2");
    socket_ptr->send_to(boost::asio::buffer(start.data(), start.size()), remote_endpoint);
//     ROS_INFO("Comms started");
  }
  catch (std::exception &e) 
  {
    ROS_INFO("Exception in UDPClient::startSession(). Content: %s", e.what());
    return false;
  }
    
  return true;
}


  
  
protected:

  void readThread()
  {
    std::string topic;
    std::vector<uint8_t> buffer;
    buffer.reserve(max_length);
    
    
    
    while(ros::ok() && running)
    {
      topic.clear();
      int result = getChunk(topic, buffer);
      if(result < 0) {
        if (result == -3) // Timeout
          running = false;
        continue;
      }
      socket_ptr->send_to(boost::asio::buffer(start.data(), start.size()), remote_endpoint);
      
      // Deserialize and publish
      if(topic == imageTopic) 
      {
        ROS_INFO("Deserializing and publishing topic %s", imageTopic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        image_pub.publish(msg);
        sensor_msgs::CameraInfo msg_info = general_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        cam_pub.publish(msg_info);
      }
      if(topic == imageTopic_2) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        image_pub_2.publish(msg);
        sensor_msgs::CameraInfo msg_info = general_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        cam_pub_2.publish(msg_info);
      }
      if(topic == imageTopic_3) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        image_pub_3.publish(msg);
        sensor_msgs::CameraInfo msg_info = downsampled_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        cam_pub_3.publish(msg_info);
      }
      if(topic == imageTopic_4) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        image_pub_4.publish(msg);
        sensor_msgs::CameraInfo msg_info = downsampled_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        cam_pub_4.publish(msg_info);
      }
      if(topic == imageTopic_5) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        image_pub_5.publish(msg);
        sensor_msgs::CameraInfo msg_info = downsampled_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        cam_pub_5.publish(msg_info);
      }
      if(topic == inspectionTopic_1) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        inspection_pub_1.publish(msg);
        sensor_msgs::CameraInfo msg_info = inspection_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        inspection_cam_pub_1.publish(msg_info);
      }
      if(topic == inspectionTopic_2) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        inspection_pub_2.publish(msg);
        sensor_msgs::CameraInfo msg_info = inspection_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        inspection_cam_pub_2.publish(msg_info);
      }
      if (topic == thermal_camera_topic)
      {
	ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        thermal_pub.publish(msg);
	// TODO: camera info of the thermal camera
        sensor_msgs::CameraInfo msg_info = inspection_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        thermal_cam_pub.publish(msg_info);
      }
      if(topic == depthTopic) 
      {
        ROS_INFO("Deserializing and publishing topic %s", depthTopic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        depth_pub.publish(msg);
        sensor_msgs::CameraInfo msg_info = general_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        depth_cam_pub.publish(msg_info);
      }
      if(topic == depthTopic_2) 
      {
        ROS_INFO("Deserializing and publishing topic %s", depthTopic_2.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        depth_pub_2.publish(msg);
        sensor_msgs::CameraInfo msg_info = general_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        
        depth_cam_pub.publish(msg_info);
      }
      if(topic == depthTopic_3) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        depth_pub_3.publish(msg);
        sensor_msgs::CameraInfo msg_info = downsampled_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        
        depth_cam_pub_3.publish(msg_info);
      }
      if(topic == depthTopic_4) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        depth_pub_4.publish(msg);
        sensor_msgs::CameraInfo msg_info = downsampled_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        
        depth_cam_pub_4.publish(msg_info);
      }
      if(topic == depthTopic_5) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        depth_pub_5.publish(msg);
        sensor_msgs::CameraInfo msg_info = downsampled_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        
        depth_cam_pub_5.publish(msg_info);
      }
      if(topic == odomTopic) 
      {
        nav_msgs::Odometry odom = deserializePublish<nav_msgs::Odometry>(buffer.data(), buffer.size(), odom_pub);
      }
      if(topic == odomTopic_2) 
      {
        nav_msgs::Odometry odom = deserializePublish<nav_msgs::Odometry>(buffer.data(), buffer.size(), odom_pub_2);
      }
      if(topic == rssi_topic) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<rssi_get::Nvip_status>(buffer.data(), buffer.size(), rssi_pub);
      }
      if(topic == siar_status_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<siar_driver::SiarStatus>(buffer.data(), buffer.size(), siar_status_pub);
      }
      if (topic == point_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<sensor_msgs::PointCloud2>(buffer.data(), buffer.size(), point_pub);
      }
      if (topic == geo_tf_topic) 
      {
        geometry_msgs::TransformStamped odom_trans = deserialize<geometry_msgs::TransformStamped>(buffer.data(), buffer.size());
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = base_frame_id;
        odom_trans.header.stamp = ros::Time::now();
      
        // Publish the odometry TF
        tf_broadcaster.sendTransform(odom_trans);
      }
      if (topic == arm_mode_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<std_msgs::Bool>(buffer.data(), buffer.size(), arm_mode_pub);
      }
      if (topic == arm_torque_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<std_msgs::UInt8>(buffer.data(), buffer.size(), arm_torque_pub);
      }
    }
  }
  
  void allCamerasCallback(const std_msgs::BoolConstPtr &msg)
  { 
    // Serialize msg and write in serial port
    serializeWrite<std_msgs::Bool>(allCamerasTopic, *msg);
  }
  
  void slowCallback(const std_msgs::BoolConstPtr &msg)
  { 
    // Serialize msg and write in serial port
    serializeWrite<std_msgs::Bool>(slowTopic, *msg);
  }
  
  void publishDepthCallback(const std_msgs::BoolConstPtr &msg)
  {
    // In this case there is no need in adding a cycle --> all data will be published
    serializeWrite<std_msgs::Bool>(publishDepthTopic, *msg);
  }
  
  void joyCallback(const sensor_msgs::JoyConstPtr &msg)
  {
    // All data will be published (no sampling)
    if (joy_trans != NULL) {
      sensor_msgs::Joy trans_msg = joy_trans->translateJoy(*msg);
      
      serializeWrite<sensor_msgs::Joy>(joyTopic, trans_msg);
    } else {
      serializeWrite<sensor_msgs::Joy>(joyTopic, *msg);
    }
  }
  
  void posReceived(const std_msgs::Float32ConstPtr &msg)
  {
    serializeWrite<std_msgs::Float32>(posTopic, *msg);
  }
  
  void widthReceived(const std_msgs::Float32ConstPtr &msg)
  {
    serializeWrite<std_msgs::Float32>(widthTopic, *msg);
  }
};

#endif








