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
#include <siar_driver/SiarBatteryMonitor.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include "cycle.hpp"
#include "udp_manager.hpp"
#include <boost/thread/scoped_thread.hpp>

class UDPClient : public UDPManager
{
private:

  // ROS params
  double joyRate, joyMinRate;
  std::string odomTopic, odomTopic_2, imageTopic, imageTopic_2, depthTopic, depthTopic_2;
  std::string camera_1, camera_2;
  std::string camTopic, camTopic_2, depthCamTopic, depthCamTopic_2;
  std::string allCamerasTopic, publishDepthTopic, joyTopic, rssi_topic, slowTopic;
  std::string motor_battery_topic, elec_battery_topic, point_topic;
  
  // ROS stuff
  ros::Publisher image_pub, odom_pub, odom_pub_2, image_pub_2, depth_pub, depth_pub_2;
  ros::Publisher cam_pub, cam_pub_2, depth_cam_pub, depth_cam_pub_2;
  ros::Publisher rssi_pub, motor_battery_pub, elec_battery_pub, point_pub;
  
  ros::Subscriber publish_depth_sub, all_cameras_sub, joy_sub, slow_sub;
  ros::NodeHandle nh;
//   tf::TransformBroadcaster tf_broadcaster;
  std::string base_frame_id, odom_frame_id;
  
  // Camera Info stuff
  sensor_msgs::CameraInfo general_info;
  
  // UDP stuff
  std::string ip_address;
  
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
    if (!lnh.getParam("base_frame_id", base_frame_id))
      base_frame_id = "/base_link";
    if (!lnh.getParam("odom_frame_id", odom_frame_id))  
      odom_frame_id = "/odom";
    if (!lnh.getParam("rssi_topic", rssi_topic))
      rssi_topic = "/rssi_nvip_2400";
    if (!lnh.getParam("motor_battery_topic", motor_battery_topic))
      motor_battery_topic = "/motor_battery_status";
    if (!lnh.getParam("elec_battery_topic", elec_battery_topic))
      elec_battery_topic = "/elec_battery_status";
    if (!lnh.getParam("joy_topic", joyTopic))
      joyTopic = "/joy";
    if (!lnh.getParam("point_cloud_topic", point_topic))
      point_topic = "/rgbd_odom_node/point_cloud";
    // Set the image topics 
    imageTopic = camera_1 + "/rgb/image_raw/compressed";
    imageTopic_2 = camera_2 + "/rgb/image_raw/compressed";
    depthTopic = camera_1 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_2 = camera_2 + "/depth_registered/image_raw/compressedDepth";
          
    // Create publishers
    image_pub = nh.advertise<sensor_msgs::CompressedImage>(imageTopic, 1);
    image_pub_2 = nh.advertise<sensor_msgs::CompressedImage>(imageTopic_2, 1);
    depth_pub = nh.advertise<sensor_msgs::CompressedImage>(depthTopic, 1);
    depth_pub_2 = nh.advertise<sensor_msgs::CompressedImage>(depthTopic_2, 1);
    rssi_pub = nh.advertise<rssi_get::Nvip_status>(rssi_topic, 1);
    motor_battery_pub = nh.advertise<siar_driver::SiarBatteryMonitor>(motor_battery_topic, 1);
    elec_battery_pub = nh.advertise<siar_driver::SiarBatteryMonitor>(elec_battery_topic, 1);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>(point_topic, 1);
    
    camTopic = camera_1 + "/rgb/camera_info";
    camTopic_2 = camera_2 + "/rgb/camera_info";
    depthCamTopic = camera_1 + "/depth_registered/camera_info";
    depthCamTopic_2 = camera_2 + "/depth_registered/camera_info";
    cam_pub = nh.advertise<sensor_msgs::CameraInfo>(camTopic, 1);
    cam_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(camTopic_2, 1);
    depth_cam_pub = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic, 1);
    depth_cam_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(depthCamTopic_2, 1);
    
    odom_pub = nh.advertise<nav_msgs::Odometry>(odomTopic, 1);
    odom_pub_2 = nh.advertise<nav_msgs::Odometry>(odomTopic_2, 1);
    
    publishDepthTopic = "/publish_depth";
    allCamerasTopic = "/all_cameras";
    
    // Create subscribers
    publish_depth_sub = nh.subscribe(publishDepthTopic, 1, &UDPClient::publishDepthCallback, this);
    all_cameras_sub = nh.subscribe(allCamerasTopic, 1, &UDPClient::allCamerasCallback, this);
    slow_sub = nh.subscribe(slowTopic, 1, &UDPClient::slowCallback, this);
    joy_sub = nh.subscribe(joyTopic, 1, &UDPClient::joyCallback, this);
    
    
    // Set the general camera info
    general_info.D.resize(5);
    general_info.height = 480;
    general_info.width = 640;
    for (int i = 0; i < 5; i++) {
      general_info.D[i] = 0.0;
    }
    general_info.K[0] = 570.3422241210938;
    general_info.K[1] = 0.0;
    general_info.K[2] = 319.5;
    general_info.K[3] = 0.0;
    general_info.K[4] = 570.3422241210938;
    general_info.K[5] = 239.5;
    general_info.K[6] = 0.0;
    general_info.K[7] = 0.0;
    general_info.K[8] = 1.0;
    
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
    
    general_info.R[0] = 1.0;
    general_info.R[1] = 0.0;
    general_info.R[2] = 0.0;
    general_info.R[3] = 0.0;
    general_info.R[4] = 1.0;
    general_info.R[5] = 0.0;
    general_info.R[6] = 0.0;
    general_info.R[7] = 0.0;
    general_info.R[8] = 1.0;
    
    general_info.roi.do_rectify = false;
    general_info.roi.x_offset = 0;
    general_info.roi.y_offset = 0;
    general_info.roi.height = 0;
    general_info.roi.width = 0;
    
    general_info.distortion_model = "plumb_bob";
    
    init();
  }

  ~UDPClient()
  {
    endSession();
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
      if(topic == depthTopic) 
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
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
        ROS_INFO("Deserializing and publishing topic %s", imageTopic.c_str());
        sensor_msgs::CompressedImage msg = deserialize<sensor_msgs::CompressedImage>(buffer.data(), buffer.size());
        msg.header.stamp = ros::Time::now();
        depth_pub_2.publish(msg);
        sensor_msgs::CameraInfo msg_info = general_info;
        msg_info.header = msg.header;
        msg_info.header.frame_id = msg.header.frame_id;
        
        depth_cam_pub.publish(msg_info);
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
      if(topic == motor_battery_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<siar_driver::SiarBatteryMonitor>(buffer.data(), buffer.size(), motor_battery_pub);
      }
      if (topic == elec_battery_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<siar_driver::SiarBatteryMonitor>(buffer.data(), buffer.size(), elec_battery_pub);
      }
      if (topic == point_topic)
      {
        ROS_INFO("Deserializing and publishing topic %s", topic.c_str());
        deserializePublish<sensor_msgs::PointCloud2>(buffer.data(), buffer.size(), point_pub);
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
    serializeWrite<sensor_msgs::Joy>(joyTopic, *msg);
  }
};

#endif








