#ifndef __SERIALSERVER_H__
#define __SERIALSERVER_H__

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <string.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <rssi_get/Nvip_status.h>
#include <siar_driver/SiarStatus.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "cycle.hpp"
#include "udp_manager.hpp"

#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>

using boost::asio::ip::udp;

class UDPServer : public UDPManager
{
private:

  // ROS params
  double odomRate, imageRate, depthRate, siar_status_rate, rssiRate, point_rate;
  std::string odomTopic, imageTopic, imageTopic_2, depthTopic, depthTopic_2, depthTopic_3, depthTopic_4;
  std::string allCamerasTopic, publishDepthTopic, joyTopic, rssi_topic, point_topic;
  std::string siar_status_topic, slowTopic, geo_tf_topic;
  std::string camera_1, camera_2, camera_3, camera_4;
  std::string posTopic, widthTopic;
  int jpeg_quality, min_quality;
  bool quality_set, quality_set_2;
  
  // Subscriber rates
  Cycle odomCycle, imageCycle, imageCycle_2, depthCycle, depthCycle_2, rssiCycle, depthCycle_3, depthCycle_4;
  Cycle siarStatusCycle, pointCycle;
  
  // Publishers and subscribers
  ros::Publisher publish_depth_pub, all_cameras_pub, joy_pub, slow_pub, elec_x_pos_pub, width_pos_pub;
  ros::Subscriber odom_sub, image_sub, image_sub_2, depth_sub, depth_sub_2, rssi_sub, depth_sub_3, depth_sub_4, geo_tf_sub;
  ros::Subscriber  siar_status_sub, point_sub;
  ros::NodeHandle nh;
  
public:
  
  // Default constructor
  UDPServer():UDPManager()
  {
    ROS_INFO("In UDPServer::UDPServer");
    // Read node parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("odom_topic", odomTopic))
      odomTopic = "odom";
    if(!lnh.getParam("odom_rate", odomRate))
      odomRate = 5.0; // At first the odometry measures are discraded
    if(!lnh.getParam("joy_topic", joyTopic))
      joyTopic = "/joy";
    if(!lnh.getParam("width_topic", widthTopic))
      widthTopic = "/width_pos";
    if(!lnh.getParam("pos_topic", posTopic))
      posTopic = "/set_x_pos";
    if(!lnh.getParam("camera_1", camera_1))
      camera_1 = "/front";
    if(!lnh.getParam("camera_3", camera_3))
      camera_3 = "/front_left_web";
    if(!lnh.getParam("camera_4", camera_4))
      camera_4 = "/front_right_web";
    if(!lnh.getParam("image_rate", imageRate))
      imageRate = 5.0; 
    if(!lnh.getParam("depth_rate", depthRate))
      depthRate = 4.0; 
    if(!lnh.getParam("camera_2", camera_2))
      camera_2 = "/back";
    if(!lnh.getParam("jpeg_quality", jpeg_quality))
      jpeg_quality = 60;
    if(!lnh.getParam("port", port))
      port = 16000;
    if (!lnh.getParam("rssi_topic", rssi_topic))
      rssi_topic = "/rssi_nvip_2400";
    if (!lnh.getParam("rssi_rate", rssiRate))
      rssiRate = 1.0;
    if (!lnh.getParam("siar_status_topic", siar_status_topic))
      siar_status_topic = "/siar_status";
    if (!lnh.getParam("siar_status_rate", siar_status_rate))
      siar_status_rate = 5;
    if (!lnh.getParam("point_cloud_topic", point_topic))
      point_topic = "/rgbd_odom_node/point_cloud";
    if (!lnh.getParam("point_cloud_rate", point_rate))
      point_rate = 1.0;
       
    // Set the image topics 
    imageTopic = camera_1 + "/rgb/image_raw/compressed";
    imageTopic_2 = camera_2 + "/rgb/image_raw/compressed";
    depthTopic = camera_1 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_2 = camera_2 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_3 = camera_3 + "/depth_registered/image_raw/compressedDepth";
    depthTopic_4 = camera_4 + "/depth_registered/image_raw/compressedDepth";
    geo_tf_topic = "/rgbd_odom/transform";
    
    allCamerasTopic = "/all_cameras";
    publishDepthTopic = "/publish_depth";
    slowTopic = "/slow_motion";
    
    // Create publishers
    publish_depth_pub = nh.advertise<std_msgs::Bool>(publishDepthTopic, 1);
    all_cameras_pub = nh.advertise<std_msgs::Bool>(allCamerasTopic, 1);
    slow_pub = nh.advertise<std_msgs::Bool>(slowTopic, 1);
    joy_pub = nh.advertise<sensor_msgs::Joy>(joyTopic, 1);
    elec_x_pos_pub = nh.advertise<std_msgs::Float32>(posTopic, 1);
    width_pos_pub = nh.advertise<std_msgs::Float32>(widthTopic, 1);
    
    // Create subscribers and setup rates
    odom_sub = nh.subscribe(odomTopic, 1, &UDPServer::odomCallback, this);
    odomCycle.init(odomRate);
    image_sub = nh.subscribe(imageTopic, 1, &UDPServer::imageCallback, this);
    imageCycle.init(imageRate);
    image_sub_2 = nh.subscribe(imageTopic_2, 1, &UDPServer::imageCallback_2, this);
    imageCycle_2.init(imageRate);
    depth_sub = nh.subscribe(depthTopic, 1, &UDPServer::depthCallback, this);
    depthCycle.init(depthRate);
    depth_sub_2 = nh.subscribe(depthTopic_2, 1, &UDPServer::depthCallback_2, this);
    depthCycle_2.init(depthRate);
    depth_sub_3 = nh.subscribe(depthTopic_3, 1, &UDPServer::depthCallback_3, this);
    depthCycle_3.init(depthRate);
    depth_sub_4 = nh.subscribe(depthTopic_4, 1, &UDPServer::depthCallback_4, this);
    depthCycle_4.init(depthRate);
    rssi_sub = nh.subscribe(rssi_topic, 1, &UDPServer::rssiCallback, this);
    rssiCycle.init(rssiRate);
    siar_status_sub = nh.subscribe(siar_status_topic, 1, &UDPServer::siarStatusCallback, this);
    siarStatusCycle.init(siar_status_rate);
    point_sub = nh.subscribe(point_topic, 1, &UDPServer::pointCallback, this);
    pointCycle.init(point_rate);
    geo_tf_sub = nh.subscribe(geo_tf_topic, 1, &UDPServer::tfCallback, this);
    
    // Set default compression quality 
    int i = 0;
    
    quality_set = setJPEGQuality(jpeg_quality, imageTopic);
    quality_set_2 = setJPEGQuality(jpeg_quality, imageTopic_2);
//     setJPEGQuality(jpeg_quality, depthTopic); TODO: png quality of the depth images
//     setJPEGQuality(jpeg_quality, depthTopic_2);

    // Init the UDP session
    init(); 
  }

  ~UDPServer()
  {
    endSession();
  }
  
  // Create the socket and wait for the client to connect
  virtual bool startSession() {
    try
    {
      std::vector<uint8_t> buffer;
      buffer.resize(max_udp_length);
      socket_ptr.reset( new udp::socket(io_service, udp::endpoint(udp::v4(), port)) );
      
      boost::system::error_code error;
      ROS_INFO("Waiting for the client");
      
      std::string s;
      
      while (s != start) {
        size_t len = socket_ptr->receive_from(boost::asio::buffer(buffer, max_udp_length), remote_endpoint, 0, error);
        
        if (len == start.size()) {

          s.resize(len);
          for (size_t i = 0; i < len; i++) {
            s.at(i) = buffer[i];
          }
          ROS_INFO("About to start a session. Received: %s", s.c_str());
        }
      }
      ROS_INFO("Client found!");
      
      if (error && error != boost::asio::error::message_size) 
        throw boost::system::system_error(error);
    } 
    catch (std::exception &e) {
      ROS_INFO("Exception in UPDServer::startSession(). Content: %s", e.what());
      return false;
    }
    return true;
  }
  
  
protected:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!odomCycle.newCycle())
      return;
    
    // Serialize msg and write in serial port
    serializeWrite<nav_msgs::Odometry>(odomTopic, *msg);
  }
  
  void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!imageCycle.newCycle())
    {
      return;
    }
    
    
    // Check for quality set
    // Serialize msg and write over UDP
    if (!quality_set) 
      quality_set = setJPEGQuality(jpeg_quality, imageTopic);
    else
      serializeWrite<sensor_msgs::CompressedImage>(imageTopic, *msg); 
  } 
  
  void imageCallback_2(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!imageCycle_2.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    if (!quality_set_2) 
      quality_set_2 = setJPEGQuality(jpeg_quality, imageTopic_2);
    else 
      serializeWrite<sensor_msgs::CompressedImage>(imageTopic_2, *msg); 
  } 
  
  void depthCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!depthCycle.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(depthTopic, *msg); 
  } 
  
  void depthCallback_2(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!depthCycle_2.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(depthTopic_2, *msg); 
  } 
  
  void depthCallback_3(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!depthCycle_3.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(depthTopic_3, *msg); 
  }
  void depthCallback_4(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!depthCycle_4.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(depthTopic_4, *msg); 
  } 
  
  void rssiCallback(const rssi_get::Nvip_status::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!rssiCycle.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    serializeWrite<rssi_get::Nvip_status>(rssi_topic, *msg); 
  } 
  
  void siarStatusCallback(const siar_driver::SiarStatus::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!siarStatusCycle.newCycle())
    {
      return;
    }
    // Check for quality set
    // Serialize msg and write over UDP
    serializeWrite<siar_driver::SiarStatus>(siar_status_topic, *msg); 
  }
  
  void tfCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
  {
    serializeWrite<geometry_msgs::TransformStamped>(geo_tf_topic, *msg);
    
  }
  
  void readThread()
  {
    std::string topic;
    std::vector<uint8_t> buffer;
    buffer.reserve(max_length);
    
    while(ros::ok() && running)
    {
      // Read one message from serial port
      topic.clear();
      int result = getChunk(topic, buffer);
      if(result < 0) {
        if (result == -3) 
          running = false;
        continue;
      }
        
      
      // Deserialize and publish
      if (topic == allCamerasTopic) {
	deserializePublish<std_msgs::Bool>(buffer.data(), buffer.size(), all_cameras_pub);
      }
      if (topic == publishDepthTopic) {
	deserializePublish<std_msgs::Bool>(buffer.data(), buffer.size(), publish_depth_pub);
      }
      if (topic == joyTopic) {
        deserializePublish<sensor_msgs::Joy>(buffer.data(), buffer.size(), joy_pub);
      }
      if (topic == slowTopic) {
        deserializePublish<std_msgs::Bool>(buffer.data(), buffer.size(), slow_pub);
      }
      if (topic == posTopic) {
        deserializePublish<std_msgs::Float32>(buffer.data(), buffer.size(), elec_x_pos_pub);
      }
      if (topic == widthTopic) {
        deserializePublish<std_msgs::Float32>(buffer.data(), buffer.size(), width_pos_pub);
      }
    }
  }
  
  void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!pointCycle.newCycle())
      return;
    
    // Serialize msg and write in serial port
    serializeWrite<sensor_msgs::PointCloud2>(point_topic, *msg);
  }
  
  
  bool setJPEGQuality(int quality, const std::string image_topic) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter param;
    dynamic_reconfigure::Config conf;
    param.name = "jpeg_quality";
    param.value = quality;
    conf.ints.push_back(param);
    srv_req.config = conf;

    std::string service = image_topic + "/set_parameters";
    if (!ros::service::call(service, srv_req, srv_resp)) {
      ROS_ERROR("Could not call the service %s reconfigure to %d", service.c_str(), quality);
      return false;
    }
    ROS_INFO("New quality %d", jpeg_quality);
    return true;
  }
};

#endif


