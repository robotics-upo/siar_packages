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
#include <libelium_waspmote_gas_node/GasMeasure.h>
#include <siar_driver/SiarStatus.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
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
  std::string odomTopic, imageTopic, imageTopic_2, imageTopic_3, imageTopic_4, imageTopic_5;
  std::string inspectionImageTopic1, inspectionImageTopic2;
  std::string depthTopic, depthTopic_2, depthTopic_3, depthTopic_4, depthTopic_5;
  std::string allCamerasTopic, publishDepthTopic, joyTopic, rssi_topic, point_topic;
  std::string siar_status_topic, slowTopic, geo_tf_topic;
  std::string camera_1, camera_2, camera_3, camera_4, camera_5, inspection_camera1, inspection_camera2;
  std::string posTopic, widthTopic;
  int jpeg_quality, min_quality;
  bool quality_set, quality_set_2;
  std::string thermal_camera_topic;
  u_int8_t comm_mode;
  
  // Subscriber rates
  Cycle odomCycle, imageCycle, imageCycle_2, imageCycle_3, imageCycle_4, imageCycle_5;
  Cycle inspectionCycle_1, inspectionCycle_2, thermalCycle;
  Cycle depthCycle, depthCycle_2, rssiCycle, depthCycle_3, depthCycle_4, depthCycle_5;
  Cycle siarStatusCycle, pointCycle;
  
  // Publishers and subscribers
  ros::Publisher publish_depth_pub, all_cameras_pub, joy_pub, slow_pub, elec_x_pos_pub, width_pos_pub;
  ros::Subscriber odom_sub, image_sub, image_sub_2, image_sub_3, image_sub_4, image_sub_5;
  ros::Subscriber inspection_sub_1, inspection_sub_2, thermal_sub;
  ros::Subscriber depth_sub, depth_sub_2, rssi_sub, gas_sub, depth_sub_3, depth_sub_4, depth_sub_5, geo_tf_sub;
  ros::Subscriber  siar_status_sub, point_sub;
  ros::Subscriber arm_mode_sub, arm_torque_sub;
  ros::NodeHandle nh;
  
public:
  
  // Default constructor
  UDPServer():UDPManager()
  {
    comm_mode = 0;
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
    if(!lnh.getParam("camera_5", camera_5))
      camera_5 = "/up_web";
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
    if (!lnh.getParam("inspection_camera_1", inspection_camera1))
      inspection_camera1 = "/inspection1_cam";
    if (!lnh.getParam("inspection_camera_2", inspection_camera2))
      inspection_camera2 = "/inspection2_cam";
    if (!lnh.getParam("thermal_camera_topic", thermal_camera_topic))
      thermal_camera_topic = "/flip_image/compressed";
       
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
    
    inspectionImageTopic1 = inspection_camera1 + "/image_raw/compressed";
    inspectionImageTopic2 = inspection_camera2 + "/image_raw/compressed";
    
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
    image_sub_3 = nh.subscribe(imageTopic_3, 1, &UDPServer::imageCallback_3, this);
    imageCycle_3.init(imageRate);
    image_sub_4 = nh.subscribe(imageTopic_4, 1, &UDPServer::imageCallback_4, this);
    imageCycle_4.init(imageRate);
    image_sub_5 = nh.subscribe(imageTopic_5, 1, &UDPServer::imageCallback_5, this);
    imageCycle_5.init(imageRate);
    inspection_sub_1 = nh.subscribe(inspectionImageTopic1, 1, &UDPServer::inspectionCallback_1, this);
    inspectionCycle_1.init(imageRate);
    inspection_sub_2 = nh.subscribe(inspectionImageTopic2, 1, &UDPServer::inspectionCallback_2, this);
    inspectionCycle_2.init(imageRate);
    thermal_sub = nh.subscribe(thermal_camera_topic, 1, &UDPServer::thermalCallback, this);
    thermalCycle.init(imageRate);
    depth_sub = nh.subscribe(depthTopic, 1, &UDPServer::depthCallback, this);
    depthCycle.init(depthRate);
    depth_sub_2 = nh.subscribe(depthTopic_2, 1, &UDPServer::depthCallback_2, this);
    depthCycle_2.init(depthRate);
    depth_sub_3 = nh.subscribe(depthTopic_3, 1, &UDPServer::depthCallback_3, this);
    depthCycle_3.init(depthRate);
    depth_sub_4 = nh.subscribe(depthTopic_4, 1, &UDPServer::depthCallback_4, this);
    depthCycle_4.init(depthRate);
    depth_sub_5 = nh.subscribe(depthTopic_5, 1, &UDPServer::depthCallback_5, this);
    depthCycle_5.init(depthRate);
    rssi_sub = nh.subscribe(rssi_topic, 1, &UDPServer::rssiCallback, this);
    rssiCycle.init(rssiRate);
    gas_sub = nh.subscribe("gas_info", 1, &UDPServer::gasCallback, this);
    siar_status_sub = nh.subscribe(siar_status_topic, 1, &UDPServer::siarStatusCallback, this);
    siarStatusCycle.init(siar_status_rate);
    point_sub = nh.subscribe(point_topic, 1, &UDPServer::pointCallback, this);
    pointCycle.init(point_rate);
    geo_tf_sub = nh.subscribe(geo_tf_topic, 1, &UDPServer::tfCallback, this);
    
    arm_torque_sub = nh.subscribe("/arm_torque", 1, &UDPServer::armTorqueCallback, this);
    arm_mode_sub = nh.subscribe("/arm_mode", 1, &UDPServer::armModeCallback, this);
    
    // Set default compression quality 
    int i = 0;
    
    quality_set = setJPEGQuality(jpeg_quality, imageTopic);
    quality_set_2 = setJPEGQuality(jpeg_quality, imageTopic_2);
    setJPEGQuality(jpeg_quality, inspectionImageTopic1);

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
    if(!imageCycle_2.newCycle() && comm_mode > 1)
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
  
  void imageCallback_3(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!imageCycle_3.newCycle() || comm_mode > 0)
    {
      return;
    }
    
    
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(imageTopic_3, *msg); 
  } 
  
  void imageCallback_4(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!imageCycle_4.newCycle() || comm_mode > 0)
    {
      return;
    }
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(imageTopic_4, *msg); 
  }
  
  void imageCallback_5(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!imageCycle_5.newCycle() || comm_mode > 0)
    {
      return;
    }
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(imageTopic_5, *msg); 
  }
  
  void inspectionCallback_1(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!inspectionCycle_1.newCycle() || comm_mode > 1)
    {
      return;
    }
    
    
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(inspectionImageTopic1, *msg); 
  } 
  
  void inspectionCallback_2(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!inspectionCycle_2.newCycle() || comm_mode > 1)
    {
      return;
    }
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(inspectionImageTopic2, *msg); 
  }
  
  void thermalCallback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    // Check if the cycle has elapsed
    if (!thermalCycle.newCycle())
      return;
    
    serializeWrite<sensor_msgs::CompressedImage>(thermal_camera_topic, *msg);
  }
  
  void depthCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!depthCycle.newCycle())
    {
      return;
    }
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
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(depthTopic_4, *msg); 
  }
  
  void depthCallback_5(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    // Check if it is time for sending new data 
    if(!depthCycle_5.newCycle())
    {
      return;
    }
    // Serialize msg and write over UDP
    serializeWrite<sensor_msgs::CompressedImage>(depthTopic_5, *msg); 
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

  void gasCallback(const libelium_waspmote_gas_node::GasMeasure::ConstPtr &msg)
  {
    serializeWrite<libelium_waspmote_gas_node::GasMeasure>("gas_info", *msg);
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
      if (topic == "comms_mode") {
	// Handle the arm_mode
	std_msgs::UInt8  msg;
	try 
	{
	  ros::serialization::IStream stream(buffer.data(), buffer.size());
	  ros::serialization::Serializer<std_msgs::UInt8>::read(stream, msg);
	  comm_mode = msg.data;
	  ROS_INFO("Received a new comm mode: %d", (int)msg.data);
          if (msg.data > 1) 
            setJPEGQuality(jpeg_quality*0.5, imageTopic);
          else
            setJPEGQuality(jpeg_quality, imageTopic);
          
	} 
	catch (std::exception &e) 
	{
	}
	
	
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
  
  void armModeCallback(const std_msgs::Bool::ConstPtr &msg) {
    serializeWrite<std_msgs::Bool>("/arm_mode", *msg);
  }
  void armTorqueCallback(const std_msgs::UInt8::ConstPtr &msg) {
    serializeWrite<std_msgs::UInt8>("/arm_torque", *msg);
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


