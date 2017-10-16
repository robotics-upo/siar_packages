#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "siar_arm/siar_arm.hpp"

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>

// aruco stuff
aruco::MarkerDetector mDetector_;
aruco::CameraParameters camParam_;
vector<aruco::Marker> markers_;

// node params
bool useRectifiedImages_;
std::string marker_frame_;
std::string camera_frame_;
std::string reference_frame_;
double marker_size_;

// ROS pub-sub
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;

image_transport::Publisher image_pub_;
image_transport::Publisher debug_pub_;
ros::Publisher marker_pub_;
ros::Publisher marker_list_pub_;
tf::TransformListener tfListener_;

ros::Subscriber cam_info_sub_;
aruco_msgs::MarkerArray::Ptr marker_msg_;
cv::Mat inImage_;
bool useCamInfo_;
std_msgs::UInt32MultiArray marker_list_msg_;

void ArucoMarkersCallback(const  aruco_msgs::MarkerArray::ConstPtr& arm_pose){
  
  
  
  
  
}






int main(int argc, char **argv)
{

	ros::init(argc, argv, "arm_control_pose");

	ros::NodeHandle n;
	
	ros::Subscriber marker_sub = n.subscribe("/aruco_marker_publisher/markers", 1000, ArucoMarkersCallback);
	
        ros::spin();

}
