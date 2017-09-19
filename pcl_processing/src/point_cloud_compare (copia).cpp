#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "processPointCloud.h"
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include "iostream"

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"

//tf::TransformListener tfListener;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

ros::Publisher pub_points_;
sensor_msgs::PointCloud2 pc_ref;



PointCloud cloud;
const DP data;
const DP ref;   
PM::ICP icp;
PM::TransformationParameters T;
  
void callback(const PointCloud::ConstPtr& msg)
{

  sensor_msgs::PointCloud2 pc2_data;
  PointCloud pc_data;

  pc_data = *msg;
  
  //Convert the result into a ROS msg
  pcl::toROSMsg(pc_data, pc2_data);
  
  PointCloud pcl_cloud;
  pcl::fromROSMsg(pc2_data, pcl_cloud);
  for (size_t i = 0; i < pc_data.points.size (); ++i)
    if(pc_data.points[i].z > 3.5)
    { 
      pc_data.points[i].x = 0.0/0.0; //result NaN value
      pc_data.points[i].y = 0.0/0.0;
      pc_data.points[i].z = 0.0/0.0;
    }
  
  icp.setDefault();
    
  T = icp(data, ref);

  pub_points_.publish(pc2_data);


}




int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  //Subscribe the node to the point cloud from the ROS bag file
  
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("pcl_ref/t181.pcd", cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file pcl_ref/t181.pcd \n");
    return (-1);
  }
	      
  cloud.header.frame_id = "/front_rgb_optical_frame";
  
  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("cleanedPoints",  1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("front/points", 1, callback);  
  
  
  
  ref(DP::load("pcl_ref/t181.pcd"));

  
  
  while(ros::ok())
  {
    ros::spinOnce();
  }
  
  
  return(0);
}
