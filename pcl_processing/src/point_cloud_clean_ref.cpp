#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "processPointCloud.h"
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include "iostream"
#include <pcl/filters/filter.h>

//tf::TransformListener tfListener;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_points_;
  sensor_msgs::PointCloud2 points_msg2;


int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  //Subscribe the node to the point cloud from the ROS bag file
  
  
  PointCloud cloud;
  std::vector<int> indices; 

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("pcl_ref/output.pcd", cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file pcl_ref/output.pcd \n");
    return (-1);
  }
    
  for (size_t i = 0; i < cloud.points.size (); ++i)
     if(cloud.points[i].z > 3.5)
     { 
       cloud.points[i].x = 0.0/0.0; //result NaN value
       cloud.points[i].y = 0.0/0.0;
       cloud.points[i].z = 0.0/0.0;
     }
	      
   // pcl::removeNaNFromPointCloud(*cloud,*cloud,indices); 
  
    cloud.header.frame_id = "/front_rgb_optical_frame";
    pcl::toROSMsg(cloud, points_msg2);
  
  
  
  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("cleanedPoints",  1);
  
  
  
  
  char filename[100];
  sprintf(filename, "pcl_cleaned.pcd");
    
  ROS_INFO("filename: %s", filename);

  pcl::io::savePCDFileASCII (filename, cloud);
  
  
  
  
  
  ROS_INFO("Publishing pointcloud cleaned");

  while(ros::ok())
      pub_points_.publish(points_msg2);

  return(0);
}
