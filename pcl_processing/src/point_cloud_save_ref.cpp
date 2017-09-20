#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "processPointCloud.h"
#include <pcl_conversions/pcl_conversions.h> //hydro
#include <sensor_msgs/PointCloud2.h> //hydro
#include <pcl/io/pcd_io.h>
#include "iostream"

//tf::TransformListener tfListener;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_points_;
int writeCount=1;


void callback(const PointCloud::ConstPtr& msg)
{


  sensor_msgs::PointCloud2 points_msg2;
  PointCloud result2;

  result2 = *msg;
  
  //Convert the result into a ROS msg
  pcl::toROSMsg(result2, points_msg2);
  
  PointCloud pcl_cloud;
  pcl::fromROSMsg(points_msg2, pcl_cloud);
  

  writeCount++;
  char filename[100];
  sprintf(filename, "output%dx.pcd", writeCount);
    
  ROS_INFO("filename: %s", filename);

  pcl::io::savePCDFileASCII (filename, pcl_cloud);

  pub_points_.publish(points_msg2);


}

int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  //Subscribe the node to the point cloud from the ROS bag file
  
  
 // ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::Subscriber sub = nh.subscribe<PointCloud>("front/points", 1, callback);
  
  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("processedPoints",  1);;
  ROS_INFO("Start saving pointclouds");

  ros::spin();
}
