#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "processPointCloud.h"

#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>


tf::TransformListener * tfListener;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_points_;

PointCloud accumulatedCloud;

void callback(const PointCloud::ConstPtr& msg)
{
 
  sensor_msgs::PointCloud2 points_msg2;

  PointCloud tempCloud;
   PointCloud result1;
  PointCloud result2;

  //Process the cloud (example)
 
  pcl_ros::transformPointCloud("odom_cam",*msg,tempCloud,*tfListener);

 /* accumulatedCloud = accumulatedCloud + tempCloud;


  std::cerr << "PointCloud before filtering: " << accumulatedCloud.width * accumulatedCloud.height 
       << " data points (" << pcl::getFieldsList (accumulatedCloud) << ").";


  //Downsample voxel-grid
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (accumulatedCloud.makeShared());
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (accumulatedCloud);

  std::cerr << "PointCloud after filtering: " << accumulatedCloud.width * accumulatedCloud.height 
       << " data points (" << pcl::getFieldsList (accumulatedCloud) << ").";*/


  filterHeight(tempCloud,result1,-0.15);
 // getCylinder(result1,result2,0.2);
  getPlane(result1,result2);
  removeOutliers(result2,result1);

/*  getPlane(tempCloud,result1);
  getPlane(result1,result2);
  getCylinder(result2,result1,0.5);*/
  
  
  pcl::toROSMsg(result1,points_msg2);

  //points_msg2.header =msg->header;
  points_msg2.header.frame_id="odom_cam";


  //Send the result to other ROS nodes
  pub_points_.publish(points_msg2);


}





int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "accum_pcl");

  ros::NodeHandle nh;

  tfListener = new tf::TransformListener();

  //Subscribe the node to the point cloud from the ROS bag file
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);

  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("processedPoints",  1);

  ros::Rate r(10.0);
  while (ros::ok())
  {
  	ros::spinOnce();                   // Handle ROS events
  	r.sleep();
  }

  delete tfListener;

  
}
