#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "processPointCloud.h"
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include "iostream"
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_points_;
ros::Publisher pub_points_data;
ros::Publisher pub_points_ref;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
PointCloud pc_output;
PointCloud::Ptr pc_ref (new PointCloud);
PointCloud::Ptr pc_data (new PointCloud);


PointCloud cloud;

  
void callback(const PointCloud::ConstPtr& msg)
{

  std::vector<int> indices; 


  //pc_data = msg;
  
  pc_data->width    = msg->width ;
  pc_data->height   = msg->height;
  pc_data->is_dense = msg->is_dense;
  pc_data->points.resize (pc_data->width * pc_data->height);
  
  for (size_t i = 0; i < msg->points.size (); ++i)
  {
    if(pc_data->points[i].z > 3.5) //max distance to search
    { 
      pc_data->points[i].x = 0.0/0.0; //result NaN value
      pc_data->points[i].y = 0.0/0.0;
      pc_data->points[i].z = 0.0/0.0;
    }
    else
    {
      pc_data->points[i].x = msg->points[i].x;
      pc_data->points[i].y = msg->points[i].y;
      pc_data->points[i].z = msg->points[i].z;
    }
  }

//  pcl::removeNaNFromPointCloud(*pc_data,*pc_data,indices); 

  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //pcl::VoxelGrid<PointCloud> sor;

  pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
  icp.setMaximumIterations(1);
  icp.setInputSource(pc_data);
  icp.setInputTarget(pc_ref);
  icp.align(pc_output);
    
    
    /*   sor.setInputCloud(pc_data);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*pc_data);
  
  sor.setInputCloud(pc_ref);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter(*pc_ref);  
  
  icp.setInputSource(pc_ref);
  icp.setInputTarget(pc_data);
  icp.align(pc_output);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
*/
    pub_points_.publish(pc_output);

  
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
	      
  pc_ref->header.frame_id = "/front_rgb_optical_frame";
  pc_data->header.frame_id = "/front_rgb_optical_frame";

  
  //Publish the resultant point cloud
  pub_points_ = nh.advertise< PointCloud >("output_pointcloud",  1);
  ros::Subscriber sub = nh.subscribe< PointCloud >("front/points", 1, callback);  
  
  
  pub_points_data = nh.advertise< PointCloud >("data_pointcloud",  1);
  pub_points_ref = nh.advertise< PointCloud >("ref_pointcloud",  1);

  
  
  while(ros::ok())
  {
    ros::spinOnce();
  }
  
  
  return(0);
}
