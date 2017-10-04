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

#define PI 3.14159265359


ros::Publisher pub_points_;
ros::Publisher pub_points_data;
ros::Publisher pub_points_ref;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
PointCloud pc_output;
PointCloud::Ptr pc_ref (new PointCloud);
PointCloud::Ptr pc_data (new PointCloud);
int N = 1000;
int M = 10;
float depth = 0.2;
int center_z = 2.5;

PointCloud cloud;

  
void sectionD1400()
{
  for(int i=0; i<N; i++)
  {
    for(int j=0; j<M; j++)
    {
      pc_ref->points[i*M+j].x = 1.4*sin(2*PI*i/N);
      pc_ref->points[i*M+j].y = 1.4*cos(2*PI*i/N);
      pc_ref->points[i*M+j].z = center_z + depth * j / M;   
    }
  }
}

void sectionNT120A()
{
  for(int i=0; i<N/8; i++)
  {
    float k = i/(N/8.0);
    for(int j=0; j<M; j++)
    {
      
      pc_ref->points[i*M+j].x = 0.2*sin(PI/2*k);
      pc_ref->points[i*M+j].y = 0.2*cos(PI/2*k);
      pc_ref->points[i*M+j].z = center_z + depth * j / M;   

      pc_ref->points[(i+N/8)*M+j].x = 0.2+k*0.35/2.0;
      pc_ref->points[(i+N/8)*M+j].y = 0;
      pc_ref->points[(i+N/8)*M+j].z = center_z + depth * j / M;   

      pc_ref->points[(i+2*N/8)*M+j].x = 0.75/2+k*0.25/2.0;
      pc_ref->points[(i+2*N/8)*M+j].y = -k*0.85;
      pc_ref->points[(i+2*N/8)*M+j].z = center_z + depth * j / M;
      
 
      pc_ref->points[(i+3*N/8)*M+j].x = 0.5*sin(PI/2*k);
      pc_ref->points[(i+3*N/8)*M+j].y = -(0.5*cos(PI/2*k)+0.85);
      pc_ref->points[(i+3*N/8)*M+j].z = center_z + depth * j / M;   
    
      pc_ref->points[(i+N/2)*M+j].x = -pc_ref->points[i*M+j].x;
      pc_ref->points[(i+N/2)*M+j].y = pc_ref->points[i*M+j].y;
      pc_ref->points[(i+N/2)*M+j].z = pc_ref->points[i*M+j].z;  
      
      pc_ref->points[(i+N/2+N/8)*M+j].x = -pc_ref->points[(i+N/8)*M+j].x;
      pc_ref->points[(i+N/2+N/8)*M+j].y = pc_ref->points[(i+N/8)*M+j].y;
      pc_ref->points[(i+N/2+N/8)*M+j].z = pc_ref->points[(i+N/8)*M+j].z;
      
      pc_ref->points[(i+N/2+2*N/8)*M+j].x = -pc_ref->points[(i+2*N/8)*M+j].x;
      pc_ref->points[(i+N/2+2*N/8)*M+j].y = pc_ref->points[(i+2*N/8)*M+j].y;
      pc_ref->points[(i+N/2+2*N/8)*M+j].z = pc_ref->points[(i+2*N/8)*M+j].z;
      
      pc_ref->points[(i+N/2+3*N/8)*M+j].x = -pc_ref->points[(i+3*N/8)*M+j].x;
      pc_ref->points[(i+N/2+3*N/8)*M+j].y = pc_ref->points[(i+3*N/8)*M+j].y;
      pc_ref->points[(i+N/2+3*N/8)*M+j].z = pc_ref->points[(i+3*N/8)*M+j].z;
    }
  }
}
 

void sectionT133()
{
  for(int i=0; i<N/8; i++)
  {
    float k = i/(N/8.0);
    for(int j=0; j<M; j++)
    {
      
      pc_ref->points[i*M+j].x = 0.2*sin(PI/2*k);
      pc_ref->points[i*M+j].y = 0.2*cos(PI/2*k);
      pc_ref->points[i*M+j].z = center_z + depth * j / M;   

      pc_ref->points[(i+N/8)*M+j].x = 0.2+k*0.4/2.0;
      pc_ref->points[(i+N/8)*M+j].y = 0;
      pc_ref->points[(i+N/8)*M+j].z = center_z + depth * j / M;   

      pc_ref->points[(i+2*N/8)*M+j].x = 0.8/2+k*0.2/2.0;
      pc_ref->points[(i+2*N/8)*M+j].y = -k;
      pc_ref->points[(i+2*N/8)*M+j].z = center_z + depth * j / M;
      
 
      pc_ref->points[(i+3*N/8)*M+j].x = 0.5*sin(PI/2*k);
      pc_ref->points[(i+3*N/8)*M+j].y = -(0.5*cos(PI/2*k)+1.0);
      pc_ref->points[(i+3*N/8)*M+j].z = center_z + depth * j / M;   
    
      pc_ref->points[(i+N/2)*M+j].x = -pc_ref->points[i*M+j].x;
      pc_ref->points[(i+N/2)*M+j].y = pc_ref->points[i*M+j].y;
      pc_ref->points[(i+N/2)*M+j].z = pc_ref->points[i*M+j].z;  
      
      pc_ref->points[(i+N/2+N/8)*M+j].x = -pc_ref->points[(i+N/8)*M+j].x;
      pc_ref->points[(i+N/2+N/8)*M+j].y = pc_ref->points[(i+N/8)*M+j].y;
      pc_ref->points[(i+N/2+N/8)*M+j].z = pc_ref->points[(i+N/8)*M+j].z;
      
      pc_ref->points[(i+N/2+2*N/8)*M+j].x = -pc_ref->points[(i+2*N/8)*M+j].x;
      pc_ref->points[(i+N/2+2*N/8)*M+j].y = pc_ref->points[(i+2*N/8)*M+j].y;
      pc_ref->points[(i+N/2+2*N/8)*M+j].z = pc_ref->points[(i+2*N/8)*M+j].z;
      
      pc_ref->points[(i+N/2+3*N/8)*M+j].x = -pc_ref->points[(i+3*N/8)*M+j].x;
      pc_ref->points[(i+N/2+3*N/8)*M+j].y = pc_ref->points[(i+3*N/8)*M+j].y;
      pc_ref->points[(i+N/2+3*N/8)*M+j].z = pc_ref->points[(i+3*N/8)*M+j].z;
    }
  }
}
 




void callback(const PointCloud::ConstPtr& msg)
{

  std::vector<int> indices; 


  //pc_data = msg;
  
  pc_data->width    = msg->width ;
  pc_data->height   = msg->height;
  pc_data->is_dense = msg->is_dense;
  pc_data->points.resize (pc_data->width * pc_data->height);
  
  for (size_t i = 0; i < msg->points.size(); ++i)
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

/*  pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
  icp.setMaximumIterations(100);
  icp.setInputSource(pc_data);
  icp.setInputTarget(pc_ref);
  icp.align(pc_output);
*/    
    
  pub_points_.publish(pc_output);

  pub_points_data.publish(pc_data);
}




int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  //Subscribe the node to the point cloud from the ROS bag file
  
  
 // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("pcl_ref/t181.pcd", cloud) == -1) //* load the file
 // {
 //   PCL_ERROR("Couldn't read file pcl_ref/t181.pcd \n");
 //   return (-1);
 // }
	      
  pc_ref->header.frame_id = "/front_rgb_optical_frame";
  pc_ref->width = N ;
  pc_ref->height = M;
  pc_ref->is_dense = true;
  pc_ref->points.resize (pc_ref->width * pc_ref->height);
  
  pc_data->header.frame_id = "/front_rgb_optical_frame";

  
  //Publish the resultant point cloud
  pub_points_ = nh.advertise< PointCloud >("output_pointcloud",  1);
  ros::Subscriber sub = nh.subscribe< PointCloud >("/front/points", 1, callback);  
  
  
  pub_points_data = nh.advertise< PointCloud >("data_pointcloud",  1);
  pub_points_ref = nh.advertise< PointCloud >("ref_pointcloud",  1);

  //sectionD1400();
  //sectionNT120A();
  sectionT133();
  
  while(ros::ok())
  {
    ros::spinOnce();
    pub_points_ref.publish(pc_ref);

  }
  
  
  return(0);
}
