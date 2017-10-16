#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <visualization_msgs/Marker.h>

#include <pcl_ros/transforms.h>

#include "siar_map_analysis.h"


//Publishers
ros::Publisher pub_points_;
ros::Publisher vis_pub;

visualization_msgs::Marker section_marker;



SIAR_map_analysis map_analysis;	//Map analysis object

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
 
  sensor_msgs::PointCloud2 points_msg2;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> finalCloud;

  pcl::fromROSMsg (*msg, cloud);

  map_analysis.setInputCloud(cloud);

  //Estimate Section Type
  std::string type = map_analysis.estimateSectionType();

  std::cout << "Section type: " << type << std::endl;

  //Compute defects for a given section
  if(map_analysis.computeDefectsCustom(finalCloud)>=0)
  {

  	pcl::toROSMsg(finalCloud,points_msg2);

  	//points_msg2.header =msg->header;
  	points_msg2.header.stamp =msg->header.stamp;
  	points_msg2.header.frame_id=msg->header.frame_id;//"back_depth_optical_frame";


	  //Send the result to other ROS nodes
	  pub_points_.publish(points_msg2);


	  section_marker.header.frame_id = msg->header.frame_id;//"back_depth_optical_frame";
	  section_marker.header.stamp = msg->header.stamp;
	  section_marker.text = type;

	  //Publish marker with the section
	  vis_pub.publish( section_marker );

  }



}





int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "detect_pcl");

  ros::NodeHandle nh;

	map_analysis.setMinThresholdDefects(0.005);
	map_analysis.setMaxThresholdDefects(0.04);
	map_analysis.setGlobalAlignThreshold(0.02);

  
  section_marker.id = 0;
  section_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  section_marker.action = visualization_msgs::Marker::ADD;
  section_marker.pose.position.x = 0;
  section_marker.pose.position.y = 0;
  section_marker.pose.position.z = 2;
  section_marker.pose.orientation.x = 0.0;
  section_marker.pose.orientation.y = 0.0;
  section_marker.pose.orientation.z = 0.0;
  section_marker.pose.orientation.w = 1.0;
  section_marker.scale.x = 1.0;
  section_marker.scale.y = 0.1;
  section_marker.scale.z = 0.5;
  section_marker.color.a = 1.0; // Don't forget to set the alpha!
  section_marker.color.r = 0.0;
  section_marker.color.g = 1.0;
  section_marker.color.b = 0.0;

   
  //Subscribe the node to the point cloud from the ROS bag file. The topic has to be remapped to points2
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("points2", 1, callback);

  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("processedPoints",  1);

  vis_pub = nh.advertise<visualization_msgs::Marker>( "section_type", 1 );

 /* ros::Rate r(10.0);
  while (ros::ok())
  {
  	ros::spinOnce();                   // Handle ROS events
  	r.sleep();
  }*/

  ros::spin();

  
}
