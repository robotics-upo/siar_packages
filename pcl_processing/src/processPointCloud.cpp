
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>


#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <boost/make_shared.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/cloud_viewer.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

//#include <pcl/segmentation/progressive_morphological_filter.h>

#include "processPointCloud.h"

#include <pcl_ros/transforms.h>


typedef pcl::PointXYZ PointT;



extern tf::TransformListener tfListener;

//PCL viewer. This is just an example. It seems that this is not the best option for a multi-threaded program
//The cloud can be also seen in RViz
//pcl::visualization::CloudViewer viewer_("view");

/**
* This function receives a PCL point cloud and segment a cylinder. The results is returned as a PointCloud msg of ROS
* that can be sent to other modules
*/

//void getCylinder(pcl::PointCloud<PointT> cloud, sensor_msgs::PointCloud2 & points_msg)
void getCylinder(pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> & points_msg, double radius)
{

  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);


  std::cerr << "PointCloud has: " << cloud.points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
 /* pass.setInputCloud (cloud.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10.0, 10.0);
 // pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;*/

  cloud_filtered = cloud.makeShared();
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  //pcl::toROSMsg(*cloud_normals,points_msg);



  // Create the segmentation object for the planar model and set all the parameters
  /*seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);*/

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10);
  seg.setDistanceThreshold (0.1);
  seg.setRadiusLimits (radius-0.05, radius+0.05);
  //seg.setInputCloud (cloud_filtered2);
  //seg.setInputNormals (cloud_normals2);

   seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to a ROS msg
  //extract.setInputCloud (cloud_filtered2);
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (true);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	 
  }

 //viewer_.showCloud(cloud_cylinder);

  points_msg = *cloud_cylinder;

  //Convert the result into a ROS msg
  //pcl::toROSMsg(*cloud_cylinder,points_msg);


}



//void getCylinder(pcl::PointCloud<PointT> cloud, sensor_msgs::PointCloud2 & points_msg)
void filterHeight(pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> & points_msg, double height)
{

  pcl::PassThrough<PointT> pass;
  
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  
  std::cerr << "PointCloud has: " << cloud.points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (height-0.1, height+0.1);
 // pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

 
  points_msg = *cloud_filtered;


}



void getPlane(pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> & points_msg)
{

  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);


  std::cerr << "PointCloud has: " << cloud.points.size () << " data points." << std::endl;



  cloud_filtered = cloud.makeShared();
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10);
  seg.setDistanceThreshold (0.05);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  

  // Remove the planar inliers, extract the rest

  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.setNegative (true);
  extract.filter (*cloud_plane);
  

  //viewer_.showCloud(cloud_plane);

  points_msg = *cloud_plane;

  //Convert the result into a ROS msg
  //pcl::toROSMsg(*cloud_cylinder,points_msg);


}


void removeOutliers(pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> & points_msg)
{

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud.makeShared());
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
  sor.filter (points_msg);

  /*pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
  outrem.setInputCloud(cloud.makeShared());
  outrem.setRadiusSearch(0.25);
  outrem.setMinNeighborsInRadius (10);
    // apply filter
  outrem.filter(points_msg);*/


}

void generateNT120A(pcl::PointCloud<PointT> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z)
{
	double radius_gutter = 0.20;
	double corridor = 0.18;
	double radius_roof = 0.5;
	double height = 0.85;

	generateGenericSection(points_msg,ang_resolution,length,z_resolution,offset_z,radius_gutter,corridor,radius_roof,height);

	

}




void generateT164(pcl::PointCloud<PointT> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z)
{
	double radius_gutter = 0.20;
	double corridor = 0.40;
	double radius_roof = 0.6;
	double height = 0.9;

	generateGenericSection(points_msg,ang_resolution,length,z_resolution,offset_z,radius_gutter,corridor,radius_roof,height);

}

void generateT181(pcl::PointCloud<PointT> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z)
{
	double radius_gutter = 0.20;
	double corridor = 0.40;
	double radius_roof = 0.6;
	double height = 1.0;

	generateGenericSection(points_msg,ang_resolution,length,z_resolution,offset_z,radius_gutter,corridor,radius_roof,height);

}

void generateT133(pcl::PointCloud<PointT> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z)
{
	double radius_gutter = 0.20;
	double corridor = 0.20;
	double radius_roof = 0.5;
	double height = 1.0;

	generateGenericSection(points_msg,ang_resolution,length,z_resolution,offset_z,radius_gutter,corridor,radius_roof,height);

}


void generateD1400(pcl::PointCloud<PointT> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z)
{

	double radius_roof = 0.7;

	double angle = 0.0;
	int n_points = 2*M_PI/ang_resolution;

	points_msg.clear();


	std::vector<PointT> slice;
	double x,y;

	for(int i=0;i<n_points;i++)
	{
		double a = angle + ang_resolution*i;

		if(a>=(3.0/2.0-1.0/9.0)*M_PI && a<=(3.0/2.0+1.0/9.0)*M_PI)
			continue;

		x = radius_roof*cos(a);
		y = radius_roof*sin(a); 

		PointT p;
		p.x=x;
		p.y=-y;
		p.z=0.0;
		slice.push_back(p);

	}

	int z_points = length / z_resolution;

	for(int j=0; j < z_points; j++)
	{
		for(int i=0; i < slice.size(); i++)
		{
			PointT aux = slice[i];

			aux.z += j*z_resolution + offset_z;
			points_msg.points.push_back(aux);

		}

	}
	

}

void generateGenericSection(pcl::PointCloud<PointT> & points_msg, double ang_resolution, double length, double z_resolution, double offset_z,
				double radius_gutter, double corridor,double radius_roof, double height)
{

	double daux = radius_roof - (radius_gutter + corridor);

	double angle = 0.0;
	int n_points = 2*M_PI/ang_resolution;

	points_msg.clear();

	double alpha1 = atan2(height,radius_gutter + corridor);
	double alpha3 = atan2(radius_gutter,height);
	double alpha2 = M_PI/2.0-(alpha1 + alpha3);

	std::vector<PointT> slice;
	double x,y;

	for(int i=0;i<n_points;i++)
	{
		double a = angle + ang_resolution*i;

		if (a>=0.0 && a <= M_PI)
		{
			x = radius_roof*cos(a);
			y = radius_roof*sin(a); 

		}else if (a>M_PI && a<= M_PI + alpha1)
		{
			double B = height/daux;
			double A = -B*(radius_gutter+corridor)-height;
			double r = A/(sin(a)+B*cos(a));

			x = r*cos(a);
			y = r*sin(a);
			

		}else if((a>M_PI + alpha1 && a <= M_PI + alpha1 + alpha2) || (a>3*M_PI/2 + alpha3 && a <= 3*M_PI/2 + alpha3 + alpha2))
		{

			//continue;
			double r = height/sin(a);

			x = r*cos(a);
			y= -height;
			
		}
		else if(a>3*M_PI/2+alpha3+alpha2 && a < 2*M_PI)
		{

			double B = height/daux;
			double A = -B*(radius_gutter+corridor)-height;

			double r = A/(sin(a)-B*cos(a));

			x = r*cos(a);
			y = r*sin(a);

		}else
			continue;
		
		PointT p;
		p.x=x;
		p.y=-y;
		p.z=0.0;
		slice.push_back(p);

	}

	int z_points = length / z_resolution;

	for(int j=0; j < z_points; j++)
	{
		for(int i=0; i < slice.size(); i++)
		{
			PointT aux = slice[i];

			aux.z += j*z_resolution + offset_z;
			points_msg.points.push_back(aux);

		}

	}

}




/*
PCL 1.8 required
void detectGround(pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> & points_msg)
{
	pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud.makeShared());
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (0.7f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared());
  extract.setIndices (ground);
  extract.setNegative (true);
  extract.filter (points_msg);




}*/








