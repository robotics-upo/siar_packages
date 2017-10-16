

#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>





//void getCylinder(pcl::PointCloud<pcl::PointXYZ> cloud, sensor_msgs::PointCloud2 & points_msg);

void getCylinder(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ> & points_msg, double radius);
void filterHeight(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ> & points_msg, double height);
void getPlane(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ> & points_msg);
void removeOutliers(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ> & points_msg);
void detectGround(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ> & points_msg);
void generateNT120A(pcl::PointCloud<pcl::PointXYZ> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z);
void generateT133(pcl::PointCloud<pcl::PointXYZ> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z);
void generateT164(pcl::PointCloud<pcl::PointXYZ> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z);
void generateT181(pcl::PointCloud<pcl::PointXYZ> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z);
void generateD1400(pcl::PointCloud<pcl::PointXYZ> & points_msg, double ang_resolution, double length, double z_resolution,double offset_z);
void generateGenericSection(pcl::PointCloud<pcl::PointXYZ> & points_msg, double ang_resolution, double length, double z_resolution, double offset_z,double radius_gutter, double corridor,double radius_roof, double height);






