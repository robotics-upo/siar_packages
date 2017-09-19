

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





