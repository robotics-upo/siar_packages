#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "processPointCloud.h"


//tf::TransformListener tfListener;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_points_;



void callback(const PointCloud::ConstPtr& msg)
{
 // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  
  ///! ---       ------------- Parte de representacion, muestra cada punto por pantalla (simplemente para ver que funciona)
 // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
 //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

  //! ----------------- Inserta tu código aquí para procesar la nube de puntos (se llama a esta funcion cada vez q se recibe una nube de puntos)

  sensor_msgs::PointCloud2 points_msg2;
  PointCloud result1;
  PointCloud result2;

  //Process the cloud (example)
 // getCylinder(*msg,points_msg2);
   // getCylinder(*msg,result1,0.55);
    //getCylinder(result1,result2,0.2);

  
  
  
    result2 = *msg;
  
  

  
  
  //Convert the result into a ROS msg
  pcl::toROSMsg(result2, points_msg2);

  //Send the result to other ROS nodes
  pub_points_.publish(points_msg2);


}

int main(int argc, char** argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  //Subscribe the node to the point cloud from the ROS bag file
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);

  
  
  
  //Publish the resultant point cloud
  pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("processedPoints",  1);;

  ros::spin();
}
