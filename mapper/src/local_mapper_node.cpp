#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

struct MapPiece
{
	// Camera to robot transform
	tf::StampedTransform cam2base;
	
	// Robot to map transform 
	tf::StampedTransform base2map;
	
	// Point-cloud 
	sensor_msgs::PointCloud2 pc;
	
	MapPiece(void) 
	{
	}
	
	MapPiece(tf::StampedTransform &cam2base_, tf::StampedTransform &base2map_, sensor_msgs::PointCloud2 &pc_) 
	{
		cam2base = cam2base_;
		base2map = base2map_;
		pc = pc_;
	}
			
	MapPiece(const MapPiece &d) 
	{
		cam2base = d.cam2base;
		base2map = d.base2map;
		pc = d.pc;
	}
	
	MapPiece& operator=(const MapPiece& d)
	{
		if(this != &d)
		{
			cam2base = d.cam2base;
			base2map = d.base2map;
			pc = d.pc;
		}
		return *this;
	}
};

// Process global variables
tf::TransformListener *g_tfListener;
std::string g_baseFrame, g_mapFrame;
double g_minTranslation, g_camMaxRange; 
int g_maxPoses;
tf::StampedTransform g_lastTf;
std::vector<MapPiece> map;
bool g_newPiece;

// point cloud callback
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{	
	MapPiece p;
	
	// Get the base to map frame transform
	try
	{
		g_tfListener->lookupTransform(g_mapFrame, g_baseFrame, ros::Time(0), p.base2map);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Error: %s", ex.what());
		return;
	}
	
	// Check if the robot traversed more than minTranslation
	tf::Transform t = g_lastTf.inverse()*p.base2map;
    if(t.getOrigin().length() < g_minTranslation)
    {
		return;
    }
	
	// Get the camera to base frame transform
	try
	{
		g_tfListener->lookupTransform(g_baseFrame, msg->header.frame_id, ros::Time(0), p.cam2base);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Error: %s", ex.what());
		return;
	}
	
	// Transform point clouds into base frame
	try
	{
		pcl_ros::transformPointCloud(g_baseFrame, p.cam2base, *msg, p.pc);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Error: %s", ex.what());
		return;
	}
	
	// We only keep points at a given distance from the base frame (far points are noisy)
	double r = g_camMaxRange*g_camMaxRange;
	sensor_msgs::PointCloud pc;
	geometry_msgs::Point32 pt;
	pc.header = p.pc.header;
	sensor_msgs::PointCloud2Iterator<float> iterX(p.pc, "x");
	sensor_msgs::PointCloud2Iterator<float> iterY(p.pc, "y");
	sensor_msgs::PointCloud2Iterator<float> iterZ(p.pc, "z");
	for(int i=0; i<p.pc.width; i++, ++iterX, ++iterY, ++iterZ)
	{
		pt.x = *iterX;
		pt.y = *iterY;
		pt.z = *iterZ;
		if(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z < r)
			pc.points.push_back(pt);
	}
	sensor_msgs::convertPointCloudToPointCloud2(pc, p.pc);
	
	// Transform points to map
	try
	{
		sensor_msgs::PointCloud2 aux = p.pc;
		pcl_ros::transformPointCloud(g_mapFrame, p.base2map, aux, p.pc);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Error: %s", ex.what());
		return;
	}
	
	// Store current map piece
	map.push_back(p);
	g_newPiece = true;
	
	// Update last robot pose
	g_lastTf = p.base2map;
	
	// Remove poses if we have more than maxPoses
	while(map.size() > g_maxPoses)
		map.erase(map.begin());
		
	return;
}

void addPointCloud(sensor_msgs::PointCloud &map, sensor_msgs::PointCloud2 &input)
{
	geometry_msgs::Point32 pt;
	sensor_msgs::PointCloud2Iterator<float> iterX(input, "x");
	sensor_msgs::PointCloud2Iterator<float> iterY(input, "y");
	sensor_msgs::PointCloud2Iterator<float> iterZ(input, "z");
	for(int i=0; i<input.width; i++, ++iterX, ++iterY, ++iterZ)
	{
		pt.x = *iterX;
		pt.y = *iterY;
		pt.z = *iterZ;
		map.points.push_back(pt);
	}	
}

int main( int argc, char **argv)
{
	// Setup ROS
	ros::init(argc, argv, "local_mapper_node");
	ros::NodeHandle nh, lnh("~");
	
	// Read parameters	
	if(!lnh.getParam("min_translation", g_minTranslation))
		g_minTranslation = 0.25;
	if(!lnh.getParam("max_poses", g_maxPoses))
		g_maxPoses = 11;
	if(g_maxPoses%2 == 0)
		g_maxPoses++;
	if(!lnh.getParam("cam_max_range", g_camMaxRange))
		g_camMaxRange = 4.0;	
	if(!lnh.getParam("base_frame_id", g_baseFrame))
		g_baseFrame = "base_link";
	if(!lnh.getParam("map_frame_id", g_mapFrame))
		g_mapFrame = "map";
	
	// Initilize robot pose
	bool cached = false;
	g_tfListener = new tf::TransformListener;
	while(!cached)
	{
		try
		{
			g_tfListener->lookupTransform(g_mapFrame, g_baseFrame, ros::Time(0), g_lastTf);
			cached = true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("Error: %s",ex.what());
		}
		usleep(100000);
		ros::spinOnce();
	}
	
	// Create the requiered subscriptions
	ros::Subscriber sub = nh.subscribe("cloud", 1, &cloudCallback);
	ros::Publisher  pub = nh.advertise<sensor_msgs::PointCloud2>("/local_mapper_node/map", 1);
	
	// Spin for ever
	int seq = 0;
	g_newPiece = false;
	while(ros::ok())
	{
		// Sleeps and update ros
		usleep(100000);
		ros::spinOnce();
		
		// Check if there is a new map piece
		if(!g_newPiece)
			continue;
		g_newPiece = false;
		
		// Select the middle point
		int center = (int)(map.size()/2.0);
		
		// Create the local map
		sensor_msgs::PointCloud pc;
		//addPointCloud(pc, map[center].pc);
		for(int i=0; i<(int)map.size(); i++)
		{
			//if(i == center)
			//	continue;
								
			// Get the transform relative to the center pose
			//tf::Transform t = map[center].base2map.inverse()*map[i].base2map;
			//std::cout << "Processing piece " << i << ". Frame: " << map[i].pc.header.frame_id << ". Size: " << map[i].pc.width << ". Translation: " << t.getOrigin().length() << std::endl; 
			
			// Transform point-cloud
			sensor_msgs::PointCloud2 pc2;
			try
			{
				pcl_ros::transformPointCloud(g_baseFrame, map[center].base2map.inverse(), map[i].pc, pc2);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("Error: %s", ex.what());
				continue;
			}
			
			// Increase map
			addPointCloud(pc, pc2);
		}
		
		// Publish the new map
		pc.header.frame_id = g_baseFrame;
		pc.header.stamp = ros::Time::now();
		pc.header.seq = seq++;
		sensor_msgs::PointCloud2 outCloud;
		sensor_msgs::convertPointCloudToPointCloud2(pc, outCloud);
		pub.publish(outCloud);
	}
    
	return 0;
}
