#ifndef __SIARCOSTMAP_HPP__
#define __SIARCOSTMAP_HPP__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>

class SiarCostmap
{
	
public:

	// Class constructor
    SiarCostmap(std::string nodeName)
    {
		// Load parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("hz", m_hz))
			m_hz = 10.0;
		if(!lnh.getParam("obstacle_height", m_obstacleHeight))
			m_obstacleHeight = 0.05;
		if(!lnh.getParam("exp_decay", m_expDecay))
			m_expDecay = 0.0;
		if(!lnh.getParam("resolution", m_resolution))
			m_resolution = 0.05;
		if(!lnh.getParam("width", m_width))
			m_width = 3.0;
		if(!lnh.getParam("height", m_height))
			m_height = 4.0;
		if(!lnh.getParam("base_frame_id", m_baseFrameId))
			m_baseFrameId = "base_link";

		// Setup subscription to sensor data
		m_sub0 = m_nh.subscribe("cloud0", 1, &SiarCostmap::cloud0Callback, this);
		m_sub1 = m_nh.subscribe("cloud1", 1, &SiarCostmap::cloud1Callback, this);
		m_sub2 = m_nh.subscribe("cloud2", 1, &SiarCostmap::cloud2Callback, this);
		m_sub3 = m_nh.subscribe("cloud3", 1, &SiarCostmap::cloud3Callback, this);
		m_sub4 = m_nh.subscribe("cloud4", 1, &SiarCostmap::cloud4Callback, this);
		m_sub5 = m_nh.subscribe("cloud5", 1, &SiarCostmap::cloud5Callback, this);
		for(int i=0; i<6; i++)
			m_cloudNew[i] = false;
	
		// Setup publisher
		m_pub = m_nh.advertise<nav_msgs::OccupancyGrid>(nodeName+"/costmap", 0);
		
		// Setup timer for data processing
		timer = m_nh.createTimer(ros::Duration(1.0/m_hz), &SiarCostmap::updateTimer, this);
		
		// Initialize costmap
		m_costmap.header.frame_id = m_baseFrameId;
		m_costmap.header.seq = 0;
		m_costmap.header.stamp = ros::Time::now();
		m_costmap.info.map_load_time = m_costmap.header.stamp;
		m_costmap.info.resolution = m_resolution;
		m_costmap.info.width = (int)(m_width/m_resolution);
		m_costmap.info.height = (int)(m_height/m_resolution);
		m_costmap.info.origin.position.x = -m_height/2.0;
		m_costmap.info.origin.position.y = m_width/2.0;
		m_costmap.info.origin.position.z = 0.0;
		m_costmap.info.origin.orientation.x = 0.0; 
		m_costmap.info.origin.orientation.y = 0.0; 
		m_costmap.info.origin.orientation.z = -0.7071;
		m_costmap.info.origin.orientation.w = 0.7071;	
		m_costmap.data.resize(m_costmap.info.width*m_costmap.info.height);	
		m_maxX = m_height/2.0;
		m_minX = -m_height/2.0;
		m_maxY = m_width/2.0;
		m_minY = -m_width/2.0;
		m_divRes = 1/m_resolution;
    }
    
    // Class destructor
    ~SiarCostmap(void)
    {
	}
	
	nav_msgs::OccupancyGrid &updateCostmap(void)
	{
		// Time stamp
		ros::Time t = ros::Time::now();
		
		// Initialize all cells in the grid to unkown value
		for(int i=0; i<m_costmap.data.size(); i++)
			m_costmap.data[i] = 0;//-128;
		
		// Process the latest cloud received in each topic
		for(int i=0; i<6; i++)
		{
			// Check if we have new data
			if(!m_cloudNew[i])
				continue;
			
			// Transform point-cloud to base frame
			sensor_msgs::PointCloud2 cloud;
			try
			{
				pcl_ros::transformPointCloud(m_baseFrameId, m_cloudTf[i], m_cloud[i], cloud);
			}
			catch (tf::TransformException ex)
			{
				continue;
			} 
			
			// Evaluate every point into the cloud	
			sensor_msgs::PointCloud2Iterator<float> iterX(cloud, "x");
			sensor_msgs::PointCloud2Iterator<float> iterY(cloud, "y");
			sensor_msgs::PointCloud2Iterator<float> iterZ(cloud, "z");
			for(int i=0; i<cloud.width; i++, ++iterX, ++iterY, ++iterZ)
			{
				float x = *iterX, y = *iterY, z = *iterZ;
				if(x > m_minX && x < m_maxX && y > m_minY && y < m_maxY)
				{
					int index = point2index(x,y);
					if(m_costmap.data[index] < 127)
					{
						if(fabs(z) > m_obstacleHeight)
							m_costmap.data[index] = 127;
						else
							m_costmap.data[index] = (int8_t)(z*100.0);
					}
				}
			}
		}
		
		// Update costmap info
		m_costmap.header.seq++;
		m_costmap.header.stamp = t;
		m_costmap.info.map_load_time = t;
		
		// Prepare next sensor reading
		for(int i=0; i<6; i++)
			m_cloudNew[i] = false;
			
		return m_costmap;
	}
	
	nav_msgs::OccupancyGrid &getCostmap(void)
	{
		return m_costmap;
	}
	
private:

    void cloud0Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
		processCloud(msg, 0);
	}
	
	void cloud1Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
		processCloud(msg, 1);
	}
	
	void cloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
		processCloud(msg, 2);
	}
	
	void cloud3Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
		processCloud(msg, 3);
	}
	
	void cloud4Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
		processCloud(msg, 4);
	}
	
	void cloud5Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
		processCloud(msg, 5);
	}
	
	void updateTimer(const ros::TimerEvent& event)
	{
		// Update costmap
		updateCostmap();
		
		// Publish costmap
		m_pub.publish(m_costmap);
	}
	
	void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg, int id)
	{
		static bool gotTf[6] = {false, false, false, false, false, false};
		
		if(!gotTf[id])
		{
			try
			{
				m_tfListener.lookupTransform(m_baseFrameId, msg->header.frame_id, ros::Time(0), m_cloudTf[id]);
				gotTf[id] = true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("Error: %s", ex.what());
			}
		}
		else
		{
			m_cloud[id] = *msg;
			m_cloudNew[id] = true;
		}
	}
	
	inline int point2index(float &x, float &y)
	{
		//return (m_costmap.info.height - (int)((x-m_minX)*m_divRes))*m_costmap.info.width + m_costmap.info.width - (int)((y-m_minY)*m_divRes);
		return ((int)((x-m_minX)*m_divRes))*m_costmap.info.width + (int)((y-m_minY)*m_divRes);
	}
	
	// Params
	double m_hz, m_obstacleHeight, m_expDecay;
	double m_width, m_height, m_resolution;
	std::string m_baseFrameId;
	
	// ROS stuff
	ros::NodeHandle m_nh;	
	ros::Timer timer;
	tf::TransformListener m_tfListener;
	ros::Subscriber m_sub0, m_sub1, m_sub2, m_sub3, m_sub4, m_sub5;
	ros::Publisher m_pub;

	// Sensor data
	sensor_msgs::PointCloud2 m_cloud[6];
	bool m_cloudNew[6];
	tf::StampedTransform m_cloudTf[6];
	
	// Compute costmap
	float m_minX, m_maxX, m_minY, m_maxY;
	float m_divRes;
	nav_msgs::OccupancyGrid m_costmap; 
};


#endif

