#ifndef __SIARCOSTMAP_HPP__
#define __SIARCOSTMAP_HPP__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ANN/ANN.h>     

#define SC_POSITIVE_OBS 127
#define SC_NEGATIVE_OBS -127
#define SC_UNKNOWN -128

class SiarCostmap
{
  
public:

  // Point 2D
  struct Point2D
  {
    Point2D(void)
    {
    }
    
    Point2D(float _x, float _y)
    {
      x = _x;
      y = _y;
    }
    
    Point2D(const Point2D &d)
    {
      x = d.x;
      y = d.y;
    }
    
    float x;
    float y;
  };
  
  // Point pixel
  struct PointPix
  {
    PointPix(void)
    {
      x = 0;
      y = 0;
      positive = true;
    }
    
    PointPix(int _x, int _y)
    {
      x = _x;
      y = _y;
      positive = true;
    }
    
    PointPix(const PointPix &d)
    {
      x = d.x;
      y = d.y;
      positive = d.positive;
    }
    
    int x;
    int y;
    bool positive;
  };

  // Class constructor
    SiarCostmap(std::string nodeName)
    {
    // Load parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("hz", m_hz))
      m_hz = 10.0;
    if(!lnh.getParam("obstacle_height", m_obstacleHeight))
      m_obstacleHeight = 0.05;
    if(!lnh.getParam("obstacle_height_negative", m_obstacleHeightNeg))
      m_obstacleHeightNeg = -m_obstacleHeight;
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
    if(!lnh.getParam("tilt_compensate", m_tiltCompesante))
      m_tiltCompesante = false;
    if(!lnh.getParam("consider_sign", m_considerSign))
      m_considerSign = true;
    if(!lnh.getParam("inflate_negative", m_inflate_negative))
      m_inflate_negative = 1;
    if(!lnh.getParam("integrate_odom", m_integrateOdom))
      m_integrateOdom = false;
    if(!lnh.getParam("odom_frame_id", m_odomFrameId))
      m_odomFrameId = "odom"; 
      
    // NOTE: Changes for demo --> ignore obstacles exceeding a height
    if(!lnh.getParam("robot_height", m_robotHeight))
      m_robotHeight = 0.4;

    // Setup subscription to sensor data
    m_sub0 = m_nh.subscribe("cloud0", 1, &SiarCostmap::cloud0Callback, this);
    m_sub1 = m_nh.subscribe("cloud1", 1, &SiarCostmap::cloud1Callback, this);
    m_sub2 = m_nh.subscribe("cloud2", 1, &SiarCostmap::cloud2Callback, this);
    m_sub3 = m_nh.subscribe("cloud3", 1, &SiarCostmap::cloud3Callback, this);
    m_sub4 = m_nh.subscribe("cloud4", 1, &SiarCostmap::cloud4Callback, this);
    m_sub5 = m_nh.subscribe("cloud5", 1, &SiarCostmap::cloud5Callback, this);
    for(int i=0; i<6; i++)
      m_cloudNew[i] = false;
    m_sub6 = m_nh.subscribe("imu", 1, &SiarCostmap::imuCallback, this);
    m_imuNew = false;
	m_sub7 = m_nh.subscribe(nodeName+"/odom_integrate", 1, &SiarCostmap::odomIntegrateCallback, this);

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
    n_pix = m_costmap.info.width*m_costmap.info.height;
    m_costmap.data.resize(n_pix);  
    m_unknown_map.resize(n_pix);  
    m_maxX = m_height/2.0;
    m_minX = -m_height/2.0;
    m_maxY = m_width/2.0;
    m_minY = -m_width/2.0;
    m_divRes = 1/m_resolution;
    
    // Setup last odom transform in case of need
    m_integrateOdomInit = false;
  }
    
  // Class destructor
  ~SiarCostmap(void)
  {
  }
  
  bool updateCostmap(void)
  {
    // Time stamp
    ros::Time t = ros::Time::now();
    
    // Compute odometric trasnlation and rotation since last costmap update 
    // and also make a copy of the previous costmap before modification
        bool move_obstacles = false;
	if(m_integrateOdom)
	{
                for (size_t i = 0; i != m_unknown_map.size(); i++) 
                {
                  m_unknown_map[i] = true;
                }
                
		// Get the transform since last costmap update
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("SiarCostmap error: %s",ex.what());
			return false;
		}
		if(!m_integrateOdomInit)
		{
			m_lastOdomTf = odomTf;
			m_integrateOdomInit = true;
		}
// 		tf::Transform T = (m_lastOdomTf.inverse()*odomTf).inverse();
                tf::Transform T = odomTf.inverse()*m_lastOdomTf;
		
		// Get rotation and translation in 2D
		double yaw, pitch, roll;
		T.getBasis().getRPY(roll, pitch, yaw);
		float r00 = cos(yaw), r01 = sin(yaw);
		float r10 = -sin(yaw), r11 = cos(yaw); 
                
		
		// Get the transformed costamp if robot moved
		if(fabs(T.getOrigin().x()) > 0.025 || fabs(yaw) > 0.1)
		{
                        ROS_INFO("YAW = %f d_x=%f d_y=%f", yaw, T.getOrigin().x(), T.getOrigin().y());
                        move_obstacles = true;
			// Transform each obstacle considering the odometry
			for(size_t i=0; i<m_last_obstacles.size(); i++)
			{
                                PointPix &p = m_last_obstacles.at(i);
                                float x, y;
                                pix2point(x,y,p);
                                float x_ = T.getOrigin().x() + x * r00 + y * r01;
                                float y_ = T.getOrigin().y() + x * r10 + y * r11;
                                
                                if (i == m_last_obstacles.size() / 2) {
                                  ROS_INFO("Original point: %d ,%d   %f,%f", p.x, p.y, x, y);
                                }
                                point2pix(x_, y_, p);
                                if (i == m_last_obstacles.size() / 2) {
                                  ROS_INFO("Transformed point: %d ,%d    %f,%f" , p.x, p.y, x_, y_);
                                }
			} 
			
		
	// Save current odom for next iteration
			m_lastOdomTf = odomTf;     
		}
	}
	
    // Initialize all cells in the grid to unkown value
    for(int i=0; i<m_costmap.data.size(); i++)
      m_costmap.data[i] = SC_UNKNOWN;
    
    // Process the latest cloud received in each topic
    std::vector<SiarCostmap::PointPix> obstacles;
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
      
      // Compute roll and pitch rotation matrix from IMU
      double roll, pitch, yaw;
      float cr, sr, cp, sp, cy, sy, rx, ry;
      float r00, r01, r02, r10, r11, r12, r20, r21, r22;
      if(m_tiltCompesante)
      {
        tf::Quaternion q(m_imu.orientation.x, m_imu.orientation.y, m_imu.orientation.z, m_imu.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        sr = sin(-roll);
        cr = cos(-roll);
        sp = sin(-pitch);
        cp = cos(-pitch);
        r00 = cp;   r01 = sp*sr;   r02 = cr*sp;
        r10 =  0;   r11 = cr;    r12 = -sr;
        r20 = -sp;  r21 = cp*sr;  r22 = cp*cr;
      }
      
      // Evaluate every point into the cloud  
      sensor_msgs::PointCloud2Iterator<float> iterX(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(cloud, "z");
      for(int i=0; i<cloud.width; i++, ++iterX, ++iterY, ++iterZ)
      {
        // Tilt compensate the point if needed
        float x, y, z;
        if(m_tiltCompesante)
        {
          x = *iterX*r00 + *iterY*r01 + *iterZ*r02;
          y = *iterX*r10 + *iterY*r11 + *iterZ*r12;
          z = *iterX*r20 + *iterY*r21 + *iterZ*r22;
        }
        else
        {
          x = *iterX; 
          y = *iterY; 
          z = *iterZ;
        }
        
        // Evaluate if this is an obstacle or not
        if(x > m_minX && x < m_maxX && y > m_minY && y < m_maxY)
        {
          int index;
          if (point2index(x, y, index)) {
            m_unknown_map.at(index) = false;
          }
          if(m_costmap.data[index] != SC_POSITIVE_OBS && m_costmap.data[index] != SC_NEGATIVE_OBS)
          {
            if(z > m_obstacleHeight && z < m_robotHeight)
            {
              SiarCostmap::PointPix p;
              m_costmap.data[index] = SC_POSITIVE_OBS;
              index2pix(index, p);
              p.positive = true;
              obstacles.push_back(p);
            } 
            else if (z < m_obstacleHeightNeg) 
            {
              SiarCostmap::PointPix p;
              addNegativeObstacle(index);
              index2pix(index, p);
              p.positive = false;
              obstacles.push_back(p);                                    
            } else 
              m_costmap.data[index] = 0;
          }
        }
        
      }
      
    }
    
    // Add transformed obstacles from last observation if they correspond to unknown areas
    for (size_t i = 0; i < m_last_obstacles.size() && m_integrateOdom ; i++) {
      PointPix &p = m_last_obstacles.at(i);
      int index = p.x*m_costmap.info.width + p.y;
      if (p.x >= 0 && p.x < m_costmap.info.height && p.y < m_costmap.info.width && p.y >= 0 &&
        m_unknown_map.at(index)
      ) {
        obstacles.push_back(p);
        if (p.positive) {
          m_costmap.data[index] = SC_POSITIVE_OBS; 
        } else {
          m_costmap.data[index] = SC_NEGATIVE_OBS; 
        }
      }
    }
    
    if (m_integrateOdom && move_obstacles)
      m_last_obstacles = obstacles;
    
    if (!m_integrateOdom)
      m_last_obstacles.clear();
    
    // Apply exponential decay over obstacles
    if(m_expDecay > 0.0 && obstacles.size() > 0)
    {      
      // Build the object positions array
      ANNpointArray obsPts = annAllocPts(obstacles.size(), 2);
      for(int i=0; i<obstacles.size(); i++)
      {
        obsPts[i][0] = obstacles[i].x;
        obsPts[i][1] = obstacles[i].y;
      }
      
      // Build the Kdtree structure for effitient search
      ANNkd_tree* kdTree = new ANNkd_tree(obsPts, obstacles.size(), 2);

      // Evaluate all cell to obstacle distances and compute cost
      int k = 0;
      ANNpoint queryPt = annAllocPt(2);
      ANNidxArray nnIdx = new ANNidx[1];            
      ANNdistArray dists = new ANNdist[1];
      float C = log(127.0/0.1)/m_expDecay;
      for(int i=0; i<m_costmap.info.height; i++)
      {
        for(int j=0; j<m_costmap.info.width; j++, k++)
        {
          if(m_costmap.data[k] != SC_POSITIVE_OBS && m_costmap.data[k] != SC_NEGATIVE_OBS)
          {
            // Compute distance to closest obstacle
            queryPt[0] = i;
            queryPt[1] = j;
            kdTree->annkSearch(queryPt, 1, nnIdx, dists);
            // Evaluate the cost  
            m_costmap.data[k] = (int)(127.0*exp(-C*sqrt(dists[0])));
          }
        }
      }      
    }
    
    
    // Update costmap info
    m_costmap.header.seq++;
    m_costmap.header.stamp = t;
    m_costmap.info.map_load_time = t;
      
    return true;
  }
  
  nav_msgs::OccupancyGrid &getCostmap(void)
  {
    return m_costmap;
  }
  
  inline float getCost(float x, float y)
  {
    int index;
    if(x > m_minX && x < m_maxX && y > m_minY && y < m_maxY && point2index(x, y, index))
    {
      return m_costmap.data[index];
    }
    else
      return SC_UNKNOWN;
  }
  
  inline std::vector<float> getCost(std::vector<SiarCostmap::Point2D> &points)
  {
    int index;
    std::vector<float> cost(points.size(), SC_UNKNOWN);
    for(int i=0; i<points.size(); i++)
    {
      float x = points[i].x;
      float y = points[i].y;
      if(x > m_minX && x < m_maxX && y > m_minY && y < m_maxY)
      {
        if (point2index(x, y, index)) {
          cost[i] = m_costmap.data[index];
        }
      }
    }
    return cost;
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
  
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    m_imu = *msg;
    m_imuNew = true;
  } 
  
  void odomIntegrateCallback(const std_msgs::Bool::ConstPtr& msg)
  {
	if(msg->data == 0)
	  m_integrateOdom = false;
	else
	{
	  m_integrateOdom = true; 
	  m_integrateOdomInit = false;
    }	  
  }

  void updateTimer(const ros::TimerEvent& event)
  {
    // Update costmap
    if(updateCostmap())
    {
		// Publish costmap
		m_pub.publish(m_costmap);
	}
  }
  
  void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg, int id)
  {
      try
      {
        m_tfListener.lookupTransform(m_baseFrameId, msg->header.frame_id, ros::Time(0), m_cloudTf[id]);
        m_cloud[id] = *msg;
      m_cloudNew[id] = true;
//         gotTf[id] = true;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("Error: %s", ex.what());
      }
  }
  
  inline bool point2index(float &x, float &y, int &index)
  {
    index = ((int)((x-m_minX)*m_divRes))*m_costmap.info.width + m_costmap.info.width - (int)((y-m_minY)*m_divRes);
    return index >= 0 && index < n_pix;
  }
  
  inline bool point2pix(float &x, float &y, SiarCostmap::PointPix &pix)
  {
    pix.x = round((x-m_minX)*m_divRes);
    pix.y = m_costmap.info.width - round((y-m_minY)*m_divRes);
    return pix.x >=0 && pix.x < m_costmap.info.height &&
           pix.y >=0 && pix.y < m_costmap.info.width;
  }
  
  inline void index2point(float &x, float &y, const int &index) 
  {
    x = index/m_costmap.info.width * m_resolution + m_minX;
    y = m_minY + (m_costmap.info.width - index%m_costmap.info.width) * m_resolution;
  }
  
  inline void pix2point(float &x, float &y, const SiarCostmap::PointPix &pix) 
  {
    x = pix.x * m_resolution + m_minX;
    y = m_minY + (m_costmap.info.width - pix.y) * m_resolution;
  }
  
  inline void index2pix(int &index, SiarCostmap::PointPix &pix)
  {
    pix.x = index/m_costmap.info.width;
    pix.y = index%m_costmap.info.width;
  }
  
  inline void pix2index(int &index, const SiarCostmap::PointPix &pix)
  {
    index = pix.x * m_costmap.info.width + pix.y;
  }
  
  void addNegativeObstacle(int index)
  {
    if (m_inflate_negative < 0)
      m_inflate_negative = 0;
    for (int j = -m_inflate_negative; j<=m_inflate_negative;j++) {
      for (int i = -m_inflate_negative; i <= m_inflate_negative;i++) {
        int new_index = index+i+j*m_costmap.info.width;
        if (new_index <0 || new_index >= m_costmap.data.size())
          continue;
        m_costmap.data[index+i+j*m_costmap.info.width] = m_considerSign?SC_NEGATIVE_OBS:SC_POSITIVE_OBS;
        
      }
    }
    
  }
  
  // Params
  float m_hz, m_obstacleHeight, m_obstacleHeightNeg, m_expDecay;
  float m_width, m_height, m_resolution, m_robotHeight;
  std::string m_baseFrameId, m_odomFrameId;
  bool m_tiltCompesante, m_integrateOdom, m_integrateOdomInit;
  
  // ROS stuff
  ros::NodeHandle m_nh;  
  ros::Timer timer;
  tf::TransformListener m_tfListener;
  ros::Subscriber m_sub0, m_sub1, m_sub2, m_sub3, m_sub4, m_sub5, m_sub6, m_sub7;
  ros::Publisher m_pub;
  tf::StampedTransform m_lastOdomTf;


  // Sensor data
  sensor_msgs::PointCloud2 m_cloud[6];
  sensor_msgs::Imu m_imu;
  bool m_cloudNew[6], m_imuNew;
  tf::StampedTransform m_cloudTf[6];
  
  // Compute costmap
  float m_minX, m_maxX, m_minY, m_maxY;
  float m_divRes;
  int n_pix;
  nav_msgs::OccupancyGrid m_costmap; 
  std::vector<SiarCostmap::PointPix> m_last_obstacles;
  std::vector<bool> m_unknown_map;
  bool m_considerSign;
  int m_inflate_negative;
};


#endif

