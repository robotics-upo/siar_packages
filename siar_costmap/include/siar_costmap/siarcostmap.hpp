#ifndef __SIARCOSTMAP_HPP__
#define __SIARCOSTMAP_HPP__

#include <ros/ros.h>
#include <tf/transform_listener.h>
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
    }
    
    PointPix(int _x, int _y)
    {
      x = _x;
      y = _y;
    }
    
    PointPix(const PointPix &d)
    {
      x = d.x;
      y = d.y;
    }
    
    int x;
    int y;
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
    if(!lnh.getParam("obstacle_height_negative", m_obstacleHeight))
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
    if(!lnh.getParam("consider_sign", m_tiltCompesante))
      m_considerSign = true;

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
      double cr, sr, cp, sp, cy, sy, rx, ry;
      double r00, r01, r02, r10, r11, r12, r20, r21, r22;
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
          point2index(x, y, index);
          if(m_costmap.data[index] != SC_POSITIVE_OBS && m_costmap.data[index] != SC_NEGATIVE_OBS)
          {
            if(fabs(z) > m_obstacleHeight)
            {
              SiarCostmap::PointPix p;
              m_costmap.data[index] = SC_POSITIVE_OBS;
              index2pix(index, p);
              obstacles.push_back(p);
            } 
            else if (z < m_obstacleHeightNeg) 
            {
              SiarCostmap::PointPix p;
              m_costmap.data[index] = m_considerSign?SC_NEGATIVE_OBS:SC_POSITIVE_OBS;
              index2pix(index, p);
              obstacles.push_back(p);                                    
            } else 
              m_costmap.data[index] = 0;
          }
        }
        
      }
    }
    
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
//           else if(m_costmap.data[k] == SC_UNKNOWN)  NOTE: Commented by chur... why not to calculate cost in unknown pixels?
//             m_costmap.data[k] = 0;
        }
      }      
    }
    
    // Update costmap info
    m_costmap.header.seq++;
    m_costmap.header.stamp = t;
    m_costmap.info.map_load_time = t;
    
    // Prepare next sensor reading
    //for(int i=0; i<6; i++)
    //  m_cloudNew[i] = false;
      
    return m_costmap;
  }
  
  nav_msgs::OccupancyGrid &getCostmap(void)
  {
    return m_costmap;
  }
  
  inline float getCost(float x, float y)
  {
    int index;
    if(x > m_minX && x < m_maxX && y > m_minY && y < m_maxY)
    {
      point2index(x, y, index);
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
        point2index(x, y, index);
        cost[i] = m_costmap.data[index];
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
  
  inline void point2index(float &x, float &y, int &index)
  {
    index = ((int)((x-m_minX)*m_divRes))*m_costmap.info.width + m_costmap.info.width - (int)((y-m_minY)*m_divRes);
  }
  
  inline void point2pix(float &x, float &y, SiarCostmap::PointPix &pix)
  {
    pix.x = (int)((x-m_minX)*m_divRes);
    pix.y = m_costmap.info.width - (int)((y-m_minY)*m_divRes);
  }
  
  inline void index2pix(int &index, SiarCostmap::PointPix &pix)
  {
    pix.x = index/m_costmap.info.width;
    pix.y = index%m_costmap.info.width;
  }
  
  // Params
  double m_hz, m_obstacleHeight, m_obstacleHeightNeg, m_expDecay;
  double m_width, m_height, m_resolution;
  std::string m_baseFrameId;
  bool m_tiltCompesante;
  
  // ROS stuff
  ros::NodeHandle m_nh;  
  ros::Timer timer;
  tf::TransformListener m_tfListener;
  ros::Subscriber m_sub0, m_sub1, m_sub2, m_sub3, m_sub4, m_sub5, m_sub6;
  ros::Publisher m_pub;

  // Sensor data
  sensor_msgs::PointCloud2 m_cloud[6];
  sensor_msgs::Imu m_imu;
  bool m_cloudNew[6], m_imuNew;
  tf::StampedTransform m_cloudTf[6];
  
  // Compute costmap
  float m_minX, m_maxX, m_minY, m_maxY;
  float m_divRes;
  nav_msgs::OccupancyGrid m_costmap; 
  bool m_considerSign;
};


#endif

