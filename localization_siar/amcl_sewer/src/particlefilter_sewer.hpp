#ifndef __PARTICLEFILTER_HPP__
#define __PARTICLEFILTER_HPP__

#include <vector>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <time.h>
#include <math.h>
#include <sewer_graph/sewer_graph.h>
#include <fstream>

//Struct that contains the data concerning one Particle
struct Particle
{
  //Position
  float x;
  float y;
  
  // Yaw angle (from north or with respect to MAP?)
  float a;

  // Weight (One weight)
  float w;
};

//Class definition
class ParticleFilter
{
public:

  //!Default contructor 
  ParticleFilter(std::string &node_name) : 
  m_randVar(boost::mt19937(time(0)), boost::normal_distribution<>(0, 0.4)), s_g(NULL)
  {
    // Setup random number generator from GSL
    gsl_rng_env_setup();
    m_randomType = gsl_rng_default;
    m_randomValue = gsl_rng_alloc(m_randomType);
    
    // Read node parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("base_frame_id", m_baseFrameId))
      m_baseFrameId = "base_link";  
    if(!lnh.getParam("odom_frame_id", m_odomFrameId))
      m_odomFrameId = "odom";  
    if(!lnh.getParam("global_frame_id", m_globalFrameId))
      m_globalFrameId = "map";  

    // Read amcl parameters
    if(!lnh.getParam("update_rate", m_updateRate))
      m_updateRate = 10.0;
    if(!lnh.getParam("min_particles", m_minParticles))
      m_minParticles = 300;
    if(!lnh.getParam("max_particles", m_maxParticles))
      m_maxParticles = 600;
    if(m_minParticles > m_maxParticles)
      m_maxParticles = m_minParticles;
    if(!lnh.getParam("odom_x_mod", m_odomXMod))
      m_odomXMod = 0.2;
    if(!lnh.getParam("odom_y_mod", m_odomYMod))
      m_odomYMod = 0.2;
    if(!lnh.getParam("odom_a_mod", m_odomAMod))
      m_odomAMod = 0.2;
    if(!lnh.getParam("initial_x", m_initX))
      m_initX = 0.0;
    if(!lnh.getParam("initial_y", m_initY))
      m_initY = 0.0;
    if(!lnh.getParam("initial_a", m_initA))
      m_initA = 0.0;  
    if(!lnh.getParam("initial_x_dev", m_initXDev))
      m_initXDev = 0.3;
    if(!lnh.getParam("initial_y_dev", m_initYDev))
      m_initYDev = 0.3;
    if(!lnh.getParam("initial_a_dev", m_initADev))
      m_initADev = 0.2;  
    if(!lnh.getParam("update_min_d", m_dTh))
      m_dTh = 0.5;
    if(!lnh.getParam("update_min_a", m_aTh))
      m_aTh = 0.2;
    if(!lnh.getParam("resample_interval", m_resampleInterval))
      m_resampleInterval = 0;
    // Get the parameters related with the sewer
    if (!lnh.getParam("detect_manhole_topic", m_detectManholeTopic))
      m_detectManholeTopic = "/manhole";
      
    
    if (!lnh.getParam("sewer_graph_file", m_sewer_graph_file))
      m_sewer_graph_file = "/home/siar/siar_ws/src/siar_packages/localization_siar/sewer_graph/test/sewer_graph_1";
    
    if (!lnh.getParam("min_manhole_detected", m_min_manhole_detected))
      m_min_manhole_detected = 10;
    if (!lnh.getParam("edgeDev", m_edgeDev))
      m_edgeDev = 1.0;
    
    edgeConst1 = 1./(m_edgeDev*sqrt(2*M_PI));
    edgeConst2 = 1./(2*m_edgeDev*m_edgeDev);
    
    if (!lnh.getParam("forkDev", m_forkDev))
      m_forkDev = 3.0;
    if (!lnh.getParam("fork_dist", m_fork_dist))
      m_fork_dist = 3.0;
    
    forkConst1 = 1./(m_forkDev*sqrt(2*M_PI));
    forkConst2 = 1./(2*m_forkDev*m_forkDev);
    
    if (!lnh.getParam("manholeDev", m_manholeDev))
      m_manholeDev = 0.6;
    if (!lnh.getParam("manholeThres", m_manholeThres))
      m_manholeThres = 0.15;
    
    manholeConst1 = 1./(m_manholeDev*sqrt(2*M_PI)); 
    manholeConst2 = 1./(2*m_manholeDev*m_manholeDev);
    
    // Init the statistic stuff
    stats_file_open = false;
    if (!lnh.getParam("stats_file", stats_filename))
      stats_filename = "~/manhole_stats.txt";
    try {
      stats_file.open(stats_filename.c_str());
      stats_file_open = true;
    }catch (std::exception &e) {
      std::cerr << "Could not open the stats file: " << stats_filename << std::endl;
    }
    m_ground_truth_manhole_sub = m_nh.subscribe("ground_truth", 1, &ParticleFilter::manholeGroundTruthCallback, this);
    
    // Save trajectory
    traj_file_open = false;
    if (!lnh.getParam("traj_file", traj_filename))
      traj_filename = "~/traj_.txt";
    try {
      traj_file.open(traj_filename.c_str());
      traj_file_open = true;
    }catch (std::exception &e) {
      std::cerr << "Could not open the stats file: " << traj_filename << std::endl;
    }
    
    // Init the particle filter internal stuff
    m_nUpdates = 0;
    m_init = false;
    m_doUpdate = false;
    m_p.resize(m_maxParticles);
    
    // Launch subscribers
    m_detect_manhole_Sub = m_nh.subscribe(m_detectManholeTopic, 1, &ParticleFilter::manholeDetectedCallback, this);
    m_initialPoseSub = m_nh.subscribe(node_name+"/initial_pose", 2, &ParticleFilter::initialPoseReceived, this);
    
    // Launch publishers
    m_posesPub = m_nh.advertise<geometry_msgs::PoseArray>(node_name+"/particle_cloud", 1, true);
    m_graphPub = m_nh.advertise<visualization_msgs::Marker>(node_name+"/sewer_graph", 0);
    m_gpsPub = m_nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 2, true);

    // Launch updater timer
    updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &ParticleFilter::checkUpdateThresholdsTimer, this);
    
    // Initialize TF from odom to map as identity
    m_lastGlobalTf.setIdentity();
    
    // Initialize sewer graph
    s_g = new sewer_graph::SewerGraph(m_sewer_graph_file);
    
    // PUblish gps reference position
    ROS_INFO("Publishing gps position: %f, %f", s_g->getReferencePosition().latitude, s_g->getReferencePosition().longitude);
    m_gpsPub.publish(s_g->getReferencePosition());
    
    // Initialize filter if requested
    bool initialize = false;
    if (!lnh.getParam("initialize", initialize)) {
      initialize = false;
    }
    if (initialize)
    {
      tf::Pose pose;
      tf::Vector3 v(m_initX, m_initY, 0.0);
      pose.setOrigin(v);
      pose.setRotation(tf::createQuaternionFromYaw(m_initA));
      ROS_INFO("Setting pose: %.3f %.3f %.3f", pose.getOrigin().x(), pose.getOrigin().y(), getYawFromTf(pose));
      setInitialPose(pose, m_initXDev, m_initYDev, m_initADev);
    }
  }

  //!Default destructor
  ~ParticleFilter()
  {
    gsl_rng_free(m_randomValue);
    delete s_g;
  }
    
  //! Check motion and time thresholds for AMCL update
  bool checkUpdateThresholds()
  {
    // Publish current TF from odom to map
    m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));
    
    // If the filter is not initialized then exit
    if(!m_init)
      return false;
//     std::cout << "Checking for AMCl sewer for update" << std::endl;
          
    // Compute odometric trasnlation and rotation since last update 
    tf::StampedTransform odomTf;
    try
    {
      m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
      m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("AMCL3D error: %s",ex.what());
      return false;
    }
    tf::Transform T = m_lastOdomTf.inverse()*odomTf;
    
    // Check translation threshold
    if(T.getOrigin().length() > m_dTh)
    {
      ROS_INFO("Translation update");
      m_doUpdate = true;
    }
    
    // Check yaw threshold
    double yaw, pitch, roll;
    T.getBasis().getRPY(roll, pitch, yaw);
    if(fabs(yaw) > m_dTh)
    {
      ROS_INFO("Rotation update");
      m_doUpdate = true;
    }
    
    if (m_doUpdate) {
      update(checkManhole());
    }
    
    return false;
  }
  
  visualization_msgs::Marker getPosMarker(const std::string &ref_frame, int id = 3, double scale = 3.0) {
    float mX, mY, mA, devX, devY, devA;
    computeDev(mX, mY, mA, devX, devY, devA);
    visualization_msgs::Marker pos;
    pos.header.frame_id = ref_frame;
    pos.header.stamp = ros::Time::now();
    pos.ns = "sewer_graph";
    pos.action = visualization_msgs::Marker::ADD;
    pos.pose.orientation.w = 1.0;
    pos.id = id;
    pos.type = visualization_msgs::Marker::SPHERE_LIST;
    pos.color.b = 1.0;
    pos.color.a = 1.0;
    pos.scale.x = scale;
    pos.scale.y = scale;
    pos.scale.z = scale;
    
    geometry_msgs::Point p;
    p.x = mX;
    p.y = mY;
    p.z = 0.0;
    
    
    pos.points.push_back(p);
    return pos;
  }
  
  void publishParticles()
  {
    static int seq = 0;

    // If the filter is not initialized then exit
    if(!m_init)
      return;
      
    // Build the msg based on the particles position and orinetation  
    geometry_msgs::PoseArray particlesMsg;
    particlesMsg.header.stamp = ros::Time::now();
    particlesMsg.header.frame_id = m_globalFrameId;
    particlesMsg.header.seq = seq++;
    particlesMsg.poses.resize(m_p.size());
    for(int i=0; i<m_p.size(); i++)
    {
      particlesMsg.poses[i].position.x = m_p[i].x;
      particlesMsg.poses[i].position.y = m_p[i].y;
      particlesMsg.poses[i].position.z = 0.0;
      particlesMsg.poses[i].orientation.x = 0.0;
      particlesMsg.poses[i].orientation.y = 0.0;
      particlesMsg.poses[i].orientation.z = sin(m_p[i].a*0.5);
      particlesMsg.poses[i].orientation.w = cos(m_p[i].a*0.5);
    }
    
    // Publisg particle cloud
    m_posesPub.publish(particlesMsg);
  }
                                       
private:

  void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
  {
    checkUpdateThresholds();
    
    // Publish markers
    std::vector<visualization_msgs::Marker> markers = s_g->getMarkers(m_globalFrameId);
    for (unsigned int i = 0; i < markers.size();i++) {
      m_graphPub.publish(markers[i]);
      
    }
    m_graphPub.publish(getPosMarker(m_globalFrameId));
  }

  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
  {
    // We only accept initial pose estimates in the global frame
    if(msg->header.frame_id != m_globalFrameId)
    {
      ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
      msg->header.frame_id.c_str(),
      m_globalFrameId.c_str());
      return;  
    }
    
    // Transform into the global frame
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    double x = pose.getOrigin().x();
    double y = pose.getOrigin().y();
    
    ROS_INFO("Setting pose: %.3f %.3f %.3f", pose.getOrigin().x(), pose.getOrigin().y(), getYawFromTf(pose));
    
    
    ROS_INFO("Distance to closest Edge = %f", s_g->getDistanceToClosestEdge(x , y));
    ROS_INFO("Distance to closest Manhole = %f", s_g->getDistanceToClosestManhole(x, y));
    
    // Initialize the filter
    setInitialPose(pose, m_initXDev, m_initYDev, m_initADev);
  }

  //! 3D point-cloud callback
  void manholeDetectedCallback(const std_msgs::BoolConstPtr& msg)
  {    
    manhole_hist.push_back(msg->data);
  }
  
  //! 3D point-cloud callback
  void manholeGroundTruthCallback(const std_msgs::BoolConstPtr& msg)
  {
    if (!msg->data) {
      return;
    }
    float x, y, z, xDev, yDev, aDev;
    
    computeDev(x, y, z, xDev, yDev, aDev);
    int manhole_id = s_g -> getClosestVertex(x, y, sewer_graph::MANHOLE);
    
    if (stats_file_open) {
      stats_file << x << " " << y << " " << z <<"\t ";
      stats_file << xDev << " " << yDev << " " << aDev <<"\t ";
      stats_file << s_g -> getVertexContent(manhole_id).e.toString(' ') << std::endl;
    }
  }
  
  void update(bool detected_manhole) {
    // Compute odometric trasnlation and rotation since last update 
    tf::StampedTransform odomTf;
    try
    {
      m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
      m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }
    
    // Extract current robot roll and pitch
    double yaw, r, p;
    odomTf.getBasis().getRPY(r, p, yaw);
    
    // Perform particle prediction based on odometry
    float delta_x, delta_y;
    double delta_r, delta_p, delta_a;
    tf::Transform T = m_lastOdomTf.inverse()*odomTf;
    delta_x = T.getOrigin().getX();
    delta_y = T.getOrigin().getY();
    T.getBasis().getRPY(delta_r, delta_p, delta_a);
    if(!predictParticles(delta_x, delta_y, (float)delta_a))
    {
      ROS_ERROR("ParticleFilterSewer::manholeDetectedCallback --> Prediction error!");
      return;
    }
      
    // Perform particle update based on current point-cloud
    if(!updateParticles(detected_manhole))
    {
      ROS_ERROR("ParticleFilterSewer::manholeDetectedCallback --> Update error!");
      return;
    }
      
    // Update time and transform information
    m_lastOdomTf = odomTf;
    m_doUpdate = false;
            
    // Publish particles
    publishParticles();
  }

  //!This function implements the PF prediction stage. Translation in X, Y and Z 
  //!in meters and yaw angle incremenet in rad
  //! TODO: In a first stage the predict stage will remain equal to the amcl_3d but without the z
  bool predictParticles(float delta_x, float delta_y, float delta_a)
  {
    float xDev, yDev, aDev;
    xDev = fabs(delta_x*m_odomXMod);
    yDev = fabs(delta_y*m_odomYMod);
    aDev = fabs(delta_a*m_odomAMod); 
    
    //Make a prediction for all particles according to the odometry
    for(int i=0; i<(int)m_p.size(); i++)
    {
      float sa = sin(m_p[i].a);
      float ca = cos(m_p[i].a);
      float randX = delta_x + gsl_ran_gaussian(m_randomValue, xDev);
      float randY = delta_y + gsl_ran_gaussian(m_randomValue, yDev);
      m_p[i].x += ca*randX - sa*randY;
      m_p[i].y += sa*randX + ca*randY;
      m_p[i].a += delta_a + gsl_ran_gaussian(m_randomValue, aDev);
    }
    
    return true;
  }
  
  
  
  // Update Particles taking into account
  // No input is necessary --> we will get the closest Manhole, which will be used for weighting purposes(with some dispersion)
  bool updateParticles(bool detected_manhole)
  {  
    double wt = 0.0;
    bool fork = false;
    
    float x,y,a,xd,yd,ad;
    computeDev(x,y,a,xd,yd,ad);
    
    if (traj_file_open) {
      traj_file << x << "\t" << y << std::endl;
    }
    
    if (detected_manhole) 
      ROS_INFO("Performing Update with Manhole");
    else if (s_g->getDistanceToClosestVertex(x, y, sewer_graph::FORK) < m_fork_dist) {
      fork = true;
      ROS_INFO("In a fork");
    }
    
    for(int i=0; i<(int)m_p.size(); i++)
    {
      double tx = m_p[i].x;
      double ty = m_p[i].y;
      // Evaluate the weight of the range sensors
      m_p[i].w = computeEdgeWeight(tx, ty); // Compute weight as a function of the distance to the closest edge
      if (detected_manhole) {
        
        m_p[i].w = computeManholeWeight(tx, ty);
      } else if (fork) {
        m_p[i].w = computeForkWeight(tx, ty);
      }
        
    //Update the particle weight
    //m_p[i].w = (1.0-alpha)*m_p[i].w + alpha*wi;
    //m_p[i].w = wi;
      
      //Increase the summatory of weights
      wt += m_p[i].w;
    }
    
    //Normalize all weights
    for(int i=0; i<(int)m_p.size(); i++)
    {
      m_p[i].w /= wt;  
    }  
    
    // Re-compute global TF according to new weight of samples
    computeGlobalTf();

    //Do the resampling if needed
    m_nUpdates++;
    if(m_nUpdates > m_resampleInterval || detected_manhole)
    {
      m_nUpdates = 0;
      resample();
    }
    /*wt = 0;
    for(int i=0; i<(int)m_p.size(); i++)
      wt += m_p[i].w*m_p[i].w;
    float nEff = 1/wt;
    if(nEff < ((float)m_p.size())/10.0)
      resample();*/
    //resample();

    return true;
  }
  
  double computeEdgeWeight(double x, double y) {
    double dist = s_g->getDistanceToClosestEdge(x,y);
    
    return edgeConst1*exp(-dist*dist*edgeConst2);
  }
  
  double computeForkWeight(double x, double y) {
    double dist = s_g->getDistanceToClosestEdge(x,y);
    return forkConst1*exp(-dist*dist*forkConst2);
    
  }
  
  double computeManholeWeight(double x, double y) {
    double dist = s_g->getDistanceToClosestManhole(x, y);
    return manholeConst1*exp(-dist*dist*manholeConst2) + m_manholeThres;
  }

  //! Set the initial pose of the particle filter
  void setInitialPose(tf::Pose initPose, float xDev, float yDev, float aDev)
  {
    // Resize particle set
    m_p.resize(m_maxParticles);
    
    // Sample the given pose
    tf::Vector3 t = initPose.getOrigin();
    float a = getYawFromTf(initPose);
    float dev = std::max(xDev, yDev);
    float gaussConst1 = 1./(dev*sqrt(2*M_PI));
    float gaussConst2 = 1./(2*dev*dev);
    float dist = 0.0, wt = 0.0;
    m_p[0].x = t.x();
    m_p[0].y = t.y();
    m_p[0].a = a;
    m_p[0].w = gaussConst1;
    wt = m_p[0].w;
    for(int i=1; i<(int)m_p.size(); i++)
    {
      m_p[i].x = t.x() + gsl_ran_gaussian(m_randomValue, xDev);
      m_p[i].y = t.y() + gsl_ran_gaussian(m_randomValue, yDev);
      m_p[i].a = a + gsl_ran_gaussian(m_randomValue, aDev);
      dist = sqrt((m_p[i].x - t.x())*(m_p[i].x - t.x()) + (m_p[i].y - t.y())*(m_p[i].y - t.y()) );
      m_p[i].w = gaussConst1*exp(-dist*dist*gaussConst2);
      wt += m_p[i].w;
    }
    for(int i=0; i<(int)m_p.size(); i++)
      m_p[i].w /= wt;
        
    // Extract TFs for future updates
    bool initialized = false;
    while (!initialized) {
      try
      {
        m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(15.0));
        m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
        initialized = true;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
      sleep(2);
    }
    computeGlobalTf();
    m_doUpdate = false;
    m_init = true;
    
    // Publish particles
    publishParticles();
  }
  
  //! Return yaw from a given TF
  float getYawFromTf(tf::Pose& pose)
  {
    double yaw, pitch, roll;
    
    pose.getBasis().getRPY(roll, pitch, yaw);
    
    return (float)yaw;
  }

  //! resample the set of particules using low-variance sampling
  int resample()
  {
    int i, m;
    float r, u, c, factor;
    std::vector<Particle> newP;

    //Initialize data and vectors
    newP.resize(m_p.size());
    factor = 1.0/((float)m_p.size());
    i = 0;
    c = m_p[0].w;
    r = factor * gsl_rng_uniform(m_randomValue);

    //Do resamplig
    for(m=0; m<m_p.size(); m++)
    {
      u = r + factor*(float)m;
      while(u > c)
      {
        i++;
        c += m_p[i].w;
      }
      newP[m] = m_p[i];
      newP[m].w = factor;
    }
    
    //Asign the new particles set
    m_p = newP;

    return 0;
  }
  
  // Computes TF from odom to global frames
  void computeGlobalTf()
  {        
    // Compute mean value from particles
    Particle p;
    p.x = 0.0;
    p.y = 0.0;
    p.a = 0.0;
    p.w = 0.0;
    for(int i=0; i<m_p.size(); i++)
    {
      p.x += m_p[i].w * m_p[i].x;
      p.y += m_p[i].w * m_p[i].y;
      p.a += m_p[i].w * m_p[i].a;
    }
    
    // Compute the TF from odom to global
    std::cout << "New TF:\n\t" << p.x << ", " << p.y << std::endl;
    m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(p.a*0.5), cos(p.a*0.5)), tf::Vector3(p.x, p.y, 0.0))*m_lastOdomTf.inverse();
  }
  
  void computeDev(float &mX, float &mY, float &mA, float &devX, float &devY, float &devA)
  {        
    // Compute mean value from particles
    devX = mX = 0.0;
    devY = mY = 0.0;
    devA = mA = 0.0;
    for(int i=0; i<m_p.size(); i++)
    {
      mX += m_p[i].w * m_p[i].x;
      mY += m_p[i].w * m_p[i].y;
      mA += m_p[i].w * m_p[i].a;
    }
    for(int i=0; i<m_p.size(); i++)
    {
      devX += m_p[i].w * (m_p[i].x-mX) * (m_p[i].x-mX);
      devY += m_p[i].w * (m_p[i].y-mY) * (m_p[i].y-mY);
      devA += m_p[i].w * (m_p[i].a-mA) * (m_p[i].a-mA);
    }
    devX = sqrt(devX);
    devY = sqrt(devY);
    devA = sqrt(devA);
  }
  
  bool checkManhole() {
    int n_trues = 0;
    
    for (unsigned int i = 0;i < manhole_hist.size(); i++) {
      
      if (manhole_hist[i])
        n_trues++;
    }
    
    
    ROS_INFO("Number of detections %d. Number of positive: %d", (int)manhole_hist.size(), n_trues);
    
    manhole_hist.clear();
    
    return n_trues >= m_min_manhole_detected;
  }
  
  //! Indicates if the filter was initialized
  bool m_init;
  
  //! Particles 
  std::vector<Particle> m_p;
  
  //! Particles roll and pich (given by IMU)
  double m_roll, m_pitch;
  
  //! Number of particles in the filter
  int m_maxParticles;
  int m_minParticles;
  
  //! Odometry characterization
  double m_odomXMod, m_odomYMod, m_odomZMod, m_odomAMod;
  double m_initX, m_initY, m_initA;
  double m_initXDev, m_initYDev, m_initADev;
  
  //! Resampling control
  int m_nUpdates;
  int m_resampleInterval;
  
  //! Thresholds for filter updating
  double m_dTh, m_aTh, m_tTh;
  tf::StampedTransform m_lastOdomTf;
  tf::Transform m_lastGlobalTf;
  bool m_doUpdate;
  double m_updateRate;

  //! Node parameters
  std::string m_detectManholeTopic;
  std::string m_baseFrameId;
  std::string m_odomFrameId;
  std::string m_globalFrameId;
  std::string m_inOdomTfTopic;
  
  //! ROS msgs and data
  ros::NodeHandle m_nh;
  tf::TransformBroadcaster m_tfBr;
  tf::TransformListener m_tfListener;
  ros::Subscriber m_detect_manhole_Sub, m_initialPoseSub, m_odomTfSub;
  ros::Publisher m_posesPub, m_graphPub, m_gpsPub;
  ros::Timer updateTimer;
  
  // Sewer stuff
  std::string m_sewer_graph_file;
  sewer_graph::SewerGraph *s_g;
  
  //! Random number generator
  const gsl_rng_type *m_randomType;
  gsl_rng *m_randomValue;
  
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > m_randVar;   

  std::vector<bool> manhole_hist;
  int m_min_manhole_detected;
  double m_edgeDev, m_manholeDev, m_manholeThres;
  double edgeConst1, edgeConst2;
  double manholeConst1, manholeConst2;
  double m_forkDev, forkConst1, forkConst2, m_fork_dist;
  
  //For saving stats
  ros::Subscriber m_ground_truth_manhole_sub;
  std::ofstream stats_file;
  std::string stats_filename;
  bool stats_file_open;
  
  //For saving trajetory
  std::ofstream traj_file;
  std::string traj_filename;
  bool traj_file_open;
};

#endif


