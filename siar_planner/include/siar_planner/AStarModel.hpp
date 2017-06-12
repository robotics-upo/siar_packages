#ifndef ASTAR_MODEL__HPP__
#define ASTAR_MODEL__HPP__

#include "siar_controller/command_evaluator.hpp"
#include "siar_planner/AStarState.hpp"
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <random>

class AStarModel 
{
public:
  //! @brief Constructor from NodeHandles
  AStarModel(ros::NodeHandle &nh, ros::NodeHandle &pn);
  
  void occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg);
  
  visualization_msgs::Marker testIntegration(AStarState &st, bool relaxed = false, bool one_wheel = true);
  
  //! @brief Integrates the model and returns the cost associated with 
  //! @return Negative --> collision. Positive --> Arc's longitude
  double integrate(AStarState &st, geometry_msgs::Twist &cmd, double T, bool relaxed = false, bool one_wheel = true);
  
  virtual geometry_msgs::Twist generateRandomCommand();
  
  inline bool isInit() const {return map_init;}
  
  visualization_msgs::Marker getMarker(AStarState &st, int id = 0); 
  
  inline bool isCollision(AStarState &st) {
    bool ret_val;
    m_ce.applyFootprint(st.state[0], st.state[1], st.state[2], m_world, ret_val);
    return ret_val;
  }
  
protected:
  nav_msgs::OccupancyGrid m_world;
  siar_controller::CommandEvaluator m_ce;
  
  bool map_init;
  
  template< class RealType = double > class uniform_real_distribution;
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
  
  ros::Subscriber map_sub;
  
  visualization_msgs::Marker m;
};

AStarModel::AStarModel(ros::NodeHandle &nh, ros::NodeHandle& pn):m_ce(pn), map_init(false), gen(rd()), dis(-m_ce.getCharacteristics().theta_dot_max, m_ce.getCharacteristics().theta_dot_max)
{
  
  map_sub = nh.subscribe("/altitude_map", 2, &AStarModel::occupancyGridCallback, this);
}

void AStarModel::occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg)
{
  map_init = true;
  m_world = *msg;
  m_ce.initializeFootprint(*msg);
  
}

double AStarModel::integrate(AStarState& st, geometry_msgs::Twist& cmd, double T, bool relaxed, bool one_wheel)
{
  geometry_msgs::Twist v_ini;
  v_ini.linear.x = m_ce.getCharacteristics().v_max;
  
  if (st.state.size() < 3) {
    ROS_ERROR("AStarModel::integrate --> cannot integrate the model --> too few states. State size: %u", (unsigned int) st.state.size());
  }
  
  // Debug:
//   ROS_INFO("Calling evaluate trajectory. Delta_t = %f. T_hor = %f. ", m_ce.
  
  double ret_val;
  if (!relaxed) 
    ret_val = m_ce.evaluateTrajectory(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2]);
  else
    ret_val = m_ce.evaluateTrajectoryRelaxed(v_ini, cmd, cmd, m_world, m, st.state[0], st.state[1], st.state[2], one_wheel);
  
  st.state = m_ce.getLastState();
  
  
  return ret_val;
}

geometry_msgs::Twist AStarModel::generateRandomCommand() {
  
  geometry_msgs::Twist ret;
  
  ret.linear.x = m_ce.getCharacteristics().v_max;
  ret.angular.z = dis(gen);
  ret.linear.y = ret.linear.z = ret.angular.x = ret.angular.y = 0.0;
  
  return ret;
}

visualization_msgs::Marker AStarModel::getMarker(AStarState& st, int id)
{
  visualization_msgs::Marker m;
  
  if (m_ce.getFootprint() != NULL) {
    m_ce.getFootprint()->addPoints(st.state[0], st.state[1], st.state[2], m, id, true, "/map");
  } else {
    ROS_ERROR("AStarModel::getMarker: --> Footprint is not initialized");
  }
  
  return m;
}

visualization_msgs::Marker AStarModel::testIntegration(AStarState& st, bool relaxed, bool one_wheel)
{
  auto comm = generateRandomCommand();
  double cost = integrate(st, comm, 0.5, relaxed, one_wheel);
  
  ROS_INFO("AStarModel::testIntegration --> Command = %f, %f --> Cost = %f", comm.linear.x, comm.angular.z, cost);
  
  
  return m;
}



#endif