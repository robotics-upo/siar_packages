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
  
  //! @brief Integrates the model and returns the cost associated with 
  //! @return Negative --> collision. Positive --> Arc's longitude
  double integrate(AStarState &st, geometry_msgs::Twist &cmd, double T);
  
  virtual geometry_msgs::Twist generateRandomCommand();
  
  inline bool isInit() const {return map_init;}
  
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
};

AStarModel::AStarModel(ros::NodeHandle &nh, ros::NodeHandle& pn):m_ce(pn), map_init(false), gen(rd()), dis(-m_ce.getCharacteristics().theta_dot_max, m_ce.getCharacteristics().theta_dot_max)
{
  
  map_sub = nh.subscribe("/altitude_map", 2, &AStarModel::occupancyGridCallback, this);
}

void AStarModel::occupancyGridCallback(nav_msgs::OccupancyGridConstPtr msg)
{
  map_init = true;
  m_world = *msg;
  
}

double AStarModel::integrate(AStarState& st, geometry_msgs::Twist& cmd, double T)
{
  geometry_msgs::Twist v_ini;
  v_ini.linear.x = m_ce.getCharacteristics().v_max;
  visualization_msgs::Marker m;
  return m_ce.evaluateTrajectoryMinVelocity(v_ini, cmd, cmd, m_world, m);
}

geometry_msgs::Twist AStarModel::generateRandomCommand() {
  
  geometry_msgs::Twist ret;
  
  ret.linear.x = m_ce.getCharacteristics().v_max;
  ret.angular.z = dis(gen);
  ret.linear.y = ret.linear.z = ret.angular.x = ret.angular.y = 0.0;
  
  return ret;
}

#endif