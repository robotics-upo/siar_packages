#ifndef SIAR_PLANNER_ACTION_SERVER_HPP__
#define SIAR_PLANNER_ACTION_SERVER_HPP__

#include "siar_planner/PassForkAction.h"
#include <ros/ros.h>
#include "siar_planner/Astar.hpp"
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/OccupancyGrid.h>

class SiarPlannerActionServer 
{
public:
  
  SiarPlannerActionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  //! @brief Start new action
  void passFork(const siar_planner::PassForkGoal::ConstPtr& goal);
  
protected:
  ros::NodeHandle nh_;
  typedef actionlib::SimpleActionServer<siar_planner::PassForkAction> PFActionServer;
  PFActionServer PFActionServer_;
  ros::Subscriber reverse_sub;
  ros::Publisher cmd_vel_pub; // Will command the commanded velocity of the plan
  
  // Planner
  AStar a_star;
  
  // Map related stuff
  ros::Subscriber *map_sub_;
  bool has_map;
  nav_msgs::OccupancyGrid nav_map;
  
  // Status flag
  bool reverse;
  
  
  
  
  // ROS Communication stuff -----------------------------
  //! @brief To cancel an ongoing action
  void preemptCB();
  
  void reverseCB(const std_msgs::Bool::ConstPtr &msg) {
    reverse = (msg->data != 0);
  }
  
  void mapCB(const nav_msgs::OccupancyGrid &map);
};

SiarPlannerActionServer::SiarPlannerActionServer(ros::NodeHandle& nh, ros::NodeHandle& pnh):PFActionServer_(nh_, "pass_fork", boost::bind(&SiarPlannerActionServer::passFork, this, _1), false), 
a_star(nh, pnh),map_sub_(NULL), has_map(false),reverse(false)
{
  //register the goal and feeback callbacks
//   PFActionServer_.registerPreemptCallback(boost::bind(&SiarPlannerActionServer::preemptCB, this, _1));

  //subscribe to the data topic of interest
  reverse_sub = nh.subscribe("/reverse", 1, &SiarPlannerActionServer::reverseCB, this);
  PFActionServer_.start();
}

void SiarPlannerActionServer::passFork(const siar_planner::PassForkGoal_< std::allocator< void > >::ConstPtr& goal_msg)
{
  AStarState start, goal;
  start.state.push_back(0);start.state.push_back(0);start.state.push_back(0);
  
  // TODO: How to infere the goal from the map?
  has_map = false;
  ros::Rate r(0.1);
  
  // Subscribe to the map topic (only when needed)
  map_sub_ = new ros::Subscriber;
  *map_sub_ = nh_.subscribe("/map", 1, &SiarPlannerActionServer::mapCB, this);
  
  while (!has_map) {
    ros::spinOnce();
    r.sleep();
  }
  
  
  if (goal_msg->direction == 0) {
    // Straight --> just go forward a certain distance? 
    
    goal.state.push_back(4.0);
    goal.state.push_back(0.0);
    goal.state.push_back(0.0); // No rotation?
  }
  
  std::list<AStarNode> path;
  
  a_star.getPath(start, goal, path);
  
  
  map_sub_->shutdown();
  delete map_sub_;
}

void SiarPlannerActionServer::mapCB(const nav_msgs::OccupancyGrid& map)
{
  if (!has_map) {
    has_map = true;
    nav_map = map;
  } else {
    //  If the original map is received, the following could be used for localization
    
  }
  
  
}



#endif