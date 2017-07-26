#ifndef SIAR_PLANNER_ACTION_SERVER_HPP__
#define SIAR_PLANNER_ACTION_SERVER_HPP__

#include "siar_planner/PassForkAction.h"
#include <ros/ros.h>
#include "siar_planner/Astar.hpp"
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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
  ros::Subscriber reverse_sub, goal_sub; // Also the commands can be sent as a regular topic (without action server)
  ros::Publisher cmd_vel_pub, path_pub; // Will command the commanded velocity of the 
  ros::Publisher graph_pub;
  
  // Planner
  AStar a_star;
  
  // Status flag
  bool reverse;
  
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
  
  
  // ROS Communication stuff -----------------------------
  //! @brief To cancel an ongoing action
  void preemptCB();
  
  void reverseCB(const std_msgs::Bool::ConstPtr &msg) {
    reverse = (msg->data != 0);
  }
  
  //! @brief Calculates path and gets the 
  void calculatePath(const geometry_msgs::PoseStamped &pose);
};

SiarPlannerActionServer::SiarPlannerActionServer(ros::NodeHandle& nh, ros::NodeHandle& pnh):PFActionServer_(nh_, "pass_fork", boost::bind(&SiarPlannerActionServer::passFork, this, _1), false), 
a_star(nh, pnh),reverse(false)
{
  //register the goal and feeback callbacks
//   PFActionServer_.registerPreemptCallback(boost::bind(&SiarPlannerActionServer::preemptCB, this, _1));
  
  // Publishers
  path_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 2, true);
  graph_pub = nh.advertise<visualization_msgs::Marker>("graph_marker", 2, true);

  // Subscribers
  reverse_sub = nh.subscribe("/reverse", 1, &SiarPlannerActionServer::reverseCB, this);
  goal_sub = nh.subscribe("/move_base_simple/goal", 1, &SiarPlannerActionServer::goalCB, this);
  // Start server
  PFActionServer_.start();
}

void SiarPlannerActionServer::passFork(const siar_planner::PassForkGoal_< std::allocator< void > >::ConstPtr& goal_msg)
{
  // TODO: How to infere the goal from the map?
  ros::Rate r(0.1);
  
  if (goal_msg->direction == 0) {
    // Straight --> just go forward a certain distance? 
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = a_star.getModel().getFrameID();
  }
}

void SiarPlannerActionServer::calculatePath(const geometry_msgs::PoseStamped &pose) {
  AStarState start, goal;
  ros::Time t0, t1;
  t0 = ros::Time::now();
  start.state.push_back(0);start.state.push_back(0);start.state.push_back(0);
  
  goal.state.push_back(pose.pose.position.x);
  goal.state.push_back(pose.pose.position.y);
  
  // Conversion from quaternion message to yaw (TODO: Is there a better way to do this?)
  double roll, pitch, yaw;
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  goal.state.push_back(yaw);
  
  std::list<AStarNode> path;
  double cost = a_star.getPath(start, goal, path);
  
  t1 = ros::Time::now();
  ROS_INFO("Path calculated. Cost = %f.\t Expended time: %f", cost, (t1 - t0).toSec());
  
  visualization_msgs::Marker m = a_star.getPathMarker(path);
  path_pub.publish(m);
  graph_pub.publish(a_star.getGraphMarker());
}

void SiarPlannerActionServer::goalCB(const geometry_msgs::PoseStamped_< std::allocator< void > >::ConstPtr& msg)
{
  calculatePath(*msg);
}




#endif