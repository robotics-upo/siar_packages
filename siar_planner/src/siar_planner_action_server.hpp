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

//! @class This class takes as inputs a desired 2D pose (with yaw) or a desired direction in the fork and tries to get a path
//! TODO: output a sequence of cmd_vel!! 

class SiarPlannerActionServer 
{
public:
  
  SiarPlannerActionServer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  //! @brief Start new action
  void passFork(const siar_planner::PassForkGoal::ConstPtr& goal);
  
  bool getCurrentVel(geometry_msgs::Twist &curr_cmd);
  
  double getDeltaT() const {return a_star.getDeltaT();}
  
protected:
  ros::NodeHandle nh_;
  typedef actionlib::SimpleActionServer<siar_planner::PassForkAction> PFActionServer;
  PFActionServer PFActionServer_;
  ros::Subscriber reverse_sub, goal_sub; // Also the commands can be sent as a regular topic (without action server)
  ros::Publisher path_pub; 
  ros::Publisher graph_pub;
  
  // Path related info
  std::list<AStarNode> curr_path;
  ros::Time start_time, end_time;
  
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
  
  // TODO: transform the pose (now it assumes it is indicated in base_link
  
  goal.state.push_back(pose.pose.position.x);
  goal.state.push_back(pose.pose.position.y);
  
  // Conversion from quaternion message to yaw (TODO: Is there a better way to do this?)
  double roll, pitch, yaw;
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  goal.state.push_back(yaw);
  
  double cost = a_star.getPath(start, goal, curr_path);
  
  t1 = ros::Time::now();
  ROS_INFO("Path calculated. Cost = %f.\t Expended time: %f", cost, (t1 - t0).toSec());
  
  visualization_msgs::Marker m = a_star.getPathMarker(curr_path);
  path_pub.publish(m);
  graph_pub.publish(a_star.getGraphMarker());
  
  start_time = ros::Time::now();
  end_time = start_time + ros::Duration(a_star.getDeltaT() * curr_path.size());
}

void SiarPlannerActionServer::goalCB(const geometry_msgs::PoseStamped_< std::allocator< void > >::ConstPtr& msg)
{
  calculatePath(*msg);
}

bool SiarPlannerActionServer::getCurrentVel(geometry_msgs::Twist& curr_cmd)
{
  ros::Time t = ros::Time::now();
  
  curr_cmd.linear.y = curr_cmd.linear.z = curr_cmd.angular.x = curr_cmd.angular.y = 0.0;
  curr_cmd.linear.x = curr_cmd.angular.z = 0.0;
  
  if (t < start_time || t > end_time) {
    return false;
  }
  
  double mission_t = (t - start_time).toSec();
  
  int n = floor(mission_t / a_star.getDeltaT() );
  
  ROS_INFO("getCurrentVel. T - Start_time= %f\t n = %d", (t - start_time).toSec(), n);
  
  int cont = 0;
  auto it = curr_path.begin();
  for (; cont < n; cont++, it++) {
  }
  
  
  
  curr_cmd.linear.x = it->command_lin;
  curr_cmd.angular.z = it->command_ang;
  
  
  return true;
}



#endif