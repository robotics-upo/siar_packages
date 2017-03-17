#ifndef __SIAR_CONTROLLER_HPP__
#define __SIAR_CONTROLLER_HPP__

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <siar_controller/SiarControllerConfig.h>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include "command_evaluator.hpp"

namespace siar_controller {

class SiarController {
public:
  SiarController(ros::NodeHandle &nh, ros::NodeHandle &pn);
  
  bool computeCmdVel(geometry_msgs::Twist &cmd, const geometry_msgs::Twist &v_ini);
  
  ~SiarController();
  
protected:
  // Camera info subscribers and initialization
  ros::Subscriber costmap_sub, cmd_vel_sub, odom_sub;
  ros::Publisher cmd_vel_pub;
  
  // Dynamic reconfigure stuff TODO
  typedef dynamic_reconfigure::Server<SiarControllerConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;
  ReconfigureServer::CallbackType call_type;  
  SiarControllerConfig _conf;
  
  
  nav_msgs::OccupancyGrid last_map; // Saves the last map to make the calculations
  geometry_msgs::Twist last_command, last_velocity;
  bool occ_received;
  
  CommandEvaluator *cmd_eval;

  // Callbacks  
  void parametersCallback(SiarControllerConfig &config, uint32_t level);
  void altitudeCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void cmdvelCallback(const geometry_msgs::Twist &msg);
  void odomCallback(const nav_msgs::Odometry &msg);
  
  void loop();
};

SiarController::~SiarController()
{
  delete cmd_eval;
}

SiarController::SiarController(ros::NodeHandle& nh, ros::NodeHandle& pn):reconfigure_server_(),config_init_(false),occ_received(false), cmd_eval(NULL)
{
  // Dynamic reconfigure initialization
  reconfigure_server_.reset(new ReconfigureServer(pn));
  call_type = boost::bind(&SiarController::parametersCallback, this, _1, _2);
//   reconfigure_server_->setCallback(call_type);
//   
//   ROS_INFO("Waiting for dynamic reconfigure initial values.");
//   while (!config_init_ && ros::ok())
//   {
//     boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//   }
  ROS_DEBUG("Dynamic reconfigure configuration received.");
  
  if (!config_init_) {
    ROS_ERROR("Halted when reading the dynamic configuration");
    return;
  }
  
  // ROS publishers/subscribers
  // Now camera info subscribers
  costmap_sub = nh.subscribe("/altitude_map", 2, &SiarController::altitudeCallback, this);
  cmd_vel_sub = nh.subscribe("/cmd_vel_in", 2, &SiarController::cmdvelCallback, this);
  odom_sub = nh.subscribe("/odom", 2, &SiarController::odomCallback, this);
  
  // Now the publishers
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel_out"), 2);
  
  ros::Rate r(_conf.T);
  while (ros::ok()) {
    ros::spinOnce();
    loop();
    r.sleep();
  }
}


void SiarController::parametersCallback(SiarControllerConfig& config, uint32_t level)
{
  if (!config_init_) {
    // Declare the evaluator
    RobotCharacteristics model;
    model.a_max = 1.0;
    model.a_max_theta = 1.0;
    model.v_max = config.v_max;
    model.theta_dot_max = config.alpha_max;
    cmd_eval = new CommandEvaluator(config.w_dist, config.w_safe, config.T_hor, model, config.delta_T); // TODO: insert the footprint related data (now only default values)
  }
  _conf = config;
}



void SiarController::cmdvelCallback(const geometry_msgs::Twist& msg)
{
  // The callback only copies the data to the class
  last_command = msg;
}

void SiarController::altitudeCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  last_map = *msg;
  if (!occ_received) {
    
    ROS_INFO("SiarController --> Received first altitude map");
    occ_received = true;
  }
  
}

void SiarController::odomCallback(const nav_msgs::Odometry& msg)
{
  last_velocity = msg.twist.twist; // TODO: Check bounds
}


void SiarController::loop() {
  // Main loop --> we have to 
  geometry_msgs::Twist cmd_vel_msg = last_command;
  cmd_vel_msg.angular.x = 0.0;
  cmd_vel_msg.angular.y = 0.0;
  cmd_vel_msg.angular.z = 0.0; // TODO: Only discard rotation if a mode is activated
  if (_conf.operation_mode == 0) {
    // Manual --> bypass the last command
    cmd_vel_msg = last_command;
  } else if (!computeCmdVel(cmd_vel_msg, last_velocity)) {
    ROS_ERROR("Could not get a feasible velocity --> Stopping the robot");
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
  }
  cmd_vel_pub.publish(cmd_vel_msg);
}

bool SiarController::computeCmdVel(geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini)
{
  float vt_orig = cmd_vel.angular.z; // TODO: discard rotations?
  float vx_orig = cmd_vel.linear.x;

  float ang_vel_inc = 0.025;
  float lin_vel_dec = 0.05;

  double lowest_cost = 1e100;
  
  geometry_msgs::Twist best_cmd, curr_cmd;
  best_cmd = cmd_vel;
  best_cmd.linear.x = 0.0;
  best_cmd.angular.z = 0.0;
  
  curr_cmd = cmd_vel;
  
  if (!cmd_eval) {
    ROS_ERROR("SiarController::loop --> Command Evaluator is not configured\n");
  }

  //Linear vel
  for(unsigned int l = 0; l <= 3; l++) 
  { 
    if(fabs(curr_cmd.linear.x) < 0.1)
      continue;
    if (vx_orig > 0.0)
      curr_cmd.linear.x = vx_orig - l * lin_vel_dec;
    if (vx_orig < -0.0)
      curr_cmd.linear.x = vx_orig + l * lin_vel_dec;

    curr_cmd.angular.z = vt_orig;
    double curr_cost = cmd_eval->evualateTrajectory(v_ini, curr_cmd, cmd_vel, last_map);
    if (curr_cost < lowest_cost && curr_cost > 0.0) {
      best_cmd = curr_cmd;
      lowest_cost = curr_cost;
    }
    
    //Angular vel
    for(unsigned int v=1; v <= 5; v++)
    {
      //To the right
      curr_cmd.angular.z = vt_orig + ang_vel_inc * v;
      if(fabs(curr_cmd.angular.z) > cmd_eval->getCharacteristics().theta_dot_max)
              curr_cmd.angular.z = cmd_eval->getCharacteristics().theta_dot_max;
      
      curr_cost = cmd_eval->evualateTrajectory(v_ini, curr_cmd, cmd_vel, last_map);
      
      if (curr_cost < lowest_cost && curr_cost > 0.0) {
        best_cmd = curr_cmd;
      }

      //to the left
      curr_cmd.angular.z = vt_orig - ang_vel_inc * v;
      if(fabs(curr_cmd.angular.z) > cmd_eval->getCharacteristics().theta_dot_max)
              curr_cmd.angular.z = cmd_eval->getCharacteristics().theta_dot_max;
      
      double curr_cost = cmd_eval->evualateTrajectory(v_ini, curr_cmd, cmd_vel, last_map);
      
      if (curr_cost < lowest_cost && curr_cost > 0.0) {
        best_cmd = curr_cmd;
        lowest_cost = curr_cost;
      }
    }
  }
  cmd_vel = best_cmd;
}

}

#endif