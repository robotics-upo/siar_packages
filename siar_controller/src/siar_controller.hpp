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
#include <std_msgs/Int8.h>

#include <visualization_msgs/MarkerArray.h>

#include "command_evaluator.hpp"

namespace siar_controller {

class SiarController {
public:
  SiarController(ros::NodeHandle &nh, ros::NodeHandle &pn);
  
  bool computeCmdVel(geometry_msgs::Twist &cmd, const geometry_msgs::Twist &v_ini);
  
  ~SiarController();
  
protected:
  // Camera info subscribers and initialization
  ros::Subscriber costmap_sub, cmd_vel_sub, odom_sub, mode_sub;
  ros::Publisher cmd_vel_pub, footprint_marker_pub;
  
  // Operation mode
  int operation_mode;
  
  // Last command for taking into account the a_max
  geometry_msgs::Twist last_command;
  
  // Dynamic reconfigure stuff
  typedef dynamic_reconfigure::Server<SiarControllerConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;
  ReconfigureServer::CallbackType call_type;  
  SiarControllerConfig _conf;
  
  // Representation stuff
  visualization_msgs::MarkerArray markers;
  void copyMarker(visualization_msgs::Marker &dst , const visualization_msgs::Marker &orig) const;
  visualization_msgs::Marker m;
  
  // Occ. stuff
  nav_msgs::OccupancyGrid last_map; // Saves the last map to make the calculations
  bool occ_received;

  // Command evaluation
  geometry_msgs::Twist user_command, last_velocity;
  CommandEvaluator *cmd_eval;
  
  
  // Optimization search
  float ang_vel_inc;
  float lin_vel_dec;
  double lowest_cost, curr_cost;
  geometry_msgs::Twist best_cmd, curr_cmd;
  int n_commands, n_best;
  void evaluateAndActualizeBest(const geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini);

  // Callbacks  
  void parametersCallback(SiarControllerConfig &config, uint32_t level);
  void altitudeCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void cmdvelCallback(const geometry_msgs::Twist &msg);
  void odomCallback(const nav_msgs::Odometry &msg);
  void modeCallback(const std_msgs::Int8 &msg);
  
  // Get the test velocity set 
  std::vector <geometry_msgs::Twist> getAccelTestSet(double v_x);
  std::vector <geometry_msgs::Twist> getDiscreteTestSet(double v_command_x, bool force_recompute = false);
  std::vector <geometry_msgs::Twist> discrete_test_set_forward, discrete_test_set_backward;
  
  void loop();
};

SiarController::~SiarController()
{
  delete cmd_eval;
}

SiarController::SiarController(ros::NodeHandle& nh, ros::NodeHandle& pn):operation_mode(0),
reconfigure_server_(),config_init_(false),occ_received(false), cmd_eval(NULL)
{
  // Dynamic reconfigure initialization
  reconfigure_server_.reset(new ReconfigureServer(pn));
  call_type = boost::bind(&SiarController::parametersCallback, this, _1, _2);
  reconfigure_server_->setCallback(call_type);
//   
  ROS_INFO("Waiting for dynamic reconfigure initial values.");
  while (!config_init_ && ros::ok())
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    ros::spinOnce();
  }
  
  
  if (!config_init_) {
    ROS_ERROR("Halted when reading the dynamic configuration");
    return;
  }
  ROS_INFO("Dynamic reconfigure configuration received.");
  
  // ROS publishers/subscribers
  // Now camera info subscribers
  costmap_sub = nh.subscribe("/altitude_map", 2, &SiarController::altitudeCallback, this);
  cmd_vel_sub = nh.subscribe("/cmd_vel_in", 2, &SiarController::cmdvelCallback, this);
  odom_sub = nh.subscribe("/odom", 2, &SiarController::odomCallback, this);
  mode_sub = nh.subscribe("/operation_mode", 2, &SiarController::modeCallback, this);
  
  // Now the publishers
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel_out"), 2);
  footprint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_marker", 10);
  
  ROS_INFO("Update Rate: %f", _conf.T);
  ros::Rate r(1.0/_conf.T);
  while (ros::ok()) {
    ros::spinOnce();
    loop();
    r.sleep();
  }
  last_command.linear.x = last_command.linear.y = last_command.linear.z = 0.0;
  last_command.angular.x = last_command.angular.y = last_command.angular.z = 0.0;
  
  // Initialize odometry measures: TODO
  
}


void SiarController::parametersCallback(SiarControllerConfig& config, uint32_t level)
{
  _conf = config;
  if (!config_init_) {
    // Declare the evaluator
    config_init_ = true;
    RobotCharacteristics model;
    model.a_max = config.a_max;
    model.a_max_theta = config.a_theta_max;
    model.v_max = config.v_max;
    model.theta_dot_max = config.alpha_max;
//     ROS_INFO("R_L = /*%*/f, R_W = %f, W_W = %f", config.robot_longitude, config.robot_width, config.wheel_width);
    SiarFootprint *p = new SiarFootprint(0.025, config.robot_longitude, config.robot_width, config.wheel_width);
    cmd_eval = new CommandEvaluator(config.w_dist, config.w_safe, config.T_hor, model, config.delta_T, p); // TODO: insert the footprint related data (now only default values)
  } else {
    RobotCharacteristics model;
    model.a_max = config.a_max;
    model.a_max_theta = config.a_theta_max;
    model.v_max = config.v_max;
    model.theta_dot_max = config.alpha_max;
    SiarFootprint *p = new SiarFootprint(0.025, config.robot_longitude, config.robot_width, config.wheel_width);
    cmd_eval->setParameters(config.w_dist, config.w_safe, config.T_hor, model, config.delta_T, p);
  }
  markers.markers.reserve(_conf.n_lin * (_conf.n_ang + 1) * 2 + 20);
  ang_vel_inc = _conf.a_max * _conf.delta_T / (float)_conf.n_ang;
  lin_vel_dec = _conf.a_theta_max * _conf.delta_T/ (float)_conf.n_lin;
  
  getDiscreteTestSet(0.6, true);
  
}



void SiarController::cmdvelCallback(const geometry_msgs::Twist& msg)
{
  // The callback only copies the data to the class
  user_command = msg;
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

void SiarController::modeCallback(const std_msgs::Int8& msg)
{
  operation_mode = msg.data;
  ROS_INFO("Operation mode changed to: %d", operation_mode);
}



void SiarController::loop() {
  // Main loop --> we have to 
  geometry_msgs::Twist cmd_vel_msg = user_command;
  if (operation_mode == 1)
    cmd_vel_msg.angular.z = 0.0;
  
  if (operation_mode != 0) {
//     if (occ_received && !computeCmdVel(cmd_vel_msg, last_velocity)) {
    if (occ_received && !computeCmdVel(cmd_vel_msg, last_command)) {
      ROS_ERROR("Could not get a feasible velocity --> Stopping the robot");
      // Stop the robot 
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_msg.linear.x = 0.0;
    } else if (!occ_received) 
      ROS_INFO("SiarController --> Warning: no altitude map");
  }
  cmd_vel_pub.publish(cmd_vel_msg);
  last_command = cmd_vel_msg;
}

bool SiarController::computeCmdVel(geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini)
{
  if (!cmd_eval) {
    ROS_ERROR("SiarController::loop --> Command Evaluator is not configured\n");
    return -1.0;
  }
  
  if (fabs(cmd_vel.linear.x) < lin_vel_dec && fabs(last_command.linear.x) < lin_vel_dec*2.0) {
    cmd_vel.angular.z = 0.0;
    cmd_vel.linear.x = 0.0;
    return true;
  }
  
  // Initialize search
  n_commands = 0;
  n_best = -1;
  lowest_cost = 1e100;
  best_cmd = cmd_vel;
  best_cmd.linear.x = 0.0;
  
  // Get test set (different options available)
  std::vector<geometry_msgs::Twist> test_set;
//   test_set = getAccelTestSet(cmd_vel.linear.x);
  test_set = getDiscreteTestSet(cmd_vel.linear.x);
  markers.markers.resize(test_set.size());
  
  ROS_INFO (" Checking %d velocities. ", (int) test_set.size());
  
  //Linear vel
  for (unsigned int i = 0; i < test_set.size(); i++) 
  { 
    curr_cmd = test_set.at(i);
    evaluateAndActualizeBest(cmd_vel, v_ini);
  }
  
  ROS_INFO("End loop: Best command: %f,%f \t Orig command %f, %f",
    best_cmd.linear.x, best_cmd.angular.z, cmd_vel.linear.x, cmd_vel.angular.z);

  if (test_set.size() > 0) {
    footprint_marker_pub.publish(markers);
  }
  cmd_vel = best_cmd;
  cmd_vel.angular.z *= -1.0; // TODO: necessary?
  return lowest_cost < 1e50;
}

std::vector< geometry_msgs::Twist > SiarController::getAccelTestSet(double x)
{
  std::vector<geometry_msgs::Twist> ret;
  
  curr_cmd = last_command;
  
  float vt_orig = last_command.angular.z; // TODO: discard rotations?
  float vx_orig = last_command.linear.x;
  
  vx_orig += lin_vel_dec * boost::math::sign(x - vx_orig);
  if (vx_orig > _conf.v_max) 
    vx_orig = _conf.v_max;
  
  
  for(int l = 0; l <= _conf.n_lin; l++) 
  { 
    curr_cmd.angular.z = vt_orig;
    curr_cmd.linear.x = vx_orig - l * lin_vel_dec * boost::math::sign(vx_orig);
    if(fabs(curr_cmd.linear.x) < lin_vel_dec) {
      if (l == 0) {
        curr_cmd.angular.z = 0.0;
      }
      ret.push_back(curr_cmd);
      break;
    }

    ret.push_back(curr_cmd);
    
    //Angular vel
    for(int v=1; v <= _conf.n_ang; v++)
    {
      //To the left
      curr_cmd.angular.z = vt_orig + ang_vel_inc * v;
      ret.push_back(curr_cmd);
      //to the right
      curr_cmd.angular.z = vt_orig - ang_vel_inc * v;
      ret.push_back(curr_cmd);
    }
  }
  
  return ret;
}

std::vector< geometry_msgs::Twist > SiarController::getDiscreteTestSet(double v_command_x, bool force_recompute)
{
  ROS_INFO("Get Discrete Test set: n_lin = %d n_ang = %d" , _conf.n_lin, _conf.n_ang);
  if (discrete_test_set_forward.size() == 0 || force_recompute) {
    double ang_vel_inc = _conf.alpha_max / (double)_conf.n_ang;
    discrete_test_set_forward.clear();
    discrete_test_set_backward.clear();
    curr_cmd.angular.x = curr_cmd.angular.y = curr_cmd.angular.z = 0.0;
    curr_cmd.linear.x = curr_cmd.linear.y = curr_cmd.linear.z = 0.0;
    for (int i = 1; i <= _conf.n_lin; i++) {
      curr_cmd.angular.z = 0.0;
      curr_cmd.linear.x = (double)i * _conf.v_max / (double)_conf.n_lin;
      discrete_test_set_forward.push_back(curr_cmd);
      ROS_INFO("Discrete test set. Command %d. vx = %f. v_theta = %f", (int)discrete_test_set_forward.size(), curr_cmd.linear.x, curr_cmd.angular.z);
      curr_cmd.linear.x *= -1.0;
      discrete_test_set_backward.push_back(curr_cmd);
      for (int j = 1; j <= _conf.n_ang; j++) {
        // To the left
        curr_cmd.angular.z = ang_vel_inc * j;
        discrete_test_set_backward.push_back(curr_cmd);
        curr_cmd.linear.x *= -1.0;
        discrete_test_set_forward.push_back(curr_cmd);
        ROS_INFO("Discrete test set. Command %d. vx = %f. v_theta = %f", (int)discrete_test_set_forward.size(), curr_cmd.linear.x, curr_cmd.angular.z);
        //to the right
        curr_cmd.angular.z = - ang_vel_inc * j;
        discrete_test_set_forward.push_back(curr_cmd);
        ROS_INFO("Discrete test set. Command %d. vx = %f. v_theta = %f", (int)discrete_test_set_forward.size(), curr_cmd.linear.x, curr_cmd.angular.z);
        curr_cmd.linear.x *= -1.0;
        discrete_test_set_backward.push_back(curr_cmd);
      }
    }
  }
  if (v_command_x > 0.05) 
    return discrete_test_set_forward;
  else if (v_command_x < -0.05)
    return discrete_test_set_backward;
  
  std::vector< geometry_msgs::Twist > ret;
  return ret;
}



void SiarController::evaluateAndActualizeBest(const geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini)
{
  curr_cost = cmd_eval->evualateTrajectory(v_ini, curr_cmd, cmd_vel, last_map, m);
  if (curr_cost < lowest_cost && curr_cost >= 0.0) {
    best_cmd = curr_cmd;
    lowest_cost = curr_cost;
        
    if (n_best >= 0) {
      markers.markers[n_best].color.g = 1.0;
      markers.markers[n_best].color.r = 0.0;
      markers.markers[n_best].color.b = 0.0;
      markers.markers[n_best].color.a = 0.7;
    }

  
    n_best = n_commands;
    m.color.b = 1.0;
    m.color.r = 1.0; // Best marker on purple
    m.color.g = 0.0;
    m.color.a = 1.0;
  } else if (curr_cost > 0.0) {
    m.color.b = 0.0;
    m.color.r = 0.0; // Best marker on green
    m.color.g = 1.0;
    m.color.a = 0.7;
  } else {
    m.color.b = 0.0; // Collision
    m.color.r = 1.0; // Best marker on red
    m.color.g = 0.0;
    m.color.a = 0.7;
  }
  
  copyMarker(markers.markers[n_commands], m);
  markers.markers[n_commands].id = n_commands;
  
  n_commands++;
//   ROS_INFO("N_commands = %d", n_commands);
}


void SiarController::copyMarker(visualization_msgs::Marker& dst, const visualization_msgs::Marker& orig) const
{
  dst.points.resize(orig.points.size());
  dst.points.assign(orig.points.begin(), orig.points.end());
  dst.color = orig.color;
  dst.scale = orig.scale;
  dst.action = orig.action;
  dst.header = orig.header;
  
  dst.type = orig.type;
  dst.lifetime = ros::Duration(_conf.T); // TODO: check the fields to copy
}


}

#endif