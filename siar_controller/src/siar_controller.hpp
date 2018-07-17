#ifndef __SIAR_CONTROLLER_HPP__
#define __SIAR_CONTROLLER_HPP__

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <siar_controller/SiarControllerConfig.h>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include <functions/functions.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>

#include <visualization_msgs/MarkerArray.h>

#include "siar_controller/command_evaluator.hpp"
#include "siar_driver/SiarStatus.h"

#include <stdlib.h>
#include <ctime>

namespace siar_controller {

class SiarController {
public:
  //! @brief Default constructor
  SiarController(ros::NodeHandle &nh, ros::NodeHandle &pn);
  
  //! @brief Gets the best command
  //! @param mode If 2 --> Allows wheels to enter inside the gutter
  bool computeCmdVel(geometry_msgs::Twist &cmd, const geometry_msgs::Twist &v_ini);
  
  ~SiarController();
  
protected:
  // Camera info subscribers and initialization
  ros::Subscriber costmap_sub, cmd_vel_sub, odom_sub, mode_sub, planned_cmd_vel_sub, status_sub;
  ros::Publisher cmd_vel_pub, footprint_marker_pub, trajectory_marker_pub;
  
  // Operation mode
  int operation_mode;
  std::vector <double> min_wheel_left, min_wheel_right;
  
  // Last command for taking into account the a_max
  geometry_msgs::Twist last_command;
  
  // Dynamic reconfigure stuff
  bool use_dynamic_reconfigure;
  void initDynamicReconfigure(ros::NodeHandle& pn);
  typedef dynamic_reconfigure::Server<SiarControllerConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;
  ReconfigureServer::CallbackType call_type;  
  SiarControllerConfig _conf;
  RobotCharacteristics model;
  
  // Representation stuff
  visualization_msgs::MarkerArray markers;
  void copyMarker(visualization_msgs::Marker &dst , const visualization_msgs::Marker &orig) const;
  visualization_msgs::Marker m, best;
  
  // Occ. stuff
  nav_msgs::OccupancyGrid last_map; // Saves the last map to make the calculations
  bool occ_received;

  // Command evaluation
  geometry_msgs::Twist user_command, last_velocity, planned_cmd;
  CommandEvaluator *cmd_eval;
  double width_thres;
  double safety_width;
  
  // Include ways to escape the local minima
  double t_unfeasible,ang_scape_inc, max_t_unfeasible;
  
  // Optimization search
  float ang_vel_inc;
  float lin_vel_dec;
  double lowest_cost, curr_cost;
  geometry_msgs::Twist best_cmd, curr_cmd;
  int n_commands, n_best;
  
  //! @brief Evaluates a command and then actualizes the best velocity so far if necessary
  void evaluateAndActualizeBest(const geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini);

  // Callbacks  
  void parametersCallback(SiarControllerConfig &config, uint32_t level);
  void altitudeCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void cmdvelCallback(const geometry_msgs::Twist &msg);
  void cmdvelCallbackPlanned(const geometry_msgs::Twist &msg);
  void odomCallback(const nav_msgs::Odometry &msg);
  void modeCallback(const std_msgs::Int8 &msg);
  void statusCallback(const siar_driver::SiarStatus &msg);
  
  //! @brief Get parameters from parameter server
  void getParameters(ros::NodeHandle &pn);
  
  //! @brief Get the test velocity set 
  std::vector <geometry_msgs::Twist> getAccelTestSet(double v_x);
  //! @brief Initializes the discrete_test_set_forward and discrete_test_set_backward according to the configuration information
  void initializeDiscreteTestSet();
  //! @brief Retrieves the file_test_set_forward and backward from file (the filename is specified in the configuration)
  void initializeTestSetFromFile(const std::string &velocityset_filename, int n_interpol = 1, double mult = 1);
  
  bool file_test_set_init;
  std::vector <geometry_msgs::Twist> discrete_test_set_forward, discrete_test_set_backward;
  std::vector <geometry_msgs::Twist> file_test_set_forward, file_test_set_backward;
  
  //! @brief Adds a velocity to the forward test, the same velocity with opposite angular velocity. Then does the same to backward test but changing the sign of v.x
  void addVelocityToTestSets(const geometry_msgs::Twist &v, std::vector<geometry_msgs::Twist> &set_forward, std::vector<geometry_msgs::Twist> &set_backward);
  
  void loop();
};

SiarController::~SiarController()
{
  delete cmd_eval;
}

SiarController::SiarController(ros::NodeHandle& nh, ros::NodeHandle& pn):operation_mode(0),
reconfigure_server_(),config_init_(false),occ_received(false), cmd_eval(NULL), t_unfeasible(0), ang_scape_inc(0.05), max_t_unfeasible(0.8)
{
  getParameters(pn);
  
  if (use_dynamic_reconfigure) {
    config_init_ = false;
    initDynamicReconfigure(pn);
  }
  
  // ROS publishers/subscribers
  // Now camera info subscribers
  costmap_sub = nh.subscribe("/altitude_map", 2, &SiarController::altitudeCallback, this);
  cmd_vel_sub = nh.subscribe("/cmd_vel_in", 2, &SiarController::cmdvelCallback, this);
  planned_cmd_vel_sub = nh.subscribe("/planned_cmd_vel", 2, &SiarController::cmdvelCallbackPlanned, this);
  odom_sub = nh.subscribe("/odom", 2, &SiarController::odomCallback, this);
  mode_sub = nh.subscribe("/operation_mode", 2, &SiarController::modeCallback, this);
  
  // Now the publishers
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel_out"), 2);
  trajectory_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_marker", 10);
  footprint_marker_pub = nh.advertise<visualization_msgs::Marker>("/best_marker", 10);

    status_sub = nh.subscribe("/siar_status", 2, &SiarController::statusCallback, this);

  
//   ROS_INFO("Update Rate: %f", _conf.T);
  ros::Rate r(1.0/_conf.T);
  while (ros::ok()) {
    ros::spinOnce();
    loop();
    r.sleep();
  }
  last_command.linear.x = last_command.linear.y = last_command.linear.z = 0.0;
  last_command.angular.x = last_command.angular.y = last_command.angular.z = 0.0;
  
  // Initialize random sequence
  srand((unsigned)std::time(NULL));


}

void SiarController::getParameters(ros::NodeHandle& pn)
{
  config_init_ = true;
  pn.param("use_dynamic_reconfigure", use_dynamic_reconfigure, false);
  
  model = RobotCharacteristics(pn);
  
  double r_l, r_w, w_w;
  pn.param("robot_longitude", r_l, 0.78);
  pn.param("robot_width", r_w, 0.56);
  pn.param("wheel_width", w_w, 0.075);
//     ROS_INFO("R_L = /*%*/f, R_W = %f, W_W = %f", r_l, r_w, w_w);
  SiarFootprint *p = new SiarFootprint(0.025, r_l, r_w, w_w);
  
  pn.param("w_dist", _conf.w_dist, 1.0);
  pn.param("w_safe", _conf.w_safe, 1.0);
  pn.param("T_hor", _conf.T_hor, 3.0);
  pn.param("delta_T", _conf.delta_T, 0.2);
  pn.param("T", _conf.T, 0.1);
  pn.param("n_lin", _conf.n_lin, 3);
  pn.param("n_ang", _conf.n_ang, 12);
  pn.param("width_thres", width_thres, 0.02);
  pn.param("safety_width", safety_width, 0.02);
  pn.getParam("min_wheel_left", min_wheel_left);
  pn.getParam("min_wheel_right", min_wheel_right);
  
  markers.markers.reserve(_conf.n_lin * (_conf.n_ang + 1) * 2 + 20);
  ang_vel_inc = model.a_max * _conf.delta_T / (float)_conf.n_ang;
  lin_vel_dec = model.a_max_theta * _conf.delta_T/ (float)_conf.n_lin;
  
  initializeDiscreteTestSet();
  
  std::string velocityset_filename;
  pn.param("velocityset_filename", velocityset_filename, std::string("velocityset_file"));
  
  // Perform interpolation between consecutive velocity fields
  int n_interpols = 2;
  pn.getParam("n_interpols", n_interpols);
  file_test_set_init = false;
  // Adjust the velocity and other date according to v_mult
  pn.param("v_mult", _conf.v_mult, 1.0);
  initializeTestSetFromFile(velocityset_filename, n_interpols, _conf.v_mult);
  _conf.delta_T /= _conf.v_mult;
  _conf.T_hor /= _conf.v_mult;
  
  cmd_eval = new CommandEvaluator(_conf.w_dist, _conf.w_safe, _conf.T_hor, model, _conf.delta_T, p); // TODO: insert the footprint related data (now only default values)
  
  config_init_ = true;
}

void SiarController::initDynamicReconfigure(ros::NodeHandle &pn)
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
}



void SiarController::parametersCallback(SiarControllerConfig& config, uint32_t level)
{
  _conf = config;
  if (!config_init_) {
    // Declare the evaluator
    config_init_ = true;
    model.a_max = config.a_max;
    model.a_max_theta = config.a_theta_max;
    model.v_max = config.v_max;
    model.theta_dot_max = config.alpha_max;
//     ROS_INFO("R_L = /*%*/f, R_W = %f, W_W = %f", config.robot_longitude, config.robot_width, config.wheel_width);
    SiarFootprint *p = new SiarFootprint(0.025, config.robot_longitude, config.robot_width, config.wheel_width);
    cmd_eval = new CommandEvaluator(config.w_dist, config.w_safe, config.T_hor, model, config.delta_T, p); 
  } else {
    model.a_max = config.a_max;
    model.a_max_theta = config.a_theta_max;
    model.v_max = config.v_max;
    model.theta_dot_max = config.alpha_max;
    SiarFootprint *p = new SiarFootprint(0.025, config.robot_longitude, config.robot_width, config.wheel_width);
    cmd_eval->setParameters(config.w_dist, config.w_safe, config.T_hor, model, config.delta_T, p);
  }
  markers.markers.reserve(_conf.n_lin * (_conf.n_ang + 1) * 2 + 20);
  ang_vel_inc = model.a_max * _conf.delta_T / (float)_conf.n_ang;
  lin_vel_dec = model.a_max_theta * _conf.delta_T/ (float)_conf.n_lin;
  
  initializeDiscreteTestSet();
}

void SiarController::cmdvelCallback(const geometry_msgs::Twist& msg)
{
  // The callback only copies the data to the class
  user_command = msg;
}

void SiarController::cmdvelCallbackPlanned(const geometry_msgs::Twist& msg)
{
  planned_cmd = msg;
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
  if (operation_mode > 0) {
    if (operation_mode == 100) {
      // Bypass the planned velocity
      // Fully autonomous mode TODO: Check it!!
      cmd_vel_msg = planned_cmd;
    }
    if (!occ_received) {
      ROS_INFO("SiarController --> Warning: no altitude map");
    } else if (!computeCmdVel(cmd_vel_msg, last_command)) {
      if (fabs(user_command.linear.x) >= lin_vel_dec) {
        // The USER wants to go
        t_unfeasible +=_conf.T;
        
        ROS_ERROR("Could not get a feasible velocity --> Stopping the robot. Time without speed = %f", t_unfeasible);
        // Stop the robot 
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
      } else {
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        t_unfeasible = 0.0;
      } 
    } else {
      t_unfeasible = 0.0; // A valid command has been generated --> restart the time counter
      if (operation_mode != 1) {
        //cmd_vel_msg.angular.z *= 0.4;
        //cmd_vel_msg.linear.x *= 0.4;
      } else {
        double mult = fabs(user_command.linear.x / model.v_max);
	cmd_vel_msg.angular.z *= mult;
	cmd_vel_msg.linear.x *= mult;
      }
    }
  } 
  
  cmd_vel_pub.publish(cmd_vel_msg);
  last_command = cmd_vel_msg;
}

bool SiarController::computeCmdVel(geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini)
{
  if (!cmd_eval) {
    ROS_ERROR("SiarController::loop --> Command Evaluator is not configured\n");
    return false;
  }
  
  n_commands = 0;
  n_best = -1;
  lowest_cost = 1e100;
  best_cmd = cmd_vel;
  best_cmd.linear.x = 0.0;
  
  
  // Get test set (different options available)
  std::vector<geometry_msgs::Twist> test_set;
  if (operation_mode > 0) {
    if (file_test_set_forward.size() > 0) {
      if (cmd_vel.linear.x > model.v_min) 
        test_set = file_test_set_forward;
      else if (cmd_vel.linear.x < -model.v_min)
        test_set = file_test_set_backward;
    } else {
      test_set = getAccelTestSet(cmd_vel.linear.x);
    }
  }
  
  // Clear the previous marker
  best.points.resize(0);
  
  markers.markers.resize(test_set.size());
  
  ROS_INFO (" Checking %d velocities. ", (int) test_set.size());
  
  //Linear vel
  for (unsigned int i = 0; i < test_set.size(); i++) 
  { 
    curr_cmd = test_set.at(i);
    evaluateAndActualizeBest(cmd_vel, v_ini); // Actualizes best velocity and best marker
  }
  
  ROS_INFO("End loop: Best command: %f,%f \t Orig command %f, %f",
    best_cmd.linear.x, best_cmd.angular.z, cmd_vel.linear.x, cmd_vel.angular.z);

  if (test_set.size() > 0) {
    trajectory_marker_pub.publish(markers);
    footprint_marker_pub.publish(best);
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
  if (fabs(vx_orig) > model.v_max) 
    vx_orig = model.v_max * boost::math::sign(vx_orig);
  
  
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

void SiarController::initializeTestSetFromFile(const std::string& velocityset_filename, int n_interpol, double mult)
{
  if (!file_test_set_init) {
    file_test_set_init = true;
    file_test_set_forward.clear();
    file_test_set_backward.clear();
    
    std::vector<std::vector <double> > M;
    
    geometry_msgs::Twist v, v_ant, v_new;
    v.linear.x = v.linear.y = v.linear.z = 0.0;
    v.angular.x = v.angular.y = v.angular.z = 0.0;
    
    bool loaded = functions::getMatrixFromFile(velocityset_filename, M);
    if (loaded) 
    {
      for ( auto vec : M )
      {
        if (vec.size() < 2)
          continue;
        
        v.linear.x = vec.at(0) * mult;
        v.angular.z = vec.at(1) * mult;
        
        ROS_INFO("File test set. Forward Command %d. vx = %f. v_theta = %f", (int)file_test_set_forward.size(), v.linear.x, v.angular.z);
        addVelocityToTestSets(v, file_test_set_forward, file_test_set_backward);
        
        if (file_test_set_forward.size() > 1) {
          // Perform n_interpols
          v_new = v_ant;
          double inc_x = (v.linear.x - v_ant.linear.x) / (double)(n_interpol + 1);
          double inc_z = (v.angular.z - v_ant.angular.z) / (double)(n_interpol + 1);
          for (int i = 1; i <= n_interpol; i++) {
            v_new.linear.x += inc_x;
            v_new.angular.z += inc_z;
            addVelocityToTestSets(v_new, file_test_set_forward, file_test_set_backward);
          }
        }
        
        v_ant = v;
        
      }
    } else {
      ROS_ERROR("Could not load test velocity set from file: %s", velocityset_filename.c_str());
    }
  }
}

void SiarController::addVelocityToTestSets(const geometry_msgs::Twist& v1, std::vector< geometry_msgs::Twist >& set_forward, std::vector< geometry_msgs::Twist >& set_backward)
{
  geometry_msgs::Twist v = v1;
  // First forward velocities
  set_forward.push_back(v);
  if (fabs(v.angular.z) > 1e-10) {
    v.angular.z *= -1.0;
    set_forward.push_back(v);
  }
  // Then backward ones
  v.linear.x *= -1.0;
  set_backward.push_back(v);
  if (fabs(v.angular.z) > 1e-10) {
    v.angular.z *= -1.0;
    set_backward.push_back(v);
  }
}


void SiarController::initializeDiscreteTestSet()
{
  double ang_vel_inc = model.a_max_theta / (double)_conf.n_ang;
  discrete_test_set_forward.clear();
  discrete_test_set_backward.clear();
  curr_cmd.angular.x = curr_cmd.angular.y = curr_cmd.angular.z = 0.0;
  curr_cmd.linear.x = curr_cmd.linear.y = curr_cmd.linear.z = 0.0;
  for (int i = 0; i <= _conf.n_lin; i++) {
    curr_cmd.angular.z = 0.0;
    if (i == 0) 
      curr_cmd.linear.x = model.v_min; // Added the v_min option
    else
      curr_cmd.linear.x = model.v_min + (double)i * (model.v_max - model.v_min) / (double)_conf.n_lin;
    
    addVelocityToTestSets(curr_cmd, discrete_test_set_forward, discrete_test_set_backward);
    for (int j = 1; j <= _conf.n_ang / (i+1); j++) { // Added reduction of angular velocities as a function of the current speed
      // To the left
      curr_cmd.angular.z = ang_vel_inc * j;
      discrete_test_set_backward.push_back(curr_cmd);
      addVelocityToTestSets(curr_cmd, discrete_test_set_forward, discrete_test_set_backward);
    }
  }
}

void SiarController::evaluateAndActualizeBest(const geometry_msgs::Twist& cmd_vel, const geometry_msgs::Twist &v_ini)
{
  m.points.clear();
  if ((int)min_wheel_left.size()  < operation_mode + 1) 
  {
    ROS_ERROR("SiarController --> evaluateAndActualizeBest. Could not set the operation mode to %d", operation_mode);
    ROS_ERROR("M_wheel sizes --> L = %d\t R = %d", (int)min_wheel_left.size(), (int)min_wheel_right.size());
    return;
  }
  
  double m_w_l = min_wheel_left[operation_mode - 1];
  double m_w_r = min_wheel_right[operation_mode - 1];
  

  if ((m_w_l < 1e-3 && m_w_r < 1e-3) || 
    (m_w_l > 0.97 && m_w_r > 0.97)
  ) {
    curr_cost = cmd_eval->evaluateTrajectory(v_ini, curr_cmd, cmd_vel, last_map, m);
  } else {
    cmd_eval->setMinWheelLeft(m_w_l);
    cmd_eval->setMinWheelRight(m_w_r);
    curr_cost = cmd_eval->evaluateTrajectoryRelaxed(v_ini, curr_cmd, cmd_vel, last_map, m);
  }
    
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
    m.color.r = 0.2; // Best marker on blue
    m.color.g = 0.2;
    m.color.a = 1.0;
    
    copyMarker(best, m);
  } else if (curr_cost > 0.0) {
    m.color.b = 0.0;
    m.color.r = 0.0; // Feasible marker on green
    m.color.g = 1.0;
    m.color.a = 0.5;
  } else {
    m.color.b = 0.0; // Collision marker on red
    m.color.r = 1.0; 
    m.color.g = 0.0;
    m.color.a = 0.5;
  }
  copyMarker(markers.markers[n_commands], m);
  
  markers.markers[n_commands].id = n_commands;
  
  n_commands++;
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
  dst.lifetime = ros::Duration(_conf.T*3.0); // TODO: check the fields to copy
}

void SiarController::statusCallback(const siar_driver::SiarStatus& msg)
{
  // Check if the width has changed enough to perform an actualization of the footprint
  double new_width = msg.width - 0.04 + safety_width;
  if (fabs(new_width - _conf.robot_width) > width_thres && cmd_eval != NULL ) {
    cmd_eval->setWidth(new_width);
    _conf.robot_width = new_width;
    ROS_INFO("Setting new width footprint: %f", new_width);
  }
}



}

#endif
