/***********************************************************************/
/**                                                                    */
/** siar_teleop_joy.h                                                  */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Ignacio Perez-Hurtado                                              */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <stdlib.h>

///////////////////////////////////////////////////
// Default values for buttons and other values   //
// These values can be modified via rosparam     //
// NOTE: Axes inputs range [0,1] in ROS          //
// NOTE: button inputs are 0 or 1 (pressed) (int)//
// It has been configured to control with the   ///
// Logitech wireless pad controller              //

#define STOP_EXIT	      1
#define FREQ                100 
#define PANIC_FREQ          300
#define LINEAR_VELOCITY_AXIS  1
#define ANGULAR_VELOCITY_AXIS 0
#define AUTO_BUTTON           3
#define SLOW_BUTTON   	      5
#define MAX_VELOCITY_BUTTON   7
#define PANIC_BUTTON          2
#define START_BUTTON          9
#define BACK_BUTTON           8
#define REVERSE_BUTTON        0
// New buttons ARM and width
#define ARM_TORQUE_BUTTON     4
#define WIDTH_AXIS            2
#define WIDTH_AXIS_2          3
// --END NEW BUTTONS ---

#define MAX_LINEAR_VELOCITY   1.0
#define MAX_ANGULAR_VELOCITY  1.5707963

#define MIN_VEL_VARIATION     0.001
#define SLOW_MULTIPLIER       0.8

#define MAX_JOY_TIME          8.0
#define JOY_PRIORITY_TIME     5.0

#define MAX_AUTO_MODE         1
#define MAX_TIME_DECAY        0.98


////////////////////////////////////////////////////
// Variables that store the actual values        ///
// The ros-parameters have the same name         ///

// General inputs
int panicButton;
int startButton;
int backButton;
int reverseButton; // For enabling backwards mode

// Velocity command related inputs
double maxLinearVelocity;
double maxAngularVelocity;
double slow_multiplier;
int linearVelocityAxis;
int angularVelocityAxis;
int maxVelocityButton;
int slowButton;
int auto_button;

int arm_torque_button;
int width_pos_axis, width_pos_axis_2;
uint8_t arm_torque = 0; // Current state of arm_torque
int ant_arm_torque_button = 0;
double ant_width_pos = 0.0;

//////////////////////////////////

double currentLinearVelocity  = 0.0;
double currentAngularVelocity = 0.0;

//////////////////////////////////
// Status flags

bool panic          = false;
bool startPressed   = false;
bool backPressed   = false;
bool backwards = false;
bool ant_reverse_but = false;
bool slow_mode = false;
bool publishSlow = false;
bool ant_auto_button = false;
int auto_mode = 0;
int max_auto_mode;


ros::Time last_joy_time, last_remote_joy_time;
ros::Publisher reverse_pub;
ros::Publisher slow_pub;
ros::Publisher mode_pub;

ros::Publisher width_pos_pub;
ros::Publisher arm_torque_pub;

bool setAutomaticMode(int new_mode);

void interpretJoy(const sensor_msgs::Joy::ConstPtr& joy) {
  startPressed = joy->buttons[startButton] == 1;
  panic = panic | (joy->buttons[panicButton] == 1);
  backPressed = joy->buttons[backButton] == 1;
  
  
  if (!ant_auto_button && joy->buttons[auto_button] == 1) {
    // Request for mode change
    auto_mode++;
    if (auto_mode > max_auto_mode) 
      auto_mode = 0;
    setAutomaticMode(auto_mode);
  }
  ant_auto_button = joy->buttons[auto_button] == 1;
  
  if (!ant_reverse_but && joy->buttons[reverseButton] == 1) {
    backwards = !backwards;
    std_msgs::Bool msg;
    msg.data = backwards;
    reverse_pub.publish(msg);
  }
  if (panic)
  {
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    
    // TODO: Extend panic to width and arm!!!
  } 
  else
  {
    // If the max velocity button is not pressed --> velocity commands are attenuated
    double multiplier = (joy->buttons[maxVelocityButton] == 0)?0.5:1.0;
    if (slow_mode) {
      multiplier *= slow_multiplier;
    }
    
    // A positive angular velocity will rotate the reference frame to the left
    // While a positive axes value means that the stick is to the right --> change sign
    currentAngularVelocity =-maxAngularVelocity * multiplier * joy->axes[angularVelocityAxis];
    currentLinearVelocity = maxLinearVelocity * multiplier * joy->axes[linearVelocityAxis];
    currentLinearVelocity *= backwards?-1.0:1.0; // Only the linear velocity should change when going backwards
    
    if (joy->buttons[slowButton]) {
      slow_mode = !slow_mode;
      publishSlow = true;
    }
    
    // Arm Button
    int curr_arm_but = joy->buttons[arm_torque_button];
    if (curr_arm_but && curr_arm_but != ant_arm_torque_button) {
      arm_torque++;
      if (arm_torque > 2) {
        arm_torque = 0;
      }
      std_msgs::UInt8 msg;
      msg.data = arm_torque;
      arm_torque_pub.publish(msg);
    }
    ant_arm_torque_button = curr_arm_but;
    
    // Width velocity
    double width_pos = joy->axes[width_pos_axis]; // NOTE: the minus is to make the right commands be positive
    double width_pos_2 = joy->axes[width_pos_axis_2]; // NOTE: the minus is to make the right commands be positive
    double norm_sq = width_pos * width_pos + width_pos_2 * width_pos_2;
    ROS_INFO("Width pos = %f\tWidth pos 2 = %f\tnorm_sq = %f", width_pos, width_pos_2,norm_sq);
    if (norm_sq > 0.95 && width_pos > -0.05) {
      std_msgs::Float32 msg;
      msg.data = width_pos_2;
      width_pos_pub.publish(msg);
    } 
  }
}

void remoteJoyReceived(const sensor_msgs::Joy::ConstPtr& joy) {
  if ( (ros::Time::now() - last_joy_time).toSec() > JOY_PRIORITY_TIME) {
    last_remote_joy_time = ros::Time::now();
    interpretJoy(joy);
  }
}

void joyReceived(const sensor_msgs::Joy::ConstPtr& joy)
{
  // First of all, panic mode: if pressed --> the panic mode is activated (TODO: deactivate panic)
  last_joy_time = ros::Time::now();
  interpretJoy(joy);
}
  


void sendCmdVel(double linearVelocity, double angularVelocity, ros::Publisher& vel_pub)
{
  geometry_msgs::Twist vel;
  vel.angular.z = angularVelocity;
  vel.linear.x = linearVelocity;
  vel.linear.z = 0.0;
  vel.linear.y = 0.0;
  vel_pub.publish(vel);
}

void sendSlowCmd(bool slow_mode, ros::Publisher &pub) {
  std_msgs::Bool msg;
  msg.data = slow_mode;
  slow_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SiarTeleopJoy");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  bool stopExit;
  pn.param<bool>("stop_on_exit",stopExit, STOP_EXIT);
  
  double freq;
  pn.param<double>("freq",freq,FREQ);
  double panicFreq;
  pn.param<double>("panic_freq",panicFreq,PANIC_FREQ);
  
  pn.param<int>("panic_button", panicButton, PANIC_BUTTON);
  pn.param<int>("start_button", startButton, START_BUTTON);
  pn.param<int>("back_button", backButton, BACK_BUTTON);
  pn.param<int>("reverse_button", reverseButton, REVERSE_BUTTON);
  
  pn.param<int>("linear_velocity_axis",linearVelocityAxis,LINEAR_VELOCITY_AXIS);
  pn.param<int>("angular_velocity_axis",angularVelocityAxis,ANGULAR_VELOCITY_AXIS);
  
  pn.param<int>("auto_button", auto_button, AUTO_BUTTON);
  pn.param<int>("slow_button", slowButton, SLOW_BUTTON);
  pn.param<int>("turbo_button", maxVelocityButton, MAX_VELOCITY_BUTTON);
  
  pn.param<int>("width_pos_axis", width_pos_axis, WIDTH_AXIS);
  pn.param<int>("width_pos_axis_2", width_pos_axis_2, WIDTH_AXIS_2);
  pn.param<int>("arm_torque_button", arm_torque_button, ARM_TORQUE_BUTTON);
  
  pn.param<double>("max_linear_velocity",maxLinearVelocity,MAX_LINEAR_VELOCITY);
  pn.param<double>("max_angular_velocity",maxAngularVelocity,MAX_ANGULAR_VELOCITY);

  double max_joy_time, joy_priority_time;
  pn.param<double>("max_joy_time", max_joy_time, MAX_JOY_TIME);
  pn.param<double>("joy_priority_time", joy_priority_time, JOY_PRIORITY_TIME);
  
  
  pn.param<double>("slow_multipier", slow_multiplier, SLOW_MULTIPLIER);
  
  pn.param<int>("max_auto_mode", max_auto_mode, MAX_AUTO_MODE);
  double max_time_decay;
  pn.param<double>("max_time_decay", max_time_decay, MAX_TIME_DECAY);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  reverse_pub = n.advertise<std_msgs::Bool>("/reverse", 1);
  slow_pub = n.advertise<std_msgs::Bool>("/slow_motion", 1);
  mode_pub = n.advertise<std_msgs::Int8>("/operation_mode", 1 );
  
  // Width and arm
  width_pos_pub = n.advertise<std_msgs::Float32>("width_pos", 1);
  arm_torque_pub = n.advertise<std_msgs::UInt8>("arm_torque", 1);
  
  // ---------END WIDTH ARM ---
  
  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 5, joyReceived);
  ros::Subscriber remote_joy_sub = n.subscribe<sensor_msgs::Joy>("/remote_joy", 5, remoteJoyReceived);

  ros::Rate rate(freq);
  ros::Rate panicRate(panicFreq);
  bool panic_showed = false;
  bool start = false;
  bool first_start = true;
  ROS_INFO("Siar teleop node. Press START to have fun. Press BACK at any moment to exit.");
  while (n.ok() && !backPressed) {
    if (!start) 
    {
      sendCmdVel(0.0, 0.0, vel_pub);
      start = startPressed;
      rate.sleep();
    } 
    else if (panic) 
    {
      sendCmdVel(0.0, 0.0, vel_pub);
      panicRate.sleep();
      if (!panic_showed) 
      {
	panic_showed = true;
	ROS_ERROR("Panic mode ON. Press START to exit panic mode.");
      }
      if (startPressed) 
      {
	panic = false;
	panic_showed = false; // Turn it off for future PANICs
	ROS_INFO("Panic mode OFF. Entering normal mode (already started)"); 
      }
    } 
    else 
    {
      if (first_start) 
      {
	first_start = false;
	ROS_INFO("The show has started, please have fun.");
      }
      if ((ros::Time::now() - last_remote_joy_time).toSec() > max_joy_time &&
        last_remote_joy_time.toSec() > last_joy_time.toSec()
      ) {
        currentAngularVelocity *= max_time_decay;
        currentLinearVelocity *= max_time_decay;
      } 
      sendCmdVel(currentLinearVelocity, currentAngularVelocity, vel_pub);
      
      if (publishSlow) {
        sendSlowCmd(slow_mode, slow_pub);
        publishSlow = false;
      }
      rate.sleep();
    }
    ros::spinOnce();
  }
  if (backPressed) {
    ROS_INFO("Back button pressed --> stopping Siar and exiting the program. ");
    // Before exiting --> stop Siar
    sendCmdVel(0.0, 0.0, vel_pub);
    if (stopExit) {
      ROS_INFO("Killing all ros nodes.");
      system("rosnode kill -a");
    }
  }
  
  // Before exiting --> stop Siar
  sendCmdVel(0.0, 0.0, vel_pub);
  
  return 0;
}


bool setAutomaticMode(int new_mode) 
{
  std_msgs::Int8 msg;
  msg.data = new_mode;
  mode_pub.publish(msg);
}
