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
#include <siar_driver/SiarLightCommand.h>
#include <stdlib.h>
#include <siar_arm/armServosMoveAction.h>
#include <actionlib/client/simple_action_client.h>

///////////////////////////////////////////////////
// Default values for buttons and other values   //
// These values can be modified via rosparam     //
// NOTE: Axes inputs range [0,1] in ROS          //
// NOTE: button inputs are 0 or 1 (pressed) (int)//
// It has been configured to control with the   ///
// Logitech wireless pad controller              //

#define STOP_EXIT	      0
#define FREQ                100 
#define PANIC_FREQ          300

// axes
#define LINEAR_VELOCITY_AXIS  1
#define ANGULAR_VELOCITY_AXIS 0

// buttons
#define AUTO_BUTTON           3
#define SLOW_BUTTON   	      5
#define MAX_VELOCITY_BUTTON   10
#define FRONT_LIGHT_BUTTON    7
#define REAR_LIGHT_BUTTON     6
#define PANIC_BUTTON          2
#define START_BUTTON          9
#define BACK_BUTTON           8
#define REVERSE_BUTTON        0
#define MED_LIGHT_BUTTON      1
#define ARM_MODE_BUTTON	      4
// New buttons ARM and width
#define WIDTH_AXIS            4
#define WIDTH_AXIS_2          5
#define WHEEL_AXIS            2
#define WHEEL_AXIS_2          3
// --END NEW BUTTONS ---

#define MAX_LINEAR_VELOCITY   1.0
#define MAX_ANGULAR_VELOCITY  1

#define MIN_VEL_VARIATION     0.001
#define SLOW_MULTIPLIER       0.8

#define MAX_JOY_TIME          8.0
#define JOY_PRIORITY_TIME     5.0

#define MAX_AUTO_MODE         1
#define MAX_TIME_DECAY        0.98


typedef actionlib::SimpleActionClient<siar_arm::armServosMoveAction> MoveArmClient;

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
int front_light_button, rear_light_button;
int slowButton;
int auto_button;
int arm_mode_button;

int arm_pan_tilt_button;
int arm_nav_forwards_button;
int arm_nav_backwards_button;

// Arm and width buttons and axes
int width_pos_axis, width_pos_axis_2, wheel_pos_axis, wheel_pos_axis_2;
int arm_axis_pan, arm_axis_tilt;
int med_light_button, last_med_light_button;
uint8_t arm_torque = 0; // Current state of arm_torque

//////////////////////////////////

double currentLinearVelocity  = 0.0;
double currentAngularVelocity = 0.0;

double currWidth = 1.0;
bool last_change_width = false;

//////////////////////////////////
// Status flags

bool panic          = false;
bool startPressed   = false;
bool backPressed   = false;
bool backwards = false;
bool ant_reverse_but = false;
bool slow_mode = false;
bool last_slow = false;
bool publishSlow = false;
bool ant_auto_button = false;
bool ant_arm_button = false;
int auto_mode = 0;
int max_auto_mode;
bool front_light = false;
bool rear_light = false;
bool middle_light = false;
bool last_front_button = false;
bool last_rear_button = false;

bool arm_mode = false;


ros::Time last_joy_time, last_remote_joy_time;
ros::Publisher reverse_pub;
ros::Publisher slow_pub;
ros::Publisher mode_pub;
ros::Publisher width_pos_pub;
ros::Publisher light_cmd_pub;

MoveArmClient *move_arm_client;

// New publishers for arm stuff
ros::Publisher arm_pan_pub, arm_tilt_pub, arm_mode_pub;
ros::Publisher arm_torque_pub, arm_clear_pub;

bool setAutomaticMode(int new_mode);
void publishLight();

void interpretArm(const sensor_msgs::Joy::ConstPtr& joy);
void interpretLights(const sensor_msgs::Joy::ConstPtr& joy);

void interpretJoy(const sensor_msgs::Joy::ConstPtr& joy) {
  panic = panic | (joy->buttons[panicButton] == 1);
  if (!ant_arm_button && joy->buttons[arm_mode_button] == 1) {
    arm_mode = !arm_mode;
    std_msgs::Bool msg;
    msg.data = arm_mode?1:0;
    arm_mode_pub.publish(msg);
  }
  ant_arm_button = joy->buttons[arm_mode_button] == 1;
  if (arm_mode) {
    interpretArm(joy);
  } else {
    startPressed = joy->buttons[startButton] == 1;
    backPressed = joy->buttons[backButton] == 1;
    
    if (!ant_auto_button && joy->buttons[auto_button] == 1) {
      // Request for mode change
      if (auto_mode > 0) 
	auto_mode = 0;
      else
	auto_mode = 1;
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
      currentAngularVelocity =maxAngularVelocity * multiplier * joy->axes[angularVelocityAxis];
      currentLinearVelocity = maxLinearVelocity * multiplier * joy->axes[linearVelocityAxis];
      currentLinearVelocity *= backwards?-1.0:1.0; // Only the linear velocity should change when going backwards
      
      if (joy->buttons[slowButton] && !last_slow) {
	slow_mode = !slow_mode;
	publishSlow = true;
	
      }
      last_slow = joy->buttons[slowButton] == 1;
      
      // Width position
      double width_pos = joy->axes[width_pos_axis]; 
      double width_pos_2 = joy->axes[width_pos_axis_2];
  //     ROS_INFO("Width pos = %f\tWidth pos 2 = %f\tnorm_sq = %f", width_pos, width_pos_2,norm_sq);
      if (width_pos > 0.95 && fabs(width_pos_2) < 0.05) { // Maximum width
	std_msgs::Float32 msg;
	msg.data = 0.0;
	width_pos_pub.publish(msg);
	last_change_width = true;
	currWidth = 0.0;
      } else if (fabs(width_pos_2) > 0.95) {
	if (!last_change_width) {
	  std_msgs::Float32 msg;
	  if (width_pos_2 > 0.0) {
	    currWidth += 0.05;
	  } else {
	    currWidth -= 0.05;
	  }
	  currWidth = std::min(1.0, currWidth);
	  currWidth = std::max(-1.0, currWidth);
	  msg.data = currWidth;
	  width_pos_pub.publish(msg);
	  last_change_width = true;
	}
	
      } else {
	last_change_width = false;
      }
      
      
      
      double wheel_pos = joy->axes[wheel_pos_axis];
      double wheel_pos_2 = joy->axes[wheel_pos_axis_2];
      if (fabs(wheel_pos) > 0.95) {
	if (auto_mode > 0) {
	  if (currentLinearVelocity < 0.0)
	    wheel_pos *= -1.0; 
	  auto_mode = (wheel_pos > 0 )?3:2;
	}
	setAutomaticMode(auto_mode);
      } else if (wheel_pos_2 > 0.95) {
	if (auto_mode > 0) {
	  auto_mode = 4;
	}
	setAutomaticMode(auto_mode);
      } else if (wheel_pos_2 < -0.95) {
	if (auto_mode > 0) {
	  auto_mode = 5;
	}
	setAutomaticMode(auto_mode);
      } else {
	// Return to normal operation mode
	if (auto_mode > 0) {
	  auto_mode = 1;
	}
	setAutomaticMode(auto_mode);
      }
      
      interpretLights(joy);
      
    }
  }
}

void interpretArm(const sensor_msgs::Joy::ConstPtr& joy)
{
  // TODO: Complete this --> in principle it would just make one of the axis of the joystick to the pan&tilt.
  // maybe then we can add a target point and move it arround
  if (!panic) {
    std_msgs::Float32 msg;

    msg.data = joy->axes[arm_axis_pan];
    arm_pan_pub.publish(msg);
    msg.data = joy->axes[arm_axis_tilt];
    arm_tilt_pub.publish(msg);
    
    double lateral = joy->axes[width_pos_axis]; 
    double vertical = joy->axes[width_pos_axis_2];
    siar_arm::armServosMoveGoal goal;
    if (lateral > 0.95) {
      goal.mov_name = "left";
      move_arm_client->sendGoal(goal);
      ROS_INFO("Sending left arm command.");
    } else if (lateral < -0.95) {
      goal.mov_name = "right";
      move_arm_client->sendGoal(goal);
      ROS_INFO("Sending right arm command.");
    }
    if (vertical > 0.95) {
      goal.mov_name = "front";
      move_arm_client->sendGoal(goal);
    } else if (vertical < -0.95) {
      goal.mov_name = "back";
      move_arm_client->sendGoal(goal);
      ROS_INFO("Sending back arm command.");
    }
    
    if (joy->buttons[auto_button] == 1) {
      goal.mov_name = "park";
      move_arm_client->sendGoal(goal);
      ROS_INFO("Sending park arm command.");
    }
    
    if (joy->buttons[reverseButton] == 1) {
      goal.mov_name = "pan_tilt";
      move_arm_client->sendGoal(goal);
      ROS_INFO("Sending pan and tilt arm command.");
    }
    
    if (joy->buttons[slowButton] == 1) {
      goal.mov_name = "navigation";
      move_arm_client->sendGoal(goal);
      ROS_INFO("Sending navigation command.");
    }
    if (joy->buttons[startButton] == 1) {
      std_msgs::UInt8 msg;
      msg.data = arm_torque++;
      if (arm_torque > 2) {
	arm_torque = 0;
      }
      arm_torque_pub.publish(msg);
      usleep(1000);
      std_msgs::Bool msg2;
      msg2.data = 0;
      arm_clear_pub.publish(msg2);
    }
    
    // The lights go equal
    interpretLights(joy);
  } else {
    std_msgs::Float32 msg;
    msg.data = 0.0;
    arm_tilt_pub.publish(msg);
    arm_pan_pub.publish(msg);
    
  }
  
  
}

void interpretLights(const sensor_msgs::Joy::ConstPtr& joy) {
  // Light buttons
  if (joy->buttons[front_light_button] == 1 && !last_front_button) {
    front_light = !front_light;
    publishLight();
    
  }
  last_front_button = joy->buttons[front_light_button];
  if (joy->buttons[rear_light_button] == 1 && !last_rear_button) {
    rear_light = !rear_light;
    publishLight();
    
  }
  last_rear_button = joy->buttons[rear_light_button];
  if (joy->buttons[med_light_button] == 1 && last_med_light_button == 0) {
    middle_light = !middle_light;
    publishLight();
  }
  last_med_light_button = joy->buttons[med_light_button];
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
 

  if (auto_mode == 0.0 || panic ){
    geometry_msgs::Twist vel;
    vel.angular.z = angularVelocity;
    vel.linear.x = linearVelocity;
    vel.linear.z = 0.0;
    vel.linear.y = 0.0;
    vel_pub.publish(vel);

  }
}

void sendSlowCmd(bool slow_mode, ros::Publisher &pub) {
  std_msgs::Bool msg;
  msg.data = slow_mode;
  slow_pub.publish(msg);
}

void publishLight()
{
  siar_driver::SiarLightCommand msg;
  msg.front = front_light;
  msg.rear = rear_light;
  msg.middle = middle_light;
  light_cmd_pub.publish(msg);
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
  
  pn.param<int>("arm_pan_axis", arm_axis_pan, angularVelocityAxis);
  pn.param<int>("arm_tilt_axis", arm_axis_tilt, linearVelocityAxis);
  
  pn.param<int>("auto_button", auto_button, AUTO_BUTTON);
  pn.param<int>("slow_button", slowButton, SLOW_BUTTON);
  pn.param<int>("turbo_button", maxVelocityButton, MAX_VELOCITY_BUTTON);
  
  pn.param<int>("width_pos_axis", width_pos_axis, WIDTH_AXIS);
  pn.param<int>("width_pos_axis_2", width_pos_axis_2, WIDTH_AXIS_2);
  pn.param<int>("med_light_button", med_light_button, MED_LIGHT_BUTTON);
  pn.param<int>("wheel_pos_axis", wheel_pos_axis, WHEEL_AXIS);
  pn.param<int>("wheel_pos_axis_2", wheel_pos_axis_2, WHEEL_AXIS_2);
  pn.param<int>("front_light_button", front_light_button, FRONT_LIGHT_BUTTON);
  pn.param<int>("rear_light_button", rear_light_button, REAR_LIGHT_BUTTON);
  pn.param<int>("arm_mode_button", arm_mode_button, ARM_MODE_BUTTON);
  
  pn.param<int>("arm_pan_tilt_button", arm_pan_tilt_button, auto_button);
  pn.param<int>("arm_nav_forwards_button", arm_nav_forwards_button, slowButton);
  pn.param<int>("arm_nav_backwards_button", arm_nav_backwards_button, arm_nav_backwards_button);
  
  pn.param<double>("max_linear_velocity",maxLinearVelocity,MAX_LINEAR_VELOCITY);
  pn.param<double>("max_angular_velocity",maxAngularVelocity,MAX_ANGULAR_VELOCITY);

  double max_joy_time, joy_priority_time;
  pn.param<double>("max_joy_time", max_joy_time, MAX_JOY_TIME);
  pn.param<double>("joy_priority_time", joy_priority_time, JOY_PRIORITY_TIME);
  
  
  pn.param<double>("slow_multipier", slow_multiplier, SLOW_MULTIPLIER);
  
  pn.param<int>("max_auto_mode", max_auto_mode, MAX_AUTO_MODE);
  double max_time_decay;
  pn.param<double>("max_time_decay", max_time_decay, MAX_TIME_DECAY);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  reverse_pub = n.advertise<std_msgs::Bool>("reverse", 1);
  slow_pub = n.advertise<std_msgs::Bool>("slow_motion", 1);
  mode_pub = n.advertise<std_msgs::Int8>("operation_mode", 1 );
  
  arm_pan_pub = n.advertise<std_msgs::Float32>("arm_pan", 1); // Publishers for sending velocity commands to pan&tilt
  arm_tilt_pub = n.advertise<std_msgs::Float32>("arm_tilt", 1);
  
  arm_torque_pub = n.advertise<std_msgs::UInt8>("arm_torque", 1); // Publishers for low level arm control
  arm_clear_pub = n.advertise<std_msgs::Bool>("arm_clear_status", 1);
  
  arm_mode_pub = n.advertise<std_msgs::Bool>("arm_mode", 1);
  
  move_arm_client = new MoveArmClient("move_arm");
  
  // Width, costmap, light
  width_pos_pub = n.advertise<std_msgs::Float32>("width_pos", 1);
  light_cmd_pub = n.advertise<siar_driver::SiarLightCommand>("light_cmd", 1);
  
  // ---------END WIDTH ARM ---
  
  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 5, joyReceived);
  ros::Subscriber remote_joy_sub = n.subscribe<sensor_msgs::Joy>("remote_joy", 5, remoteJoyReceived);

  ros::Rate rate(freq);
  ros::Rate panicRate(panicFreq);
  bool panic_showed = false;
  bool start = false;
  bool first_start = true;
  ROS_INFO("Siar teleop node. Press START to have fun. Press BACK at any moment to exit.");
  while (n.ok()) 
  {
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
        last_remote_joy_time.toSec() > last_joy_time.toSec() )
      {
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
    
    if (backPressed) {
      ROS_INFO("Back button pressed --> stopping Siar and raposa bag. ");
      // Before exiting --> stop Siar
      // sendCmdVel(0.0, 0.0, vel_pub);
      
      int ret_val;
      
      ret_val = system ("rosnode kill /rosbag_raposa");
      
      int cont = 0;
      while (backPressed && cont < 5) {
        sendCmdVel(0.0, 0.0, vel_pub);
        // ros::spinOnce();
        sleep(1);
        cont++;
      }
      
      if (cont >= 5) {
        ROS_INFO("Killing all ros nodes and shutting down.");
        ret_val = system("rosnode kill -a");
        ret_val = system("shutdown now");
      } else {
        
        pid_t pid = fork();
        
        if (pid == 0) 
        {
          //Client
          ret_val = system("roslaunch siar_driver bag.launch");
          return 0;
        }
        backPressed = false;
      }
    }
    ros::spinOnce();

  }
  
  
  // Before exiting --> stop Siar
  sendCmdVel(0.0, 0.0, vel_pub);
  delete move_arm_client;
  
  return 0;
}


bool setAutomaticMode(int new_mode) 
{
  std_msgs::Int8 msg;
  msg.data = new_mode;
  mode_pub.publish(msg);
}
