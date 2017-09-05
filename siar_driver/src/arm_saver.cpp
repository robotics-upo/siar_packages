/***********************************************************************/
/**                                                                    */
/** arm_saver.cpp                                                  */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Gonzalo Mier                                                       */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <siar_driver/SiarStatus.h>
#include <stdlib.h>
#include <functions/functions.h>

///////////////////////////////////////////////////
// This program saves a trajectory. Press START Button to save each position
// Then break the program (CTRL + C) to write it to a MATLAB file
// 

#define FREQ                20 
#define START_BUTTON          9

int startButton;
bool startPressed = false;

void joyReceived(const sensor_msgs::Joy::ConstPtr& joy) {
  startPressed = joy->buttons[startButton] == 1;
}

boost::array <int16_t, 5> curr_pos;
bool pos_received = false;

void herculexPosCb(const siar_driver::SiarStatus::ConstPtr& msg) {
  curr_pos = msg->herculex_position;
  pos_received = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SiarTeleopJoy");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  std::vector <std::vector<int16_t> > traj;
  
  if (argc < 1) {
    std::cerr << "Usage: " << argv[0] << " <filename> [wait_time (cents of a second) default 200]\n";
    return -1;
  }
  int wait_time = 200;
  if (argc > 1) 
    wait_time = atoi(argv[2]);
    
  
  double freq;
  pn.param<double>("freq",freq,FREQ);
  pn.param<int>("start_button", startButton, START_BUTTON);
  
  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 5, joyReceived);
  ros::Subscriber status_sub = n.subscribe<siar_driver::SiarStatus>("/siar_status", 1, herculexPosCb);
  ros::Rate rate(freq);
  
  std::vector<int16_t> v;
  v.resize(6);
  v[5] = wait_time;
  
  while (n.ok()) {
    ros::spinOnce();
    if (startPressed && pos_received) {
      for (int i = 0; i < 5; i++)
        v[i] = curr_pos[i];
        
      traj.push_back(v);
    }
    rate.sleep();
  }
  
  std::ostringstream os;
  
  for (size_t i = 0; i < traj.size(); i++) {
    for (int j = 0; j < 6; j++) {
      os << traj[i][j] << " ";
      
    }
    os << "\n";
  }
  
  functions::writeStringToFile(std::string(argv[1]), os.str());
  
  return 0;
}
