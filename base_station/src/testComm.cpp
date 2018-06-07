
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <math.h>
#include <boost/lexical_cast.hpp>

#include "functions/functions.h"

#include <sstream>
#include <fstream>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <text file> ..." << std::endl;
    return -1;
  }
  
  ros::init(argc, argv, "testComm");
  ros::NodeHandle nh;
  
  string s(argv[1]);
  string topic("/alert_text");

  ros::Publisher text_pub = nh.advertise<std_msgs::String>(topic,2);

  string text = functions::loadStringFile(s);
  
  
  cout << "Publishing " << text << "\n to topic: " << topic << endl;
  
  std_msgs::String msg;
  msg.data = text;
  
  
  while (ros::ok()) { 
    text_pub.publish(msg);
    sleep (1);
    
  }
  
  return 0;
}