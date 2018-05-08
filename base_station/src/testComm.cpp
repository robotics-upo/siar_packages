
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <boost/lexical_cast.hpp>

#include <sstream>
#include <fstream>
#include "functions/FormattedTime.h"
#include "functions/functions.h"

#include "Comms.h"

#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <uav1> <uav2> ..." << std::endl;
    return -1;
  }
  
  ros::init(argc, argv, "testComm");
  Comms comms(argc, argv);
  uint n_uavs;
  std::vector<uint> uavs;
   
  for (int i = 1; i < argc; i++) {
    std::istringstream iss(argv[i]);
    iss >> n_uavs;
    uavs.push_back(n_uavs);
  }
  n_uavs = uavs.size();
  
  comms.setTopics("ual_", "quad_state_estimation", "v_pref", "cmd_vel", "state", "emergency_stop");
  
  cout << "Starting Communication with UAVs: " << functions::printVector(uavs) << endl;
  comms.startComms(uavs);
  
//   std::string s;
//   std::cin >> s;
  while (ros::ok()) { sleep (1);}
  
  comms.shutdownComms();
  
  return 0;
}