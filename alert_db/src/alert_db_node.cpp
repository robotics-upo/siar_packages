#include "alert_db/alert.hpp"
#include "alert_db.hpp"
#include <iostream>
#include <ros/ros.h>


using namespace std;
using namespace alert_db;

int main(int argc, char **argv) 
{
  // Alert DB instance
  std::string node_name = "alert_db_node";      
  ros::init(argc, argv, node_name);
  
  ros::NodeHandle nh;
  ros::NodeHandle lnh("~");
  AlertDB db(nh, lnh);
  
  
  
  cout << "In Alert DB node\n";
  
  ros::spin();
  
  db.generateReports();
  
  return 0;
}