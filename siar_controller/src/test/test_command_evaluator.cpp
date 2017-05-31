#include "siar_controller/command_evaluator.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/serialization.h>
#include <functions/functions.h>


using namespace siar_controller;
using namespace std;

bool loadAltMap(nav_msgs::OccupancyGrid& map, const char *filename);

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_trajectory_evaluator");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("footprint_marker", 10, true);
  
  double cellsize = 0.02;
  
  RobotCharacteristics rob;
  rob.a_max = 1.0; 
  rob.a_max_theta = 1.0;
  rob.v_max = 0.4;
  rob.theta_dot_max = 0.7;
  
  
  geometry_msgs::Twist vel, vel_ini;
  

  vel.linear.y = vel.linear.z = 0.0;
  vel_ini.linear.x = vel_ini.linear.y = vel_ini.linear.z = 0.0;
  vel.angular.x = vel.angular.y = 0.0;
  vel_ini.angular.x = vel_ini.angular.y = vel_ini.angular.z = 0.0;
  
  
  if (argc > 1) {
    vel.linear.x = atof(argv[1]);
  } else {
    vel.linear.x = 0.4;
  }
  if (argc > 2) {
    vel.angular.z = atof(argv[2]);
  } else
    vel.angular.z = 0.0;
  if (argc > 3) {
    vel_ini.linear.x = atof(argv[3]);
  }
  if (argc > 4) {
    vel_ini.angular.z = atof(argv[4]);
  }
  
  ROS_INFO("Vel_x = %f \t Vel_th = %f", vel.linear.x, vel.angular.z);
  
  double T, delta_T;
  
  if (argc > 5) {
    T = atof(argv[5]);
  } else
    T = 2.0;
  
  ROS_INFO("T = %f", T);
  
  CommandEvaluator eval(1.0, 1.0, T, rob);
  
  nav_msgs::OccupancyGrid oc;
  oc.header.frame_id = "base_link";
  oc.header.seq = 0;
  oc.header.stamp = ros::Time::now();
  oc.info.height = 4 / cellsize;
  oc.info.width = 3 / cellsize;
  oc.info.resolution = cellsize;
  oc.info.origin.position.x =  oc.info.origin.position.y = oc.info.origin.position.z = 0.0;
  oc.info.origin.orientation.x =  oc.info.origin.position.y = oc.info.origin.position.z = 0.0;
  oc.info.origin.orientation.w = 1.0;
  for (int i = 0; i < oc.info.height;i++) {
    for (int j = 0; j < oc.info.width;j++) {
      oc.data.push_back(0);
    }
  }
  
  if (argc > 6) {
    ROS_INFO("Loading map: %s", argv[6]);
    loadAltMap(oc, argv[6]);
  }
  
  if (argc > 7) {
    // Saving grid
    ostringstream os;
    int cont = 0;
    for (int i = 0; i < oc.info.height; i++) {
      for (int j = 0; j < oc.info.width; j++, cont++) {
        os << (int)oc.data.at(cont) << " ";
        
      }
      os << endl;
    }
    ROS_INFO("Exporting the map to: %s", argv[7]);
    functions::writeStringToFile(argv[7], os.str());
  }
  
  
  
  visualization_msgs::Marker m;
  cout << "Evaluating trajectory. Result = " << eval.evualateTrajectory(vel_ini, vel, vel, oc, m);
  cout << endl;
  
  ROS_INFO("Publishing marker. N_points = %d", (int)m.points.size());
  marker_pub.publish(m);
  
  while (ros::ok()) {
  
    ros::spinOnce();
    sleep(1);
  }
 
  return 0;
}

bool loadAltMap(nav_msgs::OccupancyGrid& map, const char *filename)
{
  // TODO: implement this
  string f(filename);
  vector<double> v = functions::getVectorFromFile(f);
  
  ROS_INFO("Got %lu values in the map file", v.size());
  
  for (unsigned int i = 0; i < v.size() && i < map.data.size(); i++) {
    map.data[i] = (int8_t)v[i];
    
    cout << (int)map.data[i] <<" ";
  }
  cout << endl;
}

