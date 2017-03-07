#include "sewer_graph/sewer_graph.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace sewer_graph;

int main(int argc, char ** argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <filename> [<output_kml_file>]\n";
    return -1;
  }
  string s(argv[1]);
  SewerGraph g(s);
  cout << g.toString_2() << endl; 
  
  if (argc > 2) {
    string kml_file(argv[2]);
    cout << "Exporting KML file to " << kml_file << endl;
    g.exportKMLFile(kml_file);
  }
  
  ros::init(argc, argv, "test_sewer");
  ros::NodeHandle nh;
  
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
  ROS_INFO("Publishing markers");
  while (ros::ok()) {
    std::vector<visualization_msgs::Marker> vec = g.getMarkers("/map");
    
    for (unsigned int i = 0; i < vec.size(); i++) {
      marker_pub.publish(vec.at(i));
    }
    ros::spinOnce();
    sleep(1);
  }
  
  
  return 0;
}