#include <ros/ros.h>
#include <siar_driver/SiarArmCommand.h>
#include <functions/functions.h>
#include <std_msgs/UInt8.h>

using namespace siar_driver;
using namespace functions;
using namespace std;

int main(int argc, char **argv) {
  vector <SiarArmCommand> v;
  SiarArmCommand com;
  bool reverse = false;
  if (argc > 6) {
    cout << "Making one command test.\n";
    // The argument has to be interpreted as the values of the command
    for (int i = 0; i < 5; i++) {
      com.joint_values[i] = atoi(argv[i + 1]);
    }
    com.command_time = atoi(argv[6]);
    v.push_back(com);
  } else if (argc > 1) {
    reverse = argc > 2;
    // The first argument will be the file where the commands are located
    vector<vector<double> > mat;
     
    if (!reverse) {
      cout << "Making regular file test\n";
    } else {
      cout << "Making REVERSE file test\n";
    }
    
    if (getMatrixFromFile(argv[1], mat)) {
      for (size_t i = 0; i < mat.size(); i++) {
        if (mat[i].size() < 6)
          continue;
        for (int j = 0; j < 5; j++) {
          com.joint_values[j] = mat[i][j];
        }
        com.command_time = mat[i][5];
        
        v.push_back(com);
      }
    } else {
      cerr << "Arm Tester: could not open the file: " <<  argv[1] << "\n";
    }
    cout << "Got " << v.size() << "commands\n";
  } else {
    cerr << "Usage: " << argv[0] << " <file_name>\n";
    cerr << "Or: " << argv[0] << " <j0> <j1> <j2> <j3> <j4> <j5> <command_time>\n";
  }
  
  // ROS stuff
  ros::init(argc, argv, "arm_tester");
  ros::NodeHandle n;
  ros::Publisher arm_pub = n.advertise<SiarArmCommand>("/arm_cmd", 1);
  ros::Publisher arm_torque_pub = n.advertise<std_msgs::UInt8>("/arm_torque", 1);
  
  sleep(1);
  
  // First activate the motors
  std_msgs::UInt8 msg_torque;
  msg_torque.data = 2; // Activate
  arm_torque_pub.publish(msg_torque);
  
  sleep(1);
  
  size_t a;
  for (size_t i = 0; i < v.size();i++) {
    a = i;
    if (reverse) 
      a = v.size() - i - 1;
    cout << "Publishing arm command number: "<< i << endl;
    cout << "Command: " ;
    
    for (int j = 0; j < 5; j++) {
      cout << v[a].joint_values[j] << " ";
    }
    cout << "\t Time: " << (int)v[a].command_time << endl;
    arm_pub.publish(v[a]);
    usleep ( v[a].command_time * 10000);
  }
  msg_torque.data = 1; // After execution --> brake the motors
  arm_torque_pub.publish(msg_torque);
  
  return 0;
}
