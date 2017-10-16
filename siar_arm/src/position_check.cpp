#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "siar_arm/siar_arm.hpp"

#include <aruco_msgs/MarkerArray.h>

SiarArmControl arm1;

void ReadServosCallback(const siar_driver::SiarStatus::ConstPtr& arm_pose)
{
	for(int i=0; i<5; i++)
		 arm1.read_values[i] = arm_pose->herculex_position[i]; 
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "position_check");

	ros::NodeHandle n;

	siar_driver::SiarArmCommand servo_command;


	ros::Subscriber arm_pos_sub = n.subscribe("/siar_status", 2, ReadServosCallback);

	arm1.arm_pos_pub = n.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 2);
	
	ros::Rate r(10);
	
	arm1.write_values[0] = 512;
	arm1.write_values[1] = 1008;
	arm1.write_values[2] = 1294;
	arm1.write_values[3] = 437;
	arm1.write_values[4] = 792;

	while (ros::ok())
	{    
    	   ros::spinOnce();
	   
	   arm1.writeServos();
	   usleep(3000);
	   
	   arm1.write_values[0] = 650;
	   arm1.writeServos();
           usleep(3000);

           arm1.write_values[0] = 350;
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[0] = 512;
           arm1.writeServos();
           usleep(3000);

	   arm1.write_values[1] = 1200;     
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[1] = 700;
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[1] = 1008;
           arm1.writeServos();
           usleep(3000);

	   arm1.write_values[2] = 1500;     
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[2] = 1000;
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[2] = 1294;
           arm1.writeServos();
           usleep(3000);
		
	   arm1.write_values[3] = 600;     
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[3] = 250;
           arm1.writeServos();
           usleep(3000);

           arm1.write_values[3] = 437;
           arm1.writeServos();
           usleep(3000);
	   
	   r.sleep();
	}
    return 0;

}





