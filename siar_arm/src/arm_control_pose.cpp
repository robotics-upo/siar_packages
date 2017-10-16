#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "siar_arm/siar_arm.hpp"




SiarArmControl arm1;


void ReadServosCallback(const siar_driver::SiarStatus::ConstPtr& arm_pose)
{
	for(int i=0; i<5; i++)
		 arm1.read_values[i] = arm_pose->herculex_position[i]; 
}

void GoalPosCallback(const geometry_msgs::PoseStamped::ConstPtr& arm_pose_goal){
		
	geometry_msgs::PoseStamped goal_pose;
	goal_pose.pose.position = arm_pose_goal->pose.position;
	goal_pose.pose.orientation = arm_pose_goal->pose.orientation;
	 
	arm1.movelArm2Pose(goal_pose,5);

}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "arm_control_pose");

	ros::NodeHandle n;

	siar_driver::SiarArmCommand servo_command;



	ros::Subscriber arm_pos_sub = n.subscribe("/mensaje_david_status", 1000, ReadServosCallback);

	ros::Subscriber goal_pos_sub = n.subscribe("/posicion_requerida", 1000, GoalPosCallback);



	arm1.arm_pos_pub = n.advertise<siar_driver::SiarArmCommand>("/mensaje_david_comando", 1000);

	while (ros::ok())
	{    
    	ros::spinOnce();
	}
    return 0;

}
























