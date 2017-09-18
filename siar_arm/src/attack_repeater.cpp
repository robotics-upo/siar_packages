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

void  Move2MarkerCallback(const aruco_msgs::MarkerArray::ConstPtr& markers_read){
		
/*
	geometry_msgs::PoseStamped goal_pose;
	goal_pose.pose.position = arm_pose_goal->pose.position;
	goal_pose.pose.orientation = arm_pose_goal->pose.orientation;
	arm1.movelArm2Pose(goal_pose,5);
*/


	geometry_msgs::Point goal_point;
	for( int i=0; i<markers_read->markers.size();i++) 
	  goal_point = markers_read->markers[i].pose.pose.position;
	 
	arm1.movelArm2Point(goal_point,0);	
	
	ROS_INFO("goal_point: %f %f %f", goal_point.x, goal_point.y, goal_point.z);
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "attack_repeater");

	ros::NodeHandle n;

	siar_driver::SiarArmCommand servo_command;



	ros::Subscriber images_right_sub = n.subscribe("/aruco_marker_publisher_back_right/markers", 2, Move2MarkerCallback);

	ros::Subscriber images_left_sub = n.subscribe("/aruco_marker_publisher_back_left/markers", 2, Move2MarkerCallback);

	ros::Subscriber arm_pos_sub = n.subscribe("/siar_status", 2, ReadServosCallback);


	

	arm1.arm_pos_pub = n.advertise<siar_driver::SiarArmCommand>("/arm_cmd", 2);
	
	arm1.moveArmHL(0);
	ros::Rate r(10);

	while (ros::ok())
	{    
    	   ros::spinOnce();
	   r.sleep();
	}
    return 0;

}





