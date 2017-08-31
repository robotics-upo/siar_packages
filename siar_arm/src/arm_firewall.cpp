#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "siar_arm.h"
#include <queue>


using namespace std;

SiarArmControl arm1;
queue <siar_driver::SiarArmCommand> v_commands;

siar_driver::SiarStatus robot_status;



void ReadServosCallback(const siar_driver::SiarStatus::ConstPtr& arm_pose)
{

 	for(int i=0; i<5; i++)
	{
		 robot_status.herculex_position[i] = arm_pose->herculex_position[i]; 
		 robot_status.herculex_temperature[i] = arm_pose->herculex_temperature[i]; 
		 robot_status.herculex_status[i] = arm_pose->herculex_status[i]; 
		 robot_status.herculex_torque[i] = arm_pose->herculex_torque[i];  
	}
	
}





void CommandCallback(const siar_driver::SiarArmCommand::ConstPtr& arm_command)
{
	
	siar_driver::SiarArmCommand command;
	
	// Nested ifs to be sure that the command is valid and reachable
	if(arm_command->joint_values[0]<2000 && arm_command->joint_values[0]>0)
	if(arm_command->joint_values[1]<2000 && arm_command->joint_values[1]>0)
	if(arm_command->joint_values[2]<2000 && arm_command->joint_values[2]>0)
	if(arm_command->joint_values[3]<2000 && arm_command->joint_values[3]>0)
	if(arm_command->joint_values[4]<2000 && arm_command->joint_values[4]>0)
	{
		for(int i=0; i<5; i++)
			command.joint_values[i] = arm_command->joint_values[i];
		command.command_time = arm_command->command_time;
		command.header = arm_command->header;
		v_commands.push(command);
	}	

}




int main(int argc, char **argv)
{

	ros::init(argc, argv, "arm_firewall");

	ros::NodeHandle n;

	siar_driver::SiarArmCommand command;

	int next_value = 1;

	ros::Subscriber arm_pos_sub = n.subscribe("/mensaje_david_status", 1000, ReadServosCallback);

	ros::Subscriber command_sub = n.subscribe("/mensaje_david_comando", 1000, CommandCallback);



	ros::Publisher arm_command_pub = n.advertise<siar_driver::SiarArmCommand>("/mensaje_al_brazo_control_brazo", 1000);

	while (ros::ok())
	{    
    	ros::spinOnce();
    	
    	
    	
    	if(!v_commands.empty())
    	{
			command = v_commands.front();
    	
			next_value = 1;
			
			for(int i = 0; i<5; i++)
			{
				if (robot_status.herculex_temperature[i]<0 && robot_status.herculex_temperature[i]>50)
				{
					ROS_ERROR("TEMPERATURE OF THE %d LINK IS OUT OF RANGE: %d", i, robot_status.herculex_temperature[i]);
					next_value = 0;
				}
				if (robot_status.herculex_status[i]!=1)
				{
					ROS_ERROR("%d LINK STATUS: %d", i, robot_status.herculex_status[i]);
					next_value = 0;
				}
				if ((robot_status.herculex_position[i] - command.joint_values[i])*(robot_status.herculex_position[i] - command.joint_values[i])>17)
				{
					next_value = 0;
				}											
			}	
				
			
			
			
			if( next_value == 1) 
			{
							
				arm_command_pub.publish(command);
				v_commands.pop();
				next_value = 0;
			}
		}
    	
    	
	}
    return 0;

}













