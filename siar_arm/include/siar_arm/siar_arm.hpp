#ifndef _SIAR_ARM_HPP_
#define _SIAR_ARM_HPP_


#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "iostream"
#include <functions/functions.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <queue>
#include <aruco_msgs/MarkerArray.h>


//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/server/simple_action_server.h>
//#include "siar_arm/armServosMoveAction.h"



#define PI 3.14159265
#define L1 0.0475
#define L2 0.215
#define L3 0.155
#define L4 0.080

//typedef actionlib::SimpleActionClient<siar_arm::armServosMoveAction> Client;
//typedef actionlib::SimpleActionServer<siar_arm::armServosMoveAction> Server;

using siar_driver::SiarArmCommand;
using namespace siar_driver;
using namespace functions;
using namespace std;

class SiarArmControl{
	
	private:

		int inverse_function_values[5];

		
		void writeServos()
		{	
			int command_time = 0;
			
			for(int i=0; i<5; i++)
			{
				servo_command.joint_values[i] = write_values[i];
				command_time = max( command_time, abs(write_values[i] - read_values[i]));
			}
			servo_command.command_time = command_time * 2;
					
		//	cout << servo_command.joint_values[0] << ", "<< servo_command.joint_values[1] << ", "<< servo_command.joint_values[2] << ", "<< servo_command.joint_values[3] << ", "<< servo_command.joint_values[4] << endl;			
				
			
			arm_pos_pub.publish(servo_command);
			
			
			
			//geometry_msgs::PoseStamped A = forwardKinematics(write_values);
			//geometry_msgs::Point B = A.pose.position;
			//ROS_INFO("Computed: %f %f %f", B.x, B.y, B.z);
			usleep ( servo_command.command_time * 10000);

			

		}
		
	
	public:

		siar_driver::SiarArmCommand servo_command;
		
		ros::Publisher arm_pos_pub;

		int read_values[5];
		int write_values[5];
	
		
		
		
		void inverseKinematics(const geometry_msgs::PoseStamped& pose)
		{
			double x = pose.pose.position.x;
			double y = pose.pose.position.y;
			double z = pose.pose.position.z;
			double a = pose.pose.orientation.x;
			double b = pose.pose.orientation.y;
			double c = pose.pose.orientation.z;
			
						
			int coordenadas_correctas = 1;

			double beta, beta_a, beta_p, beta_pp, gamma, delta, delta_a, delta_p;
			double x_p;
			double L_a, L;
			double q1, q2, q3, q4, q5;
			
			int result[5];


			x_p = sqrt( pow(x,2) + pow(y,2)  )- L1;
			L = sqrt( pow(x_p,2) + pow(z,2) );
			L_a = sqrt(pow((z - L4*sin(b)),2) + pow((x_p - L4*cos(b)),2));
			beta_p = atan2(z - L4*sin(b), x_p - L4*cos(b));
			beta_pp= atan2(z, x_p);
			beta_a = acos((pow(L2,2)+pow(L_a,2)-pow(L3,2))/(2*L2*L_a));
			beta =  beta_p + beta_a;
			gamma = acos((pow(L2,2)+pow(L3,2)-pow(L_a,2))/(2*L2*L3));
			delta_a = PI - (beta_a + gamma);
			delta_p  = acos((pow(L_a,2)+pow(L4,2)-pow(L,2))/(2*L_a*L4));
			if (beta_p>=beta_pp)
				delta = delta_p + delta_a;
			else
			{
				delta = 2*PI - (delta_p - delta_a);
				if (isnan(delta)) {
					delta = PI + delta_a;
				}
			}

			if (isnan(gamma))
			{
				coordenadas_correctas = 0; 
			}
			
			
			q1 = atan2(y,x);
			q2 = beta;
			q3 = gamma - PI;
			q4 = (delta - PI);
			q5 = a;
			
			//ROS_INFO("x: %f; y: %f;z: %f",x,y,z);
			//ROS_INFO("q1: %f, q2: %f, q3: %f, q4: %f, q5:%f",q1,q2,q3,q4,q5);
			

			result[0] = (q1*2.0/PI*(228.0-805.0)/2.0+512.0);
			result[1] = (q2*2.0/PI*(452.0-1580.0)/2.0+1008.0);
			result[2] = (q3*2.0/PI*(1855.0-743.0)/2.0+1294.0);
			result[3] = (q4*2.0/PI*(156.0-722.0)/2.0+437.0);
			result[4] = (q5*2.0/PI*(242.0-792.0)/2.0+792.0);
						
			
			
			for( int i=0; i<5; i++)
				inverse_function_values[i] = result[i];
			
			
		}

		geometry_msgs::PoseStamped forwardKinematics(const int* joint_values )
		{
	
			geometry_msgs::PoseStamped pose;

			float q1, q2, q3, q4, q5;
			
			q1 = (joint_values[0]-512.0)/((228.0-805.0)/2.0)*PI/2.0;
			q2 = (joint_values[1]-1008.0)/((452.0-1580.0)/2.0)*PI/2.0;
			q3 = (joint_values[2]-1294.0)/((1855.0-743.0)/2.0)*PI/2.0;
			q4 = (joint_values[3]-437.0)/((156.0-722.0)/2.0)*PI/2.0;
			q5 = (joint_values[4]-792.0)/((242.0-792.0)/2.0)*PI/2.0;
						
			
			pose.pose.position.x = cos(q1) * (L1 + L2*cos(q2) + L3*cos(q2 + q3) + L4*cos(q2 + q3 + q4));
			pose.pose.position.y = sin(q1) * (L1 + L2*cos(q2) + L3*cos(q2 + q3) + L4*cos(q2 + q3 + q4));
			pose.pose.position.z = L2*sin(q2) + L3*sin(q2 + q3) + L4*sin(q2 + q3 + q4);
			pose.pose.orientation.x = q5;
			pose.pose.orientation.y = q2+q3+q4;
			pose.pose.orientation.z = q1;


			return pose;
			
		}

		void moveArm2Point(const geometry_msgs::Point& goal_point)
		{
			movelArm2Point(goal_point, 0);	
		}
		void movelArm2Point(const geometry_msgs::Point& goal_point, uint8_t n_points)
		{
			geometry_msgs::PoseStamped goal_pose;
			
			goal_pose.pose.position.x = goal_point.x;
			goal_pose.pose.position.y = goal_point.y;
			goal_pose.pose.position.z = goal_point.z;
			goal_pose.pose.orientation.x = 0;
			goal_pose.pose.orientation.y = 0;
			goal_pose.pose.orientation.z = 0;
				
			movelArm2Pose( goal_pose, n_points);
					
		}


		void moveArm2Pose(const geometry_msgs::PoseStamped& goal_pose)
		{
			movelArm2Pose( goal_pose, 0);
		}
		
		void movelArm2Pose(const geometry_msgs::PoseStamped& goal_pose, uint8_t n_points)
		{	
	
			int arm_position_actual[5];
			
			geometry_msgs::PoseStamped actual_pose, next_pose;
			
			
			for(int i=0; i<5; i++)
				arm_position_actual[i] = read_values[i];
						
			actual_pose = forwardKinematics(arm_position_actual);
			
			
			
			for( int i = 0; i < n_points+1; i++)	
			{	
				
				next_pose.pose.position.x = (goal_pose.pose.position.x - actual_pose.pose.position.x)*(i+1)/(n_points+1) + actual_pose.pose.position.x ;
				next_pose.pose.position.y = (goal_pose.pose.position.y - actual_pose.pose.position.y)*(i+1)/(n_points+1) + actual_pose.pose.position.y ;
				next_pose.pose.position.z = (goal_pose.pose.position.z - actual_pose.pose.position.z)*(i+1)/(n_points+1) + actual_pose.pose.position.z ;
				next_pose.pose.orientation.x = (goal_pose.pose.orientation.x - actual_pose.pose.orientation.x)*(i+1)/(n_points+1) + actual_pose.pose.orientation.x ;
				next_pose.pose.orientation.y = (goal_pose.pose.orientation.y - actual_pose.pose.orientation.y)*(i+1)/(n_points+1) + actual_pose.pose.orientation.y ;
				next_pose.pose.orientation.z = (goal_pose.pose.orientation.z - actual_pose.pose.orientation.z)*(i+1)/(n_points+1) + actual_pose.pose.orientation.z ;		
				
				
				
							
				inverseKinematics(next_pose);
				for(int j=0; j<5; j++)
				{
					write_values[j] = inverse_function_values[j];
				}
				writeServos();
						
			}

		}
		
		
		void moveArm(const int* joint_values )
		{
			for(int j=0; j<5; j++)
			{
				write_values[j] = joint_values[j];
			}
			writeServos();
		}

		
		// --------------- Preconfigured movements ------------------------- //
		
		
		// from HOME to BASE_POS = 0
		// from BASE_POS to HOME = 1
		// from DEPLOY{1,2,3,4,5} to BASE_POS = {2,3,4,5,6}
		// from BASE_POS to DEPLOY{1,2,3,4,5} = {7,8,9,10,11}
		// from PICK_UP to BASE_POS = 12
		// from BASE_POS to PICK_UP = 13
		
		void moveArmHL(const uint option)  // Move the arm from to a point to another by a high level function
		{
			SiarArmCommand com;
			vector<vector<double> > mat;
			
			string f_path;
			if (option < 12)
			{
				switch( option)
				{
					case 0: f_path = "/paths/home2base"; break;
					case 1: f_path = "/paths/base2home"; break;
					case 2: f_path = "/paths/base2deploy12ground"; break;
					case 3: f_path = "/paths/base2deploy22ground"; break;
					case 4: f_path = "/paths/base2deploy32ground"; break;
					case 5: f_path = "/paths/base2deploy42ground"; break;
					case 6: f_path = "/paths/base2deploy52ground"; break;
					case 7: f_path = "/paths/base2deploy12base"; break;
					case 8: f_path = "/paths/base2deploy22base"; break;
					case 9: f_path = "/paths/base2deploy32base"; break;
					case 10: f_path = "/paths/base2deploy42base"; break;
					case 11: f_path = "/paths/base2deploy52base"; break;
				}
				
							
				
				if (getMatrixFromFile(f_path, mat)) {
					for (size_t i = 0; i < mat.size(); i++) {
						for (int j = 0; j < 5; j++) {
							write_values[j] = mat[i][j];
							writeServos();
						}
					
					}
				} else {
				  cerr << "Arm Tester: could not open the file: " <<  f_path << "\n";
				}

							
			}
			if (option == 12)
			{
				
				geometry_msgs::PoseStamped base_pose;
				
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////				
				///////////////////////////////////////////////////
				//////      INDICAR CUAL ES LA POSE BASE //////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////
				///////////////////////////////////////////////////				
				///////////////////////////////////////////////////						
				
				movelArm2Pose( base_pose, 10);
				
				
			}
			
			if (option == 13)
			{
				
				
				
			}

			
		}

/*	void followMarker()
	{
	      
	      
	      btQuaternion q(target_state->target.transform.rotation.x, target_state->target.transform.rotation.y, target_state->target.transform.rotation.z, target_state->target.transform.rotation.w);
	      double roll, pitch, yaw;
	      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);  
	}
*/

		
		

};




#endif /* _SIAR_ARM_H_ */
