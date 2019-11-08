#include "siar_arm/Point2Fire.hpp"

namespace SIAR
{
	Point2Fire::Point2Fire(ros::NodeHandle &nh, ros::NodeHandle &pnh) // : siar_arm_{nh, pnh}
	{
		
		float kp_tilt,ki_tilt,kd_tilt;
		float kp_pan,ki_pan,kd_pan;
		
		pnh.param<std::string>("robot_name", robot_name_,"siar");
		pnh.param<float>("kp_pan", kp_pan,1.0);
		pnh.param<float>("ki_pan", ki_pan,0.0);
		pnh.param<float>("kd_pan", kd_pan,0.0);
		pnh.param<float>("kp_tilt", kp_tilt,1.0);
		pnh.param<float>("ki_tilt", ki_tilt,0.0);
		pnh.param<float>("kd_tilt", kd_tilt,0.0);
		pnh.param<float>("fire_offset_x", fire_offset_x_,0.0);
		pnh.param<float>("fire_offset_x", fire_offset_y_,0.0);
		
		pan_control_ = control_toolbox::Pid(kp_pan,ki_pan,kd_pan);
		tilt_control_ = control_toolbox::Pid(kp_tilt,ki_tilt,kd_tilt);
							
		fire_detec_sub_ = nh.subscribe( "/firedetections2D", 2,  &Point2Fire::FireDetectCallback, this);
		fire_cam_info_sub_ = nh.subscribe("/" + robot_name_ + "/thermal_camera/camera_info", 2,  &Point2Fire::CamInfoCallback, this);
		pointing_fire_pub_ = nh.advertise<std_msgs::Bool>("/" + robot_name_ + "/pointing_fire_pub", 2);
		arm_pan_pub_ = nh.advertise<std_msgs::Float32>("/" + robot_name_ +"/arm_pan", 2);
		arm_tilt_pub_ = nh.advertise<std_msgs::Float32>("/" + robot_name_ +"/arm_tilt", 2);			
		
		//~ siar_arm_.load_data(siar_arm_.mot_file, siar_arm_.ang_file);	
		
		
		ros::Rate r(ros::Duration(0.1));
		ROS_INFO("Preloop");
		
		while (ros::ok()) {
			ros::spinOnce();
			ControlLoop();
			r.sleep();
		}		
	}	

	void Point2Fire::FireDetectCallback(const fireawareness_ros::FireDetections2D::ConstPtr& msg)
	{
		if (!center_img_) {  last_command_time_ = ros::Time::now(); return;}
		
		float area_fire{0.0};
		
		bool fire_detected{false};
		
		for( auto& detection: msg->detections)
		{
			if( detection.moments[0] > area_fire)
			{
				last_fire_detected_ =  Point2D<float>(detection.u, detection.v) - *center_img_; 
				area_fire_detected_ = area_fire;
				fire_detected = true;
					ROS_INFO("Fire detected");

			}
		}
		if( !fire_detected )
		{
			last_command_time_ = ros::Time::now();
			last_fire_detected_ = {};
		}
	}

	void Point2Fire::CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
	{
		center_img_ = Point2D<float>(msg->K[2], msg->K[5]);
			ROS_INFO("Cam data received");

	}
	
	void Point2Fire::ControlLoop( void)
	{	
		if(last_fire_detected_)
		{			
			const auto& current_time  = ros::Time::now();
			
			//~ rad2motor(const double &angle, uint16_t &command, uint8_t motor);
			arm_pan_pub_.publish(ComputePID( pan_control_, pan_ref_ , last_fire_detected_->x,  fire_offset_x_, current_time - last_command_time_ ));
			arm_tilt_pub_.publish(ComputePID( tilt_control_, tilt_ref_ , last_fire_detected_->y,  fire_offset_y_, current_time - last_command_time_ ));					
			last_command_time_ = current_time;
			
				ROS_INFO("Publishing commands");
		}
	}
	
	std_msgs::Float32 Point2Fire::ComputePID( Pid& _pid, float& _ref, float _p,  float _offset, const ros::Duration& _t )
	{
		std_msgs::Float32 force;
		 _ref += _pid.computeCommand( _p - _offset, _t);
		 force.data =_ref ;
		return force;
	}
	
}
	
	
	

