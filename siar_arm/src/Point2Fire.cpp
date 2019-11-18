#include "siar_arm/Point2Fire.hpp"

namespace SIAR
{
	Point2Fire::Point2Fire(ros::NodeHandle &nh, ros::NodeHandle &pnh) // : siar_arm_{nh, pnh}
	{
		
		float kp_tilt,ki_tilt,kd_tilt;
		float kp_pan,ki_pan,kd_pan;
		
		pnh.param<std::string>("mot_pan_arm_file", mot_pan_arm_file_, "");
		pnh.param<std::string>("mot_tilt_arm_file", mot_tilt_arm_file_, "");
		
		pnh.param<std::string>("robot_name", robot_name_,"siar");
		pnh.param<float>("kp_pan", kp_pan,0.6);
		pnh.param<float>("ki_pan", ki_pan,0.0);
		pnh.param<float>("kd_pan", kd_pan,0.0);
		pnh.param<float>("kp_tilt", kp_tilt,1.0);
		pnh.param<float>("ki_tilt", ki_tilt,0.0);
		pnh.param<float>("kd_tilt", kd_tilt,0.0);
		pnh.param<float>("fire_offset_x", fire_offset_x_,0.0);
		pnh.param<float>("fire_offset_x", fire_offset_y_,0.0);
		
		pan_control_ = control_toolbox::Pid(kp_pan,ki_pan,kd_pan);
		tilt_control_ = control_toolbox::Pid(kp_tilt,ki_tilt,kd_tilt);
		
		//~ load_data(mot_pan_arm_file_, mot_tilt_arm_file_) ;
		
							
		fire_detec_sub_ = nh.subscribe( "firedetections2D", 2,  &Point2Fire::FireDetectCallback, this);
		fire_detec_3D_sub_ = nh.subscribe( "firedetections3D", 2,  &Point2Fire::FireDetect3DCallback, this);
		fire_cam_info_sub_ = nh.subscribe("/" + robot_name_ + "/thermal_camera/camera_info", 2,  &Point2Fire::CamInfoCallback, this);
		pointing_fire_pub_ = nh.advertise<std_msgs::Bool>("/" + robot_name_ + "/pointing_fire_pub", 2);
		arm_pan_pub_ = nh.advertise<std_msgs::Float32>("/" + robot_name_ +"/arm_pan", 2);
		arm_tilt_pub_ = nh.advertise<std_msgs::Float32>("/" + robot_name_ +"/arm_tilt", 2);					
		fire_detected_pub_ = nh.advertise<std_msgs::Bool>("/" + robot_name_ +"/fire_detected", 2);					
	}	

	void Point2Fire::load_data(const std::string &mot_pan_arm_file, const std::string &mot_tilt_arm_file) 
	{
		std::ifstream infile_pan(mot_pan_arm_file_);
		infile_pan >> pan_min_;
		while( infile_pan >> pan_max_ ){};
		if( pan_min_ > pan_max_ )
		{
			std::swap(pan_min_, pan_max_);				
		} 
		pan_ref_ = pan_center_ = (pan_min_ + pan_max_)/2.0;

		std::ifstream infile_tilt(mot_tilt_arm_file_);
		infile_tilt >> tilt_min_;
		while( infile_tilt >> tilt_max_ ){};
		if( tilt_min_ > tilt_max_ )
		{
			std::swap(tilt_min_, tilt_max_);				
		} 
		tilt_ref_ = tilt_center_ = (tilt_min_ + tilt_max_)/2.0;
	}		

	void Point2Fire::FireDetectCallback(const fireawareness_ros::FireDetections2D::ConstPtr& msg)
	{
		if (!center_img_) {  last_command_time_ = ros::Time::now(); return;}
		
		float area_fire{0.0};
				
		for( auto& detection: msg->detections)
		{
			if( detection.moments[0] > area_fire)
			{
				last_fire_detected_ =  Point2D<float>(detection.u, detection.v) - *center_img_; 
				area_fire_detected_ = area_fire;
				detections_updated_ = true;
			}
		}
		if( !detections_updated_ )
		{
			pan_ref_ = pan_center_;
			tilt_ref_ = tilt_center_;
			last_command_time_ = ros::Time::now();
			last_fire_detected_ = {};
		}
		
		std_msgs::Bool is_detected;
		is_detected.data = last_fire_detected_?true:false;

		fire_detected_pub_.publish(is_detected);
	}

	void Point2Fire::FireDetect3DCallback(const fireawareness_ros::FireDetections3D::ConstPtr& msg)
	{
		detect_fov_.clear();
		for( auto& p : msg->fov)
		{
			detect_fov_.push_back(Point2D<float>( p.x, p.y ));
		} 
	}



	void Point2Fire::CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
	{
		center_img_ = Point2D<float>(msg->K[2], msg->K[5]);
		max_point_img_ = Point2D<float>(msg->width, msg->height);
	}
	
	void Point2Fire::ControlLoop( void)
	{	
		if(last_fire_detected_)
		{			
			const auto& current_time  = ros::Time::now();

			if( !detect_fov_.empty() )
			{
				Point2D<float> perc_point_detect{ last_fire_detected_->x / max_point_img_.x, last_fire_detected_->y / max_point_img_.y };
				
				dist_2_fire_.x = perc_point_detect.x * ( (detect_fov_.at(0).x - detect_fov_.at(1).x) * perc_point_detect.y + 
									(detect_fov_.at(3).x - detect_fov_.at(2).x) * (1 - perc_point_detect.y)) + 
									(detect_fov_.at(1).x * perc_point_detect.y + detect_fov_.at(2).x * (1 - perc_point_detect.y));

				dist_2_fire_.y = perc_point_detect.y * ( (detect_fov_.at(0).y - detect_fov_.at(3).y) * perc_point_detect.x + 
									(detect_fov_.at(1).y - detect_fov_.at(2).y) * (1 - perc_point_detect.x)) + 
									(detect_fov_.at(3).y * perc_point_detect.x + detect_fov_.at(2).y * (1 - perc_point_detect.x));

				ROS_INFO("fire detected at: %f    %f  ", dist_2_fire_.x, dist_2_fire_.y );
				std::cout<<"ENTRA"<<std::endl;
			}

			std::cout<<"ME PASO"<<std::endl;


			arm_pan_pub_.publish(ComputePID( pan_control_, pan_ref_ , last_fire_detected_->x,  fire_offset_x_, current_time - last_command_time_ , pan_min_, pan_max_));
			arm_tilt_pub_.publish(ComputePID( tilt_control_, tilt_ref_ , -last_fire_detected_->y,  fire_offset_y_, current_time - last_command_time_, tilt_min_, tilt_max_ ));					
			last_command_time_ = current_time;
			
			if(detections_updated_)
			{
				detections_updated_ = false;
			}	
			else
			{	
				not_detections_++;
				if( not_detections_ > max_not_detections_)
				{
					not_detections_ = 0;
					last_fire_detected_ = {};
				}
			}
			
		}
	}
	
	std_msgs::Float32 Point2Fire::ComputePID( Pid& _pid, float& _ref, float _p,  float _offset, const ros::Duration& _t, float _limit_min, float _limit_max )
	{
		std_msgs::Float32 force;
		 _ref = _pid.computeCommand( _p - _offset, _t);
		 _ref = std::max( std::min( _ref, _limit_max), _limit_min);
		 force.data =_ref ;
		return force;
	}
	
}
	
	
	

