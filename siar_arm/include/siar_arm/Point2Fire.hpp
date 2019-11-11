#ifndef POINT2FIRE_HPP_
#define POINT2FIRE_HPP_


#include <experimental/optional>

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CameraInfo.h>

#include "siar_arm/siar_arm_ros.hpp"
#include "fireawareness_ros/FireDetections2D.h"


namespace SIAR
{
	template<class T>
	class Point2D{
		public:
			T x;
			T y;
			Point2D( T _x, T _y): x(_x), y(_y) {};
			Point2D operator+(const Point2D &c1)
			{
				return Point2D(this->x + c1.x,this->y + c1.y);
			};
			Point2D operator-(const Point2D &c1)
			{
				return Point2D(this->x - c1.x,this->y - c1.y);
			};
			auto mod()
			{
				return sqrt(pow(this->x,2.0) + pow(this->y,2.0));
			};				
	};	
	
	
class Point2Fire
{	
	public:
		using Pid = control_toolbox::Pid;
	
	public:
		ros::Subscriber fire_detec_sub_;
		ros::Subscriber fire_cam_info_sub_;
		ros::Publisher pointing_fire_pub_;
		ros::Publisher arm_pan_pub_;
		ros::Publisher arm_tilt_pub_;
		
		std::string mot_pan_arm_file_{};
		std::string mot_tilt_arm_file_{};
		std::string robot_name_{};
		
		ros::Time   last_command_time_ { ros::Time::now() };
		
		Pid pan_control_;
		Pid tilt_control_;
		
		
		
		float pan_ref_{0.0};
		float tilt_ref_{0.0};
		float pan_center_{0.0};
		float tilt_center_{0.0};		
		float pan_min_{0.0};
		float tilt_min_{0.0};
		float pan_max_{0.0};
		float tilt_max_{0.0};
		
		float fire_offset_x_, fire_offset_y_;
		
		std::experimental::optional<Point2D<float>> center_img_;
		std::experimental::optional<Point2D<float>> last_fire_detected_;
		float area_fire_detected_{0.0};
				
		Point2Fire(ros::NodeHandle &nh, ros::NodeHandle &pnh);
		~Point2Fire() = default;
		
		void load_data(const std::string &mot_pan_arm_file, const std::string &mot_tilt_arm_file) ;
		
		
		void FireDetectCallback(const fireawareness_ros::FireDetections2D::ConstPtr& msg);
		void CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		void ControlLoop( void);
		std_msgs::Float32 ComputePID( Pid& _pid, float& _ref, float _p,  float _offset, const ros::Duration& _t, float _limit_min, float _limit_max  );	
};		
}

#endif
