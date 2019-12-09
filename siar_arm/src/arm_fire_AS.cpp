#include "math.h"
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
#include "fireawareness_ros/FireDetections3D.h"

#include "actionlib/server/simple_action_server.h"
#include "upo_actions/FireDetectionAction.h"



template<class T>
class Point2D{
	public:
		T x;
		T y;
		Point2D( T _x, T _y): x(_x), y(_y) {};
		Point2D(): x(0.0), y(0.0) {};
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
	

	private:
		using Pid = control_toolbox::Pid;
	
	public:

		actionlib::SimpleActionServer<upo_actions::FireDetectionAction> fire_detection_action_server_;

        upo_actions::FireDetectionFeedback feedback_; //variable stores the feedback/intermediate results
        upo_actions::FireDetectionResult result_; //variable stores the final output

		ros::Subscriber fire_detec_sub_;
		ros::Subscriber fire_detec_3D_sub_;
		ros::Subscriber fire_cam_info_sub_;
		ros::Publisher pointing_fire_pub_;
		ros::Publisher arm_pan_pub_;
		ros::Publisher arm_tilt_pub_;
		ros::Publisher arm_mode_as_pub_;
		ros::Publisher fire_detected_pub_;

        ros::NodeHandle nh;

        std::string action_name_;
		
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
		float pan_min_{-0.99};
		float tilt_min_{-0.99};
		float pan_max_{0.99};
		float tilt_max_{0.99};
		
		int not_detections_ {5};
		int max_not_detections_ {0};
		bool detections_updated_ {false};
		
		float fire_offset_x_, fire_offset_y_;

        float pos2D_u_fire, pos2D_v_fire;
        float pos2Dmax_u_fire_, pos2Dmin_u_fire_;
        float pos2Dmax_v_fire_, pos2Dmin_v_fire_;
		
        std::vector<Point2D<float>> detect_fov_;
		
		std::experimental::optional<Point2D<float>> center_img_;
		Point2D<float> max_point_img_;
		Point2D<float> dist_2_fire_;
		std::experimental::optional<Point2D<float>> last_fire_detected_;
		float area_fire_detected_{0.0};

        Point2Fire(ros::NodeHandle nh,ros::NodeHandle pnh,std::string name);
		~Point2Fire() = default;
		
		void load_data(const std::string &mot_pan_arm_file, const std::string &mot_tilt_arm_file) ;
		
		void initializeSubscribers(ros::NodeHandle &nh);
		void initializePublishers(ros::NodeHandle &nh);
		void actionCb(const upo_actions::FireDetectionGoalConstPtr &goal);
		void FireDetectCallback(const fireawareness_ros::FireDetections2D::ConstPtr& msg);
		void FireDetect3DCallback(const fireawareness_ros::FireDetections3D::ConstPtr& msg);
		void CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		void ControlLoop( void);
		std_msgs::Float32 ComputePID(Pid& _pid, float& _ref, float _p,  float _offset, const ros::Duration& _t, float _limit_min, float _limit_max);
			
};	

		Point2Fire::Point2Fire(ros::NodeHandle nh, ros::NodeHandle pnh, std::string name):
        fire_detection_action_server_(name,boost::bind(&Point2Fire::actionCb, this,_1),false),action_name_(name)
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
            pnh.param<float>("pos2Dmax_u_fire", pos2Dmax_u_fire_,94.0);
            pnh.param<float>("pos2Dmin_u_fire", pos2Dmin_u_fire_,91.0);
            pnh.param<float>("pos2Dmax_v_fire", pos2Dmax_v_fire_,53.);
            pnh.param<float>("pos2Dmin_v_fire", pos2Dmin_v_fire_,51.0);
            
            pan_control_ = control_toolbox::Pid(kp_pan,ki_pan,kd_pan);
            tilt_control_ = control_toolbox::Pid(kp_tilt,ki_tilt,kd_tilt);
            
            //~ load_data(mot_pan_arm_file_, mot_tilt_arm_file_) ;
            initializePublishers(nh);
            initializeSubscribers(nh);

            fire_detection_action_server_.start();
					
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
		
		void Point2Fire::initializeSubscribers(ros::NodeHandle &nh)
        {
		    fire_detec_sub_ = nh.subscribe( "firedetections2D", 2,  &Point2Fire::FireDetectCallback, this);
		    fire_detec_3D_sub_ = nh.subscribe( "firedetections3D", 2,  &Point2Fire::FireDetect3DCallback, this);
		    fire_cam_info_sub_ = nh.subscribe("/" + robot_name_ + "/thermal_camera/camera_info", 2,  &Point2Fire::CamInfoCallback, this);
		    ROS_INFO("Subscribers Initialized node Point2Fire");
	    }
		
        void Point2Fire::initializePublishers(ros::NodeHandle &nh)
        {
            pointing_fire_pub_ = nh.advertise<std_msgs::Bool>("pointing_fire_pub", 2);
            arm_pan_pub_ = nh.advertise<std_msgs::Float32>("arm_pan", 2);
            arm_tilt_pub_ = nh.advertise<std_msgs::Float32>("arm_tilt", 2);					
            arm_mode_as_pub_ = nh.advertise<std_msgs::Bool>("arm_mode", 2);					
            fire_detected_pub_ = nh.advertise<std_msgs::Bool>("fire_detected", 2);	
            ROS_INFO("Publishers Initialized node Point2Fire");
        }

        void Point2Fire::actionCb(const upo_actions::FireDetectionGoalConstPtr &goal)
        {
            ros::Rate rate(50);
            bool exe_goal;
            bool success = false;
            exe_goal = goal->fire_centered;
            std_msgs::Bool pub_mode_arm_;

            if (exe_goal)
            {
                pub_mode_arm_.data = true;
                arm_mode_as_pub_.publish(pub_mode_arm_);
                
                ROS_INFO("Init actionCb centering fire");
                while (!success)
                {
                    ControlLoop();
                    
                    if ( ((pos2D_u_fire > pos2Dmax_u_fire_) || (pos2D_u_fire < pos2Dmin_u_fire_)) || 
                         ((pos2D_v_fire > pos2Dmax_v_fire_) || (pos2D_v_fire < pos2Dmin_v_fire_)) )
                    {

                        feedback_.pos_x_fire = pos2D_u_fire;
                        feedback_.pos_y_fire = pos2D_v_fire;
                        fire_detection_action_server_.publishFeedback(feedback_);
                        

                        if (fire_detection_action_server_.isPreemptRequested() || !ros::ok())
                        {
                            goal->fire_centered == false;

                            ROS_INFO("%s: Preempted", action_name_.c_str());
                            // set the action state to preempted
                            fire_detection_action_server_.setPreempted();
                            result_.fire_finded == false;

                            break;
                            
                        }
                        rate.sleep();
                        
                        // ROS_INFO("I am in while loop");
                    }
                    else
                    {
                        success = true;
                        ROS_INFO("Fire in the center of the Image detected");
                    }
                }             
                

                if(success)
                {
                    result_.fire_finded == true;
                    ROS_INFO("%s: Succeeded", action_name_.c_str());
                    // set the action state to succeeded
                    fire_detection_action_server_.setSucceeded(result_);
                    std_msgs::Float32 clean_pub_;
                    clean_pub_.data = 0.0;
                    arm_pan_pub_.publish(clean_pub_);
                    arm_tilt_pub_.publish(clean_pub_);
                }

            }
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
                    pos2D_u_fire = detection.u;
                    pos2D_v_fire = detection.v;
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
		
        void Point2Fire::ControlLoop(void)
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

                    // ROS_INFO("fire detected at:  x=%f   y=%f  ", dist_2_fire_.x, dist_2_fire_.y );
                }

                arm_pan_pub_.publish(ComputePID( pan_control_, pan_ref_ , last_fire_detected_->x,  fire_offset_x_, current_time - last_command_time_ , pan_min_, pan_max_));
			    arm_tilt_pub_.publish(ComputePID( tilt_control_, tilt_ref_ , -last_fire_detected_->y,  fire_offset_y_, current_time - last_command_time_, tilt_min_, tilt_max_));				
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


    int main(int argc, char** argv) 
    {
        ros::init(argc, argv, "siar_fire_arm_node");

        ROS_INFO("Starting SIAR FIRE Arm node");

        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        Point2Fire point2fire(nh,pnh,"fire_detection_AS");
        
        ros::Rate r(ros::Duration(0.05));
        while (ros::ok()) {
            ros::spinOnce();
            // point2fire.ControlLoop();
            r.sleep();
        }	

        return 0;
    
    }

