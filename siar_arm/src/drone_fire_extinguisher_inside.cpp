#include "math.h"
#include <experimental/optional>

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>

#include "fireawareness_ros/FireDetections2D.h"
#include "fireawareness_ros/FireDetections3D.h"

#include "actionlib/server/simple_action_server.h"
#include "upo_actions/FireExtinguishAction.h"

#include <sensor_msgs/Joy.h>

#include <fstream>

template<class T>
class Point2D{  //pork se ha creado esto??
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

		actionlib::SimpleActionServer<upo_actions::FireExtinguishAction> fire_detection_action_server_;

        //store action outputs: feedback and result 
        std::string action_name_; //taken from node name
        upo_actions::FireExtinguishFeedback feedback_; //dar valores
        upo_actions::FireExtinguishResult result_; //dar valores

        //Subscribers and Publishers
		ros::Subscriber fire_detec_sub_;
		ros::Subscriber fire_cam_info_sub_;
        ros::Publisher throw_water_pub_;
        ros::Publisher local_vel_pub_;
        ros::Rate rt_{ros::Duration(0.10)}; //rate for publishing outputs

        //Velocities output message
        sensor_msgs::Joy target_vel_msg;
        double target_yaw_rate{0.0};

        //Pid objects and gains
        Pid Pid_local_yaw_rate;
        float kp_yaw_rate, ki_yaw_rate, kd_yaw_rate;

        //Search fire params
        double search_yaw_angle; //rotation angle for searching fire pixel
        double search_yaw_rate; //in radians

        bool action_goal_handle_started{false};


        ros::Time last_command_time_{ros::Time::now()}; // no se si es necesario

        //Times for setting back the detection to false (if it was lost from image)
        ros::Time current_update_time{ros::Time::now()};
        ros::Time last_fire_detected_time{ros::Time::now()};
        ros::Duration detection_time_elapsed;
        ros::Duration not_fire_detected_threshold_time{ros::Duration(0.5)}; //set this by param


        std::experimental::optional<Point2D<float>> center_img_; //principal point (cx, cy) from camera ¿¿¿que tiene esta libreria???
        
        bool fire_detected{false};
        bool fire_not_found{false};
        bool fire_is_centered{false};
        fireawareness_ros::FireDetections2D fire_detection_2D;

        std::experimental::optional<Point2D<float>> last_fire_detected_; //last fire detected position

        float threshold_max_u_fire_, threshold_min_u_fire_;

        float fire_offset_u_, fire_offset_v_;

        double target_yaw_rate_upper_limit, target_yaw_rate_lower_limit;

        std_msgs::Bool throw_water_msg;
        bool throw_water_flag;
        float throw_water_time;
        ros::Duration throw_water_duration{ros::Duration(5.0)};


        Point2Fire(ros::NodeHandle nh, ros::NodeHandle pnh, std::string name);
		~Point2Fire() = default;

        void initializeSubscribers(ros::NodeHandle &nh);
		void initializePublishers(ros::NodeHandle &nh);

        void updateFireDetect();
        void actionCb(const upo_actions::FireExtinguishGoalConstPtr &goal);
        void find_in_angle(double yaw_angle_);
        double executePIDs();

		std::string robot_name_{}; 
		
		
		void FireDetectCallback(const fireawareness_ros::FireDetections2D::ConstPtr& msg);
		void CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		
        void throw_water();
		
		
};	

        //Initializing the server in the constructor like this, we create a new thread on which action callback will be running
		Point2Fire::Point2Fire(ros::NodeHandle nh, ros::NodeHandle pnh, std::string name):
        fire_detection_action_server_(name, boost::bind(&Point2Fire::actionCb, this,_1),false), action_name_(name)
        {

            //Initialize vel output message
            target_vel_msg.axes.resize(4);
            target_vel_msg.axes[0] = 0.0; //vel_x
            target_vel_msg.axes[1] = 0.0; //vel_y
            target_vel_msg.axes[2] = 0.0; //vel_z
            target_vel_msg.axes[3] = 0.0; //yaw_rate

            //Pid gains
            pnh.param<float>("kp_yaw_rate", kp_yaw_rate, 0.001); //set these
            pnh.param<float>("ki_yaw_rate", ki_yaw_rate, 0.0);
            pnh.param<float>("kd_yaw_rate", kd_yaw_rate, 0.0001);
            

            //Initialize PIDs
            Pid_local_yaw_rate = control_toolbox::Pid(kp_yaw_rate, ki_yaw_rate, kd_yaw_rate);

            //U-V coordinates where we want to center the fire
            pnh.param<float>("fire_offset_u", fire_offset_u_, 0.0);
            
            //Search parameters
            pnh.param<double>("search_yaw_angle", search_yaw_angle, 360.0);
            pnh.param<double>("search_yaw_rate", search_yaw_rate, 0.4); //set these

            //Thresholds for considering a centered fire
            pnh.param<float>("threshold_max_u_fire", threshold_max_u_fire_, 0.6);
            pnh.param<float>("threshold_min_u_fire", threshold_min_u_fire_, -threshold_max_u_fire_); 
           

            //Set Saturation limits
            pnh.param<double>("target_yaw_rate_upper_limit", target_yaw_rate_upper_limit, 0.2);
            pnh.param<double>("target_yaw_rate_lower_limit", target_yaw_rate_lower_limit, -target_yaw_rate_upper_limit);
            
            //Set extinguisher parameters
            pnh.param<bool>("throw_water_flag", throw_water_flag, false);
            pnh.param<float>("throw_water_time", throw_water_time, 7.0);
            throw_water_duration = ros::Duration(throw_water_time);

            //Initialize output vel message
            target_vel_msg.header.stamp = ros::Time::now();
            target_vel_msg.header.frame_id = "joy_vel_frame"; //check this frame

        
            //Initialize publishers, subscribers and action server
            initializePublishers(nh);
            initializeSubscribers(nh);
            fire_detection_action_server_.start();
					
	    }	
				
		
		void Point2Fire::initializeSubscribers(ros::NodeHandle &nh)
        {
            fire_detec_sub_ = nh.subscribe("firedetections2D", 2,  &Point2Fire::FireDetectCallback, this);
            fire_cam_info_sub_ = nh.subscribe("thermal_camera/camera_info", 1,  &Point2Fire::CamInfoCallback, this);
		    ROS_INFO("Subscribers Initialized for class Point2Fire");
	    }
		
        void Point2Fire::initializePublishers(ros::NodeHandle &nh)
        {	
            local_vel_pub_ = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 2);	
            throw_water_pub_ = nh.advertise<std_msgs::Bool>("state_pump", 2);
            ROS_INFO("Publishers Initialized for class Point2Fire");
        }

        //This update will run in the main thread
        void Point2Fire::updateFireDetect() //este más o menos lo dejo como Gonzalo lo hizo
        {

            // ROS_INFO("Entrando en hilo");
            if (!center_img_) {  
                last_command_time_ = ros::Time::now(); //¿¿por que?? -> creo k esto no hace nada clave
                return;
            }
                    
            for( auto& detection: fire_detection_2D.detections)  //que hace esto??
            {
                if( detection.moments[0] > 0.0) //mirar que valores puede tener esto -> calcula el punto medio del objeto en la imagen 
                {
                    last_fire_detected_ =  Point2D<float>(detection.u, detection.v) - *center_img_;  
                }
            }

            //check if fire dissapeared from image
            current_update_time = ros::Time::now();
            detection_time_elapsed = current_update_time - last_fire_detected_time;
            if(detection_time_elapsed >= not_fire_detected_threshold_time){

                fire_detected = false;
            }

            //Check if action goal handle was initialize (received a message)
            if(action_goal_handle_started){ //otra manera de ver esto??
                //Send action feedback
                feedback_.pos_x_fire = last_fire_detected_->x; 
                feedback_.pos_y_fire = last_fire_detected_->y;
                fire_detection_action_server_.publishFeedback(feedback_);

                // Check if action is Preempted
                if (fire_detection_action_server_.isPreemptRequested() || !ros::ok())
                {
                    //Stop drone
                    target_vel_msg.axes[3] = 0.0;
                    target_vel_msg.header.stamp = ros::Time::now();
                    local_vel_pub_.publish(target_vel_msg);
                    rt_.sleep(); 

                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    result_.fire_finded = false;
                    fire_detection_action_server_.setPreempted(result_);
                }

                // Check if action is cancel
            }

        }

        //This action callback will run on its own thread
        void Point2Fire::actionCb(const upo_actions::FireExtinguishGoalConstPtr &goal)
        {   
            // goal = fire_detection_action_server_.acceptNewGoal(); //trying to fix a problem
            // ROS_INFO("Entered to the callback");
            action_goal_handle_started = true;

            bool execute_action = false;
            execute_action = goal->fire_centered; //if true, search will start
            fire_not_found = false;
            fire_is_centered = false;
            bool fire_was_lost = false;
             

            if (execute_action) 
            {  
                ROS_INFO("Init actionCb looking for a fire");
                
                //Check if fire is already on image. If not, try to find it
                if(!fire_detected){
                    ROS_INFO("No fire on image, Starting fire centering"); 
                    find_in_angle(search_yaw_angle);
                }
                else{
                    ROS_INFO("Fire already on image"); 
                }

                //If fire was found, try to center it
                if (fire_detected){
                    ROS_INFO("Starting fire centering");  

                    while (!fire_is_centered && !fire_was_lost){

                        if (!fire_detected){
                        ROS_INFO("Lost position of fire while centering");
                        fire_was_lost = true;
                        continue;
                        }

                        executePIDs();

                        //Check Centering
                        if ( ((last_fire_detected_->x > threshold_min_u_fire_) && (last_fire_detected_->x < threshold_max_u_fire_)))
                        {
                            fire_is_centered = true;
                            continue;
                        }

                        
                    }      
                }   

                //Stop movement
                target_vel_msg.axes[3] = 0.0;
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();   
                
                if (fire_not_found){
                    ROS_INFO("Fire was not in the zone, exiting action callback");
                    result_.fire_finded == false;
                    fire_detection_action_server_.setAborted(result_);
                }
                else if (fire_was_lost){
                    ROS_INFO("Fire was in the zone, but was lost while centering, exiting action callback");
                    result_.fire_finded == false;
                    fire_detection_action_server_.setAborted(result_);
                }
                else if(fire_is_centered){
                    ROS_INFO("Fire was centered in the horizontal, starting to throw water");
                    throw_water_flag = true;
                    throw_water_msg.data = throw_water_flag;
                    throw_water();
                    
                    //Send Success
                    ROS_INFO("Finished, water throw. Action succeded");
                    result_.fire_finded == true;
                    fire_detection_action_server_.setSucceeded(result_);
                }
                else{
                    ROS_INFO("What the hell just happened!!!");
                    result_.fire_finded == false;
                    fire_detection_action_server_.setAborted(result_); 
                }   
                
            }
        }

        void Point2Fire::find_in_angle(double search_yaw_angle_){

                //With search_yaw_rate and search_yaw_angle set rotation time
                double yaw_angle_in_radians = search_yaw_angle_ * M_PI / 180.0;
                ros::Duration rotation_time = ros::Duration(search_yaw_angle_/search_yaw_rate); //esto esta en segundos??

                ros::Time movement_start_time = ros::Time::now();
                ros::Time movement_current_time = ros::Time::now();
                ros::Duration movement_time_elapsed = ros::Duration(0.0);
            
                //1. First and only movement, rotation to left
                ROS_INFO("1. Doing rotation to left");
                //Set vels
                target_vel_msg.axes[3] = search_yaw_rate; 
                while(!fire_detected && (movement_time_elapsed <= rotation_time)){ 
                    //publish vel
                    target_vel_msg.header.stamp = ros::Time::now();
                    local_vel_pub_.publish(target_vel_msg); //ojo, rellenar los otros campos del mensaje
                    rt_.sleep();
                    movement_current_time = ros::Time::now();
                    movement_time_elapsed = movement_current_time - movement_start_time;
                }

                //Stop movement (fire was found or search movement finished) 
                target_vel_msg.axes[3] = 0.0; 
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();

                ROS_INFO("Search finished");
                if (fire_detected){
                    ROS_INFO("Found pixel of fire");  
                    return;
                }
                else{
                    ROS_INFO("Did't find any fire"); 
                    fire_not_found = true;  
                }

        }


        

		void Point2Fire::FireDetectCallback(const fireawareness_ros::FireDetections2D::ConstPtr& msg)
        {
            ROS_INFO_ONCE("A fire has been detected!!");
            fire_detection_2D.detections = msg->detections;
            fire_detection_2D.header = msg->header;
            fire_detected = true;
            last_fire_detected_time = ros::Time::now();
	    }


		void Point2Fire::CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
        {
            center_img_ = Point2D<float>(msg->K[2], msg->K[5]);
        }
		


        double Point2Fire::executePIDs()
        {
            ros::Time current_time  = ros::Time::now();

            //Incremental control
            //mirar signos de last_fire y del control incremental
            target_yaw_rate += Pid_local_yaw_rate.computeCommand( last_fire_detected_->x - fire_offset_u_, current_time - last_command_time_); 				
            last_command_time_ = current_time;

            // //Apply saturation limits to y_vel- Commented until setting PID gains
            // if (target_yaw_rate > target_yaw_rate_upper_limit)
            //   target_yaw_rate = target_yaw_rate_upper_limit;
            // else if (target_yaw_rate < target_yaw_rate_lower_limit_)
            //   target_yaw_rate = target_yaw_rate_lower_limit_;


            target_vel_msg.axes[3] = target_yaw_rate;
            target_vel_msg.header.stamp = ros::Time::now();
            local_vel_pub_.publish(target_vel_msg);
            rt_.sleep(); 

            ROS_INFO("valor de last en X= %f   Y=%f",last_fire_detected_->x, last_fire_detected_->y);

            
        }



        void Point2Fire::throw_water(){
            
            // ros::Rate rtcw_(ros::Duration(throw_water_time));// mirar como hacer para que funcione
            ros::Rate rtcw_(throw_water_duration);
            ROS_INFO("Throwing water for %f seconds", throw_water_time);
            throw_water_pub_.publish(throw_water_msg);
            rtcw_.sleep();
            throw_water_flag = false;
            throw_water_msg.data = throw_water_flag;
            throw_water_pub_.publish(throw_water_msg);
        }


    int main(int argc, char** argv) 
    {
        ros::init(argc, argv, "drone_fire_extinguisher_inside");

        ROS_INFO("Starting drone fire extinguisher for inside");

        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        Point2Fire point2fire(nh,pnh,"drone_fire_extinguisher_inside");
        
        ros::Rate r(ros::Duration(0.05));
        while (ros::ok()) {
            ros::spinOnce();
            point2fire.updateFireDetect();
            r.sleep();
        }	

        return 0;
    
    }

