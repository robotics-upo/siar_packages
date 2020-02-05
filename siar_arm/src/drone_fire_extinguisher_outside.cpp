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
        double target_vel_y{0.0};
        double target_vel_z{0.0};

        //Pid objects and gains
        Pid Pid_local_vel_z;
        Pid Pid_local_vel_y;
        float kp_vel_z, ki_vel_z, kd_vel_z;
        float kp_vel_y, ki_vel_y, kd_vel_y;

        //Search fire params
        double search_square_size; //size of square for searching fire pixel
        double search_vel_y;
        double search_vel_z;


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
        float threshold_max_v_fire_, threshold_min_v_fire_;

        float fire_offset_u_, fire_offset_v_;

        double target_vel_y_upper_limit, target_vel_z_upper_limit, target_vel_y_lower_limit, target_vel_z_lower_limit;

        std_msgs::Bool throw_water_msg;
        bool throw_water_flag;
        float throw_water_time;


        Point2Fire(ros::NodeHandle nh, ros::NodeHandle pnh, std::string name);
		~Point2Fire() = default;

        void initializeSubscribers(ros::NodeHandle &nh);
		void initializePublishers(ros::NodeHandle &nh);

        void updateFireDetect();
        void actionCb(const upo_actions::FireExtinguishGoalConstPtr &goal);
        void find_in_square(double search_square_size_);
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
            pnh.param<float>("kp_vel_z", kp_vel_z, 0.1); //set these
            pnh.param<float>("ki_vel_z", ki_vel_z, 0.0);
            pnh.param<float>("kd_vel_z", kd_vel_z, 0.0);
            pnh.param<float>("kp_vel_y", kp_vel_y, 0.1);
            pnh.param<float>("ki_vel_y", ki_vel_y, 0.0);
            pnh.param<float>("kd_vel_y", kd_vel_y, 0.0);

            //Initialize PIDs
            Pid_local_vel_y = control_toolbox::Pid(kp_vel_y, ki_vel_y, kd_vel_y);
            Pid_local_vel_z = control_toolbox::Pid(kp_vel_z, ki_vel_z, kd_vel_z);

            //U-V coordinates where we want to center the fire
            pnh.param<float>("fire_offset_u", fire_offset_u_, 0.0);
            pnh.param<float>("fire_offset_v", fire_offset_v_, 0.0);
            
            //Search parameters
            pnh.param<double>("search_square_size", search_square_size, 1.5);
            pnh.param<double>("search_vel_y", search_vel_y, 0.4); //set these
            pnh.param<double>("search_vel_z", search_vel_z, 0.4);

            //Thresholds for considering a centered fire
            pnh.param<float>("threshold_max_u_fire", threshold_max_u_fire_, 0.1);
            pnh.param<float>("threshold_max_v_fire", threshold_max_v_fire_, 0.5);
            pnh.param<float>("threshold_min_u_fire", threshold_min_u_fire_, -threshold_max_u_fire_); 
            pnh.param<float>("threshold_min_v_fire", threshold_min_v_fire_, -threshold_max_v_fire_);

            //Set Saturation limits
            pnh.param<double>("target_vel_y_upper_limit", target_vel_y_upper_limit, 0.2);
            pnh.param<double>("target_vel_z_upper_limit", target_vel_z_upper_limit, 0.2);
            pnh.param<double>("target_vel_y_lower_limit", target_vel_y_lower_limit, -target_vel_y_upper_limit);
            pnh.param<double>("target_vel_z_lower_limit", target_vel_z_lower_limit, -target_vel_z_upper_limit);
            
            //Set extinguisher parameters
            pnh.param<bool>("throw_water_flag", throw_water_flag, false);
            pnh.param<float>("throw_water_time", throw_water_time, 5.0);

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
		    fire_detec_sub_ = nh.subscribe( "firedetections2D", 2,  &Point2Fire::FireDetectCallback, this);
		    fire_cam_info_sub_ = nh.subscribe("/" + robot_name_ + "/thermal_camera/camera_info", 1,  &Point2Fire::CamInfoCallback, this);
		    ROS_INFO("Subscribers Initialized for class Point2Fire");
	    }
		
        void Point2Fire::initializePublishers(ros::NodeHandle &nh)
        {	
            local_vel_pub_ = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 2);	// si se pone '/' al principio, coge el namespace?? o es como en los SDF??
            throw_water_pub_ = nh.advertise<std_msgs::Bool>("state_pump", 2);
            ROS_INFO("Publishers Initialized for class Point2Fire");
        }

        //This update will run in the main thread
        void Point2Fire::updateFireDetect() //este más o menos lo dejo como Gonzalo lo hizo
        {
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

            //Send action feedback
            feedback_.pos_x_fire = last_fire_detected_->x; 
            feedback_.pos_y_fire = last_fire_detected_->y;
            fire_detection_action_server_.publishFeedback(feedback_);

            //check if fire dissapeared from image
            current_update_time = ros::Time::now();
            detection_time_elapsed = current_update_time - last_fire_detected_time;
            if(detection_time_elapsed >= not_fire_detected_threshold_time){

                fire_detected = false;
            }

            // Check if action is Preempted
            if (fire_detection_action_server_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                result_.fire_finded = false;
                fire_detection_action_server_.setPreempted(result_);
            }

            // Check if action is cancel

        }

        //This action callback will run on its own thread
        void Point2Fire::actionCb(const upo_actions::FireExtinguishGoalConstPtr &goal)
        {   

            bool execute_action = false;
            execute_action = goal->fire_centered; //if true, search will start
            fire_not_found = false;
            fire_is_centered = false;
            bool fire_was_lost = false;
             

            if (execute_action) 
            //se puede parar esto segun un stop del action? -> pero entonces habria k comprobar a menudo
            {  
                ROS_INFO("Init actionCb looking for a fire");
                
                //Check if fire is already on image. If not, try to find it
                if(!fire_detected){
                    ROS_INFO("No fire on image, Starting fire centering"); 
                    find_in_square(search_square_size);
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
                        if ( ((last_fire_detected_->x > threshold_min_u_fire_) && (last_fire_detected_->x < threshold_max_u_fire_)) &&
                            ((last_fire_detected_->y > threshold_min_v_fire_) && (last_fire_detected_->y < threshold_max_v_fire_)) )
                        {
                            fire_is_centered = true;
                            continue;
                        }

                        
                    }      
                }     
                
                if (fire_not_found){
                    ROS_INFO("Fire was not in the zone, exiting action callback");
                    result_.fire_finded == false;
                    // fire_detection_action_server_.setRejected(result_);
                }
                else if (fire_was_lost){
                    ROS_INFO("Fire was in the zone, but was lost while centering, exiting action callback");
                    result_.fire_finded == false;
                    fire_detection_action_server_.setAborted(result_);
                }
                else if(fire_is_centered){
                    ROS_INFO("Fire was centered, starting to throw water");
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
                    // fire_detection_action_server_.setRejected(result_); 
                }   
                
            }
        }

        void Point2Fire::find_in_square(double search_square_size_){
            
            //With search_vel and square_size set times
            double side_time = search_square_size/search_vel_y; //in case vel_z = vel_y
            ros::Duration short_side_time = ros::Duration(side_time/2); //esto esta en segundos??
            ros::Duration large_side_time = ros::Duration(side_time); 

            ros::Time movement_start_time = ros::Time::now();
            ros::Time movement_current_time = ros::Time::now();
            ros::Duration movement_time_elapsed;
        
            //1. first movement to left short
            //Set vels
            target_vel_msg.axes[1] = search_vel_y; //vel_y
            target_vel_msg.axes[2] = 0.0; //vel_z
            while(!fire_detected && (movement_time_elapsed <= short_side_time)){ //meter aqui un cancel por stop del action???
                //publish vel
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg); //ojo, rellenar los otros campos del mensaje
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //2. first movement up short
            target_vel_msg.axes[1] = 0.0; 
            target_vel_msg.axes[2] = search_vel_z; 
            movement_start_time = ros::Time::now();
            movement_current_time = ros::Time::now();
            while(!fire_detected && (movement_time_elapsed <= short_side_time)){
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //3. first movement to right large
            target_vel_msg.axes[1] = -search_vel_y; 
            target_vel_msg.axes[2] = 0.0; 
            movement_start_time = ros::Time::now();
            movement_current_time = ros::Time::now();
            while(!fire_detected && (movement_time_elapsed <= large_side_time)){
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //4. first movement down large
            target_vel_msg.axes[1] = 0.0; 
            target_vel_msg.axes[2] = -search_vel_z; 
            movement_start_time = ros::Time::now();
            movement_current_time = ros::Time::now();
            while(!fire_detected && (movement_time_elapsed <= large_side_time)){
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //5. second movement left large
            target_vel_msg.axes[1] = search_vel_y; 
            target_vel_msg.axes[2] = 0.0; 
            movement_start_time = ros::Time::now();
            movement_current_time = ros::Time::now();
            while(!fire_detected && (movement_time_elapsed <= large_side_time)){
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //6. second movement up short
            target_vel_msg.axes[1] = 0.0; 
            target_vel_msg.axes[2] = search_vel_z; 
            movement_start_time = ros::Time::now();
            movement_current_time = ros::Time::now();
            while(!fire_detected && (movement_time_elapsed <= short_side_time)){
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //7. return to origin (right) short
            target_vel_msg.axes[1] = -search_vel_y; 
            target_vel_msg.axes[2] = 0.0; 
            movement_start_time = ros::Time::now();
            movement_current_time = ros::Time::now();
            while(!fire_detected && (movement_time_elapsed <= short_side_time)){
                target_vel_msg.header.stamp = ros::Time::now();
                local_vel_pub_.publish(target_vel_msg);
                rt_.sleep();
                movement_current_time = ros::Time::now();
                movement_time_elapsed = movement_current_time - movement_start_time;
            }

            //Stop movement (fire was found or movement got back to initial pose)
            target_vel_msg.axes[1] = 0.0; 
            target_vel_msg.axes[2] = 0.0; 
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
            target_vel_y += Pid_local_vel_y.computeCommand( last_fire_detected_->x - fire_offset_u_, current_time - last_command_time_); 
            target_vel_z += Pid_local_vel_z.computeCommand( last_fire_detected_->y - fire_offset_v_, current_time - last_command_time_);				
            last_command_time_ = current_time;

            // //Apply saturation limits to y_vel- Commented until setting PID gains
            // if (target_vel_y > target_vel_y_upper_limit)
            //   target_vel_y = target_vel_y_upper_limit;
            // else if (target_vel_y < target_vel_y_lower_limit_)
            //   target_vel_y = target_vel_y_lower_limit_;

            // //Apply saturation limits to z_vel- Commented until setting PID gains
            // if (target_vel_y > target_vel_y_upper_limit)
            //   target_vel_y = target_vel_y_upper_limit;
            // else if (target_vel_y < target_vel_y_lower_limit_)
            //   target_vel_y = target_vel_y_lower_limit_;

            target_vel_msg.axes[1] = target_vel_y;
            target_vel_msg.axes[2] = target_vel_z;
            target_vel_msg.header.stamp = ros::Time::now();
            local_vel_pub_.publish(target_vel_msg);
            rt_.sleep(); 

            ROS_INFO("valor de last en X= %f   Y=%f",last_fire_detected_->x, last_fire_detected_->y);

            
        }



        void Point2Fire::throw_water(){
            
            // ros::Rate rtcw_(ros::Duration(throw_water_time));// mirar como hacer para que funcione
            ros::Rate rtcw_(ros::Duration(5.0));
            throw_water_pub_.publish(throw_water_msg);
            rtcw_.sleep();
            throw_water_flag = false;
            throw_water_msg.data = throw_water_flag;
            throw_water_pub_.publish(throw_water_msg);
        }


    int main(int argc, char** argv) 
    {
        ros::init(argc, argv, "drone_fire_extinguisher_outside");

        ROS_INFO("Starting drone fire extinguisher for outside");

        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        Point2Fire point2fire(nh,pnh,"drone_fire_extinguisher_outside");
        
        ros::Rate r(ros::Duration(0.05));
        while (ros::ok()) {
            ros::spinOnce();
            point2fire.updateFireDetect();
            r.sleep();
        }	

        return 0;
    
    }

