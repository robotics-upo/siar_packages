#include <ros/ros.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <siar_driver/SiarStatus.h>

double g_width, g_shift;

visualization_msgs::MarkerArray buildRobotModel(std::string frame_id, float width, float shift)
{
	int id = 0;
	visualization_msgs::MarkerArray model;
	visualization_msgs::Marker marker;
	geometry_msgs::Point p;
	
	// Add front-left wheel
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "siar_model";
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.29;
	marker.pose.position.y = width/2.0 - 0.12;
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x = -0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.08;
	marker.color.a = 1.0; 
	marker.color.r = 75.0/255.0;
	marker.color.g = 75.0/255.0;
	marker.color.b = 75.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add front-right wheel
	marker.id = id++;
	marker.pose.position.x = 0.29;
	marker.pose.position.y = -(width/2.0 - 0.12);
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.08;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add back-right wheel
	marker.id = id++;
	marker.pose.position.x = -0.29;
	marker.pose.position.y = -(width/2.0 - 0.12);
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.08;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add back-left wheel
	marker.id = id++;
	marker.pose.position.x = -0.29;
	marker.pose.position.y = width/2.0 - 0.12;
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x = -0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.08;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add central-left wheel
	marker.id = id++;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = width/2.0 - 0.12;
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x = -0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.08;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add central-right wheel
	marker.id = id++;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = -(width/2.0 - 0.12);
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.08;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add front-left gearbox
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.29;
	marker.pose.position.y = width/2.0 - 0.18;
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.04;
	marker.scale.z = 0.16;
	marker.color.a = 1.0; 
	marker.color.r = 175.0/255.0;
	marker.color.g = 175.0/255.0;
	marker.color.b = 175.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add front-right gearbox
	marker.id = id++;
	marker.pose.position.x = 0.29;
	marker.pose.position.y = -(width/2.0 - 0.18);
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.04;
	marker.scale.z = 0.16;
	marker.color.a = 1.0; 
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add back-right gearbox
	marker.id = id++;
	marker.pose.position.x = -0.29;
	marker.pose.position.y = -(width/2.0 - 0.18);
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.04;
	marker.scale.z = 0.16;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add back-left gearbox
	marker.id = id++;
	marker.pose.position.x = -0.29;
	marker.pose.position.y = width/2.0 - 0.18;
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.04;
	marker.scale.z = 0.16;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add center-left gearbox
	marker.id = id++;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = width/2.0 - 0.18;
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.04;
	marker.scale.z = 0.16;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add center-right gearbox
	marker.id = id++;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = -(width/2.0 - 0.18);
	marker.pose.position.z = 0.13;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.04;
	marker.scale.z = 0.16;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add lower body
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = shift;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.15;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.8;
	marker.scale.y = 0.12;
	marker.scale.z = 0.10;
	marker.color.a = 1.0; 
	marker.color.r = 175.0/255.0;
	marker.color.g = 175.0/255.0;
	marker.color.b = 175.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add electronic box
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = shift;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.235;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.26;
	marker.scale.y = 0.19;
	marker.scale.z = 0.07;
	marker.color.a = 1.0; 
	marker.color.r = 255.0/255.0;
	marker.color.g = 140.0/255.0;
	marker.color.b = 0.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);
	marker.id = id++;
	marker.pose.position.x = shift;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.305;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.26;
	marker.scale.y = 0.26;
	marker.scale.z = 0.07;
	marker.points.clear();
	model.markers.push_back(marker);
	marker.id = id++;
	marker.pose.position.x = shift;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.355;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.03;
	marker.color.r = 255.0/255.0;
	marker.color.g = 255.0/255.0;
	marker.color.b = 255.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Robot front arms
	marker.id = id++;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.scale.x = 0.02;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;
	marker.color.a = 1.0; 
	marker.color.r = 175.0/255.0;
	marker.color.g = 175.0/255.0;
	marker.color.b = 175.0/255.0;
	marker.points.clear();
	p.x = 0.35;
	p.y = width/2.0 - 0.18;
	p.z = 0.14;
	marker.points.push_back(p);
	p.x = shift+0.4-0.05;
	p.y = 0.06;
	marker.points.push_back(p);
	p.x = shift+0.4-0.05;
	p.y = -0.06;
	marker.points.push_back(p);
	p.x = 0.35;
	p.y = -(width/2.0 - 0.18);
	marker.points.push_back(p);
	p.x = 0.23;
	p.y = -(width/2.0 - 0.18);
	marker.points.push_back(p);
	p.x = shift+0.4-0.17;
	p.y = -0.06;
	marker.points.push_back(p);
	p.x = shift+0.4-0.17;
	p.y = 0.06;
	marker.points.push_back(p);
	p.x = 0.23;
	p.y = width/2.0 - 0.18;
	marker.points.push_back(p);
	model.markers.push_back(marker);
	
	// Robot center arms
	marker.id = id++;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points.clear();
	p.x = 0.06;
	p.y = width/2.0 - 0.18;
	p.z = 0.14;
	marker.points.push_back(p);
	p.x = shift+0.06;
	p.y = 0.06;
	marker.points.push_back(p);
	p.x = shift+0.06;
	p.y = -0.06;
	marker.points.push_back(p);
	p.x = 0.06;
	p.y = -(width/2.0 - 0.18);
	marker.points.push_back(p);
	p.x = -0.06;
	p.y = -(width/2.0 - 0.18);
	marker.points.push_back(p);
	p.x = shift-0.06;
	p.y = -0.06;
	marker.points.push_back(p);
	p.x = shift-0.06;
	p.y = 0.06;
	marker.points.push_back(p);
	p.x = -0.06;
	p.y = width/2.0 - 0.18;
	marker.points.push_back(p);
	model.markers.push_back(marker);
	
	// Robot rear arms
	marker.id = id++;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points.clear();
	marker.points.clear();
	p.x = -0.35;
	p.y = width/2.0 - 0.18;
	p.z = 0.14;
	marker.points.push_back(p);
	p.x = -(-shift+0.4-0.05);
	p.y = 0.06;
	marker.points.push_back(p);
	p.x = -(-shift+0.4-0.05);
	p.y = -0.06;
	marker.points.push_back(p);
	p.x = -0.35;
	p.y = -(width/2.0 - 0.18);
	marker.points.push_back(p);
	p.x = -0.23;
	p.y = -(width/2.0 - 0.18);
	marker.points.push_back(p);
	p.x = -(-shift+0.4-0.17);
	p.y = -0.06;
	marker.points.push_back(p);
	p.x = -(-shift+0.4-0.17);
	p.y = 0.06;
	marker.points.push_back(p);
	p.x = -0.23;
	p.y = width/2.0 - 0.18;
	marker.points.push_back(p);
	model.markers.push_back(marker);
	
	// Add batteries
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = shift+0.23;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.235;
	marker.pose.orientation.x =  0.0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = 0.17;
	marker.scale.y = 0.18;
	marker.scale.z = 0.07;
	marker.color.a = 1.0; 
	marker.color.r = 0.0/255.0;
	marker.color.g = 0.0/255.0;
	marker.color.b = 0.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);
	marker.id = id++;
	marker.pose.position.x = shift-0.23;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.235;
	marker.points.clear();
	model.markers.push_back(marker);
	return model;
}

void statusCallback(const siar_driver::SiarStatus::ConstPtr& msg)
{
	g_width = msg->width;
	g_shift = msg->electronics_x;
}

int main( int argc, char **argv)
{
	// Setup ROS
	ros::init(argc, argv, "siar_model_viz");
	ros::NodeHandle nh;
	ros::NodeHandle lnh("~");
	ros::Publisher pubModel;
	ros::Subscriber subStatus;
	
	// Read parameters
	std::string frame_id;
	double hz;
	if(!lnh.getParam("frame_id", frame_id))
		frame_id = "/base_link";	
	if(!lnh.getParam("hz", hz))
		hz = 10.0;	
	
	// Create the publisher and subscribers
	g_width = 0.6;
	g_shift = 0.1;
	pubModel = lnh.advertise<visualization_msgs::MarkerArray>("siar_model", 0);
	subStatus = lnh.subscribe("siar_status", 1, statusCallback);
	
	// Loop for ever
	ros::Rate loop_rate(hz);
	while(ros::ok())
	{
		// Create the model
		visualization_msgs::MarkerArray model = buildRobotModel(frame_id, g_width, g_shift);	
		
		// Pibllish model
		pubModel.publish(model);
		
		// Update ros and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}


