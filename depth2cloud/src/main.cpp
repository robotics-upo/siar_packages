#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <inttypes.h>
#include <string>

class Depth2Cloud
{
public:
	Depth2Cloud(void) : it(nh)
	{	
		// Resolve the camera topic
		if(nh.resolveName("depth") == "/depth") 
		{
			ROS_WARN("Depth2PointCloud: depth has not been remapped! Typical command-line usage:\n"
				 "\t$ ./depth2pointcloud_node depth:=<depth topic> info:=<camera info topic> cloud:=<cloud topic>");
			exit(0);
		}	
		if(nh.resolveName("info") == "/info") 
		{
			ROS_WARN("Depth2PointCloud: depth has not been remapped! Typical command-line usage:\n"
				 "\t$ ./depth2pointcloud_node depth:=<depth topic> info:=<camera info topic> cloud:=<cloud topic>");
			exit(0);
		}	
		depthTopic = nh.resolveName("depth");
		camInfoTopic = nh.resolveName("info");
		
		// Resolv output point-cloud topic
		if(nh.resolveName("cloud") == "/cloud") 
		{
			ROS_WARN("Depth2PointCloud: scan has not been remapped! Typical command-line usage:\n"
				 "\t$ ./depth2pointcloud_node depth:=<depth topic> info:=<camera info topic> cloud:=<cloud topic>");
		}
		cloudTopic = nh.resolveName("cloud");

		// Load parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("range_min", range_min))
			range_min = 0.6;
		if(!lnh.getParam("range_max", range_max))
			range_max = 5.0;
		if(!lnh.getParam("down_sampling", down_sampling))
			down_sampling = 1;
		if(down_sampling != 1 && down_sampling != 2 && down_sampling != 4)
			down_sampling = 1;
		if(!lnh.getParam("roi_x", roiX))
			roiX = -1;
		if(!lnh.getParam("roi_y", roiY))
			roiY = -1;
		if(!lnh.getParam("roi_w", roiW))
			roiW = -1;
		if(!lnh.getParam("roi_h", roiH))
			roiH = -1;
		isInit = false;

		// Create subscibers
		sub_caminfo = nh.subscribe(camInfoTopic, 1, &Depth2Cloud::camInfoCb, this); // Receive Depth camera info
		sub_depth = it.subscribe(depthTopic, 1, &Depth2Cloud::depthCb, this); // Receive depth images directly from Kinect
	
		// Advertise the new point-cloud
		pub_pc = nh.advertise<sensor_msgs::PointCloud2>(cloudTopic, 1);
	}
	
	~Depth2Cloud(void)
	{
	}
	
	// Processing callback for camera info
	void camInfoCb(const sensor_msgs::CameraInfo::ConstPtr& camInfo)
	{
		if(isInit)
			return;
			
		// Get camera calibration
		fx = camInfo->K[0];
		fy = camInfo->K[4];
		cx = camInfo->K[2];
		cy = camInfo->K[5];
		/*fx = 368.096588;
		fy = 368.096588;
		cx = 261.696594;
		cy = 202.522202;*/
		
		// Get image dimensions
		height = camInfo->height/down_sampling;
		width = camInfo->width/down_sampling;
		
		// Check ROI is ok
		if(roiX == -1 || roiY == -1 || roiW == -1 || roiH == -1)
		{
			roiX = 0;
			roiY = 0;
			roiW = camInfo->width;
			roiH = camInfo->height;
		}
		if(roiX < 0 || roiY<0 || roiX+roiW > camInfo->width || roiY+roiH > camInfo->height)
		{
			std::cout << "Erroneous ROI, ignoring..." << std::cout;
			roiX = 0;
			roiY = 0;
			roiW = camInfo->width;
			roiH = camInfo->height;
		}

		isInit = true;
	}
	
	void depthCb(const sensor_msgs::ImageConstPtr& msg)
	{		
		if(!isInit)
			return;
		
		// Get image from msg without copy
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
			return;
		}
		
		// Convert from depth to point-cloud
		const float constant = 1.0f / fx;
		int depth_idx = 0;
		geometry_msgs::Point32 pt;
		float *depth_data = (float *)(cv_ptr->image.data);
		outCloud.header = msg->header;
		outCloud.points.clear();
		for(int v=roiY; v<roiY+roiH; v+=down_sampling)
		{
			for(int u=roiX; u<roiX+roiW; u+=down_sampling)
			{
				pt.z = cv_ptr->image.at<float>(v,u);
				if(pt.z > range_min && pt.z < range_max)
				{
					pt.x = (float)(u-cx) * pt.z * constant;
					pt.y = (float)(v-cy) * pt.z * constant;
					outCloud.points.push_back(pt);
				}
			}
		}
		/*
		depth_idx = roiY*cv_ptr->image.cols;
		for(int v=roiY; v<roiY+roiH; v+=down_sampling, depth_idx+=(down_sampling-1)*cv_ptr->image.cols)
		{
			depth_idx += roiX;
			for(int u=roiX; u<roiX+roiW; u+=down_sampling, depth_idx+=down_sampling)
			{
				pt.z = depth_data[depth_idx];
				if(pt.z > range_min && pt.z < range_max)
				{
					pt.x = (float)(u-cx) * pt.z * constant;
					pt.y = (float)(v-cy) * pt.z * constant;
					outCloud.points.push_back(pt);
				}
			}
		}/*
		for(int v=0; v<cv_ptr->image.rows; v+=down_sampling, depth_idx+=(down_sampling-1)*cv_ptr->image.cols)
		{
			for(int u=0; u<cv_ptr->image.cols; u+=down_sampling, depth_idx+=down_sampling)
			{
				pt.z = depth_data[depth_idx];
				if(pt.z > range_min && pt.z < range_max)
				{
					pt.x = (float)(u-cx) * pt.z * constant;
					pt.y = (float)(v-cy) * pt.z * constant;
					outCloud.points.push_back(pt);
				}
			}
		}*/
		
		// Publish point-cloud
		sensor_msgs::convertPointCloudToPointCloud2(outCloud, outCloud2);
		pub_pc.publish(outCloud2);
	}
			
protected:

	// ROS handler and subscribers
	ros::NodeHandle nh;
	image_transport::ImageTransport it; 
	image_transport::Subscriber sub_depth;
	ros::Subscriber sub_caminfo;	
	
	// Camera calibration parameters
	bool isInit;
	float fx, fy, cx, cy;	
	int height, width;
	
	// Point clouds
	sensor_msgs::PointCloud outCloud;	
	sensor_msgs::PointCloud2 outCloud2;	
	ros::Publisher pub_pc;
	
	// System params
	std::string depthTopic, camInfoTopic, cloudTopic;
	double range_min, range_max;
	int down_sampling;
	int roiX, roiY, roiW, roiH;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth2cloud_node");  
  Depth2Cloud node;
  ros::spin();
}








