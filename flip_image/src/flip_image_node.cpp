#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher pub;

int flip_type = -1;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		// Flip input image
		cv_bridge::CvImage flipImage;               
		cv::flip(cv_bridge::toCvShare(msg, "")->image, flipImage.image, flip_type);     // -1 means horizontal and vertical flip
                flipImage.header = msg->header;
                flipImage.encoding = msg->encoding;
		
		// Publish flip image
        pub.publish(flipImage.toImageMsg());
	}
	catch(...)
	{ 
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flip_image_node");
	ros::NodeHandle nh;
        
        nh.getParam("flip_type",flip_type);
        
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("input_image", 1, imageCallback);
	pub = it.advertise("flip_image", 1);
	ros::spin();
}




