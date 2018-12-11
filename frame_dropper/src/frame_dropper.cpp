#include <ros/ros.h>
#include <image_transport/image_transport.h>
// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// OpenCV to ROS stuff
#include <cv_bridge/cv_bridge.h>

class FrameDropper
{
public:
  FrameDropper(void) : it(nh)
  {  
    // Resolve the camera topic
    cameraTopic = nh.resolveName("in");
    imageTopic = cameraTopic+"/rgb/image_raw";
    depthTopic = cameraTopic + "/depth_registered/image_raw";
    cameraTopic = nh.resolveName("out");
    imageOutTopic = cameraTopic+"/rgb/image_raw";
    depthOutTopic = cameraTopic + "/depth_registered/image_raw";
    
    
    // Load parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("frame_skip", frameSkip))
      frameSkip = 3;     
    if(!lnh.getParam("publish_depth", publish_depth)) 
      publish_depth = false;
    if(!lnh.getParam("scale", scale))
      scale = 1.0;
    if(!lnh.getParam("downsample_depth", downsample_depth))
      downsample_depth = true;
    
    // Create subsciber
    sub = it.subscribe(imageTopic, 1, &FrameDropper::imageCb, this); 
    if (publish_depth) 
      depth_sub = it.subscribe(depthTopic, 1, &FrameDropper::depthCb, this);
    
    // Advertise the new Depth Image
    ROS_INFO("Depth topic: %s Depth out topic: %s PUblish depth = %d", depthTopic.c_str(), depthOutTopic.c_str(), publish_depth);
    pub = it.advertise(imageOutTopic, 1);
    if (publish_depth)
      depth_pub = it.advertise(depthOutTopic, 1);
    
    imgCount = 0;
    depthCount = 0;
  }
  
  ~FrameDropper(void)
  {
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      
    imgCount++;
    if(imgCount < frameSkip)
    {
      return;
    }
    imgCount = 0;
    cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat dst(msg->width * scale,msg->height * scale,img_cv->image.type());
    cv::resize(img_cv->image, dst, cv::Size(0,0), scale, scale);
    img_cv->image = dst;
    
    sensor_msgs::ImagePtr pt = img_cv->toImageMsg();
    pub.publish(pt);
//                 ROS_ERROR("Img res: %d, %d", pt->height, pt->width);
  }
  
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
      
    depthCount++;
    if(depthCount < frameSkip)
    {
      return;
    }
    depthCount = 0;
    double scale = downsample_depth?this->scale*0.5:this->scale;
    cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg, "32FC1");
    cv::Mat dst(msg->width * scale,msg->height * scale,img_cv->image.type());
    cv::resize(img_cv->image, dst, cv::Size(0,0), scale, scale);
    img_cv->image = dst;
    
    sensor_msgs::ImagePtr pt = img_cv->toImageMsg();
    depth_pub.publish(pt);
  }
  
  
protected:

  // ROS handler and subscribers
  ros::NodeHandle nh;
  image_transport::ImageTransport it; 
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  image_transport::Publisher depth_pub;
  image_transport::Subscriber depth_sub;
  
  // System params
  std::string cameraTopic, imageTopic, imageOutTopic;
  std::string depthTopic, depthOutTopic;
  int frameSkip;
  int imgCount, depthCount;
  double scale; // Scaling in the image
  bool publish_depth, downsample_depth;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "frame_dropper");  
  FrameDropper node;
  ros::spin();
}








