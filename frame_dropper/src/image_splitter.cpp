#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// OpenCV to ROS stuff
#include <cv_bridge/cv_bridge.h>

class ImageSplitter
{
public:
  ImageSplitter(void) : it(nh)
  {  
    // Resolve the camera topic
    cameraName = nh.resolveName("in");
    cameraInfoTopic = cameraName+"/rgb/camera_info";
    imageTopic = cameraName+"/rgb/image_raw";
    depthTopic = cameraName + "/depth_registered/image_raw";
    depthInfoTopic = cameraName+"/depth_registered/camera_info";
    cameraName = nh.resolveName("out");
    cameraInfoOutTopic = cameraName+"/rgb/camera_info";
    imageOutTopic = cameraName+"/rgb/image_raw";
    depthOutTopic = cameraName + "/depth_registered/image_raw";
    depthInfoOutTopic = cameraName+"/depth_registered/camera_info";
    
    cameraName_2 = nh.resolveName("in_2");
    cameraInfoTopic_2 = cameraName_2 + "/rgb/camera_info";
    imageTopic_2 = cameraName_2 +"/rgb/image_raw";
    depthTopic_2 = cameraName_2 + "/depth_registered/image_raw";
    depthInfoTopic_2  = cameraName_2 + "/depth_registered/camera_info";
    cameraName_2 = nh.resolveName("out_2");
    cameraInfoOutTopic_2 = cameraName_2 + "/rgb/camera_info";
    imageOutTopic_2 = cameraName_2 + "/rgb/image_raw";
    depthOutTopic_2 = cameraName_2 + "/depth_registered/image_raw";
    depthInfoOutTopic_2 = cameraName_2 + "/depth_registered/camera_info";
    
    reverseTopic = "/reverse";
    allCamerasTopic = "/all_cameras";
    publishDepthTopic = "/publish_depth";
    
    // Load parameters
    ros::NodeHandle lnh("~");
    if(!lnh.getParam("frame_skip", frameSkip))
      frameSkip = 2;     
    if(!lnh.getParam("use_depth", use_depth)) 
      use_depth = true;
    if(!lnh.getParam("publish_depth", publish_depth)) 
      publish_depth = true;
    if(!lnh.getParam("scale", scale))
      scale = 1.0;
    if(!lnh.getParam("publish_all", publish_all))
      publish_all = false;
    if(!lnh.getParam("scale_depth", scale_depth))
      scale_depth = scale * 0.5;
    if(!lnh.getParam("only_depth", only_depth))
      only_depth = false;
    if(!lnh.getParam("flip_1", flip_1))
      flip_1 = false;
    if(!lnh.getParam("flip_2", flip_2))
      flip_2 = false;
    
    reverse = false;

    // Camera info pubs and subs
    camera_info_sub = nh.subscribe(cameraInfoTopic, 1, &ImageSplitter::infoCb, this);
    camera_info_sub_2 = nh.subscribe(cameraInfoTopic_2, 1, &ImageSplitter::infoCb_2, this);
    depth_info_sub = nh.subscribe(depthInfoTopic, 1, &ImageSplitter::depthInfoCb, this);
    depth_info_sub_2 = nh.subscribe(depthInfoTopic_2, 1, &ImageSplitter::depthInfoCb_2, this);

    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(cameraInfoOutTopic, 1);
    camera_info_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(cameraInfoOutTopic_2, 1);
    depth_info_pub = nh.advertise<sensor_msgs::CameraInfo>(depthInfoOutTopic, 1);
    depth_info_pub_2 = nh.advertise<sensor_msgs::CameraInfo>(depthInfoOutTopic_2, 1);
    

    // Create bool subscribers
    sub_all = nh.subscribe(allCamerasTopic, 1, &ImageSplitter::publishAllCallback, this);
    sub_reverse = nh.subscribe(reverseTopic, 1, &ImageSplitter::reverseCallback, this);
    if (!only_depth)
      sub_depth = nh.subscribe(publishDepthTopic, 1, &ImageSplitter::publishDepthCallback, this);
    
    // Create RGB subscribers and pubs
    if (!only_depth) {
      sub = it.subscribe(imageTopic, 1, &ImageSplitter::imageCb, this); 
      sub_2 = it.subscribe(imageTopic_2, 1, &ImageSplitter::imageCb_2, this); 
      pub = it.advertise(imageOutTopic, 1);
      pub_2 = it.advertise(imageOutTopic_2, 1);
    }
    // Create depth sub and pub
    if (use_depth || only_depth) 
    {
      depth_sub = it.subscribe(depthTopic, 1, &ImageSplitter::depthCb, this);
      depth_sub_2 = it.subscribe(depthTopic_2, 1, &ImageSplitter::depthCb_2, this);
      depth_pub = it.advertise(depthOutTopic, 1);
      depth_pub_2 = it.advertise(depthOutTopic_2, 1);
    }
      
    imgCount = 0;
    depthCount = 0;
    imgCount_2 = 0;
    depthCount_2 = 0;
  }
  
  ~ImageSplitter(void)
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
    
    if (flip_1)
      cv::flip(dst, dst, -1);
    
    sensor_msgs::ImagePtr pt = img_cv->toImageMsg();
    if (reverse && publish_all)  {
      pub_2.publish(pt);
      camera_info_pub_2.publish(camera_info_1);
    }
    if (!reverse) {
      pub.publish(pt);
      camera_info_pub.publish(camera_info_1);
    }
  }
  
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
      
    depthCount++;
    if(depthCount < frameSkip)
    {
      return;
    }
    depthCount = 0;
    if (publish_depth || only_depth)
    {
      cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg, "32FC1");
      
      double scale = this->scale_depth;
      
      cv::Mat dst(msg->width * scale,msg->height * scale,img_cv->image.type());
      cv::resize(img_cv->image, dst, cv::Size(0,0), scale, scale);
      img_cv->image = dst;
    
      sensor_msgs::ImagePtr pt = img_cv->toImageMsg();
    
      
      if (reverse && publish_all) {
      	depth_pub_2.publish(pt);
        depth_info_pub_2.publish(depth_info_2);
      }
      if (!reverse) {
      	depth_pub.publish(pt);
        depth_info_pub.publish(depth_info_1);
      }
    }
  }
 
  void imageCb_2(const sensor_msgs::ImageConstPtr& msg)
  {
      
    imgCount_2++;
    if(imgCount_2 < frameSkip)
    {
      return;
    }
    imgCount_2 = 0;
    
    cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat dst(msg->width * scale,msg->height * scale,img_cv->image.type());
    cv::resize(img_cv->image, dst, cv::Size(0,0), scale, scale);
    
    if (flip_2)
      cv::flip(dst, dst, -1);
    
    img_cv->image = dst;
    
    
      
     
    sensor_msgs::ImagePtr pt = img_cv->toImageMsg();
    
    if (reverse) {
      pub.publish(pt);
      camera_info_pub.publish(camera_info_2);
    } else if (publish_all) {
      camera_info_pub_2.publish(camera_info_2);
      pub_2.publish(pt);
    }
  }
  
  void depthCb_2(const sensor_msgs::ImageConstPtr& msg)
  {
    depthCount_2++;
    if(depthCount_2 < frameSkip)
    {
      return;
    }
    
    depthCount_2 = 0;
    
    if (publish_depth || only_depth)
    {
      cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg, "32FC1");
      
      double scale = this->scale_depth;
      
      cv::Mat dst(msg->width * scale,msg->height * scale,img_cv->image.type());
      cv::resize(img_cv->image, dst, cv::Size(0,0), scale, scale);
      img_cv->image = dst;
    
      sensor_msgs::ImagePtr pt = img_cv->toImageMsg();
    
      if (reverse) {
        depth_pub.publish(pt);
        depth_info_pub_2.publish(depth_info_2);
      } else if (publish_all) {
        depth_pub_2.publish(pt);
        depth_info_pub.publish(depth_info_2);
      }
    }
  }
  
  void reverseCallback(const std_msgs::BoolConstPtr &msg)
  {
//     ROS_INFO("Changing the cameras. New value = %d", msg->data);
    reverse = msg->data;
  }
  
  void publishAllCallback(const std_msgs::BoolConstPtr &msg)
  {
    publish_all = msg->data;
  }
  
  void publishDepthCallback(const std_msgs::BoolConstPtr &msg)
  {
    publish_depth = msg->data;
  }


  void infoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info_1 = *msg;

    for (int i = 0; i < camera_info_1.K.size(); i++) {
      if (fabs(camera_info_1.K[i] - 1.0) > 1e-6 ) {
        camera_info_1.K[i] *= scale;
      }
    }
    for (int i = 0; i < camera_info_1.P.size(); i++) {
      if (fabs(camera_info_1.P[i] - 1.0) > 1e-6 ) {
        camera_info_1.P[i] *= scale;
      }
    }
    camera_info_sub.shutdown();
  }


  void depthInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    depth_info_1 = *msg;

    for (int i = 0; i < depth_info_1.K.size(); i++) {
      if (fabs(depth_info_1.K[i] - 1.0) > 1e-6 ) {
        depth_info_1.K[i] *= scale;
      }
    }
    for (int i = 0; i < depth_info_1.P.size(); i++) {
      if (fabs(depth_info_1.P[i] - 1.0) > 1e-6 ) {
        depth_info_1.P[i] *= scale;
      }
    }
    depth_info_sub.shutdown();
  }

  void infoCb_2(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info_2 = *msg;

    for (int i = 0; i < camera_info_2.K.size(); i++) {
      if (fabs(camera_info_2.K[i] - 1.0) > 1e-6 ) {
        camera_info_2.K[i] *= scale;
      }
    }
    for (int i = 0; i < camera_info_2.P.size(); i++) {
      if (fabs(camera_info_2.P[i] - 1.0) > 1e-6 ) {
        camera_info_2.P[i] *= scale;
      }
    }
    camera_info_sub_2.shutdown();
  }


  void depthInfoCb_2(const sensor_msgs::CameraInfoConstPtr& msg) {
    depth_info_2 = *msg;

    for (int i = 0; i < depth_info_2.K.size(); i++) {
      if (fabs(depth_info_2.K[i] - 1.0) > 1e-6 ) {
        depth_info_2.K[i] *= scale;
      }
    }
    for (int i = 0; i < depth_info_2.P.size(); i++) {
      if (fabs(depth_info_2.P[i] - 1.0) > 1e-6 ) {
        depth_info_2.P[i] *= scale;
      }
    }
    // Subscribe only once
    depth_info_sub_2.shutdown();
  }

protected:

  // ROS handler and subscribers
  ros::NodeHandle nh;
  image_transport::ImageTransport it; 
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  image_transport::Publisher depth_pub;
  image_transport::Subscriber depth_sub;
  image_transport::Publisher pub_2;
  image_transport::Subscriber sub_2;
  image_transport::Publisher depth_pub_2;
  image_transport::Subscriber depth_sub_2;
  ros::Subscriber sub_reverse, sub_depth, sub_all;
  ros::Subscriber camera_info_sub, camera_info_sub_2, depth_info_sub, depth_info_sub_2;

  ros::Publisher camera_info_pub, camera_info_pub_2, depth_info_pub, depth_info_pub_2;
  
  // Topics 
  std::string cameraName, imageTopic, imageOutTopic, cameraInfoTopic, cameraInfoOutTopic;
  std::string depthTopic, depthOutTopic, depthInfoTopic, depthInfoOutTopic;
  std::string cameraName_2, imageTopic_2, imageOutTopic_2, cameraInfoTopic_2, cameraInfoOutTopic_2;
  std::string depthTopic_2, depthOutTopic_2, depthInfoTopic_2, depthInfoOutTopic_2;
  std::string publishDepthTopic, allCamerasTopic, reverseTopic;
  
  int frameSkip;
  int imgCount, depthCount;
  int imgCount_2, depthCount_2;
  double scale, scale_depth; // Scaling in the image
  bool publish_depth, use_depth, reverse, publish_all, only_depth;
  bool flip_1, flip_2;
  sensor_msgs::CameraInfo camera_info_1, depth_info_1;
  sensor_msgs::CameraInfo camera_info_2, depth_info_2;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_splitter");  
  ImageSplitter node;
  ros::spin();
}
