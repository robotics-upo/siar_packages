#ifndef JOY_TRANSLATOR_HPP
#define JOY_TRANSLATOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class JoyTranslator {
public:
  JoyTranslator(ros::NodeHandle &nh, ros::NodeHandle &lnh) {
    lnh.getParam("new_buttons", new_buttons);
    lnh.getParam("new_axes", new_axes);
    
    std::cout << "New buttons vector size: "<< new_buttons.size();
    std::cout << "   New axesvector size: " << new_axes.size() << std::endl;
    
    lnh.getParam("new_button_size", new_button_size);
    lnh.getParam("new_axes_size", new_axes_size);
    joy_trans_service = false;
    lnh.getParam("joy_trans_service", joy_trans_service);
    if (joy_trans_service) {
      joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 2, &JoyTranslator::joyCb, this);
      joy_pub = nh.advertise<sensor_msgs::Joy>("/translated_joy", 1);
      
    }
  }
  
  const sensor_msgs::Joy translateJoy(const sensor_msgs::Joy &msg) {
    sensor_msgs::Joy ret = msg;
    
    ret.buttons.resize(new_button_size);
    ret.axes.resize(new_axes_size);
    
    for (size_t i = 0; i < msg.buttons.size() && i < new_buttons.size(); i++) {
      if (ret.buttons.size() > new_buttons[i] && new_buttons[i] >= 0) 
        ret.buttons[new_buttons[i]] = msg.buttons[i];
    }
    
    for (size_t i = 0; i < msg.axes.size() && i < new_axes.size(); i++) {
      if (ret.axes.size() > new_axes[i] && new_axes[i] >= 0) 
        ret.axes[new_axes[i]] = msg.axes[i];
    }
    
    return ret;
  }
  
protected: 
   // Joy buttons translation
  std::vector<int> new_buttons, new_axes;
  int new_button_size, new_axes_size;
  bool joy_trans_service;
  
  ros::Publisher joy_pub;
  ros::Subscriber joy_sub;
  
  void joyCb(const sensor_msgs::JoyConstPtr &joy) {
    joy_pub.publish(translateJoy(*joy));
  }
  
};


#endif
