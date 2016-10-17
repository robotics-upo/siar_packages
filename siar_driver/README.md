# siar_packages

This metapackage compiles developments that have been made for the SIAR project, which belongs to the Echord++ FP7 European project in the Urban Robotics Challenge "Sewer Inspection".

It is composed by the following pacakges:

* *frame_dropper* Contains two utilities: "frame_dropper" and "image_splitter" for replicating image flow that could be used for transmitting images over the network.
* *rssi_get* Contains a utility for publishing the perceived RSSI of nVIP2400 equipments
* *siar_driver* A driver for controlling the SIAR platform
* *udp_bridge* A bridge module to transmit data over a UDP link

## Compilation
In order to build the package, clone it inside the *src* directory of your Catkin workspace and compile it by using *catkin_make* as usual.


# frame_dropper

This module has been included for easily replicating image flows, so they can be configured for image transmission over a data link. 

Note that the utilities assume 32FC1 format when publishing depth data.

Two utilities have been included:

## frame_dropper

This node gets an image from the topic "/in/rgb/image_raw" and republishes it rescaled and with frame dropping options. Parameters:

### ROS Parameters

* *in*: Uses resolvename to ask for the name of the input image flow
* *out*: Uses resolvename to ask for the name of the output image flow
* *frame_skip*: Number of frames to be received before publishing one image in the output flow
* *scale*: Scale (1 means no scaling)
* *publish_depth*: Also duplicates the depth image flow from /in/depth_registered/image_raw

### Published topics

* *out/rgb/image_raw* Image transport publisher with the output flow

### Subscribed topics

* *out/rgb/image_raw* Image transport subscriber to the input flow


## image_splitter

This node extends the functionality of frame_dropper by taking two different input image flows, that could be interchanged online by publishing in the "reverse" topic.

### ROS Parameters

* *in_1*: Uses resolvename to ask for the name of the first input image flow
* *out_1*: Uses resolvename to ask for the name of the first output image flow
* *in_2*: Uses resolvename to ask for the name of the second input image flow
* *out_2*: Uses resolvename to ask for the name of the second output image flow
* *frame_skip*: Number of frames to be received before publishing one image in the output flow
* *scale*: Scale (1 means no scaling)
* *publish_depth*: Also duplicates the depth image flow from /in/depth_registered/image_raw
* *use_depth*: If false, the depth image transport publisher is not instantiated
* *publish_all*: Publishes both flows

### Published topics

* *out/rgb/image_raw* Image transport publisher with the output flow

### Subscribed topics

* *out/rgb/image_raw* Image transport subscriber to the input flow
* */reverse* Out_1 takes the images from In_2 if /reverse is true. If false, normal flow
* */publish_depth* To active or deactivate depth publishing online
* */publish_all* Publishes both flows (in reverse order if publish depth is activated)

# rssi_get

Uses a scraping technique to obtain the RSSI data and other configuration from the nVIP2400 WiFi equipment

# siar_driver
This module acts as an interface between the SIAR control board and other ROS modules. With this module, standard command velocity messages can be sent to the robot and also odometry is provided. 

The package is composed by three programs:

* *siar_driver* is a driver for controlling the robot by using ROS.
* *siar_teleop_joy* is a program for teleoperating the robot with a joystick by using ROS (*optional feature*).
* *siar_calibration_node* a simple node for calibrating the odometry estimation and commands of SIAR. Use: rosrun siar_driver siar_calibration_node <type of test> <total time> <raw velocity command> The program accepts joystick commands before starting the test (a joy node should be running).

The raposa_teleop_joy program is based on the following tutorial: 
*Writing a Teleoperation Node for a Linux-Supported Joystick* 
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode 
under license http://creativecommons.org/licenses/by/3.0/


- See http://www.idmind.pt for more information about the ID-Mind company. 
- See http://www.ros.org for more information about the ROS framework.

## General requirements

* Siar platform from IdMind.
  
* Computer to be located on the platform running Ubuntu Linux 14.04 and ROS Indigo. It has to be connected via USB to the SIAR control board.

* An ArduIMU v3 IMU device. Note that this package depends on the package arduimu_v3, which can be retrieved in: https://github.com/robotics-upo/arduimu_v3.git.

* (*Optional*): A wireless joystick or gamepad with at least 8 buttons and 1 axis compatible with ROS . The system has been tested with the *Logitech Wireless F710* gamepad. 

* (*Optional*): An id-mind controller (Frog style) conected via 900 MHz modem

## Easy start

This package has a simple "start.launch" file which can be used to easily execute all the required ROS nodes for teleoperating the SIAR platform with a joystick controller. By default, the imu is disabled. Please refer to the launch file for different options.


## How to command the robot by using ROS

The driver accepts motion commands from the next ROS topics:

* **/cmd_vel** of type **geometry_msgs::Twist** in order to get instant angular and linear velocities.

## Required ROS topics

The next topic is required in order to run the *siar_teleop_joy* and the *siar_calibration_node* programs:

* **/joy** of type **sensor_msgs::Joy** in order get the input of the joystick/gamepad (note that a joy_node from the ROS joy package should be run in order to access the joy and publish the joystick commands.


## Published ROS topics

The next topics are published by the *siar_driver*: 

* **/odom** of type **nav_msgs::Odometry** --> Odometry estimation

* **/arm_pos** of type **std_msgs::Int16** --> Readed position of the arm

* **/tf**. If publish_tf parameter is true --> the odometry estimation is published as a tf between the frames body and odom (see parameters)

The next topics are published by the *siar_teleop_joy*:

* **/cmd_vel** of type **geometry_msgs::Twist** in order to command the robot by reading the status of the joystick.

* **/slow_motion** of type **std_msgs::Bool** indicates whether the slow mode is ON or OFF

* **/reverse** of type **std_msgs::Bool** to indicate a change in reverse mode

## ROS parameters

Parameters of the *teresa_driver* program:

* **siar_device_1**: device of the control board (i.e. /dev/ttyUSB0). It is advised to use the id of the device: /dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTB3LEN7-if00-port0

* **siar_device_2**: Second device of the control board, when using the configuration with two control boards (i.e. /dev/ttyUSB0). It is advised to use the id of the device: /dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTB3LEN7-if00-port0

* **joy_device**: Device of the IdMind controller

* **base_frame_id**: base_frame identifier

* **odom_frame_id**: odom_frame identifier

* **freq**: Frequency in hertzs of the main loop.

* **freq_imu**: Work frequency of the ArduIMU, if present

* **use_imu**: 1 if using IMU, 0 otherwise (angular velocity will be calculated by using the motor encoders)

* **publish_tf**: if true, it publishes the tf related to the odometry

Parameters of the *teresa_teleop_joy* program:

* **freq**: Frequency in hertzs of the main loop.

* **panic_freq**: Frequency in hertzs of the main loop in case of pushing the panic button in the joystick.

* **linear_velocity_axis**: Id of the axis to control the linear velocity.

* **angular_velocity_axis**: Id of the axis to control the angular velocity.

* **max_velocity_button**: Id of the button to allow the maximun velocity.

* **panic_button**: Id of the panic button in the joystick.

* **slow_button**: Id of the button to switch to slow mode ON/OFF

* **turbo_button**: Id of the button to enable full speed (only when pressed)

* **back_button**: Id of the BACK button. ** If this button is pressed, the execution of all ROS nodes is halted  if stop_on_exit parameter is true**

* **stop_on_exit**: If true, all ROS nodes are killed when the back button is pressed (useful for stopping ROSbag).

* **max_linear_velocity**: Maximun allowed linear velocity in m/s.

* **max_angular_velocity**: Maximun allowd angular velocity in rad/s.

* **max_joy_time** If no joystick command is received within this time --> stop SIAR. Useful when transmitting joy commands over the network

# udp_bridge

A bridge module to transmit SIAR related data over a UDP link. It contains two nodes: udp_server should be runned in the PC onboard the SIAR and udp_client in the base station.

Note that two independents cores should be running in both PCs (base station and remote) and bridge will link them.

Note that this package is still under development, and if the client goes down, it is possible that the server freezes, so it should be also restarted.

## udp_server

It is the node that has to be executed in the SIAR PC. The subscribed topics will be transmitted over UDP at a determinate maximum rate. 
The published topics will publish whenever new data associated to this topic is received over UDP.

### Params

 * *odomTopic* (/odom) The odometry topic
 * *odomRate* (5.0) Maximum rate in Hertzs of the odometry
 * *joy_topic* (/joy) The joy topic
 * *camera_1* (/front) Name of the primary camera
 * *camera_2* (/back) Name of the secondary camera
 * *image_rate* (5.0) Maximum rate of the images
 * *depth_rate* (2.0) Maximum rate of the depth images (should be lower than the image rate)
 * *jpeg_quality* (60) JPEG quality of the flows of images of cameras 1 and 2 (NOTE: it is convenient to duplicate the original flows, so they are not altered by this program)
 * *port* (16000); The UDP port to be used
 * *rssi_topic* ("/rssi_nvip_2400") The topic that contains the RSSI info and other link information
 * *rssi_rate* (1.0) Maximum rate of the RSSI 
 * *motor_battery_topic* ("/motor_battery_status") Name of the first battery status topic
 * *elec_battery_topic* ("/elec_battery_status") Second battery status topic
 * *batteryRate* (0.2) Maximum rate for the battery status
 * *point_topic* ("/rgbd_odom_node/point_cloud") Experimental: transmit point clouds (not tested)
 * *point_rate* (1.0) Experimental: point cloud rate

### Published topics

 * */publish_depth_pub* Information to activate or deactivate depth publication from the base station
 * */publish_all* Information to activate or deactivate simultaneous image emision from two cameras
 * */slow* Activate or deactivate SIAR slow mode from the base station
 * */joy* Joystick commands 
 
### Subscribed topics

 * */camera_1/rgb/image_raw* Image transport subscriber 1
 * */camera_1/depth_registered/image_raw* Image transport subscriber 1b. Depth images
 * */camera_2/rgb/image_raw* Image transport subscriber 2
 * */camera_2/depth_registered/image_raw* Image transport subscriber 2b. Depth images
 * */motor_battery_status* Motor battery status
 * */elec_battery_status* Electronic battery status
 * */rssi_nvip_2400* nVIP link status
 * */rbgd_odom_node/point_cloud* Point cloud subscriber

## udp_client

This node should be used in the base station.

The subscribed and published topics are complementary to the udp_server, so they will not be listed below. Only extra publishers are listed.

### Params

 * *ip_address* (192.168.168.11) IP where the server is running (note that the server should be running before starting the program)
 * *odomTopic* (/odom) The odometry topic
 * *joy_topic* (/joy) The joy topic
 * *camera_1* (/front) Name of the primary camera
 * *camera_2* (/back) Name of the secondary camera
 * *port* (16000); The UDP port to be used
 * *rssi_topic* ("/rssi_nvip_2400") The topic that contains the RSSI info and other link information
 * *motor_battery_topic* ("/motor_battery_status") Name of the first battery status topic
 * *elec_battery_topic* ("/elec_battery_status") Second battery status topic
 * *point_topic* ("/rgbd_odom_node/point_cloud") Experimental: transmit point clouds (not tested)
 
### Extra Published topics

 * */camera_1/rgb/camera_info* Camera info topic associated to the camera 1 image flow (actual values are hardcoded)
 * */camera_1/depth_registered/camera_info* Camera info topic associated to the camera 1 depth image flow (actual values are hardcoded)
 * */camera_2/rgb/camera_info* Camera info topic associated to the camera 2 rgb image flow (actual values are hardcoded)
 * */camera_2/depth_registered/camera_info* Camera info topic associated to the camera 2 depth image flow (actual values are hardcoded)