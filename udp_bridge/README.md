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
