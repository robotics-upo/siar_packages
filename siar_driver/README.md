# siar_driver
This module acts as an interface between the SIAR control board and other ROS modules. With this module, standard command velocity messages can be sent to the robot and also odometry is provided. 

The package is composed by three nodes:

* *siar_node* is a driver for controlling the robot by using ROS.
* *siar_teleop_joy* is a program for teleoperating the robot with a joystick by using ROS (*optional feature*).
* *siar_calibration_node* a simple node for calibrating the odometry estimation and commands of SIAR. Use: rosrun siar_driver siar_calibration_node \<type of test\> \<total time\> \<raw velocity command\> The program accepts joystick commands before starting the test (a joy node should be running).

The raposa_teleop_joy program is based on the following tutorial: 
*Writing a Teleoperation Node for a Linux-Supported Joystick* 
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode 
under license http://creativecommons.org/licenses/by/3.0/


- See http://www.idmind.pt for more information about the ID-Mind company. 
- See http://www.ros.org for more information about the ROS framework.

## General requirements

* Siar platform from IdMind.
  
* Computer to be located on the platform running Ubuntu Linux 14.04 and ROS Indigo. It has to be connected via USB to the SIAR control board.

* An ArduIMU v3 IMU device. Note that this package depends on the package arduimu_v3.

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

Parameters of the *siar_driver* program:

* **siar_device_1**: device of the control board (i.e. /dev/ttyUSB0). It is advised to use the id of the device: /dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTB3LEN7-if00-port0

* **siar_device_2**: Second device of the control board, when using the configuration with two control boards (i.e. /dev/ttyUSB0). It is advised to use the id of the device: /dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTB3LEN7-if00-port0

* **joy_device**: Device of the IdMind controller

* **base_frame_id**: base_frame identifier

* **odom_frame_id**: odom_frame identifier

* **freq**: Frequency in hertzs of the main loop.

* **freq_imu**: Work frequency of the ArduIMU, if present

* **use_imu**: 1 if using IMU, 0 otherwise (angular velocity will be calculated by using the motor encoders)

* **publish_tf**: if true, it publishes the tf related to the odometry

Parameters of the *siar_teleop_joy* program:

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
