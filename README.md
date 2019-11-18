# siar_packages

This metapackage compiles developments that have been made for the SIAR project, which belongs to the Echord++ FP7 European project in the Urban Robotics Challenge "Sewer Inspection".

This metapackage is composed by the following packages. For more details about them, please refere to their included README.md files:

* *frame_dropper* Contains two utilities: "frame_dropper" and "image_splitter" for replicating image flow that could be used for transmitting images over the network.
* *rssi_get* Contains a utility for publishing the perceived RSSI of nVIP2400 equipments
* *siar_arm* A driver for high-level control of the manipulator onboard SIAR 
* *siar_driver* A driver for controlling the SIAR platform
* *siar_rqt_plugin* Provides a 3D representation of the robot in RViz
* *udp_bridge* A bridge module to transmit data over a UDP link

## Compilation

Make sure that you have created a workspace and have configured the proper environment variables with the "setup.bash" script (see http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Recommended steps:

 > roscd 
 
 > cd ../src
  
 > git clone https://github.com/robotics-upo/arduimu_v3.git
 
 > git clone https://github.com/robotics-upo/functions.git
 
 > git clone https://github.com/robotics-upo/siar_packages.git
 
 > cd ..
 
 > catkin_make
