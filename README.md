# siar_packages

This metapackage compiles developments that have been made for the SIAR project, which belongs to the Echord++ FP7 European project in the Urban Robotics Challenge "Sewer Inspection".

It also includes the localization system is presented in the paper "RGBD-based Robot Localization in Sewer Networks" of D. Alejo, F. Caballero and L. Merino that has been submitted to 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems. For details regarding to the packages and the execution of the presented experiments, please refer to the README.md file in the "/localization_siar" subdirectory.

This metapackage is composed by the following packages. For more details about them, please refere to their included README.md files:

* *frame_dropper* Contains two utilities: "frame_dropper" and "image_splitter" for replicating image flow that could be used for transmitting images over the network.
* *localization_siar* A metapackage that groups all the packages necessary for the localization in the SIAR platform. Please refer to its internal README.md for more details.
* *rssi_get* Contains a utility for publishing the perceived RSSI of nVIP2400 equipments
* *siar_driver* A driver for controlling the SIAR platform
* *udp_bridge* A bridge module to transmit data over a UDP link

## Compilation

Before compiling, please check the dependencies

In order to build the package, clone it inside the *src* directory of your Catkin workspace and compile it by using *catkin_make* as usual.

## Dependencies

- libkml-1.2.0 (or higher)
- Rgbd-odom package. Can be downloaded from: https://github.com/robotics-upo/rgbd_odom.git
