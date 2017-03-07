# localization_siar

This metapackage contains most of the development for localization of the SIAR platform inside a sewer network. 

This localization system is presented in the paper "RGBD-based Robot Localization in Sewer Networks" of D. Alejo, F. Caballero and L. Merino that
has been submitted to 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems.

The bags for


It is composed by the following pacakges:

* *amcl_sewer* A customized version of the popular AMCL algorithm adapted to the particularities of the sewer network.
* *functions* Some general-purpose functions and classes regarding to file management, handling with RealVectors and many more
* *manhole_detector* A keras-based CNN detector trained with over 19k images for detecting manholes with the up-facing camera of the robot
* *sewer_graph* Uses the simple_graph library to acquire the sewer network topology from file and store it in memory
* *simple_graph* A lightweight library for storing sparse graphs

## Compilation

Before compiling, please check the dependencies

In order to build the package, clone it inside the *src* directory of your Catkin workspace and compile it by using *catkin_make* as usual.

## Dependencies

- libkml-1.2.0 (or higher)
- libxml++-1.0 (or higher) or any version. Sometime it is necessary to make a symbolic link in /usr/include/libkml++ to the folder /usr/include/libkml++-1.0/libkml++

# amcl_sewer

This module has been included for performing the localization of the SIAR platform while navigating through the sewer network. 

It uses Monte-Carlo based localization with two different weights. 

First, it calculates the distance of the particle with the graph (i.e. distance to closest edge). Particles that are farther 
from the graph are less likely to represent the state of the SIAR.

Second, whenever the manhole detector detects a manhole the weights are changed to the distance to the closest manhole.
In this way, we can precisely locate the SIAR platform in the sewer network.

The main node is described below.

## amcl_node

This node gets an image from the topic "/in/rgb/image_raw" and republishes it rescaled and with frame dropping options. 

The parameters are (default values given inside parentheses):

### ROS Parameters

* *base_frame_id*: (base_link) The name of the base link of the platform
* *odom_frame_id*: (odom) The reference frame of the odometry
* *global_frame_id*: (map) Global reference frame of the AMCL
* *update_rate*: (10) Update rate of the AMCL 
* *min_particles*: (300) Minimum size of the particle set
* *max_particles*: (600) Maximum size of the particle set
* *odom_x_mod*: (0.2) Odometry noise in x coord
* *odom_y_mod*: (0.2) Odometry noise in y coord
* *odom_a_mod*: (0.2) Odometry noise in yaw
* *initial_x*: (0.0) Initial position of the robot (x)
* *initial_y*: (0.0) Initial position of the robot (y)
* *initial_a*: (0.0) Initial position of the robot (yaw)
* *initial_x_dev*: (0.3) Std_dev of the initial position of the robot (x)
* *initial_y_dev*: (0.3) Std_dev of the initial position of the robot (y)
* *initial_a_dev*: (0.2) Std_dev of the initial position of the robot (yaw)
* *initialize*: (false) If true, automatically initializes the filter with the provided initial data
* *update_min_d*: (0.5) Minimum displacement for performing prediction + update
* *update_min_a*: (0.2) Minimum angular displacement for performing prediction + update
* *resample_interval*: (0) Number (-1) of updates before resampling
* *manhole_detect_topic*: (/manhole) Topic (type Bool) that is true if a manhole is detected
* *sewer_graph_file*: (...) Location of the Sewer Graph File (see example in test subdirectory of sewer_graph)
* *min_manhole_detected*: (10) Minimum number of detections before an update for changing to Manhole weighting
* *edgeDev* (1.0) Std_dev from the sewer_graph
* *forkDev* (3.0) Std_dev from the sewer_graph in non-straight sections
* *fork_dist* (3.0) If a fork node is closer than this distance from the mean position of the particle set, we consider the SIAR inside a fork
* *manholeDev* (0.6) Std_dev for the manhole weighting
* *manholeThres* (0.15) Additive weight for manhole weighting
* *stats_file* (~/manhole_stats.txt) Saves statistics whenever passing below a manhole (which has to be manually labeled)
* *traj_file* (~/traj_.txt) Saves the mean position of the particle set in after each prediction or update steps



### Published topics

* */amcl_node/particle_cloud* (geometry_msgs/PoseArray) pose of the particles
* */amcl_node/sewer_graph* (visualization_msgs/Marker) Sewer Graph Visualization
* */gps/fix* (sensor_msgs/NatSatFix) Location of the (0,0) local position in global coordinates (ENU transform between local and global)


### Subscribed topics

* *(manhole_detect_topic)* (std_msgs/Bool) Output of the manhole detector module (if true --> SIAR is below a manhole)
* */ground_truth* (std_msgs/Bool) If true we are really below a manhole (hand labeled)
* */amcl_node/initial_pose* (geometry_msgs/PoseWithCovarianceStampedConstPtr)

## Launching the experiments

In the "launch" subdirectory, two launch files are given for easilly executing the performing


# manhole_detector

This package contains scripts for CNN detecting manhole, faking the detector according to sequences of images that are below a sewer (hand labeled)
and generating data for training the CNN according to the handmade labels.

## Dependencies

It makes use of the keras python library for using neural networks for detection. Please refer to https://keras.io/ for installation. 

It requires the hdf5 and h5py for loading the trained network

Keras should be configured (go to ~/.keras) for using Theanos backend.

## cnn_detector.py

This node uses the CNN for generating a bool message that will be true whenever a manhole is detected

### Use:

 cnn_detector.py <camera name> <cnn_file>
 
 The cnn_file is saved in the test subdirectory. The camera name is usually "/up"
 
### Published topics

* */manhole* (std_msgs/Bool) Indicates whether SIAR is below a manhole or not
* */manhole_marker* (visualization_msgs/Marker) A marker that will be published if the manhole is detected

### Subscribed topics

* */{camera_name}/depth_registered/image_raw" (sensor_msgs/Image) THe input depth image that is used as input of the CNN 

 
## fake_detector.py

This node loads the handmade label file that identifies the sequence number of the images that are below a manhole and uses it to generate
a bool message indicating it.

### Use:
 fake_detector.py <camera name> <label_file>
 
### Published topics

* */manhole* (std_msgs/Bool) Indicates whether SIAR is below a manhole or not

### Subscribed topics

* */{camera_name}/depth_registered/image_raw" (sensor_msgs/Image) THe input depth image for checking its sequence number

## generate_dataset_learning

Loads a bag file and generates four data files with downsampled depth (80x60) images.:

1 - "positive_depth": All depth images that are below a manhole according to the label file
2 - "positive_rgb: All RGB images that are below a manhole according to the label file
3 - "negative_depth": All depth images that are not below a manhole according to the label file
4 - "negative_rgb": All RGB images that are not below a manhole according to the label file

### Use:

 * *generate_dataset_learning*  <bag file> <input_file> [<camera_name>] [<skip first n images>] 
 
 (camera_name defaults to "/up")
