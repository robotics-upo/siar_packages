cmake_minimum_required(VERSION 2.8.3)
project(udp_bridge)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  siar_driver
  std_msgs
  rssi_get
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES udp_bridge
 CATKIN_DEPENDS dynamic_reconfigure image_transport nav_msgs roscpp rospy rssi_get sensor_msgs siar_driver std_msgs tf 
 DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(udp_server src/udp_server_node.cpp)
add_executable(udp_client src/udp_client_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(udp_server ${siar_driver_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(udp_client ${siar_driver_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(udp_server
   ${catkin_LIBRARIES}
)
target_link_libraries(udp_client
   ${catkin_LIBRARIES}
)
