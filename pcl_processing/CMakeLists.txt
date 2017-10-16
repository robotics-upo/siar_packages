cmake_minimum_required(VERSION 2.8.3)
project(pcl_processing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  depth_image_proc
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  tf2
)



catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES pcl_processing
 CATKIN_DEPENDS depth_image_proc image_transport pcl_ros roscpp rospy sensor_msgs tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#find_package(libpointmatcher REQUIRED PATHS ${LIBPOINTMATCHER_INSTALL_DIR})
#include(libnaboConfig)
#include_directories(${libpointmatcher_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} )

## Declare a C++ executable
add_executable(point_cloud_subscriber src/point_cloud_subscriber.cpp src/processPointCloud.cpp)

add_executable(accumulate_cloud src/accumulateCloud.cpp src/processPointCloud.cpp)

add_executable(point_cloud_save_ref src/point_cloud_save_ref.cpp src/processPointCloud.cpp)

add_executable(point_cloud_clean_ref src/point_cloud_clean_ref.cpp src/processPointCloud.cpp)

add_executable(point_cloud_compare src/point_cloud_compare.cpp src/processPointCloud.cpp)

add_executable(map_analysis src/map_analysis.cpp src/siar_map_analysis.cpp src/siar_section.cpp src/processPointCloud.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pcl_processing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(point_cloud_subscriber
  ${catkin_LIBRARIES}
)

target_link_libraries(accumulate_cloud
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_save_ref
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_clean_ref
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_compare
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(map_analysis
  ${catkin_LIBRARIES}
)


