cmake_minimum_required(VERSION 2.8.3)
project(arduimu_v3)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS eigen3
)

###########
## Build ##
###########

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}, ${EIGEN3_INCLUDE_DIR} 
)

## Declare a cpp executable
add_executable(arduimu_v3 src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(arduimu_v3
   ${catkin_LIBRARIES}
)
