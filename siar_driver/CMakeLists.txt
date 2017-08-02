cmake_minimum_required(VERSION 2.8.3)
project(siar_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  arduimu_v3
  kdl_parser
  message_generation
  std_msgs
  nav_msgs
  robot_state_publisher
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SiarBatteryMonitor.msg
  SiarPowerSupply.msg
  SiarStatus.msg 
  SiarArmCommand.msg
  SiarLightCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES siar_driver
 CATKIN_DEPENDS roscpp rospy arduimu_v3 kdl_parser std_msgs message_generation message_runtime nav_msgs robot_state_publisher sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(siar_node src/siar_node.cpp)
add_executable(siar_calibration_node src/siar_calibration_node.cpp)
add_executable(siar_teleop_joy src/siar_teleop_joy.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(siar_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(siar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(siar_teleop_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(siar_node
  ${catkin_LIBRARIES}
)
target_link_libraries(siar_calibration_node
  ${catkin_LIBRARIES}
)

target_link_libraries(siar_teleop_joy ${catkin_LIBRARIES})
