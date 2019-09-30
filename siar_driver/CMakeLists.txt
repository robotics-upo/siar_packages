cmake_minimum_required(VERSION 2.8.3)
project(siar_driver)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  arduimu_v3
  functions
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

# find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
# add_action_files(DIRECTORY action FILES DoDishes.action)
# generate_messages(DEPENDENCIES actionlib_msgs)

###################################
## catkin specific configuration ##
###################################
#TODO: the dependence with functions is just for the arm tester
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES siar_driver
 CATKIN_DEPENDS roscpp rospy arduimu_v3 functions kdl_parser std_msgs message_generation message_runtime nav_msgs robot_state_publisher sensor_msgs tf
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
add_executable(siar_raw_motors src/siar_raw_motors.cpp)
add_executable(siar_teleop_joy src/siar_teleop_joy.cpp)
#add_executable(mbzirc_teleop_joy src/mbzirc_teleop_joy.cpp)
add_executable(siar_teleop_joy_simulation src/siar_teleop_joy_simulation.cpp)
add_executable(arm_tester src/arm_tester.cpp)
add_executable(arm_saver src/arm_saver.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(siar_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(siar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(siar_raw_motors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(siar_teleop_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
#add_dependencies(mbzirc_teleop_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(siar_teleop_joy_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(arm_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)
add_dependencies(arm_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} siar_driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(siar_node
  ${catkin_LIBRARIES}
)
target_link_libraries(siar_calibration_node
  ${catkin_LIBRARIES}
)
target_link_libraries(siar_raw_motors ${catkin_LIBRARIES})
target_link_libraries(siar_teleop_joy ${catkin_LIBRARIES})
#target_link_libraries(mbzirc_teleop_joy ${catkin_LIBRARIES})
target_link_libraries(siar_teleop_joy_simulation ${catkin_LIBRARIES})
target_link_libraries(arm_tester ${catkin_LIBRARIES})
target_link_libraries(arm_saver ${catkin_LIBRARIES})
