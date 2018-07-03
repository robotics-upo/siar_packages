cmake_minimum_required(VERSION 2.8.3)
project(base_station)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  compressed_depth_image_transport
  cv_bridge
  image_transport
  functions
  rviz
  siar_driver
)

set(qt_gui_cpp_USE_QT_MAJOR_VERSION 5 )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Ogre REQUIRED)

# System dependencies are found with CMake's conventions
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui QtWebKit REQUIRED)
  include(${QT_USE_FILE})
  endif()

SET ( QWT_INCLUDE /usr/include/qwt/)

# roslint_cpp()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base_station
 CATKIN_DEPENDS rosbag roscpp sensor_msgs std_msgs compressed_depth_image_transport cv_bridge functions rviz
 DEPENDS boost opencv Qt 
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${QWT_INCLUDE})
#
SET( siar_gui_UIS
       ./ui/siar_gui.ui
  )
  
  set (siar_gui_SRCS
  src/BaseStation.cpp
  src/Comms.cpp
  src/CameraSettings.cpp
)

set(siar_gui_HDRS
  src/BaseStation.h
  src/Comms.h
  src/CameraSettings.cpp
)


if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(siar_gui_MOCS ${siar_gui_HDRS})
  qt5_wrap_ui(siar_gui_UIS_H ${siar_gui_UIS})
else()
  qt4_wrap_cpp(siar_gui_MOCS ${siar_gui_HDRS})
  qt4_wrap_ui(siar_gui_UIS_H ${siar_gui_UIS})
endif()

 
# 
SET (BASE_SRCS src/Comms.cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${QWT_INCLUDE}
  ${OGRE_INCLUDE_DIRS}
)

# 
# message(${catkin_LIBRARIES})

# 
# ## Base station exec
add_executable(${PROJECT_NAME} src/base_station.cpp ${BASE_SRCS} ${siar_gui_HDRS} ${siar_gui_SRCS} ${siar_gui_MOCS} ${siar_gui_UIS_H})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
)

add_dependencies(${PROJECT_NAME} siar_driver_generate_messages_cpp)

add_executable(test_comm src/testComm.cpp)
target_link_libraries(test_comm ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets qwt-qt5)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTWEBKIT_LIBRARY} qwt)
endif()
# 
# 
# ## Additional software
# add_executable(get_images_labeling src/get_images_labeling.cpp)
# 
# target_link_libraries(get_images_labeling
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OpenCV_LIBRARIES}
# )
# 


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS base_station base_station_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_base_station.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
