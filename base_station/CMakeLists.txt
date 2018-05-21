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
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Qt4 REQUIRED QtCore QtGui)
include(${QT_USE_FILE})
SET ( QWT_INCLUDE /usr/include/qwt/)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base_station
 CATKIN_DEPENDS rosbag roscpp sensor_msgs std_msgs compressed_depth_image_transport cv_bridge functions
 DEPENDS boost opencv Qt 
)

###########
## Build ##
###########

# include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${QWT_INCLUDE})
# 
#  SET( BASE_STATION_UIS
#        ./ui/base_station.ui
#   )
# 
# ## QT stuff
# QT4_WRAP_UI( GUI_HDRS ${ARCAS_UIS} )
# QT4_WRAP_CPP( BASE_STATION_MOC_SRCS src/ArcasVisualizer.h src/Comms.h) # NOTE: Beware on including the files here into the add executable. If not: vtable errors
# SET (BASE_SRCS src/ArcasVisualizer.cpp src/Comms.cpp)
# 
# ## Base station exec
# add_executable(base_station src/base_station.cpp ${GUI_HDRS} ${ARCAS_SRCS} ${ARCAS_MOC_SRCS})
# target_link_libraries(arcas_visualizer ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTTEST_LIBRARY} ${catkin_LIBRARIES})
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
