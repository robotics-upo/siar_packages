project(sewer_graph)
cmake_minimum_required(VERSION 2.6)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp functions visualization_msgs roscpp geometry_msgs simple_graph)
# find_package(Boost REQUIRED)
# find_package(kml REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES sewer_graph
 CATKIN_DEPENDS roscpp functions visualization_msgs roscpp geometry_msgs simple_graph
 DEPENDS Boost kml
)

include_directories(include ${catkin_INCLUDE_DIRS})

SET(FP_SOURCE src/earthlocation.cpp src/general.cpp src/sewer_graph.cpp) 
SET(LIBS ${catkin_LIBRARIES} kmlbase kmldom kmlengine kmlconvenience dxflib)

add_library(sewer_graph ${FP_SOURCE}
)
target_link_libraries(sewer_graph ${LIBS})


# Test Earth Location
 add_executable(test_earth_location src/exec/test_earth_location.cpp)
 target_link_libraries(test_earth_location ${PROJECT_NAME} ${LIBS})
 
# Test sewer graph
add_executable(test_sewer_graph src/exec/test_sewer_graph.cpp)
target_link_libraries(test_sewer_graph ${PROJECT_NAME} ${LIBS})

add_executable(dxf_to_sewer_graph src/exec/dxf_to_sewer_graph.cpp src/test_creationclass.cpp)
target_link_libraries(dxf_to_sewer_graph ${PROJECT_NAME} ${LIBS})

add_executable(get_results src/exec/get_results.cpp)
target_link_libraries(get_results ${PROJECT_NAME} ${LIBS})

 ## Mark executables and/or libraries for installation
install(TARGETS sewer_graph
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
