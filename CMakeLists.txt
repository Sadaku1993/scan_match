cmake_minimum_required(VERSION 2.8.3)
project(scan_match)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ceres_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
  std_msgs
  tf
  time_util
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8.0 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAG "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LIKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   ceres_msgs#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   usr_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scan_match
#  CATKIN_DEPENDS ceres_msgs geometry_msgs nav_msgs pcl_ros roscpp sensor_msgs std_msgs tf time_util usr_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include	
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}") 

## Declare a C++ library
# add_library(scan_match
#   src/${PROJECT_NAME}/scan_match.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(scan_match ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(scan_match_node src/scan_match_node.cpp)
# add_executable(buffer_with_segpnt src/buffer_with_segpnt.cpp)
# add_executable(buffer src/buffer.cpp)
add_executable(buffer_geo src/buffer_geo.cpp)
# add_executable(buffer_tiny src/buffer_tiny.cpp)
# add_executable(buffer_geo_ie src/buffer_geo_ie.cpp)
add_executable(buffer_geo_ond src/buffer_geo_ond.cpp)
add_executable(buffer_sq1 src/buffer_sq1.cpp)
add_executable(buffer_sq2_color src/buffer_sq2_color.cpp)
add_executable(icp_geo_sq1 src/icp_geo_sq1.cpp)
add_executable(icp_sq2_color src/icp_sq2_color.cpp)
# add_executable(gicp_with_segpnt src/gicp_with_segpnt.cpp)
# add_executable(gicp src/gicp.cpp)
add_executable(gicp_geo src/gicp_geo.cpp)
add_executable(gicp_ond src/gicp_ond.cpp)
add_executable(gicp_geo_sq1 src/gicp_geo_sq1.cpp)
add_executable(gicp_color_sq src/gicp_color_sq.cpp)
add_executable(gicp_for_loop src/gicp_for_loop.cpp)
add_executable(gicp_for_loop_color src/gicp_for_loop_color.cpp)
add_executable(icp_for_loop src/icp_for_loop.cpp)
add_executable(icp_for_loop_color src/icp_for_loop_color.cpp)
add_executable(graph_viewer src/graph_viewer.cpp)
# add_executable(graph_viewer_ond src/graph_viewer_ond.cpp)
add_executable(loop_detector src/loop_detector.cpp)
add_executable(loop_detector_ond src/loop_detector_ond.cpp)
add_executable(loop_detector_color src/loop_detector_color.cpp)
add_executable(map_integrator src/map_integrator.cpp)
add_executable(map_integrator_color src/map_integrator_color.cpp)
# add_executable(map_integrator_new src/map_integrator_new.cpp)
add_executable(complement_filtered src/complement_filtered.cpp)
# add_executable(complement2 src/complement2.cpp)
# add_executable(set_drift_dyaw src/set_drift_dyaw.cpp src/drift_calculator.cpp)
# add_executable(position_publisher src/position_publisher.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(scan_match_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(scan_match_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(buffer_geo
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(buffer_geo_ond
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(buffer_sq1
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(buffer_sq2_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(icp_geo_sq1
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(icp_sq2_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gicp_geo
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gicp_ond
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gicp_geo_sq1
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gicp_color_sq
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gicp_for_loop
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(gicp_for_loop_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(icp_for_loop
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(icp_for_loop_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(loop_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(loop_detector_ond
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(loop_detector_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(map_integrator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(map_integrator_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(graph_viewer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(complement_filtered
  ${catkin_LIBRARIES}
)


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
# install(TARGETS scan_match scan_match_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scan_match.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
