cmake_minimum_required(VERSION 3.0.2)
project(adi_3dtof_floor_detector)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std = c++ 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_view
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  tf2
  tf2_ros
  pcl_ros
  dynamic_reconfigure
)

find_package(Eigen3 QUIET)
find_package(OpenCV REQUIRED)

if(CMAKE_BUILD_TYPE)
  string(TOLOWER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
  if ((${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64") AND (${BUILD_TYPE} STREQUAL "release"))
    #To enable NEON optimizations on device
    add_compile_definitions(ENABLE_NEON_OPTIMIZATION)
    #To verify NEON optimizations
    #add_compile_definitions(ENABLE_NEON_OPTIMIZATION_VERIFICATION)
  endif()
endif()

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#add_message_files(
#FILES
#Message1.msg
#Message2.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#FILES
#Service1.srv
#Service2.srv
#)

## Generate actions in the 'action' folder
#add_action_files(
#FILES
#Action1.action
#Action2.action
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#DEPENDENCIES
#std_msgs #Or other packages containing msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/FloorDetectorParams.cfg
#cfg / DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adi_3dtof_floor_detector
  CATKIN_DEPENDS roscpp std_msgs image_geometry sensor_msgs pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

if(NOT HOST_BUILD)
  message(WARNING "If the package is being built on the TOF module, ignore this warning; otherwise, use the command below to build the package on the host.
  catkin_make -DCMAKE_BUILD_TYPE=Release -DHOST_BUILD=TRUE")
endif()

## Declare a C++ library
#add_library(${ PROJECT_NAME }
#src / ${ PROJECT_NAME } / adi_3dtof_floor_detector.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${ PROJECT_NAME } ${ ${ PROJECT_NAME } _EXPORTED_TARGETS } ${ catkin_EXPORTED_TARGETS })

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# The macro should be passed using cmake command
# ex: catkin_make -DHOST_BUILD=TRUE,
# By default it is set to FALSE
if(NOT HOST_BUILD)
  add_compile_definitions(ENABLE_ADI_3DTOF_ADTF31XX_SENSOR)
  add_executable(${PROJECT_NAME}_node src/adi_3dtof_floor_detector_main.cpp src/adi_3dtof_floor_detector_algorithm.cpp src/adi_3dtof_floor_detector_application.cpp src/input_sensor_file.cpp src/input_sensor_adtf31xx.cpp src/image_proc_utils.cpp src/output_sensor_file.cpp src/adi_3dtof_floor_detector_input_thread.cpp src/adi_3dtof_floor_detector_output_thread.cpp src/input_sensor_file_rosbagbin.cpp src/input_sensor_ros_topics.cpp)
else()
  add_executable(${PROJECT_NAME}_node src/adi_3dtof_floor_detector_main.cpp src/adi_3dtof_floor_detector_algorithm.cpp src/adi_3dtof_floor_detector_application.cpp src/input_sensor_file.cpp src/image_proc_utils.cpp src/output_sensor_file.cpp src/adi_3dtof_floor_detector_input_thread.cpp src/adi_3dtof_floor_detector_output_thread.cpp src/input_sensor_file_rosbagbin.cpp src/input_sensor_ros_topics.cpp)
endif()

#Example Node
add_executable(adi_3dtof_floor_detector_example_node src/adi_3dtof_floor_detector_example.cpp src/image_proc_utils.cpp )

#Dynamic Reconfigure
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${ PROJECT_NAME } _node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${ PROJECT_NAME } _node ${ ${ PROJECT_NAME } _EXPORTED_TARGETS } ${ catkin_EXPORTED_TARGETS })
if(NOT HOST_BUILD)
  set(ADITOF_LIB1 ${PROJECT_SOURCE_DIR}/libs/libaditof.so)
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64")
  set(ADITOF_LIB2 ${PROJECT_SOURCE_DIR}/libs/libcompressed_depth_image_transport.so)
  set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs/libfloor_plane_detection.a)
elseif (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64|X86_64")
  set(ADITOF_LIB2 ${PROJECT_SOURCE_DIR}/libs_linux_x86/libcompressed_depth_image_transport.so)
  set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs_linux_x86/libfloor_plane_detection.a)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ADITOF_LIB1}
  ${ADITOF_LIB2}
  ${FLOOR_DETECTOR_CORE_LIB})

target_link_libraries(${PROJECT_NAME}_example_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

#all install targets should use catkin DESTINATION variables
#See http:  // ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
#scripts / my_python_script
#DESTINATION ${ CATKIN_PACKAGE_BIN_DESTINATION }
#)

#Copy the lib to devel / lib
set(COMPRESSED_DEPTH_IMAGE_TRANSPORT_LIB ${CATKIN_DEVEL_PREFIX}/lib/libcompressed_depth_image_transport.so)
add_custom_command( TARGET ${PROJECT_NAME}_node POST_BUILD COMMAND cp ${ADITOF_LIB2} ${COMPRESSED_DEPTH_IMAGE_TRANSPORT_LIB} )

set(RQT_RVIZ_ORIGINAL_FILE ${PROJECT_SOURCE_DIR}/rqt_config/adi_3dtof_floor_detector_rqt_rviz.rviz)
set(RQT_RVIZ_DEST_FILE ~/.ros/adi_3dtof_floor_detector_rqt_rviz.rviz)
add_custom_command( TARGET ${PROJECT_NAME}_node POST_BUILD COMMAND cp ${RQT_RVIZ_ORIGINAL_FILE} ${RQT_RVIZ_DEST_FILE} )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
 install(TARGETS ${PROJECT_NAME}_example_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${ PROJECT_NAME }
#ARCHIVE DESTINATION ${ CATKIN_PACKAGE_LIB_DESTINATION }
#LIBRARY DESTINATION ${ CATKIN_PACKAGE_LIB_DESTINATION }
#RUNTIME DESTINATION ${ CATKIN_GLOBAL_BIN_DESTINATION }
#)

## Mark cpp header files for installation
#install(DIRECTORY include / ${ PROJECT_NAME } /
#DESTINATION ${ CATKIN_PACKAGE_INCLUDE_DESTINATION }
#FILES_MATCHING PATTERN "*.h"
#PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
# #myfile1
# #myfile2
#DESTINATION ${ CATKIN_PACKAGE_SHARE_DESTINATION }
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#if (TARGET ${ PROJECT_NAME } - test)
#target_link_libraries(${ PROJECT_NAME } - test ${ PROJECT_NAME })
#endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)
