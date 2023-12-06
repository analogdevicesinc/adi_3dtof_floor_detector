cmake_minimum_required(VERSION 3.8)
project(adi_3dtof_floor_detector)

# c standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# c++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# add include files as per PCL version installed
if(${PCL_VERSION} VERSION_EQUAL 1.10.0)
  include_directories(include_floor_plane_detection/pcl_1_10_0)
elseif(${PCL_VERSION} VERSION_EQUAL 1.12.1)
  include_directories(include_floor_plane_detection/pcl_1_12_1)
endif()

#target_compile_definitions(${PROJECT_NAME} PRIVATE "ENABLE_FUNCTION_PROFILING")

# neon Intrinsic
if(CMAKE_BUILD_TYPE)
  string(TOLOWER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
  if ((${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64") AND (${BUILD_TYPE} STREQUAL "release"))
    #To enable NEON optimizations on device
    add_compile_definitions(ENABLE_NEON_OPTIMIZATION)
    #To verify NEON optimizations
    #add_compile_definitions(ENABLE_NEON_OPTIMIZATION_VERIFICATION)
  endif()
endif()

# host specific build command
if(NOT HOST_BUILD)
  message(WARNING "If the package is being built on the TOF module, ignore this warning; otherwise, use the command below to build the package on the host.
  catkin_make -DCMAKE_BUILD_TYPE=Release -DHOST_BUILD=TRUE")
endif()

# includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#List common dependencies
SET(COMMON_DEPS
  rclcpp 
  std_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  pcl_ros 
  pcl_conversions 
  cv_bridge 
  image_geometry 
  image_transport 
  compressed_depth_image_transport
  OpenCV
)

#List of common source files.
SET(COMMON_SRC_FILES 
  src/adi_3dtof_floor_detector_main.cpp 
  src/adi_3dtof_floor_detector_algorithm.cpp 
  src/adi_3dtof_floor_detector_application.cpp 
  src/adi_3dtof_floor_detector_input_thread.cpp 
  src/adi_3dtof_floor_detector_output_thread.cpp
  src/image_proc_utils.cpp
  src/input_sensor_file.cpp 
  src/input_sensor_file_rosbagbin.cpp 
  src/output_sensor_file.cpp 
  src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp
)

# executables
# main node
if(NOT HOST_BUILD)
  # compile for platforms with 3D ToF Sensor
  add_compile_definitions(ENABLE_ADI_3DTOF_ADTF31XX_SENSOR)
  add_executable(${PROJECT_NAME}_node ${COMMON_SRC_FILES} src/input_sensor_adtf31xx.cpp)
else()
  # compile for non 3D ToF sensor platform(ex:Host machines)
  add_executable(${PROJECT_NAME}_node ${COMMON_SRC_FILES} src/input_sensor_ros_topics.cpp)
endif()

# target the dependencies for the chosen node
ament_target_dependencies(${PROJECT_NAME}_node ${COMMON_DEPS})

# linking dependencies
if(NOT HOST_BUILD)
  # required for sensor platform only
  set(ADITOF_LIB1 ${PROJECT_SOURCE_DIR}/libs/libaditof.so)
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64")
  if(${PCL_VERSION} VERSION_EQUAL 1.10.0)
    set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs/libfloor_plane_detection.a)
  endif()
elseif (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64|X86_64")
  if(${PCL_VERSION} VERSION_EQUAL 1.12.1)
    set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs_linux_x86/libfloor_plane_detection.a)
  endif()
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${ADITOF_LIB1}
  ${FLOOR_DETECTOR_CORE_LIB}
)

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME})

# example node only for host

if(HOST_BUILD)
  # example node executable
  add_executable(${PROJECT_NAME}_example_node
    src/adi_3dtof_floor_detector_example.cpp
    src/image_proc_utils.cpp
    src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp
  )

  ament_target_dependencies(${PROJECT_NAME}_example_node ${COMMON_DEPS})

  target_link_libraries(${PROJECT_NAME}_example_node
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}_example_node
  DESTINATION lib/${PROJECT_NAME})

endif()

# install
install(DIRECTORY rviz 
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rqt_config
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
#endif()

ament_package()

