cmake_minimum_required(VERSION 2.8.3)
project(bluespace_ai_xsens_mti_driver)

cmake_policy(SET CMP0057 NEW)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

###########
## Build ##
###########

link_directories(lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)
add_executable(
	xsens_mti_node
	src/main.cpp
	src/xdainterface.cpp
	src/xdacallback.cpp
  src/kalman_filter.cpp
)

target_include_directories(xsens_mti_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(xsens_mti_node 
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(
	xsens_mti_node
	xscontroller
	xscommon
	xstypes
	pthread
	dl
)

install(TARGETS xsens_mti_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
