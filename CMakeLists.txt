cmake_minimum_required(VERSION 3.8)
project(crazy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(ae_hyu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Create executables
add_executable(controller_node
  src/crazy_controller.cpp
  src/utils.cpp
)

add_executable(pp_controller_node
  src/pp_controller.cpp
  src/utils_pp.cpp
  src/utils.cpp
)

# Specify dependencies
ament_target_dependencies(controller_node
  rclcpp
  ackermann_msgs
  ae_hyu_msgs
  nav_msgs
  sensor_msgs
  rcl_interfaces
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(pp_controller_node
  rclcpp
  ackermann_msgs
  ae_hyu_msgs
  nav_msgs
  sensor_msgs
  rcl_interfaces
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Link libraries
target_link_libraries(controller_node
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(pp_controller_node
  Eigen3::Eigen
  yaml-cpp
)

target_compile_features(controller_node PUBLIC cxx_std_17)
target_compile_features(pp_controller_node PUBLIC cxx_std_17)

# Install targets
install(TARGETS
  controller_node
  pp_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()