cmake_minimum_required(VERSION 3.8)
project(motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==============================================================================
# Node 1: waypoint_control_node
# ==============================================================================
add_executable(waypoint_control_node src/waypoint_control_node.cpp)
ament_target_dependencies(waypoint_control_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs
)
target_link_libraries(waypoint_control_node nlohmann_json::nlohmann_json)
target_include_directories(waypoint_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(waypoint_control_node PUBLIC c_std_99 cxx_std_17)

add_executable(point_iterator src/point_iterator.cpp)
ament_target_dependencies(point_iterator 
  rclcpp 
  std_msgs 
)

install(TARGETS 
  waypoint_control_node
  point_iterator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()