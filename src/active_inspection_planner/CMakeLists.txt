cmake_minimum_required(VERSION 3.8)
project(active_inspection_planner)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(nbv_utility_node 
src/nbv_utility_node.cpp 
src/realtime_stereo_mapper.cpp 
  src/search_algo.cpp)
# target_include_directories(nbv_utility_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(nbv_utility_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(nbv_utility_node 
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(nbv_utility_node 
  rclcpp std_msgs sensor_msgs nav_msgs cv_bridge tf2
)
install(TARGETS nbv_utility_node
  DESTINATION lib/${PROJECT_NAME})

target_compile_features(nbv_utility_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
