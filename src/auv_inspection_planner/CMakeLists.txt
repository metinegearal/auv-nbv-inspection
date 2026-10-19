cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(auv_inspection_planner)
enable_language(Fortran)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  # PCL's find modules establish their own policy scope.
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
set(_planner_open3d_sdk "${CMAKE_CURRENT_SOURCE_DIR}/.dependencies/open3d-devel-linux-x86_64-cxx11-abi-0.19.0")
if(NOT Open3D_ROOT AND NOT DEFINED ENV{Open3D_ROOT} AND
    EXISTS "${_planner_open3d_sdk}/lib/cmake/Open3D/Open3DConfig.cmake")
  set(Open3D_ROOT "${_planner_open3d_sdk}" CACHE PATH "Open3D C++ SDK prefix")
endif()
find_package(Open3D 0.19.0 EXACT CONFIG QUIET)
if(NOT Open3D_FOUND)
  message(FATAL_ERROR
    "Open3D 0.19.0 C++ SDK was not found. Run:\n"
    "  python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/bootstrap_open3d.py\n"
    "Then rebuild. For an SDK installed elsewhere, pass -DOpen3D_ROOT=<sdk-prefix>.")
endif()
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

file(GLOB FITPACK_SOURCES CONFIGURE_DEPENDS vendor/scipy/fitpack/*.f)
add_library(planner_numerics STATIC ${FITPACK_SOURCES} vendor/scipy/lbfgsb.c
  vendor/scipy/ckdtree/build.cxx vendor/scipy/ckdtree/query.cxx
  vendor/scipy/ckdtree/query_ball_point.cxx)
target_compile_features(planner_numerics PRIVATE cxx_std_17)
target_link_libraries(planner_numerics PUBLIC ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
add_library(planner_migration src/migration/route.cpp src/migration/collision.cpp src/migration/nbv.cpp
  src/migration/optimization.cpp src/migration/configuration.cpp src/migration/mission.cpp)
target_compile_features(planner_migration PUBLIC cxx_std_17)
target_include_directories(planner_migration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor>
  $<INSTALL_INTERFACE:include>)
target_compile_options(planner_migration PRIVATE -ffp-contract=off)
target_link_libraries(planner_migration PUBLIC Eigen3::Eigen Open3D::Open3D planner_numerics)
# The upstream Linux SDK needs libc++ for its rendering dependency. An SDK
# prepared by tools/bootstrap_open3d.py carries the matching runtime libraries.
get_filename_component(_planner_open3d_lib "${Open3D_DIR}/../.." ABSOLUTE)
find_library(OPEN3D_LIBCXX NAMES libc++.so.1 HINTS "${Open3D_ROOT}/lib" "${_planner_open3d_lib}")
find_library(OPEN3D_LIBCXXABI NAMES libc++abi.so.1 HINTS "${Open3D_ROOT}/lib" "${_planner_open3d_lib}")
if(OPEN3D_LIBCXX AND OPEN3D_LIBCXXABI)
  target_link_libraries(planner_migration PUBLIC ${OPEN3D_LIBCXX} ${OPEN3D_LIBCXXABI})
endif()
add_library(planner_migration_adapters src/migration/adapters.cpp)
target_link_libraries(planner_migration_adapters PUBLIC planner_migration ${OCTOMAP_LIBRARIES})
target_include_directories(planner_migration_adapters PUBLIC ${OCTOMAP_INCLUDE_DIRS})
add_executable(planner_replay test/migration/replay.cpp)
target_link_libraries(planner_replay PRIVATE planner_migration_adapters)
add_executable(inspection_planner_node src/migration/node.cpp)
set_target_properties(inspection_planner_node planner_replay PROPERTIES
  INSTALL_RPATH "${_planner_open3d_lib}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)
target_link_libraries(inspection_planner_node PRIVATE planner_migration_adapters ${OpenCV_LIBRARIES})
target_include_directories(inspection_planner_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(inspection_planner_node PUBLIC rclcpp nav_msgs sensor_msgs std_msgs cv_bridge)
install(TARGETS inspection_planner_node planner_replay DESTINATION lib/${PROJECT_NAME})
install(TARGETS planner_migration planner_migration_adapters planner_numerics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(FILES vendor/json.hpp DESTINATION include)

add_library(inspection_policy src/policy/mapping.cpp src/search_algo.cpp)
target_compile_features(inspection_policy PUBLIC cxx_std_17)
target_include_directories(inspection_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(inspection_policy PUBLIC Eigen3::Eigen)

add_library(inspection_map_adapters src/adapters/maps.cpp)
target_compile_features(inspection_map_adapters PUBLIC cxx_std_17)
target_include_directories(inspection_map_adapters PUBLIC ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(inspection_map_adapters PUBLIC inspection_policy ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(nbv_utility_node src/nbv_utility_node.cpp)
target_include_directories(nbv_utility_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nbv_utility_node inspection_map_adapters ${OpenCV_LIBRARIES})
ament_target_dependencies(nbv_utility_node rclcpp std_msgs sensor_msgs nav_msgs cv_bridge tf2)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS inspection_policy inspection_map_adapters EXPORT inspectionTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS nbv_utility_node DESTINATION lib/${PROJECT_NAME})
ament_export_targets(inspectionTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL octomap)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME planner_differential
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_differential.py)
  add_test(NAME planner_telemetry
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_telemetry.py)
  add_test(NAME planner_nbv_cache
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_nbv_cache.py)
  add_test(NAME planner_collision_recovery
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_collision_recovery.py)
  add_test(NAME planner_goal_standoff
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_goal_standoff.py)
  add_test(NAME planner_target_lifecycle
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_target_lifecycle.py)
  add_test(NAME planner_eligible_scoring
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/migration/test_eligible_scoring.py)
  set_tests_properties(planner_differential planner_telemetry planner_nbv_cache planner_collision_recovery planner_goal_standoff planner_target_lifecycle planner_eligible_scoring PROPERTIES
    ENVIRONMENT "PLANNER_REPLAY=$<TARGET_FILE:planner_replay>;PYTHONDONTWRITEBYTECODE=1;OPENBLAS_NUM_THREADS=1;OMP_NUM_THREADS=1;MPLCONFIGDIR=/tmp/planner_migration_mpl"
    TIMEOUT 240)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  enable_testing()
  add_executable(inspection_policy_regression test/policy_regression.cpp)
  target_link_libraries(inspection_policy_regression PRIVATE inspection_policy)
  add_test(NAME inspection_policy_regression COMMAND inspection_policy_regression)
endif()
ament_package()
