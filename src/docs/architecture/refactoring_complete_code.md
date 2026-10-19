# Complete refactoring code bundle

Full current contents of changed/new source, build, manifest, resource and verification files.
Documentation is linked from [the architecture index](../new_arch/README.md).
This generated review artifact contains no omitted code. Protocol `...` bodies are actual abstract declarations.
The optimizer moved to `auv_experiments`; the separately removed legacy planner Tools are not restored.

### NBV_paper_ros2/src/auv_inspection_planner/CMakeLists.txt

SHA-256: `2cf5f76928c8f1af38c540e9fc90aab0aee2e90bf565c9bcbec79e1e07f1d657`

````cmake
cmake_minimum_required(VERSION 3.8)
project(auv_inspection_planner)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
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
````

### NBV_paper_ros2/src/auv_inspection_planner/include/auv_inspection_planner/adapters/maps.hpp

SHA-256: `eeddea6d0e0636b93600a495a57aa6e3ac9309954abce4084799e3080ab30476`

````cpp
#pragma once
#include <octomap/ColorOcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "auv_inspection_planner/domain/contracts.hpp"
namespace inspection
{
class OctomapAdapter final : public MapReader, public MapWriter, public MapSnapshotStore {
  octomap::ColorOcTree tree_; Diagnostics & diagnostics_;

public:
  OctomapAdapter(double resolution, Diagnostics & diagnostics)
  :tree_(resolution), diagnostics_(diagnostics) {}
  std::size_t count() const override {return tree_.getNumLeafNodes();}
  std::vector<MapLeaf> leaves() const override
  {
    std::vector<MapLeaf> result;
    for(auto it = tree_.begin_leafs(), end = tree_.end_leafs(); it != end; ++it) {
      result.push_back({Eigen::Vector3d(it.getX(), it.getY(), it.getZ()),
          tree_.isNodeOccupied(*it)});
    }
    return result;
  }
  void insert(
    const Cloud & values, const Colors & colors, const Eigen::Vector3d & location,
    double range) override
  {
    octomap::Pointcloud cloud;
    for(const auto & point:values) {cloud.push_back(point.x(), point.y(), point.z());}
    octomap::point3d origin(location.x(), location.y(), location.z());
    tree_.insertPointCloud(cloud, origin, range);
    for(std::size_t i = 0; i < cloud.size(); ++i) {
      tree_.integrateNodeColor(cloud[i].x(), cloud[i].y(), cloud[i].z(), colors[i][0], colors[i][1],
          colors[i][2]);
    }
  }
  void save(const std::string & path) override
  {
    tree_.write(path); diagnostics_.text("Final map saved to " + path + "\n", false, true);
  }
};
class PclIndex final : public SpatialIndex {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;

public:
  explicit PclIndex(const std::vector<Eigen::Vector3d> & points)
  {
    for(const auto & p:points) {points_->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));}
    if(!points_->empty()) {tree_.setInputCloud(points_);}
  }
  std::size_t size() const override {return points_->size();}
  Eigen::Vector3d point(std::size_t i) const override
  {
    const auto & p = (*points_)[i]; return Eigen::Vector3d(p.x, p.y, p.z);
  }
  int nearest(
    std::size_t i, int k, std::vector<int> & indices,
    std::vector<float> & distances) const override
  {
    return tree_.nearestKSearch((*points_)[i], k, indices, distances);
  }
  int radius(
    const Eigen::Vector3d & p, double radius, std::vector<int> & indices,
    std::vector<float> & distances) const override
  {
    return tree_.radiusSearch(pcl::PointXYZ(p.x(), p.y(), p.z()), radius, indices, distances);
  }
};
class PclIndexes final : public SpatialIndexes {
public:
  std::unique_ptr<SpatialIndex> create(const std::vector<Eigen::Vector3d> & points) const override
  {
    return std::make_unique<PclIndex>(points);
  }
};
}
````

### NBV_paper_ros2/src/auv_inspection_planner/include/auv_inspection_planner/adapters/mission_io.hpp

SHA-256: `1135d7d797e852cdf1392d0ccb12b642d45aeb5ea5d7c4a5071e3f2c767a3a81`

````cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include "auv_inspection_planner/domain/contracts.hpp"
namespace inspection
{
class RosDiagnostics final : public Diagnostics {
  rclcpp::Logger logger_;

public:
  explicit RosDiagnostics(rclcpp::Logger logger)
  :logger_(logger) {}
  void text(const std::string & text, bool error, bool flush) override
  {
    auto & stream = error ? std::cerr : std::cout; stream << text; if(flush) {stream.flush();}
  }
  void info(const std::string & message) override {RCLCPP_INFO(logger_, "%s", message.c_str());}
  void warning(const std::string & message) override {RCLCPP_WARN(logger_, "%s", message.c_str());}
  void error(const std::string & message) override {RCLCPP_ERROR(logger_, "%s", message.c_str());}
};
class SystemClock final : public Clock {
public:
  Time now() const override
  {
    return std::chrono::high_resolution_clock::now();
  }
};
class FileMissionMetrics final : public MissionMetrics {
  const std::string path_; std::ofstream stream_; bool started_ = false;

public:
  explicit FileMissionMetrics(std::string path)
  :path_(std::move(path)) {}
  void begin() override {if(!started_) {stream_.open(path_, std::ios::app); started_ = true;}}
  void record(double elapsed, std::size_t added, std::size_t leaves) override
  {
    stream_ << "Process Time (ms): " << elapsed << ", Exploration Score: " << added <<
      ", Total Leaves: " << leaves << "\n";
  }
};
class RosMissionOutput final : public MissionOutput {
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoints_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_;

public:
  RosMissionOutput(
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoints,
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished)
  :waypoints_(waypoints), finished_(finished) {}
  void trajectory(const std::vector<Eigen::Vector4d> & values) override
  {
    std_msgs::msg::Float32MultiArray msg;
    for(const auto & p:values) {
      msg.data.push_back(p.x());msg.data.push_back(p.y());msg.data.push_back(p.z());
      msg.data.push_back(p.w());
    }
    waypoints_->publish(msg);
  }
  void finished() override {std_msgs::msg::Bool msg;msg.data = true;finished_->publish(msg);}
};
inline std::shared_ptr<const ColorImage> color_values(const cv::Mat & image)
{
  std::vector<std::array<unsigned char, 3>> pixels;
  for(int v = 0; v < image.rows; ++v) {
    for(int u = 0; u < image.cols; ++u) {
      const auto p = image.at<cv::Vec3b>(v, u); pixels.push_back({p[0], p[1], p[2]});
    }
  }
  return std::make_shared<const ColorImage>(ColorImage{image.rows, image.cols, std::move(pixels)});
}
inline std::shared_ptr<const DepthImage> depth_values(const cv::Mat & image)
{
  std::vector<float> pixels;
  for(int v = 0; v < image.rows; ++v) {
    for(int u = 0; u < image.cols; ++u) {
      pixels.push_back(image.at<float>(v, u));
    }
  }
  return std::make_shared<const DepthImage>(DepthImage{image.rows, image.cols, std::move(pixels)});
}
}
````

### NBV_paper_ros2/src/auv_inspection_planner/include/auv_inspection_planner/domain/contracts.hpp

SHA-256: `0419db190233142f787affb6ae7a4572c41c3944a3bcd7f8cb9b0ac7dd36223e`

````cpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cstdio>
#include "auv_inspection_planner/domain/search.hpp"
namespace inspection
{
struct MapperConfig
{
  const double fov_deg = 90.0, focal_len = 256.0, baseline = 0.5, resolution = 0.1,
    max_depth = 4.0, min_depth = 0.5, z_min_target = -32.0, info_gain_threshold = 0.15,
    sparsity_multiplier = 1.2, min_utility_threshold = 0.02;
};
struct Pose { const Eigen::Vector3d position; const double yaw, pitch, roll; };
struct DepthImage
{
  const int rows, cols; const std::vector<float> pixels;
  bool empty() const {return rows == 0 || cols == 0;}
  float at(int row, int col) const {return pixels[row * cols + col];}
};
struct ColorImage
{
  const int rows, cols; const std::vector<std::array<unsigned char, 3>> pixels;
  bool empty() const {return rows == 0 || cols == 0;}
  std::array<unsigned char, 3> at(int row, int col) const {return pixels[row * cols + col];}
};
struct MapLeaf { const Eigen::Vector3d point; const bool occupied; };
using Cloud = std::vector<Eigen::Vector3f>;
using Colors = std::vector<std::array<unsigned char, 3>>;
using Candidates = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using Route = std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>;
using Plan = std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector4d>>;
class MapReader {
public:
  virtual ~MapReader() = default;
  virtual std::vector<MapLeaf> leaves() const= 0; virtual std::size_t count() const= 0;
};
class MapWriter {
public:
  virtual ~MapWriter() = default;
  virtual void insert(const Cloud &, const Colors &, const Eigen::Vector3d &, double) = 0;
};
class MapSnapshotStore {
public:
  virtual ~MapSnapshotStore() = default; virtual void save(const std::string &) = 0;
};
class SpatialIndex {
public:
  virtual ~SpatialIndex() = default;
  virtual std::size_t size() const= 0;
  virtual Eigen::Vector3d point(std::size_t) const= 0;
  virtual int nearest(std::size_t, int, std::vector<int> &, std::vector<float> &) const= 0;
  virtual int radius(
    const Eigen::Vector3d &, double, std::vector<int> &,
    std::vector<float> &) const= 0;
};
class SpatialIndexes {
public:
  virtual ~SpatialIndexes() = default;
  virtual std::unique_ptr<SpatialIndex> create(const std::vector<Eigen::Vector3d> &) const= 0;
};
class Diagnostics {
public:
  virtual ~Diagnostics() = default;
  virtual void text(const std::string &, bool error, bool flush) = 0;
  virtual void info(const std::string &) = 0; virtual void warning(const std::string &) = 0;
  virtual void error(const std::string &) = 0;
};
class Clock {
public:
  using Time = std::chrono::high_resolution_clock::time_point;
  virtual ~Clock() = default; virtual Time now() const= 0;
};
class MissionMetrics {
public:
  virtual ~MissionMetrics() = default;
  virtual void begin() = 0;
  virtual void record(double, std::size_t, std::size_t) = 0;
};
class MissionOutput {
public:
  virtual ~MissionOutput() = default;
  virtual void trajectory(const std::vector<Eigen::Vector4d> &) = 0; virtual void finished() = 0;
};
inline std::string format_text(const char * value) {return value;}
template<class ... Args> std::string format_text(const char * format, Args... args)
{
  int length = std::snprintf(nullptr, 0, format, args ...);
  std::vector<char> buffer(length + 1);
  std::snprintf(buffer.data(), buffer.size(), format, args ...);
  return std::string(buffer.data(), length);
}
}
````

### NBV_paper_ros2/src/auv_inspection_planner/include/auv_inspection_planner/domain/search.hpp

SHA-256: `63edf3778efc1cbdae15f2dc0f1bd2cd3035756050eb35d5d26a9141f93e2849`

````cpp
#ifndef SEARCH_ALGO_HPP
#define SEARCH_ALGO_HPP

#include <vector>
#include <unordered_set>
#include <tuple>
#include <cmath>
#include <queue>
#include <functional>
#include <string>
#include <Eigen/Dense>

// Voxel Index Structure and Hash for O(1) lookups
struct VoxelIdx
{
  int x, y, z;
  bool operator==(const VoxelIdx & o) const {return x == o.x && y == o.y && z == o.z;}
};

struct VoxelHash
{
  std::size_t operator()(const VoxelIdx & k) const
  {
    return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) <<
           1);
  }
};

using VoxelSet = std::unordered_set<VoxelIdx, VoxelHash>;

class SearchPolicy {
public:
  static std::vector<Eigen::Vector3d> astar_3d(
    const Eigen::Vector3d & start_pos,
    const Eigen::Vector3d & goal_pos,
    const VoxelSet & occ_set,
    double res,
    int max_steps = 50000,
    const std::function<void(const std::string &)> & failure = {});

  static bool check_line_of_sight(
    const Eigen::Vector3d & p_start,
    const Eigen::Vector3d & p_end,
    const VoxelSet & occ_set,
    double res,
    int gap_multiplier = 4);
};

#endif
````

### NBV_paper_ros2/src/auv_inspection_planner/include/auv_inspection_planner/use_cases/mapping.hpp

SHA-256: `c6da8f86c822a77bec208d2fb03f7f63570f2f28a9198ed75031f4a08caced04`

````cpp
#pragma once
#include "auv_inspection_planner/domain/contracts.hpp"
namespace inspection
{
class MapUpdate {
  const MapperConfig config_; MapReader & reader_; MapWriter & writer_;
  std::size_t previous_leaves_ = 0;
  void generate_global_point_cloud(
    const DepthImage &, const ColorImage &,
    const Eigen::Vector3d &, const Eigen::Matrix3d &, Cloud &, Colors &);

public:
  MapUpdate(const MapperConfig & config, MapReader & reader, MapWriter & writer)
  : config_(config), reader_(reader), writer_(writer) {}
  void process_frame(
    const Eigen::Vector3d &, double, double, double, const ColorImage &,
    const DepthImage &);
};
class RoutePlanner {
  const double resolution_; Diagnostics & diagnostics_;

public:
  RoutePlanner(double resolution, Diagnostics & diagnostics)
  :resolution_(resolution), diagnostics_(diagnostics) {}
  Route plan_multi_waypoint_route(const Eigen::Vector3d &, Candidates &, const VoxelSet &);
  std::vector<Eigen::Vector4d> generate_seamless_mission_trajectory(
    const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3d> &, int);
};
class CandidatePlanner {
  const MapperConfig config_; const double resolution_;
  MapReader & reader_; const SpatialIndexes & indexes_; RoutePlanner & route_;
  Diagnostics & diagnostics_;

public:
  CandidatePlanner(
    const MapperConfig & config, MapReader & reader, const SpatialIndexes & indexes,
    RoutePlanner & route, Diagnostics & diagnostics)
  :config_(config), resolution_(config.resolution), reader_(reader), indexes_(indexes),
    route_(route), diagnostics_(diagnostics) {}
  Plan generate_candidate_viewpoints(const Eigen::Vector3d &);
};
}
````

### NBV_paper_ros2/src/auv_inspection_planner/include/auv_inspection_planner/use_cases/mission.hpp

SHA-256: `bf7d26b648568c3b277215cd768252a3c52662ac4b8c7477fa58e791a8b1e7e4`

````cpp
#pragma once
#include "auv_inspection_planner/use_cases/mapping.hpp"
namespace inspection
{
class InspectionMission {
  MapUpdate & update_; CandidatePlanner & planner_; MapReader & reader_; MapSnapshotStore & store_;
  MissionOutput & output_; MissionMetrics & metrics_; Clock & clock_; Diagnostics & diagnostics_;
  const std::string map_path_;
  bool odom_received_ = false, started_mapping_ = false, finished_mapping_ = false;
  int iter_count_ = 0; std::size_t previous_node_leaves_ = 0;
  Eigen::Vector3d current_position_{0, 0, 0}, latched_pos_{0, 0, 0};
  double yaw_ = 0, pitch_ = 0, roll_ = 0, latched_yaw_ = 0, latched_pitch_ = 0, latched_roll_ = 0;
  std::shared_ptr<const ColorImage> left_image_;
  std::shared_ptr<const DepthImage> depth_frame_;

public:
  InspectionMission(
    MapUpdate & update, CandidatePlanner & planner, MapReader & reader,
    MapSnapshotStore & store, MissionOutput & output, MissionMetrics & metrics, Clock & clock,
    Diagnostics & diagnostics, std::string path)
  :update_(update), planner_(planner), reader_(reader), store_(store), output_(output),
    metrics_(metrics),
    clock_(clock), diagnostics_(diagnostics), map_path_(std::move(path)) {}
  bool accepts_pose(const Eigen::Vector3d & p) const
  {
    return !(std::abs(p.x()) < 0.01 && std::abs(p.y()) < 0.01);
  }
  void pose(const Pose & pose)
  {
    current_position_ = pose.position; roll_ = pose.roll; pitch_ = pose.pitch; yaw_ = pose.yaw;
    odom_received_ = true;
  }
  void magnetic(double x, double y)
  {
    double ya = std::atan2(x, y) - (M_PI / 2.0); if(ya < -M_PI) {ya += 2 * M_PI;} yaw_ = ya;
  }
  void rgb(std::shared_ptr<const ColorImage> frame) {left_image_ = std::move(frame);}
  void depth(std::shared_ptr<const DepthImage> frame)
  {
    depth_frame_ = std::move(frame); latched_pos_ = current_position_; latched_yaw_ = yaw_;
    latched_pitch_ = pitch_; latched_roll_ = roll_;
  }
  void timer_callback()
  {
    diagnostics_.info(format_text(
        "Timer triggered. Iteration: %d, left_images: %d, depth_frames: %d, odom_received: %s",
        iter_count_, (!left_image_ || left_image_->empty()) ? 0 : 1,
        (!depth_frame_ || depth_frame_->empty()) ? 0 : 1, odom_received_ ? "true" : "false"));
    if (finished_mapping_) {return;}
    if ((!left_image_ || left_image_->empty()) || (!depth_frame_ || depth_frame_->empty())) {
      return;
    }
    if (!odom_received_) {return;}


    auto start_proc = clock_.now();

    update_.process_frame(latched_pos_, latched_yaw_, latched_pitch_, latched_roll_, *left_image_,
        *depth_frame_);

    if (iter_count_ % 20 == 0) {
      metrics_.begin();
      auto start_time = clock_.now();

      const std::string & map_path = map_path_;
      store_.save(map_path);

      auto [viewpoints, trajectories] = planner_.generate_candidate_viewpoints(current_position_);

            // --- NEW DEBUGGING BLOCK ---
      diagnostics_.info(format_text("A* Pipeline Finished. Trajectories size: %zu",
          trajectories.size()));
      if (!trajectories.empty()) {
        diagnostics_.info(format_text("First Trajectory Point: [%.2f, %.2f, %.2f]",
                            trajectories[0].x(), trajectories[0].y(), trajectories[0].z()));
      }
            // ---------------------------

      if (!trajectories.empty()) {

                // CRITICAL SAFETY CHECK: Prevent publishing [0,0,0] if A* failed weirdly
        if (trajectories.size() < 2 && std::abs(trajectories[0].x()) < 0.1 &&
          std::abs(trajectories[0].y()) < 0.1)
        {
          diagnostics_.error(format_text(
              "ALGORITHM RETURNED [0,0,0] INSTEAD OF A PATH. ABORTING PUBLISH!"));
        } else {
          started_mapping_ = true;
          output_.trajectory(trajectories);
          diagnostics_.info(format_text("Published valid trajectory to PathPlanner."));
        }
      } else if (started_mapping_) {
        finished_mapping_ = true;
        output_.finished();
        diagnostics_.info(format_text("Mapping finished. Explored all accessible frontiers."));
      } else {
        diagnostics_.info(format_text("No new viewpoints generated. Continuing exploration..."));
      }

      auto end_time = clock_.now();
      auto process_time_ms = std::chrono::duration<double,
          std::milli>(end_time - start_time).count();

      size_t current_leaves = reader_.count();
      size_t leaves_added = current_leaves - previous_node_leaves_;
      previous_node_leaves_ = current_leaves;

      metrics_.record(process_time_ms, leaves_added, current_leaves);
    }
    iter_count_++;
  }
};
}
````

### NBV_paper_ros2/src/auv_inspection_planner/package.xml

SHA-256: `9ee1a4c246f73f46fa1c794c1ff2cba5699525c10f59df7ac29de123f4bb95df`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_inspection_planner</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>pcl_conversions</depend>
  <depend>tf2</depend>
  <depend>octomap</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>eigen</depend>
  <depend>libpcl-all-dev</depend>
  <depend>libopencv-dev</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_inspection_planner/src/adapters/maps.cpp

SHA-256: `f685145495f55063dec0fdc3a5da9a4df81b5daba0e8e985359004d61e0a15ab`

````cpp
#include "auv_inspection_planner/adapters/maps.hpp"
````

### NBV_paper_ros2/src/auv_inspection_planner/src/nbv_utility_node.cpp

SHA-256: `02d837407b2afc06d5182fdf00622cf41f1f0725a1a10f132ca34fc7627c6332`

````cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "auv_inspection_planner/use_cases/mission.hpp"
#include "auv_inspection_planner/adapters/maps.hpp"
#include "auv_inspection_planner/adapters/mission_io.hpp"
#include <fstream>
#include <chrono>

class NBVUtilityNode : public rclcpp::Node {
public:
  NBVUtilityNode()
  : Node("nbv_utility_node")
  {
    const inspection::MapperConfig cfg{90.0, 256.0, 0.5,
      declare_parameter("voxel_resolution", 0.1), declare_parameter("max_depth", 5.0),
      declare_parameter("min_depth", 0.5), declare_parameter("z_min_target", -32.0),
      declare_parameter("info_gain_threshold", 0.05), declare_parameter("sparsity_multiplier", 0.8),
      declare_parameter("min_utility_threshold", 0.005)};
        // QoS for sensor data
    rclcpp::QoS qos(10);
    qos.best_effort();

        // Subscribers
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "holocean/depth/image_raw", qos,
            std::bind(&NBVUtilityNode::depth_callback, this, std::placeholders::_1));

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "holocean/cameraDown/image_raw", qos,
            std::bind(&NBVUtilityNode::rgb_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "holocean/odom", qos,
            std::bind(&NBVUtilityNode::odom_callback, this, std::placeholders::_1));

    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "holocean/mag", qos,
            std::bind(&NBVUtilityNode::mag_callback, this, std::placeholders::_1));

        // Publishers
    waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object/areaWaypoints",
      10);
    finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("/movement/finished_execution", 10);

    diagnostics_ = std::make_unique<inspection::RosDiagnostics>(get_logger());
    map_ = std::make_unique<inspection::OctomapAdapter>(cfg.resolution, *diagnostics_);
    update_ = std::make_unique<inspection::MapUpdate>(cfg, *map_, *map_);
    route_ = std::make_unique<inspection::RoutePlanner>(cfg.resolution, *diagnostics_);
    candidates_ = std::make_unique<inspection::CandidatePlanner>(cfg, *map_, indexes_, *route_,
      *diagnostics_);
    output_ = std::make_unique<inspection::RosMissionOutput>(waypoint_pub_, finished_pub_);
    mission_ = std::make_unique<inspection::InspectionMission>(*update_, *candidates_, *map_, *map_,
            *output_, metrics_, clock_, *diagnostics_,
            "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/final_map_ros2.bt");
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]{mission_->timer_callback();});
    RCLCPP_INFO(get_logger(), "✅ Object Detector Node Started in C++");
  }

private:
  inspection::PclIndexes indexes_;
  inspection::SystemClock clock_;
  inspection::FileMissionMetrics metrics_{"evaluation_parameters.txt"};
  std::unique_ptr<inspection::RosDiagnostics> diagnostics_;
  std::unique_ptr<inspection::OctomapAdapter> map_;
  std::unique_ptr<inspection::MapUpdate> update_;
  std::unique_ptr<inspection::RoutePlanner> route_;
  std::unique_ptr<inspection::CandidatePlanner> candidates_;
  std::unique_ptr<inspection::RosMissionOutput> output_;
  std::unique_ptr<inspection::InspectionMission> mission_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, rgb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::TimerBase::SharedPtr timer_;


  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    if(!mission_->accepts_pose(position)) {return;}
    const auto & o = msg->pose.pose.orientation;
    tf2::Quaternion q(o.x, o.y, o.z, o.w); tf2::Matrix3x3 m(q);
    double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
    mission_->pose(inspection::Pose{position, yaw, pitch, roll});
  }
  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    mission_->magnetic(msg->magnetic_field.x, msg->magnetic_field.y);
  }
  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    mission_->rgb(inspection::color_values(cv_bridge::toCvCopy(msg, "bgr8")->image));
  }
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    mission_->depth(inspection::depth_values(cv_bridge::toCvCopy(msg, msg->encoding)->image));
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NBVUtilityNode>());
  rclcpp::shutdown();
  return 0;
}
````

### NBV_paper_ros2/src/auv_inspection_planner/src/policy/mapping.cpp

SHA-256: `57ba31642b3c8b44b3561187b42346b50732468c2183f6819245fbab364ac1f4`

````cpp
#include "auv_inspection_planner/use_cases/mapping.hpp"
#include <algorithm>
#include <numeric>
#include <sstream>
namespace inspection
{
void MapUpdate::generate_global_point_cloud(
  const DepthImage & depth_image, const ColorImage & color_image,
  const Eigen::Vector3d & location, const Eigen::Matrix3d & rot_matrix,
  Cloud & out_cloud, Colors & out_colors)
{
  int height = depth_image.rows;
  int width = depth_image.cols;
  double fx = (width / 2.0) / std::tan((config_.fov_deg * M_PI / 180.0) / 2.0);
  double fy = fx;
  double cx = width / 2.0;
  double cy = height / 2.0;

  double pitch_rad = 45.0 * M_PI / 180.0;
  double cos_p = std::cos(pitch_rad);
  double sin_p = std::sin(pitch_rad);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float Z = depth_image.at(v, u);       // Assuming 32FC1 depth map
      if (Z > config_.min_depth && Z < config_.max_depth) {
                // Pinhole camera math (Equivalent to numpy meshgrid logic)
        double X = (u - cx) * Z / fx;
        double Y = (v - cy) * Z / fy;

                // Local rotation (Vehicle Y must be -X)
        double local_x = Z * cos_p - Y * sin_p;
        double local_y = -X;
        double local_z = -Z * sin_p - Y * cos_p;

        Eigen::Vector3d local_pt(local_x, local_y, local_z);
        Eigen::Vector3d global_pt = rot_matrix * local_pt + location;

        out_cloud.emplace_back(global_pt.x(), global_pt.y(), global_pt.z());

                // Colors
        auto color = color_image.at(v, u);
                // OpenCV is BGR by default, swap to RGB
        out_colors.push_back({color[2], color[1], color[0]});
      }
    }
  }
}

void MapUpdate::process_frame(
  const Eigen::Vector3d & location, double yaw, double pitch, double roll,
  const ColorImage & rgb_frame, const DepthImage & depth_map)
{
    // Apply camera offset exactly as in Python
  Eigen::Vector3d offset(0.35, 0.1125 + 0.275, -0.5);
  Eigen::Vector3d cam_location = location + offset;

    // Rotation Matrix
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix <<
    cy * cp, cy *sp *sr - sy * cr, cy *sp *cr + sy * sr,
    sy *cp, sy *sp *sr + cy * cr, sy *sp *cr - cy * sr,
    -sp, cp *sr, cp *cr;

  Cloud cloud;
  Colors colors;

    // Pass to projection
  generate_global_point_cloud(depth_map, rgb_frame, cam_location, rotation_matrix, cloud, colors);

  if (cloud.size() > 0) {
    writer_.insert(cloud, colors, cam_location, config_.max_depth);

    size_t current_leaves = reader_.count();
    size_t exploration_score = current_leaves - previous_leaves_;
    (void)exploration_score;
    previous_leaves_ = current_leaves;

        // You can log exploration_score here as you did with the Python file writing
  }
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector4d>>
CandidatePlanner::generate_candidate_viewpoints(const Eigen::Vector3d & current_position)
{
  Eigen::Vector3d current_pos_offset = current_position + Eigen::Vector3d(0.35, 0.1125 + 0.275,
      -0.5);

  {std::ostringstream message;
    message << "\n[DEBUG 1] Starting generate_candidate_viewpoints." << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message   << "[DEBUG 1] Current Offset Pos: [" << current_pos_offset.x() << ", "
              << current_pos_offset.y() << ", " <<
      current_pos_offset.z() << "]" << std::endl; diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector3d> occupied_points;
  std::vector<Eigen::Vector3d> empty_points;
  VoxelSet occ_set;

  int close_points_filtered = 0;

    // Extract points directly from OctoMap
  for (const auto & leaf : reader_.leaves()) {
    Eigen::Vector3d pt = leaf.point;

    if ((pt - current_pos_offset).norm() < 0.6) {
      close_points_filtered++;
      continue;
    }

    if (leaf.occupied) {
      occupied_points.push_back(pt);
      occ_set.insert({
          static_cast<int>(std::floor(pt.x() / resolution_)),
          static_cast<int>(std::floor(pt.y() / resolution_)),
          static_cast<int>(std::floor(pt.z() / resolution_))
            });
    } else {
      empty_points.push_back(pt);
    }
  }

  {std::ostringstream message; message << "[DEBUG 2] OctoMap Extraction:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Filtered (dist < 0.6): " << close_points_filtered << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Occupied points: " << occupied_points.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message; message << "  -> Empty points: " << empty_points.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

  if (occupied_points.empty()) {
    {std::ostringstream message;
      message << "[WARN] No occupied points found! Aborting." << std::endl;
      diagnostics_.text(message.str(), false, true);}
    return {{}, {}};
  }

    // ==========================================
    // 1. Calculate Information Gain (Surface Sparsity via PCL)
    // ==========================================
  std::vector<Eigen::Vector3d> occupied_for_index;
  for (const auto & p : occupied_points) {
    if (p.z() > config_.z_min_target) {     // z_min_target
      occupied_for_index.push_back(p);
    }
  }

  auto occ_cloud = indexes_.create(occupied_for_index);

  {std::ostringstream message;
    message << "[DEBUG 3] Z-Filtering (Threshold: " << config_.z_min_target << "):" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Valid points remaining: " << occ_cloud->size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector3d> frontier_centers;
  std::vector<double> frontier_scores;

  if (occ_cloud->size() > 10) {


    std::vector<int> pointIdx(11);
    std::vector<float> pointDistSq(11);

    double max_rho = 0.0, min_rho = 1e9;
    std::vector<double> rhos(occ_cloud->size());

        // Calculate sparsity (rho) for each point
    for (size_t i = 0; i < occ_cloud->size(); ++i) {
      if (occ_cloud->nearest(i, 11, pointIdx, pointDistSq) > 0) {
        double mean_dist = 0.0;
        for (size_t j = 1; j < pointIdx.size(); ++j) {
          mean_dist += std::sqrt(pointDistSq[j]);
        }
        mean_dist /= 10.0;
        rhos[i] = mean_dist;
        max_rho = std::max(max_rho, mean_dist);
        min_rho = std::min(min_rho, mean_dist);
      }
    }

    std::vector<Eigen::Vector3d> free_for_index;
    for (const auto & p : empty_points) {
      free_for_index.push_back(p);
    }
    const bool has_free = !free_for_index.empty();
    auto free_cloud = indexes_.create(free_for_index);

    for (size_t i = 0; i < occ_cloud->size(); ++i) {
      double base_score = (rhos[i] - min_rho) / (max_rho - min_rho + 1e-6);
      if (base_score > config_.info_gain_threshold &&
        rhos[i] > (config_.resolution * config_.sparsity_multiplier))
      {

        double adjusted_score = base_score;
        if (has_free) {
          std::vector<int> freeIdx;
          std::vector<float> freeDistSq;
          int free_counts = free_cloud->radius(occ_cloud->point(i), config_.resolution * 3.0,
              freeIdx, freeDistSq);
          double damping = std::exp(-0.05 * free_counts);
          adjusted_score *= damping;
        }

        if (adjusted_score >= 0.05) {
          frontier_centers.push_back(Eigen::Vector3d(occ_cloud->point(i).x(),
              occ_cloud->point(i).y(), occ_cloud->point(i).z()));
          frontier_scores.push_back(adjusted_score);
        }
      }
    }
  }

  {std::ostringstream message; message << "[DEBUG 4] Frontier Extraction:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> High-value frontiers found: " << frontier_centers.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

    // ==========================================
    // 2. Evaluate Utility & Generate Viewpoints
    // ==========================================
    // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> candidates;
    // double alpha_cost = 1.0;
    // double min_utility_threshold = config_.min_utility_threshold;

    // std::vector<double> dists = {1.5, 1.2, 1.8, 1.0};
    // std::vector<double> pitches = {45.0, 60.0, 30.0, 75.0, 15.0};

    // int skipped_wall = 0;
    // int skipped_los = 0;
    // int skipped_utility = 0;

    // for (size_t i = 0; i < frontier_centers.size(); ++i) {
    //     Eigen::Vector3d f = frontier_centers[i];
    //     Eigen::Vector3d vec = f - current_pos_offset;
    //     vec.z() = 0;

    //     double norm = vec.norm();
    //     Eigen::Vector3d dir_xy = (norm < 1e-5) ? Eigen::Vector3d(1, 0, 0) : (vec / norm);

    //     bool found_view = false;
    //     for (double dist : dists) {
    //         if (found_view) break;
    //         for (double p_deg : pitches) {
    //             double pitch = p_deg * M_PI / 180.0;
    //             Eigen::Vector3d viewpoint = f - dir_xy * (dist * std::cos(pitch)) + Eigen::Vector3d(0, 0, dist * std::sin(pitch));

    //             VoxelIdx vp_idx = {
    //                 static_cast<int>(std::floor(viewpoint.x() / config_.resolution)),
    //                 static_cast<int>(std::floor(viewpoint.y() / config_.resolution)),
    //                 static_cast<int>(std::floor(viewpoint.z() / config_.resolution))
    //             };

    //             if (occ_set.count(vp_idx)) {
    //                 skipped_wall++;
    //                 continue;
    //             }

    //             if (SearchAlgo::check_line_of_sight(viewpoint, f, occ_set, resolution_)) {
    //                 double travel_cost = (viewpoint - current_pos_offset).norm() + 0.1;
    //                 double utility = frontier_scores[i] / (alpha_cost * travel_cost);

    //                 if (utility >= min_utility_threshold) {
    //                     candidates.push_back({viewpoint, f});
    //                     found_view = true;
    //                     break;
    //                 } else {
    //                     skipped_utility++;
    //                 }
    //             } else {
    //                 skipped_los++;
    //             }
    //         }
    //     }
    // }

    // std::cout << "[DEBUG 5] Candidate Generation:" << std::endl;
    // std::cout << "  -> Rejected (Inside Wall): " << skipped_wall << std::endl;
    // std::cout << "  -> Rejected (No Line of Sight): " << skipped_los << std::endl;
    // std::cout << "  -> Rejected (Low Utility): " << skipped_utility << std::endl;
    // std::cout << "  -> Accepted Viewpoints: " << candidates.size() << std::endl;


  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> candidates;
  double alpha_cost = 1.0;
  double min_utility_threshold = config_.min_utility_threshold;

  std::vector<double> dists = {1.5, 2.0, 1.2};   // Keep it close for good stereo disparity
  std::vector<double> pitches = {30.0, 45.0, 60.0};

    // 8 points around the compass (45-degree increments)
  std::vector<double> yaws = {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0};

  int skipped_wall = 0;
  int skipped_los = 0;
  int skipped_utility = 0;

  for (size_t i = 0; i < frontier_centers.size(); ++i) {
    Eigen::Vector3d f = frontier_centers[i];

    bool found_view = false;

        // Search 360 degrees around the frontier for a valid viewing angle!
    for (double yaw_deg : yaws) {
      if (found_view) {break;}

      double yaw_rad = yaw_deg * M_PI / 180.0;
      Eigen::Vector3d dir_xy(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);

      for (double dist : dists) {
        if (found_view) {break;}
        for (double p_deg : pitches) {
          double pitch = p_deg * M_PI / 180.0;

                    // Generate viewpoint independent of AUV's current location
          Eigen::Vector3d viewpoint = f - dir_xy * (dist * std::cos(pitch)) + Eigen::Vector3d(0, 0,
              dist * std::sin(pitch));

          VoxelIdx vp_idx = {
            static_cast<int>(std::floor(viewpoint.x() / config_.resolution)),
            static_cast<int>(std::floor(viewpoint.y() / config_.resolution)),
            static_cast<int>(std::floor(viewpoint.z() / config_.resolution))
          };

          if (occ_set.count(vp_idx)) {
            skipped_wall++;
            continue;
          }

          if (SearchPolicy::check_line_of_sight(viewpoint, f, occ_set, config_.resolution)) {
                        // Calculate travel cost from the AUV's actual position
            double travel_cost = (viewpoint - current_pos_offset).norm() + 0.1;
            double utility = frontier_scores[i] / (alpha_cost * travel_cost);

            if (utility >= min_utility_threshold) {
              candidates.push_back({viewpoint, f});
              found_view = true;
              break;               // Move to the next frontier!
            } else {
              skipped_utility++;
            }
          } else {
            skipped_los++;
          }
        }
      }
    }
  }

    // ==========================================
    // 2.7 FOV-BASED VIEWPOINT CULLING
    // ==========================================
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> optimized_candidates;
  if (!candidates.empty()) {
    double fov_rad = 80.0 * M_PI / 180.0;
    double max_view_range = 3.0;
    std::unordered_set<VoxelIdx, VoxelHash> covered_frontiers;

    for (const auto & [vp, target] : candidates) {
      VoxelIdx target_idx = {
        static_cast<int>(std::round(target.x() * 1000)),
        static_cast<int>(std::round(target.y() * 1000)),
        static_cast<int>(std::round(target.z() * 1000))
      };

      if (covered_frontiers.count(target_idx)) {continue;}
      optimized_candidates.push_back({vp, target});

      Eigen::Vector3d forward_vec = target - vp;
      double dist_to_target = forward_vec.norm();
      if (dist_to_target < 1e-5) {continue;}
      Eigen::Vector3d forward_dir = forward_vec / dist_to_target;

      for (const auto & f : frontier_centers) {
        VoxelIdx f_idx = {
          static_cast<int>(std::round(f.x() * 1000)),
          static_cast<int>(std::round(f.y() * 1000)),
          static_cast<int>(std::round(f.z() * 1000))
        };

        if (covered_frontiers.count(f_idx)) {continue;}

        Eigen::Vector3d vec_to_f = f - vp;
        double dist_to_f = vec_to_f.norm();

        if (dist_to_f <= max_view_range) {
          Eigen::Vector3d dir_to_f = vec_to_f / dist_to_f;
          double cos_angle = std::clamp(forward_dir.dot(dir_to_f), -1.0, 1.0);
          double angle = std::acos(cos_angle);

          if (angle <= (fov_rad / 2.0)) {
            covered_frontiers.insert(f_idx);
          }
        }
      }
    }
    candidates = optimized_candidates;
  }

  {std::ostringstream message; message << "[DEBUG 6] FOV Culling:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Optimized Viewpoints: " << candidates.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

    // Route Planning & Smoothing Phase
  auto [full_path, target_tracking] = route_.plan_multi_waypoint_route(current_pos_offset,
      candidates, occ_set);

  {std::ostringstream message; message << "[DEBUG 7] A* Routing:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Path nodes generated: " << full_path.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector4d> smooth_traj = route_.generate_seamless_mission_trajectory(full_path,
      target_tracking, 200);

  {std::ostringstream message; message << "[DEBUG 8] Spline Smoothing:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Final smooth waypoints: " << smooth_traj.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message; message << "=======================================\n" << std::endl;
    diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector3d> raw_viewpoints;
  for (const auto & c : candidates) {
    raw_viewpoints.push_back(c.first);
  }

  return {raw_viewpoints, smooth_traj};
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
RoutePlanner::plan_multi_waypoint_route(
  const Eigen::Vector3d & current_location,
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & candidates,
  const VoxelSet & occ_set)
{
  if (candidates.empty()) {return Route{};}

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> unvisited = candidates;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ordered_candidates;
  Eigen::Vector3d curr_pos = current_location;

    // Greedy TSP
  while (!unvisited.empty()) {
    auto best_it = unvisited.begin();
    double min_dist = 1e9;

    for (auto it = unvisited.begin(); it != unvisited.end(); ++it) {
      double dist = (curr_pos - it->first).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_it = it;
      }
    }
    ordered_candidates.push_back(*best_it);
    curr_pos = best_it->first;
    unvisited.erase(best_it);
  }

  std::vector<Eigen::Vector3d> full_path;
  std::vector<Eigen::Vector3d> target_tracking;
  Eigen::Vector3d start_pos = current_location;

    // A* Path Concatenation
  for (const auto & vp_target : ordered_candidates) {
    {std::ostringstream message;
      message << "Planning path from " << start_pos.transpose() << " to " <<
        vp_target.first.transpose() << std::endl; diagnostics_.text(message.str(), false, true);}
    std::vector<Eigen::Vector3d> segment = SearchPolicy::astar_3d(
            start_pos, vp_target.first, occ_set, resolution_, 50000,
      [this](const std::string & message) {diagnostics_.text(message, true, false);});

    if (segment.empty()) {continue;}

    if (!full_path.empty() && (full_path.back() - segment.front()).norm() < 1e-4) {
      segment.erase(segment.begin());
    }

    full_path.insert(full_path.end(), segment.begin(), segment.end());
    for (size_t i = 0; i < segment.size(); ++i) {
      target_tracking.push_back(vp_target.second);
    }
    start_pos = vp_target.first;
  }

  return {full_path, target_tracking};
}

Eigen::Vector4d catmull_rom(
  const Eigen::Vector4d & p0, const Eigen::Vector4d & p1,
  const Eigen::Vector4d & p2, const Eigen::Vector4d & p3, double t)
{
  double t2 = t * t;
  double t3 = t2 * t;
  return 0.5 * ((2.0 * p1) +
         (-p0 + p2) * t +
         (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
         (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

double unwrap_angle(double previous_angle, double new_angle)
{
  double d = new_angle - previous_angle;
  d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
  return previous_angle + d;
}

std::vector<Eigen::Vector4d> RoutePlanner::generate_seamless_mission_trajectory(
  const std::vector<Eigen::Vector3d> & full_path,
  const std::vector<Eigen::Vector3d> & target_tracking,
  int num_points)
{
  if (full_path.size() < 4) {
    std::vector<Eigen::Vector4d> traj_xyz_yaw;
    for(size_t i = 0; i < full_path.size(); ++i) {
      double yaw = std::atan2(target_tracking[i].y() - full_path[i].y(),
                                    target_tracking[i].x() - full_path[i].x());
            // Add full_path[i].z() here
      traj_xyz_yaw.push_back(Eigen::Vector4d(full_path[i].x(), full_path[i].y(), full_path[i].z(),
          yaw));
    }
    return traj_xyz_yaw;
  }

    // 1. Prepare 4D Control Points [X, Y, Z, Yaw]
  std::vector<Eigen::Vector4d> control_points;
  double prev_yaw = 0.0;

  for (size_t i = 0; i < full_path.size(); ++i) {
    double raw_yaw = std::atan2(target_tracking[i].y() - full_path[i].y(),
                                    target_tracking[i].x() - full_path[i].x());

    if (i > 0) {raw_yaw = unwrap_angle(prev_yaw, raw_yaw);}
    prev_yaw = raw_yaw;

    control_points.push_back(Eigen::Vector4d(full_path[i].x(), full_path[i].y(), full_path[i].z(),
        raw_yaw));
  }

    // 2. Generate smooth spline (FIXED)
  std::vector<Eigen::Vector4d> trajectory;
  int segments = control_points.size() - 3;

    // Safety check just in case
  if (segments < 1) {return control_points;}

    // Distribute exactly 'num_points' evenly across the entire spline
  for (int i = 0; i < num_points; ++i) {
        // Find where we are globally on the path (0.0 to segments)
    double global_t = (static_cast<double>(i) / std::max(1, num_points - 1)) * segments;

        // Find which specific segment we are inside
    int segment_idx = std::min(static_cast<int>(global_t), segments - 1);

        // Find our local progress inside that specific segment (0.0 to 1.0)
    double local_t = global_t - segment_idx;

    Eigen::Vector4d pt = catmull_rom(
            control_points[segment_idx], control_points[segment_idx + 1],
            control_points[segment_idx + 2], control_points[segment_idx + 3], local_t);

        // Re-wrap yaw to [-pi, pi] for ROS Odometry standards
    double final_yaw = std::fmod(pt.w() + M_PI, 2.0 * M_PI);
    if (final_yaw < 0) {final_yaw += 2.0 * M_PI;}
    final_yaw -= M_PI;

    trajectory.push_back(Eigen::Vector4d(pt.x(), pt.y(), pt.z(), final_yaw));
  }

  return trajectory;
}
}
````

### NBV_paper_ros2/src/auv_inspection_planner/src/realtime_stereo_mapper.cpp

SHA-256: `4c521620f07110f9a15d812e13294df33be9de0cd3fbaaddcd201d8575453dcb`

````cpp
#include "realtime_stereo_mapper.hpp"
````

### NBV_paper_ros2/src/auv_inspection_planner/src/realtime_stereo_mapper.hpp

SHA-256: `9f04c410dca17070e2540f74ff81a0a391752f90a2cf3d3839516e109359ee0a`

````cpp
#pragma once
// Policy types are independently reusable; composition is in nbv_utility_node.cpp.
#include "auv_inspection_planner/use_cases/mapping.hpp"
````

### NBV_paper_ros2/src/auv_inspection_planner/src/search_algo.cpp

SHA-256: `ca660bc05f089073b4f0c1b8c06481091cfdf97d9fa2d0a5789b264b76309cd5`

````cpp
#include "auv_inspection_planner/domain/search.hpp"

// --- Helper structures for A* ---
struct AStarNode
{
  VoxelIdx idx;
  double f_score;
  bool operator>(const AStarNode & o) const {return f_score > o.f_score;}
};

inline double heuristic(const VoxelIdx & a, const VoxelIdx & b)
{
    // Manhattan distance is optimal for 6-way voxel movement
  return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
}

bool find_nearest_free_voxel(
  const VoxelIdx & origin, const VoxelSet & occ_set, int max_radius, VoxelIdx & result)
{
  if (!occ_set.count(origin)) {
    result = origin;
    return true;
  }

  std::queue<std::pair<VoxelIdx, int>> pending;
  VoxelSet visited;
  pending.push({origin, 0});
  visited.insert(origin);

  const std::vector<VoxelIdx> directions = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  while (!pending.empty()) {
    const auto [current, distance] = pending.front();
    pending.pop();

    if (distance >= max_radius) {
      continue;
    }

    for (const auto & direction : directions) {
      VoxelIdx neighbor = {
        current.x + direction.x,
        current.y + direction.y,
        current.z + direction.z};

      if (!visited.insert(neighbor).second) {
        continue;
      }

      if (!occ_set.count(neighbor)) {
        result = neighbor;
        return true;
      }

      pending.push({neighbor, distance + 1});
    }
  }

  return false;
}

std::vector<Eigen::Vector3d> SearchPolicy::astar_3d(
  const Eigen::Vector3d & start_pos, const Eigen::Vector3d & goal_pos,
  const VoxelSet & occ_set, double res, int max_steps,
  const std::function<void(const std::string &)> & failure)
{
  VoxelIdx start_idx = {
    static_cast<int>(std::floor(start_pos.x() / res)),
    static_cast<int>(std::floor(start_pos.y() / res)),
    static_cast<int>(std::floor(start_pos.z() / res))};
  VoxelIdx goal_idx = {
    static_cast<int>(std::floor(goal_pos.x() / res)),
    static_cast<int>(std::floor(goal_pos.y() / res)),
    static_cast<int>(std::floor(goal_pos.z() / res))};

  constexpr int endpoint_search_radius = 5;
  VoxelIdx free_start_idx;
  VoxelIdx free_goal_idx;
  const bool start_was_occupied = occ_set.count(start_idx) != 0;
  const bool goal_was_occupied = occ_set.count(goal_idx) != 0;

  if (!find_nearest_free_voxel(start_idx, occ_set, endpoint_search_radius, free_start_idx) ||
    !find_nearest_free_voxel(goal_idx, occ_set, endpoint_search_radius, free_goal_idx))
  {
        // std::cerr << "A* Error: Start or Goal is inside an obstacle!\n";
    return {};
  }

    // if (start_was_occupied)
    // {
    //     std::cerr << "A* Warning: shifted occupied start voxel to a nearby free voxel.\n";
    // }
    // if (goal_was_occupied)
    // {
    //     std::cerr << "A* Warning: shifted occupied goal voxel to a nearby free voxel.\n";
    // }

  start_idx = free_start_idx;
  goal_idx = free_goal_idx;

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
  open_set.push({start_idx, 0.0});

  std::unordered_map<VoxelIdx, VoxelIdx, VoxelHash> came_from;
  std::unordered_map<VoxelIdx, double, VoxelHash> g_score;
  g_score[start_idx] = 0.0;

  std::vector<VoxelIdx> directions = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  int steps = 0;
  while (!open_set.empty() && steps < max_steps) {
    steps++;
    VoxelIdx current = open_set.top().idx;
    open_set.pop();

    if (current == goal_idx) {
      std::vector<Eigen::Vector3d> path;
      VoxelIdx curr = current;
      while (came_from.find(curr) != came_from.end()) {
        path.push_back({(curr.x + 0.5) * res, (curr.y + 0.5) * res, (curr.z + 0.5) * res});
        curr = came_from[curr];
      }
      path.push_back({(start_idx.x + 0.5) * res, (start_idx.y + 0.5) * res,
          (start_idx.z + 0.5) * res});
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const auto & d : directions) {
      VoxelIdx neighbor = {current.x + d.x, current.y + d.y, current.z + d.z};

      if (occ_set.count(neighbor)) {
        continue;
      }

      double tentative_g = g_score[current] + 1.0;

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal_idx);
        open_set.push({neighbor, f});
      }
    }
  }
  if (failure) {failure("A* failed to find a path.\n");}
  return {};
}

bool SearchPolicy::check_line_of_sight(
  const Eigen::Vector3d & p_start, const Eigen::Vector3d & p_end,
  const VoxelSet & occ_set, double res, int gap_multiplier)
{
  Eigen::Vector3d ray_vec = p_end - p_start;
  double dist = ray_vec.norm();
  int steps = static_cast<int>(dist / res);
  int radius = gap_multiplier / 2;

  if (steps <= radius + 1) {
    return true;
  }

    // Precompute spherical offsets
  std::vector<VoxelIdx> offsets;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (dx * dx + dy * dy + dz * dz <= radius * radius) {
          offsets.push_back({dx, dy, dz});
        }
      }
    }
  }

    // Walk along the line
  for (int i = 1; i < steps - radius; ++i) {
    Eigen::Vector3d p = p_start + (ray_vec * (static_cast<double>(i) / steps));
    VoxelIdx base_idx = {
      static_cast<int>(std::floor(p.x() / res)),
      static_cast<int>(std::floor(p.y() / res)),
      static_cast<int>(std::floor(p.z() / res))};

    for (const auto & offset : offsets) {
      VoxelIdx check_idx = {base_idx.x + offset.x, base_idx.y + offset.y, base_idx.z + offset.z};
      if (occ_set.count(check_idx)) {
        return false;         // Collision in the gap
      }
    }
  }
  return true;
}
````

### NBV_paper_ros2/src/auv_inspection_planner/src/search_algo.hpp

SHA-256: `0a36cbf5d3cdb9babd5272043b04e791cf4702d8812a8756cf5f2e67af80a539`

````cpp
#pragma once
#include "auv_inspection_planner/domain/search.hpp"
#include <iostream>

// Preserve the original standalone API and its stderr diagnostics at the outer boundary.
class SearchAlgo : public SearchPolicy {
public:
  static std::vector<Eigen::Vector3d> astar_3d(
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
    const VoxelSet & occupied, double resolution, int max_steps = 50000)
  {
    return SearchPolicy::astar_3d(start, goal, occupied, resolution, max_steps,
             [](const std::string & message) {std::cerr << message;});
  }
};
````

### NBV_paper_ros2/src/auv_inspection_planner/test/policy_regression.cpp

SHA-256: `fae9cbba093bfad8797e34c3f8b9e27f5c92cfdf42226ac1a69403edb4316e23`

````cpp
#include <cmath>
#include <cstdlib>
#include "auv_inspection_planner/use_cases/mapping.hpp"

void require(bool condition) {if(!condition) {std::abort();}}
struct Map final : inspection::MapReader, inspection::MapWriter
{
  inspection::Cloud cloud; inspection::Colors colors;
  Eigen::Vector3d origin; double range = 0.;
  std::vector<inspection::MapLeaf> leaves() const override {return {};}
  std::size_t count() const override {return cloud.size();}
  void insert(
    const inspection::Cloud & points, const inspection::Colors & rgb,
    const Eigen::Vector3d & location, double max_range) override
  {
    cloud = points; colors = rgb; origin = location; range = max_range;
  }
};
int main()
{
  Map map;
  const inspection::MapperConfig config;
  inspection::MapUpdate update(config, map, map);
  update.process_frame(Eigen::Vector3d(10., 20., -30.), 0., 0., 0.,
    inspection::ColorImage{2, 2, {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}},
    inspection::DepthImage{2, 2, {.5f, 4.f, 0.f, 2.f}});
  require(map.cloud.size() == 1 && map.colors[0] == std::array<unsigned char, 3>({12, 11, 10}));
  require((map.origin - Eigen::Vector3d(10.35, 20.3875, -30.5)).norm() < 1e-12);
  require((map.cloud[0].cast<double>() - Eigen::Vector3d(10.35 + std::sqrt(2.), 20.3875,
    -30.5 - std::sqrt(2.))).norm() < 3e-6);
  require(map.range == 4.);
  const auto path = SearchPolicy::astar_3d(Eigen::Vector3d(.05, .05, .05),
      Eigen::Vector3d(.25, .05, .05), {}, .1);
  require(path.size() == 3);
  require((path.front() - Eigen::Vector3d(.05, .05, .05)).norm() < 1e-12);
  require((path.back() - Eigen::Vector3d(.25, .05, .05)).norm() < 1e-12);
  require(SearchPolicy::check_line_of_sight(path.front(), path.back(), {}, .1));
  std::string error;
  require(SearchPolicy::astar_3d(path.front(), path.back(), {}, .1, 0,
    [&error](const std::string & value){error = value;}).empty());
  require(error == "A* failed to find a path.\n");
}
````

### NBV_paper_ros2/src/auv_contracts/auv_contracts/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_contracts/auv_contracts/diagnostics.py

SHA-256: `41d55cdd079a498d2296609e9ff8df72bc5d147545f2aaa16c6144514cd1d6f0`

````python
import math

def json_safe(value):
    """Retain nonfinite diagnostic values without producing invalid JSON."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if hasattr(value, 'tolist'):
        return json_safe(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value
````

### NBV_paper_ros2/src/auv_contracts/auv_contracts/mission_results.py

SHA-256: `9bdbf95d38dc862f0d3b74ef4b673b1aaec1379ca25b49e84b25c628e0c01263`

````python
"""Version 1 of the existing human-readable mission result protocol.

The filename and text are retained for existing tools. Both producer and consumer
use this module so wording changes cannot silently diverge between components.
"""
SCHEMA_VERSION = 1
MISSION_STATS_FILENAME = 'optuna_mission_stats.txt'
DISTANCE_LABEL = 'Total Distance Traveled:'


def distance_record(distance):
    return f'{DISTANCE_LABEL} {distance:.2f} meters\n'


def distance_from_records(records):
    distance = 999.0
    for line in records:
        if DISTANCE_LABEL in line:
            distance = float(line.split(':')[1].replace('meters', '').strip())
    return distance
````

### NBV_paper_ros2/src/auv_contracts/auv_contracts/ports.py

SHA-256: `17d35960ca493066a989f97ad2a59e0e9c40ee7679d92d84bc1644f87e550c0a`

````python
"""Small policy-owned protocols; implementations live in outer components."""
from typing import Protocol, Any, Callable, Mapping, Sequence

class Diagnostics(Protocol):
    def info(self, message: str, **options) -> None: ...
    def warning(self, message: str, **options) -> None: ...
    def error(self, message: str, **options) -> None: ...
    def debug(self, message: str, **options) -> None: ...

class Clock(Protocol):
    def time(self) -> float: ...
    def perf_counter(self) -> float: ...

class PipelineTimingSink(Protocol):
    def record_stage(self, stage, duration_ms, iteration=None, frame_id=None,
                     source_stamp_s=None, details=None) -> None: ...

class FilterEventSink(Protocol):
    def record_filter(self, stage, input_count, output_count, details,
                      iteration=None, frame_id=None) -> None: ...

class TrajectorySink(Protocol):
    def record_trajectory(self, time_s, position, cumulative_distance) -> None: ...

class MissionTelemetry(PipelineTimingSink, FilterEventSink, Protocol):
    def record_depth_pose(self, sequence, time_s, position, roll, pitch, yaw) -> None: ...
    def record_iteration(self, *args, **kwargs) -> None: ...

class OccupancyReader(Protocol):
    def extractPointCloud(self) -> Any: ...
    def getNumLeafNodes(self) -> int: ...

class MapIntegrator(Protocol):
    def insertPointCloud(self, *args, **kwargs) -> Any: ...

class MapSnapshotStore(Protocol):
    def write(self, path: str) -> Any: ...

class OccupancyMap(OccupancyReader, MapIntegrator, MapSnapshotStore, Protocol):
    """Convenience composition for the mission that uses all three roles."""

class TextSink(Protocol):
    def write(self, text: str) -> Any: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...

class TrajectoryOutput(Protocol):
    def encode(self, trajectory, camera_offset) -> tuple[Any, Any]: ...
    def publish(self, values: Sequence[float]) -> None: ...

class CompletionOutput(Protocol):
    def publish(self, finished: bool) -> None: ...

class StereoDepthEstimator(Protocol):
    def compute(self, left, right, calibration) -> Any: ...
    def timings(self) -> Mapping[str, float]: ...

class Optimizer(Protocol):
    def minimize(self, objective: Callable, initial, **options) -> Any: ...

class PointCloudPreprocessor(Protocol):
    def prepare(self, points, position, parameters, timed) -> Any: ...

class TrialRunner(Protocol):
    def run(self, parameters: Mapping[str, float]) -> float | None: ...
````

### NBV_paper_ros2/src/auv_contracts/auv_contracts/values.py

SHA-256: `00941aac3b2a3bbe6b2a8157cd03c7a79b2970bd689851505aaa75c05eb55e56`

````python
"""Immutable values shared across policy boundaries; no middleware types."""
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Any

def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value

@dataclass(frozen=True)
class Configuration:
    values: Mapping[str, Any]
    def __post_init__(self):
        object.__setattr__(self, 'values', _freeze(dict(self.values)))
    def __getattr__(self, name):
        try:
            return self.values[name]
        except KeyError as error:
            raise AttributeError(name) from error

@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass(frozen=True)
class Stamp:
    sec: int
    nanosec: int

@dataclass(frozen=True)
class PoseSample:
    position: Vector3
    orientation: Quaternion
    stamp: Stamp

@dataclass(frozen=True)
class Trajectory:
    points: tuple[tuple[float, float, float, float], ...]
    def __post_init__(self):
        object.__setattr__(self, 'points', tuple(tuple(p) for p in self.points))

@dataclass(frozen=True)
class ArrayEvent:
    data: tuple[float, ...]
    def __post_init__(self):
        object.__setattr__(self, 'data', tuple(self.data))

@dataclass(frozen=True)
class Flag:
    data: bool

@dataclass(frozen=True)
class CameraCalibration:
    focal_length_px: float = 256.0
    baseline_meters: float = 0.5

@dataclass(frozen=True)
class TrialResult:
    coverage: float
    rmse: float
    distance: float
````

### NBV_paper_ros2/src/auv_contracts/package.xml

SHA-256: `6dbc2af3733f687f663476149d5d5d18a11122a151ade39acc3cddc50399be95`

````xml
<?xml version="1.0"?>
<package format="3">
  <name>auv_contracts</name><version>0.0.0</version>
  <description>Framework-independent AUV values and client-owned boundary contracts</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <export><build_type>ament_python</build_type></export>
</package>
````

### NBV_paper_ros2/src/auv_contracts/resource/auv_contracts

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````text

````

### NBV_paper_ros2/src/auv_contracts/setup.cfg

SHA-256: `30321eab1c1ad3ef4f6d660b4bfb7441b1ac96f9a48973bc7dd79bf34db5319f`

````ini
[develop]
script_dir=$base/lib/auv_contracts
[install]
install_scripts=$base/lib/auv_contracts
````

### NBV_paper_ros2/src/auv_contracts/setup.py

SHA-256: `468cf9d0b2295ddace042ea972ae7290b52ecb68829c3304489b177a77da9cbe`

````python
from setuptools import find_packages, setup
setup(name='auv_contracts', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/auv_contracts']),
                  ('share/auv_contracts', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      description='Framework-independent AUV values and client-owned boundary contracts', license='TODO: License declaration')
````

### NBV_paper_ros2/src/auv_experiments/auv_experiments/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_experiments/auv_experiments/optuna_optimizer.py

SHA-256: `728dbb0980bf74bf9ebe1ec4b37f4d4d0dd870d657e4cc72b5db6fe3e407d84e`

````python
import optuna
from auv_benchmark.auv_benchmark import evaluate_map_accuracy
from .runner_adapter import RosTrialRunner
from .trial_policy import TrialObjective

objective = TrialObjective(RosTrialRunner(), evaluate_map_accuracy, print)

def main():
    study_name = "auv_nbv_optimization"

    # Robust storage with a 20-second timeout queue to prevent SQLite locks
    storage = optuna.storages.RDBStorage(
        url="sqlite:///optuna_auv_study.db",
        engine_kwargs={"connect_args": {"timeout": 20.0}}
    )

    study = optuna.create_study(
        study_name=study_name, 
        storage=storage, 
        directions=["maximize", "minimize"], 
        load_if_exists=True 
    )

    print(f"\nOptimization results will be saved to: sqlite:///optuna_auv_study.db")
    study.optimize(objective, n_trials=60)

    print("\n=======================================")
    print("Optimization Complete!")
    print("Best Trials on the Pareto Front:")
    for trial in study.best_trials:
        print(f"Trial {trial.number}: Coverage {trial.values[0]:.2f}%, Distance {trial.values[1]:.2f}m")
        print(f"  Params: {trial.params}")

    import pandas as pd
    df = study.trials_dataframe()
    df.to_csv("optuna_results_summary.csv")
    print("Saved summary to optuna_results_summary.csv")
if __name__ == '__main__':
    main()
````

### NBV_paper_ros2/src/auv_experiments/auv_experiments/runner_adapter.py

SHA-256: `dde2794244efb9213a1eeb805f136039937842b7014e25200fb59580218c63d2`

````python
import subprocess
import time
import signal
import os
from pathlib import Path
import yaml
from auv_bringup.configuration import apply_trial_overrides
from auv_contracts.mission_results import MISSION_STATS_FILENAME, distance_from_records


ROS_WORKSPACE = Path(__file__).resolve().parents[3]
BASE_PARAMS_PATH = ROS_WORKSPACE / 'src' / 'auv_bringup' / 'config' / 'params.yaml'
TEMP_PARAMS_PATH = ROS_WORKSPACE / 'temp_optuna_params.yaml'
MAP_PATH = ROS_WORKSPACE / 'results' / 'final_map_ros2.bt'
MISSION_STATS_PATH = ROS_WORKSPACE / 'results' / MISSION_STATS_FILENAME


def write_params_yaml(params_dict):
    """Write the current ROS config with the active planner overrides."""
    with BASE_PARAMS_PATH.open() as params_file:
        params = yaml.safe_load(params_file)

    apply_trial_overrides(params, params_dict, MAP_PATH, ROS_WORKSPACE / 'results')

    with TEMP_PARAMS_PATH.open('w') as params_file:
        yaml.safe_dump(params, params_file, sort_keys=False)


class RosTrialRunner:
    def run(self, params_dict):
        # Write to YAML so ROS 2 can read it
        write_params_yaml(params_dict)

        # Clean up old map files so we don't evaluate the previous run
        for output_path in (MAP_PATH, MISSION_STATS_PATH):
            output_path.unlink(missing_ok=True)

        # Launch the ROS 2 system
        cmd = (
            f"source /opt/ros/jazzy/setup.bash && "
            f"source {ROS_WORKSPACE}/install/setup.bash && "
            f"ros2 launch auv_bringup mapper.launch.py params_file:={TEMP_PARAMS_PATH}"
        )

        print("Launching ROS 2 stack...")
        process = subprocess.Popen(
            cmd,
            shell=True,
            executable='/bin/bash',
            preexec_fn=os.setsid
        )

        # Wait for the mission to finish (Timeout after 300 seconds)
        timeout = 300 
        start_time = time.time()
        mission_success = False

        while (time.time() - start_time) < timeout:
            if MISSION_STATS_PATH.exists():
                mission_success = True
                break
            time.sleep(2.0)

        # Kill the ENTIRE process group cleanly
        print("Terminating simulation run...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass 

        time.sleep(3.0) 
        mission_success = mission_success or MISSION_STATS_PATH.exists()

        # Evaluate Fitness
        if not mission_success:
            print("Trial timed out before results were saved. Penalizing heavily.")
            return None 

        with MISSION_STATS_PATH.open() as f:
            return distance_from_records(f)
````

### NBV_paper_ros2/src/auv_experiments/auv_experiments/trial_policy.py

SHA-256: `e07096de80a3ac791c6b7aa870736e83ed0e3d69f03391be0cc036bd98191c9e`

````python
from auv_contracts.ports import TrialRunner

class TrialObjective:
    def __init__(self, runner: TrialRunner, evaluate, report):
        self.runner = runner
        self.evaluate = evaluate
        self.report = report
    def __call__(self, trial):
        # Sample all 6 parameters based on the requested ranges
        params_dict = {
            'min_utility_threshold': trial.suggest_float('min_utility_threshold', 0.05, 0.35),
            'nbv_standoff': trial.suggest_float('nbv_standoff', 1.0, 2.5),
            'nbv_distance_weight': trial.suggest_float('nbv_distance_weight', 0.5, 2.0),
            'nbv_rotation_weight': trial.suggest_float('nbv_rotation_weight', 0.1, 1.0),
            'nbv_collision_weight': trial.suggest_float('nbv_collision_weight', 1.0, 5.0),
            'nbv_min_raw_gain': trial.suggest_float('nbv_min_raw_gain', 0.2, 1.0),
        }

        self.report(f"\n--- Starting Trial {trial.number} ---")
        self.report(f"Params: {params_dict}")

        total_distance = self.runner.run(params_dict)
        if total_distance is None:
            return 0.0, 999.0

        coverage, rmse = self.evaluate(headless=True)

        if rmse > 0.4:
            coverage = coverage * 0.5 

        self.report(f"Trial {trial.number} Results -> Coverage: {coverage:.2f}%, Distance: {total_distance:.2f}m")
        return coverage, total_distance
````

### NBV_paper_ros2/src/auv_experiments/package.xml

SHA-256: `c33cf2fc455327f4540e9c6fdf68ab50768e79036d979a668f11d7e6d632c08a`

````xml
<?xml version="1.0"?>
<package format="3">
  <name>auv_experiments</name><version>0.0.0</version>
  <description>Outer AUV experiment orchestration</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>auv_bringup</exec_depend>
  <exec_depend>auv_benchmark</exec_depend>
  <exec_depend>auv_contracts</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>python3-pandas</exec_depend>

  <export><build_type>ament_python</build_type></export>
</package>
````

### NBV_paper_ros2/src/auv_experiments/resource/auv_experiments

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````text

````

### NBV_paper_ros2/src/auv_experiments/setup.cfg

SHA-256: `890fdcb38448574de04f3370e4ff958a69a2732d7fa4c2bc1795cf59dbb08a85`

````ini
[develop]
script_dir=$base/lib/auv_experiments
[install]
install_scripts=$base/lib/auv_experiments
````

### NBV_paper_ros2/src/auv_experiments/setup.py

SHA-256: `ec6f1889ad06c1e0a4023a994a164b89426011e057e2e8354027e5ce3fce5f82`

````python
from setuptools import find_packages, setup
setup(name='auv_experiments', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/auv_experiments']),
                  ('share/auv_experiments', ['package.xml'])],
      install_requires=['setuptools', 'optuna', 'PyYAML', 'pandas'], zip_safe=True,
      entry_points={'console_scripts': ['optuna_optimizer = auv_experiments.optuna_optimizer:main']},
      description='Outer AUV experiment orchestration', license='TODO: License declaration')
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/adapters/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/adapters/map_geometry.py

SHA-256: `25d92dc43e2ecfb762582cbc49b8825340d0d91439174fba8f756bd1356f33c1`

````python
import numpy as np
import open3d as o3d
import pyoctomap
import matplotlib.pyplot as plt

class Open3DMapEvaluation:
    def __init__(self, config):
        self.config = config

    def prepare(self):
        print("=======================================")
        print("  AUV MAPPING BENCHMARK EVALUATOR")
        print("=======================================\n")

        # ==========================================
        # 1. LOAD AND PREPARE OCTOMAP
        # ==========================================
        print(f"[1/5] Loading OctoMap: {self.config.octomap_file.split('/')[-1]}")
        tree = pyoctomap.ColorOcTree(0.1) # Must match your MapperConfig resolution
        tree=tree.read(self.config.octomap_file)

        occupied_points, _, _ = tree.extractPointCloud()

        # Filter Map: Remove seafloor and stray noise below Z = -31.9 (giving a 10cm buffer)
        valid_map_points = occupied_points[occupied_points[:, 2] > (self.config.surface_z + 0.1)]
        # valid_map_points = occupied_points[
        #     (occupied_points[:, 2] > (self.config.surface_z + 0.1)) & 
        #     (occupied_points[:, 1] >= -84.75)
        # ]

        map_pcd = o3d.geometry.PointCloud()
        map_pcd.points = o3d.utility.Vector3dVector(valid_map_points)

        # Statistical Outlier Removal (Denoising)
        map_pcd, _ = map_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
        print(f"      -> Map points after filtering: {len(map_pcd.points)}")

        # ==========================================
        # 2. LOAD AND PREPARE GROUND TRUTH MESH
        # ==========================================
        print(f"[2/5] Loading Ground Truth: {self.config.ground_truth_file.split('/')[-1]}")
        gt_mesh = o3d.io.read_triangle_mesh(self.config.ground_truth_file)

        # Scale Unreal Engine cm to ROS meters
        gt_mesh.scale(0.01, center=(0, 0, 0))
        gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=1000000)

        gt_points = np.asarray(gt_pcd.points)

        # Fix Unreal Engine Left-Handed Y-Axis Mirroring
        gt_points[:, 1] = -gt_points[:, 1]

        # Filter GT: Remove the seafloor from the Unreal export
        gt_valid = gt_points[gt_points[:, 2] > (self.config.surface_z + 0.1)]
        gt_pcd.points = o3d.utility.Vector3dVector(gt_valid)
        print(f"      -> GT points after filtering: {len(gt_pcd.points)}")

        # ==========================================
        # 3. ALIGNMENT (INITIAL GUESS + ICP)
        # ==========================================
        print("[3/5] Aligning Coordinate Systems...")

        # Shift GT center to the known AUV global coordinates
        gt_current_center = gt_pcd.get_center()
        gt_pcd.translate(-gt_current_center) # Move to 0,0,0 temporarily

        # Move to the user's known global center
        # Note: Z is shifted slightly up so the pipe rests on the -32 surface
        gt_pcd.translate([self.config.obj_center_x, self.config.obj_center_y, self.config.surface_z + 1.0]) 

        # Run Fine Iterative Closest Point (ICP) for millimeter perfect matching
        print("      -> Running Point-to-Point ICP...")
        icp_result = o3d.pipelines.registration.registration_icp(
            map_pcd, gt_pcd, max_correspondence_distance=2.0,
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
        )

        # Apply the mathematical alignment to the map
        map_pcd.transform(icp_result.transformation)


        self.map_pcd, self.gt_pcd = map_pcd, gt_pcd

    def map_distances(self):
        return self.map_pcd.compute_point_cloud_distance(self.gt_pcd)
    def ground_truth_distances(self):
        return self.gt_pcd.compute_point_cloud_distance(self.map_pcd)
    def visualize(self, distances):
        map_pcd, gt_pcd = self.map_pcd, self.gt_pcd
        # ==========================================
        # 5. HEATMAP COLORIZATION & VISUALIZATION
        # ==========================================
        print("[5/5] Generating Error Heatmap...")

        # Color the Ground Truth a faint transparent grey
        gt_pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # Normalize Map distances for the Heatmap: Green (0.0m error) -> Red (0.2m error)
        max_error_viz = 0.2 
        normalized_errors = np.clip(distances / max_error_viz, 0, 1)

        cmap = plt.get_cmap("RdYlGn_r") # Red-Yellow-Green (Reversed)
        error_colors = cmap(normalized_errors)[:, :3]
        map_pcd.colors = o3d.utility.Vector3dVector(error_colors)

        print("Opening 3D Visualizer. Close the window to exit.")
        print("LEGEND: Green = Perfect Match | Red = Noisy/Inaccurate | Grey = Ground Truth")

        o3d.visualization.draw_geometries([gt_pcd, map_pcd], 
                                          window_name="AUV Mapping Benchmark",
                                          width=1280, height=720)

````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/adapters/reports.py

SHA-256: `b500f64f75fc3ba2f11eafc3a3ead31a89a60be7b7dbec3a6599c22a96b2b5a6`

````python
import csv
from datetime import datetime
from pathlib import Path
from ..plot_adapter import generate_results_plot
from auv_contracts.mission_results import MISSION_STATS_FILENAME, distance_record

class FileReportWriter:
    def __init__(self, results_dir, diagnostics):
        self.results_dir = results_dir
        self.results_path = Path(results_dir).expanduser()
        self.diagnostics = diagnostics

    def write(self, snapshot, total_time):

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_path = self.results_path
        csv_filename = results_path / f"trajectory_benchmark_{timestamp_str}.csv"
        # txt_filename = f"mission_stats_{timestamp_str}.txt"
        txt_filename = results_path / MISSION_STATS_FILENAME

        # 1. Save Trajectory CSV
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time_s', 'X_m', 'Y_m', 'Z_m', 'Cumulative_Distance_m'])
            for row in snapshot.trajectory_log:
                writer.writerow([f"{row[0]:.2f}", f"{row[1]:.3f}", f"{row[2]:.3f}", f"{row[3]:.3f}", f"{row[4]:.3f}"])

        # 2. Save Academic Stats Summary
        with open(txt_filename, 'w') as f:
            f.write("=== AUV Exploration Benchmark ===\n")
            f.write(f"Total Mission Time (Makespan): {total_time:.2f} seconds\n")
            f.write(distance_record(snapshot.total_distance))
            if total_time > 0:
                f.write(f"Average Flight Speed: {(snapshot.total_distance / total_time):.3f} m/s\n")
            f.write(f"Total Trajectory Waypoints Logged: {len(snapshot.trajectory_log)}\n")
            metrics, diagnostics_path = self._save_diagnostics(
                results_path, timestamp_str, snapshot)
            f.write(f"Peak Speed: {metrics['peak_speed_mps']:.3f} m/s\n")
            if metrics['mean_abs_heading_error_deg'] is None:
                f.write("Mean Absolute Heading Error: N/A\n")
                f.write("Maximum Absolute Heading Error: N/A\n")
            else:
                f.write(
                    "Mean Absolute Heading Error: "
                    f"{metrics['mean_abs_heading_error_deg']:.3f} deg\n")
                f.write(
                    "Maximum Absolute Heading Error: "
                    f"{metrics['max_abs_heading_error_deg']:.3f} deg\n")
            if metrics['trajectory_rmse_m'] is None:
                f.write("Planned vs Real Trajectory RMSE: N/A\n")
            else:
                f.write(
                    "Planned vs Real Trajectory RMSE: "
                    f"{metrics['trajectory_rmse_m']:.3f} m\n")

        self.diagnostics.info(
            f"💾 Saved {csv_filename}, {diagnostics_path}, and {txt_filename}")
        plot_path = generate_results_plot(
            self.results_dir, show_reference_objects=snapshot.show_reference_objects)
        if plot_path is not None:
            self.diagnostics.info(f'Saved mid-run overview plot to {plot_path}')


    def _save_diagnostics(self, results_path, timestamp_str, snapshot):
        diagnostics_path = results_path / f"trajectory_diagnostics_{timestamp_str}.csv"
        with diagnostics_path.open('w', newline='') as stream:
            fieldnames = [
                'Time_s', 'Speed_mps', 'Horizontal_Speed_mps',
                'Vertical_Speed_mps', 'Heading_Error_deg',
                'X_m', 'Y_m', 'Z_m', 'Planned_X_m', 'Planned_Y_m',
                'Planned_Z_m', 'Planned_Yaw_rad']
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for sample in snapshot.diagnostic_log:
                writer.writerow({
                    'Time_s': f"{sample['time_s']:.3f}",
                    'Speed_mps': f"{sample['speed_mps']:.5f}",
                    'Horizontal_Speed_mps': f"{sample['horizontal_speed_mps']:.5f}",
                    'Vertical_Speed_mps': f"{sample['vertical_speed_mps']:.5f}",
                    'Heading_Error_deg': '' if sample['heading_error_deg'] is None
                    else f"{sample['heading_error_deg']:.3f}",
                    'X_m': f"{sample['x_m']:.5f}",
                    'Y_m': f"{sample['y_m']:.5f}",
                    'Z_m': f"{sample['z_m']:.5f}",
                    'Planned_X_m': '' if sample['planned_x_m'] is None
                    else f"{sample['planned_x_m']:.5f}",
                    'Planned_Y_m': '' if sample['planned_y_m'] is None
                    else f"{sample['planned_y_m']:.5f}",
                    'Planned_Z_m': '' if sample['planned_z_m'] is None
                    else f"{sample['planned_z_m']:.5f}",
                    'Planned_Yaw_rad': '' if sample['planned_yaw_rad'] is None
                    else f"{sample['planned_yaw_rad']:.5f}",
                })

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.diagnostics.warning('Matplotlib unavailable; skipping diagnostic plots.')
            return snapshot.metrics, diagnostics_path

        times = [sample['time_s'] for sample in snapshot.diagnostic_log]
        speeds = [sample['speed_mps'] for sample in snapshot.diagnostic_log]
        horizontal_speeds = [
            sample['horizontal_speed_mps'] for sample in snapshot.diagnostic_log]
        vertical_speeds = [
            sample['vertical_speed_mps'] for sample in snapshot.diagnostic_log]
        heading_samples = [
            sample for sample in snapshot.diagnostic_log
            if sample['heading_error_deg'] is not None]

        figure, axis = plt.subplots(figsize=(11, 6), constrained_layout=True)
        axis.plot(times, speeds, label='3D speed', color='#1d4ed8')
        axis.plot(times, horizontal_speeds, label='Horizontal speed', color='#16a34a')
        axis.plot(times, vertical_speeds, label='Vertical speed', color='#dc2626')
        axis.set_title('Momentum protection: speed over time')
        axis.set_xlabel('Time (s)')
        axis.set_ylabel('Speed (m/s)')
        axis.grid(True, alpha=0.25)
        axis.legend()
        speed_plot_path = results_path / 'momentum_protection_speed_time.png'
        figure.savefig(speed_plot_path, dpi=160)
        plt.close(figure)

        if heading_samples:
            figure, axis = plt.subplots(figsize=(11, 6), constrained_layout=True)
            axis.plot(
                [sample['time_s'] for sample in heading_samples],
                [sample['heading_error_deg'] for sample in heading_samples],
                color='#7c3aed',
                label='Heading error',
            )
            axis.axhline(0.0, color='#374151', linewidth=0.8)
            axis.set_title('Heading error over time')
            axis.set_xlabel('Time (s)')
            axis.set_ylabel('Heading error (deg)')
            axis.grid(True, alpha=0.25)
            axis.legend()
            heading_plot_path = results_path / 'heading_error_time.png'
            figure.savefig(heading_plot_path, dpi=160)
            plt.close(figure)

        metrics = snapshot.metrics
        if metrics['trajectory_rmse_m'] is not None:
            planned_samples = [
                sample for sample in snapshot.diagnostic_log
                if sample['planned_x_m'] is not None]
            figure, axis = plt.subplots(figsize=(11, 7), constrained_layout=True)
            axis.plot(
                [sample['planned_x_m'] for sample in planned_samples],
                [sample['planned_y_m'] for sample in planned_samples],
                '--', color='#2563eb', label='Planned path')
            axis.plot(
                [sample['x_m'] for sample in planned_samples],
                [sample['y_m'] for sample in planned_samples],
                color='#111827', label='Real path')
            axis.set_title(
                f"Planned vs real trajectory (RMSE: "
                f"{metrics['trajectory_rmse_m']:.3f} m)")
            axis.set_xlabel('X (m)')
            axis.set_ylabel('Y (m)')
            axis.set_aspect('equal', adjustable='datalim')
            axis.grid(True, alpha=0.25)
            axis.legend()
            figure.savefig(results_path / 'planned_vs_real_trajectory.png', dpi=160)
            plt.close(figure)

        return metrics, diagnostics_path
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/auv_benchmark.py

SHA-256: `903c029455ca0922a236ea4bccb292ce1a4451f3fc144f54e4b8442928a2f047`

````python
"""Composition facade retaining the standalone evaluator API and defaults."""
if not __package__:
    # Retain direct file execution as well as normal package/module execution.
    import importlib.util
    from pathlib import Path
    import sys
    package_dir = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location(
        'auv_benchmark', package_dir / '__init__.py',
        submodule_search_locations=[str(package_dir)])
    package = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = package
    spec.loader.exec_module(package)
    __package__ = spec.name
from .domain.evaluation import MapEvaluationConfig

DEFAULT_CONFIG = MapEvaluationConfig(
    '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/results/final_map_ros2.bt',
    '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Tools/3D_scan/GT/objects/SM_MERGED_pipe_m_l100_390.OBJ')

def evaluate_map_accuracy(headless=False, config=DEFAULT_CONFIG):
    from .adapters.map_geometry import Open3DMapEvaluation
    from .use_cases.map_evaluation import EvaluateMap
    return EvaluateMap(Open3DMapEvaluation(config), print).execute(headless)

if __name__ == '__main__':
    evaluate_map_accuracy()
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/domain/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/domain/evaluation.py

SHA-256: `c489d7f453f8c4a3bf590ea9989e922187b3f40d2d11ccd4b9d1791cc34e87a0`

````python
from dataclasses import dataclass
from types import MappingProxyType
from typing import Protocol

@dataclass(frozen=True)
class ReportSnapshot:
    trajectory_log: tuple
    diagnostic_log: tuple
    total_distance: float
    metrics: object
    show_reference_objects: bool
    def __post_init__(self):
        object.__setattr__(self, 'trajectory_log', tuple(tuple(v) for v in self.trajectory_log))
        object.__setattr__(self, 'diagnostic_log', tuple(MappingProxyType(dict(v)) for v in self.diagnostic_log))
        object.__setattr__(self, 'metrics', MappingProxyType(dict(self.metrics)))

class ReportWriter(Protocol):
    def write(self, snapshot: ReportSnapshot, total_time: float) -> None: ...

class EvaluationGeometry(Protocol):
    def prepare(self): ...
    def map_distances(self): ...
    def ground_truth_distances(self): ...
    def visualize(self, distances): ...

@dataclass(frozen=True)
class MapEvaluationConfig:
    octomap_file: str
    ground_truth_file: str
    obj_center_x: float = -13.0
    obj_center_y: float = -81.9
    surface_z: float = -32.0
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/midrun_logger.py

SHA-256: `a2db5594d3169e701998342d5bc9068c208532ea4cdd19eb7e6002485768ca1f`

````python
"""Compatibility exports; storage and visualization have separate owners."""
from auv_telemetry.file_sinks import MidrunLogger
from .plot_adapter import generate_results_plot
from datetime import datetime

def default_run_id():
    return datetime.now().strftime('%Y%m%d_%H%M%S')
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/plot_adapter.py

SHA-256: `7a765a052a1bfdab2cb1baa0affcc353b40e6bdcb788c28f4675efbe98bf16cd`

````python
"""File-backed telemetry and plots for mid-mission NBV evaluation."""

import csv
import json
import math
from datetime import datetime
from pathlib import Path


def generate_results_plot(results_dir='results', prefix='benchmark',
                          planner_prefix='planner', show_reference_objects=True):
    """Create an XY overview with NBV scores, future routes, and camera cones."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Wedge
    except ImportError:
        return None

    root = Path(results_dir).expanduser()
    trajectory_path = root / f'{prefix}_trajectory.csv'
    depth_path = root / f'{planner_prefix}_depth_poses.csv'
    iteration_path = root / f'{planner_prefix}_nbv_iterations.jsonl'
    future_path = root / f'{planner_prefix}_future_trajectories.csv'
    if not trajectory_path.exists():
        return None

    trajectory = []
    with trajectory_path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            trajectory.append((float(row['x_m']), float(row['y_m'])))

    poses = []
    if depth_path.exists():
        with depth_path.open(newline='') as stream:
            poses = list(csv.DictReader(stream))

    candidates = []
    if iteration_path.exists():
        with iteration_path.open(encoding='utf-8') as stream:
            for line in stream:
                if line.strip():
                    candidates.extend(json.loads(line)['candidates'])

    figure, axis = plt.subplots(figsize=(12, 9), constrained_layout=True)
    if trajectory:
        axis.plot(*zip(*trajectory), color='#1f2937', linewidth=1.6,
                  label='Vehicle trajectory', zorder=2)
        axis.scatter(*trajectory[0], color='#16a34a', s=45, label='Start', zorder=4)
        axis.scatter(*trajectory[-1], color='#dc2626', s=45, label='Latest', zorder=4)

    if candidates:
        scores = [candidate['utility'] for candidate in candidates]
        scatter = axis.scatter(
            [candidate['position'][0] for candidate in candidates],
            [candidate['position'][1] for candidate in candidates],
            c=scores, cmap='viridis', s=34, alpha=0.8,
            label='NBV viewpoints (utility)', zorder=3)
        figure.colorbar(scatter, ax=axis, label='NBV utility')
        for candidate in candidates:
            trajectory_id = candidate.get('trajectory_id')
            if trajectory_id is not None:
                axis.annotate(
                    str(trajectory_id),
                    (candidate['position'][0], candidate['position'][1]),
                    xytext=(3, 3), textcoords='offset points',
                    fontsize=7, color='#111827', zorder=5)

    if show_reference_objects:
        axis.add_patch(Rectangle(
            (-14.1, -82.7), 2.1, 1.6,
            facecolor='#2563eb', edgecolor='#1e3a8a', alpha=0.16,
            linewidth=1.2, label='Rectangle object'))
        axis.add_patch(Rectangle(
            (-14.1, -87.4), 2.1, 1.6,
            facecolor='#2563eb', edgecolor='#1e3a8a', alpha=0.16,
            linewidth=1.2, label='Mirrored rectangle object'))
        pipe_objects = (
            ((-14.1, -81.1), (-15.7, -78.5)),
            ((-12.1, -81.1), (-11.0, -78.5)),
            ((-14.1, -88.4), (-15.7, -91.0)),
            ((-12.1, -88.4), (-11.0, -91.0)),
        )
        for index, (start, end) in enumerate(pipe_objects):
            axis.plot(
                [start[0], end[0]], [start[1], end[1]],
                color='#a16207', linewidth=7, alpha=0.22,
                solid_capstyle='round',
                label='Pipe objects' if index == 0 else (
                    'Mirrored pipe objects' if index == 2 else None))
            axis.plot(
                [start[0], end[0]], [start[1], end[1]],
                color='#92400e', linewidth=1.2, alpha=0.9,
                solid_capstyle='round')

    if future_path.exists():
        by_iteration = {}
        with future_path.open(newline='') as stream:
            for row in csv.DictReader(stream):
                by_iteration.setdefault(row['iteration'], []).append(row)
        for index, (iteration, rows) in enumerate(by_iteration.items()):
            axis.plot([float(row['x_m']) for row in rows],
                      [float(row['y_m']) for row in rows],
                      '--', linewidth=1.0, alpha=0.65,
                      label='Future trajectory' if index == 0 else None)

    for index, pose in enumerate(poses):
        x, y = float(pose['x_m']), float(pose['y_m'])
        yaw = math.degrees(float(pose['yaw_rad']))
        axis.add_patch(Wedge((x, y), 0.45, yaw - 25, yaw + 25,
                             facecolor='#f97316', alpha=0.12,
                             edgecolor='#f97316', linewidth=0.7,
                             label='Camera cone' if index == 0 else None))
        axis.plot(x, y, '.', color='#f97316', markersize=2)

    axis.set_title('NBV planning process overview')
    axis.set_xlabel('X (m)')
    axis.set_ylabel('Y (m)')
    axis.set_aspect('equal', adjustable='datalim')
    axis.grid(True, alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc='best')
    output_path = root / f'{prefix}_trajectory_overview.png'
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/run_baselines.py

SHA-256: `0e25d420cbf08b3851110cbb56161c92a30e2ed48a5c452a4a56e1e022336367`

````python
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from auv_contracts.values import PoseSample, Vector3, Quaternion, Stamp, ArrayEvent, Flag
from auv_telemetry.file_sinks import MidrunLogger
from .adapters.reports import FileReportWriter
from .use_cases.trajectory_evaluation import TrajectoryEvaluation

class BenchmarkLogger(Node):
    def __init__(self):
        super().__init__('benchmark_logger')
        results_dir = self.declare_parameter('results_dir', 'results').value
        show = self.declare_parameter('show_reference_objects', True).value
        self.evaluation = TrajectoryEvaluation(
            MidrunLogger(results_dir, trajectory_only=True),
            FileReportWriter(results_dir, self.get_logger()), time, self.get_logger(), show)
        self.odom_sub = self.create_subscription(Odometry, 'holocean/odom', self.odom_callback, 10)
        self.planned_trajectory_sub = self.create_subscription(
            Float32MultiArray, 'object/areaWaypoints', self.planned_trajectory_callback, 10)
        self.finish_sub = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_callback, 10)
        self.timer = self.create_timer(0.5, self.evaluation.log_trajectory_callback)
    def odom_callback(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
        self.evaluation.odom_callback(PoseSample(
            Vector3(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w), Stamp(t.sec, t.nanosec)))
    def planned_trajectory_callback(self, msg):
        self.evaluation.planned_trajectory_callback(ArrayEvent(tuple(msg.data)))
    def finish_callback(self, msg):
        self.evaluation.finish_callback(Flag(msg.data))
    def destroy_node(self):
        self.evaluation.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/use_cases/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/use_cases/map_evaluation.py

SHA-256: `30415b3ef01aafed365939958e4be595082539fb6e36ba38b1ec13da02deedd6`

````python
import numpy as np
from ..domain.evaluation import EvaluationGeometry

class EvaluateMap:
    def __init__(self, geometry: EvaluationGeometry, report):
        self.geometry = geometry
        self.report = report
    def execute(self, headless=False):
        self.geometry.prepare()
        # ==========================================
        # 4. CALCULATE ACADEMIC METRICS
        # ==========================================
        self.report("[4/5] Calculating Paper Metrics...")

        # A. ACCURACY (How wrong is the map?) -> Distance from Map to GT
        map_to_gt_dists = np.asarray(self.geometry.map_distances())
        mean_error = np.mean(map_to_gt_dists)
        rmse = np.sqrt(np.mean(map_to_gt_dists**2))

        # B. COMPLETENESS / COVERAGE (How much of the pipe did we see?) -> Distance from GT to Map
        gt_to_map_dists = np.asarray(self.geometry.ground_truth_distances())

        # If a GT point is within 15cm of a mapped point, we consider it "Covered"
        coverage_threshold = 0.15 
        covered_points = np.sum(gt_to_map_dists < coverage_threshold)
        coverage_percentage = (covered_points / len(gt_to_map_dists)) * 100.0

        self.report("\n=======================================")
        self.report("          BENCHMARK RESULTS")
        self.report("=======================================")
        self.report(f"Mean Error (Accuracy):  {mean_error:.4f} m")
        self.report(f"RMSE (Accuracy):        {rmse:.4f} m")
        self.report(f"Surface Coverage:       {coverage_percentage:.2f} % (at {coverage_threshold}m threshold)")
        self.report("=======================================\n")

        if headless:
            return coverage_percentage, rmse

        self.geometry.visualize(map_to_gt_dists)
````

### NBV_paper_ros2/src/auv_benchmark/auv_benchmark/use_cases/trajectory_evaluation.py

SHA-256: `606804be2fb24d374b2e1b506367cd57a59a998fe43646de3640c2192bc9468c`

````python
import math
from auv_contracts.ports import TrajectorySink, Clock, Diagnostics
from ..domain.evaluation import ReportWriter, ReportSnapshot

class TrajectoryEvaluation:
    def __init__(self, telemetry: TrajectorySink, writer: ReportWriter,
                 clock: Clock, diagnostics: Diagnostics, show_reference_objects=True):
        self.telemetry = telemetry
        self.writer = writer
        self.clock = clock
        self.diagnostics = diagnostics
        self.show_reference_objects = show_reference_objects
        # State variables
        self.mission_started = False
        self.mission_finished = False
        self.start_time = 0.0
        self.end_time = 0.0
        self.total_distance = 0.0
        self.data_saved = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.trajectory_log = [] # Stores (timestamp, x, y, z, distance_from_start)
        self.diagnostic_log = []
        self.planned_trajectory = []

        self.diagnostics.info("📊 Benchmark Logger Initialized. Waiting for Odometry...")

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


    def planned_trajectory_callback(self, msg):
        """Keep the latest planner trajectory, encoded as flattened [x,y,z,yaw]."""
        if len(msg.data) < 4 or len(msg.data) % 4 != 0:
            return
        self.planned_trajectory = [
            tuple(float(value) for value in msg.data[index:index + 4])
            for index in range(0, len(msg.data), 4)
        ]


    def _nearest_planned_point(self):
        if not self.planned_trajectory:
            return None
        return min(
            self.planned_trajectory,
            key=lambda point: (
                (point[0] - self.current_x) ** 2
                + (point[1] - self.current_y) ** 2
                + (point[2] - self.current_z) ** 2))


    def odom_callback(self, msg):
        if self.mission_finished:
            return

        # Check if this is the first valid odometry message (not 0,0,0)
        if not self.mission_started and (abs(msg.position.x) > 0.01):
            self.mission_started = True
            self.start_time = self.clock.time()
            self.diagnostics.info("🚀 Mission Started! Timer running.")

        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z
        orientation = msg.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(sin_yaw, cos_yaw)


    def log_trajectory_callback(self):
        if not self.mission_started or self.mission_finished:
            return

        current_time = self.clock.time() - self.start_time

        # Calculate distance traveled since last log
        if len(self.trajectory_log) > 0:
            last_pt = self.trajectory_log[-1]
            dx = self.current_x - last_pt[1]
            dy = self.current_y - last_pt[2]
            dz = self.current_z - last_pt[3]
            segment_dist = math.sqrt(dx**2 + dy**2 + dz**2)
            self.total_distance += segment_dist
            elapsed = max(current_time - last_pt[0], 1e-6)
            speed = segment_dist / elapsed
            horizontal_speed = math.hypot(dx, dy) / elapsed
            vertical_speed = dz / elapsed
        else:
            speed = 0.0
            horizontal_speed = 0.0
            vertical_speed = 0.0

        planned_point = self._nearest_planned_point()
        if planned_point is None:
            heading_error_deg = None
            planned_x = planned_y = planned_z = planned_yaw = None
        else:
            planned_x, planned_y, planned_z, planned_yaw = planned_point
            heading_error_deg = math.degrees(
                self._wrap_angle(planned_yaw - self.current_yaw))

        self.trajectory_log.append((
            current_time, self.current_x, self.current_y, self.current_z, self.total_distance
        ))
        self.telemetry.record_trajectory(
            current_time,
            (self.current_x, self.current_y, self.current_z),
            self.total_distance)
        self.diagnostic_log.append({
            'time_s': current_time,
            'speed_mps': speed,
            'horizontal_speed_mps': horizontal_speed,
            'vertical_speed_mps': vertical_speed,
            'heading_error_deg': heading_error_deg,
            'x_m': self.current_x,
            'y_m': self.current_y,
            'z_m': self.current_z,
            'planned_x_m': planned_x,
            'planned_y_m': planned_y,
            'planned_z_m': planned_z,
            'planned_yaw_rad': planned_yaw,
        })


    def finish_callback(self, msg):
        if msg.data and not self.mission_finished:
            self.mission_finished = True
            if self.start_time == 0.0:
                self.start_time = self.clock.time() # Handle immediate aborts

            self.end_time = self.clock.time()
            total_time = self.end_time - self.start_time

            self.diagnostics.info(f"🛑 Mission Finished! Total Time: {total_time:.2f} s")
            self.save_benchmark_data(total_time)


    def _trajectory_rmse(self):
        errors = []
        for sample in self.diagnostic_log:
            if sample['planned_x_m'] is None:
                continue
            errors.append(
                (sample['x_m'] - sample['planned_x_m']) ** 2
                + (sample['y_m'] - sample['planned_y_m']) ** 2
                + (sample['z_m'] - sample['planned_z_m']) ** 2)
        if not errors:
            return None
        return math.sqrt(sum(errors) / len(errors))


    def _diagnostic_metrics(self):
        heading_errors = [
            abs(sample['heading_error_deg'])
            for sample in self.diagnostic_log
            if sample['heading_error_deg'] is not None]
        speeds = [sample['speed_mps'] for sample in self.diagnostic_log]
        trajectory_rmse = self._trajectory_rmse()
        return {
            'peak_speed_mps': max(speeds, default=0.0),
            'mean_abs_heading_error_deg': (
                sum(heading_errors) / len(heading_errors)
                if heading_errors else None),
            'max_abs_heading_error_deg': max(heading_errors, default=None),
            'trajectory_rmse_m': trajectory_rmse,
        }


    def save_benchmark_data(self, total_time):
        if self.data_saved:
            return
        snapshot = ReportSnapshot(tuple(self.trajectory_log), tuple(self.diagnostic_log),
                                  self.total_distance, self._diagnostic_metrics(),
                                  self.show_reference_objects)
        self.writer.write(snapshot, total_time)
        self.data_saved = True

    def close(self):
        """Save the partial trajectory when shutdown is requested."""
        if not self.data_saved:
            end_time = self.clock.time()
            total_time = (
                end_time - self.start_time if self.start_time > 0.0 else 0.0)
            self.diagnostics.info(
                'Shutdown requested; saving the current trajectory and plot.')
            self.save_benchmark_data(total_time)

````

### NBV_paper_ros2/src/auv_benchmark/package.xml

SHA-256: `c80636764f6921de975ba246f7f99a5703904260973b33b28c62f49cc3277c51`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_benchmark</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>python3-matplotlib</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>auv_contracts</exec_depend>
  <exec_depend>auv_telemetry</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-open3d</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_benchmark/setup.py

SHA-256: `b6218ae4855e531dc60c0c790485cf98d8039b8e6e485ee4b7a902315bc54ff2`

````python
from setuptools import find_packages, setup

package_name = 'auv_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'open3d', 'pyoctomap', 'matplotlib'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_baselines = auv_benchmark.run_baselines:main',
            'plot_benchmark_results = auv_benchmark.plot_results:main',
        ],
    },
)
````

### NBV_paper_ros2/src/auv_bringup/auv_bringup/configuration.py

SHA-256: `914bbda9a6f77e5205064ced071a38ba5111dd2426e0e6ae39b3bc22e2ce66cb`

````python
"""Deployment-owned translation of experiment overrides to the ROS YAML schema."""


def apply_trial_overrides(parameters, overrides, map_path, results_dir):
    planner = parameters['planner_node']['ros__parameters']
    for key, value in overrides.items():
        planner[key] = float(value)
    planner['octomap_path'] = str(map_path)
    planner['results_dir'] = str(results_dir)
    parameters['run_baselines']['ros__parameters']['results_dir'] = str(results_dir)
    return parameters
````

### NBV_paper_ros2/src/auv_bringup/package.xml

SHA-256: `bc18400b7eaa65878c32d4305ba7c873722b808a98166687d873ab5930db0c21`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_bringup</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>auv_sim_bridge</exec_depend>
  <exec_depend>auv_stereo_depth</exec_depend>
  <exec_depend>auv_motion_control</exec_depend>
  <exec_depend>auv_planner_reference</exec_depend>
  <exec_depend>auv_benchmark</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_bringup/setup.py

SHA-256: `36e7fce8b3e4ce0a2b41171aae53ab82f6bf5b118726f13150d462b26efe759b`

````python
import os

from setuptools import find_packages, setup

package_name = 'auv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/mapper.launch.py']),
        (os.path.join('share', package_name, 'config'),
         ['config/params.yaml', 'config/nbv_params.yaml', 'config/holoocean_config.json',
          'config/README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
````

### NBV_paper_ros2/src/docs/architecture/verification/generate_refactor_artifacts.py

SHA-256: `c52a6d7dae439212a07ab5548bb19b7c9aa055b1204f998bd1a9979353d9abf1`

````python
"""Generate a source-complete review bundle and current package topology."""
import ast
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess
import xml.etree.ElementTree as ET

SOURCE = Path(__file__).resolve().parents[3]
REPOSITORY = SOURCE.parent
OUTPUT = SOURCE / 'docs/architecture'


def git_paths(directory, arguments, prefix=Path()):
    result = subprocess.run(['git', *arguments], cwd=directory,
                            check=True, capture_output=True, text=True)
    return [prefix / name for name in result.stdout.splitlines()]


def source_bundle():
    paths = git_paths(REPOSITORY, ['diff', '--name-only', '--', 'src'])
    paths += git_paths(REPOSITORY, ['ls-files', '--others', '--exclude-standard', 'src'])
    vendor = Path('src/auv_stereo_depth/auv_stereo_depth/BGNet')
    paths += git_paths(REPOSITORY / vendor, ['diff', '--name-only'], vendor)
    paths += git_paths(REPOSITORY / vendor, ['ls-files', '--others', '--exclude-standard'], vendor)
    selected = []
    for path in sorted(set(paths)):
        absolute = REPOSITORY / path
        if not absolute.is_file() or '__pycache__' in path.parts:
            continue
        if path.suffix not in {'.py', '.cpp', '.hpp', '.h', '.xml', '.cfg'} and (
                path.name != 'CMakeLists.txt' and 'resource' not in path.parts):
            continue
        selected.append(path)
    text = ['# Complete refactoring code bundle', '',
            'Full current contents of changed/new source, build, manifest, resource and verification files.',
            'Documentation is linked from [the architecture index](../new_arch/README.md).',
            'This generated review artifact contains no omitted code. Protocol `...` bodies are actual abstract declarations.',
            'The optimizer moved to `auv_experiments`; the separately removed legacy planner Tools are not restored.', '']
    manifest = []
    languages = {'.py': 'python', '.cpp': 'cpp', '.hpp': 'cpp', '.h': 'cpp', '.xml': 'xml', '.cfg': 'ini'}
    for path in selected:
        data = (REPOSITORY / path).read_bytes()
        content = data.decode('utf-8')
        digest = hashlib.sha256(data).hexdigest()
        manifest.append({'path': str(path), 'sha256': digest, 'bytes': len(data)})
        language = 'cmake' if path.name == 'CMakeLists.txt' else languages.get(path.suffix, 'text')
        fence = '`' * max(4, max((len(match[0]) for match in re.finditer(r'`+', content)), default=0) + 1)
        text += [f'### NBV_paper_ros2/{path}', '', f'SHA-256: `{digest}`', '',
                 fence + language, content[:-1] if content.endswith('\n') else content, fence, '']
    (OUTPUT / 'refactoring_complete_code.md').write_text('\n'.join(text))
    (OUTPUT / 'refactoring_code_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return len(selected)


def cpp_class_counts(path):
    source = path.read_text()
    masked = re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"',
                    lambda match: ' ' * len(match[0]), source, flags=re.S)
    concrete = abstract = 0
    for match in re.finditer(r'\b(?:class|struct)\s+\w+[^;{}]*\{', masked):
        if re.search(r'enum\s*$', masked[:match.start()]):
            continue
        index = match.end()
        level = 1
        while level and index < len(masked):
            level += (masked[index] == '{') - (masked[index] == '}')
            index += 1
        body = masked[match.end():index - 1]
        is_abstract = bool(re.search(r'virtual\b[^;{}]*=\s*0\s*;', body))
        abstract += is_abstract
        concrete += 1
    return abstract, concrete


def topology():
    packages = {ET.parse(path).getroot().findtext('name'): path.parent
                for path in SOURCE.glob('*/package.xml')}
    edges = {name: set() for name in packages}
    counts = {name: [0, 0] for name in packages}
    for name, directory in packages.items():
        manifest = ET.parse(directory / 'package.xml').getroot()
        edges[name].update(element.text for element in manifest
                           if element.tag in {'depend', 'exec_depend', 'build_depend', 'build_export_depend'}
                           and element.text in packages and element.text != name)
        for path in directory.rglob('*'):
            if not path.is_file() or any(part in {'test', '__pycache__', '.git'} for part in path.parts):
                continue
            if path.suffix == '.py':
                tree = ast.parse(path.read_text())
                for node in ast.walk(tree):
                    if isinstance(node, ast.Import):
                        imports = [alias.name.split('.')[0] for alias in node.names]
                    elif isinstance(node, ast.ImportFrom) and not node.level:
                        imports = [(node.module or '').split('.')[0]]
                    else:
                        imports = []
                    edges[name].update(dependency for dependency in imports
                                       if dependency in packages and dependency != name)
                    if isinstance(node, ast.ClassDef) and path.name != 'setup.py':
                        counts[name][1] += 1
                        counts[name][0] += any(ast.unparse(base).split('.')[-1] in {'Protocol', 'ABC'}
                                               for base in node.bases)
            elif path.suffix in {'.cpp', '.hpp', '.h'}:
                abstract, classes = cpp_class_counts(path)
                counts[name][0] += abstract
                counts[name][1] += classes
    cycles = []
    def visit(node, path):
        if node in path:
            cycles.append(path[path.index(node):] + [node])
            return
        for dependency in sorted(edges[node]):
            visit(dependency, path + [node])
    for name in packages:
        visit(name, [])
    assert not cycles, cycles
    rows = []
    metrics = {}
    for name in sorted(packages):
        incoming = sum(name in dependencies for dependencies in edges.values())
        outgoing = len(edges[name])
        abstract, classes = counts[name]
        instability = outgoing / (incoming + outgoing) if incoming + outgoing else 0.
        abstractness = abstract / classes if classes else 0.
        distance = abs(abstractness + instability - 1.)
        conditional = not incoming + outgoing or not classes
        status = ('Balanced' if distance <= .1 else 'Zone of Pain'
                  if abstractness + instability < 1 else 'Zone of Uselessness')
        metrics[name] = dict(fan_in=incoming, fan_out=outgoing, Na=abstract, Nc=classes,
                             I=instability, A=abstractness, D=distance, conditional=conditional)
        rows.append(f'| {name} | {incoming} | {outgoing} | {abstract} | {classes} | '
                    f'{instability:.3f}{"*" if not incoming + outgoing else ""} | '
                    f'{abstractness:.3f}{"*" if not classes else ""} | '
                    f'{distance:.3f}{"*" if conditional else ""} | {status}{"*" if conditional else ""} |')
    values = [row['D'] for row in metrics.values()]
    defined = [row['D'] for row in metrics.values() if not row['conditional']]
    violations = [(source, target) for source, dependencies in edges.items() for target in dependencies
                  if metrics[target]['I'] > metrics[source]['I']]
    text = ['# Implemented component topology and metrics', '',
            'Snapshot of the ten current ROS packages. Arrows mean dependent → dependency.',
            'Edges combine production imports and internal runtime/build manifest dependencies (including launch closure).',
            'External libraries, topic flow, tests and transitive edges are excluded.', '', '```mermaid', 'graph TD']
    for name in sorted(packages):
        text.append(f'    {name}[{name}]')
    for name in sorted(edges):
        for dependency in sorted(edges[name]):
            text.append(f'    {name} --> {dependency}')
    text += ['```', '', 'Detected cycles: **none**.',
             'Strictly increasing-instability edges: ' + ('**none**.' if not violations else str(violations)), '',
             '## Metrics', '',
             'Ca/Ce are distinct package neighbors. Nc counts production Python classes/C++ class and struct definitions;',
             'Na counts explicit Protocol/ABC definitions and pure-virtual C++ interfaces. Packaging/test classes are excluded.',
             'Vendored BGNet source remains counted as in the original audit, including checkout-only research classes.',
             '`I = Ce/(Ca+Ce)`, `A = Na/Nc`, `D = |A+I-1|`. The original audit threshold D ≤ 0.100 is retained.',
             'An asterisk marks a zero imputation for an undefined denominator and conditional D/classification.', '',
             '| Package | Fan-in | Fan-out | Na | Nc | I | A | D | Classification |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---|', *rows, '',
             f'All-package conditional mean D: {statistics.mean(values):.4f}; population variance: {statistics.pvariance(values):.4f}.',
             f'Defined-subset mean D: {statistics.mean(defined):.4f}; population variance: {statistics.pvariance(defined):.4f}.', '',
             '## Interpretation', '',
             'These are release-package metrics, not library-layer metrics. Each runtime package still contains concrete adapters',
             'alongside its policy, so a Zone of Pain label alone does not prove a remaining inward-dependency violation.',
             'The C++ policy libraries are separately reusable targets inside their existing package releases.',
             'The original seven-package mean is not directly comparable to this ten-package snapshot.',
             'Further splitting releases solely to improve D would require an explicit reuse/release decision, not extra empty interfaces.', '']
    (OUTPUT / 'refactoring_topology.md').write_text('\n'.join(text))
    (OUTPUT / 'refactoring_topology.json').write_text(json.dumps({'dependencies': {k: sorted(v) for k, v in edges.items()},
                                                               'metrics': metrics}, indent=2) + '\n')
    return len(packages), sum(map(len, edges.values()))


if __name__ == '__main__':
    packages, edges = topology()
    files = source_bundle()
    print(f'Generated complete code for {files} files; {packages} packages, {edges} edges, no cycles.')
````

### NBV_paper_ros2/src/docs/architecture/verification/mapping_equivalence.cpp

SHA-256: `ffc7afec2ef25866997af547ed882ba41a640366ab2cea1b337943b211e43f54`

````cpp
// Compile twice with ORIGINAL_MAPPER and the pre-refactor source, then without
// it against inspection_policy. Compare the output text and .bt files bytewise.
#include <opencv2/opencv.hpp>
#include <octomap/ColorOcTree.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#ifdef ORIGINAL_MAPPER
#include "realtime_stereo_mapper.hpp"
#else
#include "auv_inspection_planner/use_cases/mapping.hpp"
#include "auv_inspection_planner/adapters/maps.hpp"
struct QuietDiagnostics final : inspection::Diagnostics {
  void text(const std::string&,bool,bool) override {}
  void info(const std::string&) override {}
  void warning(const std::string&) override {}
  void error(const std::string&) override {}
};
#endif

int main(int argc,char** argv) {
  if(argc!=2) return 2;
  const std::string prefix=argv[1];
  std::ofstream results(prefix+".txt");
  results<<std::setprecision(17);
#ifdef ORIGINAL_MAPPER
  MapperConfig config;
  config.info_gain_threshold=100.0;
  RealTimeStereoMapper mapping(config);
#else
  const inspection::MapperConfig config{90.,256.,.5,.1,4.,.5,-32.,100.,1.2,.02};
  QuietDiagnostics diagnostics;
  inspection::OctomapAdapter map(config.resolution,diagnostics);
  inspection::PclIndexes indexes;
  inspection::RoutePlanner routes(config.resolution,diagnostics);
  inspection::MapUpdate mapping(config,map,map);
  inspection::CandidatePlanner candidates(config,map,indexes,routes,diagnostics);
#endif
  for(int frame=0;frame<3;++frame) {
    cv::Mat depth(16,16,CV_32FC1), color(16,16,CV_8UC3);
    for(int v=0;v<16;++v) for(int u=0;u<16;++u) {
      float z=1.0f+.05f*u+.025f*v;
      if(u==0) z=0.5f;
      if(u==1) z=4.0f;
      if(u==2) z=std::numeric_limits<float>::quiet_NaN();
      depth.at<float>(v,u)=z;
      color.at<cv::Vec3b>(v,u)=cv::Vec3b(u+frame,v+frame,u+v);
    }
    const Eigen::Vector3d position(frame*.2,frame*.1,-20.);
#ifdef ORIGINAL_MAPPER
    mapping.process_frame(position,.1*frame,.02*frame,-.03*frame,color,depth);
    results<<mapping.get_current_leaves()<<'\n';
#else
    std::vector<float> pixels;
    std::vector<std::array<unsigned char,3>> colors;
    for(int v=0;v<16;++v) for(int u=0;u<16;++u) {
      pixels.push_back(depth.at<float>(v,u));
      const auto c=color.at<cv::Vec3b>(v,u); colors.push_back({c[0],c[1],c[2]});
    }
    mapping.process_frame(position,.1*frame,.02*frame,-.03*frame,
      inspection::ColorImage{16,16,std::move(colors)},inspection::DepthImage{16,16,std::move(pixels)});
    results<<map.count()<<'\n';
#endif
  }
#ifdef ORIGINAL_MAPPER
  const auto plan=mapping.generate_candidate_viewpoints(Eigen::Vector3d(0.,0.,-20.));
  mapping.save_map(prefix+".bt");
#else
  const auto plan=candidates.generate_candidate_viewpoints(Eigen::Vector3d(0.,0.,-20.));
  map.save(prefix+".bt");
#endif
  results<<plan.first.size()<<' '<<plan.second.size()<<'\n';
  for(const auto& p:plan.first) results<<p.transpose()<<'\n';
  for(const auto& p:plan.second) results<<p.transpose()<<'\n';
}
````

### NBV_paper_ros2/src/docs/architecture/verification/test_policy_boundaries.py

SHA-256: `89cdb68d2660d5c119a5af81bec52a0766ae132de5d7ea38a64dcb5fc3df6492`

````python
"""Run with /usr/bin/python3 -m pytest; no ROS, Torch or simulator startup."""
import ast
from contextlib import contextmanager
from dataclasses import FrozenInstanceError
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[3]
for package in ('auv_contracts', 'auv_stereo_depth', 'auv_planner_reference', 'auv_sim_bridge',
                'auv_benchmark', 'auv_experiments', 'auv_bringup'):
    sys.path.insert(0, str(SOURCE / package))

from auv_contracts.values import Configuration, CameraCalibration, Trajectory, ArrayEvent
from auv_contracts.mission_results import distance_record, distance_from_records
from auv_stereo_depth.use_cases.depth import StereoDepthUseCase
from auv_sim_bridge.use_cases.simulation import SimulationRunner
from auv_sim_bridge.domain.simulation import Command
from auv_sim_bridge.adapters.sensor_dispatch import SensorDispatch
from auv_benchmark.use_cases.map_evaluation import EvaluateMap
from auv_benchmark.use_cases.trajectory_evaluation import TrajectoryEvaluation
from auv_experiments.trial_policy import TrialObjective
from auv_bringup.configuration import apply_trial_overrides


class Diagnostics:
    def __init__(self):
        self.events = []

    def __getattr__(self, level):
        return lambda message, **options: self.events.append((level, message))


def test_configuration_and_trajectory_take_immutable_snapshots():
    original = {'nested': {'weights': [1., 2.]}}
    config = Configuration(original)
    original['nested']['weights'][0] = 99.
    assert config.nested['weights'] == (1., 2.)
    with pytest.raises(TypeError):
        config.nested['weights'] = ()
    points = [[1., 2., 3., 4.]]
    trajectory = Trajectory(points)
    points[0][0] = 99.
    assert trajectory.points == ((1., 2., 3., 4.),)
    with pytest.raises(FrozenInstanceError):
        trajectory.points = ()


def test_depth_keeps_invalid_disparity_mask_and_center_sample():
    disparity = np.array([[-2., 0., .1], [1., 2., 4.]], dtype=np.float32)
    values = iter([10., 10.3, 10.4])
    estimator = SimpleNamespace(estimate=lambda left, right, started:
                                (disparity.copy(), 100., 100., 10.2))
    use_case = StereoDepthUseCase(estimator, SimpleNamespace(perf_counter=lambda: next(values)))
    depth = use_case.compute('left', 'right', CameraCalibration(256., .5))
    np.testing.assert_array_equal(depth, [[0., 0., 0.], [128., 64., 32.]])
    timings = use_case.timings()
    assert timings['center_depth_m'] == 64.
    assert timings['postprocessing_ms'] == pytest.approx(100.)
    assert timings['total_compute_ms'] == pytest.approx(400.)
    timings.clear()
    assert use_case.timings()['center_depth_m'] == 64.


def test_simulation_preserves_step_dispatch_pump_order_and_limit():
    events = []
    command = Command(tuple(range(8)))

    class Sessions:
        @contextmanager
        def open(self):
            events.append('open')
            yield SimpleNamespace(step=lambda received:
                                  events.append(('step', received)) or {'sensor': 1})
            events.append('close')

    runner = SimulationRunner(Sessions(), SimpleNamespace(dispatch=lambda state, session:
                              events.append(('dispatch', state))), lambda: command,
                              lambda: events.append('pump'), Diagnostics(), step_limit=2)
    runner.run()
    assert events == ['open', ('step', command), ('dispatch', {'sensor': 1}), 'pump',
                      ('step', command), ('dispatch', {'sensor': 1}), 'pump', 'close']


def test_sensor_dispatch_preserves_order_and_continues_after_sensor_failure():
    events = []

    class Outputs:
        def __getattr__(self, name):
            def publish(value):
                events.append(name)
                if name == '_publish_dvl':
                    raise ValueError('failed sensor')
            return publish

    state = dict(FrontCamera=np.zeros((1, 1, 3)), DownCamera=np.zeros((1, 1, 3)),
                 LocationSensor=[1., 2., -3.])
    diagnostics = Diagnostics()
    sensors = SensorDispatch(Outputs(), diagnostics, lambda location: [4., 5., 6.])
    session = SimpleNamespace(set_ocean_currents=lambda agent, vector:
                              events.append((agent, vector)))
    sensors.dispatch(state, session)
    assert events == ['_publish_camera', '_publish_camera_down', '_publish_range_finder',
                      '_publish_dvl', '_publish_imu', '_publish_depth', '_publish_mag',
                      ('auv0', [4., 5., 6.]), '_publish_location']
    assert any('dvl failed' in message for level, message in diagnostics.events)


def test_map_scoring_keeps_strict_coverage_threshold():
    geometry = SimpleNamespace(prepare=lambda: None,
        map_distances=lambda: [.1, .3], ground_truth_distances=lambda: [.149, .15, .2],
        visualize=lambda distances: pytest.fail('headless evaluation must not visualize'))
    coverage, rmse = EvaluateMap(geometry, lambda message: None).execute(headless=True)
    assert coverage == pytest.approx(100. / 3.)
    assert rmse == pytest.approx(np.sqrt(.05))


def test_trajectory_metrics_and_save_remain_idempotent():
    writes = []
    evaluation = TrajectoryEvaluation(None, SimpleNamespace(write=lambda *args: writes.append(args)),
                                      SimpleNamespace(time=lambda: 10.), Diagnostics())
    evaluation.total_distance = 5.
    evaluation.diagnostic_log = [dict(planned_x_m=0., planned_y_m=0., planned_z_m=0.,
        x_m=3., y_m=4., z_m=0., heading_error_deg=-10., speed_mps=2.)]
    evaluation.save_benchmark_data(10.)
    evaluation.close()
    assert len(writes) == 1
    snapshot, elapsed = writes[0]
    assert snapshot.metrics['trajectory_rmse_m'] == 5.
    assert snapshot.metrics['mean_abs_heading_error_deg'] == 10.
    assert snapshot.metrics['peak_speed_mps'] == 2.
    evaluation.diagnostic_log[0]['x_m'] = 99.
    assert snapshot.diagnostic_log[0]['x_m'] == 3.
    assert elapsed == 10.


@pytest.mark.parametrize('distance,rmse,expected', [(7., .41, (40., 7.)), (7., .4, (80., 7.)),
                                                  (None, .1, (0., 999.))])
def test_trial_objective_preserves_penalties(distance, rmse, expected):
    sampled = []
    trial = SimpleNamespace(number=1, suggest_float=lambda name, low, high:
                            sampled.append((name, low, high)) or low)
    objective = TrialObjective(SimpleNamespace(run=lambda parameters: distance),
                               lambda **options: (80., rmse), lambda message: None)
    assert objective(trial) == expected
    assert len(sampled) == 6


def test_result_protocol_and_config_translation_keep_existing_wire_format():
    assert distance_record(12.345) == 'Total Distance Traveled: 12.35 meters\n'
    assert distance_from_records(['heading\n', distance_record(12.345)]) == 12.35
    assert distance_from_records([]) == 999.
    params = {'planner_node': {'ros__parameters': {'untouched': True}},
              'run_baselines': {'ros__parameters': {}}}
    assert apply_trial_overrides(params, {'nbv_standoff': 2}, 'map.bt', 'results') is params
    assert params['planner_node']['ros__parameters'] == {
        'untouched': True, 'nbv_standoff': 2., 'octomap_path': 'map.bt', 'results_dir': 'results'}
    assert params['run_baselines']['ros__parameters']['results_dir'] == 'results'


def test_python_policy_modules_have_no_framework_or_io_imports():
    forbidden = {'rclpy', 'rclcpp', 'sensor_msgs', 'nav_msgs', 'std_msgs', 'cv_bridge',
                 'torch', 'torchvision', 'open3d', 'pyoctomap', 'holoocean', 'cv2',
                 'matplotlib', 'subprocess', 'os', 'pathlib'}
    paths = list((SOURCE / 'auv_contracts/auv_contracts').glob('*.py'))
    for package in ('auv_planner_reference', 'auv_stereo_depth', 'auv_sim_bridge', 'auv_benchmark'):
        for layer in ('domain', 'use_cases'):
            paths.extend((SOURCE / package / package / layer).glob('*.py'))
    paths.append(SOURCE / 'auv_experiments/auv_experiments/trial_policy.py')
    for path in paths:
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                names = [alias.name.split('.')[0] for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [(node.module or '').split('.')[0]]
                assert 'adapters' not in (node.module or '').split('.'), path
            else:
                continue
            assert not forbidden.intersection(names), path
````

### NBV_paper_ros2/src/auv_motion_control/CMakeLists.txt

SHA-256: `5ce1ceac7c83a42e0633a030eaaeec86644f403ad31675f1fa37573853639daa`

````cmake
cmake_minimum_required(VERSION 3.8)
project(auv_motion_control)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(motion_policy src/control_policy.cpp)
target_compile_features(motion_policy PUBLIC cxx_std_17)
target_include_directories(motion_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(motion_platform_adapters src/thruster_adapters.cpp)
target_link_libraries(motion_platform_adapters PUBLIC motion_policy)
add_executable(waypoint_control_node src/waypoint_control_node.cpp)
target_link_libraries(waypoint_control_node motion_platform_adapters)
ament_target_dependencies(waypoint_control_node rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs)
add_executable(point_iterator src/point_iterator.cpp)
target_link_libraries(point_iterator motion_policy)
ament_target_dependencies(point_iterator rclcpp std_msgs)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS motion_policy motion_platform_adapters EXPORT motionTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS waypoint_control_node point_iterator DESTINATION lib/${PROJECT_NAME})
ament_export_targets(motionTargets HAS_LIBRARY_TARGET)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  enable_testing()
  add_executable(motion_policy_regression test/policy_regression.cpp)
  target_link_libraries(motion_policy_regression PRIVATE motion_platform_adapters)
  add_test(NAME motion_policy_regression COMMAND motion_policy_regression)
endif()
ament_package()
````

### NBV_paper_ros2/src/auv_motion_control/include/auv_motion_control/adapters/ros_control_io.hpp

SHA-256: `b533fa96b0fcb546b1b06da6bc15e51262a69bad099edf81f4a1ec7f098d6e4d`

````cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "auv_motion_control/domain/contracts.hpp"
namespace auv_motion_control
{
class RosControlIO final : public Clock, public Gains, public Diagnostics, public CommandOutput {
  rclcpp::Node & node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_, report_;

public:
  RosControlIO(
    rclcpp::Node & node,
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command,
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr report)
  : node_(node), command_(command), report_(report) {}
  double now() const override {return node_.get_clock()->now().seconds();}
  double yaw() const override {return node_.get_parameter("kp_yaw").as_double();}
  double distance() const override {return node_.get_parameter("kp_dist").as_double();}
  void write(const std::string & text) override
  {
    RCLCPP_INFO(node_.get_logger(), "%s", text.c_str());
  }
  void command(const MotorCommand & command) override
  {
    std_msgs::msg::Float32MultiArray message;
    message.data.assign(command.values.begin(), command.values.end());
    command_->publish(message);
  }
  void reached(const std::vector<float> & values) override
  {
    std_msgs::msg::Float32MultiArray message; message.data = values; report_->publish(message);
  }
};
}
````

### NBV_paper_ros2/src/auv_motion_control/include/auv_motion_control/adapters/thrusters.hpp

SHA-256: `fc16de09a404ec690e0844054d2ce190efe8933b24282ec8992b73ed4fd20c03`

````cpp
#pragma once
#include <algorithm>
#include <map>
#include "auv_motion_control/domain/contracts.hpp"
namespace auv_motion_control
{
class ConfiguredModes final : public Modes {
  const std::map<std::string, ControlMode> modes_;

public:
  ConfiguredModes()
  : modes_{{"yaw_centerize", {0.6, true, false}},
      {"full_centerize", {0.25, true, false}}, {"no_centerize", {0.6, false, true}}} {}
  ControlMode select(const std::string & name) const override
  {
    const auto mode = modes_.find(name);
    return mode == modes_.end() ? ControlMode{0.6, false, false} : mode->second;
  }
};
class EightThrusterMixer final : public ThrusterMixer {
public:
  std::vector<double> mix(
    const std::vector<char> & keys, double val, std::vector<double> cmd,
    double vertical_val) const override
  {
    auto has_key = [&](char k) {return std::find(keys.begin(), keys.end(), k) != keys.end();};

    if (has_key('i')) {
      for (int i = 0; i < 4; i++) {cmd[i] += vertical_val;}
    }
    if (has_key('k')) {
      for (int i = 0; i < 4; i++) {cmd[i] -= vertical_val;}
    }
    if (has_key('j')) {
      cmd[4] += val;
      cmd[7] += val;
      cmd[5] -= val;
      cmd[6] -= val;
    }
    if (has_key('l')) {
      cmd[4] -= val;
      cmd[7] -= val;
      cmd[5] += val;
      cmd[6] += val;
    }
    if (has_key('w')) {
      for (int i = 4; i < 8; i++) {cmd[i] -= val;}
    }
    if (has_key('s')) {
      for (int i = 4; i < 8; i++) {cmd[i] += val;}
    }
    if (has_key('a')) {
      cmd[4] += val;
      cmd[6] += val;
      cmd[5] -= val;
      cmd[7] -= val;
    }
    if (has_key('d')) {
      cmd[4] -= val;
      cmd[6] -= val;
      cmd[5] += val;
      cmd[7] += val;
    }
    return cmd;
  }
  void yaw(std::vector<double> & cmd, double yaw_command) const override
  {
    cmd[4] -= yaw_command;
    cmd[7] -= yaw_command;
    cmd[5] += yaw_command;
    cmd[6] += yaw_command;
  }
};
}
````

### NBV_paper_ros2/src/auv_motion_control/include/auv_motion_control/domain/contracts.hpp

SHA-256: `84e0ec350b025c1a60ea3d919c99b779a26fc23a8e5229b1d5fd5c8af7adf242`

````cpp
#pragma once
#include <vector>
#include <string>
#include <memory>
#include <cstdio>
#include <cstddef>
namespace auv_motion_control
{
struct Position { const double x, y, z; };
struct MotorCommand { const std::vector<double> values; };
struct WaypointInput { const std::vector<float> data; };
struct ControlConfig
{
  const double kp_yaw, kp_dist;
  const bool dvl_active;
  const double stopping_distance, initial_waypoint_x, initial_waypoint_y,
    initial_waypoint_z, max_yaw_command, yaw_smoothing_rate,
    min_forward_thrust, max_forward_thrust, command_multiplier;
};
struct QueueConfig
{
  const double camera_x, camera_y, camera_z, initial_waypoint_x,
    initial_waypoint_y, initial_waypoint_z, initial_waypoint_yaw,
    timer_period_s, waypoint_reached_distance;
};
struct ControlMode { const double vertical_multiplier; const bool centered, accepts_angle; };
class Modes {
public:
  virtual ~Modes() = default; virtual ControlMode select(const std::string &) const= 0;
};
class Clock {
public:
  virtual ~Clock() = default; virtual double now() const= 0;
};
class Gains {
public:
  virtual ~Gains() = default; virtual double yaw() const= 0; virtual double distance() const= 0;
};
class Diagnostics {
public:
  virtual ~Diagnostics() = default; virtual void write(const std::string &) = 0;
};
class CommandOutput {
public:
  virtual ~CommandOutput() = default;
  virtual void command(const MotorCommand &) = 0;
  virtual void reached(const std::vector<float> &) = 0;
};
class WaypointOutput {
public:
  virtual ~WaypointOutput() = default;
  virtual void waypoints(const std::vector<float> &, std::size_t) = 0;
  virtual void request_nbv() = 0;
};
class ThrusterMixer {
public:
  virtual ~ThrusterMixer() = default;
  virtual std::vector<double> mix(
    const std::vector<char> &, double, std::vector<double>,
    double) const= 0;
  virtual void yaw(std::vector<double> &, double) const= 0;
};
inline std::string format_text(const char * value) {return value;}
template<class ... Args> std::string format_text(const char * format, Args... args)
{
  const int size = std::snprintf(nullptr, 0, format, args ...);
  std::vector<char> text(static_cast<std::size_t>(size) + 1);
  std::snprintf(text.data(), text.size(), format, args ...);
  return std::string(text.data(), static_cast<std::size_t>(size));
}
}
````

### NBV_paper_ros2/src/auv_motion_control/include/auv_motion_control/domain/math_pid_utils.hpp

SHA-256: `147b483e7cbeae93da3b23164350e6434b3c75779e2d0540880570a769caed0e`

````cpp
#pragma once

#include <cmath>
#include <vector>
#include <array>

namespace utils
{

    // --- Math Utilities ---
inline double normalize_angle_deg(double angle)
{
  return std::fmod(angle + 180.0, 360.0) - 180.0;
}

inline double quaternion_to_roll_deg(double x, double y, double z, double w)
{
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  return std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
}

inline double quaternion_to_pitch_rad(double x, double y, double z, double w)
{
  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    return std::copysign(M_PI / 2.0, sinp);
  }
  return std::asin(sinp);
}

inline double diff_2d(const std::vector<double> & a, const std::vector<double> & b)
{
  return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
}

inline double diff_3d(const std::vector<double> & a, const std::vector<double> & b)
{
  return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2));
}

inline std::vector<double> world_to_auv_coordinates(
  const std::vector<double> & world_pos,
  double yaw)
{
  std::vector<double> auv_pos(3, 0.0);
  auv_pos[0] = std::cos(yaw) * world_pos[0] - std::sin(yaw) * world_pos[1];
  auv_pos[1] = std::sin(yaw) * world_pos[0] + std::cos(yaw) * world_pos[1];
  auv_pos[2] = world_pos[2];
  return auv_pos;
}

    // --- PID Controller ---
class PID {
public:
  PID(double kp, double ki, double kd, double setpoint = 0.0, double filter_coeff = 0.2)
  : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), filter_coeff_(filter_coeff),
    integral_(0.0), prev_error_(0.0) {}

  double update(double measurement)
  {
    double error = setpoint_ - measurement;
    integral_ += error;
    double derivative = (error - prev_error_);

            // Apply simple low-pass filter to derivative if needed
    derivative = filter_coeff_ * derivative + (1.0 - filter_coeff_) * prev_derivative_;

    prev_error_ = error;
    prev_derivative_ = derivative;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

private:
  double kp_, ki_, kd_, setpoint_, filter_coeff_;
  double integral_;
  double prev_error_;
  double prev_derivative_ = 0.0;
};

} // namespace utils
````

### NBV_paper_ros2/src/auv_motion_control/include/auv_motion_control/use_cases/trajectory_queue.hpp

SHA-256: `50133a110c32ebf468516983565715f9d65e07f18b36fe8af2c6c0e8dc34947a`

````cpp
#pragma once
#include <algorithm>
#include <cmath>
#include "auv_motion_control/domain/contracts.hpp"
namespace auv_motion_control
{
class TrajectoryQueue {
  WaypointOutput & output_; Diagnostics & diagnostics_;
  std::vector<double> camera_position_;
  std::vector<std::vector<double>> waypoints_;
  double camera_position_x_ = 0.35;
  double camera_position_y_ = 0.1125 + 0.275;
  double camera_position_z_ = -0.5;
  double initial_waypoint_x_ = -16.0;
  double initial_waypoint_y_ = -81.0;
  double initial_waypoint_z_ = -30.0;
  double initial_waypoint_yaw_ = 0.0;
  double timer_period_s_ = 1.0;
  double waypoint_reached_distance_ = 0.4;

public:
  TrajectoryQueue(const QueueConfig & config, WaypointOutput & output, Diagnostics & diagnostics)
  : output_(output), diagnostics_(diagnostics)
  {
    camera_position_x_ = config.camera_x;
    camera_position_y_ = config.camera_y;
    camera_position_z_ = config.camera_z;
    initial_waypoint_x_ = config.initial_waypoint_x;
    initial_waypoint_y_ = config.initial_waypoint_y;
    initial_waypoint_z_ = config.initial_waypoint_z;
    initial_waypoint_yaw_ = config.initial_waypoint_yaw;
    timer_period_s_ = config.timer_period_s;
    waypoint_reached_distance_ = config.waypoint_reached_distance;
        // Initial setup
    camera_position_ = {camera_position_x_, camera_position_y_, camera_position_z_};
    waypoints_.push_back({
        initial_waypoint_x_, initial_waypoint_y_,
        initial_waypoint_z_, initial_waypoint_yaw_});
        // waypoints_.push_back({-56.0, -100.0, -30.0, 3.14});

  }
  void point_callback(const WaypointInput & msg)
  {
    waypoints_.clear();
    std::vector<double> new_wp(msg.data.begin(), msg.data.end());
    waypoints_.push_back(new_wp);
    timer_callback();
    diagnostics_.write(format_text("Published 1 waypoints (from object/detected)."));
  }

  void pointSegment_callback(const WaypointInput & msg)
  {
    std::vector<std::vector<double>> wps;

        // Replicates numpy.reshape(-1, 4)
    for (size_t i = 0; i + 3 < msg.data.size(); i += 4) {
      diagnostics_.write(format_text("Adding segmentation waypoint..."));
      std::vector<double> wp = {
        msg.data[i] - camera_position_[0],
        msg.data[i + 1] - camera_position_[1],
        msg.data[i + 2] - camera_position_[2],
        msg.data[i + 3]
      };
      wps.push_back(wp);
    }

    if (!wps.empty()) {
      waypoints_ = wps;
      diagnostics_.write(format_text("Received %zu segmentation waypoints.", wps.size()));
            // Send the new route now: the vehicle keeps moving between timer ticks.
      timer_callback();
    }
  }

  void timer_callback()
  {
    if (!waypoints_.empty()) {
      std::vector<float> values;
      for (const auto & wp : waypoints_) {
        float x = 0.0f, y = 0.0f, z = -29.5f, yaw = 0.0f;

                // Safely pad every waypoint to exactly 4 floats to prevent numpy crashes
        if (wp.size() >= 2) {x = wp[0]; y = wp[1];}
        if (wp.size() >= 3) {z = wp[2];}
        if (wp.size() >= 4) {yaw = wp[3];}

        values.insert(values.end(), {x, y, z, yaw});
      }

      output_.waypoints(values, waypoints_.size());
      diagnostics_.write(format_text("Published %zu waypoints.", waypoints_.size()));
    }
  }

  void stop_callback(const WaypointInput & msg)
  {
    if (msg.data.size() < 3) {return;}

    std::vector<double> data = {msg.data[0], msg.data[1], msg.data[2]};

    if (!waypoints_.empty() && waypoints_[0].size() >= 2) {
            // Replaces python Tools.mapping2D.diff
      double dist = std::hypot(waypoints_[0][0] - data[0], waypoints_[0][1] - data[1]);

      if (dist < waypoint_reached_distance_) {
        waypoints_.erase(waypoints_.begin());
      }
    }

    diagnostics_.write(format_text("Received stop signal, giving next waypoint."));

        // Replaces try/except blocks to remove specific target point from list safely
    auto it = std::find_if(waypoints_.begin(), waypoints_.end(),
        [&data](const std::vector<double> & wp) {
          if (wp.size() >= 3) {
            return  wp[0] == data[0] && wp[1] == data[1] && wp[2] == data[2];
          }
          return false;
        });

    if (it != waypoints_.end()) {
      waypoints_.erase(it);
    }

        // Limit the waypoints array to the last 200 elements (self.waypoints[-200:])
        // if (waypoints_.size() > 200) {
        //     waypoints_.erase(waypoints_.begin(), waypoints_.end() - 200);
        // }

        // Trigger the publisher manually
    timer_callback();

        // NEW: If the queue just hit zero, instantly demand more waypoints!
    if (waypoints_.empty()) {
      diagnostics_.write(format_text("Waypoint queue empty. Demanding new NBV trajectory NOW!"));
      output_.request_nbv();
    }
  }
};
}
````

### NBV_paper_ros2/src/auv_motion_control/include/auv_motion_control/use_cases/waypoint_controller.hpp

SHA-256: `e22eb41ef880f3fbce9d23173903830d43ed4f1077502801490ff3cb7cde6b48`

````cpp
#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include "auv_motion_control/domain/contracts.hpp"
#include "auv_motion_control/domain/math_pid_utils.hpp"
namespace auv_motion_control
{
class WaypointController {
  Clock & clock_; Gains & gains_; Diagnostics & diagnostics_; CommandOutput & output_;
  const Modes & modes_; const ThrusterMixer & mixer_;
  std::vector<double> waypoint_;
  std::vector<double> magnet_;
  std::vector<float> obstacles_;
  std::vector<double> motor_command_;

  double current_yaw_ = 0.0;
  double original_yaw_ = 0.0;
  double current_depth_ = 0.0;
  double current_dist_seabed_ = 0.0;

  utils::PID pid_dist_;
  utils::PID pid_yaw_;

  double kp_dist_;
  double kp_yaw_;
  double stopping_distance_ = 0.25;
  double initial_waypoint_x_ = 10.0;
  double initial_waypoint_y_ = 10.0;
  double initial_waypoint_z_ = 5.0;
  double max_yaw_command_ = 10.0;
  double yaw_smoothing_rate_ = 0.8;
  double min_forward_thrust_ = 1.0;
  double max_forward_thrust_ = 10.0;
  double command_multiplier_ = 2.0;

  bool finished_execution_ = false;
  std::optional<double> hardcoded_angle_ = std::nullopt;
  std::optional<double> fixed_angle_ = std::nullopt;
  bool yaw_smoothing_initialized_ = false;
  double smoothed_desired_yaw_ = 0.0;
  double last_yaw_smoothing_time_ = 0.0;
  std::string mode_ = "yaw_centerize";

  std::vector<double> last_base_command_;
  std::vector<double> last_motor_command_;

  std::vector<double> current_position_ = {0.0, 0.0, 0.0};
  std::vector<double> dvl_position_ = {0.0, 0.0, 0.0};
  std::optional<std::vector<double>> dvl_reset_position_ = std::nullopt;
  bool dvl_active_;  // Assuming True based on Python fallback

  // --- Publishers ---

public:
  WaypointController(
    const ControlConfig & config, Clock & clock, Gains & gains,
    Diagnostics & diagnostics, CommandOutput & output, const Modes & modes,
    const ThrusterMixer & mixer)
  : clock_(clock), gains_(gains), diagnostics_(diagnostics), output_(output),
    modes_(modes), mixer_(mixer), pid_dist_(0.75, 0.3, 0.1, 0.0, 0.2),
    pid_yaw_(0.6, 0.3, 0.1, 0.0, 0.2)
  {
    kp_yaw_ = config.kp_yaw;
    kp_dist_ = config.kp_dist;
    dvl_active_ = config.dvl_active;
    stopping_distance_ = config.stopping_distance;
    initial_waypoint_x_ = config.initial_waypoint_x;
    initial_waypoint_y_ = config.initial_waypoint_y;
    initial_waypoint_z_ = config.initial_waypoint_z;
    max_yaw_command_ = config.max_yaw_command;
    yaw_smoothing_rate_ = config.yaw_smoothing_rate;
    min_forward_thrust_ = config.min_forward_thrust;
    max_forward_thrust_ = config.max_forward_thrust;
    command_multiplier_ = config.command_multiplier;
    waypoint_ = {initial_waypoint_x_, initial_waypoint_y_, initial_waypoint_z_};
    magnet_ = {0.0, 0.0, 0.0};
    obstacles_.resize(9, 0.0);
    motor_command_.resize(8, 0.0);
    last_base_command_.resize(8, 0.0);
    last_motor_command_.resize(8, 0.0);


  }
  void start()
  {
    move_towards_waypoint({initial_waypoint_x_, initial_waypoint_y_, initial_waypoint_z_});
  }
  double get_time_now() {return clock_.now();}

  double smooth_desired_yaw(double target_yaw)
  {
    target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
    const double now = get_time_now();
    if (!yaw_smoothing_initialized_) {
      smoothed_desired_yaw_ = target_yaw;
      last_yaw_smoothing_time_ = now;
      yaw_smoothing_initialized_ = true;
      return smoothed_desired_yaw_;
    }

    const double dt = std::max(0.0, now - last_yaw_smoothing_time_);
    last_yaw_smoothing_time_ = now;
    const double yaw_error = std::atan2(
      std::sin(target_yaw - smoothed_desired_yaw_),
      std::cos(target_yaw - smoothed_desired_yaw_));
    const double max_step = std::max(0.0, yaw_smoothing_rate_) * dt;
    smoothed_desired_yaw_ += std::clamp(yaw_error, -max_step, max_step);
    smoothed_desired_yaw_ = std::atan2(
      std::sin(smoothed_desired_yaw_), std::cos(smoothed_desired_yaw_));
    return smoothed_desired_yaw_;
  }

  void publish_cmd(std::vector<double> motor_command)
  {
    last_base_command_ = motor_command;
    last_motor_command_ = motor_command;

    output_.command(MotorCommand{motor_command});
  }

  void finished_execution_callback(bool value)
  {
    finished_execution_ = value;
  }

  void mode_callback(const std::string & value) {mode_ = value;}

  void angle_callback(float value)
  {
    if (modes_.select(mode_).accepts_angle) {hardcoded_angle_ = value;}
  }

  void dvl_reset_callback(bool value)
  {
    if (value) {
      dvl_reset_position_ = dvl_position_;
      dvl_active_ = true;
      diagnostics_.write(format_text("DVL position reset"));
    }
  }

  void position_callback(const Position & msg)
  {
    auto pos = msg;
    dvl_position_ = {pos.x, pos.y, 0.0};
    if (dvl_active_) {
      current_position_.assign({pos.x, pos.y, pos.z});
    } else {
      current_position_.assign({0.0, 0.0, pos.z});
    }
    move_towards_waypoint(waypoint_);
  }

  void magnetometer_callback(const Position & msg)
  {
    magnet_ = {msg.x, msg.y, msg.z};
    double ya = std::atan2(magnet_[0], magnet_[1]) - (M_PI / 2.0);
    if (ya < -M_PI) {ya += M_PI * 2.0;}
    current_yaw_ = ya;
    original_yaw_ = std::atan2(magnet_[0], magnet_[1]);
  }

  void depth_callback(float value) {current_depth_ = value;}

  void dist_callback(float value)
  {
    current_dist_seabed_ = value;
  }

  void obstacle_callback(const WaypointInput & msg)
  {
    obstacles_.assign(msg.data.begin(), msg.data.end());
  }

  void waypoint_callback(const WaypointInput & msg)
  {
    if (msg.data.size() >= 3) {
      waypoint_ = {msg.data[0], msg.data[1], msg.data[2]};
      if (msg.data.size() >= 4) {fixed_angle_ = msg.data[3];}
      diagnostics_.write(format_text("Received new waypoint: [%.2f, %.2f, %.2f]", waypoint_[0],
          waypoint_[1], waypoint_[2]));
      if (fixed_angle_.has_value()) {
        diagnostics_.write(format_text("Fixed angle: [%.2f]", fixed_angle_.value()));
      }
      move_towards_waypoint(waypoint_);
    }
  }

  void move_towards_waypoint(std::vector<double> target)
  {
    if (finished_execution_) {return;}

    if (dvl_active_ && dvl_reset_position_.has_value()) {
      for (size_t i = 0; i < 3; ++i) {target[i] += dvl_reset_position_.value()[i];}
    }
    diagnostics_.write(format_text(
        "From current position: [%.2f, %.2f, %.2f] moving towards waypoint: [%.2f, %.2f, %.2f]",
        current_position_[0], current_position_[1], current_position_[2], target[0], target[1],
        target[2]));

    std::vector<double> direction(3);
    for (size_t i = 0; i < 3; ++i) {direction[i] = target[i] - current_position_[i];}
    if (std::abs(direction[2]) <= stopping_distance_) {direction[2] = 0.0;}

    double distance = std::sqrt(
      direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);

    // Stopping condition
    if (utils::diff_2d(target, current_position_) < stopping_distance_) {
      std::vector<float> f_target(target.begin(), target.end());
      output_.reached(f_target);
    }

    if (distance == 0.0) {return;}
    for (size_t i = 0; i < 3; ++i) {direction[i] /= distance;}

    double target_yaw =
      fixed_angle_.has_value() ? fixed_angle_.value() : std::atan2(direction[1], direction[0]);
    double desired_yaw = smooth_desired_yaw(target_yaw);
    double raw_yaw_diff = desired_yaw - current_yaw_;
    double yaw_error = hardcoded_angle_.has_value() ?
      hardcoded_angle_.value() :
      std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));

    // Fetch dynamic parameters
    kp_yaw_ = gains_.yaw();
    kp_dist_ = gains_.distance();

    std::vector<double> cmd(8, 0.0);
    double vert_mult = modes_.select(mode_).vertical_multiplier;
    for (int i = 0; i < 4; i++) {cmd[i] = vert_mult * motor_command_[i];}

    // ==========================================
    // NEW: BLENDED CONTINUOUS CONTROL LOGIC
    // ==========================================

    // 1. Calculate Continuous Yaw (No Deadband)
    double yaw_command = 0.0;
    if (modes_.select(mode_).centered) {
      // YOUR original math: directly multiply error by gain
      yaw_command = kp_yaw_ * yaw_error;

      // Clamp it to maximum thrust
      yaw_command = std::clamp(
        yaw_command, -max_yaw_command_, max_yaw_command_);

      // YOUR original inversion to match thruster polarity
      yaw_command = -yaw_command;
    }

    // 2. Calculate Dynamic Surge Speed (Cosine Scaling)
    // Keep horizontal and vertical thrust independent to avoid Z motion
    // increasing with the distance to a far-away XY waypoint.
    double horizontal_distance = std::hypot(direction[0], direction[1]);
    double base_thrust = std::clamp(
      horizontal_distance * kp_dist_, min_forward_thrust_, max_forward_thrust_);
    double vertical_thrust = std::clamp(
      std::abs(direction[2]) * kp_dist_, 0.0, max_forward_thrust_);

    // Scale thrust based on how straight we are aiming to prevent corner overshoot
    // cos(yaw_error) smoothly drops the speed as the angle increases.
    // std::max(0.1, ...) ensures we never completely stop moving forward.
    double speed_scaling = std::max(0.1, std::cos(yaw_error));
    double forward_thrust = base_thrust * speed_scaling;

    // 3. Apply Positional Movement (Surge/Sway)
    auto auv_pos = utils::world_to_auv_coordinates(direction, 3.0 * M_PI / 2.0 - original_yaw_);
    auto keys = determine_keys(auv_pos);
    cmd = mixer_.mix(keys, forward_thrust, cmd, vertical_thrust);

    // 4. Overlay Yaw Command simultaneously
    if (modes_.select(mode_).centered) {
      // Note: Check if you need to invert yaw_command here based on your thruster layout
      mixer_.yaw(cmd, yaw_command);
    }

    // ==========================================

    motor_command_ = cmd;
    std::vector<double> scaled_cmd = cmd;
    for (auto & val : scaled_cmd) {val *= command_multiplier_;}

    publish_cmd(scaled_cmd);
  }

  std::vector<char> determine_keys(const std::vector<double> & dir)
  {
    std::vector<char> keys;
    if (obstacles_.size() > 4 && obstacles_[4] == 0.0) {
      if (dir[0] > 0.1) {
        keys.push_back('w');
      } else if (dir[0] < -0.1) {
        keys.push_back('s');
      }
      if (dir[1] > 0.1) {
        keys.push_back('d');
      } else if (dir[1] < -0.1) {
        keys.push_back('a');
      }
    } else {
      if (dir[0] > 0.1) {
        keys.push_back('d');
      } else if (dir[0] < -0.1) {
        keys.push_back('a');
      }
    }

    if (dir[2] > 0.1) {
      keys.push_back('i');
    } else if (dir[2] < -0.1) {
      keys.push_back('k');
    }
    return keys;
  }

  std::vector<double> apply_yaw_control(double yaw_error, double val, std::vector<double> cmd)
  {
    double yaw_command = std::clamp(kp_yaw_ * yaw_error, -val, val);
    yaw_command = -yaw_command;
    mixer_.yaw(cmd, yaw_command);
    return cmd;
  }
};
}
````

### NBV_paper_ros2/src/auv_motion_control/package.xml

SHA-256: `c5a0e8f73027740027fb5ab1c4ce9f3f7654b851dc1d85c707c888c96a4742fd`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_motion_control</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_motion_control/src/PathPlanning.py

SHA-256: `c70d81c1a26dfe80f10e1f4d407906d7272a5565efd955270572b07480264683`

````python
"""
Legacy two-dimensional waypoint planning.

Generate exploration targets using the optional repository exploration memory.
"""

from cv_bridge import CvBridge
from legacy_ports import Distance, ExplorationMemory
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
# from myMessages.msg import objectDetected2D


def boustrophedon_path_planning(area, step=20):
    """
    Generate 2D waypoints using Boustrophedon (lawnmower) pattern.

    Parameters
    ----------
    area : tuple
        Bounds ((xmin, xmax), (ymin, ymax)).
    step : float
        Distance between sweep lines in the input coordinate units.

    Returns
    -------
    list
        Waypoints as (x, y) coordinates.

    """
    (xmin, xmax), (ymin, ymax) = area

    # Create y positions for each sweep line
    y_positions = np.arange(ymin, ymax, step)

    waypoints = []

    for i, y in enumerate(y_positions):
        if i % 2 == 0:
            # Move left → right
            x_line = np.linspace(xmin, xmax, int((xmax - xmin) / step) + 1)
        else:
            # Move right → left
            x_line = np.linspace(xmax, xmin, int((xmax - xmin) / step) + 1)

        for x in x_line:
            waypoints.append((x, y))

    return waypoints
    # return [[15,-110]]


class PathPlanner(Node):
    def __init__(
        self, exploration: ExplorationMemory = None, distance: Distance = None
    ):
        if exploration is None or distance is None:
            from legacy_adapters import RepositoryExplorationMemory, repository_distance

            exploration = (
                exploration
                if exploration is not None
                else RepositoryExplorationMemory()
            )
            distance = distance if distance is not None else repository_distance
        self.exploration = exploration
        self.distance = distance
        super().__init__('path_planner')

        self.bridge = CvBridge()
        # self.areaSearch([(-300, -100), (-200, 190)])
        # self.areaSearch([(-15,-80), (-11,-90)]) #(-15,-80), (-11,-90) /// (-10, 20), (-115, -85)
        # 9,-93,-30
        self.waypoints = [
            (-16, -81, -30, 0)
        ]  # ,(-15,-80),(-11,-80),(-11,-90),(-15,-90)]
        self.camera_position = [0.35, 0.1125 + 0.275, -0.5]
        # self.waypoints = [(56,-119,2)]
        # self.waypoints = [(15,-110,10)] # docking station
        # self.waypoints = [(134,-106,-31.5)] # to the orange flare
        # self.waypoints = calculate_route((20, -130),(10,-157),(25, -150),10)
        # Legacy object-detection subscription remains disabled.
        self.stop_sub = self.create_subscription(
            Float32MultiArray, '/motion/report', self.stop_callback, 10
        )
        self.point_sub = self.create_subscription(
            Float32MultiArray, 'object/detected', self.point_callback, 10
        )
        self.pointSegment_sub = self.create_subscription(
            Float32MultiArray, 'object/areaWaypoints', self.pointSegment_callback, 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.publisher = self.create_publisher(
            Float32MultiArray, '/trajectory/waypoint', 10
        )
        self.planner_sub = self.create_publisher(Bool, '/planner/report', 10)

        self.get_logger().info('✅ Path planner Started')

    def areaSearch(self, area):
        area = ((area[0][0], area[1][0]), (area[1][1], area[0][1]))
        print('Area to search:', area)
        # try:
        wps = boustrophedon_path_planning(area, step=5)
        # except:
        #     area=((area[0][0],area[1][0]),(area[0][1],area[1][1]))

        print('Generated waypoints:', wps)
        for wp in wps:
            if not self.exploration.is_scanned((wp[0], wp[1])):
                print('Adding waypoint:', wp)
                self.waypoints.append((wp[0], wp[1]))
                self.exploration.add_waypoint((wp[0], wp[1]))
            else:
                print('Already scanned:', wp)

    def point_callback(self, msg):
        point = msg.data
        self.waypoints = [point.tolist()]
        self.get_logger().info(f'Published {point} waypoints.')

    def pointSegment_callback(self, msg):
        data = np.array(msg.data).reshape(-1, 4)
        wps = []
        for wp in data:
            print('Adding segmentation waypoint:', wp)
            if len(wp) == 3:
                wps.append((wp[0], wp[1], wp[2]))
            elif len(wp) == 4:
                wps.append(
                    (
                        wp[0] - self.camera_position[0],
                        wp[1] - self.camera_position[1],
                        wp[2] - self.camera_position[2],
                        wp[3],
                    )
                )
        if wps:
            self.waypoints = wps
            self.get_logger().info(f'Received {len(wps)} segmentation waypoints.')

    # def pointSegment_callback(self, msg):
    #     data=np.array(msg.data).reshape(-1,2)
    #     # self.areaSearch([data[:2],data[2:]])
    #     wps=[]
    #     for wp in data:
    #         if not isItScanned((wp[0],wp[1])):
    #             print("Adding segmentation waypoint:",wp)
    #             wps.append((wp[0],wp[1],4))
    #             addWaypoint((wp[0],wp[1]))
    #         else:
    #             print("Already scanned segmentation waypoint:",wp)
    #     # self.waypoints.extend(wps)
    #     self.waypoints=wps
    #     self.get_logger().info(f"Received {len(wps)} segmentation waypoints.")

    def timer_callback(self):
        if self.waypoints:
            msg = Float32MultiArray()
            flat_waypoints = []
            for wp in self.waypoints:
                if len(wp) == 2:
                    flat_waypoints.extend(
                        [wp[0], wp[1], -30.0]
                    )  # Assuming constant depth of -29.0
                elif len(wp) == 3:
                    flat_waypoints.extend(
                        [wp[0], wp[1], wp[2]]
                    )  # Ensure depth is not belove -30.0
                    # Alternate constant depth (-31.5) remains disabled.
                elif len(wp) == 4:
                    flat_waypoints.extend([wp[0], wp[1], wp[2], wp[3]])
            msg.data = flat_waypoints
            self.publisher.publish(msg)
            self.get_logger().info(f'Published {len(self.waypoints)} waypoints.')
        else:
            self.get_logger().info('No more waypoints to publish.')

    def stop_callback(self, msg):
        try:
            # data=tuple(msg.data[:2].tolist())
            data = msg.data[:3].tolist()
            if self.distance(self.waypoints[0][:2], data[:2]) < 1.0:
                self.waypoints.pop(0)
        except BaseException:
            return 0
        self.get_logger().info('Received stop signal, giving next waypoint.')
        msg = Float32MultiArray()
        # self.planner_sub.publish(Bool(data=True))# debug
        try:
            try:
                self.waypoints.remove(tuple(msg.data[:3].tolist()))
            except BaseException:
                self.waypoints.remove(data)
            self.exploration.add_waypoint(data[:2])
            if not self.waypoints:
                pass
                # self.planner_sub.publish(Bool(data=True))
            else:
                self.waypoints = self.waypoints[-5:]
        except ValueError:
            print('Waypoint not in list')

        self.timer_callback()

    # def cam_callback(self, msg):
    #     # Display if needed
    #     # cv2.imshow("HoloOcean Camera", frame); cv2.waitKey(1)
    #     # Create a dummy object detection message
    #     pos=msg.position

    #     self.target_pub.publish(pos)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
````

### NBV_paper_ros2/src/auv_motion_control/src/control_policy.cpp

SHA-256: `384369e907a71a49fcfef2a23a6b84f035cd5bcbb5cf145cdeda501efa0ca678`

````cpp
#include "auv_motion_control/use_cases/waypoint_controller.hpp"
#include "auv_motion_control/use_cases/trajectory_queue.hpp"
````

### NBV_paper_ros2/src/auv_motion_control/src/legacy_adapters.py

SHA-256: `1b742ff67d99cb4b01fb6986c880688042fb23a7728410b4922677a187f8d82e`

````python
"""
Explicit integration with optional repository-level legacy helpers.

Those helpers must be importable through the caller's normal Python environment.
This dormant adapter is not installed by the C++ package and is not launched.
"""


class RepositoryExplorationMemory:

    def __init__(self):
        from Tools.explore2D import addWaypoint, isItScanned

        self._add = addWaypoint
        self._scanned = isItScanned

    def add_waypoint(self, point):
        return self._add(point)

    def is_scanned(self, point):
        return self._scanned(point)


def repository_distance(first, second):
    from Tools.mapping2D import diff

    return diff(first, second)
````

### NBV_paper_ros2/src/auv_motion_control/src/legacy_ports.py

SHA-256: `296397db72f9523a823c5a9df843766ecfbb831a719feb36f8625fedeb7b5d11`

````python
"""Optional ports for the unbuilt, legacy 2D planner."""

from typing import Protocol


class ExplorationMemory(Protocol):

    def add_waypoint(self, point) -> None: ...
    def is_scanned(self, point) -> bool: ...


class Distance(Protocol):

    def __call__(self, first, second) -> float: ...
````

### NBV_paper_ros2/src/auv_motion_control/src/math_pid_utils.hpp

SHA-256: `6a7d722bc7fd1f7ffca8a2a60fe8b6e3a214af6b2ccb05cc9c3111314d4e02df`

````cpp
#pragma once
#include "auv_motion_control/domain/math_pid_utils.hpp"
````

### NBV_paper_ros2/src/auv_motion_control/src/point_iterator.cpp

SHA-256: `fcaf4ebf7812443af842738f056f838397dfb361c5063d2408451733426f7744`

````cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include "auv_motion_control/use_cases/trajectory_queue.hpp"
class PathPlanner : public rclcpp::Node, public auv_motion_control::WaypointOutput,
  public auv_motion_control::Diagnostics {
public:
  PathPlanner()
  : Node("path_planner")
  {
    this->declare_parameter<double>("camera_x", 0.35);
    this->declare_parameter<double>("camera_y", 0.1125 + 0.275);
    this->declare_parameter<double>("camera_z", -0.5);
    this->declare_parameter<double>("initial_waypoint_x", -16.0);
    this->declare_parameter<double>("initial_waypoint_y", -81.0);
    this->declare_parameter<double>("initial_waypoint_z", -30.0);
    this->declare_parameter<double>("initial_waypoint_yaw", 0.0);
    this->declare_parameter<double>("timer_period_s", 1.0);
    this->declare_parameter<double>("waypoint_reached_distance", 0.4);

        // Publishers
    waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/trajectory/waypoint",
      10);
    planner_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/report", 10);
    request_nbv_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/request_nbv", 1);

    const auv_motion_control::QueueConfig config{
      get_parameter("camera_x").as_double(),
      get_parameter("camera_y").as_double(),
      get_parameter("camera_z").as_double(),
      get_parameter("initial_waypoint_x").as_double(),
      get_parameter("initial_waypoint_y").as_double(),
      get_parameter("initial_waypoint_z").as_double(),
      get_parameter("initial_waypoint_yaw").as_double(),
      get_parameter("timer_period_s").as_double(),
      get_parameter("waypoint_reached_distance").as_double()};
    queue_ = std::make_unique<auv_motion_control::TrajectoryQueue>(config, *this, *this);
        // Subscribers
    stop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/motion/report", 10,
      std::bind(&PathPlanner::stop_callback, this, std::placeholders::_1));
    point_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object/detected", 10,
      std::bind(&PathPlanner::point_callback, this, std::placeholders::_1));
    point_segment_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object/areaWaypoints", 10,
      std::bind(&PathPlanner::pointSegment_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(config.timer_period_s)), [this]{queue_->timer_callback();});
    RCLCPP_INFO(get_logger(), "✅ Path planner Started");
  }
  void write(const std::string & text) override {RCLCPP_INFO(get_logger(), "%s", text.c_str());}
  void waypoints(const std::vector<float> & values, std::size_t count) override
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "waypoints"; msg.layout.dim[0].size = count;
    msg.layout.dim[0].stride = count * 4;
    msg.layout.dim[1].label = "coordinates"; msg.layout.dim[1].size = 4;
    msg.layout.dim[1].stride = 4; msg.data = values;
    waypoint_pub_->publish(msg);
  }
  void request_nbv() override
  {
    std_msgs::msg::Bool msg; msg.data = true; request_nbv_pub_->publish(msg);
  }

private:
  std::unique_ptr<auv_motion_control::TrajectoryQueue> queue_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_nbv_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_segment_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void point_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    queue_->point_callback(auv_motion_control::WaypointInput{msg->data});
  }
  void pointSegment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    queue_->pointSegment_callback(auv_motion_control::WaypointInput{msg->data});
  }
  void stop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    queue_->stop_callback(auv_motion_control::WaypointInput{msg->data});
  }

};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
````

### NBV_paper_ros2/src/auv_motion_control/src/thruster_adapters.cpp

SHA-256: `aebe741d5be59c3517d5edfd258860e01bf498541060a6c9da33d0f491a421a8`

````cpp
#include "auv_motion_control/adapters/thrusters.hpp"
````

### NBV_paper_ros2/src/auv_motion_control/src/waypoint_control_node.cpp

SHA-256: `87e288827e83ffeed54751ada265a0ea7c0079400827efae1b09de6027d8953b`

````cpp
#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "auv_motion_control/use_cases/waypoint_controller.hpp"
#include "auv_motion_control/adapters/ros_control_io.hpp"
#include "auv_motion_control/adapters/thrusters.hpp"

using std::placeholders::_1;

class WaypointControlNode : public rclcpp::Node {
public:
  WaypointControlNode()
  : Node("waypoint_control_node")
  {
    // --- Declare and Get Parameters from config/params.yaml ---
    this->declare_parameter<double>("kp_yaw", 0.6);
    this->declare_parameter<double>("kp_dist", 0.75);
    this->declare_parameter<bool>("dvl_active", false);
    this->declare_parameter<double>("stopping_distance", 0.25);
    this->declare_parameter<double>("initial_waypoint_x", 10.0);
    this->declare_parameter<double>("initial_waypoint_y", 10.0);
    this->declare_parameter<double>("initial_waypoint_z", 5.0);
    this->declare_parameter<double>("max_yaw_command", 10.0);
    this->declare_parameter<double>("yaw_smoothing_rate", 0.8);
    this->declare_parameter<double>("min_forward_thrust", 1.0);
    this->declare_parameter<double>("max_forward_thrust", 10.0);
    this->declare_parameter<double>("command_multiplier", 2.0);

    // Publishers
    target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/holo/cmd", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motion/report", 10);

    const auv_motion_control::ControlConfig config{get_parameter("kp_yaw").as_double(),
      get_parameter("kp_dist").as_double(),
      get_parameter("dvl_active").as_bool(),
      get_parameter("stopping_distance").as_double(),
      get_parameter("initial_waypoint_x").as_double(),
      get_parameter("initial_waypoint_y").as_double(),
      get_parameter("initial_waypoint_z").as_double(),
      get_parameter("max_yaw_command").as_double(),
      get_parameter("yaw_smoothing_rate").as_double(),
      get_parameter("min_forward_thrust").as_double(),
      get_parameter("max_forward_thrust").as_double(),
      get_parameter("command_multiplier").as_double()};
    io_ = std::make_unique<auv_motion_control::RosControlIO>(*this, target_pub_, stop_pub_);
    controller_ = std::make_unique<auv_motion_control::WaypointController>(
      config, *io_, *io_, *io_, *io_, modes_, mixer_);
    // Subscribers
    waypoint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/trajectory/waypoint", 10, std::bind(&WaypointControlNode::waypoint_callback, this, _1));
    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "holocean/odom", 10, std::bind(&WaypointControlNode::position_callback, this, _1));
    distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/distance", 10, std::bind(&WaypointControlNode::dist_callback, this, _1));
    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/scalar", 10, std::bind(&WaypointControlNode::depth_callback, this, _1));
    sub_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "holocean/mag", 10, std::bind(&WaypointControlNode::magnetometer_callback, this, _1));
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "object/obstacle", 10, std::bind(&WaypointControlNode::obstacle_callback, this, _1));
    angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/aruco/angle", 10, std::bind(&WaypointControlNode::angle_callback, this, _1));
    // mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    //   "/movement/mode", 10, std::bind(&WaypointControlNode::mode_callback, this, _1));
    finished_execution_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/movement/finished_execution", 10,
      std::bind(&WaypointControlNode::finished_execution_callback, this, _1));
    dvl_reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/movement/dvl_reset", 10, std::bind(&WaypointControlNode::dvl_reset_callback, this, _1));

    controller_->start();
    RCLCPP_INFO(this->get_logger(), "✅ Waypoint Control Node Started (Cleaned Edition)");
  }

private:
  auv_motion_control::ConfiguredModes modes_;
  auv_motion_control::EightThrusterMixer mixer_;
  std::unique_ptr<auv_motion_control::RosControlIO> io_;
  std::unique_ptr<auv_motion_control::WaypointController> controller_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr stop_pub_;

  // --- Subscribers ---
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_, depth_sub_, angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_execution_sub_, dvl_reset_sub_;

  void finished_execution_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {controller_->finished_execution_callback(msg->data);}
  void mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    controller_->mode_callback(msg->data);
  }
  void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {controller_->angle_callback(msg->data);}
  void dvl_reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {controller_->dvl_reset_callback(msg->data);}
  void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    controller_->position_callback({msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z});
  }
  void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    controller_->magnetometer_callback({msg->magnetic_field.x, msg->magnetic_field.y,
        msg->magnetic_field.z});
  }
  void depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    controller_->depth_callback(msg->data);
  }
  void dist_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {controller_->dist_callback(msg->data);}
  void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {controller_->obstacle_callback(auv_motion_control::WaypointInput{msg->data});}
  void waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    controller_->waypoint_callback(auv_motion_control::WaypointInput{msg->data});
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointControlNode>());
  rclcpp::shutdown();
  return 0;
}
````

### NBV_paper_ros2/src/auv_motion_control/test/policy_regression.cpp

SHA-256: `c6f17d5baf91dcbea7caf27ee20558e4a60c529d7af8c25ab2319b7c6b20f544`

````cpp
#include <cmath>
#include <cstdlib>
#include <vector>
#include "auv_motion_control/use_cases/trajectory_queue.hpp"
#include "auv_motion_control/adapters/thrusters.hpp"

void require(bool condition) {if(!condition) {std::abort();}}
struct Output final : auv_motion_control::WaypointOutput, auv_motion_control::Diagnostics
{
  std::vector<std::vector<float>> messages;
  std::size_t requests = 0;
  void waypoints(const std::vector<float> & values, std::size_t count) override
  {
    require(values.size() == count * 4); messages.push_back(values);
  }
  void request_nbv() override {++requests;}
  void write(const std::string &) override {}
};
int main()
{
  Output output;
  auv_motion_control::TrajectoryQueue queue({.35, .3875, -.5, 354., 828., -298., 0., 1., .5}, output,
    output);
  queue.timer_callback();
  require(output.messages.back() == std::vector<float>({354.f, 828.f, -298.f, 0.f}));
  queue.pointSegment_callback({{10.f, 20.f, -30.f, 1.f, 11.f, 21.f, -31.f, 2.f}});
  const auto encoded = output.messages.back();
  require(encoded.size() == 8 && std::abs(encoded[0] - 9.65f) < 1e-6);
  require(std::abs(encoded[1] - 19.6125f) < 1e-6 && encoded[2] == -29.5f && encoded[3] == 1.f);
  // Preserve the existing empty-message behavior: the queue retains its route.
  queue.pointSegment_callback({{}});
  queue.timer_callback();
  require(output.messages.back() == encoded);
  queue.stop_callback({{encoded[0], encoded[1], encoded[2]}});
  require(output.messages.back().size() == 4);
  const auto last = output.messages.back();
  queue.stop_callback({{last[0], last[1], last[2]}});
  require(output.requests == 1);
  auv_motion_control::EightThrusterMixer mixer;
  auto mixed = mixer.mix({'w', 'a', 'i'}, 2., std::vector<double>(8, 0.), 3.);
  require(mixed == std::vector<double>({3., 3., 3., 3., 0., -4., 0., -4.}));
  mixer.yaw(mixed, 1.);
  require(mixed == std::vector<double>({3., 3., 3., 3., -1., -3., 1., -5.}));
  auv_motion_control::ConfiguredModes modes;
  require(modes.select("full_centerize").vertical_multiplier == .25);
  require(!modes.select("unknown").centered);
}
````

### NBV_paper_ros2/src/auv_stereo_depth/package.xml

SHA-256: `7304b469fb1873e6d03d5c909e9d7aaa99f05974ab5c69f8ebf28493b241f865`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_stereo_depth</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>message_filters</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>auv_contracts</exec_depend>
  <exec_depend>auv_telemetry</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-torch</exec_depend>
  <exec_depend>python3-torchvision</exec_depend>
  <exec_depend>python3-pil</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/datasets/kitti12_dataset.py

SHA-256: `3087cb23473311a925890d0092a923f5226eb9871e349fcb65083e48bfd1348f`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
import os
import random
from torch.utils.data import Dataset
from PIL import Image
import numpy as np
from .data_io import get_transform, read_all_lines
import cv2
import torchvision
from PIL import ImageFilter
class KITTI_12_Dataset(Dataset):
    def __init__(self, datapath, list_filename, training):
        self.datapath = datapath
        self.left_filenames, self.right_filenames, self.disp_filenames,self.mask_filenames = self.load_path(list_filename)
        self.training = training
        if self.training:
            assert self.disp_filenames is not None
    def load_path(self, list_filename):
        lines = read_all_lines(list_filename)
        splits = [line.split() for line in lines]
        left_images = [x[0] for x in splits]
        right_images = [x[1] for x in splits]
        if len(splits[0]) == 2:  # ground truth not available
            return left_images, right_images, None
        elif len(splits[0]) == 3:
            disp_images = [x[2] for x in splits]
            return left_images, right_images, disp_images,None
        else:
            disp_images = [x[2] for x in splits]
            mask_images = [x[3] for x in splits]
            return left_images, right_images, disp_images,mask_images
    def load_image(self, filename):
        return Image.open(filename).convert('L')

    def load_disp(self, filename):
        data = Image.open(filename)
        data = np.array(data, dtype=np.float32) / 256.
        return data

    def __len__(self):
        return len(self.left_filenames)

    def __getitem__(self, index):
        left_img = self.load_image(os.path.join(self.datapath, self.left_filenames[index]))
        right_img = self.load_image(os.path.join(self.datapath, self.right_filenames[index]))
        if self.mask_filenames:
            mask = self.load_image(os.path.join(self.datapath, self.mask_filenames[index]))
        else:
            mask = None
        if self.disp_filenames:  # has disparity ground truth
            disparity = self.load_disp(os.path.join(self.datapath, self.disp_filenames[index]))
            if self.mask_filenames:
                mask = np.asarray(mask)
                temp = mask > 0
                disparity = disparity * temp
        else:
            disparity = None

        if self.training:
            #rgb2gray
            left_img = left_img.convert('L')
            right_img = right_img.convert('L')
                
            w, h = left_img.size
            crop_w, crop_h = 512, 256

            x1 = random.randint(0, w - crop_w)
            y1 = random.randint(0, h - crop_h)

            # random crop
            left_img = left_img.crop((x1, y1, x1 + crop_w, y1 + crop_h))
            right_img = right_img.crop((x1, y1, x1 + crop_w, y1 + crop_h))
            disparity = disparity[y1:y1 + crop_h, x1:x1 + crop_w]
            
            left_img = np.ascontiguousarray(left_img, dtype=np.float32)
            right_img = np.ascontiguousarray(right_img, dtype=np.float32)
            # to tensor, normalize
            preprocess = get_transform()
            left_img = preprocess(left_img)
            right_img = preprocess(right_img)
            disparity = np.expand_dims(disparity, 0)
            # return [left_img,right_img],-disparity
            return {"left": left_img,
                   "right": right_img,
                   "disparity": disparity}

        else:
            w, h = left_img.size
            top_pad = 384 - h
            right_pad = 1280 - w
            assert top_pad > 0 and right_pad > 0

                                   
            left_img = np.ascontiguousarray(left_img, dtype=np.float32)
            right_img = np.ascontiguousarray(right_img, dtype=np.float32)
                        
            left_img = np.lib.pad(left_img, ((top_pad, 0), (0, right_pad)), mode='symmetric', )
            right_img = np.lib.pad(right_img, ((top_pad, 0), (0, right_pad)), mode='symmetric')
            disparity = np.lib.pad(disparity, ((top_pad, 0), (0, right_pad)), mode='constant', constant_values=0)
            
            preprocess = get_transform()
            left_img = preprocess(left_img)
            right_img = preprocess(right_img)

            disparity = np.expand_dims(disparity, 0)
            # return [left_img,right_img],-disparity
            return {"left": left_img,
                   "right": right_img,
                   "disparity": disparity}
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/datasets/kitti_dataset.py

SHA-256: `0c3d5d68dd114acdda70aa21518528e56706755d324614c394540e92b4908b1f`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
import os
import random
from torch.utils.data import Dataset
from PIL import Image
import numpy as np
from .data_io import get_transform, read_all_lines
import cv2
import torchvision
from PIL import ImageFilter
class KITTIDataset(Dataset):
    def __init__(self, datapath, list_filename, training):
        self.datapath = datapath
        self.left_filenames, self.right_filenames, self.disp_filenames,self.mask_filenames = self.load_path(list_filename)
        self.training = training
        if self.training:
            assert self.disp_filenames is not None
    def load_path(self, list_filename):
        lines = read_all_lines(list_filename)
        splits = [line.split() for line in lines]
        left_images = [x[0] for x in splits]
        right_images = [x[1] for x in splits]
        if len(splits[0]) == 2:  # ground truth not available
            return left_images, right_images, None
        elif len(splits[0]) == 3:
            disp_images = [x[2] for x in splits]
            return left_images, right_images, disp_images,None
        else:
            disp_images = [x[2] for x in splits]
            mask_images = [x[3] for x in splits]
            return left_images, right_images, disp_images,mask_images
    def load_image(self, filename):
        return Image.open(filename).convert('L')

    def load_disp(self, filename):
        data = Image.open(filename)
        data = np.array(data, dtype=np.float32) / 256.
        return data

    def __len__(self):
        return len(self.left_filenames)

    def __getitem__(self, index):
        left_img = self.load_image(os.path.join(self.datapath, self.left_filenames[index]))
        right_img = self.load_image(os.path.join(self.datapath, self.right_filenames[index]))
        if self.mask_filenames:
            mask = self.load_image(os.path.join(self.datapath, self.mask_filenames[index]))
        else:
            mask = None
        if self.disp_filenames:  # has disparity ground truth
            disparity = self.load_disp(os.path.join(self.datapath, self.disp_filenames[index]))
            if self.mask_filenames:
                mask = np.asarray(mask)
                temp = mask > 0
                disparity = disparity * temp
        else:
            disparity = None

        if self.training:
            #rgb2gray
            left_img = left_img.convert('L')
            right_img = right_img.convert('L')
            
            w, h = left_img.size
            crop_w, crop_h = 512, 256

            x1 = random.randint(0, w - crop_w)
            y1 = random.randint(0, h - crop_h)

            # random crop
            left_img = left_img.crop((x1, y1, x1 + crop_w, y1 + crop_h))
            right_img = right_img.crop((x1, y1, x1 + crop_w, y1 + crop_h))
            disparity = disparity[y1:y1 + crop_h, x1:x1 + crop_w]
            
            left_img = np.ascontiguousarray(left_img, dtype=np.float32)
            right_img = np.ascontiguousarray(right_img, dtype=np.float32)
            # to tensor, normalize
            preprocess = get_transform()
            left_img = preprocess(left_img)
            right_img = preprocess(right_img)
            disparity = np.expand_dims(disparity, 0)
            # return [left_img,right_img],-disparity
            return {"left": left_img,
                   "right": right_img,
                   "disparity": disparity}
        else:
            w, h = left_img.size
            top_pad = 384 - h
            right_pad = 1280 - w
            assert top_pad > 0 and right_pad > 0
                                 
            left_img = np.ascontiguousarray(left_img, dtype=np.float32)
            right_img = np.ascontiguousarray(right_img, dtype=np.float32)
            
            left_img = np.lib.pad(left_img, ((top_pad, 0), (0, right_pad)), mode='symmetric', )
            right_img = np.lib.pad(right_img, ((top_pad, 0), (0, right_pad)), mode='symmetric')
            disparity = np.lib.pad(disparity, ((top_pad, 0), (0, right_pad)), mode='constant', constant_values=0)
            
            preprocess = get_transform()
            left_img = preprocess(left_img)
            right_img = preprocess(right_img)
        
            disparity = np.expand_dims(disparity, 0)
            # return [left_img,right_img],-disparity
            return {"left": left_img,
                   "right": right_img,
                   "disparity": disparity}
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/models/bgnet.py

SHA-256: `e9b2f3b87ce1f6f7d193167aadb7e13da97c295a2d90bc02f66ee8b5bc050348`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
from __future__ import print_function
from .feature_extractor_fast import feature_extraction
from .submodules3d import CoeffsPredictor
from .submodules2d import HourglassRefinement
from .submodules import SubModule,convbn_2d_lrelu,convbn_3d_lrelu,convbn_2d_Tanh
if __package__ == 'models':
    from nets.warp import disp_warp
else:
    from ..nets.warp import disp_warp
import torch
import torch.nn as nn
import torch.nn.functional as F
import time        
class Slice(SubModule):
    def __init__(self):
        super(Slice, self).__init__()
    def forward(self, bilateral_grid, wg, hg, guidemap): 
        guidemap = guidemap.permute(0,2,3,1).contiguous() #[B,C,H,W]-> [B,H,W,C]
        guidemap_guide = torch.cat([wg, hg, guidemap], dim=3).unsqueeze(1) # Nx1xHxWx3
        coeff = F.grid_sample(bilateral_grid, guidemap_guide,align_corners =False)
        return coeff.squeeze(2) #[B,1,H,W]

      
class GuideNN(SubModule):
    def __init__(self, params=None):
        super(GuideNN, self).__init__()
        self.params = params
        self.conv1 = convbn_2d_lrelu(32, 16, 1, 1, 0)
        self.conv2 = convbn_2d_Tanh(16, 1, 1, 1, 0)

    def forward(self, x):
        return self.conv2(self.conv1(x)) 
        
def groupwise_correlation(fea1, fea2, num_groups):
    B, C, H, W = fea1.shape
    assert C % num_groups == 0
    channels_per_group = C // num_groups
    cost = (fea1 * fea2).view([B, num_groups, channels_per_group, H, W]).mean(dim=2)
    assert cost.shape == (B, num_groups, H, W)
    return cost


def build_gwc_volume(refimg_fea, targetimg_fea, maxdisp, num_groups):
    B, C, H, W = refimg_fea.shape
    #[B,G,D,H,W]
    volume = refimg_fea.new_zeros([B, num_groups, maxdisp, H, W])
    for i in range(maxdisp):
        if i > 0:
            volume[:, :, i, :, i:] = groupwise_correlation(refimg_fea[:, :, :, i:], targetimg_fea[:, :, :, :-i],                                                           num_groups)
        else:
            volume[:, :, i, :, :] = groupwise_correlation(refimg_fea, targetimg_fea, num_groups)
    volume = volume.contiguous()
    return volume
def correlation(fea1, fea2):
    B, C, H, W = fea1.shape
    cost = (fea1 * fea2).mean(dim=1)
    assert cost.shape == (B, H, W)
    return cost

def disparity_regression(x, maxdisp):
    assert len(x.shape) == 4
    disp_values = torch.arange(0, maxdisp + 1, dtype=x.dtype, device=x.device)
    disp_values = disp_values.view(1, maxdisp + 1, 1, 1)
    return torch.sum(x * disp_values, 1, keepdim=True)
class BGNet(SubModule):
    def __init__(self):
        super(BGNet, self).__init__()
        self.softmax = nn.Softmax(dim = 1)
        # self.refinement_net = HourglassRefinement()
      
        self.feature_extraction = feature_extraction()
        self.coeffs_disparity_predictor = CoeffsPredictor()


        self.dres0 = nn.Sequential(convbn_3d_lrelu(44, 32, 3, 1, 1),
                                   convbn_3d_lrelu(32, 16, 3, 1, 1))
        self.guide = GuideNN()
        self.slice = Slice()
        self.weight_init()





 
    def forward(self, left_input, right_input):         
        left_low_level_features_1,  left_gwc_feature  = self.feature_extraction(left_input)
        _,                          right_gwc_feature = self.feature_extraction(right_input)
        
        guide = self.guide(left_low_level_features_1) #[B,1,H,W]
        # torch.cuda.synchronize()
        # start = time.time()
        cost_volume = build_gwc_volume(left_gwc_feature,right_gwc_feature,25,44)
        cost_volume = self.dres0(cost_volume)
        #coeffs:[B,D,G,H,W]
        coeffs = self.coeffs_disparity_predictor(cost_volume)

        list_coeffs = torch.split(coeffs,1,dim = 1)
        index = torch.arange(0,97)
        index_float = index/4.0
        index_a = torch.floor(index_float)
        index_b = index_a + 1
        
        index_a = torch.clamp(index_a, min=0, max= 24)
        index_b = torch.clamp(index_b, min=0, max= 24)
        
        wa = index_b - index_float
        wb = index_float   - index_a

        list_float = []  
        device = list_coeffs[0].get_device()
        wa  = wa.view(1,-1,1,1)
        wb  = wb.view(1,-1,1,1)
        wa = wa.to(device)
        wb = wb.to(device)
        wa = wa.float()
        wb = wb.float()

        N, _, H, W = guide.shape
        #[H,W]
        hg, wg = torch.meshgrid([torch.arange(0, H), torch.arange(0, W)]) # [0,511] HxW
        if device >= 0:
            hg = hg.to(device)
            wg = wg.to(device)
        #[B,H,W,1]
        hg = hg.float().repeat(N, 1, 1).unsqueeze(3) / (H-1) * 2 - 1 # norm to [-1,1] NxHxWx1
        wg = wg.float().repeat(N, 1, 1).unsqueeze(3) / (W-1) * 2 - 1 # norm to [-1,1] NxHxWx1
        slice_dict = []
        # torch.cuda.synchronize()
        # start = time.time()
        for i in range(25):
            slice_dict.append(self.slice(list_coeffs[i], wg, hg, guide)) #[B,1,H,W]
        slice_dict_a = []
        slice_dict_b = []
        for i in range(97):
            inx_a = i//4
            inx_b = inx_a + 1
            inx_b  = min(inx_b,24)
            slice_dict_a.append(slice_dict[inx_a])
            slice_dict_b.append(slice_dict[inx_b])
            
        final_cost_volume = wa * torch.cat(slice_dict_a,dim = 1) + wb * torch.cat(slice_dict_b,dim = 1)
        slice = self.softmax(final_cost_volume)
        disparity_samples = torch.arange(0, 97, dtype=slice.dtype, device=slice.device).view(1, 97, 1, 1)
        
        disparity_samples = disparity_samples.repeat(slice.size()[0],1,slice.size()[2],slice.size()[3])
        half_disp = torch.sum(disparity_samples * slice,dim = 1).unsqueeze(1)
        out2 = F.interpolate(half_disp * 2.0, scale_factor=(2.0, 2.0),
                                      mode='bilinear',align_corners =False).squeeze(1)
                                            
        return out2,out2

````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/models/bgnet_plus.py

SHA-256: `2f8bdf37dfbb4d10058942d7b51cc7c93a2887bddf62e2ba3d3bdd0b7f42105e`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
from __future__ import print_function
from .feature_extractor_fast import feature_extraction
from .submodules3d import CoeffsPredictor
from .submodules2d import HourglassRefinement
from .submodules import SubModule, convbn_2d_lrelu, convbn_3d_lrelu,convbn_2d_Tanh
if __package__ == 'models':
    from nets.warp import disp_warp
else:
    from ..nets.warp import disp_warp
import torch
import torch.nn as nn
import torch.nn.functional as F
import time
class Slice(SubModule):
    def __init__(self):
        super(Slice, self).__init__()
    
    def forward(self, bilateral_grid, wg, hg, guidemap): 
        guidemap = guidemap.permute(0,2,3,1).contiguous() #[B,C,H,W]-> [B,H,W,C]
        guidemap_guide = torch.cat([wg, hg, guidemap], dim=3).unsqueeze(1) # Nx1xHxWx3        
        coeff = F.grid_sample(bilateral_grid, guidemap_guide,align_corners =False)
        return coeff.squeeze(2) #[B,1,H,W]

       
class GuideNN(SubModule):
    def __init__(self, params=None):
        super(GuideNN, self).__init__()
        self.params = params
        self.conv1 = convbn_2d_lrelu(32, 16, 1, 1, 0)
        self.conv2 = convbn_2d_Tanh(16, 1, 1, 1, 0)

    def forward(self, x):
        return self.conv2(self.conv1(x))
        
def groupwise_correlation(fea1, fea2, num_groups):
    B, C, H, W = fea1.shape
    assert C % num_groups == 0
    channels_per_group = C // num_groups

    cost = (fea1 * fea2).view([B, num_groups, channels_per_group, H, W]).mean(dim=2)
    assert cost.shape == (B, num_groups, H, W)
    return cost


def build_gwc_volume(refimg_fea, targetimg_fea, maxdisp, num_groups):
    B, C, H, W = refimg_fea.shape
    #[B,G,D,H,W]
    volume = refimg_fea.new_zeros([B, num_groups, maxdisp, H, W])
    for i in range(maxdisp):
        if i > 0:            
            volume[:, :, i, :, i:] = groupwise_correlation(refimg_fea[:, :, :, i:], targetimg_fea[:, :, :, :-i],
                                                           num_groups)
        else:
            volume[:, :, i, :, :] = groupwise_correlation(refimg_fea, targetimg_fea, num_groups)
    volume = volume.contiguous()
    return volume
def correlation(fea1, fea2):
    B, C, H, W = fea1.shape
    cost = (fea1 * fea2).mean(dim=1)
    assert cost.shape == (B, H, W)
    return cost

def disparity_regression(x, maxdisp):
    assert len(x.shape) == 4
    disp_values = torch.arange(0, maxdisp + 1, dtype=x.dtype, device=x.device)
    disp_values = disp_values.view(1, maxdisp + 1, 1, 1)
    return torch.sum(x * disp_values, 1, keepdim=True)
class BGNet_Plus(SubModule):
    def __init__(self):
        super(BGNet_Plus, self).__init__()
        self.softmax = nn.Softmax(dim = 1)
        self.refinement_net = HourglassRefinement()
        self.feature_extraction = feature_extraction()
        self.coeffs_disparity_predictor = CoeffsPredictor()

        self.dres0 = nn.Sequential(convbn_3d_lrelu(44, 32, 3, 1, 1),
                                   convbn_3d_lrelu(32, 16, 3, 1, 1))
        self.guide = GuideNN()
        self.slice = Slice()
        self.weight_init()
 
    def forward(self, left_input, right_input):        


        left_low_level_features_1,  left_gwc_feature  = self.feature_extraction(left_input)
        _,                          right_gwc_feature = self.feature_extraction(right_input)
        guide = self.guide(left_low_level_features_1) #[B,1,H,W]
        cost_volume = build_gwc_volume(left_gwc_feature,right_gwc_feature,25,44)

        cost_volume = self.dres0(cost_volume)
        coeffs = self.coeffs_disparity_predictor(cost_volume)

        list_coeffs = torch.split(coeffs,1,dim = 1)
        index = torch.arange(0,97)
        index_float = index/4.0
        index_a = torch.floor(index_float)
        index_b = index_a + 1
        
        index_a = torch.clamp(index_a, min=0, max= 24)
        index_b = torch.clamp(index_b, min=0, max= 24)
        
        wa = index_b - index_float
        wb = index_float   - index_a

        list_float = []  
        device = list_coeffs[0].get_device()
        wa  = wa.view(1,-1,1,1)
        wb  = wb.view(1,-1,1,1)
        wa = wa.to(device)
        wb = wb.to(device)
        wa = wa.float()
        wb = wb.float()

        N, _, H, W = guide.shape
        #[H,W]
        hg, wg = torch.meshgrid([torch.arange(0, H), torch.arange(0, W)]) # [0,511] HxW
        if device >= 0:
            hg = hg.to(device)
            wg = wg.to(device)
        #[B,H,W,1]
        hg = hg.float().repeat(N, 1, 1).unsqueeze(3) / (H-1) * 2 - 1 # norm to [-1,1] NxHxWx1
        wg = wg.float().repeat(N, 1, 1).unsqueeze(3) / (W-1) * 2 - 1 # norm to [-1,1] NxHxWx1
        slice_dict = []
        # torch.cuda.synchronize()
        # start = time.time()
        for i in range(25):
            slice_dict.append(self.slice(list_coeffs[i], wg, hg, guide)) #[B,1,H,W]
        slice_dict_a = []
        slice_dict_b = []
        for i in range(97):
            inx_a = i//4
            inx_b = inx_a + 1
            inx_b  = min(inx_b,24)
            slice_dict_a.append(slice_dict[inx_a])
            slice_dict_b.append(slice_dict[inx_b])
            
        final_cost_volume = wa * torch.cat(slice_dict_a,dim = 1) + wb * torch.cat(slice_dict_b,dim = 1)
        slice = self.softmax(final_cost_volume)
        disparity_samples = torch.arange(0, 97, dtype=slice.dtype, device=slice.device).view(1, 97, 1, 1)
        
        disparity_samples = disparity_samples.repeat(slice.size()[0],1,slice.size()[2],slice.size()[3])
        half_disp = torch.sum(disparity_samples * slice,dim = 1).unsqueeze(1)
        left_half = F.interpolate(
                            left_input,
                            scale_factor=1 / pow(2, 1),
                            mode='bilinear',
                            align_corners=False)
        right_half = F.interpolate(
                    right_input,
                    scale_factor=1 / pow(2, 1),
                    mode='bilinear',
                    align_corners=False)
        refinement_disp = self.refinement_net(half_disp,left_half,right_half)
        out1 = F.interpolate(refinement_disp * 2.0, scale_factor=(2.0, 2.0),
                              mode='bilinear',align_corners =False).squeeze(1)
        out2 = F.interpolate(half_disp * 2.0, scale_factor=(2.0, 2.0),
                                      mode='bilinear',align_corners =False).squeeze(1)
                                            
        return out1,out2        
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/models/submodules2d.py

SHA-256: `71a652ad5f9361313721e3ec4dc7dc07cc7ce5aabffb61b74d22198653bbf8ea`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
from __future__ import print_function
import torch.nn as nn
from .submodules import SubModule, convbn_2d_lrelu,deconvbn_2d_lrelu
import torch 
if __package__ == 'models':
    from nets.warp import disp_warp
else:
    from ..nets.warp import disp_warp
import torch.nn.functional as F
def disparity_regression(x, maxdisp):
    assert len(x.shape) == 4
    disp_values = torch.arange(0, maxdisp + 1, dtype=x.dtype, device=x.device)
    disp_values = disp_values.view(1, maxdisp + 1, 1, 1)
    return torch.sum(x * disp_values, 1, keepdim=True)
def conv2d(in_channels, out_channels, kernel_size=3, stride=1, dilation=1, groups=1):
    return nn.Sequential(nn.Conv2d(in_channels, out_channels, kernel_size=kernel_size,
                                   stride=stride, padding=dilation, dilation=dilation,
                                   bias=False, groups=groups),
                         nn.BatchNorm2d(out_channels),
                         nn.LeakyReLU(0.1, inplace=True))
# GANet feature
class BasicConv(nn.Module):

    def __init__(self, in_channels, out_channels, deconv=False, is_3d=False, bn=True, relu=True, **kwargs):
        super(BasicConv, self).__init__()
        self.relu = relu
        self.use_bn = bn
        if is_3d:
            if deconv:
                self.conv = nn.ConvTranspose3d(in_channels, out_channels, bias=False, **kwargs)
            else:
                self.conv = nn.Conv3d(in_channels, out_channels, bias=False, **kwargs)
            self.bn = nn.BatchNorm3d(out_channels)
        else:
            if deconv:
                self.conv = nn.ConvTranspose2d(in_channels, out_channels, bias=False, **kwargs)
            else:
                self.conv = nn.Conv2d(in_channels, out_channels, bias=False, **kwargs)
            self.bn = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        x = self.conv(x)
        if self.use_bn:
            x = self.bn(x)
        if self.relu:
            x = F.relu(x, inplace=True)
        return x


class Conv2x(nn.Module):

    def __init__(self, in_channels, out_channels, deconv=False, is_3d=False, concat=True, bn=True, relu=True,
                 mdconv=False):
        super(Conv2x, self).__init__()
        self.concat = concat

        if deconv and is_3d:
            kernel = (3, 4, 4)
        elif deconv:
            kernel = 4
        else:
            kernel = 3
        self.conv1 = BasicConv(in_channels, out_channels, deconv, is_3d, bn=True, relu=True, kernel_size=kernel,
                               stride=2, padding=1)

        if self.concat:
            # if mdconv:
                # self.conv2 = DeformConv2d(out_channels * 2, out_channels, kernel_size=3, stride=1)
            # else:
            self.conv2 = BasicConv(out_channels * 2, out_channels, False, is_3d, bn, relu, kernel_size=3,
                                       stride=1, padding=1)
        else:
            self.conv2 = BasicConv(out_channels, out_channels, False, is_3d, bn, relu, kernel_size=3, stride=1,
                                   padding=1)

    def forward(self, x, rem):
        x = self.conv1(x)
        assert (x.size() == rem.size())
        if self.concat:
            x = torch.cat((x, rem), 1)
        else:
            x = x + rem
        x = self.conv2(x)
        return x
class HourglassRefinement(SubModule):
    """Height and width need to be divided by 16"""

    def __init__(self):
        super(HourglassRefinement, self).__init__()

        # Left and warped error
        in_channels = 2
        self.conv1 = conv2d(in_channels, 16)
        self.conv2 = conv2d(1, 16)  # on low disparity

        self.conv_start = conv2d(32, 32)

        self.conv1a = BasicConv(32, 48, kernel_size=3, stride=2, padding=1)
        self.conv2a = BasicConv(48, 64, kernel_size=3, stride=2, padding=1)
        self.conv3a = BasicConv(64, 96, kernel_size=3, stride=2, padding=1)
        self.conv4a = BasicConv(96, 128, kernel_size=3, stride=2,padding=1)
        
        self.deconv4a = Conv2x(128, 96, deconv=True)
        self.deconv3a = Conv2x(96, 64, deconv=True)
        self.deconv2a = Conv2x(64, 48, deconv=True)
        self.deconv1a = Conv2x(48, 32, deconv=True)

        self.conv1b = Conv2x(32, 48)
        self.conv2b = Conv2x(48, 64)
        self.conv3b = Conv2x(64, 96)
        self.conv4b = Conv2x(96, 128)

        self.deconv4b = Conv2x(128, 96, deconv=True)
        self.deconv3b = Conv2x(96, 64, deconv=True)
        self.deconv2b = Conv2x(64, 48, deconv=True)
        self.deconv1b = Conv2x(48, 32, deconv=True)

        self.final_conv = nn.Conv2d(32, 1, 3, 1, 1)

    def forward(self, low_disp, left_img, right_img):
        scale_factor = left_img.size(-1) / low_disp.size(-1)
        if scale_factor == 1.0:
            disp = low_disp
        else:
            disp = F.interpolate(low_disp, size=left_img.size()[-2:], mode='bilinear')
            disp = disp * scale_factor
        warped_right = disp_warp(right_img, disp)[0]  # [B, C, H, W]
        error = warped_right - left_img  # [B, C, H, W]
        concat1 = torch.cat((error, left_img), dim=1)  # [B, 6, H, W]
        conv1 = self.conv1(concat1)  # [B, 16, H, W]
        conv2 = self.conv2(disp)  # [B, 16, H, W]
        x = torch.cat((conv1, conv2), dim=1)  # [B, 32, H, W]

        x = self.conv_start(x)
        rem0 = x
        x = self.conv1a(x)
        rem1 = x
        x = self.conv2a(x)
        rem2 = x
        x = self.conv3a(x)
        rem3 = x
        x = self.conv4a(x)
        rem4 = x

        x = self.deconv4a(x, rem3)
        rem3 = x

        x = self.deconv3a(x, rem2)
        rem2 = x
        x = self.deconv2a(x, rem1)
        rem1 = x
        x = self.deconv1a(x, rem0)
        rem0 = x

        x = self.conv1b(x, rem1)
        rem1 = x
        x = self.conv2b(x, rem2)
        rem2 = x
        x = self.conv3b(x, rem3)
        rem3 = x
        x = self.conv4b(x, rem4)

        x = self.deconv4b(x, rem3)
        x = self.deconv3b(x, rem2)
        x = self.deconv2b(x, rem1)
        x = self.deconv1b(x, rem0)  # [B, 32, H, W]

        residual_disp = self.final_conv(x)  # [B, 1, H, W]

        disp = F.relu(disp + residual_disp, inplace=True)  # [B, 1, H, W]
        # disp = disp.squeeze(1)  # [B, H, W]

        return disp            
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/models/submodules3d.py

SHA-256: `b76d54b1ea90a2faaa530f2eaf0a21df210f09a108ae8a6bf82f1f0a58bc8794`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
from __future__ import print_function
import torch
import torch.nn as nn
import torch.utils.data
from .submodules import SubModule, convbn_3d_lrelu, convbn_transpose_3d
class HourGlass(SubModule):
    def __init__(self, inplanes=16):
        super(HourGlass, self).__init__()

        self.conv1 = convbn_3d_lrelu(inplanes, inplanes * 2, kernel_size=3, stride=2, pad=1)
        self.conv2 = convbn_3d_lrelu(inplanes * 2, inplanes * 2, kernel_size=3, stride=1, pad=1)

        self.conv1_1 = convbn_3d_lrelu(inplanes * 2, inplanes * 4, kernel_size=3, stride=2, pad=1)
        self.conv2_1 = convbn_3d_lrelu(inplanes * 4, inplanes * 4, kernel_size=3, stride=1, pad=1)

        self.conv3 = convbn_3d_lrelu(inplanes * 4, inplanes * 8, kernel_size=3, stride=2, pad=1)
        self.conv4 = convbn_3d_lrelu(inplanes * 8, inplanes * 8, kernel_size=3, stride=1, pad=1)

        self.conv5 = convbn_transpose_3d(inplanes * 8, inplanes * 4, kernel_size=3, padding=1,
                                         output_padding=(0, 1, 1), stride=(1, 2, 2), bias=False)
        self.conv6 = convbn_transpose_3d(inplanes * 4, inplanes * 2, kernel_size=3, padding=1,
                                         output_padding=(0, 1, 1), stride=(1, 2, 2), bias=False)
        self.conv7 = convbn_transpose_3d(inplanes * 2, inplanes, kernel_size=3, padding=1,
                                         output_padding=(0, 1, 1), stride=(1, 2, 2), bias=False)
        self.last_for_guidance = convbn_3d_lrelu(inplanes, 32, kernel_size=3, stride=1, pad=1)
        self.weight_init()
#modify from Deeppruner code
class CoeffsPredictor(HourGlass):

    def __init__(self, hourglass_inplanes=16):
        super(CoeffsPredictor, self).__init__(hourglass_inplanes)

    def forward(self, input):
        output0 = self.conv1(input)
        output0_a = self.conv2(output0) + output0

        output0 = self.conv1_1(output0_a)
        output0_c = self.conv2_1(output0) + output0

        output0 = self.conv3(output0_c)
        output0 = self.conv4(output0) + output0

        output1 = self.conv5(output0) + output0_c
        output1 = self.conv6(output1) + output0_a
        output1 = self.conv7(output1)
        #[B,G,D,H,W] -> [B,D,G,H,W]
        coeffs = self.last_for_guidance(output1).permute(0,2,1,3,4).contiguous()
        return coeffs




````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/nets/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/predict.py

SHA-256: `15f69e97da89c61faf57019e0e3923e21b57f82f21e55e2032b6f97255bf9a27`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
from __future__ import print_function, division
import argparse
import os
import torch.backends.cudnn as cudnn
from torch.utils.data import DataLoader
import torch.utils.data
import time
if __package__:
    from .datasets import __datasets__
    from .prediction_registry import MODEL_FACTORIES, OUTPUT_SUBDIRECTORIES
else:
    from datasets import __datasets__
    from prediction_registry import MODEL_FACTORIES, OUTPUT_SUBDIRECTORIES
import gc
import skimage
import skimage.io
import skimage.transform
from PIL import Image
parser = argparse.ArgumentParser(description='BGNet')
parser.add_argument('--model', default='bgnet', help='select a model structure')
parser.add_argument('--dataset', required=True, help='dataset name', choices=__datasets__.keys())
parser.add_argument('--datapath', required=True, help='data path')
parser.add_argument('--savepath', required=True, help='save path')
parser.add_argument('--testlist', required=True, help='testing list')
parser.add_argument('--resume', required=True, help='the directory to save logs and checkpoints')

args = parser.parse_args()
datapath = args.datapath
StereoDataset = __datasets__[args.dataset]
kitti_real_test = args.testlist
kitti_real_test_dataset = StereoDataset(datapath, kitti_real_test, False)
TestImgLoader = DataLoader(kitti_real_test_dataset, batch_size= 1, shuffle=False, num_workers=1, drop_last=False)

if args.model in MODEL_FACTORIES:
    model = MODEL_FACTORIES[args.model]().cuda()
sub_name = OUTPUT_SUBDIRECTORIES.get(args.dataset)
checkpoint = torch.load(args.resume,map_location=lambda storage, loc: storage)
model.load_state_dict(checkpoint) 
model.eval()
for batch_idx, sample in enumerate(TestImgLoader):
    print('predict the sample:',batch_idx)
    imgL, imgR = sample['left'], sample['right']
    imgL = imgL.cuda()
    imgR = imgR.cuda()
    pred,_ = model(imgL, imgR)
    pred = pred[0].data.cpu().numpy() * 256
    # print('pred',pred.shape)
    filename = datapath + sub_name + '{:0>6d}'.format(batch_idx) +'_10.png'
    gt = Image.open(filename)
    w, h = gt.size
    top_pad = 384 - h
    right_pad = 1280 - w
    temp = pred[top_pad:, :-right_pad]
    os.makedirs(args.savepath,exist_ok=True)
    skimage.io.imsave(args.savepath + '{:0>6d}'.format(batch_idx) +'_10.png',temp.astype('uint16'))
    
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/predict_sample.py

SHA-256: `f2c08357faf70a94021611ce2b235446dc95b3dfc87666171cfeeb641b33928f`

````python
# -*- coding: UTF-8 -*-
# ---------------------------------------------------------------------------
# Official code of our paper:Bilateral Grid Learning for Stereo Matching Network
# Written by Bin Xu
# ---------------------------------------------------------------------------
from __future__ import print_function, division
import argparse
import os
import torch.backends.cudnn as cudnn
from torch.utils.data import DataLoader
import torch.utils.data
import time
import gc
import skimage
import skimage.io
import skimage.transform
import numpy as np
from PIL import Image
if __package__:
    from .datasets.data_io import get_transform
    from .models.bgnet import BGNet
    from .models.bgnet_plus import BGNet_Plus
else:
    from datasets.data_io import get_transform
    from models.bgnet import BGNet
    from models.bgnet_plus import BGNet_Plus

model = BGNet_Plus().cuda()

checkpoint = torch.load('models/Sceneflow-IRS-BGNet-Plus.pth',map_location=lambda storage, loc: storage)
model.load_state_dict(checkpoint) 
model.eval()
left_img = Image.open('sample/im0.png').convert('L')
right_img = Image.open('sample/im1.png').convert('L')
w, h = left_img.size
h1 = h % 64
w1 = w % 64
h1 = h  - h1
w1 =  w - w1
h1 = int(h1)
w1 = int(w1)
left_img = left_img.resize((w1, h1),Image.ANTIALIAS)
right_img = right_img.resize((w1, h1),Image.ANTIALIAS)
left_img = np.ascontiguousarray(left_img, dtype=np.float32)
right_img = np.ascontiguousarray(right_img, dtype=np.float32)
preprocess = get_transform()
left_img = preprocess(left_img)
right_img = preprocess(right_img)
pred,_ = model(left_img.unsqueeze(0).cuda(), right_img.unsqueeze(0).cuda()) 
pred = pred[0].data.cpu().numpy() * 256   
skimage.io.imsave('sample_disp.png',pred.astype('uint16'))
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/BGNet/prediction_registry.py

SHA-256: `b035c43939efc823cbd4159734124a982091eb49d7017bdb980f5d6f317c9082`

````python
"""Research-runner extension points; model and output choices live together here."""
if __package__:
    from .models.bgnet import BGNet
    from .models.bgnet_plus import BGNet_Plus
else:
    from models.bgnet import BGNet
    from models.bgnet_plus import BGNet_Plus

MODEL_FACTORIES = {'bgnet': BGNet, 'bgnet_plus': BGNet_Plus}
OUTPUT_SUBDIRECTORIES = {
    'kitti_12': 'testing/colored_0/',
    'kitti': 'testing/image_2/',
}
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/adapters/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/adapters/bgnet.py

SHA-256: `106a9bc1f5380040b5160ae37d0117b2b0993154e088006b6221cdf23c4d9d5c`

````python
"""Torch/OpenCV implementation. Model loading occurs only at construction."""
from pathlib import Path
import time
import cv2
import numpy as np
from PIL import Image
import torch
import torchvision.transforms as transforms
from ..BGNet.models.bgnet_plus import BGNet_Plus

class BGNetDisparityEstimator:
    def __init__(self, checkpoint_path=None, clock=time):
        self.clock = clock
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = BGNet_Plus().to(self.device)
        checkpoint_path = checkpoint_path or (
            Path(__file__).resolve().parents[1] / 'BGNet' / 'models' /
            'Sceneflow-IRS-BGNet-Plus.pth')
        checkpoint = torch.load(str(checkpoint_path), map_location=self.device)
        self.model.load_state_dict(checkpoint)
        self.model.eval()
        self.preprocess = transforms.Compose([transforms.ToTensor()])
        try:
            self.resample_filter = Image.ANTIALIAS
        except AttributeError:
            self.resample_filter = Image.Resampling.LANCZOS

    def estimate(self, left_img, right_img, started):
        # 1. Convert OpenCV BGR arrays to PIL Grayscale images
        left_pil = Image.fromarray(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY))
        right_pil = Image.fromarray(cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY))

        orig_w, orig_h = left_pil.size

        # 2. Resize to nearest multiple of 64 (Required by BGNet)
        h1 = int(orig_h - (orig_h % 64))
        w1 = int(orig_w - (orig_w % 64))

        left_resized = left_pil.resize((w1, h1), self.resample_filter)
        right_resized = right_pil.resize((w1, h1), self.resample_filter)

        # 3. Convert to Float32 arrays and apply transforms
        left_np = np.ascontiguousarray(left_resized, dtype=np.float32)
        right_np = np.ascontiguousarray(right_resized, dtype=np.float32)

        left_tensor = self.preprocess(left_np).unsqueeze(0).to(self.device)
        right_tensor = self.preprocess(right_np).unsqueeze(0).to(self.device)
        preprocessing_ms = (self.clock.perf_counter() - started) * 1000.0

        # 4. Neural Network Inference (No Gradients for speed and memory)
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)
        inference_started = self.clock.perf_counter()
        with torch.no_grad():
            pred, _ = self.model(left_tensor, right_tensor)
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)
        inference_ms = (self.clock.perf_counter() - inference_started) * 1000.0

        # 5. Extract disparity map in pixels (Do NOT multiply by 256)
        postprocessing_started = self.clock.perf_counter()
        disparity_resized = pred[0].data.cpu().numpy()

        # 6. Resize disparity back to original dimensions (e.g., 256x256 -> 288x288)
        # When upscaling the image, the pixel disparities must also be multiplied by the scale factor
        width_ratio = orig_w / w1
        disparity_orig = cv2.resize(disparity_resized, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        disparity_orig = disparity_orig * width_ratio


        return disparity_orig, preprocessing_ms, inference_ms, postprocessing_started
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/compute_depth.py

SHA-256: `6977583b06d0b5e9240e82f95c4026c95a7f97f8e8eff6c25ea4233060f0063e`

````python
"""Backward-compatible explicit composition facade for non-ROS callers."""
from auv_contracts.values import CameraCalibration
_default = None

def create_depth_use_case():
    import time
    from .adapters.bgnet import BGNetDisparityEstimator
    from .use_cases.depth import StereoDepthUseCase
    return StereoDepthUseCase(BGNetDisparityEstimator(clock=time), time)

def compute_stereo_depth(left_img, right_img, focal_length_px, baseline_meters):
    global _default
    if _default is None:
        _default = create_depth_use_case()
    return _default.compute(left_img, right_img,
                            CameraCalibration(focal_length_px, baseline_meters))

def get_last_depth_timings():
    return {} if _default is None else _default.timings()
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/depth_estimation.py

SHA-256: `46de5f82e0ca08c79ca051345910a16c3670e945103d1fc227328568a1d0842c`

````python
from .compute_depth import create_depth_use_case
from auv_contracts.values import CameraCalibration
from std_msgs.msg import Float32, Header, Float32MultiArray
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
from auv_telemetry.file_sinks import PipelineFileSink

# NEW: Import message_filters for stereo synchronization
import message_filters

class obstacleAvoidance(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.depth_use_case = create_depth_use_case()
        self.calibration = CameraCalibration(256, 0.5)
        self.bridge = CvBridge()
        self.declare_parameter('sync_queue_size', 1)
        self.declare_parameter('sync_slop_s', 0.02)
        self.declare_parameter('results_dir', 'results')
        sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        sync_slop_s = float(self.get_parameter('sync_slop_s').value)
        self.frame_id = 0
        self.telemetry = PipelineFileSink(
            self.get_parameter('results_dir').value,
            prefix='auv_stereo_depth')

        # Optimize QoS to ONLY keep the absolute newest frame (queue size 1)
        # This prevents the node from getting "stuck" processing old backlog frames
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1 
        )

        # 1. Create message filter subscribers (they do not trigger callbacks directly)
        self.sub_cam_left = message_filters.Subscriber(
            self, Image, 'holocean/camera/image_raw', qos_profile=qos)
        self.sub_cam_right = message_filters.Subscriber(
            self, Image, 'holocean/cameraDown/image_raw', qos_profile=qos)

        # 2. Synchronize the two image streams based on their timestamps.
        # Keep only the newest pair while the model is processing the previous one.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_cam_left, self.sub_cam_right], 
            queue_size=sync_queue_size,
            slop=sync_slop_s
        )
        self.ts.registerCallback(self.sync_cam_callback)

        self.depth_image_pub = self.create_publisher(Image, 'holocean/depth/image_raw', qos)

        self.get_logger().info("✅ obstacle_detector_node Started (Synchronized)")

    def sync_cam_callback(self, msg_left, msg_right):
        # This callback ONLY fires when a perfectly matched pair of images arrives
        callback_started = time.perf_counter()
        self.frame_id += 1
        frame_id = self.frame_id
        source_stamp_s = (
            msg_left.header.stamp.sec
            + msg_left.header.stamp.nanosec * 1e-9)

        # Convert Left Image
        conversion_started = time.perf_counter()
        frame_left = self.bridge.imgmsg_to_cv2(msg_left, desired_encoding='bgr8')
        frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)

        # Convert Right (Secondary) Image
        frame_right = self.bridge.imgmsg_to_cv2(msg_right, desired_encoding='bgr8')
        frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
        conversion_ms = (time.perf_counter() - conversion_started) * 1000.0

        # Compute Depth
        depth = self.depth_use_case.compute(frame_right, frame_left, self.calibration)
        depth_timings = self.depth_use_case.timings()
        for stage, duration_ms in depth_timings.items():
            if stage.endswith('_ms'):
                self.telemetry.record_stage(
                    stage, duration_ms, frame_id=frame_id,
                    source_stamp_s=source_stamp_s)

        # Publish using the exact timestamp of the left image to prevent temporal smearing downstream
        publish_started = time.perf_counter()
        self._publish_depth(depth, msg_left.header.stamp)
        publish_ms = (time.perf_counter() - publish_started) * 1000.0
        total_ms = (time.perf_counter() - callback_started) * 1000.0
        self.telemetry.record_stage(
            'image_conversion', conversion_ms, frame_id=frame_id,
            source_stamp_s=source_stamp_s)
        self.telemetry.record_stage(
            'depth_publish', publish_ms, frame_id=frame_id,
            source_stamp_s=source_stamp_s)
        self.telemetry.record_stage(
            'depth_callback_total', total_ms, frame_id=frame_id,
            source_stamp_s=source_stamp_s)

    def _publish_depth(self, depth_data, original_stamp):
        try:
            if depth_data is None:
                return

            depth_np = np.array(depth_data)
            if depth_np.ndim == 2 or (depth_np.ndim == 3 and depth_np.shape[2] == 1):
                if depth_np.ndim == 3:
                    depth_f = depth_np[:, :, 0].astype(np.float32)
                else:
                    depth_f = depth_np.astype(np.float32)

                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                # CRITICAL: Pass the timestamp of the original image, not the current time!
                img_msg.header.stamp = original_stamp
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            else:
                self.get_logger().warn("Unsupported depth shape: %s" % (str(depth_np.shape),))
        except Exception as e:
            self.get_logger().error("Failed to publish depth: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = obstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/domain/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/domain/depth.py

SHA-256: `2a6f2bf74ec133666b7e96f775d336c1b57d6516789ec7cf38dcd1b5362f43c3`

````python
from typing import Protocol
from auv_contracts.values import CameraCalibration

class DisparityEstimator(Protocol):
    def estimate(self, left, right, started):
        """Return disparity pixels, preprocessing/inference ms, postprocess start."""
        ...
````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/use_cases/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_stereo_depth/auv_stereo_depth/use_cases/depth.py

SHA-256: `92da45cffbb39e0a909b21e731f341177cb9a57ed5f4c9e57b5935588ddfecde`

````python
"""Metric depth policy over disparity values; no model or ROS dependencies."""
from auv_contracts.ports import Clock
from ..domain.depth import DisparityEstimator

class StereoDepthUseCase:
    def __init__(self, estimator: DisparityEstimator, clock: Clock):
        self.estimator = estimator
        self.clock = clock
        self.last_depth_timings = {}

    def compute(self, left_img, right_img, calibration):
        started = self.clock.perf_counter()
        (disparity_orig, preprocessing_ms, inference_ms,
         postprocessing_started) = self.estimator.estimate(left_img, right_img, started)
        # 7. Convert Disparity to Depth (Meters)
        # Filter invalid disparities to prevent divide-by-zero
        disparity_orig[disparity_orig <= 0.0] = 0.1

        depth_map = (calibration.focal_length_px * calibration.baseline_meters) / disparity_orig

        # Mask out bad depth
        depth_map[disparity_orig == 0.1] = 0.0 

        h, w = depth_map.shape
        cy, cx = h // 2, w // 2

        # 2. Extract the metric depth at the center
        center_depth = depth_map[cy, cx]
        postprocessing_ms = (self.clock.perf_counter() - postprocessing_started) * 1000.0
        self.last_depth_timings = {
            'preprocessing_ms': preprocessing_ms,
            'inference_ms': inference_ms,
            'postprocessing_ms': postprocessing_ms,
            'total_compute_ms': (self.clock.perf_counter() - started) * 1000.0,
            'center_depth_m': float(center_depth),
        }

        return depth_map

    def timings(self):
        return dict(self.last_depth_timings)
````

### NBV_paper_ros2/src/auv_stereo_depth/setup.py

SHA-256: `4ec135148b45c5efedf09ef8ba280f3f800cd3c65c6782fc27406fd2209a453c`

````python
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = 'auv_stereo_depth'


class RuntimeBuild(build_py):
    """Keep checkout-only research runners outside the inference distribution."""

    def find_package_modules(self, package, package_dir):
        modules = super().find_package_modules(package, package_dir)
        research = {'predict', 'predict_sample', 'prediction_registry'}
        return [module for module in modules
                if not (package == 'auv_stereo_depth.BGNet' and module[1] in research)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', '*.datasets', '*.dataloader']),
    package_data={'auv_stereo_depth.BGNet': ['models/Sceneflow-IRS-BGNet-Plus.pth']},
    cmdclass={'build_py': RuntimeBuild},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'torch', 'torchvision', 'Pillow'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_estimation = auv_stereo_depth.depth_estimation:main'
        ],
    },
)
````

### NBV_paper_ros2/src/auv_planner_reference/package.xml

SHA-256: `9b4b591ade5fa2356b1f8336badbecce3509718710182cdb544e27496a21aa4a`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_planner_reference</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-scipy</exec_depend>
  <exec_depend>python3-open3d</exec_depend>

  <exec_depend>auv_contracts</exec_depend>
  <exec_depend>auv_telemetry</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/adapters/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/adapters/files.py

SHA-256: `75379c1e3a9b056b8340f7698db346f1a100cb2a81dff36aba016f9cfac217ef`

````python
from pathlib import Path

def open_evaluation_log(results_dir):
    return open(Path(results_dir) / 'evaluation_parameters.txt', 'a')
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/adapters/octomap.py

SHA-256: `cc638250ed661af7791246310c778dc7d4ef964a7df9ab0d61daad65266589b4`

````python
import pyoctomap

class OctomapAdapter:
    def __init__(self, resolution):
        self._tree = pyoctomap.ColorOcTree(resolution)
    def extractPointCloud(self):
        return self._tree.extractPointCloud()
    def getNumLeafNodes(self):
        return self._tree.getNumLeafNodes()
    def insertPointCloud(self, *args, **kwargs):
        return self._tree.insertPointCloud(*args, **kwargs)
    def write(self, path):
        return self._tree.write(path)
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/adapters/optimization.py

SHA-256: `b909cd98db98e5212f8b32867587ae62eec6e6a548c913dec0126ffe0bc70287`

````python
from scipy.optimize import minimize

class ScipySolver:
    def minimize(self, objective, initial, **options):
        return minimize(objective, initial, **options)
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/adapters/point_cloud.py

SHA-256: `a40750e34450e9d53bce5e1aeca66aad36fc5b1624010fd132f7bdd06bb6988d`

````python
from dataclasses import dataclass
import numpy as np
import open3d as o3d

@dataclass(frozen=True)
class CloudSnapshot:
    points: object
    normals: object

class Open3DPreprocessor:
    def prepare(self, pipe_points, auv_position, parameters, timed):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pipe_points)
        pcd = timed('nbv_downsample', lambda: pcd.voxel_down_sample(
            voxel_size=float(parameters['voxel_downsample_size'])))
        pcd, _ = timed('nbv_outlier_filter', lambda: pcd.remove_statistical_outlier(
            nb_neighbors=int(parameters['outlier_neighbors']),
            std_ratio=float(parameters['outlier_std_ratio'])))

        if len(pcd.points) < 40:
            return None

        # Normal generation (oriented towards AUV)
        timed('nbv_normal_estimation', lambda: pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(parameters['normal_radius']),
            max_nn=int(parameters['normal_max_neighbors']))))
        timed('nbv_normal_orientation', lambda: pcd.orient_normals_towards_camera_location(auv_position))


        points = np.asarray(pcd.points).copy()
        normals = np.asarray(pcd.normals).copy()
        points.setflags(write=False)
        normals.setflags(write=False)
        return CloudSnapshot(points, normals)
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/adapters/ros_output.py

SHA-256: `507a7a15bf956f958b44085701ee1a4e88c3b8ca0552fd955e2cf0e9cecc5041`

````python
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool

class RosTrajectoryOutput:
    def __init__(self, publisher):
        self.publisher = publisher
    def encode(self, trajectory, camera_offset):
        wire = np.asarray(trajectory, dtype=np.float64).copy()
        wire[:, :3] += camera_offset
        wire = wire.astype(np.float32)
        executed = wire.astype(np.float64)
        executed[:, :3] -= camera_offset
        return wire, executed
    def publish(self, values):
        message = Float32MultiArray()
        message.data = list(values)
        self.publisher.publish(message)

class RosCompletionOutput:
    def __init__(self, publisher):
        self.publisher = publisher
    def publish(self, finished):
        self.publisher.publish(Bool(data=finished))
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/domain/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/domain/configuration.py

SHA-256: `f81c3ce9df1cb21c803386264eda3852cee46912a39c9199bd55b46f4df5b797`

````python
from auv_contracts.values import Configuration
from .nbv_parameters import DEFAULT_NBV_PARAMETERS

def load_configuration(read_parameter):
    values = {}
    values['resolution'] = read_parameter('voxel_resolution', 0.1)
    values['max_depth'] = read_parameter('max_depth', 3.0)
    values['min_depth'] = read_parameter('min_depth', 0.5)
    values['z_min_target'] = read_parameter(
        'z_min_target', -3000.0)
    values['depth_stride'] = read_parameter('depth_stride', 8)
    values['depth_scale'] = read_parameter('depth_scale', 1.0)
    values['frame_period'] = read_parameter('planning_period', 10)
    values['min_utility'] = read_parameter(
        'min_utility_threshold', 0.005)
    values['collision_radius'] = read_parameter(
        'collision_radius', 0.6)
    values['collision_tracking_margin'] = read_parameter(
        'collision_tracking_margin', 0.4)
    values['blacklist_cooldown_s'] = read_parameter(
        'route_blacklist_cooldown_s', 30.0)
    values['trajectory_points'] = read_parameter(
        'trajectory_points', 300)
    values['waypoint_spacing'] = read_parameter(
        'waypoint_spacing', 0.5)
    values['map_path'] = read_parameter(
        'octomap_path', 'results/final_map_ros2.bt')
    values['visit_radius'] = read_parameter('visit_radius', 0.25)
    values['results_dir'] = read_parameter(
        'results_dir', 'results')
    values['nbv_filter_logging'] = read_parameter(
        'nbv_filter_logging', True)
    values['_nbv_empty_reason'] = None
    values['_plan_failure_reason'] = None
    values['use_incremental_map_cache'] = read_parameter(
        'use_incremental_map_cache', True)
    values['nbv_parameters'] = {
        name: read_parameter(
            f'nbv_{name}', default)
        for name, default in DEFAULT_NBV_PARAMETERS.items()
        if name != 'z_min_target'
    }
    # The baseline NBV function filtered at -31.5 independently of mapping.
    values['nbv_parameters']['z_min_target'] = read_parameter(
        'nbv_z_min_target', DEFAULT_NBV_PARAMETERS['z_min_target'])
    values['astar_max_steps'] = read_parameter(
        'astar_max_steps', 50000)
    values['spline_smoothing'] = read_parameter(
        'spline_smoothing', 2.0)
    values['collision_check_spacing_factor'] = read_parameter(
        'collision_check_spacing_factor', 0.5)
    values['arrival_target_radius'] = read_parameter(
        'arrival_target_radius', 1.5)
    values['replan_on_target_arrival'] = read_parameter(
        'replan_on_target_arrival', False)
    values['optimize_motion'] = read_parameter(
        'trajectory_optimization_enabled', False)
    values['velocity_history_window'] = read_parameter(
        'velocity_history_window', 1.0)
    values['trajectory_history_window'] = read_parameter(
        'trajectory_history_window', 5.0)
    values['motion_cost_parameters'] = {
        name: read_parameter(f'trajectory_{name}', default)
        for name, default in {
            'continuity_weight': 8.0, 'reversal_weight': 4.0,
            'history_weight': 2.0, 'history_radius': 0.5,
        }.items()
    }
    values['optimizer_max_deviation'] = read_parameter(
        'trajectory_max_deviation', 0.5)
    values['optimizer_max_iterations'] = read_parameter(
        'trajectory_optimizer_max_iterations', 40)

    return Configuration(values)
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/domain/nbv_parameters.py

SHA-256: `21c26e2ee7134352061c5fca55d8cf6f83ed889cbcd23fe760f900cbfe6c3919`

````python
"""Existing NBV defaults, owned by the framework-free configuration boundary."""
DEFAULT_NBV_PARAMETERS = {
    'z_min_target': -3000.0,
    'pca_neighbors': 20,
    'frontier_neighbors': 40,
    'frontier_angle_threshold_deg': 130.0,
    'cluster_fov_radius': 1.0,
    'cluster_max_angle_deg': 45.0,
    'cluster_min_samples': 5,
    'voxel_downsample_size': 0.25,
    'outlier_neighbors': 15,
    'outlier_std_ratio': 1.5,
    'normal_radius': 0.4,
    'normal_max_neighbors': 30,
    'min_raw_gain': 0.5,
    'standoff': 1.5,
    'polar_angles_deg': [15.0, 30.0],
    'azimuth_steps': 8,
    'visited_min_distance': 0.1,
    'visited_max_distance': 3.0,
    'visited_yaw_threshold_deg': 30.0,
    'complexity_weight': 15.0,
    'distance_weight': 1.0,
    'rotation_weight': 0.5,
    'collision_weight': 2.0,
    'collision_distance_threshold': 0.8,
    'collision_distance_offset': 0.1,
}
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/get_NBV.py

SHA-256: `bf95791456742fcb521b643d3d6925c5c0ac16a6b07c34c17fa2e524cdfce63f`

````python
"""Compatibility/composition entry point for standalone NBV callers."""
from .use_cases.nbv import *
from .use_cases.nbv import calculate_nbv_waypoints as _calculate

def calculate_nbv_waypoints(occupied_points, auv_position, auv_heading,
                           observation_history=None, parameters=None,
                           timing_callback=None):
    import time
    from rclpy.logging import get_logger
    from .adapters.point_cloud import Open3DPreprocessor
    return _calculate(occupied_points, auv_position, auv_heading,
                      observation_history, parameters, timing_callback,
                      preprocessor=Open3DPreprocessor(),
                      diagnostics=get_logger('nbv_waypoint_calculator'), clock=time)
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/planner_node.py

SHA-256: `4625b4e877ee25e8029d9f0b63a1229855605ff8954c75aedabf2193b0a3706b`

````python
import time
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from auv_contracts.values import PoseSample, Vector3, Quaternion, Stamp, Flag
from auv_telemetry.file_sinks import PlannerFileSink
from .domain.configuration import load_configuration
from .use_cases.mission import InspectionMission, body_rotation, vehicle_viewpoint
from .use_cases.nbv import calculate_nbv_waypoints
from .use_cases.trajectory_optimization import optimize_trajectory
from .adapters.point_cloud import Open3DPreprocessor
from .adapters.optimization import ScipySolver
from .adapters.octomap import OctomapAdapter
from .adapters.files import open_evaluation_log
from .adapters.ros_output import RosTrajectoryOutput, RosCompletionOutput

class NBVUtilityNode(Node):
    """ROS input/output adapter; mission decisions belong to InspectionMission."""
    def __init__(self):
        super().__init__('nbv_utility_node')
        config = load_configuration(lambda name, default:
                                    self.declare_parameter(name, default).value)
        self.bridge = CvBridge()
        self.state_callbacks = MutuallyExclusiveCallbackGroup()
        occupancy = OctomapAdapter(config.resolution)
        telemetry = PlannerFileSink(config.results_dir, prefix='planner')
        output = RosTrajectoryOutput(self.create_publisher(
            Float32MultiArray, 'object/areaWaypoints', 10))
        completion = RosCompletionOutput(self.create_publisher(
            Bool, '/movement/finished_execution', 10))
        self.mission = InspectionMission(
            config, occupancy, telemetry, open_evaluation_log(config.results_dir),
            self.get_logger(), time, output, completion,
            partial(calculate_nbv_waypoints, preprocessor=Open3DPreprocessor(),
                    diagnostics=rclpy.logging.get_logger('nbv_waypoint_calculator'), clock=time),
            partial(optimize_trajectory, solver=ScipySolver()), telemetry.description)
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, 'holocean/depth/image_raw', self.depth_callback, qos)
        self.create_subscription(Image, 'holocean/cameraDown/image_raw', self.rgb_callback, qos)
        state_qos = rclpy.qos.QoSProfile(depth=1)
        state_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, 'holocean/odom', self.odom_callback,
                                 state_qos, callback_group=self.state_callbacks)
        self.create_subscription(MagneticField, 'holocean/mag', self.mag_callback,
                                 state_qos, callback_group=self.state_callbacks)
        self.create_subscription(Bool, '/planner/request_nbv', self.request_nbv_callback, 1)
        self.timer = self.create_timer(0.5, self.mission.timer_callback)

    def odom_callback(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.header.stamp
        self.mission.odom_callback(PoseSample(
            Vector3(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w), Stamp(t.sec, t.nanosec)))
    def mag_callback(self, msg):
        v = msg.magnetic_field
        self.mission.mag_callback(Vector3(v.x, v.y, v.z))
    def rgb_callback(self, msg):
        self.mission.rgb_callback(None)
    def request_nbv_callback(self, msg):
        self.mission.request_nbv_callback(Flag(msg.data))
    def depth_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        t = msg.header.stamp
        self.mission.depth_callback(frame, Stamp(t.sec, t.nanosec))
    def destroy_node(self):
        self.mission.close()
        return super().destroy_node()

def main(args=None):
    """Run the Python NBV utility node."""
    rclpy.init(args=args)
    node = NBVUtilityNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/trajectory_optimization.py

SHA-256: `7fe07666344a2554f707e425af7745351187af21e1ddb80ac1be881ac955e88a`

````python
"""Numerical composition entry point; acceptance policy stays in use_cases."""
from .use_cases.trajectory_optimization import estimate_velocity, continuity_cost
from .use_cases.trajectory_optimization import optimize_trajectory as _optimize

def optimize_trajectory(*args, **kwargs):
    from .adapters.optimization import ScipySolver
    return _optimize(*args, solver=ScipySolver(), **kwargs)
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/use_cases/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/use_cases/mission.py

SHA-256: `a956d79c25863051e85c66502cd8b5c0b703ec4f11754aac1c9d7933c3feb344`

````python
"""Mission policy. No ROS, map implementation, model, or filesystem imports."""
import math
import json
from threading import Lock
from collections import deque
import numpy as np
from auv_contracts.ports import (OccupancyMap, MissionTelemetry, TextSink,
    Diagnostics, Clock, TrajectoryOutput, CompletionOutput)
from auv_contracts.diagnostics import json_safe
from ..route_planning import (generate_smooth_trajectory, inflate_occupied,
    path_is_collision_free, plan_multi_waypoint_route,
    resample_trajectory_by_distance, reconnect_trajectory)
from .trajectory_optimization import continuity_cost, estimate_velocity

def body_rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def vehicle_viewpoint(camera_position, target, camera_offset):
    """Place the vehicle so its mounted camera reaches the desired viewpoint."""
    yaw = math.atan2(target[1] - camera_position[1], target[0] - camera_position[0])
    for _ in range(12):
        vehicle = camera_position - body_rotation(0., 0., yaw) @ camera_offset
        next_yaw = math.atan2(target[1] - vehicle[1], target[0] - vehicle[0])
        if abs(math.atan2(math.sin(next_yaw - yaw), math.cos(next_yaw - yaw))) < 1e-8:
            return vehicle
        yaw = next_yaw
    return camera_position - body_rotation(0., 0., yaw) @ camera_offset


class InspectionMission:
    def __init__(self, config, occupancy_map: OccupancyMap,
                 telemetry: MissionTelemetry, evaluation: TextSink,
                 diagnostics: Diagnostics, clock: Clock,
                 output: TrajectoryOutput, completion: CompletionOutput,
                 nbv_calculator, optimizer, telemetry_description):
        self.diagnostics = diagnostics
        self.clock = clock
        self.output = output
        self.completion = completion
        self.nbv_calculator = nbv_calculator
        self.optimizer = optimizer
        self.evaluation_file = evaluation
        self._nbv_empty_reason = config._nbv_empty_reason
        self._plan_failure_reason = config._plan_failure_reason
        self.arrival_target_radius = config.arrival_target_radius
        self.astar_max_steps = config.astar_max_steps
        self.blacklist_cooldown_s = config.blacklist_cooldown_s
        self.collision_check_spacing_factor = config.collision_check_spacing_factor
        self.collision_radius = config.collision_radius
        self.collision_tracking_margin = config.collision_tracking_margin
        self.depth_scale = config.depth_scale
        self.depth_stride = config.depth_stride
        self.frame_period = config.frame_period
        self.map_path = config.map_path
        self.max_depth = config.max_depth
        self.min_depth = config.min_depth
        self.min_utility = config.min_utility
        self.motion_cost_parameters = config.motion_cost_parameters
        self.nbv_filter_logging = config.nbv_filter_logging
        self.nbv_parameters = config.nbv_parameters
        self.optimize_motion = config.optimize_motion
        self.optimizer_max_deviation = config.optimizer_max_deviation
        self.optimizer_max_iterations = config.optimizer_max_iterations
        self.replan_on_target_arrival = config.replan_on_target_arrival
        self.resolution = config.resolution
        self.results_dir = config.results_dir
        self.spline_smoothing = config.spline_smoothing
        self.trajectory_history_window = config.trajectory_history_window
        self.trajectory_points = config.trajectory_points
        self.use_incremental_map_cache = config.use_incremental_map_cache
        self.velocity_history_window = config.velocity_history_window
        self.visit_radius = config.visit_radius
        self.waypoint_spacing = config.waypoint_spacing
        self.z_min_target = config.z_min_target
        self.state_lock = Lock()
        self.position = np.zeros(3, dtype=np.float64)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # --- LATCHED VARIABLES ---
        self.latched_position = np.zeros(3, dtype=np.float64)
        self.latched_yaw = 0.0
        self.latched_pitch = 0.0
        self.latched_roll = 0.0
        self.odom_history = deque(maxlen=3000)

        self.odom_received = False
        self.rgb_received = False
        self.depth_frame = None
        self.depth_sequence = 0
        self.processed_depth_sequence = 0
        self.iteration = 0
        self.started_mapping = False
        self.finished_mapping = False
        self.octree = occupancy_map
        self.active_trajectory = np.empty((0, 4), dtype=np.float64)
        self.camera_offset = np.array([0.35, 0.1125 + 0.275, -0.5])
        self.force_replan = False
        self.visited_targets = []
        self.selected_targets = []
        self.occupied_voxels = set()
        self.pending_occupied_voxels = set()
        self.collision_voxels = set()
        self.telemetry = telemetry
        self.failed_plan_count = 0
        self.observation_history = []
        self.blacklisted_targets = []
        self.blacklist_expiries = {}

        self.diagnostics.info(
            f'NBV utility node started: active NBV filter is Z > '
            f'{self.nbv_parameters["z_min_target"]}; '
            f'z_min_target={self.z_min_target} is not used by this filter')
        if self.nbv_filter_logging:
            self.diagnostics.info(f'NBV filter diagnostics: {telemetry_description}')


    def request_nbv_callback(self, msg):
        """Triggered instantly when C++ PathPlanner runs out of waypoints."""
        if msg.data:
            self.diagnostics.info('Received manual NBV request from PathPlanner. Forcing immediate replan!')
            self.force_replan = True


    def _motion_snapshot(self):
        """Copy a consistent state without locking the expensive planning work."""
        with self.state_lock:
            return self.position.copy(), self.yaw, list(self.odom_history)


    def odom_callback(self, msg):
        """Keep odometry updating while the default group runs planning."""
        with self.state_lock:
            self._update_odometry(msg)


    def _update_odometry(self, msg):
        """Update the vehicle pose with state_lock held."""
        position = msg.position
        if abs(position.x) < 0.01 and abs(position.y) < 0.01:
            self._log_nbv_filter('odometry_origin_guard', 1, 0,
                                 {'position': [position.x, position.y, position.z],
                                  'xy_threshold': 0.01})
            return

        self.position = np.array(
            [position.x, position.y, position.z], dtype=np.float64)
        orientation = msg.orientation
        sin_roll = 2.0 * (orientation.w * orientation.x
                          + orientation.y * orientation.z)
        cos_roll = 1.0 - 2.0 * (orientation.x ** 2 + orientation.y ** 2)
        self.roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (orientation.w * orientation.y
                           - orientation.z * orientation.x)
        self.pitch = math.asin(np.clip(sin_pitch, -1.0, 1.0))
        # sin_yaw = 2.0 * (orientation.w * orientation.z
        #                  + orientation.x * orientation.y)
        # cos_yaw = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        # self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.odom_received = True

        # NEW: Save this exact state with its timestamp into the history buffer
        msg_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if self.odom_history and msg_time < self.odom_history[-1][0]:
            self.odom_history.clear()  # Simulator clock reset: discard old motion.
        if self.odom_history and msg_time == self.odom_history[-1][0]:
            return
        self.odom_history.append((
            msg_time, 
            np.copy(self.position), 
            self.roll, 
            self.pitch, 
            self.yaw
        ))


    def mag_callback(self, msg):
        """Use the magnetometer heading when it is available."""
        with self.state_lock:
            self.yaw = math.atan2(
                msg.x, msg.y) - math.pi / 2.0
            if self.yaw < -math.pi:
                self.yaw += 2.0 * math.pi


    def rgb_callback(self, msg):
        """Record that the camera stream is available."""
        del msg
        self.rgb_received = True


    def depth_callback(self, frame, stamp):
        """Find the historical odometry that exactly matches this depth frame."""
        self.depth_frame = frame
        self.depth_sequence += 1

        _, _, history = self._motion_snapshot()
        if len(history) == 0:
            return

        # Get the time the original image was taken (passed through by obstacleAvoidance)
        depth_time = stamp.sec + stamp.nanosec * 1e-9

        # Search the history buffer for the odometry closest to the depth timestamp
        closest_odom = min(history, key=lambda x: abs(x[0] - depth_time))

        # If the closest match is within 0.1 seconds, use it! 
        if abs(closest_odom[0] - depth_time) < 0.1:
            self.latched_position = np.copy(closest_odom[1])
            self.latched_roll = closest_odom[2]
            self.latched_pitch = closest_odom[3]
            self.latched_yaw = closest_odom[4]
            self.telemetry.record_depth_pose(
                self.depth_sequence,
                depth_time,
                self.latched_position,
                self.latched_roll,
                self.latched_pitch,
                self.latched_yaw)
        else:
            self.diagnostics.warning("Depth map timestamp is too far! Closest odometry is %.3f seconds away. Using the most recent odometry instead." % (abs(closest_odom[0] - depth_time)))
            latest = history[-1]
            self.latched_position = latest[1].copy()
            self.latched_roll, self.latched_pitch, self.latched_yaw = latest[2:]


    def _depth_points(self, depth_frame):
        """Project a depth image into the vehicle world frame."""
        depth = np.asarray(depth_frame)
        if depth.ndim != 2:
            self._log_nbv_filter('depth_image_shape', int(depth.size), 0,
                                 {'shape': list(depth.shape), 'required_dimensions': 2})
            return np.empty((0, 3), dtype=np.float64)

        scale = 0.001 if depth.dtype == np.uint16 else self.depth_scale
        depth = depth.astype(np.float64) * scale
        height, width = depth.shape
        stride = max(1, int(self.depth_stride))
        rows, cols = np.mgrid[0:height:stride, 0:width:stride]
        distances = depth[rows, cols]
        valid = np.isfinite(distances)
        valid &= (distances > self.min_depth)
        valid &= (distances < self.max_depth)
        finite_distances = distances[np.isfinite(distances)]
        self._log_nbv_filter('depth_range', int(distances.size), int(np.count_nonzero(valid)),
                             {'min_depth': self.min_depth, 'max_depth': self.max_depth,
                              'comparison': 'min_depth < distance < max_depth',
                              'nonfinite': int(np.count_nonzero(~np.isfinite(distances))),
                              'at_or_below_min': int(np.count_nonzero(
                                  np.isfinite(distances) & (distances <= self.min_depth))),
                              'at_or_above_max': int(np.count_nonzero(
                                  np.isfinite(distances) & (distances >= self.max_depth))),
                              'distance_min': float(finite_distances.min()) if finite_distances.size else None,
                              'distance_max': float(finite_distances.max()) if finite_distances.size else None,
                              'depth_scale': scale, 'stride': stride,
                              'horizontal_fov_deg': 90.0, 'camera_pitch_deg': 45.0,
                              'camera_offset': self.camera_offset})
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float64)

        rows = rows[valid].astype(np.float64)
        cols = cols[valid].astype(np.float64)
        distances = distances[valid]
        focal_length = (width / 2.0) / math.tan(math.radians(90.0) / 2.0)
        center_x, center_y = width / 2.0, height / 2.0
        camera_x = (cols - center_x) * distances / focal_length
        camera_y = (rows - center_y) * distances / focal_length

        pitch = math.radians(45.0)
        local_x = distances * math.cos(pitch) - camera_y * math.sin(pitch)
        local_y = -camera_x
        local_z = -distances * math.sin(pitch) - camera_y * math.cos(pitch)
        local_points = np.column_stack((local_x, local_y, local_z))

        rotation = body_rotation(self.latched_roll, self.latched_pitch, self.latched_yaw)
        self._depth_sensor_origin = self.latched_position + rotation @ self.camera_offset
        return local_points @ rotation.T + self._depth_sensor_origin


    def _accumulate_depth(self):
        points = self._depth_points(self.depth_frame)
        if len(points) == 0:
            # Warn if the camera only sees empty water outside max_depth
            self.diagnostics.warning('Depth projection yielded 0 valid points (all out of range or invalid).', throttle_duration_sec=2.0)
            return

        before_point_cap = len(points)
        if len(points) > 4000:
            # Randomly sample exactly 4000 points to preserve the pipe's overall shape
            indices = np.random.choice(len(points), 4000, replace=False)
            points = points[indices]
        self._log_nbv_filter('depth_point_cap', before_point_cap,
                             len(points), {'limit': 4000})

        if self.use_incremental_map_cache:
            voxel_indices = np.floor(
                points / self.resolution).astype(np.int32)
            new_voxels = set(map(tuple, voxel_indices.tolist()))
            new_voxels.difference_update(self.occupied_voxels)
            self.occupied_voxels.update(new_voxels)
            self.pending_occupied_voxels.update(new_voxels)

        sensor_origin = self._depth_sensor_origin
        self.octree.insertPointCloud(
            points.astype(np.float64),
            sensor_origin=sensor_origin.astype(np.float64),
            max_range=float(self.max_depth),
        )
        self.diagnostics.info(f'Inserted {len(points)} points into OctoTree.', throttle_duration_sec=1.0)

        save_obs = True
        if self.observation_history:
            last_pos, last_yaw = self.observation_history[-1]
            dist = np.linalg.norm(self.latched_position - last_pos)

            # Normalize yaw difference
            yaw_diff = abs(math.atan2(
                math.sin(self.latched_yaw - last_yaw), 
                math.cos(self.latched_yaw - last_yaw)
            ))

            # Only save if we moved 0.5m or turned 15 degrees to save memory
            if dist < 0.5 and yaw_diff < math.radians(15):
                save_obs = False

        if save_obs:
            self.observation_history.append((np.copy(self.latched_position), self.latched_yaw))


    def _occupied_points(self):
        """Extract occupied metric points from the persistent OctoMap."""
        if self.use_incremental_map_cache:
            if not self.occupied_voxels:
                return np.empty((0, 3), dtype=np.float64)
            return (
                (np.asarray(tuple(self.occupied_voxels), dtype=np.float64) + 0.5)
                * self.resolution)

        extracted = self.octree.extractPointCloud()
        points = np.asarray(extracted[0], dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return points.reshape((-1, 3))


    def _publish_trajectory(self, trajectory, occupied):
        """Validate the vehicle coordinates reconstructed from the wire message."""
        if len(trajectory) == 0:
            return False

        wire, executed = self.output.encode(trajectory, self.camera_offset)
        if not path_is_collision_free(executed[:, :3], occupied, self.resolution):
            self._log_nbv_filter('vehicle_wire_validation', len(trajectory), 0,
                                 {'reason': 'executed_vehicle_route_collision'})
            return False
        self.output.publish(wire.flatten().tolist())
        self.active_trajectory = executed
        self.started_mapping = True
        self.diagnostics.info(
            f'Published {len(self.active_trajectory)} trajectory points')
        return True


    def _plan_and_publish(self, candidates, occupied):
        """Run A*, smooth its route, and publish only a safe trajectory."""
        self._plan_failure_reason = None
        self._failed_route_target = None
        valid_candidates = [
            candidate for candidate in candidates
            if candidate['utility'] >= self.min_utility
        ]
        for candidate in candidates:
            self._log_nbv_filter('candidate_minimum_utility', 1,
                                 int(candidate['utility'] >= self.min_utility),
                                 {'cluster_id': candidate.get('cluster_id'),
                                  'target': candidate['target'],
                                  'position': candidate['position'],
                                  'utility': candidate['utility'],
                                  'threshold': self.min_utility})
        self._log_nbv_filter('minimum_utility', len(candidates), len(valid_candidates),
                             {'threshold': self.min_utility})
        if not valid_candidates:
            self._plan_failure_reason = ('minimum_utility_removed_all' if candidates
                                         else 'no_candidates_to_plan')
            return False
        valid_candidates.sort(key=lambda candidate: candidate['utility'], reverse=True)

        route_candidates = [
            (vehicle_viewpoint(np.asarray(candidate['position'], dtype=np.float64),
                               np.asarray(candidate['target'], dtype=np.float64), self.camera_offset),
             np.asarray(candidate['target'], dtype=np.float64))
            for candidate in valid_candidates
        ]
        route_started = self.clock.perf_counter()
        position, yaw, history = self._motion_snapshot()
        velocity = estimate_velocity(
            history, float(self.velocity_history_window))
        latest_time = history[-1][0] if history else 0.0
        recent_positions = [sample[1]
                            for sample in history
                            if 0.0 <= latest_time - sample[0]
                            <= float(self.trajectory_history_window)]
        if len(recent_positions) > 64:
            indices = np.linspace(0, len(recent_positions) - 1, 64).astype(int)
            recent_positions = [recent_positions[index] for index in indices]
        route, targets = plan_multi_waypoint_route(
            position,
            route_candidates,
            occupied,
            self.resolution,
            astar_max_steps=int(self.astar_max_steps),
            current_velocity=velocity if self.optimize_motion else None,
            reversal_weight=(float(self.motion_cost_parameters['reversal_weight'])
                             if self.optimize_motion else 0.0),
        )
        self.telemetry.record_stage(
            'route_search', (self.clock.perf_counter() - route_started) * 1000.0,
            iteration=self.iteration)
        route_safe = len(route) > 0 and path_is_collision_free(
            route,
            occupied,
            self.resolution,
            self.collision_check_spacing_factor)
        self._log_nbv_filter('route_validation', len(route), len(route) if route_safe else 0,
                             {'route_points': len(route), 'collision_free': route_safe,
                              'collision_voxels': len(occupied),
                              'resolution': self.resolution,
                              'spacing_factor': self.collision_check_spacing_factor})
        if not route_safe:
            self._plan_failure_reason = 'astar_removed_all' if len(route) == 0 else 'route_collision'
            self._failed_route_target = np.copy(valid_candidates[0]['target'])
            self.diagnostics.warning('A* produced no collision-free route')
            return False

        trajectory_started = self.clock.perf_counter()
        route_options = [('spline', route, targets)]
        target_changes = np.flatnonzero(np.any(targets != targets[0], axis=1))
        if len(target_changes) and target_changes[0] >= 4:
            first_end = int(target_changes[0])
            route_options.append(('first_target_spline', route[:first_end], targets[:first_end]))
        safe_options = []
        # A voxel polyline is not a dynamically validated trajectory for this controller.
        for option_name, option_route, option_targets in route_options:
            option = generate_smooth_trajectory(
                option_route, option_targets,
                num_points=int(self.trajectory_points),
                smoothing=float(self.spline_smoothing),
                sampling_spacing=float(self.waypoint_spacing))
            if len(option) == 0:
                self._log_nbv_filter('trajectory_option', 0, 0,
                                     {'option': option_name, 'reason': 'empty_trajectory',
                                      'route_points': len(option_route), 'spline_minimum_points': 4})
                continue
            option = option.copy()
            option[0, :3] = option_route[0]
            option[-1, :3] = option_route[-1]
            if self.optimize_motion:
                option[:, 3] = np.unwrap(np.r_[yaw, option[:, 3]])[1:]
                option[0, 3] = yaw
            option = resample_trajectory_by_distance(
                option, float(self.waypoint_spacing),
                max_points=int(self.trajectory_points))
            option_safe = path_is_collision_free(
                    option[:, :3], occupied, self.resolution,
                    self.collision_check_spacing_factor)
            self._log_nbv_filter('trajectory_option', len(option),
                                 len(option) if option_safe else 0,
                                 {'option': option_name, 'collision_free': option_safe,
                                  'waypoint_spacing': self.waypoint_spacing,
                                  'requested_max_points': int(self.trajectory_points),
                                  'mandatory_corners_exceed_cap': len(option) > int(self.trajectory_points)})
            if option_safe:
                safe_options.append((option, option_targets))
                if not self.optimize_motion:
                    break  # Baseline mode needs only the first validated route.
        if not safe_options:
            self._plan_failure_reason = 'all_trajectory_options_rejected'
            # This is the first target in travel order, which can differ from
            # the highest-utility candidate. Its prefix spline also failed.
            self._failed_route_target = np.copy(targets[0])
            self.diagnostics.warning(
                'Resampled spline intersects the collision boundary; '
                'route rejected')
            self.telemetry.record_stage(
                'trajectory_generation_and_validation',
                (self.clock.perf_counter() - trajectory_started) * 1000.0,
                iteration=self.iteration, details='rejected')
            return False
        trajectory, execution_targets = min(safe_options, key=lambda option: continuity_cost(
            option[0], velocity, recent_positions, **self.motion_cost_parameters)
        ) if self.optimize_motion else safe_options[0]
        if self.optimize_motion:
            before_cost = continuity_cost(
                trajectory, velocity, recent_positions, **self.motion_cost_parameters)
            trajectory = self.optimizer(
                trajectory, position, yaw, velocity,
                occupied, self.resolution, recent_positions,
                **self.motion_cost_parameters,
                max_deviation=float(self.optimizer_max_deviation),
                max_iterations=int(self.optimizer_max_iterations),
                collision_spacing_factor=float(self.collision_check_spacing_factor))
            after_cost = continuity_cost(
                trajectory, velocity, recent_positions, **self.motion_cost_parameters)
            self.diagnostics.info(
                f'Motion optimization: speed={np.linalg.norm(velocity):.2f} m/s, '
                f'continuity cost={before_cost:.2f} -> {after_cost:.2f}')
        self.telemetry.record_stage(
            'trajectory_generation_and_validation',
            (self.clock.perf_counter() - trajectory_started) * 1000.0,
            iteration=self.iteration)

        publish_started = self.clock.perf_counter()
        latest_position, latest_yaw, _ = self._motion_snapshot()
        before_reconnect = len(trajectory)
        trajectory = reconnect_trajectory(
            trajectory, latest_position, latest_yaw,
            occupied, self.resolution, self.collision_check_spacing_factor)
        self._log_nbv_filter('trajectory_reconnection', before_reconnect, len(trajectory),
                             {'latest_vehicle_position': latest_position,
                              'latest_yaw': latest_yaw})
        if len(trajectory) == 0:
            self._plan_failure_reason = 'trajectory_reconnection_rejected'
            self.force_replan = True
            self.diagnostics.warning(
                'Vehicle moved during planning; no safe entry to the new route. '
                'Requesting a fresh plan.')
        success = self._publish_trajectory(trajectory, occupied)
        self.telemetry.record_stage(
            'trajectory_publish', (self.clock.perf_counter() - publish_started) * 1000.0,
            iteration=self.iteration,
            details='published' if success else 'rejected')
        if success:
            self.selected_targets = [np.copy(target) for target in execution_targets]
            # DELETED: We no longer add targets to memory here!
        elif self._plan_failure_reason is None:
            self._plan_failure_reason = 'trajectory_publish_rejected'
            self._failed_route_target = np.copy(execution_targets[0])
        return success


    def _expire_target_blacklist(self):
        now = self.clock.perf_counter()
        retained = []
        for target in self.blacklisted_targets:
            expiry = self.blacklist_expiries.get(tuple(target), 0.0)
            if expiry > now:
                retained.append(target)
            else:
                self.blacklist_expiries.pop(tuple(target), None)
                self._log_nbv_filter('blacklist_expired', 1, 1, {'target': target})
        self.blacklisted_targets = retained


    def _blacklist_failed_target(self):
        """Suppress the failed target briefly so another structure can be tried."""
        if self._failed_route_target is None or self.blacklist_cooldown_s <= 0.0:
            return False
        target = np.asarray(self._failed_route_target, dtype=float).copy()
        existing = next((bt for bt in self.blacklisted_targets
                         if np.linalg.norm(target - bt) < 0.5), None)
        if existing is None:
            self.blacklisted_targets.append(target)
        else:
            target = existing
        self.blacklist_expiries[tuple(target)] = self.clock.perf_counter() + self.blacklist_cooldown_s
        self._log_nbv_filter('blacklist_addition', 1, 0,
                             {'target': target, 'planning_failure': self._plan_failure_reason,
                              'radius': 0.5, 'expires': True,
                              'cooldown_s': self.blacklist_cooldown_s})
        self.diagnostics.warning(
            f'Planning rejected ({self._plan_failure_reason}); '
            f'blacklisting failed target {target.tolist()} for {self.blacklist_cooldown_s:g}s.')
        self.force_replan = True
        return True


    def _log_nbv_filter(self, stage, input_count, output_count, details):
        """Log every filter transition to ROS and a per-frame JSONL event stream."""
        if not self.nbv_filter_logging:
            return
        if stage == 'nbv_empty':
            self._nbv_empty_reason = details.get('reason')
        safe_details = json_safe(details)
        self.diagnostics.info(
            f'[NBV_FILTER iteration={self.iteration} frame={self.depth_sequence}] '
            f'{stage}: {input_count} -> {output_count} '
            f'{json.dumps(safe_details, allow_nan=False)}')
        self.telemetry.record_filter(stage, input_count, output_count, safe_details,
                                     iteration=self.iteration, frame_id=self.depth_sequence)


    def _active_trajectory_is_safe(self, occupied):
        """Check the currently published route against the updated map."""
        if len(self.active_trajectory) == 0:
            return True
        return path_is_collision_free(
            self.active_trajectory[:, :3],
            occupied,
            self.resolution,
            self.collision_check_spacing_factor)


    def _cancel_active_trajectory(self):
        """Cancel the downstream queue as well as the planner's cached route."""
        previous_count = len(self.active_trajectory)
        self.output.publish(())
        self.active_trajectory = np.empty((0, 4), dtype=np.float64)
        self.selected_targets = []
        self._log_nbv_filter('trajectory_cancelled', previous_count, 0,
                             {'reason': 'new_obstacle_on_active_route',
                              'downstream_queue_cleared': True})


    def timer_callback(self):
        """Update the map and periodically publish the next viewpoints."""
        if self.finished_mapping:
            return

        # 1. DIAGNOSTIC LOGS: Find out exactly what is blocking the pipeline
        if not self.odom_received:
            self.diagnostics.warning('Pipeline blocked: Waiting for Odometry...', throttle_duration_sec=2.0)
            return
        if not self.rgb_received:
            self.diagnostics.warning('Pipeline blocked: Waiting for RGB camera...', throttle_duration_sec=2.0)
            return
        if self.depth_frame is None:
            self.diagnostics.warning('Pipeline blocked: Waiting for Depth frame...', throttle_duration_sec=2.0)
            return
        if self.depth_sequence == self.processed_depth_sequence:
            self.diagnostics.warning('Pipeline blocked: Waiting for NEW Depth frame (Sequence unchanged)...', throttle_duration_sec=2.0)
            return

        # 2. ACCUMULATION PHASE
        started = self.clock.perf_counter()
        stage_started = self.clock.perf_counter()
        self._accumulate_depth()
        self.telemetry.record_stage(
            'depth_to_octomap', (self.clock.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)
        self.processed_depth_sequence = self.depth_sequence

        stage_started = self.clock.perf_counter()
        occupied_points = self._occupied_points()
        self.telemetry.record_stage(
            'octomap_extract', (self.clock.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)
        self.diagnostics.info(f'Iteration {self.iteration}: OctoMap total size = {len(occupied_points)} voxels.', throttle_duration_sec=2.0)

        stage_started = self.clock.perf_counter()
        if self.use_incremental_map_cache:
            new_voxels = self.pending_occupied_voxels
            self.pending_occupied_voxels = set()
            if new_voxels:
                self.collision_voxels.update(
                    inflate_occupied(
                        new_voxels,
                        self.collision_radius + self.collision_tracking_margin,
                        self.resolution))
            collision_voxels = self.collision_voxels
        else:
            occupied_indices = set(map(
                tuple, np.floor(occupied_points / self.resolution).astype(int)))
            collision_voxels = inflate_occupied(
                occupied_indices, self.collision_radius + self.collision_tracking_margin, self.resolution)
        self.telemetry.record_stage(
            'collision_inflation', (self.clock.perf_counter() - stage_started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)

        route_blocked = not self._active_trajectory_is_safe(collision_voxels)
        if route_blocked:
            self._cancel_active_trajectory()
            self.diagnostics.warning('🚨 Active trajectory collided with newly mapped space; forcing replan!')


        arrived_at_target = False
        if self.replan_on_target_arrival and self.selected_targets:
            for target in self.selected_targets:
                # Calculate absolute distance to the NBV centroid
                dist = np.linalg.norm(self.position - target)

                # Use a 1.5m radius because B-splines smooth out corners, 
                # so the AUV might not hit the exact dead-center millimeter.
                if dist < self.arrival_target_radius:
                    self.diagnostics.info(f'📍 Arrived at NBV Target (Distance: {dist:.2f}m). Forcing instant replan!')
                    arrived_at_target = True
                    break

        # 3. PLANNING PHASE
        should_plan = (
            self.iteration % max(1, int(self.frame_period)) == 0
            or route_blocked
            or self.force_replan
            or arrived_at_target
        )

        if should_plan:
            self._nbv_empty_reason = None
            self._log_nbv_filter('planning_begin', len(occupied_points), len(occupied_points),
                                 {'periodic': self.iteration % max(1, int(self.frame_period)) == 0,
                                  'route_blocked': bool(route_blocked),
                                  'forced': bool(self.force_replan),
                                  'arrived_at_target': bool(arrived_at_target),
                                  'collision_voxels': len(collision_voxels),
                                  'blacklisted_targets': self.blacklisted_targets,
                                  'minimum_utility': self.min_utility})
            self.force_replan = False # Reset the flag
            self.diagnostics.info(f'--- PLANNING TRIGGERED (Iteration {self.iteration}) ---')
            self.diagnostics.info(f'Saving OctoMap to: {self.map_path}')
            stage_started = self.clock.perf_counter()
            self.octree.write(self.map_path)
            self.telemetry.record_stage(
                'octomap_save', (self.clock.perf_counter() - stage_started) * 1000.0,
                iteration=self.iteration)

            nbv_position, nbv_yaw, nbv_history = self._motion_snapshot()
            roll, pitch = nbv_history[-1][2:4] if nbv_history else (0., 0.)
            camera_position = nbv_position + body_rotation(roll, pitch, nbv_yaw) @ self.camera_offset
            heading = np.array([math.cos(nbv_yaw), math.sin(nbv_yaw), 0.0])

            nbv_started = self.clock.perf_counter()
            try:
                self.diagnostics.info('Calculating NBV waypoints...')
                candidates = self.nbv_calculator(
                    occupied_points,
                    camera_position,
                    heading,
                    self.observation_history,
                    self.nbv_parameters,
                    timing_callback=lambda stage, duration_ms:
                        self.telemetry.record_stage(
                            stage, duration_ms, iteration=self.iteration),
                )
                self.diagnostics.info(f'Found {len(candidates)} valid NBV targets.')
            except (ValueError, RuntimeError) as error:
                self.diagnostics.error(f'NBV calculation failed: {error}')
                self._nbv_empty_reason = 'nbv_exception'
                self._log_nbv_filter('nbv_exception', len(occupied_points), 0,
                                     {'exception_type': type(error).__name__, 'error': str(error)})
                candidates = []
            self.telemetry.record_stage(
                'nbv_generation', (self.clock.perf_counter() - nbv_started) * 1000.0,
                iteration=self.iteration,
                details=f'candidates={len(candidates)}')

            self._expire_target_blacklist()
            safe_candidates = []
            for cand in candidates:
                # If this target is within 0.5m of a known bad coordinate, drop it
                is_banned = any(np.linalg.norm(cand['target'] - bt) < 0.5 for bt in self.blacklisted_targets)
                self._log_nbv_filter('candidate_blacklist', 1, int(not is_banned),
                                     {'cluster_id': cand.get('cluster_id'), 'target': cand['target'],
                                      'radius': 0.5,
                                      'matching_bans': [
                                          {'target': bt, 'distance': float(np.linalg.norm(cand['target'] - bt)),
                                           'remaining_s': max(0.0, self.blacklist_expiries[tuple(bt)] - self.clock.perf_counter())}
                                          for bt in self.blacklisted_targets
                                          if np.linalg.norm(cand['target'] - bt) < 0.5]})
                if not is_banned:
                    safe_candidates.append(cand)

            if len(safe_candidates) < len(candidates):
                self.diagnostics.info(f"Dropped {len(candidates) - len(safe_candidates)} blacklisted targets.")
            self._log_nbv_filter('blacklist', len(candidates), len(safe_candidates),
                                 {'radius': 0.5, 'blacklisted_target_count': len(self.blacklisted_targets)})

            # Pass the SAFE candidates to the planner
            published = self._plan_and_publish(safe_candidates, collision_voxels)
            failure_reason = ((self._nbv_empty_reason or 'nbv_generation_returned_no_candidates')
                              if not candidates else
                              'blacklist_removed_all' if not safe_candidates else
                              self._plan_failure_reason)
            self._log_nbv_filter('planning_result', len(candidates), int(published),
                                 {'published': bool(published),
                                  'reason': 'published' if published else failure_reason,
                                  'generated_candidates': len(candidates),
                                  'after_blacklist': len(safe_candidates),
                                  'active_trajectory_points': len(self.active_trajectory)})
            if not published:
                self.diagnostics.warning(f'[NBV_EMPTY] No trajectory published: {failure_reason}')


            if published:
                self.failed_plan_count = 0  # Reset counter on success
            else:
                if self.force_replan:
                    pass  # Vehicle simply moved during planning; not a failure of the candidates.
                elif len(safe_candidates) > 0:
                    # We had candidates, but failed to publish. 
                    # Did we identify a specific bad target to blacklist?
                    if not self._blacklist_failed_target():
                        self._log_nbv_filter('blacklist_skipped', len(safe_candidates), len(safe_candidates),
                                             {'reason': 'no_individual_failed_target',
                                              'planning_failure': self._plan_failure_reason})

                        # CRITICAL FIX: If we didn't blacklist anything but still failed 
                        # (e.g., minimum_utility_removed_all), the entire batch is unusable. Strike out!
                        self.failed_plan_count += 1
                        self.diagnostics.warning(
                            f'All safe candidates failed ({self._plan_failure_reason}). Strike {self.failed_plan_count}/3'
                        )
                else:
                    # CRITICAL FIX: Increment if NO safe candidates exist (all banned or none found).
                    self.failed_plan_count += 1
                    self.diagnostics.warning(
                        f'No valid candidates remain ({failure_reason}). Strike {self.failed_plan_count}/3'
                    )

            if self.failed_plan_count >= 3:
                self._log_nbv_filter('mission_completion', self.failed_plan_count, 0,
                                     {'reason': 'three_empty_candidate_passes',
                                      'last_empty_reason': failure_reason,
                                      'blacklisted_target_count': len(self.blacklisted_targets)})
                self.finished_mapping = True
                self.completion.publish(True)

                self.diagnostics.info('Flushing final RAM state to OctoMap file...')
                self.octree.write(self.map_path)
                self.diagnostics.info('✅ Mapping finished. Explored all accessible frontiers.')

            # CRITICAL FIX: Only end the mission if there are literally zero frontiers left.
            # if len(candidates) == 0 and self.started_mapping:
            #     self.finished_mapping = True
            #     self.completion.publish(True)

            #     # FORCE FINAL SAVE TO DISK BEFORE EXITING
            #     self.diagnostics.info('Flushing final RAM state to OctoMap file...')
            #     self.octree.write(self.map_path)

            #     self.diagnostics.info('✅ Mapping finished. Explored all accessible frontiers.')
            # elif not published and len(candidates) > 0:
            #     self.diagnostics.warning('Failed to generate a collision-free spline to the targets. Retrying next cycle...')

            elapsed_ms = (self.clock.perf_counter() - started) * 1000.0
            self.diagnostics.info(f'Planning cycle completed in {elapsed_ms:.2f} ms.')
            self.telemetry.record_iteration(
                self.iteration,
                elapsed_ms,
                self.position,
                candidates,
                self.active_trajectory,
                published,
                route_blocked,
                self.selected_targets)

            self.evaluation_file.write(
                f'Process Time (ms): {elapsed_ms}, '
                f'Total Leaves: {self.octree.getNumLeafNodes()}\n')
            self.evaluation_file.flush()
        self.telemetry.record_stage(
            'planning_cycle_total', (self.clock.perf_counter() - started) * 1000.0,
            iteration=self.iteration, frame_id=self.depth_sequence)

        self.iteration += 1


    def close(self):
        """Close the evaluation log and save the final map before destroying the ROS node."""
        self.diagnostics.info('Node shutting down. Saving final OctoMap state...')
        try:
            self.octree.write(self.map_path)
        except Exception as e:
            self.diagnostics.error(f'Failed to save final map on exit: {e}')

        self.evaluation_file.close()

````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/use_cases/nbv.py

SHA-256: `dfabfd498e9dce33716550d638cdd6b47e7a00e72fcab42b6fa5ae3b2cef52e5`

````python
import numpy as np
from scipy.spatial import KDTree




from ..domain.nbv_parameters import DEFAULT_NBV_PARAMETERS

# ==========================================
# NBV MATHEMATICAL HELPER FUNCTIONS
# ==========================================

def calculate_pca_complexity(pcd, k=20):
    points = np.asarray(pcd.points)
    n_points = len(points)

    if n_points < 3:
        return np.zeros(n_points)

    tree = KDTree(points)
    complexities = np.zeros(n_points)
    k_actual = min(int(k), n_points)
    if k_actual < 2:
        return complexities
    # Batch queries and eigensolves; cap temporary storage for large maps.
    for start in range(0, n_points, 1024):
        _, idxs = tree.query(points[start:start + 1024], k=k_actual)
        neighbors = points[idxs]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered) / (k_actual - 1)
        eigenvalues = np.linalg.eigvalsh(covariance)
        totals = eigenvalues.sum(axis=1)
        complexities[start:start + len(idxs)] = np.divide(
            eigenvalues[:, 0], totals, out=np.zeros_like(totals), where=totals > 0)
    return complexities

def detect_frontiers(pcd, k=40, angle_threshold_deg=130):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals, dtype=np.float64)
    tree = KDTree(points)

    frontier_indices = []
    angle_threshold = np.radians(angle_threshold_deg)
    if len(points) < 2:
        return frontier_indices
    k_actual = min(max(2, int(k)), len(points))
    for start in range(0, len(points), 1024):
        batch = points[start:start + 1024]
        batch_normals = normals[start:start + 1024]
        _, neighbor_idxs = tree.query(batch, k=k_actual)
        vectors = points[neighbor_idxs] - batch[:, None, :]
        u = np.cross(batch_normals, [0, 0, 1])
        vertical = np.linalg.norm(u, axis=1) < 1e-4
        u[vertical] = np.cross(batch_normals[vertical], [0, 1, 0])
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-12)
        v = np.cross(batch_normals, u)
        valid = (np.linalg.norm(vectors, axis=2) >= 1e-5) & (
            np.linalg.norm(batch_normals, axis=1)[:, None] > 0)
        angles = np.arctan2(np.einsum('nki,ni->nk', vectors, v),
                            np.einsum('nki,ni->nk', vectors, u))
        angles[~valid] = np.inf
        angles.sort(axis=1)
        counts = valid.sum(axis=1)
        has_neighbors = counts > 0
        interior = np.arange(k_actual - 1)[None, :] < (counts - 1)[:, None]
        gaps = np.subtract(angles[:, 1:], angles[:, :-1],
                           out=np.zeros((len(batch), k_actual - 1)), where=interior)
        last = angles[np.arange(len(batch)), np.maximum(counts - 1, 0)]
        wrap = np.zeros(len(batch))
        wrap[has_neighbors] = (angles[has_neighbors, 0] + 2 * np.pi
                               - last[has_neighbors])
        largest = np.maximum(gaps.max(axis=1), wrap)
        frontier_indices.extend((start + np.flatnonzero(
            has_neighbors & (largest > angle_threshold))).tolist())
    return frontier_indices

def cluster_frontiers(points, normals, frontier_indices, fov_radius=1.0, max_angle_deg=45, min_samples=5):
    if not frontier_indices:
        return []

    frontier_pts = points[frontier_indices]
    frontier_norms = normals[frontier_indices]

    unassigned = list(range(len(frontier_pts)))
    clusters = []
    cluster_id = 0
    max_angle_rad = np.radians(max_angle_deg)

    while unassigned:
        seed_idx = unassigned[0]
        seed_pt = frontier_pts[seed_idx]
        seed_norm = frontier_norms[seed_idx]

        indices = np.asarray(unassigned)
        distances = np.linalg.norm(frontier_pts[indices] - seed_pt, axis=1)
        angles = np.arccos(np.clip(frontier_norms[indices] @ seed_norm, -1.0, 1.0))
        accepted = (distances <= fov_radius) & (angles <= max_angle_rad)
        cluster_mask = indices[accepted]
        unassigned = indices[~accepted].tolist()

        if len(cluster_mask) < min_samples:
            continue

        c_pts = frontier_pts[cluster_mask]
        c_norms = frontier_norms[cluster_mask]
        c_indices = np.array(frontier_indices)[cluster_mask]

        centroid = np.mean(c_pts, axis=0)
        avg_normal = np.mean(c_norms, axis=0)
        avg_normal = avg_normal / np.linalg.norm(avg_normal)

        clusters.append({
            'id': cluster_id,
            'centroid': centroid,
            'avg_normal': avg_normal,
            'points': c_pts,
            'normals': c_norms,
            'global_indices': c_indices
        })
        cluster_id += 1

    return clusters

def generate_cone_viewpoints(
        target_pt, normal, standoff=1.5, polar_angles_deg=None,
        azimuth_steps=8):
    if polar_angles_deg is None:
        polar_angles_deg = [15, 30]
    normal = normal / np.linalg.norm(normal)
    u = np.cross(normal, np.array([0, 0, 1]))
    if np.linalg.norm(u) < 1e-4:
        u = np.cross(normal, np.array([0, 1, 0]))
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)

    candidates = []
    candidates.append({'position': target_pt + (normal * standoff)})

    azimuth_angles = np.linspace(0, 360, azimuth_steps, endpoint=False)
    for polar_deg in polar_angles_deg:
        polar_rad = np.radians(polar_deg)
        sin_p, cos_p = np.sin(polar_rad), np.cos(polar_rad)
        for az_deg in azimuth_angles:
            az_rad = np.radians(az_deg)
            x_local, y_local, z_local = sin_p * np.cos(az_rad), sin_p * np.sin(az_rad), cos_p
            ray_dir = (x_local * u) + (y_local * v) + (z_local * normal)
            candidates.append({'position': target_pt + (ray_dir * standoff)})

    return candidates

def evaluate_viewpoint(vp_pos, cluster_pts, cluster_normals, cluster_complexities,
                       complexity_weight=15.0):
    rays = np.asarray(cluster_pts) - vp_pos
    distances = np.linalg.norm(rays, axis=1)
    valid = distances >= 1e-4
    directions = rays[valid] / distances[valid, None]
    cos_theta = np.clip(np.sum(-directions * np.asarray(cluster_normals)[valid],
                               axis=1), 0.0, 1.0)
    return float(np.sum(cos_theta * (1.0 + complexity_weight * np.asarray(cluster_complexities)[valid])))

def calculate_kinematic_cost(
    auv_pos, auv_heading, vp_pos, target_pos, pcd_tree,
    distance_weight=1.0, rotation_weight=0.5, collision_weight=2.0,
    collision_distance_threshold=0.8, collision_distance_offset=0.1):
    travel_vec = vp_pos - auv_pos
    dist = np.linalg.norm(travel_vec)

    if dist > 1e-4:
        travel_dir = travel_vec / dist
        cos_turn1 = np.clip(np.dot(auv_heading, travel_dir), -1.0, 1.0)
        turn1_angle = np.arccos(cos_turn1)
    else:
        turn1_angle = 0.0

    view_vec = target_pos - vp_pos
    view_dist = np.linalg.norm(view_vec)

    if view_dist > 1e-4 and dist > 1e-4:
        view_dir = view_vec / view_dist
        cos_turn2 = np.clip(np.dot(travel_dir, view_dir), -1.0, 1.0)
        turn2_angle = np.arccos(cos_turn2)
    else:
        turn2_angle = 0.0

    total_rotation = turn1_angle + turn2_angle
    nearest_dist, _ = pcd_tree.query(vp_pos, k=1)
    collision_risk = (
        1.0 / (nearest_dist + collision_distance_offset)
        if nearest_dist < collision_distance_threshold else 0.0)
    raw_cost = (
        (distance_weight * dist)
        + (rotation_weight * total_rotation)
        + (collision_weight * collision_risk))
    return raw_cost

def normalize_array(arr):
    arr = np.array(arr)
    val_min, val_max = arr.min(), arr.max()
    if val_max - val_min < 1e-6:
        return np.zeros_like(arr)
    return (arr - val_min) / (val_max - val_min)

# ==========================================
# MAIN NBV PIPELINE FUNCTION
# ==========================================
def calculate_nbv_waypoints(
    occupied_points, auv_position, auv_heading, observation_history=None,
    parameters=None, timing_callback=None, *, preprocessor, diagnostics, clock):
    """
    Takes raw OctoMap occupied points and returns a sorted list of the best 
    Next-Best-View (NBV) waypoints for the AUV trajectory planner.
    """
    diagnostics.info('Calculating NBV waypoints...')
    def timed(stage, operation):
        started = clock.perf_counter()
        value = operation()
        if timing_callback is not None:
            timing_callback(stage, (clock.perf_counter() - started) * 1000.0)
        return value
    parameters = {**DEFAULT_NBV_PARAMETERS, **(parameters or {})}
    if observation_history is None:
        observation_history = []
    points_raw = np.asarray(occupied_points)
    if len(points_raw) == 0:
        return []

    # 1. Pre-Processing (Filter seabed & clean noise)
    z_min_target = float(parameters['z_min_target'])
    pipe_points = points_raw[points_raw[:, 2] > z_min_target]
    if len(pipe_points) == 0:
        return []

    pcd = preprocessor.prepare(pipe_points, auv_position, parameters, timed)
    if pcd is None:
        return []

    cleaned_points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    pcd_tree = KDTree(cleaned_points)

    # 2. Mathematical Pipeline
    complexities = timed('nbv_pca', lambda: calculate_pca_complexity(
        pcd, k=int(parameters['pca_neighbors'])))
    frontier_indices = timed('nbv_frontier_detection', lambda: detect_frontiers(
        pcd,
        k=int(parameters['frontier_neighbors']),
        angle_threshold_deg=float(parameters['frontier_angle_threshold_deg'])))
    clusters = timed('nbv_clustering', lambda: cluster_frontiers(
        cleaned_points,
        normals,
        frontier_indices,
        fov_radius=float(parameters['cluster_fov_radius']),
        max_angle_deg=float(parameters['cluster_max_angle_deg']),
        min_samples=int(parameters['cluster_min_samples'])))

    if not clusters:
        return []

    # 3. Candidate Generation & Raw Evaluation
    evaluation_started = clock.perf_counter()
    all_candidates = []
    min_raw_gain = float(parameters['min_raw_gain'])
    observation_positions = np.asarray(
        [sample[0] for sample in observation_history], dtype=float).reshape(-1, 3)
    observation_yaws = np.asarray([sample[1] for sample in observation_history])

    for cluster in clusters:
        vectors = cluster['centroid'] - observation_positions
        distances = np.linalg.norm(vectors, axis=1)
        angles = np.arctan2(vectors[:, 1], vectors[:, 0]) - observation_yaws
        yaw_diffs = np.abs(np.arctan2(np.sin(angles), np.cos(angles)))
        already_visited = np.any(
            (distances > parameters['visited_min_distance'])
            & (distances < parameters['visited_max_distance'])
            & (yaw_diffs < np.radians(parameters['visited_yaw_threshold_deg'])))
        if already_visited:
            continue # Drop this cluster, we have confirmed visual coverage

        # =========================================================
        # THE MISSING MATH BLOCK: Generate & evaluate viewpoints
        # =========================================================
        c_complexities = complexities[cluster['global_indices']]
        candidates = generate_cone_viewpoints(
            cluster['centroid'],
            cluster['avg_normal'],
            standoff=float(parameters['standoff']),
            polar_angles_deg=parameters['polar_angles_deg'],
            azimuth_steps=int(parameters['azimuth_steps']))

        for cand in candidates:
            gain = evaluate_viewpoint(
                cand['position'], cluster['points'], cluster['normals'], c_complexities,
                complexity_weight=float(parameters['complexity_weight']))

            # Apply the threshold
            if gain < min_raw_gain:
                continue

            cost = calculate_kinematic_cost(
                auv_position,
                auv_heading,
                cand['position'],
                cluster['centroid'],
                pcd_tree,
                distance_weight=float(parameters['distance_weight']),
                rotation_weight=float(parameters['rotation_weight']),
                collision_weight=float(parameters['collision_weight']),
                collision_distance_threshold=float(
                    parameters['collision_distance_threshold']),
                collision_distance_offset=float(
                    parameters['collision_distance_offset']))

            all_candidates.append({
                'cluster_id': cluster['id'],
                'target': cluster['centroid'],
                'position': cand['position'],
                'raw_gain': gain,
                'raw_cost': cost
            })

    # If all frontiers were either visited or below the gain threshold, exploration is done
    if timing_callback is not None:
        timing_callback('nbv_candidate_evaluation',
                        (clock.perf_counter() - evaluation_started) * 1000.0)
    if not all_candidates:
        return []

    # 4. Global Utility Normalization
    gains = [c['raw_gain'] for c in all_candidates]
    costs = [c['raw_cost'] for c in all_candidates]

    norm_gains = normalize_array(gains)
    norm_costs = normalize_array(costs)

    cluster_winners = {}
    for i, cand in enumerate(all_candidates):
        cand['utility'] = norm_gains[i] - norm_costs[i]
        cid = cand['cluster_id']

        if cid not in cluster_winners or cand['utility'] > cluster_winners[cid]['utility']:
            cluster_winners[cid] = cand

    # 5. Format and Sort Output
    final_waypoints = []
    for cid, winner in cluster_winners.items():
        final_waypoints.append({
            'cluster_id': cid,
            'position': winner['position'],
            'target': winner['target'],
            'utility': winner['utility'],
            'raw_gain': winner['raw_gain'],
            'raw_cost': winner['raw_cost']
        })

    final_waypoints.sort(key=lambda x: x['utility'], reverse=True)
    return final_waypoints
````

### NBV_paper_ros2/src/auv_planner_reference/auv_planner_reference/use_cases/trajectory_optimization.py

SHA-256: `73c25362183c34f0185bd3c8a49f99e4d1eeb1ae2692d6586b44f437f1b4d1f8`

````python
"""Geometric motion continuity for the existing untimed [x, y, z, yaw] API.

Costs are soft: a necessary obstacle detour or return to a frontier is allowed.
This is not a time-parameterized dynamic feasibility solver.
"""

import numpy as np

from ..route_planning import path_is_collision_free


def estimate_velocity(history, window=1.0):
    """Estimate world velocity from timestamped poses, avoiding twist-frame ambiguity."""
    if len(history) < 2:
        return np.zeros(3)
    latest_time = history[-1][0]
    samples = [sample for sample in history
               if 0.0 <= latest_time - sample[0] <= window]
    if len(samples) < 2:
        return np.zeros(3)
    times = np.asarray([sample[0] - latest_time for sample in samples], dtype=float)
    positions = np.asarray([sample[1] for sample in samples])
    times -= times.mean()
    denominator = np.dot(times, times)
    if denominator < 1e-9 or not np.all(np.isfinite(positions)):
        return np.zeros(3)
    return np.sum(times[:, None] * (positions - positions.mean(axis=0)),
                  axis=0) / denominator


def continuity_cost(trajectory, velocity, recent_positions=(),
                    continuity_weight=8.0, reversal_weight=4.0,
                    history_weight=2.0, history_radius=0.5):
    """Score entry direction, local reversals, yaw winding, and recent retracing."""
    if len(trajectory) < 2:
        return 0.0
    steps = np.diff(trajectory[:, :3], axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    directions = steps / np.maximum(lengths[:, None], 1e-6)
    moving = lengths > 1e-6
    directions = directions[moving]
    cost = 0.0
    speed = np.linalg.norm(velocity)
    if speed > 0.05 and len(directions):
        # A soft entry tangent, fading along the first few samples.
        alignment = directions[:3] @ (velocity / speed)
        cost += continuity_weight * min(speed / 0.3, 1.0) * np.sum(
            np.array([1.0, 0.5, 0.25])[:len(alignment)] * (1.0 - alignment)**2)
    if len(directions) > 1:
        alignment = np.sum(directions[1:] * directions[:-1], axis=1)
        cost += reversal_weight * np.sum(np.maximum(0.0, -alignment)**2)
        # Smooth circles have no locally reversed segments. Detect accumulated
        # turning over the next 10 m as well, without penalizing a long mission's
        # entire sequence of otherwise necessary turns.
        horizon = np.cumsum(lengths[moving]) <= 10.0
        turn_angles = np.arctan2(
            np.linalg.norm(np.cross(directions[:-1], directions[1:]), axis=1),
            alignment)
        excess_turn = max(0.0, np.sum(turn_angles[horizon[1:]]) - np.pi)
        cost += reversal_weight * excess_turn**2
    yaw = np.unwrap(trajectory[:, 3])
    yaw_steps = np.diff(yaw)
    # Penalize accumulated travel even if the final heading matches the start.
    shortest_yaw = abs(np.arctan2(np.sin(yaw[-1] - yaw[0]),
                                  np.cos(yaw[-1] - yaw[0])))
    excess = max(0.0, np.abs(yaw_steps).sum() - shortest_yaw - np.pi)
    cost += reversal_weight * (np.sum(yaw_steps**2) + excess**2)
    history = np.asarray(recent_positions, dtype=np.float64)
    if history.size and history_radius > 0.0:
        history = history.reshape(-1, 3)
        # Exclude the immediate pose neighborhood; departing it is unavoidable.
        history = history[np.linalg.norm(history - trajectory[0, :3], axis=1)
                          > 2.0 * history_radius]
        if len(history):
            distances = np.linalg.norm(
                trajectory[1:, None, :3] - history[None, :, :], axis=2).min(axis=1)
            cost += history_weight * np.sum(
                np.maximum(0.0, 1.0 - distances / history_radius)**2)
    return float(cost)


def optimize_trajectory(trajectory, current_position, current_yaw, velocity,
                        occupied, resolution, recent_positions=(),
                        continuity_weight=8.0, reversal_weight=4.0,
                        history_weight=2.0, history_radius=0.5,
                        max_deviation=0.5, max_iterations=40,
                        collision_spacing_factor=0.5, *, solver):
    """Optimize bounded offsets, accepting only a safer, lower-cost trajectory.

    At most 32 controls keep replanning bounded. Offsets are interpolated onto
    every input sample so collision checks retain the original path detail.
    End positions remain fixed; the initial yaw equals the measured heading.
    """
    reference = np.asarray(trajectory, dtype=np.float64).copy()
    if len(reference) < 2:
        return reference
    reference[0, :3] = current_position
    reference[:, 3] = np.unwrap(np.r_[current_yaw, reference[:, 3]])[1:]
    reference[0, 3] = current_yaw
    if not path_is_collision_free(reference[:, :3], occupied, resolution,
                                  collision_spacing_factor):
        return np.empty((0, 4))
    weights = dict(continuity_weight=continuity_weight,
                   reversal_weight=reversal_weight,
                   history_weight=history_weight, history_radius=history_radius)
    indices = np.unique(np.linspace(0, len(reference) - 1,
                                    min(32, len(reference))).round().astype(int))
    sample_indices = np.arange(len(reference))
    reference_lengths = np.linalg.norm(np.diff(reference[:, :3], axis=0), axis=1)

    def decode(offsets):
        controls = offsets.reshape(-1, 4)
        return reference + np.column_stack([
            np.interp(sample_indices, indices, controls[:, axis])
            for axis in range(4)])

    def objective(offsets):
        trial = decode(offsets)
        fidelity = 2.0 * np.sum((trial[:, :3] - reference[:, :3])**2)
        observation = 0.5 * np.mean((trial[:, 3] - reference[:, 3])**2)
        steps = np.diff(trial[:, :3], axis=0)
        smoothness = np.sum(np.diff(steps, axis=0)**2)
        # Prevent lowering the directional cost by collapsing the first steps.
        length_cost = 4.0 * np.sum(((np.linalg.norm(steps, axis=1) - reference_lengths)
                                   / np.maximum(reference_lengths, 0.1))**2)
        return (fidelity + observation + smoothness + length_cost
                + continuity_cost(trial, velocity, recent_positions, **weights))

    bounds = [(-max_deviation, max_deviation)] * (len(indices) * 4)
    for axis in range(4):
        bounds[axis] = (0.0, 0.0)
    for axis in range(3):
        bounds[-4 + axis] = (0.0, 0.0)
    # Yaw has a wider range than metric position offsets.
    for index in range(1, len(indices)):
        bounds[index * 4 + 3] = (-np.pi, np.pi)
    initial = np.zeros(len(indices) * 4)
    result = solver.minimize(objective, initial, method='L-BFGS-B', bounds=bounds,
                      options={'maxiter': int(max_iterations), 'maxfun': 12000})
    # An iteration-limited result can still be useful, but never trust NaNs or
    # an unsafe/lower-quality iterate. Backtracking preserves a safe fallback.
    if np.all(np.isfinite(result.x)):
        initial_cost = objective(initial)
        for scale in (1.0, 0.5, 0.25):
            trial = decode(scale * result.x)
            if (objective(scale * result.x) < initial_cost
                    and path_is_collision_free(trial[:, :3], occupied, resolution,
                                               collision_spacing_factor)):
                return trial
    return reference
````

### NBV_paper_ros2/src/auv_planner_reference/setup.py

SHA-256: `a27d412776d21f5113c98d14cf32b16217250c38b9b601d9f1591d1b1c3ecd32`

````python
from setuptools import find_packages, setup

package_name = 'auv_planner_reference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'open3d', 'pyoctomap'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_node = auv_planner_reference.planner_node:main'
        ],
    },
)
````

### NBV_paper_ros2/src/auv_planner_reference/test/test_nbv_config.py

SHA-256: `9d0ae0f65297fe12f0005e00726e00878fe02e6240cb53e99949ee95bda2916e`

````python
"""Keep split NBV tuning aligned with the parameters declared by the planner."""

import ast
from pathlib import Path

import yaml


def test_split_nbv_config_contains_supported_tuning_parameters():
    source_root = Path(__file__).parents[2]
    tree = ast.parse((source_root / 'auv_planner_reference/auv_planner_reference/domain/nbv_parameters.py').read_text())
    defaults = next(ast.literal_eval(node.value) for node in tree.body
                    if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'DEFAULT_NBV_PARAMETERS'
                            for target in node.targets))
    config = source_root / 'auv_bringup/config'
    tuning = yaml.safe_load((config / 'nbv_params.yaml').read_text())['planner_node']['ros__parameters']
    main = yaml.safe_load((config / 'params.yaml').read_text())['planner_node']['ros__parameters']
    assert set(tuning) == {f'nbv_{key}' for key in defaults}
    assert not set(tuning).intersection(main)
    # The shipped tuning profile deliberately overrides the mathematical default.
    assert defaults['complexity_weight'] == 15.
    assert tuning['nbv_complexity_weight'] == 30.
````

### NBV_paper_ros2/src/auv_planner_reference/test/test_nbv_filter_logging.py

SHA-256: `f079b575f55007110ad6890660adf4d5dda5532c08be3643b9aef4a4d230d8a7`

````python
"""Exercise rejection diagnostics without starting ROS or the simulator."""

import ast
import importlib.util
import json
import inspect
from functools import partial
from dataclasses import dataclass
import math
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest
from scipy.spatial import KDTree


ROOT = Path(__file__).parents[1]


def load_file(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


routes = load_file(ROOT / 'auv_planner_reference/route_planning.py', 'diagnostic_routes')
defaults = load_file(ROOT / 'auv_planner_reference/domain/nbv_parameters.py', 'diagnostic_defaults')
telemetry = load_file(
    ROOT.parent / 'auv_telemetry/auv_telemetry/file_sinks.py',
    'diagnostic_telemetry')


@pytest.fixture
def pipeline():
    source = ROOT / 'auv_planner_reference/use_cases/nbv.py'
    tree = ast.parse(source.read_text())
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             or (isinstance(node, ast.Assign)
                 and any(isinstance(target, ast.Name) and target.id == 'DEFAULT_NBV_PARAMETERS'
                         for target in node.targets))]

    class Cloud:
        def __init__(self):
            self.points = np.empty((0, 3))

        def voxel_down_sample(self, voxel_size):
            return self

        def remove_statistical_outlier(self, **kwargs):
            return self, list(range(len(self.points)))

        def estimate_normals(self, **kwargs):
            self.normals = np.tile([0., 0., 1.], (len(self.points), 1))

        def orient_normals_towards_camera_location(self, position):
            pass

    ns = dict(np=np, time=time, KDTree=KDTree, DEFAULT_NBV_PARAMETERS=defaults.DEFAULT_NBV_PARAMETERS,
              LOGGER=SimpleNamespace(info=lambda *args: None),
              o3d=SimpleNamespace(
                  geometry=SimpleNamespace(PointCloud=Cloud,
                                           KDTreeSearchParamHybrid=lambda **kwargs: None),
                  utility=SimpleNamespace(Vector3dVector=np.asarray)))
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), ns)
    adapter_source = ROOT / 'auv_planner_reference/adapters/point_cloud.py'
    adapter_nodes = [n for n in ast.parse(adapter_source.read_text()).body
                     if isinstance(n, ast.ClassDef)]
    ns['dataclass'] = dataclass
    exec(compile(ast.Module(body=adapter_nodes, type_ignores=[]), str(adapter_source), 'exec'), ns)
    ns['calculate_nbv_waypoints'] = partial(ns['calculate_nbv_waypoints'],
        preprocessor=ns['Open3DPreprocessor'](), diagnostics=ns['LOGGER'], clock=time)
    points = np.column_stack((np.linspace(0, 1, 60), np.zeros(60), np.zeros(60)))
    cluster = dict(id=0, centroid=np.array([0., 0., 0.]),
                   avg_normal=np.array([0., 0., 1.]), points=points,
                   normals=np.tile([0., 0., 1.], (60, 1)),
                   global_indices=np.arange(60))
    ns['calculate_pca_complexity'] = lambda cloud, **kwargs: np.ones(len(cloud.points))
    ns['detect_frontiers'] = lambda cloud, **kwargs: list(range(len(cloud.points)))
    ns['cluster_frontiers'] = lambda *args, **kwargs: [cluster]
    ns['generate_cone_viewpoints'] = lambda *args, **kwargs: [dict(position=np.array([0., 0., 1.]))]
    ns['evaluate_viewpoint'] = lambda *args, **kwargs: 1.0
    ns['calculate_kinematic_cost'] = lambda *args, **kwargs: 1.0
    return ns, points


@pytest.mark.parametrize('case,reason', [
    ('empty', 'occupied_input_empty'),
    ('z', 'world_z_removed_all'),
    ('small', 'minimum_cloud_size'),
    ('frontiers', 'no_frontiers'),
    ('clusters', 'clustering_removed_all'),
    ('cone', 'cone_generated_none'),
    ('gain', 'raw_gain_removed_all'),
])
def test_empty_pipeline_returns_no_candidates(pipeline, case, reason):
    ns, points = pipeline
    parameters = {'z_min_target': -1.0}
    history = []
    if case == 'empty':
        points = np.empty((0, 3))
    elif case == 'z':
        parameters['z_min_target'] = 0.0  # Equality must be rejected too.
    elif case == 'small':
        points = points[:39]
    elif case == 'frontiers':
        ns['detect_frontiers'] = lambda *args, **kwargs: []
        ns['cluster_frontiers'] = lambda *args, **kwargs: []
    elif case == 'clusters':
        ns['cluster_frontiers'] = lambda *args, **kwargs: []
    elif case == 'cone':
        ns['generate_cone_viewpoints'] = lambda *args, **kwargs: []
    elif case == 'gain':
        ns['evaluate_viewpoint'] = lambda *args, **kwargs: 0.0
    result = ns['calculate_nbv_waypoints'](
        points, np.zeros(3), np.array([1., 0., 0.]), history,
        parameters)
    assert result == []


def test_timing_preserves_candidates(pipeline):
    ns, points = pipeline
    args = (points, np.zeros(3), np.array([1., 0., 0.]))
    events = []
    baseline = ns['calculate_nbv_waypoints'](*args)
    logged = ns['calculate_nbv_waypoints'](*args, timing_callback=lambda *e: events.append(e))
    assert len(baseline) == len(logged) == 1
    for key in baseline[0]:
        np.testing.assert_equal(baseline[0][key], logged[0][key])
    assert events
    assert all(duration >= 0. for stage, duration in events)


def test_pose_history_removes_matching_frontier(pipeline):
    ns, points = pipeline
    result = ns['calculate_nbv_waypoints'](
        points, np.zeros(3), np.array([1., 0., 0.]),
        [(np.array([-1., 0., 0.]), 0.0)])
    assert result == []


def test_cluster_rejects_groups_below_minimum_sample_count(pipeline):
    ns, _ = pipeline
    source = ast.parse((ROOT / 'auv_planner_reference/use_cases/nbv.py').read_text())
    function = next(n for n in source.body if isinstance(n, ast.FunctionDef)
                    and n.name == 'cluster_frontiers')
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<cluster>', 'exec'), ns)
    result = ns['cluster_frontiers'](
        np.array([[1., 0., 0.], [1.1, 0., 0.]]), np.tile([0., 0., 1.], (2, 1)),
        [0, 1], min_samples=5)
    assert result == []


def test_node_nbv_call_matches_restored_signature(pipeline):
    ns, _ = pipeline
    tree = ast.parse((ROOT / 'auv_planner_reference/use_cases/mission.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == 'nbv_calculator']
    assert calls
    for call in calls:
        inspect.signature(ns['calculate_nbv_waypoints']).bind(
            *[object() for _ in call.args], **{kw.arg: object() for kw in call.keywords})


def test_pipeline_passes_complexity_weight_to_gain_calculation(pipeline):
    ns, points = pipeline
    source = ROOT / 'auv_planner_reference/use_cases/nbv.py'
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'evaluate_viewpoint')
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), ns)
    args = (points, np.zeros(3), np.array([1., 0., 0.]))
    zero = ns['calculate_nbv_waypoints'](*args, parameters={'complexity_weight': 0.})
    weighted = ns['calculate_nbv_waypoints'](*args, parameters={'complexity_weight': 4.})
    assert len(zero) == len(weighted) == 1
    assert zero[0]['raw_gain'] > 0.
    # Fixture complexities are all one, so each point's gain increases fivefold.
    assert weighted[0]['raw_gain'] == pytest.approx(5. * zero[0]['raw_gain'])


@pytest.mark.parametrize('occupied,budget,found', [
    ({(2, 0, 0)}, 20, False),
    (set(), 1, False),
    (set(), 20, True),
])
def test_astar_return_matches_occupied_endpoints_and_expansion_limit(occupied, budget, found):
    args = (np.array([.5, .5, .5]), np.array([2.5, .5, .5]), occupied, 1.0)
    assert bool(routes.astar_3d(*args, max_steps=budget)) == found


def test_collision_rejects_occupied_voxel_between_endpoints():
    path = np.array([[.5, .5, .5], [2.5, .5, .5]])
    assert not routes.path_is_collision_free(
        path, {(1, 0, 0)}, 1.0)


def test_route_skips_occupied_viewpoint():
    result, targets = routes.plan_multi_waypoint_route(
        np.array([.5, .5, .5]),
        [(np.array([2.5, .5, .5]), np.array([3., 0., 0.]))],
        {(2, 0, 0)}, 1.0)
    assert len(result) == len(targets) == 0


def test_jsonl_logs_iteration_and_handles_nonfinite_values(tmp_path):
    logger = telemetry.MidrunLogger(tmp_path, prefix='planner')
    logger.record_filter('world_z', 60, 0,
                         {'threshold': np.float64(-298), 'value': np.nan},
                         iteration=100, frame_id=101)
    event = json.loads(logger.filter_path.read_text())
    assert event['iteration'] == 100 and event['frame_id'] == 101
    assert event['input_count'] == 60 and event['output_count'] == 0
    assert event['details'] == {'threshold': -298.0, 'value': 'nan'}


def node_method(name, **extra_globals):
    source = ROOT / 'auv_planner_reference/use_cases/mission.py'
    tree = ast.parse(source.read_text())
    node_class = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    method = next(node for node in node_class.body
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = dict(np=np, math=math,
                     Float32MultiArray=lambda: SimpleNamespace(data=[]),
                     path_is_collision_free=routes.path_is_collision_free)
    namespace.update(extra_globals)
    helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
               and node.name in ('body_rotation', 'vehicle_viewpoint')]
    exec(compile(ast.Module(body=[*helpers, method], type_ignores=[]), str(source), 'exec'), namespace)
    # Method fixtures supply the same injected ports as the production composition root.
    output_source = ROOT / 'auv_planner_reference/adapters/ros_output.py'
    output_nodes = [n for n in ast.parse(output_source.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == 'RosTrajectoryOutput']
    exec(compile(ast.Module(body=output_nodes, type_ignores=[]), str(output_source), 'exec'), namespace)
    def invoke(node, *args, **kwargs):
        node.clock = extra_globals.get('time', time)
        if hasattr(node, 'get_logger'):
            node.diagnostics = node.get_logger()
        if hasattr(node, 'waypoint_pub'):
            node.output = namespace['RosTrajectoryOutput'](node.waypoint_pub)
        return namespace[name](node, *args, **kwargs)
    return invoke


def test_node_reports_utility_removing_all_candidates():
    events = []
    node = SimpleNamespace(min_utility=.1, _log_nbv_filter=lambda *e: events.append(e))
    candidate = dict(cluster_id=0, utility=0., target=np.zeros(3), position=np.ones(3))
    assert node_method('_plan_and_publish')(node, [candidate], set()) is False
    assert node._plan_failure_reason == 'minimum_utility_removed_all'
    assert events[-1][:3] == ('minimum_utility', 1, 0)
    assert events[0][3]['utility'] == 0


def test_depth_range_diagnostics_count_disjoint_rejections():
    events = []
    node = SimpleNamespace(depth_scale=1., depth_stride=1, min_depth=.5, max_depth=4.,
                           latched_roll=0., latched_pitch=0., latched_yaw=0.,
                           latched_position=np.zeros(3), camera_offset=np.zeros(3),
                           _log_nbv_filter=lambda *e: events.append(e))
    points = node_method('_depth_points')(node, np.array([[np.nan, .5, 4., 1.]]))
    assert len(points) == 1
    assert events[0][:3] == ('depth_range', 4, 1)
    assert events[0][3]['nonfinite'] == 1
    assert events[0][3]['at_or_below_min'] == 1
    assert events[0][3]['at_or_above_max'] == 1


def test_disabled_node_logging_does_not_access_logger_or_files():
    node = SimpleNamespace(nbv_filter_logging=False)
    node_method('_log_nbv_filter')(node, 'test', 1, 0, {})


def test_blocked_route_cancellation_reaches_downstream_queue():
    messages = []
    events = []
    node = SimpleNamespace(active_trajectory=np.ones((3, 4)),
                           selected_targets=[np.ones(3)],
                           waypoint_pub=SimpleNamespace(publish=messages.append),
                           _log_nbv_filter=lambda *event: events.append(event))
    node_method('_cancel_active_trajectory')(node)
    assert len(messages) == 1 and messages[0].data == []
    assert node.active_trajectory.shape == (0, 4)
    assert node.selected_targets == []
    assert events[0][:3] == ('trajectory_cancelled', 3, 0)




def test_depth_sensor_mount_offset_rotates_with_vehicle():
    node = SimpleNamespace(depth_scale=1., depth_stride=1, min_depth=.5, max_depth=4.,
                           latched_roll=0., latched_pitch=0., latched_yaw=math.pi / 2,
                           latched_position=np.array([10., 20., -5.]),
                           camera_offset=np.array([1., 0., -.5]),
                           _log_nbv_filter=lambda *args: None)
    depth = np.full((2, 2), np.nan)
    depth[1, 1] = 2.
    points = node_method('_depth_points')(node, depth)
    np.testing.assert_allclose(node._depth_sensor_origin, [10., 21., -5.5])
    np.testing.assert_allclose(points[0], [10., 21. + math.sqrt(2), -5.5 - math.sqrt(2)])


def test_vehicle_viewpoint_places_rotated_mount_at_requested_camera_position():
    camera = np.array([5., 8., -3.])
    target = np.array([4., 10., -4.])
    offset = np.array([.35, .1, -.25])
    source = ROOT / 'auv_planner_reference/use_cases/mission.py'
    helpers = [node for node in ast.parse(source.read_text()).body
               if isinstance(node, ast.FunctionDef) and node.name in ('body_rotation', 'vehicle_viewpoint')]
    namespace = dict(np=np, math=math)
    exec(compile(ast.Module(body=helpers, type_ignores=[]), str(source), 'exec'), namespace)
    vehicle = namespace['vehicle_viewpoint'](camera, target, offset)
    yaw = math.atan2(target[1] - vehicle[1], target[0] - vehicle[0])
    np.testing.assert_allclose(vehicle + namespace['body_rotation'](0., 0., yaw) @ offset,
                               camera, atol=1e-7)


@pytest.mark.parametrize('first_spline_safe', [True, False])
def test_rejected_combined_spline_keeps_checked_first_structural_target(first_spline_safe):
    route = np.array([[.5, .5, .5], [1.5, .5, .5], [2.5, .5, .5], [3.5, .5, .5],
                      [3.5, 1.5, .5], [3.5, 2.5, .5], [2.5, 2.5, .5], [1.5, 2.5, .5]])
    targets = np.array([[4., 0., 0.]] * 4 + [[1., 3., 0.]] * 4)
    occupied = {(2, 1, 0)}
    events, published = [], []
    logger = SimpleNamespace(info=lambda *args: None, warning=lambda *args: None)
    node = SimpleNamespace(
        min_utility=.1, camera_offset=np.zeros(3), velocity_history_window=1.,
        trajectory_history_window=5., astar_max_steps=50000, optimize_motion=False,
        motion_cost_parameters={}, resolution=1., route_search_time_budget_s=2.,
        route_max_candidates=4, collision_check_spacing_factor=.5, nbv_filter_logging=True,
        telemetry=SimpleNamespace(record_stage=lambda *args, **kwargs: None), iteration=1,
        trajectory_points=300, spline_smoothing=2., waypoint_spacing=.5,
        _log_nbv_filter=lambda *event: events.append(event),
        _motion_snapshot=lambda: (route[0], 0., []), get_logger=lambda: logger,
        _publish_trajectory=lambda trajectory, occupancy: published.append(trajectory) or True)
    def spline(path, tracked, **kwargs):
        if len(path) == len(route) or not first_spline_safe:
            # Model a combined spline cutting the obstacle inside a safe A* detour.
            positions = np.vstack((path[0], [2.5, 1.5, .5], path[-1]))
            return np.column_stack((positions, np.zeros(3)))
        return routes.generate_smooth_trajectory(path, tracked, **kwargs)
    method = node_method('_plan_and_publish', time=time,
                         estimate_velocity=lambda *args: np.zeros(3),
                         plan_multi_waypoint_route=lambda *args, **kwargs: (route, targets),
                         generate_smooth_trajectory=spline,
                         resample_trajectory_by_distance=routes.resample_trajectory_by_distance,
                         reconnect_trajectory=routes.reconnect_trajectory)
    # Utility order can differ from A* travel order: do not blame this top target.
    candidate = dict(cluster_id=0, utility=1., target=targets[-1], position=route[-1])
    result = method(node, [candidate], occupied)
    if not first_spline_safe:
        assert not result and published == []
        assert node._plan_failure_reason == 'all_trajectory_options_rejected'
        np.testing.assert_equal(node._failed_route_target, targets[0])
        return
    assert result
    assert len(published) == 1
    assert routes.path_is_collision_free(published[0][:, :3], occupied, 1.)
    np.testing.assert_allclose(published[0][-1, :3], route[3])
    assert all(np.array_equal(target, targets[0]) for target in node.selected_targets)
    options = [event for event in events if event[0] == 'trajectory_option']
    assert [(event[3]['option'], event[3]['collision_free']) for event in options] == [
        ('spline', False), ('first_target_spline', True)]


def test_vehicle_route_is_checked_instead_of_camera_offset_route():
    messages = []
    node = SimpleNamespace(camera_offset=np.array([1., 0., 0.]), resolution=1.,
                           waypoint_pub=SimpleNamespace(publish=messages.append),
                           _log_nbv_filter=lambda *args: None)
    body = np.array([[.5, .5, .5, 0.], [.5, 2.5, .5, 0.]])
    occupied = {(0, 1, 0)}
    assert routes.path_is_collision_free(body[:, :3] + node.camera_offset, occupied, 1.)
    assert node_method('_publish_trajectory')(node, body, occupied) is False
    assert messages == []


def test_wire_validation_rejects_float32_rounding_into_occupied_voxel():
    messages = []
    node = SimpleNamespace(camera_offset=np.array([.35, 0., 0.]), resolution=1.,
                           waypoint_pub=SimpleNamespace(publish=messages.append),
                           _log_nbv_filter=lambda *args: None)
    body = np.array([[.99999999, .5, .5, 0.], [.99999999, 1.5, .5, 0.]])
    occupied = {(1, 0, 0)}
    assert routes.path_is_collision_free(body[:, :3], occupied, 1.)
    assert node_method('_publish_trajectory')(node, body, occupied) is False
    assert messages == []


def test_failed_target_blacklist_expires_and_requests_another_target():
    events = []
    clock = [100.]
    failed = np.array([357.6, 828.4, -296.1])
    node = SimpleNamespace(_failed_route_target=failed, blacklist_cooldown_s=30.,
                           _plan_failure_reason='all_trajectory_options_rejected',
                           blacklisted_targets=[], blacklist_expiries={}, force_replan=False,
                           _log_nbv_filter=lambda *event: events.append(event),
                           get_logger=lambda: SimpleNamespace(warning=lambda *args: None))
    fake_time = SimpleNamespace(perf_counter=lambda: clock[0])
    blacklist = node_method('_blacklist_failed_target', time=fake_time)
    expire = node_method('_expire_target_blacklist', time=fake_time)
    assert blacklist(node)
    assert node.force_replan
    np.testing.assert_equal(node.blacklisted_targets, [failed])
    assert node.blacklist_expiries[tuple(failed)] == 130.
    assert events[-1][3]['expires'] is True
    clock[0] = 129.
    expire(node)
    assert len(node.blacklisted_targets) == 1
    clock[0] = 130.
    expire(node)
    assert node.blacklisted_targets == [] and node.blacklist_expiries == {}
    assert events[-1][0] == 'blacklist_expired'


def test_failed_target_blacklist_deduplicates_nearby_cluster_drift():
    first = np.array([1., 2., 3.])
    node = SimpleNamespace(_failed_route_target=first, blacklist_cooldown_s=30.,
                           _plan_failure_reason='all_trajectory_options_rejected',
                           blacklisted_targets=[], blacklist_expiries={}, force_replan=False,
                           _log_nbv_filter=lambda *args: None,
                           get_logger=lambda: SimpleNamespace(warning=lambda *args: None))
    method = node_method('_blacklist_failed_target', time=SimpleNamespace(perf_counter=lambda: 100.))
    assert method(node)
    node._failed_route_target = first + [.1, 0., 0.]
    assert method(node)
    assert len(node.blacklisted_targets) == len(node.blacklist_expiries) == 1


@pytest.mark.parametrize('failed,cooldown', [(None, 30.), (np.ones(3), 0.)])
def test_global_failure_or_disabled_cooldown_does_not_ban_a_target(failed, cooldown):
    node = SimpleNamespace(_failed_route_target=failed, blacklist_cooldown_s=cooldown,
                           blacklisted_targets=[], blacklist_expiries={}, force_replan=False)
    assert node_method('_blacklist_failed_target')(node) is False
    assert node.blacklisted_targets == [] and not node.force_replan
````

### NBV_paper_ros2/src/auv_planner_reference/test/test_nbv_math.py

SHA-256: `d018ee06ad7a23d4a75e7937d14136afbcf41fb8e7cd13723b8d2357fd6274f9`

````python
"""Compare batched NBV math with the scalar formulas, without Open3D or ROS."""

import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import KDTree


source = Path(__file__).parents[1] / 'auv_planner_reference' / 'use_cases/nbv.py'
names = {'calculate_pca_complexity', 'detect_frontiers', 'cluster_frontiers',
         'evaluate_viewpoint'}
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[node for node in tree.body
                          if isinstance(node, ast.FunctionDef) and node.name in names],
                     type_ignores=[])
math_functions = {'np': np, 'KDTree': KDTree}
exec(compile(helpers, str(source), 'exec'), math_functions)


@pytest.mark.parametrize('weight,expected', [(0., 3.), (5., 6.), (15., 12.)])
def test_viewpoint_gain_uses_configured_complexity_weight(weight, expected):
    gain = math_functions['evaluate_viewpoint'](
        np.array([0., 0., 1.]), np.zeros((3, 3)),
        np.tile([0., 0., 1.], (3, 1)), np.array([.1, .2, .3]),
        complexity_weight=weight)
    assert gain == pytest.approx(expected)


@pytest.mark.parametrize('count', [40, 121, 1050])
def test_pca_matches_original_covariance_formula(count):
    points = np.random.default_rng(12).normal(size=(count, 3))
    expected = []
    neighbor_tree = KDTree(points)
    for point in points:
        neighbors = points[neighbor_tree.query(point, k=20)[1]]
        centered = neighbors - neighbors.mean(axis=0)
        values = np.linalg.eigvalsh(np.cov(centered.T))
        expected.append(values[0] / values.sum())
    result = math_functions['calculate_pca_complexity'](SimpleNamespace(points=points))
    np.testing.assert_allclose(result, expected, atol=1e-13)


@pytest.mark.parametrize('count', [40, 121, 1050])
def test_frontiers_match_original_angular_gap_formula(count):
    rng = np.random.default_rng(16)
    points = rng.normal(size=(count, 3))
    normals = rng.normal(size=(count, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    normals[:2] = [[0, 0, 1], [0, 0, -1]]
    neighbor_tree = KDTree(points)
    expected = []
    for i, (point, normal) in enumerate(zip(points, normals)):
        u = np.cross(normal, [0, 0, 1])
        if np.linalg.norm(u) < 1e-4:
            u = np.cross(normal, [0, 1, 0])
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        neighbors = points[neighbor_tree.query(point, k=40)[1]]
        angles = sorted(np.arctan2(np.dot(other - point, v),
                                  np.dot(other - point, u))
                        for other in neighbors if np.linalg.norm(other - point) >= 1e-5)
        if angles and np.max(np.diff(angles + [angles[0] + 2 * np.pi])) > np.radians(130):
            expected.append(i)
    assert math_functions['detect_frontiers'](SimpleNamespace(
        points=points, normals=normals)) == expected


@pytest.mark.parametrize('count', [0, 1, 2, 39])
def test_frontiers_handle_small_and_duplicate_clouds(count):
    pcd = SimpleNamespace(points=np.zeros((count, 3)),
                          normals=np.tile([0, 0, 1], (count, 1)))
    assert math_functions['detect_frontiers'](pcd) == []


def test_greedy_clustering_keeps_order_and_discards_small_clusters():
    points = np.array([[x + dx, 0, 0] for x in [0, 3] for dx in [0, .1, .2]]
                      + [[9, 0, 0]], dtype=float)
    normals = np.tile([0, 0, 1], (len(points), 1))
    result = math_functions['cluster_frontiers'](
        points, normals, list(range(len(points))), min_samples=3)
    assert [cluster['id'] for cluster in result] == [0, 1]
    assert [cluster['global_indices'].tolist() for cluster in result] == [[0, 1, 2], [3, 4, 5]]
    np.testing.assert_allclose(result[0]['centroid'], [.1, 0, 0])


def test_viewpoint_gain_matches_original_and_skips_coincident_point():
    rng = np.random.default_rng(21)
    position = np.array([1, 2, 3])
    points = rng.normal(size=(50, 3))
    points[0] = position
    normals = rng.normal(size=(50, 3))
    complexities = rng.uniform(0, .3, 50)
    expected = 0.0
    for point, normal, complexity in zip(points, normals, complexities):
        ray = point - position
        if np.linalg.norm(ray) < 1e-4:
            continue
        expected += np.clip(np.dot(-ray / np.linalg.norm(ray), normal), 0, 1) * (1 + 15 * complexity)
    actual = math_functions['evaluate_viewpoint'](position, points, normals, complexities)
    np.testing.assert_allclose(actual, expected)
````

### NBV_paper_ros2/src/auv_planner_reference/test/test_route_search_limits.py

SHA-256: `0369905e930436b63c5a62901df9facf08125151718ef3633a45d9ef339e7486`

````python
"""Regression checks for the restored route-planning API."""

import ast
import inspect
from pathlib import Path

import numpy as np

from auv_planner_reference import route_planning as routes


def test_runtime_callers_match_restored_route_planning_signatures():
    package = Path(__file__).parents[1] / 'auv_planner_reference'
    checked = 0
    for source in package.rglob('*.py'):
        tree = ast.parse(source.read_text())
        imports = {alias.asname or alias.name: alias.name
                   for node in tree.body if isinstance(node, ast.ImportFrom)
                   and node.module == 'route_planning' for alias in node.names}
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            name = imports.get(node.func.id)
            if name is None:
                continue
            assert all(keyword.arg is not None for keyword in node.keywords)
            signature = inspect.signature(getattr(routes, name))
            try:
                signature.bind(*[object() for _ in node.args],
                               **{keyword.arg: object() for keyword in node.keywords})
            except TypeError as error:
                raise AssertionError(f'{source.name}:{node.lineno}: {name}: {error}') from error
            checked += 1
    assert checked >= 10




def test_astar_preserves_detours_around_obstacles():
    start = np.array([.5, .5, .5])
    goal = np.array([3.5, .5, .5])
    occupied = {(1, 0, 0), (2, 0, 0)}
    path = routes.astar_3d(start, goal, occupied, 1.0)
    assert len(path) == 6  # Three horizontal and two detour steps.
    assert routes.path_is_collision_free(np.asarray(path), occupied, 1.0)



def test_resampling_handles_duplicate_and_stationary_points():
    stationary = np.tile([1., 2., 3., .5], (5, 1))
    result = routes.resample_trajectory_by_distance(
        stationary, .5, max_points=1)
    np.testing.assert_allclose(result, stationary[:1])
````

### NBV_paper_ros2/src/auv_sim_bridge/package.xml

SHA-256: `da41bd561faa484d9fafb3f53802d162e1c57d57060e60c7eb81174fd07e6f41`

````xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>auv_sim_bridge</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>auv_contracts</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-numpy</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
````

### NBV_paper_ros2/src/auv_sim_bridge/setup.py

SHA-256: `4551dbd487158ff2a231ef45f484d8e3d07ee1ae5a8f1b30ba665c0e9c2df129`

````python
from setuptools import find_packages, setup

package_name = 'auv_sim_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'holoocean'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'holoocean_node = auv_sim_bridge.holoocean_node:main'
        ],
    },
)
````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/adapters/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/adapters/holoocean.py

SHA-256: `f17cc5232a13582fdd8f82cfbeb92927c236819c65863ca94f5b64dcdd52f858`

````python
from contextlib import contextmanager
import json

def load_scenario(path):
    with open(path, 'r') as stream:
        return json.load(stream)

class HoloOceanSessions:
    def __init__(self, make, scenario):
        self.make = make
        self.scenario = scenario
    @contextmanager
    def open(self):
        with self.make(scenario_cfg=self.scenario) as environment:
            yield HoloOceanSession(environment)

class HoloOceanSession:
    def __init__(self, environment):
        self.environment = environment
    def step(self, command):
        return self.environment.step(list(command.values))
    def set_ocean_currents(self, agent, velocity):
        return self.environment.set_ocean_currents(agent, velocity)
````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/adapters/ros_sensors.py

SHA-256: `ff4530056e97c1fb2e07b5432992ff87e470456aaf1837a427c1b22aa0e8e3ed`

````python
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np
import time
import traceback

from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField
from std_msgs.msg import Float32, Header, Float32MultiArray
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion, Pose, Point
from nav_msgs.msg import Odometry

# cv_bridge for numpy <-> ROS Image
from cv_bridge import CvBridge

# Helper quaternion conversion
import math
import json


class RosSensorOutputs:
    def __init__(self, publishers, bridge, clock, diagnostics, camera_info):
        for name, publisher in publishers.items():
            setattr(self, name, publisher)
        self.bridge = bridge
        self.clock = clock
        self.diagnostics = diagnostics
        self.default_cam_info = camera_info

    def _stamp(self):
        # ROS2 builtin time
        return self.clock.now().to_msg()


    def _publish_camera(self, np_img):
        """
        np_img: H x W x 3 (uint8) or H x W x 4
        Publishes sensor_msgs/Image and CameraInfo.
        """
        try:
            if np_img is None:
                return
            # Ensure uint8 and 3 channels
            if np_img.dtype != np.uint8:
                # attempt normalization/convert
                np_img = np.clip(np_img, 0, 255).astype(np.uint8)

            # if 4 channels, drop alpha
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = np_img[:, :, :3]

            # cv_bridge expects BGR by default. HoloOcean likely gives RGB.
            # We'll convert RGB -> BGR for cv2 encoding then specify rgb8 after swapping back.
            # Simpler: tell bridge encoding='rgb8' and pass numpy RGB directly.
            header = Header()
            header.stamp = self._stamp()
            header.frame_id = 'camera_link'

            img_msg = self.bridge.cv2_to_imgmsg(np_img, encoding='rgb8')
            img_msg.header.stamp = header.stamp
            img_msg.header.frame_id = header.frame_id
            self.camera_pub.publish(img_msg)

            # CameraInfo
            cam_info = CameraInfo()
            cam_info.header = img_msg.header
            cam_info.width = np_img.shape[1]
            cam_info.height = np_img.shape[0]
            cam_info.distortion_model = self.default_cam_info.distortion_model
            cam_info.k = self.default_cam_info.k
            cam_info.d = self.default_cam_info.d
            cam_info.r = self.default_cam_info.r
            cam_info.p = self.default_cam_info.p
            cam_info.binning_x = self.default_cam_info.binning_x
            cam_info.binning_y = self.default_cam_info.binning_y
            self.cam_info_pub.publish(cam_info)
        except Exception as e:
            self.diagnostics.error("Failed to publish camera: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_camera_down(self, np_img):
        """
        np_img: H x W x 3 (uint8) or H x W x 4
        Publishes sensor_msgs/Image and CameraInfo.
        """
        try:
            if np_img is None:
                return
            # Ensure uint8 and 3 channels
            if np_img.dtype != np.uint8:
                # attempt normalization/convert
                np_img = np.clip(np_img, 0, 255).astype(np.uint8)

            # if 4 channels, drop alpha
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = np_img[:, :, :3]

            # cv_bridge expects BGR by default. HoloOcean likely gives RGB.
            # We'll convert RGB -> BGR for cv2 encoding then specify rgb8 after swapping back.
            # Simpler: tell bridge encoding='rgb8' and pass numpy RGB directly.
            header = Header()
            header.stamp = self._stamp()
            header.frame_id = 'camera2_link'

            img_msg = self.bridge.cv2_to_imgmsg(np_img, encoding='rgb8')
            img_msg.header.stamp = header.stamp
            img_msg.header.frame_id = header.frame_id
            self.cameraDown_pub.publish(img_msg)
            # self.diagnostics.info("Published Down Camera Image")
            # self.diagnostics.info(f"Image header: {img_msg.header.frame_id}")

            # CameraInfo
            # cam_info = CameraInfo()
            # cam_info.header = img_msg.header
            # cam_info.width = np_img.shape[1]
            # cam_info.height = np_img.shape[0]
            # cam_info.distortion_model = self.default_cam_info.distortion_model
            # cam_info.k = self.default_cam_info.k
            # cam_info.d = self.default_cam_info.d
            # cam_info.r = self.default_cam_info.r
            # cam_info.p = self.default_cam_info.p
            # cam_info.binning_x = self.default_cam_info.binning_x
            # cam_info.binning_y = self.default_cam_info.binning_y

            # self.cam_info_pub.publish(cam_info)
        except Exception as e:
            self.diagnostics.error("Failed to publish camera: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_depth(self, depth_data):
        """
        depth_data can be:
         - 2D numpy array -> publish as sensor_msgs/Image (32FC1 or 16UC1)
         - scalar (float) -> publish as std_msgs/Float32
        """
        try:
            if depth_data is None:
                return
            if isinstance(depth_data, (float, int, np.floating, np.integer)):
                msg = Float32()
                msg.data = float(depth_data)
                self.depth_scalar_pub.publish(msg)
                return

            depth_np = np.array(depth_data)
            if depth_np.ndim == 2:
                # publish as 32-bit float image
                # ensure float32
                depth_f = depth_np.astype(np.float32)
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                img_msg.header.stamp = self._stamp()
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            elif depth_np.ndim == 3 and depth_np.shape[2] == 1:
                depth_f = depth_np[:, :, 0].astype(np.float32)
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                img_msg.header.stamp = self._stamp()
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            else:
                # fallback: try scalar conversion
                try:
                    val = float(depth_np[0])
                    msg = Float32()
                    msg.data = val
                    self.depth_scalar_pub.publish(msg)
                except Exception:
                    self.diagnostics.warn("Unsupported depth shape: %s" % (str(depth_np.shape),))
        except Exception as e:
            self.diagnostics.error("Failed to publish depth: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_imu(self, imu_data):
        """imu_data expected as dict/array: ax,ay,az; gx,gy,gz; maybe orientation"""
        try:
            if imu_data is None:
                return
            msg = Imu()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'imu_link'
            # try various formats
            if isinstance(imu_data, dict):
                lin_acc = imu_data.get('linear_acceleration') or imu_data.get('acc') or imu_data.get('accelerometer')
                ang_vel = imu_data.get('angular_velocity') or imu_data.get('gyro')
                ori = imu_data.get('orientation')
            else:
                # maybe a numpy array [ax,ay,az, gx,gy,gz] or nested
                try:
                    arr = np.array(imu_data).flatten()
                    if arr.size >= 6:
                        lin_acc = arr[0:3].tolist()
                        ang_vel = arr[3:6].tolist()
                    else:
                        lin_acc = None
                        ang_vel = None
                    ori = None
                except Exception:
                    lin_acc = ang_vel = ori = None

            if lin_acc is not None:
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = [float(x) for x in lin_acc]
            if ang_vel is not None:
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = [float(x) for x in ang_vel]
            if ori is not None:
                # accept list/tuple [x,y,z,w] or dict
                if isinstance(ori, dict):
                    msg.orientation.x = float(ori.get('x', 0.0))
                    msg.orientation.y = float(ori.get('y', 0.0))
                    msg.orientation.z = float(ori.get('z', 0.0))
                    msg.orientation.w = float(ori.get('w', 1.0))
                else:
                    q = list(ori)
                    if len(q) >= 4:
                        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = [float(x) for x in q[:4]]
            self.imu_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish IMU: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_mag(self, mag_data):
        try:
            if mag_data is None:
                return
            msg = MagneticField()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'mag_link'
            # accept sequences
            try:
                v = np.array(mag_data).flatten()
                msg.magnetic_field.x = float(v[0])
                msg.magnetic_field.y = float(v[1])
                msg.magnetic_field.z = float(v[2])
            except Exception:
                # if dict:
                if isinstance(mag_data, dict):
                    msg.magnetic_field.x = float(mag_data.get('x', 0.0))
                    msg.magnetic_field.y = float(mag_data.get('y', 0.0))
                    msg.magnetic_field.z = float(mag_data.get('z', 0.0))
            self.mag_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish magnetometer: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_dvl(self, dvl_data):
        """
        dvl_data: velocity vector or dict with 'vx','vy','vz'
        Publishes geometry_msgs/TwistStamped (linear velocity in m/s)
        """
        try:
            if dvl_data is None:
                return
            msg = TwistStamped()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'dvl_link'
            # try:
            v = np.array(dvl_data)[:3]
            msg.twist.linear.x = float(v[0])
            msg.twist.linear.y = float(v[1]) if v.size > 1 else 0.0
            msg.twist.linear.z = float(v[2]) if v.size > 2 else 0.0
            # except Exception:
            #     if isinstance(dvl_data, dict):
            #         msg.twist.linear.x = float(dvl_data.get('vx', 0.0))
            #         msg.twist.linear.y = float(dvl_data.get('vy', 0.0))
            #         msg.twist.linear.z = float(dvl_data.get('vz', 0.0))
            self.dvl_pub.publish(msg)

            dist = np.average(np.array(dvl_data)[3:])
            self.depth_distance_pub.publish(Float32(data=float(dist)))

        except Exception as e:
            self.diagnostics.error("Failed to publish DVL: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_range_finder(self, range_data):
        """
        range_data: scalar distance value (float)
        Publishes std_msgs/Float32
        """
        try:
            if range_data is None:
                return
            msg = Float32()
            msg.data = float(range_data)
            self.range_finder_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish range finder: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())


    def _publish_location(self, loc_data):
        """
        loc_data expected like [x,y,z] plus maybe orientation or velocity.
        Publishes nav_msgs/Odometry (pose + optionally twist)
        """
        try:
            if loc_data is None:
                return
            msg = Odometry()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            # accept dict or array
            if isinstance(loc_data, dict):
                pos = loc_data.get('position') or loc_data.get('pos') or loc_data.get('location')
                ori = loc_data.get('orientation')
                vel = loc_data.get('velocity')
            else:
                arr = np.array(loc_data)
                # heuristics
                if arr.size >= 3:
                    pos = arr[0:3].tolist()
                    vel = arr[3:6].tolist() if arr.size >= 6 else None
                    ori = None
                else:
                    pos = None
                    vel = None
                    ori = None

            if pos is not None:
                msg.pose.pose.position.x = float(pos[0])
                msg.pose.pose.position.y = float(pos[1])
                msg.pose.pose.position.z = float(pos[2])
            if ori is not None:
                if isinstance(ori, dict):
                    msg.pose.pose.orientation.x = float(ori.get('x', 0.0))
                    msg.pose.pose.orientation.y = float(ori.get('y', 0.0))
                    msg.pose.pose.orientation.z = float(ori.get('z', 0.0))
                    msg.pose.pose.orientation.w = float(ori.get('w', 1.0))
                else:
                    q = list(ori)
                    if len(q) >= 4:
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = [float(x) for x in q[:4]]
            if vel is not None:
                msg.twist.twist.linear.x = float(vel[0]) if len(vel) > 0 else 0.0
                msg.twist.twist.linear.y = float(vel[1]) if len(vel) > 1 else 0.0
                msg.twist.twist.linear.z = float(vel[2]) if len(vel) > 2 else 0.0

            self.odom_pub.publish(msg)
        except Exception as e:
            self.diagnostics.error("Failed to publish location/odom: %s" % str(e))
            self.diagnostics.debug(traceback.format_exc())

````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/adapters/sensor_dispatch.py

SHA-256: `df049626e5afb2b9e9052f967a6a346666930e309c8fe053b27f33447b0ae52b`

````python
"""Sensor decoding registry, separate from simulation scheduling."""
import traceback
import numpy as np

class SensorDispatch:
    def __init__(self, outputs, diagnostics, current_field, agent='auv0'):
        self.outputs = outputs
        self.diagnostics = diagnostics
        self.current_field = current_field
        self.agent = agent
        self.handlers = (self.cameras, self.range_finder, self.dvl, self.imu,
                         self.depth, self.magnetic, self.location)

    def dispatch(self, state, session):
        for handler in self.handlers:
            handler(state, session)

    def cameras(self, state, env):
        try:
            try:
                # Camera: expect HxWxC (RGB)
                cam = state.get("FrontCamera", None) if isinstance(state, dict) else None
                if cam is None:
                    # maybe state itself is an array? try attribute
                    cam = getattr(state, "FrontCamera", None)
                if cam is not None:
                    cam_np = np.array(cam)
                    # If sensor returns HxWx4 RGBA or HxWx3
                    self.outputs._publish_camera(cam_np)
            except Exception:
                self.diagnostics.debug("camera publish failed: %s" % traceback.format_exc())

            try:
                # Camera: expect HxWxC (RGB)
                cam = state.get("DownCamera", None) if isinstance(state, dict) else None
                if cam is None:
                    # maybe state itself is an array? try attribute
                    cam = getattr(state, "DownCamera", None)
                if cam is not None:
                    cam_np_2 = np.array(cam)
                    # If sensor returns HxWx4 RGBA or HxWx3
                    self.outputs._publish_camera_down(cam_np_2)
            except Exception:
                self.diagnostics.debug("camera publish failed: %s" % traceback.format_exc())

            # depth = compute_stereo_depth(state.get("DownCamera", None), state.get("FrontCamera", None), focal_length_px=256, baseline_meters=0.5)
            # self.outputs._publish_depth(depth)

        except Exception:
            self.diagnostics.debug("camera publish failed: %s" % traceback.format_exc())


    def range_finder(self, state, env):
        try:
            range_finder = state.get("RangeFinder", None) if isinstance(state, dict) else getattr(state, "RangeFinder", None)
            self.outputs._publish_range_finder(range_finder)
        except Exception:
            self.diagnostics.debug("range finder publish failed: %s" % traceback.format_exc())


    def dvl(self, state, env):
        try:
            dvl = state.get("DVLSensor", None) if isinstance(state, dict) else getattr(state, "DVLSensor", None)
            #print(dvl)
            self.outputs._publish_dvl(dvl)
        except Exception:
            self.diagnostics.debug("dvl failed: %s" % traceback.format_exc())


    def imu(self, state, env):
        try:
            imu = state.get("IMUSensor", None) if isinstance(state, dict) else getattr(state, "IMUSensor", None)
            self.outputs._publish_imu(imu)
        except Exception:
            self.diagnostics.debug("imu failed: %s" % traceback.format_exc())


    def depth(self, state, env):
        try:
            depth = state.get("DepthSensor", None) if isinstance(state, dict) else getattr(state, "DepthSensor", None)
            #print(depth)
            self.outputs._publish_depth(depth)
        except Exception:
            self.diagnostics.debug("depth failed: %s" % traceback.format_exc())


    def magnetic(self, state, env):
        try:
            mag = state.get("MagnetometerSensor", None) if isinstance(state, dict) else getattr(state, "MagnetometerSensor", None)
            #print(mag)
            self.outputs._publish_mag(mag)
        except Exception:
            self.diagnostics.debug("mag failed: %s" % traceback.format_exc())


    def location(self, state, env):
        try:
            loc = state.get("LocationSensor", None) if isinstance(state, dict) else getattr(state, "LocationSensor", None)
            current_velocity = self.current_field(loc)
            env.set_ocean_currents(self.agent, current_velocity)
            self.outputs._publish_location(loc)
        except Exception:
            self.diagnostics.debug("location failed: %s" % traceback.format_exc())
````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/domain/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/domain/simulation.py

SHA-256: `c1ea4004e0378aed8c2632589e36d43a7ed5536dc5d3d550048f49b368e7dacb`

````python
from dataclasses import dataclass
from typing import Protocol, Any, ContextManager, Callable

@dataclass(frozen=True)
class Command:
    values: tuple[float, ...]
    def __post_init__(self):
        object.__setattr__(self, 'values', tuple(self.values))

class SimulationSession(Protocol):
    def step(self, command: Command) -> Any: ...
    def set_ocean_currents(self, agent: str, velocity) -> None: ...

class SessionFactory(Protocol):
    def open(self) -> ContextManager[SimulationSession]: ...

class SensorDispatcher(Protocol):
    def dispatch(self, state, session: SimulationSession) -> None: ...

class CurrentField(Protocol):
    def __call__(self, location): ...
````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/holoocean_node.py

SHA-256: `8f77a6e0b0b81f5f3b2cdc60b15193e3a772be7f561dc346dcf82fa3dda33aef`

````python
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np
import time
import traceback

from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField
from std_msgs.msg import Float32, Header, Float32MultiArray
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion, Pose, Point
from nav_msgs.msg import Odometry

# cv_bridge for numpy <-> ROS Image
from cv_bridge import CvBridge

# Helper quaternion conversion
import math
import json

from .domain.simulation import Command
from .use_cases.current_field import vortex_field
from .use_cases.simulation import SimulationRunner
from .adapters.holoocean import HoloOceanSessions, load_scenario
from .adapters.sensor_dispatch import SensorDispatch
from .adapters.ros_sensors import RosSensorOutputs

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return qx, qy, qz, qw


class HoloOceanROS2Bridge(Node):
    def __init__(self, holoocean_make_fn, step_limit=None):
        """
        holoocean_make_fn: function holoocean.make
        scenario_cfg: config object/str passed to holoocean.make
        command: default command to pass to env.step(command). If None, pass {}.
        step_limit: optional max steps to run (None => run until env finishes or KeyboardInterrupt)
        """
        super().__init__('holocean_ros2_bridge')

        # 1. Declare and get the parameter passed from the launch file
        self.runner = None
        self.declare_parameter('holoocean_config_path', '')
        config_path = self.get_parameter('holoocean_config_path').value

        # 2. Read the JSON file into a dictionary
        if not config_path:
            self.get_logger().error("No holoocean config path provided!")
            return

        scenario_cfg = load_scenario(config_path)

        self.get_logger().info(f"Loaded HoloOcean Config for world: {scenario_cfg['world']}")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # qos = qos_profile_sensor_data

        # Publishers
        self.camera_pub = self.create_publisher(Image, 'holocean/camera/image_raw', qos)
        self.cameraDown_pub = self.create_publisher(Image, 'holocean/cameraDown/image_raw', qos)
        self.cam_info_pub = self.create_publisher(CameraInfo, 'holocean/camera/camera_info', qos)
        self.depth_image_pub = self.create_publisher(Image, 'holocean/depth/image_raw', qos)
        self.depth_scalar_pub = self.create_publisher(Float32, 'holocean/depth/scalar', qos)
        self.range_finder_pub = self.create_publisher(Float32, 'holocean/distance/scalar', qos)
        self.depth_distance_pub = self.create_publisher(Float32, 'holocean/depth/distance', qos)
        self.imu_pub = self.create_publisher(Imu, 'holocean/imu', qos)
        self.mag_pub = self.create_publisher(MagneticField, 'holocean/mag', qos)
        self.dvl_pub = self.create_publisher(TwistStamped, 'holocean/dvl', qos)
        self.odom_pub = self.create_publisher(Odometry, 'holocean/odom', qos)

        # Bridge and config
        self.bridge = CvBridge()
        self.holoocean_make_fn = holoocean_make_fn
        self.scenario_cfg = scenario_cfg
        self.step_limit = step_limit

        # CameraInfo default (override if you have calibration YAML)
        self.default_cam_info = CameraInfo()
        # fill some reasonable defaults (width/height will be set per-image)
        self.default_cam_info.distortion_model = 'plumb_bob'
        self.default_cam_info.k = [0.0]*9
        self.default_cam_info.d = [0.0]*5
        self.default_cam_info.r = [0.0]*9
        self.default_cam_info.p = [0.0]*12
        self.default_cam_info.binning_x = 0
        self.default_cam_info.binning_y = 0



        self.command = [10,10,10,10,0,0,0,0]  # default command
        self.sub_cmd = self.create_subscription(
            Float32MultiArray,
            '/holo/cmd',
            self.cmd_callback,
            10
        )


        outputs = RosSensorOutputs(
            {name: getattr(self, name) for name in (
                'camera_pub', 'cameraDown_pub', 'cam_info_pub', 'depth_image_pub',
                'depth_scalar_pub', 'range_finder_pub', 'depth_distance_pub',
                'imu_pub', 'mag_pub', 'dvl_pub', 'odom_pub')},
            self.bridge, self.get_clock(), self.get_logger(), self.default_cam_info)
        self.runner = SimulationRunner(
            HoloOceanSessions(holoocean_make_fn, scenario_cfg),
            SensorDispatch(outputs, self.get_logger(), vortex_field, agent='auv0'),
            lambda: Command(tuple(self.command)),
            lambda: rclpy.spin_once(self, timeout_sec=0.0), self.get_logger(), step_limit)

    def run(self):
        if self.runner is None:
            return
        self.get_logger().info('Starting HoloOcean bridge loop...')
        try:
            self.runner.run()
        except Exception as e:
            self.get_logger().error('Exception in holoocean loop: %s' % str(e))
            self.get_logger().error(traceback.format_exc())
            raise

    def cmd_callback(self, msg):
        self.command = list(msg.data)
        print("Received command:", self.command)



def main(args=None):
    import holoocean  # ensure holoocean is available
    rclpy.init(args=args)

    bridge_node = HoloOceanROS2Bridge(holoocean.make,  step_limit=None)

    bridge_node.run()

    try:
        # Keep the node alive after the explicit simulation run returns.
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info("Shutting down via KeyboardInterrupt")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/use_cases/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/use_cases/current_field.py

SHA-256: `7b8148b3627f4411c0ef949ad3c484712d0da003c64c3116f448d724ca48c3e4`

````python
import numpy as np

def vortex_field(location):
    x, y, z = location
    if z > 0:
        return [0, 0, 0]
    strength = 5.0
    r_squared = x**2 + y**2 + 1e-5  # avoid divide by zero
    dx = -y / r_squared * strength
    dy = x / r_squared * strength
    dz = 0.2 * np.cos(0.1 * r_squared)
    return [3*dx, 3*dy, 3*dz]
````

### NBV_paper_ros2/src/auv_sim_bridge/auv_sim_bridge/use_cases/simulation.py

SHA-256: `1daca219aac5123b03db2a2cb6fe3e2510edf94ca1df8d336bcd5a92a4a4225b`

````python
"""Simulation stepping policy; no simulator, filesystem or ROS imports."""
import traceback
from auv_contracts.ports import Diagnostics
from ..domain.simulation import SessionFactory, SensorDispatcher

class SimulationRunner:
    def __init__(self, sessions: SessionFactory, sensors: SensorDispatcher,
                 commands, pump, diagnostics: Diagnostics, step_limit=None):
        self.sessions = sessions
        self.sensors = sensors
        self.commands = commands
        self.pump = pump
        self.diagnostics = diagnostics
        self.step_limit = step_limit
        self._running = True

    def run(self):
        """
        Run HoloOcean loop inside context manager. Blocking until finished or step_limit reached.
        """
        self.diagnostics.info("Entering holoocean.make context...")
        steps = 0
        with self.sessions.open() as env:
            self.diagnostics.info("HoloOcean env created.")
            try:
                while self._running:
                    if self.step_limit is not None and steps >= self.step_limit:
                        self.diagnostics.info("Reached step limit (%d)." % self.step_limit)
                        break
                    state = env.step(self.commands())
                    steps += 1

                    self.sensors.dispatch(state, env)

                    # small sleep is not necessary — env.step controls sim time — but let ROS spin once
                    self.pump()

            except KeyboardInterrupt:
                self.diagnostics.info('KeyboardInterrupt - exiting holoocean loop.')
            except Exception as e:
                self.diagnostics.error("Exception inside HoloOcean main loop: %s" % str(e))
                self.diagnostics.error(traceback.format_exc())
            finally:
                self.diagnostics.info("HoloOcean context exited after %d steps." % steps)

````

### NBV_paper_ros2/src/auv_telemetry/package.xml

SHA-256: `08aec4d91e9194b99fa9dee34f36176985f7c38e4800027aff74bc1693db6c02`

````xml
<?xml version="1.0"?>
<package format="3">
  <name>auv_telemetry</name><version>0.0.0</version>
  <description>File-backed AUV telemetry adapters</description>
  <maintainer email="metinegearal@gmail.com">metin-ege</maintainer>
  <license>TODO: License declaration</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>auv_contracts</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
````

### NBV_paper_ros2/src/auv_telemetry/resource/auv_telemetry

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````text

````

### NBV_paper_ros2/src/auv_telemetry/setup.cfg

SHA-256: `d12a53e369080f86668e2d61997db027e71155accf9ebdbed936373b5ddaddf4`

````ini
[develop]
script_dir=$base/lib/auv_telemetry
[install]
install_scripts=$base/lib/auv_telemetry
````

### NBV_paper_ros2/src/auv_telemetry/setup.py

SHA-256: `9bd2c4228235131e2f5cedf501506da499525e507e8af22b45d5327ddccd44c3`

````python
from setuptools import find_packages, setup
setup(name='auv_telemetry', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/auv_telemetry']),
                  ('share/auv_telemetry', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      description='File-backed AUV telemetry adapters', license='TODO: License declaration')
````

### NBV_paper_ros2/src/auv_telemetry/auv_telemetry/__init__.py

SHA-256: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`

````python

````

### NBV_paper_ros2/src/auv_telemetry/auv_telemetry/file_sinks.py

SHA-256: `62acb3f9bdba6f5a809df0e10e8b5c7807310a10fe240f3f7ac220d972295e13`

````python
"""File-backed telemetry and plots for mid-mission NBV evaluation."""

import csv
import json
import math
from datetime import datetime
from pathlib import Path


class MidrunLogger:
    """Write planner and vehicle state without adding ROS message types."""

    def __init__(self, results_dir='results', prefix='benchmark',
                 pipeline_only=False, trajectory_only=False):
        self.results_dir = Path(results_dir).expanduser()
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.prefix = prefix
        self.pipeline_only = pipeline_only
        self.trajectory_only = trajectory_only
        self.trajectory_path = self.results_dir / f'{prefix}_trajectory.csv'
        self.depth_pose_path = self.results_dir / f'{prefix}_depth_poses.csv'
        self.iteration_path = self.results_dir / f'{prefix}_nbv_iterations.jsonl'
        self.future_path = self.results_dir / f'{prefix}_future_trajectories.csv'
        self.pipeline_path = self.results_dir / f'{prefix}_pipeline_timing.csv'
        self.filter_path = self.results_dir / f'{prefix}_nbv_filters.jsonl'
        self._reset_files()

    def _reset_files(self):
        if self.pipeline_only:
            with self.pipeline_path.open('w', newline='') as stream:
                csv.writer(stream).writerow([
                    'stage', 'iteration', 'frame_id', 'duration_ms',
                    'source_stamp_s', 'details'])
            return
        with self.trajectory_path.open('w', newline='') as stream:
            csv.writer(stream).writerow(
                ['time_s', 'x_m', 'y_m', 'z_m', 'cumulative_distance_m'])
            if self.trajectory_only:
                return
        with self.depth_pose_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'depth_sequence', 'depth_time_s', 'x_m', 'y_m', 'z_m',
                'roll_rad', 'pitch_rad', 'yaw_rad'])
        with self.future_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'iteration', 'waypoint_index', 'x_m', 'y_m', 'z_m', 'yaw_rad'])
        with self.pipeline_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'stage', 'iteration', 'frame_id', 'duration_ms',
                'source_stamp_s', 'details'])
        self.iteration_path.write_text('', encoding='utf-8')
        self.filter_path.write_text('', encoding='utf-8')

    def record_filter(self, stage, input_count, output_count, details,
                      iteration=None, frame_id=None):
        """Persist filter counts and reasons immediately for interrupted runs."""
        event = dict(stage=stage, iteration=iteration, frame_id=frame_id,
                     input_count=int(input_count), output_count=int(output_count),
                     details=self._json_safe(details))
        with self.filter_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(event, allow_nan=False) + '\n')

    @staticmethod
    def _json_safe(value):
        """Retain nonfinite diagnostic values without producing invalid JSON."""
        if isinstance(value, dict):
            return {str(key): MidrunLogger._json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [MidrunLogger._json_safe(item) for item in value]
        if hasattr(value, 'tolist'):
            return MidrunLogger._json_safe(value.tolist())
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        return value

    @staticmethod
    def _vector(value):
        return [float(component) for component in value]

    def record_trajectory(self, time_s, position, cumulative_distance):
        with self.trajectory_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                f'{time_s:.6f}', *[f'{value:.6f}' for value in self._vector(position)],
                f'{cumulative_distance:.6f}'])

    def record_stage(self, stage, duration_ms, iteration=None, frame_id=None,
                     source_stamp_s=None, details=''):
        with self.pipeline_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                stage,
                '' if iteration is None else iteration,
                '' if frame_id is None else frame_id,
                f'{float(duration_ms):.6f}',
                '' if source_stamp_s is None else f'{float(source_stamp_s):.9f}',
                details,
            ])

    def record_depth_pose(self, sequence, depth_time_s, position, roll, pitch, yaw):
        with self.depth_pose_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                sequence, f'{depth_time_s:.6f}',
                *[f'{value:.6f}' for value in self._vector(position)],
                f'{roll:.6f}', f'{pitch:.6f}', f'{yaw:.6f}'])

    def record_iteration(self, iteration, elapsed_ms, vehicle_position,
                         candidates, trajectory, published, route_blocked,
                         selected_targets=()):
        record = {
            'iteration': int(iteration),
            'elapsed_ms': float(elapsed_ms),
            'vehicle_position': self._vector(vehicle_position),
            'published': bool(published),
            'route_blocked': bool(route_blocked),
            'candidates': [],
        }
        selected_targets = [self._vector(target) for target in selected_targets]
        for index, candidate in enumerate(candidates):
            candidate_position = self._vector(candidate['position'])
            candidate_target = self._vector(candidate['target'])
            selected = any(
                sum((left - right) ** 2 for left, right in zip(
                    candidate_target, target)) < 1e-8
                for target in selected_targets)
            record['candidates'].append({
                'rank': index,
                'cluster_id': int(candidate.get('cluster_id', -1)),
                'position': candidate_position,
                'target': candidate_target,
                'utility': float(candidate.get('utility', 0.0)),
                'raw_gain': float(candidate.get('raw_gain', 0.0)),
                'raw_cost': float(candidate.get('raw_cost', 0.0)),
                'trajectory_id': iteration + 1 if published and selected else None,
            })
        with self.iteration_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record) + '\n')

        if len(trajectory) == 0:
            return
        with self.future_path.open('a', newline='') as stream:
            writer = csv.writer(stream)
            for waypoint_index, waypoint in enumerate(trajectory):
                values = list(waypoint)
                writer.writerow([
                    iteration, waypoint_index,
                    *[f'{float(value):.6f}' for value in values[:3]],
                    f'{float(values[3]) if len(values) > 3 else 0.0:.6f}',
                ])


class PipelineFileSink:
    """Perception depends only on the timing role."""
    def __init__(self, results_dir='results', prefix='auv_stereo_depth'):
        self._writer = MidrunLogger(results_dir, prefix, pipeline_only=True)
    def record_stage(self, *args, **kwargs):
        return self._writer.record_stage(*args, **kwargs)

class PlannerFileSink:
    def __init__(self, results_dir='results', prefix='planner'):
        self._writer = MidrunLogger(results_dir, prefix)
    @property
    def description(self):
        return str(self._writer.filter_path)
    def record_stage(self, *args, **kwargs):
        return self._writer.record_stage(*args, **kwargs)
    def record_filter(self, *args, **kwargs):
        return self._writer.record_filter(*args, **kwargs)
    def record_depth_pose(self, *args, **kwargs):
        return self._writer.record_depth_pose(*args, **kwargs)
    def record_iteration(self, *args, **kwargs):
        return self._writer.record_iteration(*args, **kwargs)
````
