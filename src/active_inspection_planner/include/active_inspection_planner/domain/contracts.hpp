#pragma once
#include <Eigen/Dense>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cstdio>
#include "active_inspection_planner/domain/search.hpp"
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
