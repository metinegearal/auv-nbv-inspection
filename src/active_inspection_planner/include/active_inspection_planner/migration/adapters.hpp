#pragma once
#include "active_inspection_planner/migration/mission.hpp"
#include <filesystem>
#include <fstream>
#include <octomap/ColorOcTree.h>

namespace inspection::migration {
class Octomap : public OccupancyMap {
public:
  explicit Octomap(double resolution) : tree(resolution) {}
  void insert(const Points &, const V3 &, double) override;
  Points occupied() const override;
  Voxels collision_voxels(double resolution) const override;
  size_t leaves() const override { return tree.getNumLeafNodes(); }
  void write(const std::string &path) override;
  void load(const std::string &path);
  octomap::ColorOcTree tree;
};
class FileSink {
public:
  explicit FileSink(const std::string &directory);
  void record(const Json &);
  void evaluation(const std::string &);

private:
  std::filesystem::path directory;
  std::ofstream evaluation_file;
  std::mutex mutex;
};
} // namespace inspection::migration
