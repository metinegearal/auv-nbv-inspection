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
