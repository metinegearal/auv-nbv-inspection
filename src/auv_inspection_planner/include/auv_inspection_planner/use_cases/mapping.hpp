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
