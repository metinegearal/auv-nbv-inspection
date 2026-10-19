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
