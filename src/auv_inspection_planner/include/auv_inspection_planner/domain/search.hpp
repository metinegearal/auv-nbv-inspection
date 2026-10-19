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
