#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <deque>
#include <functional>
#include <json.hpp>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace inspection::migration {
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;
using Points = std::vector<V3>;
using Trajectory = std::vector<V4>;
using Json = nlohmann::ordered_json;
using Key = std::array<int, 3>;
struct KeyHash {
  size_t operator()(const Key &k) const noexcept {
    size_t h = 0;
    for (int x : k)
      h ^= std::hash<int>{}(x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
using Voxels = std::unordered_set<Key, KeyHash>;
constexpr double pi = 3.141592653589793238462643383279502884;
Key voxel(const V3 &, double);
Json json(const V3 &);
Json json(const V4 &);
Json json(const Points &);
Json json(const Trajectory &);
V3 vec3(const Json &);
Points points(const Json &);
Trajectory trajectory(const Json &);
Points positions(const Trajectory &);
void unwrap(std::vector<double> &);
double interp(double x, const std::vector<double> &,
              const std::vector<double> &);
std::vector<double> linspace(double, double, int);
double monotonic();
struct Candidate {
  V3 position, target;
  double utility = 0, raw_gain = 0, raw_cost = 0;
  int cluster_id = -1;
  bool raw_scores_available = false;
};
Json json(const Candidate &);
std::vector<Candidate> candidates(const Json &);
struct SearchResult {
  Points path;
  std::string reason;
  int expansions = 0;
  int directions_checked = 0, clear_directions = 0;
};
SearchResult astar(const V3 &, const V3 &, const Voxels &, double, int = 50000);
SearchResult astar_view_standoff(const V3 &, const V3 &, const V3 &,
                                 const Voxels &, const Voxels &, double,
                                 const V3 &, double, double, int);
struct ViewProbe {
  Points goals;
  int directions_checked = 0, clear_directions = 0;
};
ViewProbe probe_nbv_views(const V3 &, const V3 &, const Voxels &, const Voxels &,
                          double, const V3 &, double, double);
SearchResult astar_view_surround(const V3 &, const V3 &, const V3 &,
                                const Voxels &, const Voxels &, double,
                                const V3 &, double, double, int);
Voxels inflate(const Voxels &, double, double);
// Collision geometry uses whole occupied cells, not extracted point centers.
Voxels inflate_clearance(const Voxels &, double, double);
double obstacle_clearance(const V3 &, const Voxels &, double);
bool clearance_free(const Points &, const Voxels &, double, double);
bool retreat_safe(const Points &, const Voxels &, double, double);
Points clearance_retreat(const V3 &, const Voxels &, const Voxels &, double,
                         double, double);
bool collision_free(const Points &, const Voxels &, double, double = 0.5);
Trajectory reconnect(const Trajectory &, const V3 &, double, const Voxels &,
                     double, double = 0.5);
struct Route {
  Points path, targets;
  std::string reason;
};
Route plan_route(const V3 &, const std::vector<Candidate> &, const Voxels &,
                 double, int = 50000, V3 = V3::Zero(), double = 2.0,
                 const std::function<SearchResult(const V3 &, const Candidate &)> & = {},
                 bool = false);
Trajectory smooth(const Points &, const Points &, int = 200, double = 2.0,
                  double = 0.0);
Trajectory resample(const Trajectory &, double, std::optional<int> = {});
Trajectory linear_fallback(const Points &, const Points &, double);
Eigen::Matrix3d body_rotation(double, double, double);
V3 vehicle_viewpoint(const V3 &, const V3 &, const V3 &);
struct Pose {
  double time;
  V3 position;
  double roll = 0, pitch = 0, yaw = 0;
};
V3 velocity(const std::deque<Pose> &, double = 1.0);
struct MotionWeights {
  double continuity = 8, reversal = 4, history = 2, radius = 0.5;
};
double continuity_cost(const Trajectory &, const V3 &, const Points & = {},
                       MotionWeights = {});
Trajectory optimize(const Trajectory &, const V3 &, double, const V3 &,
                    const Voxels &, double, const Points & = {},
                    MotionWeights = {}, double = 0.5, int = 40, double = 0.5);
class Seabed {
public:
  explicit Seabed(double resolution = 0.2);
  int update(const Points &);
  std::optional<double> mean_z;
  int sample_count = 0;

private:
  void adjust(const Key &, double, int);
  double resolution_;
  std::map<Key, double> voxels_;
  std::map<int, int> counts_;
  std::map<int, double> sums_;
  std::map<int, std::map<std::array<int, 2>, int>> xy_;
};
struct Cloud {
  Points points, normals;
};
struct Cluster {
  int id;
  V3 centroid, normal;
  Points points, normals;
  std::vector<int> indices;
};
struct CachedCluster {
  Cluster geometry;
  std::vector<double> complexities;
};
// Per-mission geometry; distant clusters keep their last local observation.
struct NBVCache {
  std::vector<CachedCluster> clusters;
  int next_cluster_id = 0;
};
using Timing = std::function<void(const std::string &, double)>;
using Diagnostic =
    std::function<void(const std::string &, const std::string &)>;
Json defaults();
Json nbv_defaults();
std::optional<Cloud> prepare_cloud(const Points &, const V3 &, const Json &,
                                   Timing = {});
std::vector<double> pca(const Points &, int = 20,
                        const std::vector<int> * = nullptr);
std::vector<int> frontiers(const Cloud &, int = 40, double = 130, double = 0.5,
                           int = 3, const std::vector<int> * = nullptr);
std::vector<Cluster> cluster_frontiers(const Cloud &, const std::vector<int> &,
                                       double = 1, double = 45, int = 5);
V3 view_axis(const V3 &, const V3 &, const Points &, double = 1.5, int = 36);
Points cone(const V3 &, const V3 &, double = 1.5,
            std::vector<double> = {15, 30}, int = 8);
double seabed_weight(double, std::optional<double>, double = 0.5,
                     double = 0.95);
std::vector<Candidate> nbv(const Points &, const V3 &, const V3 &,
                           const std::vector<std::pair<V3, double>> & = {},
                           Json = Json::object(), Timing = {}, Diagnostic = {},
                           NBVCache * = nullptr);
} // namespace inspection::migration
