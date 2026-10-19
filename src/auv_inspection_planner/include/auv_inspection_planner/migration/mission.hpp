#pragma once
#include "auv_inspection_planner/migration/core.hpp"
#include <mutex>
#include <random>

namespace inspection::migration {
struct OccupancyMap {
  virtual ~OccupancyMap() = default;
  virtual void insert(const Points &, const V3 &, double) = 0;
  virtual Points occupied() const = 0;
  virtual Voxels collision_voxels(double resolution) const {
    Voxels keys;
    for (const auto &p : occupied())
      keys.insert(voxel(p, resolution));
    return keys;
  }
  virtual size_t leaves() const = 0;
  virtual void write(const std::string &) = 0;
};
struct DepthFrame {
  int rows = 0, cols = 0, channels = 1;
  bool uint16 = false;
  std::vector<double> values;
};
using Events = std::function<void(const Json &)>;
struct MissionPorts {
  std::function<double()> clock = monotonic;
  Events telemetry = [](const Json &) {};
  Diagnostic diagnostics = [](const std::string &, const std::string &) {};
  std::function<void(const std::vector<float> &)> publish =
      [](const std::vector<float> &) {};
  std::function<void(bool)> complete = [](bool) {};
  std::function<void(const std::string &)> evaluation =
      [](const std::string &) {};
};
class Mission {
public:
  Mission(Json config, OccupancyMap &, MissionPorts = {});
  void odometry(const Pose &, const Eigen::Quaterniond &);
  void magnetometer(const V3 &);
  void depth(const DepthFrame &, double);
  void rgb() { rgb_received = true; }
  void request(bool);
  void tick();
  void close();
  Points depth_points(const DepthFrame &);
  bool plan_and_publish(std::vector<Candidate>, const Voxels &, const Voxels * = nullptr,
                        std::vector<Candidate> * = nullptr);
  bool publish_trajectory(const Trajectory &, const Voxels &, bool = false);
  bool active_safe(const Voxels &);
  bool stalled(const V3 &, const Trajectory &);
  void cancel(const Voxels * = nullptr, const Voxels * = nullptr);
  bool recover_clearance(const Voxels &, const Voxels &);
  void update_inspection_progress(const Voxels &);
  bool target_observed(const V3 &) const;
  void record_inspection_attempt(const V3 &, const std::string &);
  void blacklist_exhausted_target(const V3 &);
  void blacklist_collision_target(const V3 &);
  double blacklist_radius(const V3 &) const;
  double retry_radius() const;
  bool collision_blacklisted(const V3 &) const;
  void expire_blacklist();
  bool blacklist_failed();
  Json snapshot();
  void seed(uint32_t seed) { rng.seed(seed); }
  // Explicit state exposed for deterministic replay and differential fixtures.
  Json config, nbv_parameters;
  V3 position = V3::Zero(), latched_position = V3::Zero();
  double roll = 0, pitch = 0, yaw = 0, latched_roll = 0, latched_pitch = 0,
         latched_yaw = 0;
  std::deque<Pose> odom_history;
  bool odom_received = false, rgb_received = false, started_mapping = false,
       finished_mapping = false, force_replan = false;
  int depth_sequence = 0, processed_depth_sequence = 0, iteration = 0,
      failed_plan_count = 0;
  const V3 camera_offset{.35, .1125 + .275, -.5};
  Trajectory active_trajectory;
  Points selected_targets, visited_targets, blacklisted_targets;
  std::map<std::array<double, 3>, double> blacklist_expiries;
  std::map<std::array<double, 3>, double> blacklist_radii;
  std::map<std::array<double, 3>, double> collision_blacklist_radii;
  std::vector<std::pair<V3, double>> observation_history;
  std::optional<V3> failed_target, progress_position;
  std::optional<double> progress_time;
  std::string plan_failure, empty_reason, active_failure;
  Seabed seabed;
  NBVCache nbv_cache;
  bool recovering_clearance = false;
  double recovery_retry_time = 0;
  Voxels recovery_obstacles;
  struct InspectionAttempt {
    V3 target;
    int count = 0;
    double started = 0;
  };
  struct InspectionGoal { V3 position, target; };
  std::optional<InspectionGoal> inspection_goal;
  std::vector<InspectionAttempt> inspection_attempts;
  bool depth_has_observations = false;
  std::function<std::vector<Candidate>(
      const Points &, const V3 &, const V3 &,
      const std::vector<std::pair<V3, double>> &, Json, Timing, Diagnostic)>
      calculate;

private:
  struct State {
    V3 position;
    double yaw;
    std::deque<Pose> history;
  };
  State motion_snapshot();
  void accumulate();
  void filter(const std::string &, int, int, Json);
  void stage(const std::string &, double, bool = false,
             const std::string & = "");
  MotionWeights weights() const;
  double d(const char *k) const { return config.at(k).get<double>(); }
  int i(const char *k) const { return config.at(k).get<int>(); }
  bool b(const char *k) const { return config.at(k).get<bool>(); }
  OccupancyMap &map;
  MissionPorts ports;
  std::mutex state_mutex;
  std::optional<DepthFrame> depth_frame;
  V3 sensor_origin = V3::Zero();
  std::mt19937 rng{std::random_device{}()};
};
Json json_safe(Json);
} // namespace inspection::migration
