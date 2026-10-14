#include "active_inspection_planner/migration/mission.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace inspection::migration {
namespace {
Json nullable(const std::string &s) {
  return s.empty() ? Json(nullptr) : Json(s);
}
Json nullable(std::optional<double> v) { return v ? Json(*v) : Json(nullptr); }
std::array<double, 3> key(const V3 &v) { return {v.x(), v.y(), v.z()}; }
std::string fixed(double d, int places) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(places) << d;
  return s.str();
}
} // namespace
Json json_safe(Json j) {
  if (j.is_object() || j.is_array()) {
    for (auto &item : j)
      item = json_safe(item);
  } else if (j.is_number_float()) {
    double x = j.get<double>();
    if (!std::isfinite(x))
      return std::isnan(x) ? "nan" : x < 0 ? "-inf" : "inf";
  }
  return j;
}
Mission::Mission(Json supplied, OccupancyMap &occupancy, MissionPorts p)
    : config(defaults()), seabed(supplied.value("voxel_resolution", .1)),
      map(occupancy), ports(std::move(p)) {
  config.update(supplied);
  if (!config["route_max_candidates"].is_number_integer() ||
      i("route_max_candidates") < 1)
    throw std::invalid_argument(
        "route_max_candidates must be a positive integer");
  for (auto k : {"route_stall_timeout_s", "route_stall_min_progress_m"})
    if (!(d(k) > 0 && d(k) < INFINITY))
      throw std::invalid_argument(std::string(k) +
                                  " must be finite and positive");
  nbv_parameters = nbv_defaults();
  for (auto it = nbv_parameters.begin(); it != nbv_parameters.end(); ++it)
    it.value() = config.at("nbv_" + it.key());
  ports.diagnostics(
      "info", "NBV utility node started: active NBV filter is Z > " +
                  std::to_string(nbv_parameters["z_min_target"].get<double>()) +
                  "; z_min_target=" + std::to_string(d("z_min_target")) +
                  " is not used by this filter");
  if (b("nbv_filter_logging"))
    ports.diagnostics("info", "NBV filter diagnostics: " +
                                  config["results_dir"].get<std::string>() +
                                  "/planner_nbv_filters.jsonl");
}
Mission::State Mission::motion_snapshot() {
  std::lock_guard<std::mutex> lock(state_mutex);
  return {position, yaw, odom_history};
}
MotionWeights Mission::weights() const {
  return {d("trajectory_continuity_weight"), d("trajectory_reversal_weight"),
          d("trajectory_history_weight"), d("trajectory_history_radius")};
}
void Mission::filter(const std::string &name, int in, int out, Json details) {
  if (!b("nbv_filter_logging"))
    return;
  if (name == "nbv_empty")
    empty_reason = details.value("reason", "");
  details = json_safe(details);
  ports.diagnostics("info",
                    "[NBV_FILTER iteration=" + std::to_string(iteration) +
                        " frame=" + std::to_string(depth_sequence) + "] " +
                        name + ": " + std::to_string(in) + " -> " +
                        std::to_string(out) + " " + details.dump());
  ports.telemetry({{"type", "filter"},
                   {"stage", name},
                   {"iteration", iteration},
                   {"frame_id", depth_sequence},
                   {"input_count", in},
                   {"output_count", out},
                   {"details", details}});
}
void Mission::stage(const std::string &name, double start, bool frame,
                    const std::string &details) {
  ports.telemetry({{"type", "stage"},
                   {"stage", name},
                   {"duration_ms", (ports.clock() - start) * 1000},
                   {"iteration", iteration},
                   {"frame_id", frame ? Json(depth_sequence) : Json(nullptr)},
                   {"source_stamp_s", nullptr},
                   {"details", details}});
}
void Mission::request(bool value) {
  if (value) {
    ports.diagnostics("info", "Received manual NBV request from PathPlanner. "
                              "Forcing immediate replan!");
    force_replan = true;
  }
}
void Mission::odometry(const Pose &sample, const Eigen::Quaterniond &q) {
  std::lock_guard<std::mutex> lock(state_mutex);
  if (std::abs(sample.position.x()) < .01 &&
      std::abs(sample.position.y()) < .01) {
    filter("odometry_origin_guard", 1, 0,
           {{"position", json(sample.position)}, {"xy_threshold", .01}});
    return;
  }
  position = sample.position;
  roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()),
               1 - 2 * (q.x() * q.x() + q.y() * q.y()));
  pitch = asin(std::clamp(2 * (q.w() * q.y() - q.z() * q.x()), -1., 1.));
  odom_received = true;
  if (!odom_history.empty() && sample.time < odom_history.back().time)
    odom_history.clear();
  if (!odom_history.empty() && sample.time == odom_history.back().time)
    return;
  odom_history.push_back({sample.time, position, roll, pitch, yaw});
  if (odom_history.size() > 3000)
    odom_history.pop_front();
}
void Mission::magnetometer(const V3 &v) {
  std::lock_guard<std::mutex> lock(state_mutex);
  yaw = atan2(v.x(), v.y()) - pi / 2;
  if (yaw < -pi)
    yaw += 2 * pi;
}
void Mission::depth(const DepthFrame &frame, double stamp) {
  depth_frame = frame;
  {
    // The independent odometry callback reads these counters in guard logs.
    std::lock_guard<std::mutex> lock(state_mutex);
    ++depth_sequence;
  }
  auto state = motion_snapshot();
  if (state.history.empty())
    return;
  auto closest = std::min_element(
      state.history.begin(), state.history.end(), [&](auto &a, auto &b) {
        return std::abs(a.time - stamp) < std::abs(b.time - stamp);
      });
  bool matched = std::abs(closest->time - stamp) < .1;
  auto selected = matched ? *closest : state.history.back();
  latched_position = selected.position;
  latched_roll = selected.roll;
  latched_pitch = selected.pitch;
  latched_yaw = selected.yaw;
  if (matched)
    ports.telemetry({{"type", "depth_pose"},
                     {"sequence", depth_sequence},
                     {"depth_time_s", stamp},
                     {"position", json(latched_position)},
                     {"roll", latched_roll},
                     {"pitch", latched_pitch},
                     {"yaw", latched_yaw}});
  else
    ports.diagnostics(
        "warning",
        "Depth map timestamp is too far! Closest odometry is " +
            fixed(std::abs(closest->time - stamp), 3) +
            " seconds away. Using the most recent odometry instead.");
}
Points Mission::depth_points(const DepthFrame &frame) {
  if (frame.channels != 1) {
    filter("depth_image_shape", frame.values.size(), 0,
           {{"shape", {frame.rows, frame.cols, frame.channels}},
            {"required_dimensions", 2}});
    return {};
  }
  double scale = frame.uint16 ? .001 : d("depth_scale");
  int stride = std::max(1, i("depth_stride"));
  int total = 0, nonfinite = 0, below = 0, above = 0;
  double low = INFINITY, high = -INFINITY;
  Points local;
  double focal = (frame.cols / 2.) / tan(pi / 4), cx = frame.cols / 2.,
         cy = frame.rows / 2.;
  for (int row = 0; row < frame.rows; row += stride)
    for (int col = 0; col < frame.cols; col += stride) {
      ++total;
      double distance = frame.values.at(row * frame.cols + col) * scale;
      if (!std::isfinite(distance)) {
        ++nonfinite;
        continue;
      }
      low = std::min(low, distance);
      high = std::max(high, distance);
      if (distance <= d("min_depth"))
        ++below;
      if (distance >= d("max_depth"))
        ++above;
      if (!(distance > d("min_depth") && distance < d("max_depth")))
        continue;
      double x = (col - cx) * distance / focal,
             y = (row - cy) * distance / focal;
      local.emplace_back(distance * cos(pi / 4) - y * sin(pi / 4), -x,
                         -distance * sin(pi / 4) - y * cos(pi / 4));
    }
  filter("depth_range", total, local.size(),
         {{"min_depth", d("min_depth")},
          {"max_depth", d("max_depth")},
          {"comparison", "min_depth < distance < max_depth"},
          {"nonfinite", nonfinite},
          {"at_or_below_min", below},
          {"at_or_above_max", above},
          {"distance_min", low == INFINITY ? Json(nullptr) : Json(low)},
          {"distance_max", high == -INFINITY ? Json(nullptr) : Json(high)},
          {"depth_scale", scale},
          {"stride", stride},
          {"horizontal_fov_deg", 90.},
          {"camera_pitch_deg", 45.},
          {"camera_offset", json(camera_offset)}});
  if (local.empty())
    return {};
  auto rotation = body_rotation(latched_roll, latched_pitch, latched_yaw);
  sensor_origin = latched_position + rotation * camera_offset;
  for (auto &p : local)
    p = rotation * p + sensor_origin;
  return local;
}
void Mission::accumulate() {
  auto points_ = depth_points(*depth_frame);
  if (points_.empty()) {
    ports.diagnostics("warning", "Depth projection yielded 0 valid points (all "
                                 "out of range or invalid).");
    return;
  }
  int before = points_.size();
  if (points_.size() > 4000) {
    std::vector<size_t> indices(points_.size());
    std::iota(indices.begin(), indices.end(), 0);
    // NumPy RandomState.choice(..., replace=False) uses a masked MT19937
    // shuffle.
    for (size_t n = indices.size() - 1; n > 0; --n) {
      uint32_t mask = n;
      mask |= mask >> 1;
      mask |= mask >> 2;
      mask |= mask >> 4;
      mask |= mask >> 8;
      mask |= mask >> 16;
      uint32_t j;
      do {
        j = rng() & mask;
      } while (j > n);
      std::swap(indices[n], indices[j]);
    }
    Points sampled;
    sampled.reserve(4000);
    for (size_t n = 0; n < 4000; ++n)
      sampled.push_back(points_[indices[n]]);
    points_ = std::move(sampled);
  }
  filter("depth_point_cap", before, points_.size(), {{"limit", 4000}});
  map.insert(points_, sensor_origin, d("max_depth"));
  ports.diagnostics("info", "Inserted " + std::to_string(points_.size()) +
                                " points into OctoTree.");
  bool save = true;
  if (!observation_history.empty()) {
    auto &[pos, y] = observation_history.back();
    double delta = latched_yaw - y;
    if ((latched_position - pos).norm() < .5 &&
        std::abs(atan2(sin(delta), cos(delta))) < 15 * pi / 180)
      save = false;
  }
  if (save)
    observation_history.emplace_back(latched_position, latched_yaw);
}
bool Mission::publish_trajectory(const Trajectory &t, const Voxels &occupied) {
  if (t.empty())
    return false;
  Trajectory executed;
  std::vector<float> wire;
  for (auto &v : t) {
    V4 e = v;
    for (int a = 0; a < 4; ++a) {
      float value = float(v[a] + (a < 3 ? camera_offset[a] : 0));
      wire.push_back(value);
      e[a] = double(value) - (a < 3 ? camera_offset[a] : 0);
    }
    executed.push_back(e);
  }
  if (!collision_free(positions(executed), occupied, d("voxel_resolution"))) {
    filter("vehicle_wire_validation", t.size(), 0,
           {{"reason", "executed_vehicle_route_collision"}});
    return false;
  }
  ports.publish(wire);
  active_trajectory = executed;
  started_mapping = true;
  ports.diagnostics("info", "Published " + std::to_string(executed.size()) +
                                " trajectory points");
  return true;
}
bool Mission::plan_and_publish(std::vector<Candidate> candidates_,
                               const Voxels &occupied) {
  plan_failure.clear();
  failed_target.reset();
  std::vector<Candidate> valid;
  for (auto &c : candidates_) {
    bool accepted = c.utility >= d("min_utility_threshold");
    if (accepted)
      valid.push_back(c);
    filter("candidate_minimum_utility", 1, int(accepted),
           {{"cluster_id", c.cluster_id},
            {"target", json(c.target)},
            {"position", json(c.position)},
            {"utility", c.utility},
            {"threshold", d("min_utility_threshold")}});
  }
  filter("minimum_utility", candidates_.size(), valid.size(),
         {{"threshold", d("min_utility_threshold")}});
  if (valid.empty()) {
    plan_failure = candidates_.empty() ? "no_candidates_to_plan"
                                       : "minimum_utility_removed_all";
    return false;
  }
  std::stable_sort(valid.begin(), valid.end(),
                   [](auto &a, auto &b) { return a.utility > b.utility; });
  if (int(valid.size()) > i("route_max_candidates"))
    valid.resize(i("route_max_candidates"));
  auto route_candidates = valid;
  for (auto &c : route_candidates)
    c.position = vehicle_viewpoint(c.position, c.target, camera_offset);
  double start = ports.clock();
  auto state = motion_snapshot();
  V3 vel = velocity(state.history, d("velocity_history_window"));
  double latest = state.history.empty() ? 0 : state.history.back().time;
  Points recent;
  for (auto &h : state.history)
    if (latest - h.time >= 0 &&
        latest - h.time <= d("trajectory_history_window"))
      recent.push_back(h.position);
  if (recent.size() > 64) {
    Points p;
    for (double index : linspace(0, recent.size() - 1, 64))
      p.push_back(recent[int(index)]);
    recent = p;
  }
  bool optimize_ = b("trajectory_optimization_enabled");
  double resolution = d("voxel_resolution"),
         factor = d("collision_check_spacing_factor");
  auto route =
      plan_route(state.position, route_candidates, occupied, resolution,
                 i("astar_max_steps"), optimize_ ? vel : V3::Zero(),
                 optimize_ ? weights().reversal : 0);
  stage("route_search", start);
  bool safe = !route.path.empty() &&
              collision_free(route.path, occupied, resolution, factor);
  filter("route_validation", route.path.size(), safe ? route.path.size() : 0,
         {{"route_points", route.path.size()},
          {"collision_free", safe},
          {"collision_voxels", occupied.size()},
          {"resolution", resolution},
          {"spacing_factor", factor},
          {"reason", route.path.empty() ? route.reason
                     : safe             ? "success"
                                        : "route_collision"}});
  if (!safe) {
    plan_failure = route.path.empty() ? route.reason : "route_collision";
    if (plan_failure == "start_occupied")
      force_replan = true;
    else
      failed_target = valid[0].target;
    ports.diagnostics("warning", "A* route failed. Reason: " + plan_failure);
    return false;
  }
  start = ports.clock();
  struct Option {
    std::string name;
    Points path, targets;
  };
  std::vector<Option> options{{"spline", route.path, route.targets},
                              {"linear_fallback", route.path, route.targets}};
  for (size_t i = 0; i < route.targets.size(); ++i)
    if (route.targets[i] != route.targets[0]) {
      if (i >= 4)
        options.insert(
            options.begin() + 1,
            {"first_target_spline",
             Points(route.path.begin(), route.path.begin() + i),
             Points(route.targets.begin(), route.targets.begin() + i)});
      break;
    }
  std::vector<std::pair<Trajectory, Points>> accepted;
  for (auto &opt : options) {
    auto t = opt.name == "linear_fallback"
                 ? linear_fallback(opt.path, opt.targets, d("waypoint_spacing"))
                 : smooth(opt.path, opt.targets, i("trajectory_points"),
                          d("spline_smoothing"), d("waypoint_spacing"));
    if (t.empty()) {
      filter("trajectory_option", 0, 0,
             {{"option", opt.name},
              {"reason", "empty_trajectory"},
              {"route_points", opt.path.size()},
              {"spline_minimum_points", 4}});
      continue;
    }
    t.front().head<3>() = opt.path.front();
    t.back().head<3>() = opt.path.back();
    if (optimize_) {
      std::vector<double> y{state.yaw};
      for (auto &v : t)
        y.push_back(v.w());
      unwrap(y);
      for (size_t i = 0; i < t.size(); ++i)
        t[i].w() = y[i + 1];
      t[0].w() = state.yaw;
    }
    if (opt.name != "linear_fallback")
      t = resample(t, d("waypoint_spacing"), i("trajectory_points"));
    bool free = collision_free(positions(t), occupied, resolution, factor);
    filter("trajectory_option", t.size(), free ? t.size() : 0,
           {{"option", opt.name},
            {"collision_free", free},
            {"waypoint_spacing", d("waypoint_spacing")},
            {"requested_max_points", i("trajectory_points")},
            {"mandatory_corners_exceed_cap",
             int(t.size()) > i("trajectory_points")}});
    if (free) {
      accepted.emplace_back(t, opt.targets);
      if (!optimize_)
        break;
    }
  }
  if (accepted.empty()) {
    plan_failure = "all_trajectory_options_rejected";
    failed_target = route.targets[0];
    ports.diagnostics(
        "warning",
        "No collision-free spline or linear fallback; route rejected");
    stage("trajectory_generation_and_validation", start, false, "rejected");
    return false;
  }
  size_t winner = 0;
  if (optimize_) {
    double low = INFINITY;
    for (size_t i = 0; i < accepted.size(); ++i) {
      double cost = continuity_cost(accepted[i].first, vel, recent, weights());
      if (cost < low) {
        low = cost;
        winner = i;
      }
    }
  }
  auto t = accepted[winner].first;
  auto targets = accepted[winner].second;
  if (optimize_) {
    double before = continuity_cost(t, vel, recent, weights());
    t = optimize(t, state.position, state.yaw, vel, occupied, resolution,
                 recent, weights(), d("trajectory_max_deviation"),
                 i("trajectory_optimizer_max_iterations"), factor);
    double after = continuity_cost(t, vel, recent, weights());
    ports.diagnostics("info",
                      "Motion optimization: speed=" + fixed(vel.norm(), 2) +
                          " m/s, continuity cost=" + fixed(before, 2) + " -> " +
                          fixed(after, 2));
  }
  stage("trajectory_generation_and_validation", start);
  start = ports.clock();
  auto current = motion_snapshot();
  int before = t.size();
  t = reconnect(t, current.position, current.yaw, occupied, resolution, factor);
  filter("trajectory_reconnection", before, t.size(),
         {{"latest_vehicle_position", json(current.position)},
          {"latest_yaw", current.yaw}});
  if (t.empty()) {
    plan_failure = "trajectory_reconnection_rejected";
    force_replan = true;
    ports.diagnostics("warning", "Vehicle moved during planning; no safe entry "
                                 "to the new route. Requesting a fresh plan.");
  }
  bool success = publish_trajectory(t, occupied);
  stage("trajectory_publish", start, false, success ? "published" : "rejected");
  if (success)
    selected_targets = targets;
  else if (plan_failure.empty()) {
    plan_failure = "trajectory_publish_rejected";
    failed_target = targets[0];
  }
  return success;
}
void Mission::expire_blacklist() {
  double now = ports.clock();
  Points retained;
  for (auto &target : blacklisted_targets) {
    auto k = key(target);
    double expiry = blacklist_expiries.count(k) ? blacklist_expiries[k] : 0;
    if (expiry > now)
      retained.push_back(target);
    else {
      blacklist_expiries.erase(k);
      filter("blacklist_expired", 1, 1, {{"target", json(target)}});
    }
  }
  blacklisted_targets = retained;
}
bool Mission::blacklist_failed() {
  if (!failed_target || d("route_blacklist_cooldown_s") <= 0)
    return false;
  V3 target = *failed_target;
  bool exists = false;
  for (auto &t : blacklisted_targets)
    if ((t - target).norm() < .5) {
      target = t;
      exists = true;
      break;
    }
  if (!exists)
    blacklisted_targets.push_back(target);
  blacklist_expiries[key(target)] =
      ports.clock() + d("route_blacklist_cooldown_s");
  filter("blacklist_addition", 1, 0,
         {{"target", json(target)},
          {"planning_failure", nullable(plan_failure)},
          {"radius", .5},
          {"expires", true},
          {"cooldown_s", d("route_blacklist_cooldown_s")}});
  ports.diagnostics("warning",
                    "Planning rejected (" + plan_failure +
                        "); blacklisting failed target " + json(target).dump() +
                        " for " +
                        std::to_string(d("route_blacklist_cooldown_s")) + "s.");
  force_replan = true;
  return true;
}
bool Mission::stalled(const V3 &p, const Trajectory &remaining) {
  double now = ports.clock();
  if (!progress_position || !progress_time || now < *progress_time ||
      (p - *progress_position).norm() >= d("route_stall_min_progress_m")) {
    progress_position = p;
    progress_time = now;
    return false;
  }
  if ((p - remaining.back().head<3>()).norm() <= d("visit_radius"))
    return false;
  if (now - *progress_time < d("route_stall_timeout_s"))
    return false;
  filter("route_progress_stalled", remaining.size(), 0,
         {{"elapsed_s", now - *progress_time},
          {"displacement_m", (p - *progress_position).norm()},
          {"minimum_progress_m", d("route_stall_min_progress_m")},
          {"vehicle_position", json(p)},
          {"action", "cancel_and_replan"}});
  return true;
}
bool Mission::active_safe(const Voxels &occupied) {
  active_failure.clear();
  if (active_trajectory.empty())
    return true;
  auto state = motion_snapshot();
  auto remaining =
      reconnect(active_trajectory, state.position, state.yaw, occupied,
                d("voxel_resolution"), d("collision_check_spacing_factor"));
  if (remaining.empty()) {
    active_failure = "unsafe_current_route_entry";
    return false;
  }
  if (!collision_free(positions(remaining), occupied, d("voxel_resolution"),
                      d("collision_check_spacing_factor"))) {
    active_failure = "new_obstacle_on_active_route";
    return false;
  }
  if (stalled(state.position, remaining)) {
    active_failure = "active_route_stalled";
    return false;
  }
  return true;
}
void Mission::cancel() {
  int previous = active_trajectory.size();
  ports.publish({});
  active_trajectory.clear();
  progress_position.reset();
  progress_time.reset();
  selected_targets.clear();
  filter("trajectory_cancelled", previous, 0,
         {{"reason", active_failure.empty() ? "new_obstacle_on_active_route"
                                            : active_failure},
          {"downstream_queue_cleared", true}});
}
void Mission::tick() {
  if (finished_mapping)
    return;
  if (!odom_received) {
    ports.diagnostics("warning", "Pipeline blocked: Waiting for Odometry...");
    return;
  }
  if (!rgb_received) {
    ports.diagnostics("warning", "Pipeline blocked: Waiting for RGB camera...");
    return;
  }
  if (!depth_frame) {
    ports.diagnostics("warning",
                      "Pipeline blocked: Waiting for Depth frame...");
    return;
  }
  if (depth_sequence == processed_depth_sequence) {
    ports.diagnostics("warning", "Pipeline blocked: Waiting for NEW Depth "
                                 "frame (Sequence unchanged)...");
    return;
  }
  double started = ports.clock(), start = ports.clock();
  accumulate();
  stage("depth_to_octomap", start, true);
  processed_depth_sequence = depth_sequence;
  start = ports.clock();
  auto occupied = map.occupied();
  if (nbv_parameters["seabed_enabled"].get<bool>()) {
    int changed = seabed.update(occupied);
    filter("seabed_estimate", occupied.size(), seabed.sample_count,
           {{"mean_z", nullable(seabed.mean_z)},
            {"sample_count", seabed.sample_count},
            {"changed_voxels", changed},
            {"source",
             seabed.mean_z ? "occupied_map" : "insufficient_floor_support"}});
  }
  stage("octomap_extract", start, true);
  ports.diagnostics("info", "Iteration " + std::to_string(iteration) +
                                ": OctoMap total size = " +
                                std::to_string(occupied.size()) + " voxels.");
  start = ports.clock();
  Voxels keys;
  for (auto &v : occupied)
    keys.insert(voxel(v, d("voxel_resolution")));
  auto collision =
      inflate(keys, d("collision_radius") + d("collision_tracking_margin"),
              d("voxel_resolution"));
  stage("collision_inflation", start, true);
  bool blocked = !active_safe(collision);
  if (blocked) {
    cancel();
    ports.diagnostics("warning", "Active trajectory cancelled (" +
                                     active_failure + "); forcing replan!");
  }
  bool arrived = false;
  auto state = motion_snapshot();
  if (b("replan_on_target_arrival"))
    for (auto &t : selected_targets)
      if ((state.position - t).norm() < d("arrival_target_radius")) {
        arrived = true;
        ports.diagnostics("info", "📍 Arrived at NBV Target (Distance: " +
                                      fixed((state.position - t).norm(), 2) +
                                      "m). Forcing instant replan!");
        break;
      }
  bool periodic = iteration % std::max(1, i("planning_period")) == 0;
  if (periodic || blocked || force_replan || arrived) {
    empty_reason.clear();
    filter("planning_begin", occupied.size(), occupied.size(),
           {{"periodic", periodic},
            {"route_blocked", blocked},
            {"forced", force_replan},
            {"arrived_at_target", arrived},
            {"collision_voxels", collision.size()},
            {"blacklisted_targets", json(blacklisted_targets)},
            {"minimum_utility", d("min_utility_threshold")}});
    force_replan = false;
    ports.diagnostics("info", "--- PLANNING TRIGGERED (Iteration " +
                                  std::to_string(iteration) + ") ---");
    ports.diagnostics("info", "Saving OctoMap to: " +
                                  config["octomap_path"].get<std::string>());
    start = ports.clock();
    map.write(config["octomap_path"]);
    stage("octomap_save", start);
    state = motion_snapshot();
    double r = state.history.empty() ? 0 : state.history.back().roll,
           p = state.history.empty() ? 0 : state.history.back().pitch;
    V3 camera = state.position + body_rotation(r, p, state.yaw) * camera_offset,
       heading(cos(state.yaw), sin(state.yaw), 0);
    start = ports.clock();
    Json parameters = nbv_parameters;
    parameters["seabed_estimate_z"] = nullable(seabed.mean_z);
    parameters["seabed_sample_count"] = seabed.sample_count;
    std::vector<Candidate> candidates_;
    auto timing = [&](const std::string &name, double duration) {
      ports.telemetry({{"type", "stage"},
                       {"stage", name},
                       {"duration_ms", duration},
                       {"iteration", iteration},
                       {"frame_id", nullptr},
                       {"source_stamp_s", nullptr},
                       {"details", ""}});
    };
    try {
      ports.diagnostics("info", "Calculating NBV waypoints...");
      candidates_ = calculate(occupied, camera, heading, observation_history,
                              parameters, timing, ports.diagnostics);
      ports.diagnostics("info", "Found " + std::to_string(candidates_.size()) +
                                    " valid NBV targets.");
    } catch (const std::invalid_argument &e) {
      empty_reason = "nbv_exception";
      ports.diagnostics("error",
                        std::string("NBV calculation failed: ") + e.what());
      filter("nbv_exception", occupied.size(), 0,
             {{"exception_type", "ValueError"}, {"error", e.what()}});
    } catch (const std::runtime_error &e) {
      empty_reason = "nbv_exception";
      ports.diagnostics("error",
                        std::string("NBV calculation failed: ") + e.what());
      filter("nbv_exception", occupied.size(), 0,
             {{"exception_type", "RuntimeError"}, {"error", e.what()}});
    }
    stage("nbv_generation", start, false,
          "candidates=" + std::to_string(candidates_.size()));
    expire_blacklist();
    std::vector<Candidate> safe;
    for (auto &c : candidates_) {
      Json bans = Json::array();
      for (auto &t : blacklisted_targets)
        if ((c.target - t).norm() < .5)
          bans.push_back(
              {{"target", json(t)},
               {"distance", (c.target - t).norm()},
               {"remaining_s",
                std::max(0., blacklist_expiries.at(key(t)) - ports.clock())}});
      if (bans.empty())
        safe.push_back(c);
      filter("candidate_blacklist", 1, bans.empty() ? 1 : 0,
             {{"cluster_id", c.cluster_id},
              {"target", json(c.target)},
              {"radius", .5},
              {"matching_bans", bans}});
    }
    if (safe.size() < candidates_.size())
      ports.diagnostics("info",
                        "Dropped " +
                            std::to_string(candidates_.size() - safe.size()) +
                            " blacklisted targets.");
    filter("blacklist", candidates_.size(), safe.size(),
           {{"radius", .5},
            {"blacklisted_target_count", blacklisted_targets.size()}});
    bool published = plan_and_publish(safe, collision);
    std::string failure =
        candidates_.empty()
            ? (empty_reason.empty() ? "nbv_generation_returned_no_candidates"
                                    : empty_reason)
        : safe.empty() ? "blacklist_removed_all"
                       : plan_failure;
    filter("planning_result", candidates_.size(), int(published),
           {{"published", published},
            {"reason", published ? "published" : failure},
            {"generated_candidates", candidates_.size()},
            {"after_blacklist", safe.size()},
            {"active_trajectory_points", active_trajectory.size()}});
    if (!published)
      ports.diagnostics("warning",
                        "[NBV_EMPTY] No trajectory published: " + failure);
    if (published)
      failed_plan_count = 0;
    else if (!force_replan) {
      if (!safe.empty()) {
        if (!blacklist_failed()) {
          filter("blacklist_skipped", safe.size(), safe.size(),
                 {{"reason", "no_individual_failed_target"},
                  {"planning_failure", nullable(plan_failure)}});
          ++failed_plan_count;
          ports.diagnostics("warning", "All safe candidates failed (" +
                                           plan_failure + "). Strike " +
                                           std::to_string(failed_plan_count) +
                                           "/3");
        }
      } else {
        ++failed_plan_count;
        ports.diagnostics(
            "warning", "No valid candidates remain (" + failure + "). Strike " +
                           std::to_string(failed_plan_count) + "/3");
      }
    }
    if (failed_plan_count >= 3) {
      filter("mission_completion", failed_plan_count, 0,
             {{"reason", "three_empty_candidate_passes"},
              {"last_empty_reason", failure},
              {"blacklisted_target_count", blacklisted_targets.size()}});
      finished_mapping = true;
      ports.complete(true);
      ports.diagnostics("info", "Flushing final RAM state to OctoMap file...");
      map.write(config["octomap_path"]);
      ports.diagnostics(
          "info", "✅ Mapping finished. Explored all accessible frontiers.");
    }
    double elapsed = (ports.clock() - started) * 1000;
    ports.diagnostics("info", "Planning cycle completed in " +
                                  fixed(elapsed, 2) + " ms.");
    Json cs = Json::array();
    for (auto &c : candidates_)
      cs.push_back(json(c));
    ports.telemetry({{"type", "iteration"},
                     {"iteration", iteration},
                     {"elapsed_ms", elapsed},
                     {"vehicle_position", json(motion_snapshot().position)},
                     {"candidates", cs},
                     {"trajectory", json(active_trajectory)},
                     {"published", published},
                     {"route_blocked", blocked},
                     {"selected_targets", json(selected_targets)}});
    std::ostringstream line;
    line << std::setprecision(17) << "Process Time (ms): " << elapsed
         << ", Total Leaves: " << map.leaves() << '\n';
    ports.evaluation(line.str());
  }
  stage("planning_cycle_total", started, true);
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    ++iteration;
  }
}
void Mission::close() {
  ports.diagnostics("info",
                    "Node shutting down. Saving final OctoMap state...");
  try {
    map.write(config["octomap_path"]);
  } catch (const std::exception &e) {
    ports.diagnostics(
        "error", std::string("Failed to save final map on exit: ") + e.what());
  }
}
Json Mission::snapshot() {
  auto state = motion_snapshot();
  Json observations = Json::array();
  for (auto &[p, y] : observation_history)
    observations.push_back({json(p), y});
  return {{"position", json(state.position)},
          {"yaw", state.yaw},
          {"roll", roll},
          {"pitch", pitch},
          {"latched_position", json(latched_position)},
          {"latched_yaw", latched_yaw},
          {"latched_roll", latched_roll},
          {"latched_pitch", latched_pitch},
          {"depth_sequence", depth_sequence},
          {"processed_depth_sequence", processed_depth_sequence},
          {"iteration", iteration},
          {"failed_plan_count", failed_plan_count},
          {"finished_mapping", finished_mapping},
          {"started_mapping", started_mapping},
          {"force_replan", force_replan},
          {"active_trajectory", json(active_trajectory)},
          {"selected_targets", json(selected_targets)},
          {"visited_targets", json(visited_targets)},
          {"blacklisted_targets", json(blacklisted_targets)},
          {"observation_history", observations},
          {"_plan_failure_reason", nullable(plan_failure)},
          {"_nbv_empty_reason", nullable(empty_reason)},
          {"_active_route_failure_reason", nullable(active_failure)}};
}
} // namespace inspection::migration
