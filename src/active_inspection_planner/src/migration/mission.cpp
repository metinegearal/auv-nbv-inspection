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
  calculate = [this](const auto &raw, const auto &pos, const auto &heading,
                     const auto &history, auto parameters, auto timing,
                     auto diagnostic) {
    return nbv(raw, pos, heading, history, std::move(parameters), timing,
               diagnostic, &nbv_cache);
  };
  config.update(supplied);
  for (auto k : {"route_blacklist_radius_m", "route_collision_blacklist_radius_m"})
    if (!config[k].is_number() || !(d(k) > 0 && std::isfinite(d(k))))
      throw std::invalid_argument(std::string(k) + " must be finite and positive");
  if (!(d("route_goal_max_extension_m") >= 0 &&
        std::isfinite(d("route_goal_max_extension_m"))))
    throw std::invalid_argument("route_goal_max_extension_m must be finite and nonnegative");
  if (!config["route_target_max_attempts"].is_number_integer() ||
      i("route_target_max_attempts") < 1 ||
      !(d("route_target_timeout_s") > 0 && std::isfinite(d("route_target_timeout_s"))))
    throw std::invalid_argument("target attempt limit and timeout must be positive");
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
  depth_has_observations = false;
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
  depth_has_observations = true;
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
bool Mission::publish_trajectory(const Trajectory &t, const Voxels &occupied, bool omit_start) {
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
  bool safe = recovering_clearance
                  ? retreat_safe(positions(executed), recovery_obstacles,
                                 d("voxel_resolution"), d("collision_radius"))
                  : collision_free(positions(executed), occupied, d("voxel_resolution"));
  if (!safe) {
    filter("vehicle_wire_validation", t.size(), 0,
           {{"reason", "executed_vehicle_route_collision"}});
    return false;
  }
  // The first point is a validation anchor at the current pose. Sending it
  // after a queue clear can leave the controller holding an already-reported
  // waypoint instead of advancing to the actual destination.
  if (omit_start && wire.size() > 4)
    wire.erase(wire.begin(), wire.begin() + 4);
  ports.publish(wire);
  active_trajectory = executed;
  started_mapping = true;
  ports.diagnostics("info", "Published " + std::to_string(wire.size() / 4) +
                                " trajectory points");
  return true;
}
bool Mission::plan_and_publish(std::vector<Candidate> candidates_,
                               const Voxels &occupied, const Voxels *raw,
                               std::vector<Candidate> *evaluated) {
  plan_failure.clear();
  failed_target.reset();
  std::vector<Candidate> valid;
  bool observed_rejected = false, budget_rejected = false, collision_rejected = false;
  int normalization_pass = 0;
  if (raw)
    expire_blacklist();
  auto prepare = [&]() {
    std::vector<Candidate> eligible;
    for (auto c : candidates_) {
      if (raw) {
        bool observed = target_observed(c.target), exhausted = false;
        std::optional<V3> exhausted_center;
        for (const auto &attempt : inspection_attempts)
          if ((attempt.target - c.target).norm() < retry_radius() &&
              attempt.count >= i("route_target_max_attempts") &&
              (!inspection_goal || (inspection_goal->target - c.target).norm() >= retry_radius())) {
            exhausted = true;
            if (!exhausted_center)
              exhausted_center = attempt.target;
          }
        bool banned = std::any_of(blacklisted_targets.begin(), blacklisted_targets.end(),
            [&](const auto &t) { return (c.target - t).norm() <= blacklist_radius(t) + 1e-9; });
        if (observed || exhausted || banned) {
          observed_rejected |= observed;
          budget_rejected |= exhausted;
          collision_rejected |= banned;
          Json suppression = {{"target", json(c.target)},
                  {"reason", observed ? "observed_at_resolved_viewpoint"
                             : exhausted ? "attempt_budget_exhausted"
                             : collision_blacklisted(c.target) ? "collision_blacklist" : "blacklist"}};
          if (!observed && exhausted_center) {
            suppression["exclusion_center"] = json(*exhausted_center);
            suppression["radius"] = retry_radius();
          }
          filter("inspection_target_suppressed", 1, 0, suppression);
          continue;
        }
        if (c.raw_scores_available &&
            (!std::isfinite(c.raw_gain) || !std::isfinite(c.raw_cost))) {
          filter("candidate_score_validity", 1, 0,
                 {{"target", json(c.target)}, {"reason", "nonfinite_raw_score"}});
          continue;
        }
      }
      eligible.push_back(c);
    }
    if (raw) {
      double ming = INFINITY, maxg = -INFINITY, minc = INFINITY, maxc = -INFINITY;
      int count = 0;
      for (const auto &c : eligible)
        if (c.raw_scores_available) {
          ming = std::min(ming, c.raw_gain);
          maxg = std::max(maxg, c.raw_gain);
          minc = std::min(minc, c.raw_cost);
          maxc = std::max(maxc, c.raw_cost);
          ++count;
        }
      for (auto &c : eligible)
        if (c.raw_scores_available) {
          // A positive equal-gain pool (including one remaining view) still
          // contains information. Equal costs carry no relative cost penalty.
          double ng = maxg - ming < 1e-6 ? double(c.raw_gain > 0)
                                        : (c.raw_gain - ming) / (maxg - ming);
          double nc = maxc - minc < 1e-6 ? 0 : (c.raw_cost - minc) / (maxc - minc);
          c.utility = nbv_parameters["utility_gain_weight"].get<double>() * ng -
                      nbv_parameters["utility_cost_weight"].get<double>() * nc;
        }
      if (std::any_of(candidates_.begin(), candidates_.end(),
                      [](const auto &c) { return c.raw_scores_available; }))
        filter("utility_normalization", candidates_.size(), eligible.size(),
             {{"pass", normalization_pass++}, {"raw_candidates", count},
              {"gain_min", count ? Json(ming) : Json(nullptr)},
              {"gain_max", count ? Json(maxg) : Json(nullptr)},
              {"cost_min", count ? Json(minc) : Json(nullptr)},
              {"cost_max", count ? Json(maxc) : Json(nullptr)},
              {"threshold", d("min_utility_threshold")}});
    }
    if (raw)
      std::stable_sort(eligible.begin(), eligible.end(),
                       [](const auto &a, const auto &b) { return a.utility > b.utility; });
    std::map<int, int> per_cluster;
    std::vector<Candidate> shortlist;
    for (const auto &c : eligible) {
      if (raw && c.raw_scores_available && c.cluster_id >= 0 && ++per_cluster[c.cluster_id] > 3)
        continue;
      shortlist.push_back(c);
    }
    if (evaluated)
      *evaluated = shortlist;
    valid.clear();
    for (const auto &c : shortlist) {
      bool accepted = c.utility >= d("min_utility_threshold");
      if (accepted)
        valid.push_back(c);
      filter("candidate_minimum_utility", 1, int(accepted),
             {{"cluster_id", c.cluster_id}, {"target", json(c.target)},
              {"position", json(c.position)}, {"utility", c.utility},
              {"threshold", d("min_utility_threshold")}});
    }
    filter("minimum_utility", shortlist.size(), valid.size(),
           {{"threshold", d("min_utility_threshold")}});
    if (valid.empty()) {
      plan_failure = budget_rejected ? "target_attempts_exhausted"
                     : collision_rejected ? "collision_blacklist_removed_all"
                     : observed_rejected ? "targets_already_observed"
                     : candidates_.empty() ? "no_candidates_to_plan"
                                         : "minimum_utility_removed_all";
      return false;
    }
    if (!raw)
      std::stable_sort(valid.begin(), valid.end(),
                       [](const auto &a, const auto &b) { return a.utility > b.utility; });
    return true;
  };
  if (!prepare())
    return false;
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
  std::function<SearchResult(const V3 &, const Candidate &)> goal_fallback;
  std::vector<std::pair<V3, bool>> checked_targets;
  std::vector<Candidate> searched_views;
  int checks_remaining = i("route_max_candidates");
  if (raw) {
    goal_fallback = [&](const V3 &from, const Candidate &candidate) {
      // A preceding candidate can blacklist this region during the same pass.
      if (collision_blacklisted(candidate.target))
        return SearchResult{{}, "collision_blacklist_removed_all", 0};
      searched_views.push_back(candidate);
      --checks_remaining;
      auto result = astar_view_surround(
          from, candidate.position, candidate.target, occupied, *raw, resolution,
          camera_offset, d("max_depth"), d("route_goal_max_extension_m"),
          i("astar_max_steps"));
      if (result.directions_checked) {
        filter("nbv_collision_rays", result.directions_checked, result.clear_directions,
               {{"target", json(candidate.target)},
                {"directions_checked", result.directions_checked},
                {"clear_directions", result.clear_directions},
                {"reason", result.reason}, {"blacklist_radius_m", d("route_collision_blacklist_radius_m")}});
        if (result.reason == "all_nbv_rays_collide") {
          blacklist_collision_target(candidate.target);
          // A route can remain physically clear while every camera view of
          // its target becomes occluded. Do not keep executing that old route.
          if (inspection_goal && collision_blacklisted(inspection_goal->target)) {
            active_failure = "inspection_target_all_rays_blocked";
            cancel();
          }
        }
      }
      auto checked = std::find_if(checked_targets.begin(), checked_targets.end(),
          [&](const auto &entry) { return (entry.first - candidate.target).norm() < retry_radius(); });
      if (checked == checked_targets.end())
        checked_targets.emplace_back(candidate.target, !result.path.empty());
      else
        checked->second |= !result.path.empty();
      if (result.reason != "success")
        filter("nbv_goal_standoff", 1, result.path.empty() ? 0 : 1,
             {{"target", json(candidate.target)},
              {"original_vehicle_goal", json(candidate.position)},
              {"resolved_vehicle_goal", result.path.empty() ? Json(nullptr) : json(result.path.back())},
              {"max_extension_m", d("route_goal_max_extension_m")},
              {"reason", result.reason}, {"search_expansions", result.expansions}});
      if (result.reason == "all_nbv_rays_collide")
        return SearchResult{{}, "candidate_pool_changed", result.expansions};
      return result;
    };
  }
  Route route;
  while (true) {
    auto route_candidates = valid;
    route_candidates.erase(std::remove_if(route_candidates.begin(), route_candidates.end(),
        [&](const auto &c) {
          V3 goal = vehicle_viewpoint(c.position, c.target, camera_offset);
          return std::any_of(searched_views.begin(), searched_views.end(), [&](const auto &v) {
            return (c.target - v.target).norm() < 1e-8 && (goal - v.position).norm() < 1e-8;
          });
        }), route_candidates.end());
    if (int(route_candidates.size()) > checks_remaining)
      route_candidates.resize(checks_remaining);
    if (route_candidates.empty()) {
      route.reason = "candidate_search_budget_exhausted";
      break;
    }
    for (auto &c : route_candidates)
      c.position = vehicle_viewpoint(c.position, c.target, camera_offset);
    route = plan_route(state.position, route_candidates, occupied, resolution,
                 i("astar_max_steps"), optimize_ ? vel : V3::Zero(),
                 optimize_ ? weights().reversal : 0, goal_fallback, raw != nullptr);
    if (!raw || route.reason != "candidate_pool_changed")
      break;
    // A new blacklist changes both extrema and candidate ranks. Rebuild from
    // the full raw pool, including views previously outside the top three.
    if (!prepare()) {
      route.reason = plan_failure;
      break;
    }
  }
  for (const auto &[target, succeeded] : checked_targets)
    if (!succeeded && (!inspection_goal || (inspection_goal->target - target).norm() >= retry_radius()))
      record_inspection_attempt(target, "inspection_plan_failed");
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
    else if (raw) {
      if (!checked_targets.empty())
        failed_target = checked_targets.front().first;
    } else
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
  bool success = publish_trajectory(t, occupied, raw != nullptr);
  stage("trajectory_publish", start, false, success ? "published" : "rejected");
  if (success) {
    selected_targets = targets;
    if (raw) {
      V3 target = targets.back();
      if (!inspection_goal || (inspection_goal->target - target).norm() >= retry_radius()) {
        record_inspection_attempt(target, "inspection_attempt_started");
      }
      inspection_goal = InspectionGoal{active_trajectory.back().head<3>(), target};
    }
  } else if (plan_failure.empty()) {
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
    auto expiry = blacklist_expiries.find(k);
    // No expiry marks a repeated failure blacklisted for this mission.
    if (expiry == blacklist_expiries.end() || expiry->second > now)
      retained.push_back(target);
    else {
      blacklist_expiries.erase(k);
      blacklist_radii.erase(k);
      collision_blacklist_radii.erase(k);
      filter("blacklist_expired", 1, 1, {{"target", json(target)}});
    }
  }
  blacklisted_targets = retained;
}
bool Mission::blacklist_failed() {
  if (!failed_target)
    return false;
  V3 target = *failed_target;
  for (const auto &attempt : inspection_attempts)
    if ((attempt.target - target).norm() < retry_radius() &&
        attempt.count >= i("route_target_max_attempts")) {
      blacklist_exhausted_target(attempt.target);
      return true;
    }
  if (d("route_blacklist_cooldown_s") <= 0)
    return false;
  bool exists = false;
  for (auto &t : blacklisted_targets)
    if ((t - target).norm() <= blacklist_radius(t)) {
      target = t;
      exists = true;
      break;
    }
  // A later cooldown must never release a mission-long blacklist entry.
  if (exists && !blacklist_expiries.count(key(target)))
    return true;
  if (!exists)
    blacklisted_targets.push_back(target);
  blacklist_radii[key(target)] = exists ? std::max(blacklist_radius(target), retry_radius()) : retry_radius();
  blacklist_expiries[key(target)] =
      ports.clock() + d("route_blacklist_cooldown_s");
  filter("blacklist_addition", 1, 0,
         {{"target", json(target)},
          {"planning_failure", nullable(plan_failure)},
          {"radius", blacklist_radius(target)},
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
void Mission::blacklist_exhausted_target(const V3 &failed) {
  V3 target = failed;
  bool exists = false;
  for (const auto &entry : blacklisted_targets)
    if ((entry - target).norm() < retry_radius()) {
      target = entry;
      exists = true;
      break;
    }
  if (exists && !blacklist_expiries.count(key(target)) && blacklist_radius(target) >= retry_radius())
    return;
  if (!exists)
    blacklisted_targets.push_back(target);
  double radius = exists && !blacklist_expiries.count(key(target))
                      ? std::max(blacklist_radius(target), retry_radius()) : retry_radius();
  blacklist_expiries.erase(key(target));
  blacklist_radii[key(target)] = radius;
  filter("blacklist_addition", 1, 0,
         {{"target", json(target)}, {"reason", "attempt_budget_exhausted"},
          {"radius", radius}, {"expires", false}});
  ports.diagnostics("warning", "Repeatedly unreachable target blacklisted for "
                                "the rest of this mission: " + json(target).dump());
  force_replan = true;
}
double Mission::blacklist_radius(const V3 &target) const {
  auto radius = blacklist_radii.find(key(target));
  return radius == blacklist_radii.end() ? retry_radius() : radius->second;
}
double Mission::retry_radius() const { return d("route_blacklist_radius_m"); }
bool Mission::collision_blacklisted(const V3 &target) const {
  return std::any_of(collision_blacklist_radii.begin(), collision_blacklist_radii.end(), [&](const auto &entry) {
    V3 center(entry.first[0], entry.first[1], entry.first[2]);
    return (center - target).norm() <= entry.second + 1e-9;
  });
}
void Mission::blacklist_collision_target(const V3 &target) {
  if (collision_blacklisted(target))
    return;
  auto existing = std::find_if(blacklisted_targets.begin(), blacklisted_targets.end(),
                              [&](const auto &entry) { return (entry - target).norm() < 1e-9; });
  bool exists = existing != blacklisted_targets.end();
  if (!exists)
    blacklisted_targets.push_back(target);
  double radius = d("route_collision_blacklist_radius_m");
  if (exists && !blacklist_expiries.count(key(target)))
    radius = std::max(radius, blacklist_radius(target));
  blacklist_expiries.erase(key(target));
  blacklist_radii[key(target)] = radius;
  collision_blacklist_radii[key(target)] = d("route_collision_blacklist_radius_m");
  filter("blacklist_addition", 1, 0,
         {{"target", json(target)}, {"reason", "all_nbv_rays_collide"},
          {"radius", radius}, {"expires", false}});
  ports.diagnostics("warning", "All sampled NBV rays collide; blacklisting target and its " +
                                std::to_string(radius) + "m neighborhood for this mission: " + json(target).dump());
  force_replan = true;
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
void Mission::cancel(const Voxels *raw, const Voxels *collision) {
  int previous = active_trajectory.size();
  // Stop downstream motion before performing the surrounding-ray checks.
  ports.publish({});
  if (inspection_goal && !target_observed(inspection_goal->target)) {
    failed_target = inspection_goal->target;
    plan_failure = active_failure.empty() ? "inspection_route_cancelled" : active_failure;
    bool collision_failure = active_failure == "new_obstacle_on_active_route" ||
                             active_failure == "unsafe_current_route_entry" ||
                             active_failure == "vehicle_inside_tracking_clearance";
    if (raw && collision && collision_failure) {
      auto probe = probe_nbv_views(inspection_goal->position, inspection_goal->target,
                                   *collision, *raw, d("voxel_resolution"), camera_offset,
                                   d("max_depth"), d("route_goal_max_extension_m"));
      filter("nbv_collision_rays", probe.directions_checked, probe.clear_directions,
             {{"target", json(inspection_goal->target)},
              {"directions_checked", probe.directions_checked},
              {"clear_directions", probe.clear_directions},
              {"reason", probe.directions_checked && probe.goals.empty()
                             ? "all_nbv_rays_collide" : "recheck_viewpoints"},
              {"blacklist_radius_m", d("route_collision_blacklist_radius_m")}});
      if (probe.directions_checked && probe.goals.empty())
        blacklist_collision_target(inspection_goal->target);
      // Clear directions remain eligible for immediate A* replanning.
      force_replan = true;
    } else
      blacklist_failed();
  }
  inspection_goal.reset();
  active_trajectory.clear();
  recovering_clearance = false;
  progress_position.reset();
  progress_time.reset();
  selected_targets.clear();
  filter("trajectory_cancelled", previous, 0,
         {{"reason", active_failure.empty() ? "new_obstacle_on_active_route"
                                            : active_failure},
          {"downstream_queue_cleared", true}});
}
bool Mission::target_observed(const V3 &target) const {
  return std::any_of(visited_targets.begin(), visited_targets.end(),
                     [&](const auto &v) { return (v - target).norm() < .5; });
}
void Mission::record_inspection_attempt(const V3 &target, const std::string &stage) {
  auto record = std::find_if(inspection_attempts.begin(), inspection_attempts.end(),
      [&](const auto &a) { return (a.target - target).norm() < retry_radius(); });
  if (record == inspection_attempts.end()) {
    inspection_attempts.push_back({target, 0, 0});
    record = std::prev(inspection_attempts.end());
  }
  ++record->count;
  record->started = ports.clock();
  filter(stage, 1, stage == "inspection_attempt_started" ? 1 : 0,
         {{"target", json(target)}, {"attempt", record->count},
          {"timeout_s", d("route_target_timeout_s")},
          {"region_center", json(record->target)}, {"matching_radius_m", retry_radius()}});
  if (stage == "inspection_plan_failed" &&
      record->count >= i("route_target_max_attempts"))
    blacklist_exhausted_target(record->target);
}
void Mission::update_inspection_progress(const Voxels &raw) {
  if (!inspection_goal)
    return;
  auto goal = *inspection_goal;
  // Use the pose of this depth frame, not a prediction or a published route.
  V3 camera = latched_position + body_rotation(latched_roll, latched_pitch, latched_yaw) * camera_offset;
  V3 sight = goal.target - camera;
  double distance = sight.norm();
  double desired_yaw = atan2(goal.target.y() - latched_position.y(),
                             goal.target.x() - latched_position.x());
  double yaw_error = std::abs(atan2(sin(desired_yaw - latched_yaw), cos(desired_yaw - latched_yaw)));
  if (depth_has_observations &&
      (latched_position - goal.position).norm() <= std::max(.4, d("visit_radius")) &&
      distance > 1e-9 && distance <= d("max_depth") &&
      yaw_error <= nbv_parameters["visited_yaw_threshold_deg"].get<double>() * pi / 180 &&
      clearance_free({camera, goal.target - sight.normalized() * std::min(2 * d("voxel_resolution"), distance * .5)},
                     raw, d("voxel_resolution"), 0)) {
    visited_targets.push_back(goal.target);
    filter("inspection_target_observed", 1, 1,
           {{"target", json(goal.target)}, {"resolved_goal", json(goal.position)},
            {"measured_position", json(latched_position)}, {"camera_range_m", distance}});
    active_failure = "inspection_goal_observed";
    cancel();
    force_replan = true;
    return;
  }
  for (const auto &attempt : inspection_attempts)
    if ((attempt.target - goal.target).norm() < retry_radius() &&
        ports.clock() - attempt.started >= d("route_target_timeout_s")) {
      active_failure = "inspection_attempt_timeout";
      filter("inspection_attempt_timeout", 1, 0,
             {{"target", json(goal.target)}, {"attempt", attempt.count},
              {"elapsed_s", ports.clock() - attempt.started}});
      cancel();
      force_replan = true;
      return;
    }
}
bool Mission::recover_clearance(const Voxels &raw, const Voxels &collision) {
  auto state = motion_snapshot();
  double resolution = d("voxel_resolution"), radius = d("collision_radius"),
         margin = d("collision_tracking_margin"), now = ports.clock();
  bool inside = collision.count(voxel(state.position, resolution));
  if (recovering_clearance) {
    auto remaining = reconnect(active_trajectory, state.position, state.yaw, {}, resolution);
    if (remaining.empty() || !retreat_safe(positions(remaining), raw, resolution, radius)) {
      active_failure = "clearance_recovery_unsafe_or_stalled";
      cancel();
      recovery_retry_time = now + d("route_stall_timeout_s");
    } else if ((state.position - active_trajectory.back().head<3>()).norm() <=
                   std::max(.4, d("visit_radius")) &&
               !inside && obstacle_clearance(state.position, raw, resolution) >=
                              radius + margin + resolution) {
      active_failure = "clearance_recovered";
      cancel();
      force_replan = true;
      failed_plan_count = 0;
      return false;
    } else if (stalled(state.position, remaining)) {
      active_failure = "clearance_recovery_unsafe_or_stalled";
      cancel();
      recovery_retry_time = now + d("route_stall_timeout_s");
    } else {
      return true;
    }
  }
  if (!inside)
    return false;
  if (!active_trajectory.empty()) {
    active_failure = "vehicle_inside_tracking_clearance";
    cancel(&raw, &collision);
  }
  // Recovery is independent of NBV availability. A blocked vehicle is not a
  // completed mission, and repeatedly running the same A* cannot free its start.
  force_replan = true;
  if (now < recovery_retry_time)
    return true;
  recovery_retry_time = now + d("route_stall_timeout_s");
  auto path = clearance_retreat(state.position, raw, collision, resolution, radius, margin);
  filter("clearance_recovery", 1, path.empty() ? 0 : 1,
         {{"vehicle_position", json(state.position)}, {"path", json(path)},
          {"obstacle_clearance_m", obstacle_clearance(state.position, raw, resolution)},
          {"body_radius_m", radius}, {"tracking_margin_m", margin},
          {"reason", path.empty() ? "no_safe_retreat_hold" : "retreat_from_tracking_buffer"}});
  if (path.empty()) {
    plan_failure = "clearance_recovery_blocked";
    ports.diagnostics("warning", "Inside collision clearance; no safe retreat, holding position.");
    return true;
  }
  Trajectory retreat;
  for (const auto &p : path)
    retreat.emplace_back(p.x(), p.y(), p.z(), state.yaw);
  // Odometry can advance on its callback group during the bounded search.
  state = motion_snapshot();
  retreat = reconnect(retreat, state.position, state.yaw, {}, resolution);
  recovery_obstacles = raw;
  recovering_clearance = true;
  if (!publish_trajectory(retreat, {}, true)) {
    recovering_clearance = false;
    plan_failure = "clearance_recovery_wire_collision";
  } else {
    selected_targets.clear();
    progress_position = state.position;
    progress_time = now;
    plan_failure.clear();
  }
  return true;
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
  auto keys = map.collision_voxels(d("voxel_resolution"));
  auto collision =
      inflate_clearance(keys, d("collision_radius") + d("collision_tracking_margin"),
              d("voxel_resolution"));
  stage("collision_inflation", start, true);
  update_inspection_progress(keys);
  if (recover_clearance(keys, collision)) {
    stage("planning_cycle_total", started, true);
    std::lock_guard<std::mutex> lock(state_mutex);
    ++iteration;
    return;
  }
  bool blocked = !active_safe(collision);
  if (blocked) {
    cancel(&keys, &collision);
    ports.diagnostics("warning", "Active trajectory cancelled (" +
                                     active_failure + "); forcing replan!");
  }
  bool arrived = false;
  auto state = motion_snapshot();
  if (b("replan_on_target_arrival") && !inspection_goal)
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
    parameters["defer_utility_normalization"] = true;
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
    double maximum_ban_radius = 0;
    for (const auto &target : blacklisted_targets)
      maximum_ban_radius = std::max(maximum_ban_radius, blacklist_radius(target));
    for (auto &c : candidates_) {
      Json bans = Json::array();
      double matched_radius = 0;
      for (auto &t : blacklisted_targets)
        if ((c.target - t).norm() <= blacklist_radius(t) + 1e-9) {
          auto expiry = blacklist_expiries.find(key(t));
          Json ban = {{"target", json(t)}, {"distance", (c.target - t).norm()}};
          ban["radius"] = blacklist_radius(t);
          matched_radius = std::max(matched_radius, blacklist_radius(t));
          if (expiry == blacklist_expiries.end()) {
            ban["remaining_s"] = nullptr;
            ban["expires"] = false;
          } else
            ban["remaining_s"] = std::max(0., expiry->second - ports.clock());
          bans.push_back(ban);
        }
      if (bans.empty())
        safe.push_back(c);
      filter("candidate_blacklist", 1, bans.empty() ? 1 : 0,
             {{"cluster_id", c.cluster_id},
              {"target", json(c.target)},
              {"radius", bans.empty() ? retry_radius() : matched_radius},
              {"matching_bans", bans}});
    }
    if (safe.size() < candidates_.size())
      ports.diagnostics("info",
                        "Dropped " +
                            std::to_string(candidates_.size() - safe.size()) +
                            " blacklisted targets.");
    filter("blacklist", candidates_.size(), safe.size(),
           {{"radius", blacklisted_targets.empty() ? retry_radius() : maximum_ban_radius},
            {"blacklisted_target_count", blacklisted_targets.size()}});
    const size_t generated_count = candidates_.size();
    bool published = plan_and_publish(safe, collision, &keys, &candidates_);
    std::string failure =
        generated_count == 0
            ? (empty_reason.empty() ? "nbv_generation_returned_no_candidates"
                                    : empty_reason)
        : safe.empty() ? "blacklist_removed_all"
                       : plan_failure;
    filter("planning_result", generated_count, int(published),
           {{"published", published},
            {"reason", published ? "published" : failure},
            {"generated_candidates", generated_count},
            {"after_blacklist", safe.size()},
            {"active_trajectory_points", active_trajectory.size()}});
    if (!published)
      ports.diagnostics("warning",
                        "[NBV_EMPTY] No trajectory published: " + failure);
    if (published)
      failed_plan_count = 0;
    else if (failure == "target_attempts_exhausted" || failure == "collision_blacklist_removed_all" ||
             (!blacklisted_targets.empty() &&
              (safe.empty() || plan_failure == "minimum_utility_removed_all"))) {
      filter("inspection_waiting", safe.size(), 0,
             {{"reason", failure}, {"mission_complete", false}});
      failed_plan_count = 0;
    } else if (!force_replan) {
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
