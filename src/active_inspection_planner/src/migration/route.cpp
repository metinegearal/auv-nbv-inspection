#include "active_inspection_planner/migration/core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>

extern "C" {
void parcur_(int *, int *, int *, int *, double *, int *, double *, double *,
             double *, double *, int *, double *, int *, int *, double *, int *,
             double *, double *, double *, int *, int *, int *);
void splev_(double *, int *, double *, int *, int *, double *, double *, int *,
            int *, int *);
}
namespace inspection::migration {
Key voxel(const V3 &v, double r) {
  return {int(std::floor(v.x() / r)), int(std::floor(v.y() / r)),
          int(std::floor(v.z() / r))};
}
Json json(const V3 &v) { return {v.x(), v.y(), v.z()}; }
Json json(const V4 &v) { return {v.x(), v.y(), v.z(), v.w()}; }
Json json(const Points &p) {
  Json j = Json::array();
  for (const auto &v : p)
    j.push_back(json(v));
  return j;
}
Json json(const Trajectory &p) {
  Json j = Json::array();
  for (const auto &v : p)
    j.push_back(json(v));
  return j;
}
V3 vec3(const Json &j) {
  return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}
Points points(const Json &j) {
  Points p;
  for (const auto &v : j)
    p.push_back(vec3(v));
  return p;
}
Trajectory trajectory(const Json &j) {
  Trajectory p;
  for (const auto &v : j)
    p.emplace_back(v[0].get<double>(), v[1].get<double>(), v[2].get<double>(),
                   v[3].get<double>());
  return p;
}
Points positions(const Trajectory &t) {
  Points p;
  for (const auto &v : t)
    p.push_back(v.head<3>());
  return p;
}
Json json(const Candidate &c) {
  return {{"cluster_id", c.cluster_id}, {"position", json(c.position)},
          {"target", json(c.target)},   {"utility", c.utility},
          {"raw_gain", c.raw_gain},     {"raw_cost", c.raw_cost}};
}
std::vector<Candidate> candidates(const Json &j) {
  std::vector<Candidate> out;
  for (const auto &v : j)
    out.push_back({vec3(v.at("position")), vec3(v.at("target")),
                   v.value("utility", 0.0), v.value("raw_gain", 0.0),
                   v.value("raw_cost", 0.0), v.value("cluster_id", -1),
                   v.contains("raw_gain") && v.contains("raw_cost")});
  return out;
}
double monotonic() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
std::vector<double> linspace(double a, double b, int n) {
  std::vector<double> v(std::max(0, n));
  if (n == 1)
    v[0] = a;
  if (n > 1) {
    double step = (b - a) / (n - 1);
    for (int i = 0; i < n; ++i)
      v[i] = a + i * step;
    v.back() = b;
  }
  return v;
}
void unwrap(std::vector<double> &v) {
  double correction = 0;
  for (size_t i = 1; i < v.size(); ++i) {
    double delta = v[i] - v[i - 1] + correction;
    double mod = std::fmod(delta + pi, 2 * pi);
    if (mod < 0)
      mod += 2 * pi;
    mod -= pi;
    if (mod == -pi && delta > 0)
      mod = pi;
    if (std::abs(delta) >= pi)
      correction += mod - delta;
    v[i] += correction;
  }
}
double interp(double x, const std::vector<double> &xs,
              const std::vector<double> &ys) {
  if (x <= xs.front())
    return ys.front();
  if (x >= xs.back())
    return ys.back();
  size_t i = std::upper_bound(xs.begin(), xs.end(), x) - xs.begin() - 1;
  return ys[i] + (x - xs[i]) / (xs[i + 1] - xs[i]) * (ys[i + 1] - ys[i]);
}
SearchResult astar(const V3 &start_pos, const V3 &goal_pos,
                   const Voxels &occupied, double r, int max_steps) {
  Key start = voxel(start_pos, r), goal = voxel(goal_pos, r);
  if (occupied.count(start))
    return {{}, "start_occupied", 0};
  if (occupied.count(goal))
    return {{}, "goal_occupied", 0};
  auto h = [&](const Key &k) {
    return std::abs(k[0] - goal[0]) + std::abs(k[1] - goal[1]) +
           std::abs(k[2] - goal[2]);
  };
  using Entry = std::tuple<int, int, Key, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  open.emplace(h(start), h(start), start, 0);
  std::unordered_map<Key, Key, KeyHash> parent;
  std::unordered_map<Key, int, KeyHash> costs{{start, 0}};
  const Key dirs[] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                      {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  int expansions = 0;
  while (!open.empty() && expansions < max_steps) {
    auto [f, heur, current, g] = open.top();
    open.pop();
    if (g != costs.at(current))
      continue;
    ++expansions;
    if (current == goal) {
      Points p;
      for (;;) {
        p.push_back(
            (V3(current[0], current[1], current[2]) + V3::Constant(.5)) * r);
        if (!parent.count(current))
          break;
        current = parent.at(current);
      }
      std::reverse(p.begin(), p.end());
      return {p, "success", expansions};
    }
    for (const auto &d : dirs) {
      Key n{current[0] + d[0], current[1] + d[1], current[2] + d[2]};
      if (occupied.count(n))
        continue;
      auto old = costs.find(n);
      if (old != costs.end() && g + 1 >= old->second)
        continue;
      parent[n] = current;
      costs[n] = g + 1;
      open.emplace(g + 1 + h(n), h(n), n, g + 1);
    }
  }
  return {{},
          expansions >= max_steps ? "search_limit_reached"
                                  : "no_connected_path",
          expansions};
}
SearchResult astar_view_standoff(const V3 &start, const V3 &goal,
                                 const V3 &target, const Voxels &occupied,
                                 const Voxels &raw, double r,
                                 const V3 &camera_offset, double max_range,
                                 double max_extension, int max_steps) {
  auto exact = astar(start, goal, occupied, r, max_steps);
  if (exact.reason != "goal_occupied" && exact.path.empty())
    return exact;
  double yaw = std::atan2(target.y() - goal.y(), target.x() - goal.x());
  V3 camera = goal + body_rotation(0, 0, yaw) * camera_offset;
  V3 outward = camera - target;
  double range = outward.norm();
  if (!std::isfinite(range) || range < 1e-9)
    return {{}, "invalid_view_direction", 0};
  outward /= range;
  auto visible = [&](const V3 &from) {
    double terminal = std::min(2 * r, (from - target).norm() * .5);
    return (from - target).norm() <= max_range &&
           clearance_free({from, target + terminal * outward}, raw, r, 0);
  };
  if (!exact.path.empty())
    return visible(camera) ? exact : SearchResult{{}, "goal_view_occluded", exact.expansions};
  if (max_extension <= 0)
    return exact;
  double limit = std::min(max_extension, max_range - range);
  SearchResult result{{}, "no_safe_goal_viewpoint", 0};
  // Preserve the selected viewing ray. Moving farther away keeps its viewing
  // angle while allowing the vehicle footprint to clear the inflated surface.
  for (double extension = std::min(r, limit); extension > 0;) {
    V3 next_camera = camera + extension * outward;
    V3 next_goal = vehicle_viewpoint(next_camera, target, camera_offset);
    if (!occupied.count(voxel(next_goal, r))) {
      // The cluster target lies on a surface: exclude only its terminal cell
      // neighborhood, while rejecting intervening mapped occluders.
      if (visible(next_camera)) {
        auto leg = astar(start, next_goal, occupied, r,
                         std::max(0, max_steps - result.expansions));
        result.expansions += leg.expansions;
        if (!leg.path.empty()) {
          if (leg.path.size() == 1)
            leg.path = {start, next_goal};
          else {
            leg.path.front() = start;
            leg.path.back() = next_goal;
          }
          if (collision_free(leg.path, occupied, r))
            return {leg.path, "success_adjusted_standoff", result.expansions};
        }
        if (result.expansions >= max_steps)
          return {{}, "search_limit_reached", result.expansions};
      }
    }
    if (extension >= limit - 1e-9)
      break;
    extension = std::min(extension + r, limit);
  }
  return result;
}
ViewProbe probe_nbv_views(const V3 &goal, const V3 &target,
                          const Voxels &occupied, const Voxels &raw, double r,
                          const V3 &camera_offset, double max_range,
                          double max_extension) {
  ViewProbe probe;
  double yaw = std::atan2(target.y() - goal.y(), target.x() - goal.x());
  V3 original_camera = goal + body_rotation(0, 0, yaw) * camera_offset;
  V3 outward = original_camera - target;
  double range = outward.norm();
  if (!(r > 0) || !std::isfinite(range) || range < 1e-9 ||
      !std::isfinite(max_range) || max_range < range)
    return probe;
  double limit = std::min(max_range, range + std::max(0., max_extension));
  Points directions{outward / range};
  // A 15-degree spherical sweep includes both vertical poles. This is a
  // finite visibility/clearance probe, not proof of continuous reachability.
  for (int elevation = -75; elevation <= 75; elevation += 15)
    for (int azimuth = 0; azimuth < 360; azimuth += 15) {
      double e = elevation * pi / 180, a = azimuth * pi / 180;
      directions.emplace_back(std::cos(e) * std::cos(a),
                              std::cos(e) * std::sin(a), std::sin(e));
    }
  directions.emplace_back(0, 0, -1);
  directions.emplace_back(0, 0, 1);
  // Rays remain inside the sensor sphere. Restrict the raw-map slab tests to
  // intersecting cells so a global map does not multiply every local ray cost.
  Voxels local_raw, seen_goals;
  for (const auto &cell : raw) {
    V3 lo(cell[0] * r, cell[1] * r, cell[2] * r);
    if ((lo.array() <= (target.array() + max_range)).all() &&
        ((lo.array() + r) >= (target.array() - max_range)).all())
      local_raw.insert(cell);
  }
  for (const auto &direction : directions) {
    ++probe.directions_checked;
    bool clear = false;
    for (double distance = range;; distance = std::min(distance + r, limit)) {
      V3 camera = target + distance * direction;
      V3 next_goal = vehicle_viewpoint(camera, target, camera_offset);
      double heading = std::atan2(target.y() - next_goal.y(), target.x() - next_goal.x());
      camera = next_goal + body_rotation(0, 0, heading) * camera_offset;
      V3 sight = camera - target;
      double actual_range = sight.norm();
      if (actual_range > 1e-9 && actual_range <= max_range + 1e-9 &&
          !occupied.count(voxel(next_goal, r)) &&
          clearance_free({camera, target + sight.normalized() * std::min(2 * r, actual_range * .5)},
                         local_raw, r, 0)) {
        clear = true;
        if (seen_goals.insert(voxel(next_goal, r)).second)
          probe.goals.push_back(next_goal);
      }
      if (distance >= limit - 1e-9)
        break;
    }
    probe.clear_directions += int(clear);
  }
  return probe;
}
SearchResult astar_view_surround(const V3 &start, const V3 &goal,
                                const V3 &target, const Voxels &occupied,
                                const Voxels &raw, double r,
                                const V3 &camera_offset, double max_range,
                                double max_extension, int max_steps) {
  // Keep a working selected view and its outward adjustment unchanged.
  auto result = astar_view_standoff(start, goal, target, occupied, raw, r,
                                    camera_offset, max_range, max_extension, max_steps);
  if (!result.path.empty() || result.reason == "start_occupied" ||
      result.reason == "search_limit_reached" || result.reason == "invalid_view_direction")
    return result;
  auto probe = probe_nbv_views(goal, target, occupied, raw, r,
                               camera_offset, max_range, max_extension);
  result.directions_checked = probe.directions_checked;
  result.clear_directions = probe.clear_directions;
  if (!probe.directions_checked)
    return result;
  if (probe.goals.empty()) {
    result.reason = "all_nbv_rays_collide";
    return result;
  }
  std::stable_sort(probe.goals.begin(), probe.goals.end(), [&](const auto &a, const auto &b) {
    return (a - start).squaredNorm() < (b - start).squaredNorm();
  });
  for (const auto &next_goal : probe.goals) {
    if (result.expansions >= max_steps) {
      result.reason = "search_limit_reached";
      return result;
    }
    auto leg = astar(start, next_goal, occupied, r, max_steps - result.expansions);
    result.expansions += leg.expansions;
    if (!leg.path.empty()) {
      if (leg.path.size() == 1)
        leg.path = {start, next_goal};
      else {
        leg.path.front() = start;
        leg.path.back() = next_goal;
      }
      if (collision_free(leg.path, occupied, r)) {
        result.path = std::move(leg.path);
        result.reason = "success_surrounding_view";
        return result;
      }
    }
  }
  result.reason = result.expansions >= max_steps ? "search_limit_reached" : "no_connected_path";
  return result;
}
Voxels inflate(const Voxels &occupied, double radius, double r) {
  if (occupied.empty())
    return {};
  if (r == 0 || !std::isfinite(radius / r))
    throw std::invalid_argument("nonfinite inflation radius in voxels");
  int n = std::ceil(radius / r);
  std::vector<Key> offsets;
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z)
        if (x * x + y * y + z * z <= n * n)
          offsets.push_back({x, y, z});
  Voxels out;
  for (const auto &k : occupied)
    for (const auto &o : offsets)
      out.insert({k[0] + o[0], k[1] + o[1], k[2] + o[2]});
  return out;
}
bool collision_free(const Points &p, const Voxels &occupied, double r,
                    double factor) {
  if (p.empty())
    return false;
  for (const auto &v : p)
    if (!v.allFinite())
      return false;
  if (occupied.count(voxel(p.front(), r)))
    return false;
  for (size_t i = 1; i < p.size(); ++i) {
    int n =
        std::max(1, int(std::ceil((p[i] - p[i - 1]).norm() / (r * factor))));
    for (double t : linspace(0, 1, n + 1))
      if (occupied.count(voxel(p[i - 1] + t * (p[i] - p[i - 1]), r)))
        return false;
  }
  return true;
}
Trajectory reconnect(const Trajectory &path, const V3 &pos, double yaw,
                     const Voxels &occupied, double r, double factor) {
  if (path.empty() || !pos.allFinite() || !std::isfinite(yaw))
    return {};
  for (const auto &v : path)
    if (!v.allFinite())
      return {};
  size_t first = 0;
  if (path.size() > 1) {
    double nearest = INFINITY;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      V3 step = path[i + 1].head<3>() - path[i].head<3>();
      double t = std::clamp((pos - path[i].head<3>()).dot(step) /
                                std::max(step.squaredNorm(), 1e-12),
                            0., 1.);
      double d = (path[i].head<3>() + t * step - pos).norm();
      if (d < nearest) {
        nearest = d;
        first = i + 1;
      }
    }
  }
  Trajectory suffix;
  for (size_t i = first; i < path.size(); ++i)
    if (i == first || (path[i].head<3>() - path[i - 1].head<3>()).norm() > 1e-9)
      suffix.push_back(path[i]);
  if ((suffix.front().head<3>() - pos).norm() < 1e-9)
    suffix.erase(suffix.begin());
  Trajectory out{V4(pos.x(), pos.y(), pos.z(), yaw)};
  out.insert(out.end(), suffix.begin(), suffix.end());
  std::vector<double> y;
  for (auto &v : out)
    y.push_back(v.w());
  unwrap(y);
  for (size_t i = 0; i < out.size(); ++i)
    out[i].w() = y[i];
  Points entry{pos};
  if (out.size() > 1)
    entry.push_back(out[1].head<3>());
  return collision_free(entry, occupied, r, factor) ? out : Trajectory{};
}
Route plan_route(const V3 &position, const std::vector<Candidate> &cs,
                 const Voxels &occupied, double r, int max_steps, V3 direction,
                 double reversal,
                 const std::function<SearchResult(const V3 &, const Candidate &)> &goal_fallback,
                 bool single_goal) {
  if (cs.empty())
    return {{}, {}, "no_candidates"};
  auto remaining = cs;
  Route out{{}, {}, "no_candidates"};
  V3 start = position;
  if (direction.norm() < .05)
    direction.setZero();
  while (!remaining.empty()) {
    auto score = [&](const Candidate &c) {
      V3 delta = c.position - start;
      double distance = delta.norm();
      double align =
          delta.dot(direction) / std::max(distance * direction.norm(), 1e-9);
      return std::make_pair(-c.utility,
                            distance + reversal * std::max(0., -align));
    };
    auto it = std::min_element(
        remaining.begin(), remaining.end(),
        [&](const auto &a, const auto &b) { return score(a) < score(b); });
    Candidate c = *it;
    remaining.erase(it);
    auto result = single_goal && goal_fallback ? goal_fallback(start, c)
                   : astar(start, c.position, occupied, r, max_steps);
    if (!single_goal && result.reason == "goal_occupied" && goal_fallback) {
      result = goal_fallback(start, c);
    }
    if (!result.path.empty() && (result.reason == "success_adjusted_standoff" ||
                                 result.reason == "success_surrounding_view"))
      c.position = result.path.back();
    auto segment = result.path;
    if (segment.empty()) {
      if (single_goal && result.reason == "candidate_pool_changed")
        return {{}, {}, result.reason};
      if (result.reason == "start_occupied" && out.path.empty())
        return {{}, {}, result.reason};
      out.reason = result.reason;
      continue;
    }
    if (segment.size() == 1)
      segment = {start, c.position};
    else {
      segment.front() = start;
      segment.back() = c.position;
    }
    Points keep;
    for (size_t i = 0; i < segment.size(); ++i)
      if (i == 0 || (segment[i] - segment[i - 1]).norm() > 1e-9)
        keep.push_back(segment[i]);
    if (!collision_free(keep, occupied, r)) {
      out.reason = "leg_validation_failed";
      continue;
    }
    if (!out.path.empty() && ((out.path.back() - keep.front()).array().abs() <=
                              (1e-8 + 1e-5 * keep.front().array().abs()))
                                 .all())
      keep.erase(keep.begin());
    out.path.insert(out.path.end(), keep.begin(), keep.end());
    out.targets.insert(out.targets.end(), keep.size(), c.target);
    if (single_goal)
      break;
    direction = c.position - start;
    start = c.position;
  }
  if (!out.path.empty())
    out.reason = "success";
  return out;
}
Trajectory smooth(const Points &input, const Points &target, int count,
                  double smoothing, double spacing) {
  if (input.empty() || input.size() != target.size())
    return {};
  Points p, t;
  for (size_t i = 0; i < input.size(); ++i)
    if (i == 0 || (input[i] - input[i - 1]).norm() > 1e-9) {
      p.push_back(input[i]);
      t.push_back(target[i]);
    }
  std::vector<double> yaw, dist{0};
  for (size_t i = 0; i < p.size(); ++i) {
    yaw.push_back(std::atan2(t[i].y() - p[i].y(), t[i].x() - p[i].x()));
    if (i)
      dist.push_back(dist.back() + (p[i] - p[i - 1]).norm());
  }
  unwrap(yaw);
  if (p.size() == 1)
    return {V4(p[0].x(), p[0].y(), p[0].z(), yaw[0])};
  double length = dist.back();
  for (double &d : dist)
    d /= length;
  int iopt = 0, ipar = 1, idim = 3, m = p.size(), mx = 3 * m,
      k = std::min(3, m - 1), nest = m + 2 * k, n = 0, nc = nest * idim,
      ier = 0;
  if (smoothing == 0)
    nest = m + k + 1;
  nc = nest * idim;
  int lwrk = m * (k + 1) + nest * (6 + idim + 3 * k);
  std::vector<double> x, w(m, 1), knots(nest), coef(nc), wrk(lwrk);
  std::vector<int> iwrk(nest);
  for (auto &v : p)
    for (int a = 0; a < 3; ++a)
      x.push_back(v[a]);
  double ub = 0, ue = 1, fp = 0;
  parcur_(&iopt, &ipar, &idim, &m, dist.data(), &mx, x.data(), w.data(), &ub,
          &ue, &k, &smoothing, &nest, &n, knots.data(), &nc, coef.data(), &fp,
          wrk.data(), &lwrk, iwrk.data(), &ier);
  if (ier == 10)
    return {}; // FITPACK invalid input is scipy's ValueError.
  if (spacing > 0)
    count = std::max(count, int(std::ceil(length / spacing * 10)) + 1);
  auto samples = linspace(0, 1, count);
  Trajectory out(count, V4::Zero());
  int ext = 0;
  for (int a = 0; a < 3; ++a) {
    std::vector<double> vals(count);
    int ncoeff = n;
    splev_(knots.data(), &n, coef.data() + a * n, &ncoeff, &k, samples.data(),
           vals.data(), &count, &ext, &ier);
    for (int i = 0; i < count; ++i)
      out[i][a] = vals[i];
  }
  for (int i = 0; i < count; ++i)
    out[i].w() = interp(samples[i], dist, yaw);
  return out;
}
Trajectory resample(const Trajectory &input, double spacing,
                    std::optional<int> cap) {
  if (input.size() < 2 || spacing <= 0)
    return input;
  Trajectory p;
  for (size_t i = 0; i < input.size(); ++i)
    if (i == 0 || (input[i].head<3>() - input[i - 1].head<3>()).norm() > 1e-9)
      p.push_back(input[i]);
  std::vector<double> d{0};
  for (size_t i = 1; i < p.size(); ++i)
    d.push_back(d.back() + (p[i].head<3>() - p[i - 1].head<3>()).norm());
  double total = d.back();
  if (total <= 1e-9)
    return {p.front()};
  std::vector<double> samples;
  for (size_t i = 0; double(i) * spacing < total; ++i)
    samples.push_back(double(i) * spacing);
  if (samples.back() < total)
    samples.push_back(total);
  Trajectory out(samples.size(), V4::Zero());
  for (int a = 0; a < 4; ++a) {
    std::vector<double> vals;
    for (auto &v : p)
      vals.push_back(v[a]);
    if (a == 3)
      unwrap(vals);
    for (size_t i = 0; i < samples.size(); ++i)
      out[i][a] = interp(samples[i], d, vals);
  }
  if (cap && int(out.size()) > *cap) {
    Trajectory limited;
    int prev = -1;
    for (double x : linspace(0, out.size() - 1, *cap)) {
      int index = std::nearbyint(x);
      if (index != prev)
        limited.push_back(out[index]);
      prev = index;
    }
    return limited;
  }
  return out;
}
Trajectory linear_fallback(const Points &p, const Points &t, double spacing) {
  if (p.empty())
    return {};
  std::vector<double> y;
  for (size_t i = 0; i < p.size(); ++i)
    y.push_back(std::atan2(t[i].y() - p[i].y(), t[i].x() - p[i].x()));
  unwrap(y);
  Trajectory raw, out;
  for (size_t i = 0; i < p.size(); ++i)
    raw.emplace_back(p[i].x(), p[i].y(), p[i].z(), y[i]);
  for (size_t i = 1; i < raw.size(); ++i) {
    auto leg = resample({raw[i - 1], raw[i]}, spacing);
    out.insert(out.end(), leg.begin(), leg.end() - 1);
  }
  out.push_back(raw.back());
  return out;
}
Eigen::Matrix3d body_rotation(double roll, double pitch, double yaw) {
  double cr = cos(roll), sr = sin(roll), cp = cos(pitch), sp = sin(pitch),
         cy = cos(yaw), sy = sin(yaw);
  Eigen::Matrix3d r;
  r << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp,
      sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr;
  return r;
}
V3 vehicle_viewpoint(const V3 &cam, const V3 &target, const V3 &offset) {
  double yaw = atan2(target.y() - cam.y(), target.x() - cam.x());
  for (int i = 0; i < 12; ++i) {
    V3 v = cam - body_rotation(0, 0, yaw) * offset;
    double next = atan2(target.y() - v.y(), target.x() - v.x());
    if (std::abs(atan2(sin(next - yaw), cos(next - yaw))) < 1e-8)
      return v;
    yaw = next;
  }
  return cam - body_rotation(0, 0, yaw) * offset;
}
} // namespace inspection::migration
