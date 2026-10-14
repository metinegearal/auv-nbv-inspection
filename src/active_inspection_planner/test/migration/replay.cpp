#include "active_inspection_planner/migration/adapters.hpp"
#include <iostream>
#include <sys/resource.h>

using namespace inspection::migration;
namespace {
Voxels occupied(const Json &j) {
  Voxels out;
  for (auto &k : j)
    out.insert({k[0].get<int>(), k[1].get<int>(), k[2].get<int>()});
  return out;
}
Json keys(const Voxels &voxels) {
  std::set<Key> sorted(voxels.begin(), voxels.end());
  Json j = Json::array();
  for (auto &k : sorted)
    j.push_back(k);
  return j;
}
DepthFrame frame(const Json &j) {
  DepthFrame f;
  f.rows = j.at("rows");
  f.cols = j.at("cols");
  f.channels = j.value("channels", 1);
  f.uint16 = j.value("uint16", false);
  for (auto &x : j.at("values"))
    f.values.push_back(x.is_null() ? NAN : x.get<double>());
  return f;
}
struct ReplayMap : OccupancyMap {
  Points p;
  std::unique_ptr<Octomap> real;
  bool write_map = false;
  Json inserts = Json::array(), writes = Json::array();
  void insert(const Points &points_, const V3 &origin, double range) override {
    inserts.push_back({{"points", json(points_)},
                       {"origin", json(origin)},
                       {"max_range", range}});
    if (real)
      real->insert(points_, origin, range);
  }
  Points occupied() const override { return real ? real->occupied() : p; }
  size_t leaves() const override { return real ? real->leaves() : p.size(); }
  void write(const std::string &path) override {
    writes.push_back(path);
    if (real && write_map)
      real->write(path);
  }
};
Json run(const Json &j) {
  std::string op = j.at("op");
  double r = j.value("resolution", .2);
  auto occ = occupied(j.value("occupied", Json::array()));
  if (op == "defaults")
    return defaults();
  if (op == "astar") {
    auto result = astar(vec3(j.at("start")), vec3(j.at("goal")), occ, r,
                        j.value("max_steps", 50000));
    return {{"path", json(result.path)},
            {"reason", result.reason},
            {"expansions", result.expansions}};
  }
  if (op == "inflate")
    return keys(inflate(occ, j.at("radius"), r));
  if (op == "collision")
    return collision_free(points(j.at("path")), occ, r, j.value("factor", .5));
  if (op == "route") {
    auto route = plan_route(vec3(j.at("start")), candidates(j.at("candidates")),
                            occ, r, j.value("max_steps", 50000),
                            vec3(j.value("velocity", Json{0, 0, 0})),
                            j.value("reversal", 2.));
    return {{"path", json(route.path)},
            {"targets", json(route.targets)},
            {"reason", route.reason}};
  }
  if (op == "smooth")
    return json(smooth(points(j.at("path")), points(j.at("targets")),
                       j.value("count", 200), j.value("smoothing", 2.),
                       j.value("spacing", 0.)));
  if (op == "resample")
    return json(resample(trajectory(j.at("trajectory")), j.at("spacing"),
                         j.contains("cap") ? std::optional<int>(j["cap"])
                                           : std::nullopt));
  if (op == "reconnect")
    return json(reconnect(trajectory(j.at("trajectory")),
                          vec3(j.at("position")), j.at("yaw"), occ, r,
                          j.value("factor", .5)));
  if (op == "vehicle_viewpoint")
    return json(
        vehicle_viewpoint(vec3(j.at("camera")), vec3(j.at("target")),
                          vec3(j.value("offset", Json{.35, .3875, -.5}))));
  if (op == "seabed") {
    Seabed s(r);
    Json states = Json::array();
    for (auto &snapshot : j.at("snapshots")) {
      int changed = s.update(points(snapshot));
      states.push_back({{"changed", changed},
                        {"mean_z", s.mean_z ? Json(*s.mean_z) : Json(nullptr)},
                        {"sample_count", s.sample_count}});
    }
    return states;
  }
  if (op == "pca")
    return pca(points(j.at("points")), j.value("k", 20));
  if (op == "frontiers")
    return frontiers({points(j.at("points")), points(j.at("normals"))},
                     j.value("k", 40), j.value("angle", 130.),
                     j.value("radius", .5), j.value("min_neighbors", 3));
  if (op == "clusters") {
    auto c = cluster_frontiers(
        {points(j.at("points")), points(j.at("normals"))},
        j.at("indices").get<std::vector<int>>(), j.value("radius", 1.),
        j.value("angle", 45.), j.value("minimum", 5));
    Json out = Json::array();
    for (auto &x : c)
      out.push_back({{"id", x.id},
                     {"centroid", json(x.centroid)},
                     {"avg_normal", json(x.normal)},
                     {"points", json(x.points)},
                     {"normals", json(x.normals)},
                     {"global_indices", x.indices}});
    return out;
  }
  if (op == "axis")
    return json(view_axis(vec3(j.at("target")), vec3(j.at("position")),
                          points(j.at("points")), j.value("standoff", 1.5),
                          j.value("steps", 36)));
  if (op == "cone")
    return json(cone(
        vec3(j.at("target")), vec3(j.at("normal")), j.value("standoff", 1.5),
        j.value("polar", std::vector<double>{15, 30}), j.value("steps", 8)));
  if (op == "prepare") {
    Json p = nbv_defaults();
    p.update(j.value("parameters", Json::object()));
    auto cloud =
        prepare_cloud(points(j.at("points")), vec3(j.at("position")), p);
    if (!cloud)
      return nullptr;
    return {{"points", json(cloud->points)}, {"normals", json(cloud->normals)}};
  }
  if (op == "nbv") {
    std::vector<std::pair<V3, double>> history;
    for (auto &h : j.value("history", Json::array()))
      history.emplace_back(vec3(h[0]), h[1].get<double>());
    Json out = Json::array();
    for (auto &c : nbv(points(j.at("points")), vec3(j.at("position")),
                       vec3(j.at("heading")), history,
                       j.value("parameters", Json::object())))
      out.push_back(json(c));
    return out;
  }
  if (op == "continuity" || op == "optimize") {
    MotionWeights w;
    auto p = j.value("weights", Json::object());
    w.continuity = p.value("continuity_weight", 8.);
    w.reversal = p.value("reversal_weight", 4.);
    w.history = p.value("history_weight", 2.);
    w.radius = p.value("history_radius", .5);
    auto t = trajectory(j.at("trajectory"));
    auto v = vec3(j.at("velocity"));
    auto history = points(j.value("history", Json::array()));
    if (op == "continuity")
      return continuity_cost(t, v, history, w);
    return json(optimize(t, vec3(j.at("position")), j.at("yaw"), v, occ, r,
                         history, w, j.value("deviation", .5),
                         j.value("iterations", 40), j.value("factor", .5)));
  }
  if (op == "velocity") {
    std::deque<Pose> history;
    for (auto &p : j.at("history"))
      history.push_back({p[0].get<double>(), vec3(p[1])});
    return json(velocity(history, j.value("window", 1.)));
  }
  if (op == "octomap") {
    Octomap map(r);
    if (j.contains("load"))
      map.load(j.at("load"));
    Json out = Json::array();
    for (auto &scan : j.value("scans", Json::array())) {
      map.insert(points(scan.at("points")), vec3(scan.at("origin")),
                 scan.value("max_range", -1.));
      out.push_back(
          {{"points", json(map.occupied())}, {"leaves", map.leaves()}});
    }
    if (j.contains("load"))
      out.push_back(
          {{"points", json(map.occupied())}, {"leaves", map.leaves()}});
    return out;
  }
  if (op == "mission") {
    ReplayMap map;
    double now = 0;
    Json events = Json::array(), published = Json::array(),
         completed = Json::array(), states = Json::array(),
         evaluation = Json::array();
    std::unique_ptr<FileSink> files;
    if (j.contains("files"))
      files = std::make_unique<FileSink>(j.at("files"));
    MissionPorts ports;
    ports.clock = [&] {
      return j.value("realtime", false) ? monotonic() : now;
    };
    Mission *pointer = nullptr;
    std::optional<V3> move;
    if (j.value("real_map", false)) {
      map.real = std::make_unique<Octomap>(
          j.value("config", Json::object()).value("voxel_resolution", .1));
      map.write_map = j.value("write_map", false);
    }
    ports.telemetry = [&](const Json &event) {
      events.push_back(event);
      if (files)
        files->record(event);
      if (move && event.value("stage", "") == "route_search") {
        pointer->position = *move;
        move.reset();
      }
    };
    ports.publish = [&](auto &wire) { published.push_back(wire); };
    ports.complete = [&](bool b) { completed.push_back(b); };
    ports.evaluation = [&](auto &line) {
      evaluation.push_back(line);
      if (files)
        files->evaluation(line);
    };
    Mission m(j.value("config", Json::object()), map, ports);
    pointer = &m;
    m.seed(j.value("seed", 0));
    for (auto &e : j.at("events")) {
      std::string type = e.at("type");
      now = e.value("time", now);
      Json result = nullptr;
      if (type == "odom") {
        auto q = e.value("orientation", Json{0, 0, 0, 1});
        m.odometry({e.at("stamp").get<double>(), vec3(e.at("position"))},
                   Eigen::Quaterniond(q[3], q[0], q[1], q[2]));
      } else if (type == "mag")
        m.magnetometer(vec3(e.at("value")));
      else if (type == "rgb")
        m.rgb();
      else if (type == "depth")
        m.depth(frame(e.at("frame")), e.at("stamp"));
      else if (type == "project")
        result = json(m.depth_points(frame(e.at("frame"))));
      else if (type == "map")
        map.p = points(e.at("points"));
      else if (type == "request")
        m.request(e.value("value", true));
      else if (type == "tick")
        m.tick();
      else if (type == "cancel")
        m.cancel();
      else if (type == "plan") {
        if (e.contains("move_during_plan"))
          move = vec3(e.at("move_during_plan"));
        result =
            m.plan_and_publish(candidates(e.at("candidates")),
                               occupied(e.value("occupied", Json::array())));
      } else if (type == "publish")
        result =
            m.publish_trajectory(trajectory(e.at("trajectory")),
                                 occupied(e.value("occupied", Json::array())));
      else if (type == "active_safe")
        result = m.active_safe(occupied(e.value("occupied", Json::array())));
      else if (type == "blacklist")
        result = m.blacklist_failed();
      else if (type == "expire")
        m.expire_blacklist();
      else if (type == "candidates") {
        auto values = candidates(e.at("values"));
        m.calculate = [values](auto &, auto &, auto &, auto &, auto, auto,
                               auto) { return values; };
      } else
        throw std::invalid_argument("Unknown mission event: " + type);
      states.push_back({{"result", result}, {"state", m.snapshot()}});
    }
    return {{"states", states},        {"events", events},
            {"published", published},  {"completed", completed},
            {"inserts", map.inserts},  {"writes", map.writes},
            {"evaluation", evaluation}};
  }
  throw std::invalid_argument("Unknown operation: " + op);
}
} // namespace
int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      auto j = Json::parse(line);
      Json result;
      int repeat = j.value("repeat", 1);
      std::vector<double> ms;
      for (int i = 0; i < repeat; ++i) {
        double start = monotonic();
        result = run(j);
        ms.push_back((monotonic() - start) * 1000);
      }
      // /proc reports this executable's memory image. getrusage's high-water
      // mark can include the Python test parent's RSS inherited across exec.
      std::ifstream status("/proc/self/status");
      std::string status_line;
      long peak_rss = 0, current_rss = 0;
      while (std::getline(status, status_line)) {
        if (status_line.rfind("VmHWM:", 0) == 0)
          peak_rss = std::stol(status_line.substr(6));
        if (status_line.rfind("VmRSS:", 0) == 0)
          current_rss = std::stol(status_line.substr(6));
      }
      std::cout << Json({{"result", result},
                         {"latency_ms", ms},
                         {"max_rss_kib", peak_rss},
                         {"current_rss_kib", current_rss}})
                       .dump()
                << std::endl;
    } catch (const std::exception &e) {
      std::cout << Json({{"error", e.what()}}).dump() << std::endl;
    }
  }
}
