#include "active_inspection_planner/migration/adapters.hpp"
#include <iomanip>
#include <sstream>

namespace inspection::migration {
void Octomap::insert(const Points &points_, const V3 &origin,
                     double max_range) {
  octomap::Pointcloud cloud;
  for (auto &p : points_)
    cloud.push_back(float(p.x()), float(p.y()), float(p.z()));
  tree.insertPointCloud(
      cloud,
      octomap::point3d(float(origin.x()), float(origin.y()), float(origin.z())),
      max_range, false, false);
  tree.updateInnerOccupancy();
}
Points Octomap::occupied() const {
  Points out;
  float resolution = tree.getResolution();
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it))
      continue;
    int dimension = std::min(
        100, std::max(1, int(std::nearbyint(it.getSize() / resolution))));
    auto coord = it.getCoordinate();
    V3 center(coord.x(), coord.y(), coord.z()),
        origin = center - V3::Constant((dimension / 2 - .5) * resolution);
    for (int x = 0; x < dimension; ++x)
      for (int y = 0; y < dimension; ++y)
        for (int z = 0; z < dimension; ++z)
          out.push_back(origin + V3(x, y, z) * double(resolution));
  }
  return out;
}
Voxels Octomap::collision_voxels(double resolution) const {
  Voxels out;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it))
      continue;
    V3 center(it.getX(), it.getY(), it.getZ());
    // Include every planner cell overlapping the leaf, including coarse leaves.
    V3 half = V3::Constant(it.getSize() * .5);
    Key lo = voxel(center - half + V3::Constant(resolution * 1e-6), resolution);
    Key hi = voxel(center + half - V3::Constant(resolution * 1e-6), resolution);
    for (int x = lo[0]; x <= hi[0]; ++x)
      for (int y = lo[1]; y <= hi[1]; ++y)
        for (int z = lo[2]; z <= hi[2]; ++z)
          out.insert({x, y, z});
  }
  return out;
}
void Octomap::write(const std::string &path) { tree.write(path); }
void Octomap::load(const std::string &path) {
  std::unique_ptr<octomap::AbstractOcTree> loaded(
      octomap::AbstractOcTree::read(path));
  auto *color = dynamic_cast<octomap::ColorOcTree *>(loaded.get());
  if (!color)
    throw std::runtime_error("Expected ColorOcTree snapshot: " + path);
  tree.setResolution(color->getResolution());
  tree.swapContent(*color);
}
namespace {
std::filesystem::path expanded(std::string path) {
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (home)
      path = std::string(home) + path.substr(1);
  }
  return path;
}
std::string number(double d, int places = 6) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(places) << d;
  return s.str();
}
void csv(std::ostream &out, const std::vector<std::string> &row) {
  for (size_t i = 0; i < row.size(); ++i) {
    if (i)
      out << ',';
    bool quoted = row[i].find_first_of(",\r\n\"") != std::string::npos;
    if (quoted)
      out << '"';
    for (char c : row[i]) {
      if (c == '"')
        out << '"';
      out << c;
    }
    if (quoted)
      out << '"';
  }
  out << "\r\n";
}
std::string scalar(const Json &j) {
  if (j.is_null())
    return "";
  if (j.is_string())
    return j.get<std::string>();
  return j.dump();
}
} // namespace
FileSink::FileSink(const std::string &dir) : directory(expanded(dir)) {
  std::filesystem::create_directories(directory);
  for (auto [name, header] : std::vector<std::pair<std::string, std::string>>{
           {"planner_trajectory.csv",
            "time_s,x_m,y_m,z_m,cumulative_distance_m\r\n"},
           {"planner_depth_poses.csv", "depth_sequence,depth_time_s,x_m,y_m,z_"
                                       "m,roll_rad,pitch_rad,yaw_rad\r\n"},
           {"planner_future_trajectories.csv",
            "iteration,waypoint_index,x_m,y_m,z_m,yaw_rad\r\n"},
           {"planner_pipeline_timing.csv",
            "stage,iteration,frame_id,duration_ms,source_stamp_s,details\r\n"},
           {"planner_nbv_iterations.jsonl", ""},
           {"planner_nbv_filters.jsonl", ""}}) {
    std::ofstream file(directory / name);
    if (!file)
      throw std::runtime_error("Cannot open planner telemetry: " +
                               (directory / name).string());
    file << header;
  }
  // The Python evaluation adapter intentionally does not expand '~'.
  evaluation_file.open(std::filesystem::path(dir) / "evaluation_parameters.txt",
                       std::ios::app);
  if (!evaluation_file)
    throw std::runtime_error("Cannot open evaluation_parameters.txt");
}
void FileSink::evaluation(const std::string &line) {
  std::lock_guard<std::mutex> lock(mutex);
  evaluation_file << line;
  evaluation_file.flush();
}
void FileSink::record(const Json &event) {
  std::lock_guard<std::mutex> lock(mutex);
  std::string type = event.at("type");
  if (type == "filter") {
    Json record = event;
    record.erase("type");
    std::ofstream file(directory / "planner_nbv_filters.jsonl", std::ios::app);
    file << json_safe(record).dump() << '\n';
  } else if (type == "stage") {
    std::ofstream file(directory / "planner_pipeline_timing.csv",
                       std::ios::app);
    csv(file, {scalar(event.at("stage")), scalar(event.at("iteration")),
               scalar(event.at("frame_id")), number(event.at("duration_ms")),
               event.at("source_stamp_s").is_null()
                   ? ""
                   : number(event.at("source_stamp_s"), 9),
               scalar(event.at("details"))});
  } else if (type == "depth_pose") {
    std::ofstream file(directory / "planner_depth_poses.csv", std::ios::app);
    auto &p = event.at("position");
    csv(file,
        {scalar(event.at("sequence")), number(event.at("depth_time_s")),
         number(p[0]), number(p[1]), number(p[2]), number(event.at("roll")),
         number(event.at("pitch")), number(event.at("yaw"))});
  } else if (type == "iteration") {
    int iteration = event.at("iteration");
    bool published = event.at("published");
    Json record = {{"iteration", iteration},
                   {"elapsed_ms", event.at("elapsed_ms")},
                   {"vehicle_position", event.at("vehicle_position")},
                   {"published", published},
                   {"route_blocked", event.at("route_blocked")},
                   {"candidates", Json::array()}};
    auto targets = points(event.at("selected_targets"));
    int rank = 0;
    for (auto &candidate : event.at("candidates")) {
      auto c = candidate;
      c["rank"] = rank++;
      bool selected = false;
      for (auto &target : targets)
        if ((vec3(c.at("target")) - target).squaredNorm() < 1e-8)
          selected = true;
      c["trajectory_id"] =
          published && selected ? Json(iteration + 1) : Json(nullptr);
      record["candidates"].push_back(c);
    }
    std::ofstream iteration_file(directory / "planner_nbv_iterations.jsonl",
                                 std::ios::app);
    iteration_file << record.dump() << '\n';
    if (!event.at("trajectory").empty()) {
      std::ofstream file(directory / "planner_future_trajectories.csv",
                         std::ios::app);
      int index = 0;
      for (auto &v : event.at("trajectory"))
        csv(file, {std::to_string(iteration), std::to_string(index++),
                   number(v[0]), number(v[1]), number(v[2]), number(v[3])});
    }
  }
}
} // namespace inspection::migration
