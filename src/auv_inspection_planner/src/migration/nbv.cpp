#include "auv_inspection_planner/migration/core.hpp"
#include "scipy/ckdtree/ckdtree_decl.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <open3d/geometry/KDTreeSearchParam.h>
#include <open3d/geometry/PointCloud.h>
#include <stdexcept>

extern "C" void dsyevd_(char *, char *, int *, double *, int *, double *,
                        double *, int *, int *, int *, int *);
namespace inspection::migration {
namespace {
class Tree {
public:
  explicit Tree(const Points &points) : indices_(points.size()) {
    min_.fill(INFINITY);
    max_.fill(-INFINITY);
    for (const auto &p : points) {
      if (!p.allFinite())
        throw std::invalid_argument(
            "data must be finite, check for nan or inf values");
      for (int a = 0; a < 3; ++a) {
        data_.push_back(p[a]);
        min_[a] = std::min(min_[a], p[a]);
        max_[a] = std::max(max_[a], p[a]);
      }
    }
    std::iota(indices_.begin(), indices_.end(), 0);
    tree_ = {
        &nodes_, nullptr,     data_.data(), intptr_t(points.size()), 3,
        10,      max_.data(), min_.data(),  indices_.data(),         nullptr,
        0};
    auto hi = max_, lo = min_;
    build_ckdtree(&tree_, 0, points.size(), hi.data(), lo.data(), 1, 1);
    tree_.ctree = nodes_.data();
    tree_.size = nodes_.size();
    fix(tree_.ctree);
  }
  std::pair<std::vector<double>, std::vector<intptr_t>>
  query(const V3 &p, int k = 1, double bound = INFINITY) const {
    std::vector<double> d(k);
    std::vector<intptr_t> ii(k), ks(k);
    std::iota(ks.begin(), ks.end(), 1);
    query_knn(&tree_, d.data(), ii.data(), p.data(), 1, ks.data(), k, k, 0, 2,
              bound);
    return {d, ii};
  }
  double nearest(const V3 &p) const { return query(p).first[0]; }
  std::vector<intptr_t> ball(const V3 &p, double radius) const {
    std::vector<intptr_t> out;
    query_ball_point(&tree_, p.data(), &radius, 2, 0, 1, &out, false, true);
    return out;
  }

private:
  void fix(ckdtreenode *n) {
    if (n->split_dim == -1) {
      n->less = n->greater = nullptr;
      return;
    }
    n->less = tree_.ctree + n->_less;
    n->greater = tree_.ctree + n->_greater;
    fix(n->less);
    fix(n->greater);
  }
  ckdtree tree_;
  std::vector<ckdtreenode> nodes_;
  std::vector<double> data_;
  std::vector<intptr_t> indices_;
  std::array<double, 3> min_, max_;
};
template <class F> auto timed(const std::string &name, Timing t, F f) {
  double start = monotonic();
  auto value = f();
  if (t)
    t(name, (monotonic() - start) * 1000);
  return value;
}
bool close(double a, double b) {
  return std::abs(a - b) <= 1e-9 + 1e-6 * std::abs(b);
}
V3 axis(const V3 &target, const V3 &pos, const Tree &tree, double standoff,
        int steps) {
  if (standoff <= 0 || steps < 1)
    throw std::invalid_argument(
        "standoff and view-axis azimuth_steps must be positive");
  double yaw = atan2(pos.y() - target.y(), pos.x() - target.x());
  Points axes;
  std::vector<double> mins, means, alignments;
  for (int i = 0; i < steps; ++i) {
    double angle = yaw + i * (2 * pi / steps);
    V3 a(cos(angle) / sqrt(2), sin(angle) / sqrt(2), 1 / sqrt(2));
    axes.push_back(a);
    double low = INFINITY, sum = 0;
    for (double d : linspace(.2 * standoff, standoff, 20)) {
      double v = tree.nearest(target + a * d);
      low = std::min(low, v);
      sum += v;
    }
    mins.push_back(low);
    means.push_back(sum / 20);
    alignments.push_back(cos(angle - yaw));
  }
  double bestmin = *std::max_element(mins.begin(), mins.end()),
         bestmean = -INFINITY;
  for (int i = 0; i < steps; ++i)
    if (close(mins[i], bestmin))
      bestmean = std::max(bestmean, means[i]);
  int winner = 0;
  double align = -INFINITY;
  for (int i = 0; i < steps; ++i)
    if (close(mins[i], bestmin) && close(means[i], bestmean) &&
        alignments[i] > align) {
      winner = i;
      align = alignments[i];
    }
  return axes[winner];
}
} // namespace
std::optional<Cloud> prepare_cloud(const Points &points, const V3 &pos,
                                   const Json &p, Timing timing) {
  open3d::geometry::PointCloud raw;
  for (auto &v : points)
    if (v.allFinite())
      raw.points_.push_back(v);
  auto cloud = timed("nbv_downsample", timing, [&] {
    return raw.VoxelDownSample(p.at("voxel_downsample_size"));
  });
  int minimum = std::max({3, p.at("cluster_min_samples").get<int>(),
                          p.at("frontier_min_neighbors").get<int>() + 1});
  if (int(cloud->points_.size()) < minimum)
    return {};
  cloud = timed("nbv_outlier_filter", timing, [&] {
    auto filtered = cloud->RemoveStatisticalOutliers(p.at("outlier_neighbors"),
                                                     p.at("outlier_std_ratio"));
    Tree tree(cloud->points_);
    std::vector<bool> keep(cloud->points_.size());
    for (size_t i = 0; i < keep.size(); ++i)
      keep[i] = tree.query(cloud->points_[i], 4).first.back() <=
                p.at("normal_radius").get<double>();
    for (auto i : std::get<1>(filtered))
      keep[i] = true;
    std::vector<size_t> indices;
    for (size_t i = 0; i < keep.size(); ++i)
      if (keep[i])
        indices.push_back(i);
    return cloud->SelectByIndex(indices);
  });
  if (int(cloud->points_.size()) < minimum)
    return {};
  timed("nbv_normal_estimation", timing, [&] {
    cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
        p.at("normal_radius"), p.at("normal_max_neighbors")));
    return 0;
  });
  timed("nbv_normal_orientation", timing, [&] {
    cloud->OrientNormalsTowardsCameraLocation(pos);
    return 0;
  });
  return Cloud{cloud->points_, cloud->normals_};
}
std::vector<double> pca(const Points &points, int k,
                        const std::vector<int> *selected) {
  std::vector<double> out(points.size(), 0);
  if (points.size() < 3)
    return out;
  k = std::min(k, int(points.size()));
  if (k < 2)
    return out;
  Tree tree(points);
  std::vector<int> indices;
  if (selected)
    indices = *selected;
  else {
    indices.resize(points.size());
    std::iota(indices.begin(), indices.end(), 0);
  }
  for (int index : indices) {
    auto neighbors = tree.query(points[index], k).second;
    V3 mean = V3::Zero();
    for (int i : neighbors)
      mean += points[i];
    mean /= k;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i : neighbors) {
      V3 v = points[i] - mean;
      cov += v * v.transpose();
    }
    cov /= k - 1;
    char job = 'N', uplo = 'L';
    int n = 3, lda = 3, info = 0, lwork = 64, liwork = 32;
    double eigen[3], work[64];
    int iwork[32];
    dsyevd_(&job, &uplo, &n, cov.data(), &lda, eigen, work, &lwork, iwork,
            &liwork, &info);
    if (info)
      throw std::runtime_error("PCA eigensolve failed");
    double total = eigen[0] + eigen[1] + eigen[2];
    if (total > 0)
      out[index] = eigen[0] / total;
  }
  return out;
}
std::vector<int> frontiers(const Cloud &cloud, int k, double angle,
                           double radius, int min_neighbors,
                           const std::vector<int> *focus) {
  const auto &p = cloud.points;
  if (p.size() != cloud.normals.size())
    throw std::invalid_argument(
        "Frontier points and normals must be matching Nx3 arrays");
  if ((!std::isfinite(radius) && radius != INFINITY) || radius <= 0)
    throw std::invalid_argument("Frontier radius must be finite and positive");
  if (!std::isfinite(angle) || angle <= 0 || angle >= 360)
    throw std::invalid_argument(
        "Frontier angle threshold must be between 0 and 360 degrees");
  if (min_neighbors < 2 || k <= min_neighbors)
    throw std::invalid_argument(
        "Frontier k must exceed min_neighbors, which must be at least two");
  if (int(p.size()) <= min_neighbors)
    return {};
  std::vector<int> selected;
  if (focus)
    selected = *focus;
  else {
    selected.resize(p.size());
    std::iota(selected.begin(), selected.end(), 0);
  }
  Tree tree(p);
  std::vector<int> out;
  k = std::min(k, int(p.size()));
  for (int i : selected) {
    if (i < 0 || i >= int(p.size()))
      throw std::invalid_argument("Frontier focus indices must be a "
                                  "one-dimensional array of valid indices");
    V3 n = cloud.normals[i];
    double length = n.norm();
    if (!std::isfinite(length) || length <= 1e-12)
      n.setZero();
    else
      n /= length;
    V3 u = n.cross(V3::UnitZ());
    if (u.norm() < 1e-4)
      u = n.cross(V3::UnitY());
    u /= std::max(u.norm(), 1e-12);
    V3 v = n.cross(u);
    auto [d, ids] = tree.query(p[i], k, std::nextafter(radius, INFINITY));
    std::vector<double> angles;
    for (int j = 0; j < k; ++j) {
      if (!std::isfinite(d[j]))
        continue;
      V3 delta = p[ids[j]] - p[i];
      double x = delta.dot(u), y = delta.dot(v);
      if (std::hypot(x, y) >= 1e-5)
        angles.push_back(atan2(y, x));
    }
    if (int(angles.size()) < min_neighbors)
      continue;
    std::sort(angles.begin(), angles.end());
    double gap = angles.front() + 2 * pi - angles.back();
    for (size_t j = 1; j < angles.size(); ++j)
      gap = std::max(gap, angles[j] - angles[j - 1]);
    if (gap > angle * pi / 180)
      out.push_back(i);
  }
  return out;
}
std::vector<Cluster> cluster_frontiers(const Cloud &cloud,
                                       const std::vector<int> &input,
                                       double radius, double maxangle,
                                       int minimum) {
  if (input.empty())
    return {};
  if (!std::isfinite(radius) || radius <= 0 || minimum < 1)
    throw std::invalid_argument(
        "Cluster radius and minimum sample count must be positive");
  if (!std::isfinite(maxangle) || maxangle < 0 || maxangle > 180)
    throw std::invalid_argument(
        "Cluster normal angle must be between 0 and 180 degrees");
  Points p, n;
  std::vector<int> indices;
  for (int i : input) {
    double length = cloud.normals.at(i).norm();
    if (cloud.points.at(i).allFinite() && std::isfinite(length) &&
        length > 1e-12) {
      indices.push_back(i);
      p.push_back(cloud.points[i]);
      n.push_back(cloud.normals[i] / length);
    }
  }
  std::vector<std::vector<int>> neighborhoods(p.size());
  double dot = cos(maxangle * pi / 180);
  // query_ball_point's vector-input default returns neighbors sorted by input
  // index.
  Tree frontier_tree(p);
  for (size_t i = 0; i < p.size(); ++i)
    for (auto j : frontier_tree.ball(p[i], radius))
      if (n[j].dot(n[i]) >= dot - 1e-12)
        neighborhoods[i].push_back(j);
  std::vector<int> order(p.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    return std::make_tuple(-int(neighborhoods[a].size()), p[a].x(), p[a].y(),
                           p[a].z()) <
           std::make_tuple(-int(neighborhoods[b].size()), p[b].x(), p[b].y(),
                           p[b].z());
  });
  std::vector<bool> unassigned(p.size(), true);
  std::vector<Cluster> clusters;
  auto update = [](Cluster &c) {
    c.centroid.setZero();
    V3 normal = V3::Zero();
    for (auto &x : c.points)
      c.centroid += x;
    for (auto &x : c.normals)
      normal += x;
    c.centroid /= c.points.size();
    normal /= c.normals.size();
    if (normal.norm() > 1e-12)
      c.normal = normal / normal.norm();
  };
  for (int seed : order) {
    if (!unassigned[seed])
      continue;
    std::vector<int> ids;
    for (int i : neighborhoods[seed])
      if (unassigned[i])
        ids.push_back(i);
    if (int(ids.size()) < minimum)
      continue;
    Cluster c;
    c.id = clusters.size();
    c.normal = n[seed];
    for (int i : ids) {
      unassigned[i] = false;
      c.points.push_back(p[i]);
      c.normals.push_back(n[i]);
      c.indices.push_back(indices[i]);
    }
    update(c);
    clusters.push_back(c);
  }
  for (int i : order) {
    if (!unassigned[i])
      continue;
    std::vector<std::pair<double, int>> eligible;
    for (size_t j = 0; j < clusters.size(); ++j) {
      auto &c = clusters[j];
      double d = (c.centroid - p[i]).norm();
      if (c.normal.dot(n[i]) >= dot - 1e-12 &&
          d <= radius * (c.points.size() + 1) / c.points.size() + 1e-12)
        eligible.emplace_back(d, j);
    }
    std::stable_sort(eligible.begin(), eligible.end(),
                     [](auto a, auto b) { return a.first < b.first; });
    for (auto [distance, id] : eligible) {
      auto &c = clusters[id];
      V3 center = V3::Zero();
      for (auto &x : c.points)
        center += x;
      center = (center + p[i]) / (c.points.size() + 1);
      double far = (p[i] - center).norm();
      for (auto &x : c.points)
        far = std::max(far, (x - center).norm());
      if (far > radius + 1e-12)
        continue;
      c.points.push_back(p[i]);
      c.normals.push_back(n[i]);
      c.indices.push_back(indices[i]);
      update(c);
      break;
    }
  }
  return clusters;
}
V3 view_axis(const V3 &t, const V3 &p, const Points &occupied, double standoff,
             int steps) {
  return axis(t, p, Tree(occupied), standoff, steps);
}
Points cone(const V3 &t, const V3 &normal, double standoff,
            std::vector<double> polars, int steps) {
  V3 n = normal / normal.norm(), u = n.cross(V3::UnitZ());
  if (u.norm() < 1e-4)
    u = n.cross(V3::UnitY());
  u /= u.norm();
  V3 v = n.cross(u);
  Points out{t + n * standoff};
  for (double polar : polars)
    for (int i = 0; i < steps; ++i) {
      double a = i * (360.0 / steps) * pi / 180, b = polar * pi / 180;
      out.push_back(t + standoff * (sin(b) * cos(a) * u + sin(b) * sin(a) * v +
                                    cos(b) * n));
    }
  return out;
}
double seabed_weight(double z, std::optional<double> floor, double decay,
                     double strength) {
  if (!floor)
    return 1;
  if (!std::isfinite(*floor) || !std::isfinite(decay) ||
      !std::isfinite(strength) || decay <= 0 || strength < 0 || strength > 1)
    throw std::invalid_argument("seabed floor/decay must be finite, decay "
                                "positive, strength in [0, 1]");
  return 1 - strength * exp(-std::max(0., z - *floor) / decay);
}
Seabed::Seabed(double r) : resolution_(r) {
  if (!std::isfinite(r) || r <= 0)
    throw std::invalid_argument(
        "seabed map resolution must be finite and positive");
}
void Seabed::adjust(const Key &k, double z, int sign) {
  int band = k[2];
  counts_[band] += sign;
  sums_[band] += sign * z;
  std::array<int, 2> xy{k[0], k[1]};
  auto &c = xy_[band];
  c[xy] += sign;
  if (c[xy] <= 0)
    c.erase(xy);
  if (counts_[band] == 0) {
    counts_.erase(band);
    sums_.erase(band);
    xy_.erase(band);
  }
}
int Seabed::update(const Points &p) {
  std::map<Key, double> current;
  for (auto &v : p)
    if (v.allFinite())
      current[voxel(v, resolution_)] = v.z();
  int changed = 0;
  for (auto &[k, z] : voxels_)
    if (!current.count(k) || current[k] != z) {
      adjust(k, z, -1);
      ++changed;
    }
  for (auto &[k, z] : current)
    if (!voxels_.count(k) || voxels_[k] != z) {
      adjust(k, z, 1);
      ++changed;
    }
  voxels_ = current;
  if (changed == 0)
    return 0;
  mean_z.reset();
  sample_count = 0;
  int minimum = std::max(24, int(std::ceil(current.size() * .03)));
  auto count = [&](int b) {
    auto i = counts_.find(b);
    return i == counts_.end() ? 0 : i->second;
  };
  for (auto &[b, n] : counts_) {
    if (n < count(b - 1) || n < count(b + 1))
      continue;
    int support = 0;
    double sum = 0;
    std::set<std::array<int, 2>> xy;
    for (int band = b - 1; band <= b + 1; ++band)
      if (counts_.count(band)) {
        support += count(band);
        sum += sums_[band];
        for (auto &[k, c] : xy_[band])
          xy.insert(k);
      }
    if (support < minimum)
      continue;
    std::array<int, 2> lo = *xy.begin(), hi = lo;
    for (auto &k : xy)
      for (int a = 0; a < 2; ++a) {
        lo[a] = std::min(lo[a], k[a]);
        hi[a] = std::max(hi[a], k[a]);
      }
    int sx = hi[0] - lo[0] + 1, sy = hi[1] - lo[1] + 1;
    if (sx * resolution_ < 1 || sy * resolution_ < 1 ||
        double(xy.size()) / (sx * sy) < .15)
      continue;
    mean_z = sum / support;
    sample_count = support;
    break;
  }
  return changed;
}
std::vector<Candidate> nbv(const Points &raw, const V3 &pos, const V3 &heading,
                           const std::vector<std::pair<V3, double>> &history,
                           Json supplied, Timing timing,
                           Diagnostic diagnostic, NBVCache *cache) {
  auto log = [&](const std::string &s) {
    if (diagnostic)
      diagnostic("info", s);
  };
  log("Calculating NBV waypoints...");
  Json p = nbv_defaults();
  p.update(supplied);
  if (raw.empty()) {
    if (cache)
      cache->clusters.clear();
    return {};
  }
  std::optional<double> floor;
  int floor_count = 0;
  if (p["seabed_enabled"].get<bool>()) {
    if (p.contains("seabed_estimate_z")) {
      if (!p["seabed_estimate_z"].is_null())
        floor = p["seabed_estimate_z"];
      floor_count = p.value("seabed_sample_count", 0);
    } else {
      Seabed seabed(p["voxel_downsample_size"]);
      timed("nbv_seabed_estimation", timing,
            [&] { return seabed.update(raw); });
      floor = seabed.mean_z;
      floor_count = seabed.sample_count;
    }
  }
  if (floor) {
    seabed_weight(0, floor, p["seabed_decay_height"],
                  p["seabed_penalty_strength"]);
    log("NBV seabed floor Z=" + std::to_string(*floor) +
        "; map floor samples=" + std::to_string(floor_count));
  } else if (p["seabed_enabled"].get<bool>())
    log("NBV seabed: no supported floor layer yet; penalty disabled");
  Points filtered;
  double zmin = p["z_min_target"];
  for (auto &v : raw)
    if (v.z() > zmin)
      filtered.push_back(v);
  log("NBV target mask: " + std::to_string(raw.size()) +
      " occupied points -> " + std::to_string(filtered.size()) +
      " above z_min_target=" + std::to_string(zmin));
  if (filtered.empty()) {
    if (cache)
      cache->clusters.clear();
    return {};
  }
  auto prepared = prepare_cloud(filtered, pos, p, timing);
  if (!prepared) {
    log("NBV preprocessing left no usable surface from " +
        std::to_string(filtered.size()) + " points");
    if (!cache)
      return {};
  }
  Cloud cloud = prepared ? std::move(*prepared) : Cloud{};
  double radius = p["exploration_radius"];
  if (!std::isfinite(radius) || radius <= 0)
    throw std::invalid_argument(
        "NBV exploration radius must be finite and positive");
  std::vector<int> focus;
  for (size_t i = 0; i < cloud.points.size(); ++i)
    if ((cloud.points[i] - pos).norm() <= radius)
      focus.push_back(i);
  log("Focus Window: Processing " + std::to_string(focus.size()) + "/" +
      std::to_string(cloud.points.size()) + " points within " +
      std::to_string(radius) + "m");
  if (focus.empty() && !cache)
    return {};
  Tree occupied(raw), cleaned(cloud.points.empty() ? filtered : cloud.points);
  auto fs = timed("nbv_frontier_detection", timing, [&] {
    return frontiers(cloud, p["frontier_neighbors"],
                     p["frontier_angle_threshold_deg"], p["frontier_radius"],
                     p["frontier_min_neighbors"], &focus);
  });
  auto clusters = timed("nbv_clustering", timing, [&] {
    return cluster_frontiers(cloud, fs, p["cluster_fov_radius"],
                             p["cluster_max_angle_deg"],
                             p["cluster_min_samples"]);
  });
  int count = 0;
  std::set<int> unique;
  for (auto &c : clusters) {
    count += c.points.size();
    unique.insert(c.indices.begin(), c.indices.end());
  }
  log("NBV frontier coverage: " + std::to_string(filtered.size()) +
      " input points -> " + std::to_string(cloud.points.size()) +
      " surface points -> " + std::to_string(fs.size()) + " frontiers -> " +
      std::to_string(count) + " clustered points in " +
      std::to_string(clusters.size()) + " clusters");
  if (clusters.empty() && !cache)
    return {};
  std::vector<int> indices(unique.begin(), unique.end());
  auto complexity = timed("nbv_pca", timing, [&] {
    return pca(cloud.points, p["pca_neighbors"], &indices);
  });
  std::vector<CachedCluster> refreshed;
  for (auto &c : clusters) {
    std::vector<double> values;
    for (int index : c.indices)
      values.push_back(complexity[index]);
    if (cache)
      c.id = cache->next_cluster_id++;
    refreshed.push_back({std::move(c), std::move(values)});
  }
  if (cache) {
    std::vector<CachedCluster> retained;
    int updated = 0, removed = 0;
    double support_radius = std::max(.2, p["voxel_downsample_size"].get<double>());
    for (auto &entry : cache->clusters) {
      // Replace entries touched by the local refresh, including vanished frontiers.
      bool local = std::any_of(
          entry.geometry.points.begin(), entry.geometry.points.end(),
          [&](const V3 &v) { return (v - pos).norm() <= radius; });
      if (local) {
        ++updated;
        continue;
      }
      bool supported = std::any_of(
          entry.geometry.points.begin(), entry.geometry.points.end(),
          [&](const V3 &v) {
            return v.z() > zmin && occupied.nearest(v) <= support_radius;
          });
      if (supported)
        retained.push_back(std::move(entry));
      else
        ++removed;
    }
    log("NBV cluster cache: " + std::to_string(refreshed.size()) +
        " refreshed, " + std::to_string(retained.size()) +
        " distant retained, " + std::to_string(updated) +
        " local replaced, " + std::to_string(removed) + " unsupported removed");
    for (auto &entry : refreshed)
      retained.push_back(std::move(entry));
    cache->clusters = std::move(retained);
  }
  const auto &global_clusters = cache ? cache->clusters : refreshed;
  double started = monotonic();
  std::vector<Candidate> all;
  int visited = 0, generated = 0, rejected = 0;
  for (auto &entry : global_clusters) {
    const auto &c = entry.geometry;
    bool seen = false;
    for (auto &[hpos, hyaw] : history) {
      V3 delta = c.centroid - hpos;
      double d = delta.norm(), a = atan2(delta.y(), delta.x()) - hyaw;
      double diff = std::abs(atan2(sin(a), cos(a)));
      if (!(d > p["visited_min_distance"].get<double>() &&
            d < p["visited_max_distance"].get<double>() &&
            diff < p["visited_yaw_threshold_deg"].get<double>() * pi / 180))
        continue;
      if (d <= .6) {
        seen = true;
        break;
      }
      int n = int((d - .4) / .2);
      if (n < 1) {
        seen = true;
        break;
      }
      double clearance = INFINITY;
      for (double f : linspace(.2 / d, (d - .4) / d, n))
        clearance = std::min(clearance, occupied.nearest(hpos + f * delta));
      if (clearance > .3) {
        seen = true;
        break;
      }
    }
    if (seen) {
      ++visited;
      continue;
    }
    auto axis_ = axis(c.centroid, pos, occupied, p["standoff"],
                      p["view_axis_azimuth_steps"]);
    auto views = cone(c.centroid, axis_, p["standoff"],
                      p["polar_angles_deg"].get<std::vector<double>>(),
                      p["azimuth_steps"]);
    generated += views.size();
    for (auto &vp : views) {
      double gain = 0;
      for (size_t i = 0; i < c.points.size(); ++i) {
        V3 ray = c.points[i] - vp;
        double distance = ray.norm();
        if (distance < 1e-4)
          continue;
        double incidence =
            std::clamp((-ray / distance).dot(c.normals[i]), 0., 1.);
        double weight =
            1 + p["complexity_weight"].get<double>() * entry.complexities[i];
        gain += incidence * weight *
                seabed_weight(c.points[i].z(), floor, p["seabed_decay_height"],
                              p["seabed_penalty_strength"]);
      }
      if (gain < p["min_raw_gain"].get<double>()) {
        ++rejected;
        continue;
      }
      V3 travel = vp - pos, view = c.centroid - vp;
      double d = travel.norm(), vd = view.norm(), turn1 = 0, turn2 = 0;
      if (d > 1e-4) {
        turn1 = acos(std::clamp((travel / d).dot(heading), -1., 1.));
        if (vd > 1e-4)
          turn2 = acos(std::clamp((travel / d).dot(view / vd), -1., 1.));
      }
      double nearest = cleaned.nearest(vp),
             risk = nearest < p["collision_distance_threshold"].get<double>()
                        ? 1 / (nearest +
                               p["collision_distance_offset"].get<double>())
                        : 0;
      double cost = p["distance_weight"].get<double>() * d +
                    p["rotation_weight"].get<double>() * (turn1 + turn2) +
                    p["collision_weight"].get<double>() * risk;
      all.push_back({vp, c.centroid, 0, gain, cost, c.id, true});
    }
  }
  log("NBV candidate filtering: " + std::to_string(visited) + "/" +
      std::to_string(global_clusters.size()) +
      " clusters suppressed by pose history; " + std::to_string(generated) +
      " viewpoints generated, " + std::to_string(rejected) +
      " below minimum raw gain, " + std::to_string(all.size()) + " retained");
  if (timing)
    timing("nbv_candidate_evaluation", (monotonic() - started) * 1000);
  if (all.empty())
    return {};
  // Runtime mission eligibility must be applied before extrema, utility and
  // the per-cluster shortlist. Keep the legacy stateless reference unchanged.
  if (p.value("defer_utility_normalization", false))
    return all;
  double ming = INFINITY, maxg = -INFINITY, minc = INFINITY, maxc = -INFINITY;
  for (auto &c : all) {
    ming = std::min(ming, c.raw_gain);
    maxg = std::max(maxg, c.raw_gain);
    minc = std::min(minc, c.raw_cost);
    maxc = std::max(maxc, c.raw_cost);
  }
  std::map<int, std::vector<Candidate>> grouped;
  for (auto &c : all) {
    double ng = maxg - ming < 1e-6 ? 0 : (c.raw_gain - ming) / (maxg - ming),
           nc = maxc - minc < 1e-6 ? 0 : (c.raw_cost - minc) / (maxc - minc);
    c.utility = p["utility_gain_weight"].get<double>() * ng -
                p["utility_cost_weight"].get<double>() * nc;
    grouped[c.cluster_id].push_back(c);
  }
  auto better = [](const Candidate &a, const Candidate &b) {
    return a.utility > b.utility;
  };
  std::vector<Candidate> out;
  for (auto &[id, cs] : grouped) {
    std::stable_sort(cs.begin(), cs.end(), better);
    out.insert(out.end(), cs.begin(),
               cs.begin() + std::min(size_t(3), cs.size()));
  }
  std::stable_sort(out.begin(), out.end(), better);
  return out;
}
} // namespace inspection::migration
