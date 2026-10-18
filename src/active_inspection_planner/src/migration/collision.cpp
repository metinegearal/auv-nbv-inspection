#include "active_inspection_planner/migration/core.hpp"
#include <algorithm>
#include <cmath>

namespace inspection::migration {
namespace {
V3 lower(const Key &k, double r) { return V3(k[0], k[1], k[2]) * r; }

// Slab intersection is continuous: even a short corner crossing is rejected.
bool intersects(const V3 &a, const V3 &b, const V3 &lo, const V3 &hi) {
  double enter = 0, leave = 1;
  V3 direction = b - a;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1e-12) {
      if (a[axis] < lo[axis] || a[axis] > hi[axis])
        return false;
    } else {
      double t0 = (lo[axis] - a[axis]) / direction[axis];
      double t1 = (hi[axis] - a[axis]) / direction[axis];
      enter = std::max(enter, std::min(t0, t1));
      leave = std::min(leave, std::max(t0, t1));
      if (enter > leave)
        return false;
    }
  }
  return true;
}
} // namespace

Voxels inflate_clearance(const Voxels &occupied, double radius, double r) {
  if (!(r > 0) || !std::isfinite(r) || !(radius >= 0) || !std::isfinite(radius))
    throw std::invalid_argument("invalid collision clearance");
  if (radius == 0)
    return occupied;
  int n = int(std::ceil(radius / r)) + 1;
  std::vector<Key> offsets;
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z) {
        V3 gap(std::max(0, std::abs(x) - 1), std::max(0, std::abs(y) - 1),
               std::max(0, std::abs(z) - 1));
        if (gap.squaredNorm() * r * r <= radius * radius + 1e-12)
          offsets.push_back({x, y, z});
      }
  Voxels out;
  for (const auto &k : occupied)
    for (const auto &o : offsets)
      out.insert({k[0] + o[0], k[1] + o[1], k[2] + o[2]});
  return out;
}

double obstacle_clearance(const V3 &p, const Voxels &occupied, double r) {
  double distance = INFINITY;
  for (const auto &k : occupied) {
    V3 lo = lower(k, r), hi = lo + V3::Constant(r);
    V3 delta = (lo - p).cwiseMax(p - hi).cwiseMax(0.);
    distance = std::min(distance, delta.squaredNorm());
  }
  return std::sqrt(distance);
}

bool clearance_free(const Points &path, const Voxels &occupied, double r,
                    double radius) {
  if (path.empty() || !(r > 0) || !(radius >= 0))
    return false;
  for (const auto &p : path)
    if (!p.allFinite())
      return false;
  // A box enclosing the vehicle sphere is deliberately conservative here.
  for (size_t i = 0; i < path.size(); ++i)
    for (const auto &k : occupied) {
      V3 lo = lower(k, r) - V3::Constant(radius);
      V3 hi = lower(k, r) + V3::Constant(r + radius);
      if (intersects(path[i ? i - 1 : 0], path[i], lo, hi))
        return false;
    }
  return true;
}

bool retreat_safe(const Points &path, const Voxels &occupied, double r,
                  double radius) {
  if (!clearance_free(path, occupied, r, radius))
    return false;
  double previous = obstacle_clearance(path.front(), occupied, r);
  for (size_t i = 1; i < path.size(); ++i) {
    int steps =
        std::max(1, int(std::ceil((path[i] - path[i - 1]).norm() / (r * .25))));
    for (int step = 1; step <= steps; ++step) {
      V3 p = path[i - 1] + (double(step) / steps) * (path[i] - path[i - 1]);
      double distance = obstacle_clearance(p, occupied, r);
      if (distance + 1e-6 < previous)
        return false;
      previous = distance;
    }
  }
  return true;
}

Points clearance_retreat(const V3 &start, const Voxels &raw,
                         const Voxels &inflated, double r, double radius,
                         double margin) {
  if (!start.allFinite() || !inflated.count(voxel(start, r)))
    return {};
  // Leave extra room so stopping short of the endpoint still clears the buffer.
  double target_clearance =
      radius + margin + std::sqrt(3.) * r + std::max(.5, 2 * r);
  double limit = 2 * target_clearance;
  Voxels nearby;
  for (const auto &k : raw)
    if ((lower(k, r) + V3::Constant(r * .5) - start).norm() <=
        limit + target_clearance + 2 * r)
      nearby.insert(k);
  if (!retreat_safe({start}, nearby, r, radius))
    return {};
  Points best;
  double best_length = INFINITY;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        V3 direction = V3(x, y, z).normalized();
        for (double length = r; length <= std::min(limit, best_length) + 1e-9;
             length += r) {
          V3 end = start + length * direction;
          if (inflated.count(voxel(end, r)) ||
              obstacle_clearance(end, nearby, r) < target_clearance)
            continue;
          Points path{start, end};
          if (retreat_safe(path, nearby, r, radius)) {
            best = path;
            best_length = length;
            break;
          }
        }
      }
  return best;
}
} // namespace inspection::migration
