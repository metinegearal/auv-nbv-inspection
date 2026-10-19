#include "auv_inspection_planner/migration/core.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
extern "C" {
#include "scipy/lbfgsb.h"
}
namespace inspection::migration {
namespace {
// NumPy uses an eight-accumulator pairwise reduction for contiguous sums.
double pairwise(const double *p, size_t n) {
  if (n < 8) {
    double result = -0.;
    for (size_t i = 0; i < n; ++i)
      result += p[i];
    return result;
  }
  if (n <= 128) {
    double r[8];
    std::copy(p, p + 8, r);
    size_t i = 8;
    for (; i < n - (n % 8); i += 8)
      for (int j = 0; j < 8; ++j)
        r[j] += p[i + j];
    double result =
        ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; ++i)
      result += p[i];
    return result;
  }
  size_t middle = n / 2;
  middle -= middle % 8;
  return pairwise(p, middle) + pairwise(p + middle, n - middle);
}
double sum(const std::vector<double> &v) {
  return pairwise(v.data(), v.size());
}
double dot3(const V3 &a, const V3 &b) {
  return (a.x() * b.x() + a.y() * b.y()) + a.z() * b.z();
}
double norm3(const V3 &v) { return sqrt(dot3(v, v)); }
} // namespace
V3 velocity(const std::deque<Pose> &history, double window) {
  if (history.size() < 2)
    return V3::Zero();
  double latest = history.back().time, mean_time = 0;
  V3 mean = V3::Zero();
  int count = 0;
  for (auto &h : history)
    if (latest - h.time >= 0 && latest - h.time <= window) {
      if (!h.position.allFinite())
        return V3::Zero();
      mean_time += h.time - latest;
      mean += h.position;
      ++count;
    }
  if (count < 2)
    return V3::Zero();
  mean_time /= count;
  mean /= count;
  double denominator = 0;
  V3 numerator = V3::Zero();
  for (auto &h : history)
    if (latest - h.time >= 0 && latest - h.time <= window) {
      double t = h.time - latest - mean_time;
      denominator += t * t;
      numerator += t * (h.position - mean);
    }
  if (denominator < 1e-9)
    return V3::Zero();
  return numerator / denominator;
}
double continuity_cost(const Trajectory &t, const V3 &velocity,
                       const Points &recent, MotionWeights w) {
  if (t.size() < 2)
    return 0;
  Points directions;
  std::vector<double> lengths;
  for (size_t i = 1; i < t.size(); ++i) {
    V3 step = t[i].head<3>() - t[i - 1].head<3>();
    double len = norm3(step);
    if (len > 1e-6) {
      directions.push_back(step / std::max(len, 1e-6));
      lengths.push_back(len);
    }
  }
  double cost = 0, speed = norm3(velocity);
  if (speed > .05) {
    double weights[] = {1, .5, .25};
    std::vector<double> entry;
    for (size_t i = 0; i < std::min(size_t(3), directions.size()); ++i) {
      double a = 1 - dot3(directions[i], velocity / speed);
      entry.push_back(weights[i] * (a * a));
    }
    cost += w.continuity * std::min(speed / .3, 1.) * sum(entry);
  }
  double distance = lengths.empty() ? 0 : lengths.front();
  std::vector<double> turns, reversals;
  for (size_t i = 1; i < directions.size(); ++i) {
    double a = dot3(directions[i], directions[i - 1]);
    reversals.push_back(std::pow(std::max(0., -a), 2));
    distance += lengths[i];
    if (distance <= 10)
      turns.push_back(atan2(norm3(directions[i - 1].cross(directions[i])), a));
  }
  if (directions.size() > 1) {
    cost += w.reversal * sum(reversals);
    cost += w.reversal * std::pow(std::max(0., sum(turns) - pi), 2);
  }
  std::vector<double> yaw;
  for (auto &v : t)
    yaw.push_back(v.w());
  unwrap(yaw);
  std::vector<double> distances, squares;
  for (size_t i = 1; i < yaw.size(); ++i) {
    double d = yaw[i] - yaw[i - 1];
    distances.push_back(std::abs(d));
    squares.push_back(d * d);
  }
  double diff = yaw.back() - yaw.front();
  double excess =
      std::max(0., sum(distances) - std::abs(atan2(sin(diff), cos(diff))) - pi);
  cost += w.reversal * (sum(squares) + excess * excess);
  Points history;
  for (auto &v : recent)
    if (norm3(v - t.front().head<3>()) > 2 * w.radius)
      history.push_back(v);
  std::vector<double> history_cost;
  if (!history.empty() && w.radius > 0) {
    for (size_t i = 1; i < t.size(); ++i) {
      double d = INFINITY;
      for (auto &v : history)
        d = std::min(d, norm3(v - t[i].head<3>()));
      history_cost.push_back(std::pow(std::max(0., 1 - d / w.radius), 2));
    }
    cost += w.history * sum(history_cost);
  }
  return cost;
}
Trajectory optimize(const Trajectory &input, const V3 &position, double yaw,
                    const V3 &vel, const Voxels &occupied, double resolution,
                    const Points &recent, MotionWeights weights,
                    double deviation, int iterations, double factor) {
  Trajectory ref = input;
  if (ref.size() < 2)
    return ref;
  ref[0].head<3>() = position;
  std::vector<double> y{yaw};
  for (auto &v : ref)
    y.push_back(v.w());
  unwrap(y);
  for (size_t i = 0; i < ref.size(); ++i)
    ref[i].w() = y[i + 1];
  ref[0].w() = yaw;
  if (!collision_free(positions(ref), occupied, resolution, factor))
    return {};
  std::vector<double> indices;
  for (double d :
       linspace(0, ref.size() - 1, std::min(size_t(32), ref.size()))) {
    double i = std::nearbyint(d);
    if (indices.empty() || indices.back() != i)
      indices.push_back(i);
  }
  std::vector<double> lengths;
  for (size_t i = 1; i < ref.size(); ++i)
    lengths.push_back(norm3(ref[i].head<3>() - ref[i - 1].head<3>()));
  // scipy.minimize removes equal-bound variables before invoking L-BFGS-B.
  std::vector<int> free;
  std::vector<double> lower, upper;
  for (size_t i = 1; i < indices.size(); ++i)
    for (int a = 0; a < 4; ++a) {
      if (i == indices.size() - 1 && a < 3)
        continue;
      if (a < 3 && deviation == 0)
        continue;
      free.push_back(i * 4 + a);
      lower.push_back(a == 3 ? -pi : -deviation);
      upper.push_back(a == 3 ? pi : deviation);
    }
  auto decode = [&](const std::vector<double> &x) {
    std::vector<double> controls(indices.size() * 4, 0);
    for (size_t i = 0; i < free.size(); ++i)
      controls[free[i]] = x[i];
    Trajectory out = ref;
    for (int a = 0; a < 4; ++a) {
      std::vector<double> values;
      for (size_t i = 0; i < indices.size(); ++i)
        values.push_back(controls[i * 4 + a]);
      for (size_t i = 0; i < ref.size(); ++i)
        out[i][a] += interp(i, indices, values);
    }
    return out;
  };
  auto objective = [&](const std::vector<double> &x) {
    auto t = decode(x);
    std::vector<double> fidelity, observation, smoothness, length_cost;
    Points steps;
    for (size_t i = 0; i < t.size(); ++i) {
      for (int a = 0; a < 3; ++a)
        fidelity.push_back(std::pow(t[i][a] - ref[i][a], 2));
      observation.push_back(std::pow(t[i].w() - ref[i].w(), 2));
      if (i) {
        V3 step = t[i].head<3>() - t[i - 1].head<3>();
        steps.push_back(step);
        length_cost.push_back(std::pow(
            (norm3(step) - lengths[i - 1]) / std::max(lengths[i - 1], .1), 2));
      }
    }
    for (size_t i = 1; i < steps.size(); ++i)
      for (int a = 0; a < 3; ++a)
        smoothness.push_back(std::pow(steps[i][a] - steps[i - 1][a], 2));
    return 2 * sum(fidelity) + .5 * (sum(observation) / t.size()) +
           sum(smoothness) + 4 * sum(length_cost) +
           continuity_cost(t, vel, recent, weights);
  };
  int n = free.size(), m = 10;
  if (n == 0)
    return ref;
  std::vector<double> x(n, 0), g(n),
      wa(2 * m * n + 5 * n + 11 * m * m + 8 * m, 0);
  std::vector<int> nbd(n, 2), iwa(3 * n, 0);
  int task[2] = {}, lsave[4] = {}, isave[44] = {}, ln_task[2] = {};
  double dsave[29] = {}, f = 0;
  int nit = 0, nfev = 0;
  while (true) {
    setulb(n, m, x.data(), lower.data(), upper.data(), nbd.data(), &f, g.data(),
           1e7, 1e-5, wa.data(), iwa.data(), task, lsave, isave, dsave, 20,
           ln_task);
    if (task[0] == 3) {
      f = objective(x);
      ++nfev;
      for (int i = 0; i < n; ++i) {
        double h = 1e-8;
        double lo = x[i] - lower[i], hi = upper[i] - x[i];
        if (x[i] + h > upper[i]) {
          if (h <= std::max(lo, hi))
            h = -h;
          else
            h = hi >= lo ? hi : -lo;
        }
        auto trial = x;
        trial[i] += h;
        double dx = trial[i] - x[i];
        if (dx == 0) {
          h = sqrt(std::numeric_limits<double>::epsilon()) *
              (x[i] >= 0 ? 1 : -1) * std::max(1., std::abs(x[i]));
          trial[i] = x[i] + h;
          dx = trial[i] - x[i];
        }
        g[i] = (objective(trial) - f) / dx;
        ++nfev;
      }
    } else if (task[0] == 1) {
      ++nit;
      if (nit >= iterations) {
        task[0] = 5;
        task[1] = 504;
      }
      if (nfev > 12000) {
        task[0] = 5;
        task[1] = 502;
      }
    } else
      break;
  }
  if (std::all_of(x.begin(), x.end(),
                  [](double d) { return std::isfinite(d); })) {
    std::vector<double> zero(n, 0);
    double initial = objective(zero);
    for (double scale : {1., .5, .25}) {
      auto trial = x;
      for (auto &d : trial)
        d *= scale;
      auto t = decode(trial);
      if (objective(trial) < initial &&
          collision_free(positions(t), occupied, resolution, factor))
        return t;
    }
  }
  return ref;
}
} // namespace inspection::migration
