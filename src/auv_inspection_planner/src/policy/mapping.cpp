#include "auv_inspection_planner/use_cases/mapping.hpp"
#include <algorithm>
#include <numeric>
#include <sstream>
namespace inspection
{
void MapUpdate::generate_global_point_cloud(
  const DepthImage & depth_image, const ColorImage & color_image,
  const Eigen::Vector3d & location, const Eigen::Matrix3d & rot_matrix,
  Cloud & out_cloud, Colors & out_colors)
{
  int height = depth_image.rows;
  int width = depth_image.cols;
  double fx = (width / 2.0) / std::tan((config_.fov_deg * M_PI / 180.0) / 2.0);
  double fy = fx;
  double cx = width / 2.0;
  double cy = height / 2.0;

  double pitch_rad = 45.0 * M_PI / 180.0;
  double cos_p = std::cos(pitch_rad);
  double sin_p = std::sin(pitch_rad);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float Z = depth_image.at(v, u);       // Assuming 32FC1 depth map
      if (Z > config_.min_depth && Z < config_.max_depth) {
                // Pinhole camera math (Equivalent to numpy meshgrid logic)
        double X = (u - cx) * Z / fx;
        double Y = (v - cy) * Z / fy;

                // Local rotation (Vehicle Y must be -X)
        double local_x = Z * cos_p - Y * sin_p;
        double local_y = -X;
        double local_z = -Z * sin_p - Y * cos_p;

        Eigen::Vector3d local_pt(local_x, local_y, local_z);
        Eigen::Vector3d global_pt = rot_matrix * local_pt + location;

        out_cloud.emplace_back(global_pt.x(), global_pt.y(), global_pt.z());

                // Colors
        auto color = color_image.at(v, u);
                // OpenCV is BGR by default, swap to RGB
        out_colors.push_back({color[2], color[1], color[0]});
      }
    }
  }
}

void MapUpdate::process_frame(
  const Eigen::Vector3d & location, double yaw, double pitch, double roll,
  const ColorImage & rgb_frame, const DepthImage & depth_map)
{
    // Apply camera offset exactly as in Python
  Eigen::Vector3d offset(0.35, 0.1125 + 0.275, -0.5);
  Eigen::Vector3d cam_location = location + offset;

    // Rotation Matrix
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix <<
    cy * cp, cy *sp *sr - sy * cr, cy *sp *cr + sy * sr,
    sy *cp, sy *sp *sr + cy * cr, sy *sp *cr - cy * sr,
    -sp, cp *sr, cp *cr;

  Cloud cloud;
  Colors colors;

    // Pass to projection
  generate_global_point_cloud(depth_map, rgb_frame, cam_location, rotation_matrix, cloud, colors);

  if (cloud.size() > 0) {
    writer_.insert(cloud, colors, cam_location, config_.max_depth);

    size_t current_leaves = reader_.count();
    size_t exploration_score = current_leaves - previous_leaves_;
    (void)exploration_score;
    previous_leaves_ = current_leaves;

        // You can log exploration_score here as you did with the Python file writing
  }
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector4d>>
CandidatePlanner::generate_candidate_viewpoints(const Eigen::Vector3d & current_position)
{
  Eigen::Vector3d current_pos_offset = current_position + Eigen::Vector3d(0.35, 0.1125 + 0.275,
      -0.5);

  {std::ostringstream message;
    message << "\n[DEBUG 1] Starting generate_candidate_viewpoints." << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message   << "[DEBUG 1] Current Offset Pos: [" << current_pos_offset.x() << ", "
              << current_pos_offset.y() << ", " <<
      current_pos_offset.z() << "]" << std::endl; diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector3d> occupied_points;
  std::vector<Eigen::Vector3d> empty_points;
  VoxelSet occ_set;

  int close_points_filtered = 0;

    // Extract points directly from OctoMap
  for (const auto & leaf : reader_.leaves()) {
    Eigen::Vector3d pt = leaf.point;

    if ((pt - current_pos_offset).norm() < 0.6) {
      close_points_filtered++;
      continue;
    }

    if (leaf.occupied) {
      occupied_points.push_back(pt);
      occ_set.insert({
          static_cast<int>(std::floor(pt.x() / resolution_)),
          static_cast<int>(std::floor(pt.y() / resolution_)),
          static_cast<int>(std::floor(pt.z() / resolution_))
            });
    } else {
      empty_points.push_back(pt);
    }
  }

  {std::ostringstream message; message << "[DEBUG 2] OctoMap Extraction:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Filtered (dist < 0.6): " << close_points_filtered << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Occupied points: " << occupied_points.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message; message << "  -> Empty points: " << empty_points.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

  if (occupied_points.empty()) {
    {std::ostringstream message;
      message << "[WARN] No occupied points found! Aborting." << std::endl;
      diagnostics_.text(message.str(), false, true);}
    return {{}, {}};
  }

    // ==========================================
    // 1. Calculate Information Gain (Surface Sparsity via PCL)
    // ==========================================
  std::vector<Eigen::Vector3d> occupied_for_index;
  for (const auto & p : occupied_points) {
    if (p.z() > config_.z_min_target) {     // z_min_target
      occupied_for_index.push_back(p);
    }
  }

  auto occ_cloud = indexes_.create(occupied_for_index);

  {std::ostringstream message;
    message << "[DEBUG 3] Z-Filtering (Threshold: " << config_.z_min_target << "):" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Valid points remaining: " << occ_cloud->size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector3d> frontier_centers;
  std::vector<double> frontier_scores;

  if (occ_cloud->size() > 10) {


    std::vector<int> pointIdx(11);
    std::vector<float> pointDistSq(11);

    double max_rho = 0.0, min_rho = 1e9;
    std::vector<double> rhos(occ_cloud->size());

        // Calculate sparsity (rho) for each point
    for (size_t i = 0; i < occ_cloud->size(); ++i) {
      if (occ_cloud->nearest(i, 11, pointIdx, pointDistSq) > 0) {
        double mean_dist = 0.0;
        for (size_t j = 1; j < pointIdx.size(); ++j) {
          mean_dist += std::sqrt(pointDistSq[j]);
        }
        mean_dist /= 10.0;
        rhos[i] = mean_dist;
        max_rho = std::max(max_rho, mean_dist);
        min_rho = std::min(min_rho, mean_dist);
      }
    }

    std::vector<Eigen::Vector3d> free_for_index;
    for (const auto & p : empty_points) {
      free_for_index.push_back(p);
    }
    const bool has_free = !free_for_index.empty();
    auto free_cloud = indexes_.create(free_for_index);

    for (size_t i = 0; i < occ_cloud->size(); ++i) {
      double base_score = (rhos[i] - min_rho) / (max_rho - min_rho + 1e-6);
      if (base_score > config_.info_gain_threshold &&
        rhos[i] > (config_.resolution * config_.sparsity_multiplier))
      {

        double adjusted_score = base_score;
        if (has_free) {
          std::vector<int> freeIdx;
          std::vector<float> freeDistSq;
          int free_counts = free_cloud->radius(occ_cloud->point(i), config_.resolution * 3.0,
              freeIdx, freeDistSq);
          double damping = std::exp(-0.05 * free_counts);
          adjusted_score *= damping;
        }

        if (adjusted_score >= 0.05) {
          frontier_centers.push_back(Eigen::Vector3d(occ_cloud->point(i).x(),
              occ_cloud->point(i).y(), occ_cloud->point(i).z()));
          frontier_scores.push_back(adjusted_score);
        }
      }
    }
  }

  {std::ostringstream message; message << "[DEBUG 4] Frontier Extraction:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> High-value frontiers found: " << frontier_centers.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

    // ==========================================
    // 2. Evaluate Utility & Generate Viewpoints
    // ==========================================
    // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> candidates;
    // double alpha_cost = 1.0;
    // double min_utility_threshold = config_.min_utility_threshold;

    // std::vector<double> dists = {1.5, 1.2, 1.8, 1.0};
    // std::vector<double> pitches = {45.0, 60.0, 30.0, 75.0, 15.0};

    // int skipped_wall = 0;
    // int skipped_los = 0;
    // int skipped_utility = 0;

    // for (size_t i = 0; i < frontier_centers.size(); ++i) {
    //     Eigen::Vector3d f = frontier_centers[i];
    //     Eigen::Vector3d vec = f - current_pos_offset;
    //     vec.z() = 0;

    //     double norm = vec.norm();
    //     Eigen::Vector3d dir_xy = (norm < 1e-5) ? Eigen::Vector3d(1, 0, 0) : (vec / norm);

    //     bool found_view = false;
    //     for (double dist : dists) {
    //         if (found_view) break;
    //         for (double p_deg : pitches) {
    //             double pitch = p_deg * M_PI / 180.0;
    //             Eigen::Vector3d viewpoint = f - dir_xy * (dist * std::cos(pitch)) + Eigen::Vector3d(0, 0, dist * std::sin(pitch));

    //             VoxelIdx vp_idx = {
    //                 static_cast<int>(std::floor(viewpoint.x() / config_.resolution)),
    //                 static_cast<int>(std::floor(viewpoint.y() / config_.resolution)),
    //                 static_cast<int>(std::floor(viewpoint.z() / config_.resolution))
    //             };

    //             if (occ_set.count(vp_idx)) {
    //                 skipped_wall++;
    //                 continue;
    //             }

    //             if (SearchAlgo::check_line_of_sight(viewpoint, f, occ_set, resolution_)) {
    //                 double travel_cost = (viewpoint - current_pos_offset).norm() + 0.1;
    //                 double utility = frontier_scores[i] / (alpha_cost * travel_cost);

    //                 if (utility >= min_utility_threshold) {
    //                     candidates.push_back({viewpoint, f});
    //                     found_view = true;
    //                     break;
    //                 } else {
    //                     skipped_utility++;
    //                 }
    //             } else {
    //                 skipped_los++;
    //             }
    //         }
    //     }
    // }

    // std::cout << "[DEBUG 5] Candidate Generation:" << std::endl;
    // std::cout << "  -> Rejected (Inside Wall): " << skipped_wall << std::endl;
    // std::cout << "  -> Rejected (No Line of Sight): " << skipped_los << std::endl;
    // std::cout << "  -> Rejected (Low Utility): " << skipped_utility << std::endl;
    // std::cout << "  -> Accepted Viewpoints: " << candidates.size() << std::endl;


  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> candidates;
  double alpha_cost = 1.0;
  double min_utility_threshold = config_.min_utility_threshold;

  std::vector<double> dists = {1.5, 2.0, 1.2};   // Keep it close for good stereo disparity
  std::vector<double> pitches = {30.0, 45.0, 60.0};

    // 8 points around the compass (45-degree increments)
  std::vector<double> yaws = {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0};

  int skipped_wall = 0;
  int skipped_los = 0;
  int skipped_utility = 0;

  for (size_t i = 0; i < frontier_centers.size(); ++i) {
    Eigen::Vector3d f = frontier_centers[i];

    bool found_view = false;

        // Search 360 degrees around the frontier for a valid viewing angle!
    for (double yaw_deg : yaws) {
      if (found_view) {break;}

      double yaw_rad = yaw_deg * M_PI / 180.0;
      Eigen::Vector3d dir_xy(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);

      for (double dist : dists) {
        if (found_view) {break;}
        for (double p_deg : pitches) {
          double pitch = p_deg * M_PI / 180.0;

                    // Generate viewpoint independent of AUV's current location
          Eigen::Vector3d viewpoint = f - dir_xy * (dist * std::cos(pitch)) + Eigen::Vector3d(0, 0,
              dist * std::sin(pitch));

          VoxelIdx vp_idx = {
            static_cast<int>(std::floor(viewpoint.x() / config_.resolution)),
            static_cast<int>(std::floor(viewpoint.y() / config_.resolution)),
            static_cast<int>(std::floor(viewpoint.z() / config_.resolution))
          };

          if (occ_set.count(vp_idx)) {
            skipped_wall++;
            continue;
          }

          if (SearchPolicy::check_line_of_sight(viewpoint, f, occ_set, config_.resolution)) {
                        // Calculate travel cost from the AUV's actual position
            double travel_cost = (viewpoint - current_pos_offset).norm() + 0.1;
            double utility = frontier_scores[i] / (alpha_cost * travel_cost);

            if (utility >= min_utility_threshold) {
              candidates.push_back({viewpoint, f});
              found_view = true;
              break;               // Move to the next frontier!
            } else {
              skipped_utility++;
            }
          } else {
            skipped_los++;
          }
        }
      }
    }
  }

    // ==========================================
    // 2.7 FOV-BASED VIEWPOINT CULLING
    // ==========================================
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> optimized_candidates;
  if (!candidates.empty()) {
    double fov_rad = 80.0 * M_PI / 180.0;
    double max_view_range = 3.0;
    std::unordered_set<VoxelIdx, VoxelHash> covered_frontiers;

    for (const auto & [vp, target] : candidates) {
      VoxelIdx target_idx = {
        static_cast<int>(std::round(target.x() * 1000)),
        static_cast<int>(std::round(target.y() * 1000)),
        static_cast<int>(std::round(target.z() * 1000))
      };

      if (covered_frontiers.count(target_idx)) {continue;}
      optimized_candidates.push_back({vp, target});

      Eigen::Vector3d forward_vec = target - vp;
      double dist_to_target = forward_vec.norm();
      if (dist_to_target < 1e-5) {continue;}
      Eigen::Vector3d forward_dir = forward_vec / dist_to_target;

      for (const auto & f : frontier_centers) {
        VoxelIdx f_idx = {
          static_cast<int>(std::round(f.x() * 1000)),
          static_cast<int>(std::round(f.y() * 1000)),
          static_cast<int>(std::round(f.z() * 1000))
        };

        if (covered_frontiers.count(f_idx)) {continue;}

        Eigen::Vector3d vec_to_f = f - vp;
        double dist_to_f = vec_to_f.norm();

        if (dist_to_f <= max_view_range) {
          Eigen::Vector3d dir_to_f = vec_to_f / dist_to_f;
          double cos_angle = std::clamp(forward_dir.dot(dir_to_f), -1.0, 1.0);
          double angle = std::acos(cos_angle);

          if (angle <= (fov_rad / 2.0)) {
            covered_frontiers.insert(f_idx);
          }
        }
      }
    }
    candidates = optimized_candidates;
  }

  {std::ostringstream message; message << "[DEBUG 6] FOV Culling:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Optimized Viewpoints: " << candidates.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

    // Route Planning & Smoothing Phase
  auto [full_path, target_tracking] = route_.plan_multi_waypoint_route(current_pos_offset,
      candidates, occ_set);

  {std::ostringstream message; message << "[DEBUG 7] A* Routing:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Path nodes generated: " << full_path.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector4d> smooth_traj = route_.generate_seamless_mission_trajectory(full_path,
      target_tracking, 200);

  {std::ostringstream message; message << "[DEBUG 8] Spline Smoothing:" << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message;
    message << "  -> Final smooth waypoints: " << smooth_traj.size() << std::endl;
    diagnostics_.text(message.str(), false, true);}
  {std::ostringstream message; message << "=======================================\n" << std::endl;
    diagnostics_.text(message.str(), false, true);}

  std::vector<Eigen::Vector3d> raw_viewpoints;
  for (const auto & c : candidates) {
    raw_viewpoints.push_back(c.first);
  }

  return {raw_viewpoints, smooth_traj};
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
RoutePlanner::plan_multi_waypoint_route(
  const Eigen::Vector3d & current_location,
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & candidates,
  const VoxelSet & occ_set)
{
  if (candidates.empty()) {return Route{};}

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> unvisited = candidates;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ordered_candidates;
  Eigen::Vector3d curr_pos = current_location;

    // Greedy TSP
  while (!unvisited.empty()) {
    auto best_it = unvisited.begin();
    double min_dist = 1e9;

    for (auto it = unvisited.begin(); it != unvisited.end(); ++it) {
      double dist = (curr_pos - it->first).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_it = it;
      }
    }
    ordered_candidates.push_back(*best_it);
    curr_pos = best_it->first;
    unvisited.erase(best_it);
  }

  std::vector<Eigen::Vector3d> full_path;
  std::vector<Eigen::Vector3d> target_tracking;
  Eigen::Vector3d start_pos = current_location;

    // A* Path Concatenation
  for (const auto & vp_target : ordered_candidates) {
    {std::ostringstream message;
      message << "Planning path from " << start_pos.transpose() << " to " <<
        vp_target.first.transpose() << std::endl; diagnostics_.text(message.str(), false, true);}
    std::vector<Eigen::Vector3d> segment = SearchPolicy::astar_3d(
            start_pos, vp_target.first, occ_set, resolution_, 50000,
      [this](const std::string & message) {diagnostics_.text(message, true, false);});

    if (segment.empty()) {continue;}

    if (!full_path.empty() && (full_path.back() - segment.front()).norm() < 1e-4) {
      segment.erase(segment.begin());
    }

    full_path.insert(full_path.end(), segment.begin(), segment.end());
    for (size_t i = 0; i < segment.size(); ++i) {
      target_tracking.push_back(vp_target.second);
    }
    start_pos = vp_target.first;
  }

  return {full_path, target_tracking};
}

Eigen::Vector4d catmull_rom(
  const Eigen::Vector4d & p0, const Eigen::Vector4d & p1,
  const Eigen::Vector4d & p2, const Eigen::Vector4d & p3, double t)
{
  double t2 = t * t;
  double t3 = t2 * t;
  return 0.5 * ((2.0 * p1) +
         (-p0 + p2) * t +
         (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
         (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

double unwrap_angle(double previous_angle, double new_angle)
{
  double d = new_angle - previous_angle;
  d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
  return previous_angle + d;
}

std::vector<Eigen::Vector4d> RoutePlanner::generate_seamless_mission_trajectory(
  const std::vector<Eigen::Vector3d> & full_path,
  const std::vector<Eigen::Vector3d> & target_tracking,
  int num_points)
{
  if (full_path.size() < 4) {
    std::vector<Eigen::Vector4d> traj_xyz_yaw;
    for(size_t i = 0; i < full_path.size(); ++i) {
      double yaw = std::atan2(target_tracking[i].y() - full_path[i].y(),
                                    target_tracking[i].x() - full_path[i].x());
            // Add full_path[i].z() here
      traj_xyz_yaw.push_back(Eigen::Vector4d(full_path[i].x(), full_path[i].y(), full_path[i].z(),
          yaw));
    }
    return traj_xyz_yaw;
  }

    // 1. Prepare 4D Control Points [X, Y, Z, Yaw]
  std::vector<Eigen::Vector4d> control_points;
  double prev_yaw = 0.0;

  for (size_t i = 0; i < full_path.size(); ++i) {
    double raw_yaw = std::atan2(target_tracking[i].y() - full_path[i].y(),
                                    target_tracking[i].x() - full_path[i].x());

    if (i > 0) {raw_yaw = unwrap_angle(prev_yaw, raw_yaw);}
    prev_yaw = raw_yaw;

    control_points.push_back(Eigen::Vector4d(full_path[i].x(), full_path[i].y(), full_path[i].z(),
        raw_yaw));
  }

    // 2. Generate smooth spline (FIXED)
  std::vector<Eigen::Vector4d> trajectory;
  int segments = control_points.size() - 3;

    // Safety check just in case
  if (segments < 1) {return control_points;}

    // Distribute exactly 'num_points' evenly across the entire spline
  for (int i = 0; i < num_points; ++i) {
        // Find where we are globally on the path (0.0 to segments)
    double global_t = (static_cast<double>(i) / std::max(1, num_points - 1)) * segments;

        // Find which specific segment we are inside
    int segment_idx = std::min(static_cast<int>(global_t), segments - 1);

        // Find our local progress inside that specific segment (0.0 to 1.0)
    double local_t = global_t - segment_idx;

    Eigen::Vector4d pt = catmull_rom(
            control_points[segment_idx], control_points[segment_idx + 1],
            control_points[segment_idx + 2], control_points[segment_idx + 3], local_t);

        // Re-wrap yaw to [-pi, pi] for ROS Odometry standards
    double final_yaw = std::fmod(pt.w() + M_PI, 2.0 * M_PI);
    if (final_yaw < 0) {final_yaw += 2.0 * M_PI;}
    final_yaw -= M_PI;

    trajectory.push_back(Eigen::Vector4d(pt.x(), pt.y(), pt.z(), final_yaw));
  }

  return trajectory;
}
}
