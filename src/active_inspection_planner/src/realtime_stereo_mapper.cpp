#include "realtime_stereo_mapper.hpp"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <numeric>
#include "search_algo.hpp"

RealTimeStereoMapper::RealTimeStereoMapper(const MapperConfig& config)
    : fov_degrees_(config.fov_deg),
      focal_length_px_(config.focal_len),
      baseline_meters_(config.baseline),
      resolution_(config.resolution), // Fixes the Division by Zero!
      config_(config), 
      previous_leaves_(0)
{
    tree_ = std::make_shared<octomap::ColorOcTree>(resolution_);
}

size_t RealTimeStereoMapper::get_current_leaves() const {
    if (tree_) return tree_->getNumLeafNodes();
    return 0;
}

void RealTimeStereoMapper::generate_global_point_cloud(
    const cv::Mat& depth_image, const cv::Mat& color_image, 
    const Eigen::Vector3d& location, const Eigen::Matrix3d& rot_matrix, 
    octomap::Pointcloud& out_cloud, std::vector<octomap::ColorOcTreeNode::Color>& out_colors)
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
            float Z = depth_image.at<float>(v, u); // Assuming 32FC1 depth map
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

                out_cloud.push_back(global_pt.x(), global_pt.y(), global_pt.z());

                // Colors
                cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
                // OpenCV is BGR by default, swap to RGB
                out_colors.push_back(octomap::ColorOcTreeNode::Color(color[2], color[1], color[0])); 
            }
        }
    }
}

void RealTimeStereoMapper::process_frame(
    const Eigen::Vector3d& location, double yaw, double pitch, double roll, 
    const cv::Mat& rgb_frame, const cv::Mat& depth_map)
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
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
        -sp,   cp*sr,            cp*cr;

    octomap::Pointcloud cloud;
    std::vector<octomap::ColorOcTreeNode::Color> colors;
    
    // Pass to projection
    generate_global_point_cloud(depth_map, rgb_frame, cam_location, rotation_matrix, cloud, colors);

    if (cloud.size() > 0) {
        octomap::point3d sensor_origin(cam_location.x(), cam_location.y(), cam_location.z());
        tree_->insertPointCloud(cloud, sensor_origin, config_.max_depth); // max_range = 4.0

        // Apply colors to the tree
        for (size_t i = 0; i < cloud.size(); ++i) {
            tree_->integrateNodeColor(cloud[i].x(), cloud[i].y(), cloud[i].z(), 
                                      colors[i].r, colors[i].g, colors[i].b);
        }
        
        size_t current_leaves = tree_->getNumLeafNodes();
        size_t exploration_score = current_leaves - previous_leaves_;
        (void)exploration_score;
        previous_leaves_ = current_leaves;
        
        // You can log exploration_score here as you did with the Python file writing
    }
}

void RealTimeStereoMapper::save_map(const std::string& filename) {
    tree_->write(filename);
    std::cout << "Final map saved to " << filename << std::endl;
}

// Helper function to calculate utility-based viewpoints
std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector4d>> 
RealTimeStereoMapper::generate_candidate_viewpoints(const Eigen::Vector3d& current_position) 
{
    Eigen::Vector3d current_pos_offset = current_position + Eigen::Vector3d(0.35, 0.1125 + 0.275, -0.5);
    
    std::cout << "\n[DEBUG 1] Starting generate_candidate_viewpoints." << std::endl;
    std::cout << "[DEBUG 1] Current Offset Pos: [" << current_pos_offset.x() << ", " 
              << current_pos_offset.y() << ", " << current_pos_offset.z() << "]" << std::endl;

    std::vector<Eigen::Vector3d> occupied_points;
    std::vector<Eigen::Vector3d> empty_points;
    VoxelSet occ_set;
    
    int close_points_filtered = 0;

    // Extract points directly from OctoMap
    for (octomap::ColorOcTree::leaf_iterator it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) {
        Eigen::Vector3d pt(it.getX(), it.getY(), it.getZ());

        if ((pt - current_pos_offset).norm() < 0.6) {
            close_points_filtered++;
            continue; 
        }

        if (tree_->isNodeOccupied(*it)) {
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

    std::cout << "[DEBUG 2] OctoMap Extraction:" << std::endl;
    std::cout << "  -> Filtered (dist < 0.6): " << close_points_filtered << std::endl;
    std::cout << "  -> Occupied points: " << occupied_points.size() << std::endl;
    std::cout << "  -> Empty points: " << empty_points.size() << std::endl;

    if (occupied_points.empty()) {
        std::cout << "[WARN] No occupied points found! Aborting." << std::endl;
        return {{}, {}};
    }

    // ==========================================
    // 1. Calculate Information Gain (Surface Sparsity via PCL)
    // ==========================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr occ_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& p : occupied_points) {
        if (p.z() > config_.z_min_target) { // z_min_target
            occ_cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }
    }

    std::cout << "[DEBUG 3] Z-Filtering (Threshold: " << config_.z_min_target << "):" << std::endl;
    std::cout << "  -> Valid points remaining: " << occ_cloud->size() << std::endl;

    std::vector<Eigen::Vector3d> frontier_centers;
    std::vector<double> frontier_scores;

    if (occ_cloud->size() > 10) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(occ_cloud);

        std::vector<int> pointIdx(11);
        std::vector<float> pointDistSq(11);
        
        double max_rho = 0.0, min_rho = 1e9;
        std::vector<double> rhos(occ_cloud->size());

        // Calculate sparsity (rho) for each point
        for (size_t i = 0; i < occ_cloud->size(); ++i) {
            if (kdtree.nearestKSearch((*occ_cloud)[i], 11, pointIdx, pointDistSq) > 0) {
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& p : empty_points) {
            free_cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> free_kdtree;
        bool has_free = !free_cloud->empty();
        if (has_free) free_kdtree.setInputCloud(free_cloud);

        for (size_t i = 0; i < occ_cloud->size(); ++i) {
            double base_score = (rhos[i] - min_rho) / (max_rho - min_rho + 1e-6);
            if (base_score > config_.info_gain_threshold && rhos[i] > (config_.resolution * config_.sparsity_multiplier)) {
                
                double adjusted_score = base_score;
                if (has_free) {
                    std::vector<int> freeIdx;
                    std::vector<float> freeDistSq;
                    int free_counts = free_kdtree.radiusSearch((*occ_cloud)[i], config_.resolution * 3.0, freeIdx, freeDistSq);
                    double damping = std::exp(-0.05 * free_counts);
                    adjusted_score *= damping;
                }

                if (adjusted_score >= 0.05) {
                    frontier_centers.push_back(Eigen::Vector3d((*occ_cloud)[i].x, (*occ_cloud)[i].y, (*occ_cloud)[i].z));
                    frontier_scores.push_back(adjusted_score);
                }
            }
        }
    }

    std::cout << "[DEBUG 4] Frontier Extraction:" << std::endl;
    std::cout << "  -> High-value frontiers found: " << frontier_centers.size() << std::endl;

    // ==========================================
    // 2. Evaluate Utility & Generate Viewpoints
    // ==========================================
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> candidates;
    double alpha_cost = 1.0;
    double min_utility_threshold = config_.min_utility_threshold;

    std::vector<double> dists = {2.0, 1.5, 3.0, 1.0};
    std::vector<double> pitches = {45.0, 60.0, 30.0, 75.0, 15.0};

    int skipped_wall = 0;
    int skipped_los = 0;
    int skipped_utility = 0;

    for (size_t i = 0; i < frontier_centers.size(); ++i) {
        Eigen::Vector3d f = frontier_centers[i];
        Eigen::Vector3d vec = f - current_pos_offset;
        vec.z() = 0;
        
        double norm = vec.norm();
        Eigen::Vector3d dir_xy = (norm < 1e-5) ? Eigen::Vector3d(1, 0, 0) : (vec / norm);

        bool found_view = false;
        for (double dist : dists) {
            if (found_view) break;
            for (double p_deg : pitches) {
                double pitch = p_deg * M_PI / 180.0;
                Eigen::Vector3d viewpoint = f - dir_xy * (dist * std::cos(pitch)) + Eigen::Vector3d(0, 0, dist * std::sin(pitch));

                VoxelIdx vp_idx = {
                    static_cast<int>(std::floor(viewpoint.x() / config_.resolution)),
                    static_cast<int>(std::floor(viewpoint.y() / config_.resolution)),
                    static_cast<int>(std::floor(viewpoint.z() / config_.resolution))
                };
                
                if (occ_set.count(vp_idx)) {
                    skipped_wall++;
                    continue; 
                }

                if (SearchAlgo::check_line_of_sight(viewpoint, f, occ_set, resolution_)) {
                    double travel_cost = (viewpoint - current_pos_offset).norm() + 0.1;
                    double utility = frontier_scores[i] / (alpha_cost * travel_cost);
                    
                    if (utility >= min_utility_threshold) {
                        candidates.push_back({viewpoint, f});
                        found_view = true;
                        break;
                    } else {
                        skipped_utility++;
                    }
                } else {
                    skipped_los++;
                }
            }
        }
    }

    std::cout << "[DEBUG 5] Candidate Generation:" << std::endl;
    std::cout << "  -> Rejected (Inside Wall): " << skipped_wall << std::endl;
    std::cout << "  -> Rejected (No Line of Sight): " << skipped_los << std::endl;
    std::cout << "  -> Rejected (Low Utility): " << skipped_utility << std::endl;
    std::cout << "  -> Accepted Viewpoints: " << candidates.size() << std::endl;

    // ==========================================
    // 2.7 FOV-BASED VIEWPOINT CULLING
    // ==========================================
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> optimized_candidates;
    if (!candidates.empty()) {
        double fov_rad = 80.0 * M_PI / 180.0;
        double max_view_range = 3.0;
        std::unordered_set<VoxelIdx, VoxelHash> covered_frontiers;
        
        for (const auto& [vp, target] : candidates) {
            VoxelIdx target_idx = {
                static_cast<int>(std::round(target.x() * 1000)), 
                static_cast<int>(std::round(target.y() * 1000)),
                static_cast<int>(std::round(target.z() * 1000))
            };
            
            if (covered_frontiers.count(target_idx)) continue;
            optimized_candidates.push_back({vp, target});
            
            Eigen::Vector3d forward_vec = target - vp;
            double dist_to_target = forward_vec.norm();
            if (dist_to_target < 1e-5) continue;
            Eigen::Vector3d forward_dir = forward_vec / dist_to_target;
            
            for (const auto& f : frontier_centers) {
                VoxelIdx f_idx = {
                    static_cast<int>(std::round(f.x() * 1000)),
                    static_cast<int>(std::round(f.y() * 1000)),
                    static_cast<int>(std::round(f.z() * 1000))
                };
                
                if (covered_frontiers.count(f_idx)) continue;
                
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

    std::cout << "[DEBUG 6] FOV Culling:" << std::endl;
    std::cout << "  -> Optimized Viewpoints: " << candidates.size() << std::endl;

    // Route Planning & Smoothing Phase
    auto [full_path, target_tracking] = plan_multi_waypoint_route(current_pos_offset, candidates, occ_set);
    
    std::cout << "[DEBUG 7] A* Routing:" << std::endl;
    std::cout << "  -> Path nodes generated: " << full_path.size() << std::endl;

    std::vector<Eigen::Vector4d> smooth_traj = generate_seamless_mission_trajectory(full_path, target_tracking, 200);

    std::cout << "[DEBUG 8] Spline Smoothing:" << std::endl;
    std::cout << "  -> Final smooth waypoints: " << smooth_traj.size() << std::endl;
    std::cout << "=======================================\n" << std::endl;

    std::vector<Eigen::Vector3d> raw_viewpoints;
    for (const auto& c : candidates) raw_viewpoints.push_back(c.first);

    return {raw_viewpoints, smooth_traj};
}


std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> 
RealTimeStereoMapper::plan_multi_waypoint_route(
    const Eigen::Vector3d& current_location, 
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& candidates, 
    const VoxelSet& occ_set) 
{
    if (candidates.empty()) return {{}, {}};

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
    for (const auto& vp_target : ordered_candidates) {
        std::cout<< "Planning path from " << start_pos.transpose() << " to " << vp_target.first.transpose() << std::endl;
        std::vector<Eigen::Vector3d> segment = SearchAlgo::astar_3d(start_pos, vp_target.first, occ_set, resolution_);
        
        if (segment.empty()) continue;

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

// Catmull-Rom Spline Interpolator (Replaces scipy splprep)
Eigen::Vector4d catmull_rom(const Eigen::Vector4d& p0, const Eigen::Vector4d& p1, 
                            const Eigen::Vector4d& p2, const Eigen::Vector4d& p3, double t) 
{
    double t2 = t * t;
    double t3 = t2 * t;
    return 0.5 * ((2.0 * p1) +
                  (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

// Unwraps angles to prevent jumping from +pi to -pi
double unwrap_angle(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
    return previous_angle + d;
}

std::vector<Eigen::Vector4d> RealTimeStereoMapper::generate_seamless_mission_trajectory(
    const std::vector<Eigen::Vector3d>& full_path, 
    const std::vector<Eigen::Vector3d>& target_tracking, 
    int num_points) 
{
    if (full_path.size() < 4) {
        std::vector<Eigen::Vector4d> traj_xyz_yaw;
        for(size_t i = 0; i < full_path.size(); ++i) {
            double yaw = std::atan2(target_tracking[i].y() - full_path[i].y(), 
                                    target_tracking[i].x() - full_path[i].x());
            // Add full_path[i].z() here
            traj_xyz_yaw.push_back(Eigen::Vector4d(full_path[i].x(), full_path[i].y(), full_path[i].z(), yaw)); 
        }
        return traj_xyz_yaw;
    }

    // 1. Prepare 4D Control Points [X, Y, Z, Yaw]
    std::vector<Eigen::Vector4d> control_points;
    double prev_yaw = 0.0;
    
    for (size_t i = 0; i < full_path.size(); ++i) {
        double raw_yaw = std::atan2(target_tracking[i].y() - full_path[i].y(), 
                                    target_tracking[i].x() - full_path[i].x());
        
        if (i > 0) raw_yaw = unwrap_angle(prev_yaw, raw_yaw);
        prev_yaw = raw_yaw;

        control_points.push_back(Eigen::Vector4d(full_path[i].x(), full_path[i].y(), full_path[i].z(), raw_yaw));
    }

    // 2. Generate smooth spline (FIXED)
    std::vector<Eigen::Vector4d> trajectory;
    int segments = control_points.size() - 3;
    
    // Safety check just in case
    if (segments < 1) return control_points;

    // Distribute exactly 'num_points' evenly across the entire spline
    for (int i = 0; i < num_points; ++i) {
        // Find where we are globally on the path (0.0 to segments)
        double global_t = (static_cast<double>(i) / std::max(1, num_points - 1)) * segments;
        
        // Find which specific segment we are inside
        int segment_idx = std::min(static_cast<int>(global_t), segments - 1);
        
        // Find our local progress inside that specific segment (0.0 to 1.0)
        double local_t = global_t - segment_idx;

        Eigen::Vector4d pt = catmull_rom(
            control_points[segment_idx], control_points[segment_idx+1], 
            control_points[segment_idx+2], control_points[segment_idx+3], local_t);
        
        // Re-wrap yaw to [-pi, pi] for ROS Odometry standards
        double final_yaw = std::fmod(pt.w() + M_PI, 2.0 * M_PI);
        if (final_yaw < 0) final_yaw += 2.0 * M_PI;
        final_yaw -= M_PI;

        trajectory.push_back(Eigen::Vector4d(pt.x(), pt.y(), pt.z(), final_yaw)); 
    }
    
    return trajectory;
}