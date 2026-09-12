#ifndef REALTIME_STEREO_MAPPER_HPP
#define REALTIME_STEREO_MAPPER_HPP

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "search_algo.hpp"

struct MapperConfig {
    double fov_deg = 90.0;
    double focal_len = 256.0;
    double baseline = 0.5;
    double resolution = 0.1;
    double max_depth = 4.0;
    double min_depth = 0.5;
    double z_min_target = -32.0;
    double info_gain_threshold = 0.15;
    double sparsity_multiplier = 1.2;
    double min_utility_threshold = 0.02;
};

class RealTimeStereoMapper {
public:
    RealTimeStereoMapper(const MapperConfig& config);

    void process_frame(const Eigen::Vector3d& location, 
                       double yaw, double pitch, double roll, 
                       const cv::Mat& rgb_frame, const cv::Mat& depth_map);

    void save_map(const std::string& filename);
    
    // Returns <Viewpoints, TargetFrontiers>
    // Change the second part of the pair to Vector4d
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector4d>> 
    generate_candidate_viewpoints(const Eigen::Vector3d& current_position);

    size_t get_current_leaves() const;

private:
    double fov_degrees_;
    double focal_length_px_;
    double baseline_meters_;
    double resolution_;

    MapperConfig config_;
    size_t previous_leaves_;
    std::shared_ptr<octomap::ColorOcTree> tree_;

    void generate_global_point_cloud(
        const cv::Mat& depth_image, const cv::Mat& color_image, 
        const Eigen::Vector3d& location, const Eigen::Matrix3d& rot_matrix, 
        octomap::Pointcloud& out_cloud, std::vector<octomap::ColorOcTreeNode::Color>& out_colors);

    // --- ADDED MISSING DECLARATIONS HERE ---
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> 
    plan_multi_waypoint_route(
        const Eigen::Vector3d& current_location, 
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& candidates, 
        const VoxelSet& occ_set);

    std::vector<Eigen::Vector4d> generate_seamless_mission_trajectory(
        const std::vector<Eigen::Vector3d>& full_path, 
        const std::vector<Eigen::Vector3d>& target_tracking, 
        int num_points);
};

#endif