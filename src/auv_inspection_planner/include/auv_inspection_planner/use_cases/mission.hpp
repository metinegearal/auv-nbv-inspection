#pragma once
#include "auv_inspection_planner/use_cases/mapping.hpp"
namespace inspection
{
class InspectionMission {
  MapUpdate & update_; CandidatePlanner & planner_; MapReader & reader_; MapSnapshotStore & store_;
  MissionOutput & output_; MissionMetrics & metrics_; Clock & clock_; Diagnostics & diagnostics_;
  const std::string map_path_;
  bool odom_received_ = false, started_mapping_ = false, finished_mapping_ = false;
  int iter_count_ = 0; std::size_t previous_node_leaves_ = 0;
  Eigen::Vector3d current_position_{0, 0, 0}, latched_pos_{0, 0, 0};
  double yaw_ = 0, pitch_ = 0, roll_ = 0, latched_yaw_ = 0, latched_pitch_ = 0, latched_roll_ = 0;
  std::shared_ptr<const ColorImage> left_image_;
  std::shared_ptr<const DepthImage> depth_frame_;

public:
  InspectionMission(
    MapUpdate & update, CandidatePlanner & planner, MapReader & reader,
    MapSnapshotStore & store, MissionOutput & output, MissionMetrics & metrics, Clock & clock,
    Diagnostics & diagnostics, std::string path)
  :update_(update), planner_(planner), reader_(reader), store_(store), output_(output),
    metrics_(metrics),
    clock_(clock), diagnostics_(diagnostics), map_path_(std::move(path)) {}
  bool accepts_pose(const Eigen::Vector3d & p) const
  {
    return !(std::abs(p.x()) < 0.01 && std::abs(p.y()) < 0.01);
  }
  void pose(const Pose & pose)
  {
    current_position_ = pose.position; roll_ = pose.roll; pitch_ = pose.pitch; yaw_ = pose.yaw;
    odom_received_ = true;
  }
  void magnetic(double x, double y)
  {
    double ya = std::atan2(x, y) - (M_PI / 2.0); if(ya < -M_PI) {ya += 2 * M_PI;} yaw_ = ya;
  }
  void rgb(std::shared_ptr<const ColorImage> frame) {left_image_ = std::move(frame);}
  void depth(std::shared_ptr<const DepthImage> frame)
  {
    depth_frame_ = std::move(frame); latched_pos_ = current_position_; latched_yaw_ = yaw_;
    latched_pitch_ = pitch_; latched_roll_ = roll_;
  }
  void timer_callback()
  {
    diagnostics_.info(format_text(
        "Timer triggered. Iteration: %d, left_images: %d, depth_frames: %d, odom_received: %s",
        iter_count_, (!left_image_ || left_image_->empty()) ? 0 : 1,
        (!depth_frame_ || depth_frame_->empty()) ? 0 : 1, odom_received_ ? "true" : "false"));
    if (finished_mapping_) {return;}
    if ((!left_image_ || left_image_->empty()) || (!depth_frame_ || depth_frame_->empty())) {
      return;
    }
    if (!odom_received_) {return;}


    auto start_proc = clock_.now();

    update_.process_frame(latched_pos_, latched_yaw_, latched_pitch_, latched_roll_, *left_image_,
        *depth_frame_);

    if (iter_count_ % 20 == 0) {
      metrics_.begin();
      auto start_time = clock_.now();

      const std::string & map_path = map_path_;
      store_.save(map_path);

      auto [viewpoints, trajectories] = planner_.generate_candidate_viewpoints(current_position_);

            // --- NEW DEBUGGING BLOCK ---
      diagnostics_.info(format_text("A* Pipeline Finished. Trajectories size: %zu",
          trajectories.size()));
      if (!trajectories.empty()) {
        diagnostics_.info(format_text("First Trajectory Point: [%.2f, %.2f, %.2f]",
                            trajectories[0].x(), trajectories[0].y(), trajectories[0].z()));
      }
            // ---------------------------

      if (!trajectories.empty()) {

                // CRITICAL SAFETY CHECK: Prevent publishing [0,0,0] if A* failed weirdly
        if (trajectories.size() < 2 && std::abs(trajectories[0].x()) < 0.1 &&
          std::abs(trajectories[0].y()) < 0.1)
        {
          diagnostics_.error(format_text(
              "ALGORITHM RETURNED [0,0,0] INSTEAD OF A PATH. ABORTING PUBLISH!"));
        } else {
          started_mapping_ = true;
          output_.trajectory(trajectories);
          diagnostics_.info(format_text("Published valid trajectory to PathPlanner."));
        }
      } else if (started_mapping_) {
        finished_mapping_ = true;
        output_.finished();
        diagnostics_.info(format_text("Mapping finished. Explored all accessible frontiers."));
      } else {
        diagnostics_.info(format_text("No new viewpoints generated. Continuing exploration..."));
      }

      auto end_time = clock_.now();
      auto process_time_ms = std::chrono::duration<double,
          std::milli>(end_time - start_time).count();

      size_t current_leaves = reader_.count();
      size_t leaves_added = current_leaves - previous_node_leaves_;
      previous_node_leaves_ = current_leaves;

      metrics_.record(process_time_ms, leaves_added, current_leaves);
    }
    iter_count_++;
  }
};
}
