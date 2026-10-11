#pragma once
#include <algorithm>
#include <cmath>
#include "motion_control/domain/contracts.hpp"
namespace motion_control
{
class TrajectoryQueue {
  WaypointOutput & output_; Diagnostics & diagnostics_;
  std::vector<std::vector<double>> waypoints_;
  double initial_waypoint_x_ = -16.0;
  double initial_waypoint_y_ = -81.0;
  double initial_waypoint_z_ = -30.0;
  double initial_waypoint_yaw_ = 0.0;
  double timer_period_s_ = 1.0;
  double waypoint_reached_distance_ = 0.4;

public:
  TrajectoryQueue(const QueueConfig & config, WaypointOutput & output, Diagnostics & diagnostics)
  : output_(output), diagnostics_(diagnostics)
  {
    initial_waypoint_x_ = config.initial_waypoint_x;
    initial_waypoint_y_ = config.initial_waypoint_y;
    initial_waypoint_z_ = config.initial_waypoint_z;
    initial_waypoint_yaw_ = config.initial_waypoint_yaw;
    timer_period_s_ = config.timer_period_s;
    waypoint_reached_distance_ = config.waypoint_reached_distance;
        // Initial setup
    waypoints_.push_back({
        initial_waypoint_x_, initial_waypoint_y_,
        initial_waypoint_z_, initial_waypoint_yaw_});
        // waypoints_.push_back({-56.0, -100.0, -30.0, 3.14});

  }
  void point_callback(const WaypointInput & msg)
  {
    waypoints_.clear();
    std::vector<double> new_wp(msg.data.begin(), msg.data.end());
    waypoints_.push_back(new_wp);
    timer_callback();
    diagnostics_.write(format_text("Published 1 waypoints (from object/detected)."));
  }

  void pointSegment_callback(const WaypointInput & msg)
  {
    // Action results are already vehicle coordinates in the map frame.
    std::vector<std::vector<double>> wps;

        // Replicates numpy.reshape(-1, 4)
    for (size_t i = 0; i + 3 < msg.data.size(); i += 4) {
      diagnostics_.write(format_text("Adding segmentation waypoint..."));
      std::vector<double> wp = {
        msg.data[i],
        msg.data[i + 1],
        msg.data[i + 2],
        msg.data[i + 3]
      };
      wps.push_back(wp);
    }

    if (!wps.empty()) {
      waypoints_ = wps;
      diagnostics_.write(format_text("Received %zu segmentation waypoints.", wps.size()));
            // Send the new route now: the vehicle keeps moving between timer ticks.
      timer_callback();
    }
  }

  void clear() {waypoints_.clear();}

  void timer_callback()
  {
    if (!waypoints_.empty()) {
      std::vector<float> values;
      for (const auto & wp : waypoints_) {
        float x = 0.0f, y = 0.0f, z = -29.5f, yaw = 0.0f;

                // Safely pad every waypoint to exactly 4 floats to prevent numpy crashes
        if (wp.size() >= 2) {x = wp[0]; y = wp[1];}
        if (wp.size() >= 3) {z = wp[2];}
        if (wp.size() >= 4) {yaw = wp[3];}

        values.insert(values.end(), {x, y, z, yaw});
      }

      output_.waypoints(values, waypoints_.size());
      diagnostics_.write(format_text("Published %zu waypoints.", waypoints_.size()));
    }
  }

  void stop_callback(const WaypointInput & msg)
  {
    if (msg.data.size() < 3) {return;}

    std::vector<double> data = {msg.data[0], msg.data[1], msg.data[2]};

    if (!waypoints_.empty() && waypoints_[0].size() >= 2) {
            // Replaces python Tools.mapping2D.diff
      double dist = std::hypot(waypoints_[0][0] - data[0], waypoints_[0][1] - data[1]);

      if (dist < waypoint_reached_distance_) {
        waypoints_.erase(waypoints_.begin());
      }
    }

    diagnostics_.write(format_text("Received stop signal, giving next waypoint."));

        // Replaces try/except blocks to remove specific target point from list safely
    auto it = std::find_if(waypoints_.begin(), waypoints_.end(),
        [&data](const std::vector<double> & wp) {
          if (wp.size() >= 3) {
            return  wp[0] == data[0] && wp[1] == data[1] && wp[2] == data[2];
          }
          return false;
        });

    if (it != waypoints_.end()) {
      waypoints_.erase(it);
    }

        // Limit the waypoints array to the last 200 elements (self.waypoints[-200:])
        // if (waypoints_.size() > 200) {
        //     waypoints_.erase(waypoints_.begin(), waypoints_.end() - 200);
        // }

        // Trigger the publisher manually
    timer_callback();

        // NEW: If the queue just hit zero, instantly demand more waypoints!
    if (waypoints_.empty()) {
      diagnostics_.write(format_text("Waypoint queue empty. Demanding new NBV trajectory NOW!"));
      output_.request_nbv();
    }
  }
};
}
