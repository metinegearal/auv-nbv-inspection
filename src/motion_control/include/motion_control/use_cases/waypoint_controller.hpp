#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include "motion_control/domain/contracts.hpp"
#include "motion_control/domain/math_pid_utils.hpp"
namespace motion_control
{
class WaypointController {
  Clock & clock_; Gains & gains_; Diagnostics & diagnostics_; CommandOutput & output_;
  const Modes & modes_; const ThrusterMixer & mixer_;
  std::vector<double> waypoint_;
  std::vector<double> magnet_;
  std::vector<float> obstacles_;
  std::vector<double> motor_command_;

  double current_yaw_ = 0.0;
  double original_yaw_ = 0.0;
  double current_depth_ = 0.0;
  double current_dist_seabed_ = 0.0;

  utils::PID pid_dist_;
  utils::PID pid_yaw_;

  double kp_dist_;
  double kp_yaw_;
  double stopping_distance_ = 0.25;
  double initial_waypoint_x_ = 10.0;
  double initial_waypoint_y_ = 10.0;
  double initial_waypoint_z_ = 5.0;
  double max_yaw_command_ = 10.0;
  double yaw_smoothing_rate_ = 0.8;
  double min_forward_thrust_ = 1.0;
  double max_forward_thrust_ = 10.0;
  double command_multiplier_ = 2.0;

  bool finished_execution_ = false;
  std::optional<double> hardcoded_angle_ = std::nullopt;
  std::optional<double> fixed_angle_ = std::nullopt;
  bool yaw_smoothing_initialized_ = false;
  double smoothed_desired_yaw_ = 0.0;
  double last_yaw_smoothing_time_ = 0.0;
  std::string mode_ = "yaw_centerize";

  std::vector<double> last_base_command_;
  std::vector<double> last_motor_command_;

  std::vector<double> current_position_ = {0.0, 0.0, 0.0};
  std::vector<double> dvl_position_ = {0.0, 0.0, 0.0};
  std::optional<std::vector<double>> dvl_reset_position_ = std::nullopt;
  bool dvl_active_;  // Assuming True based on Python fallback

  // --- Publishers ---

public:
  WaypointController(
    const ControlConfig & config, Clock & clock, Gains & gains,
    Diagnostics & diagnostics, CommandOutput & output, const Modes & modes,
    const ThrusterMixer & mixer)
  : clock_(clock), gains_(gains), diagnostics_(diagnostics), output_(output),
    modes_(modes), mixer_(mixer), pid_dist_(0.75, 0.3, 0.1, 0.0, 0.2),
    pid_yaw_(0.6, 0.3, 0.1, 0.0, 0.2)
  {
    kp_yaw_ = config.kp_yaw;
    kp_dist_ = config.kp_dist;
    dvl_active_ = config.dvl_active;
    stopping_distance_ = config.stopping_distance;
    initial_waypoint_x_ = config.initial_waypoint_x;
    initial_waypoint_y_ = config.initial_waypoint_y;
    initial_waypoint_z_ = config.initial_waypoint_z;
    max_yaw_command_ = config.max_yaw_command;
    yaw_smoothing_rate_ = config.yaw_smoothing_rate;
    min_forward_thrust_ = config.min_forward_thrust;
    max_forward_thrust_ = config.max_forward_thrust;
    command_multiplier_ = config.command_multiplier;
    waypoint_ = {initial_waypoint_x_, initial_waypoint_y_, initial_waypoint_z_};
    magnet_ = {0.0, 0.0, 0.0};
    obstacles_.resize(9, 0.0);
    motor_command_.resize(8, 0.0);
    last_base_command_.resize(8, 0.0);
    last_motor_command_.resize(8, 0.0);


  }
  void start()
  {
    move_towards_waypoint({initial_waypoint_x_, initial_waypoint_y_, initial_waypoint_z_});
  }
  double get_time_now() {return clock_.now();}

  double smooth_desired_yaw(double target_yaw)
  {
    target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
    const double now = get_time_now();
    if (!yaw_smoothing_initialized_) {
      smoothed_desired_yaw_ = target_yaw;
      last_yaw_smoothing_time_ = now;
      yaw_smoothing_initialized_ = true;
      return smoothed_desired_yaw_;
    }

    const double dt = std::max(0.0, now - last_yaw_smoothing_time_);
    last_yaw_smoothing_time_ = now;
    const double yaw_error = std::atan2(
      std::sin(target_yaw - smoothed_desired_yaw_),
      std::cos(target_yaw - smoothed_desired_yaw_));
    const double max_step = std::max(0.0, yaw_smoothing_rate_) * dt;
    smoothed_desired_yaw_ += std::clamp(yaw_error, -max_step, max_step);
    smoothed_desired_yaw_ = std::atan2(
      std::sin(smoothed_desired_yaw_), std::cos(smoothed_desired_yaw_));
    return smoothed_desired_yaw_;
  }

  void publish_cmd(std::vector<double> motor_command)
  {
    last_base_command_ = motor_command;
    last_motor_command_ = motor_command;

    output_.command(MotorCommand{motor_command});
  }

  void finished_execution_callback(bool value)
  {
    finished_execution_ = value;
  }

  void mode_callback(const std::string & value) {mode_ = value;}

  void angle_callback(float value)
  {
    if (modes_.select(mode_).accepts_angle) {hardcoded_angle_ = value;}
  }

  void dvl_reset_callback(bool value)
  {
    if (value) {
      dvl_reset_position_ = dvl_position_;
      dvl_active_ = true;
      diagnostics_.write(format_text("DVL position reset"));
    }
  }

  void position_callback(const Position & msg)
  {
    auto pos = msg;
    dvl_position_ = {pos.x, pos.y, 0.0};
    if (dvl_active_) {
      current_position_.assign({pos.x, pos.y, pos.z});
    } else {
      current_position_.assign({0.0, 0.0, pos.z});
    }
    move_towards_waypoint(waypoint_);
  }

  void magnetometer_callback(const Position & msg)
  {
    magnet_ = {msg.x, msg.y, msg.z};
    double ya = std::atan2(magnet_[0], magnet_[1]) - (M_PI / 2.0);
    if (ya < -M_PI) {ya += M_PI * 2.0;}
    current_yaw_ = ya;
    original_yaw_ = std::atan2(magnet_[0], magnet_[1]);
  }

  void depth_callback(float value) {current_depth_ = value;}

  void dist_callback(float value)
  {
    current_dist_seabed_ = value;
  }

  void obstacle_callback(const WaypointInput & msg)
  {
    obstacles_.assign(msg.data.begin(), msg.data.end());
  }

  void waypoint_callback(const WaypointInput & msg)
  {
    if (msg.data.size() >= 3) {
      waypoint_ = {msg.data[0], msg.data[1], msg.data[2]};
      if (msg.data.size() >= 4) {fixed_angle_ = msg.data[3];}
      diagnostics_.write(format_text("Received new waypoint: [%.2f, %.2f, %.2f]", waypoint_[0],
          waypoint_[1], waypoint_[2]));
      if (fixed_angle_.has_value()) {
        diagnostics_.write(format_text("Fixed angle: [%.2f]", fixed_angle_.value()));
      }
      move_towards_waypoint(waypoint_);
    }
  }

  void move_towards_waypoint(std::vector<double> target)
  {
    if (finished_execution_) {return;}

    if (dvl_active_ && dvl_reset_position_.has_value()) {
      for (size_t i = 0; i < 3; ++i) {target[i] += dvl_reset_position_.value()[i];}
    }
    diagnostics_.write(format_text(
        "From current position: [%.2f, %.2f, %.2f] moving towards waypoint: [%.2f, %.2f, %.2f]",
        current_position_[0], current_position_[1], current_position_[2], target[0], target[1],
        target[2]));

    std::vector<double> direction(3);
    for (size_t i = 0; i < 3; ++i) {direction[i] = target[i] - current_position_[i];}
    if (std::abs(direction[2]) <= stopping_distance_) {direction[2] = 0.0;}

    double distance = std::sqrt(
      direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);

    // Stopping condition
    if (utils::diff_2d(target, current_position_) < stopping_distance_) {
      std::vector<float> f_target(target.begin(), target.end());
      output_.reached(f_target);
    }

    if (distance == 0.0) {return;}
    for (size_t i = 0; i < 3; ++i) {direction[i] /= distance;}

    double target_yaw =
      fixed_angle_.has_value() ? fixed_angle_.value() : std::atan2(direction[1], direction[0]);
    double desired_yaw = smooth_desired_yaw(target_yaw);
    double raw_yaw_diff = desired_yaw - current_yaw_;
    double yaw_error = hardcoded_angle_.has_value() ?
      hardcoded_angle_.value() :
      std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));

    // Fetch dynamic parameters
    kp_yaw_ = gains_.yaw();
    kp_dist_ = gains_.distance();

    std::vector<double> cmd(8, 0.0);
    double vert_mult = modes_.select(mode_).vertical_multiplier;
    for (int i = 0; i < 4; i++) {cmd[i] = vert_mult * motor_command_[i];}

    // ==========================================
    // NEW: BLENDED CONTINUOUS CONTROL LOGIC
    // ==========================================

    // 1. Calculate Continuous Yaw (No Deadband)
    double yaw_command = 0.0;
    if (modes_.select(mode_).centered) {
      // YOUR original math: directly multiply error by gain
      yaw_command = kp_yaw_ * yaw_error;

      // Clamp it to maximum thrust
      yaw_command = std::clamp(
        yaw_command, -max_yaw_command_, max_yaw_command_);

      // YOUR original inversion to match thruster polarity
      yaw_command = -yaw_command;
    }

    // 2. Calculate Dynamic Surge Speed (Cosine Scaling)
    // Keep horizontal and vertical thrust independent to avoid Z motion
    // increasing with the distance to a far-away XY waypoint.
    double horizontal_distance = std::hypot(direction[0], direction[1]);
    double base_thrust = std::clamp(
      horizontal_distance * kp_dist_, min_forward_thrust_, max_forward_thrust_);
    double vertical_thrust = std::clamp(
      std::abs(direction[2]) * kp_dist_, 0.0, max_forward_thrust_);

    // Scale thrust based on how straight we are aiming to prevent corner overshoot
    // cos(yaw_error) smoothly drops the speed as the angle increases.
    // std::max(0.1, ...) ensures we never completely stop moving forward.
    double speed_scaling = std::max(0.1, std::cos(yaw_error));
    double forward_thrust = base_thrust * speed_scaling;

    // 3. Apply Positional Movement (Surge/Sway)
    auto auv_pos = utils::world_to_auv_coordinates(direction, 3.0 * M_PI / 2.0 - original_yaw_);
    auto keys = determine_keys(auv_pos);
    cmd = mixer_.mix(keys, forward_thrust, cmd, vertical_thrust);

    // 4. Overlay Yaw Command simultaneously
    if (modes_.select(mode_).centered) {
      // Note: Check if you need to invert yaw_command here based on your thruster layout
      mixer_.yaw(cmd, yaw_command);
    }

    // ==========================================

    motor_command_ = cmd;
    std::vector<double> scaled_cmd = cmd;
    for (auto & val : scaled_cmd) {val *= command_multiplier_;}

    publish_cmd(scaled_cmd);
  }

  std::vector<char> determine_keys(const std::vector<double> & dir)
  {
    std::vector<char> keys;
    if (obstacles_.size() > 4 && obstacles_[4] == 0.0) {
      if (dir[0] > 0.1) {
        keys.push_back('w');
      } else if (dir[0] < -0.1) {
        keys.push_back('s');
      }
      if (dir[1] > 0.1) {
        keys.push_back('d');
      } else if (dir[1] < -0.1) {
        keys.push_back('a');
      }
    } else {
      if (dir[0] > 0.1) {
        keys.push_back('d');
      } else if (dir[0] < -0.1) {
        keys.push_back('a');
      }
    }

    if (dir[2] > 0.1) {
      keys.push_back('i');
    } else if (dir[2] < -0.1) {
      keys.push_back('k');
    }
    return keys;
  }

  std::vector<double> apply_yaw_control(double yaw_error, double val, std::vector<double> cmd)
  {
    double yaw_command = std::clamp(kp_yaw_ * yaw_error, -val, val);
    yaw_command = -yaw_command;
    mixer_.yaw(cmd, yaw_command);
    return cmd;
  }
};
}
