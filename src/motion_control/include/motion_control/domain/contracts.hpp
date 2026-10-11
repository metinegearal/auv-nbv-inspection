#pragma once
#include <vector>
#include <string>
#include <memory>
#include <cstdio>
#include <cstddef>
namespace motion_control
{
struct Position { const double x, y, z; };
struct MotorCommand { const std::vector<double> values; };
struct WaypointInput { const std::vector<float> data; };
struct ControlConfig
{
  const double kp_yaw, kp_dist;
  const bool dvl_active;
  const double stopping_distance, initial_waypoint_x, initial_waypoint_y,
    initial_waypoint_z, max_yaw_command, yaw_smoothing_rate,
    min_forward_thrust, max_forward_thrust, command_multiplier;
};
struct QueueConfig
{
  const double initial_waypoint_x,
    initial_waypoint_y, initial_waypoint_z, initial_waypoint_yaw,
    timer_period_s, waypoint_reached_distance;
};
struct ControlMode { const double vertical_multiplier; const bool centered, accepts_angle; };
class Modes {
public:
  virtual ~Modes() = default; virtual ControlMode select(const std::string &) const= 0;
};
class Clock {
public:
  virtual ~Clock() = default; virtual double now() const= 0;
};
class Gains {
public:
  virtual ~Gains() = default; virtual double yaw() const= 0; virtual double distance() const= 0;
};
class Diagnostics {
public:
  virtual ~Diagnostics() = default; virtual void write(const std::string &) = 0;
};
class CommandOutput {
public:
  virtual ~CommandOutput() = default;
  virtual void command(const MotorCommand &) = 0;
  virtual void reached(const std::vector<float> &) = 0;
};
class WaypointOutput {
public:
  virtual ~WaypointOutput() = default;
  virtual void waypoints(const std::vector<float> &, std::size_t) = 0;
  virtual void request_nbv() = 0;
};
class ThrusterMixer {
public:
  virtual ~ThrusterMixer() = default;
  virtual std::vector<double> mix(
    const std::vector<char> &, double, std::vector<double>,
    double) const= 0;
  virtual void yaw(std::vector<double> &, double) const= 0;
};
inline std::string format_text(const char * value) {return value;}
template<class ... Args> std::string format_text(const char * format, Args... args)
{
  const int size = std::snprintf(nullptr, 0, format, args ...);
  std::vector<char> text(static_cast<std::size_t>(size) + 1);
  std::snprintf(text.data(), text.size(), format, args ...);
  return std::string(text.data(), static_cast<std::size_t>(size));
}
}
