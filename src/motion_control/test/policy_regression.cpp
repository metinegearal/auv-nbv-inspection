#include <cmath>
#include <cstdlib>
#include <vector>
#include "motion_control/use_cases/trajectory_queue.hpp"
#include "motion_control/adapters/thrusters.hpp"

void require(bool condition) {if(!condition) {std::abort();}}
struct Output final : motion_control::WaypointOutput, motion_control::Diagnostics
{
  std::vector<std::vector<float>> messages;
  std::size_t requests = 0;
  void waypoints(const std::vector<float> & values, std::size_t count) override
  {
    require(values.size() == count * 4); messages.push_back(values);
  }
  void request_nbv() override {++requests;}
  void write(const std::string &) override {}
};
int main()
{
  Output output;
  motion_control::TrajectoryQueue queue({.35, .3875, -.5, 354., 828., -298., 0., 1., .5}, output,
    output);
  queue.timer_callback();
  require(output.messages.back() == std::vector<float>({354.f, 828.f, -298.f, 0.f}));
  queue.pointSegment_callback({{10.f, 20.f, -30.f, 1.f, 11.f, 21.f, -31.f, 2.f}});
  const auto encoded = output.messages.back();
  require(encoded.size() == 8 && std::abs(encoded[0] - 9.65f) < 1e-6);
  require(std::abs(encoded[1] - 19.6125f) < 1e-6 && encoded[2] == -29.5f && encoded[3] == 1.f);
  // Preserve the existing empty-message behavior: the queue retains its route.
  queue.pointSegment_callback({{}});
  queue.timer_callback();
  require(output.messages.back() == encoded);
  queue.stop_callback({{encoded[0], encoded[1], encoded[2]}});
  require(output.messages.back().size() == 4);
  const auto last = output.messages.back();
  queue.stop_callback({{last[0], last[1], last[2]}});
  require(output.requests == 1);
  motion_control::EightThrusterMixer mixer;
  auto mixed = mixer.mix({'w', 'a', 'i'}, 2., std::vector<double>(8, 0.), 3.);
  require(mixed == std::vector<double>({3., 3., 3., 3., 0., -4., 0., -4.}));
  mixer.yaw(mixed, 1.);
  require(mixed == std::vector<double>({3., 3., 3., 3., -1., -3., 1., -5.}));
  motion_control::ConfiguredModes modes;
  require(modes.select("full_centerize").vertical_multiplier == .25);
  require(!modes.select("unknown").centered);
}
