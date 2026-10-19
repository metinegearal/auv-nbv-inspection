#pragma once
#include <algorithm>
#include <map>
#include "auv_motion_control/domain/contracts.hpp"
namespace motion_control
{
class ConfiguredModes final : public Modes {
  const std::map<std::string, ControlMode> modes_;

public:
  ConfiguredModes()
  : modes_{{"yaw_centerize", {0.6, true, false}},
      {"full_centerize", {0.25, true, false}}, {"no_centerize", {0.6, false, true}}} {}
  ControlMode select(const std::string & name) const override
  {
    const auto mode = modes_.find(name);
    return mode == modes_.end() ? ControlMode{0.6, false, false} : mode->second;
  }
};
class EightThrusterMixer final : public ThrusterMixer {
public:
  std::vector<double> mix(
    const std::vector<char> & keys, double val, std::vector<double> cmd,
    double vertical_val) const override
  {
    auto has_key = [&](char k) {return std::find(keys.begin(), keys.end(), k) != keys.end();};

    if (has_key('i')) {
      for (int i = 0; i < 4; i++) {cmd[i] += vertical_val;}
    }
    if (has_key('k')) {
      for (int i = 0; i < 4; i++) {cmd[i] -= vertical_val;}
    }
    if (has_key('j')) {
      cmd[4] += val;
      cmd[7] += val;
      cmd[5] -= val;
      cmd[6] -= val;
    }
    if (has_key('l')) {
      cmd[4] -= val;
      cmd[7] -= val;
      cmd[5] += val;
      cmd[6] += val;
    }
    if (has_key('w')) {
      for (int i = 4; i < 8; i++) {cmd[i] -= val;}
    }
    if (has_key('s')) {
      for (int i = 4; i < 8; i++) {cmd[i] += val;}
    }
    if (has_key('a')) {
      cmd[4] += val;
      cmd[6] += val;
      cmd[5] -= val;
      cmd[7] -= val;
    }
    if (has_key('d')) {
      cmd[4] -= val;
      cmd[6] -= val;
      cmd[5] += val;
      cmd[7] += val;
    }
    return cmd;
  }
  void yaw(std::vector<double> & cmd, double yaw_command) const override
  {
    cmd[4] -= yaw_command;
    cmd[7] -= yaw_command;
    cmd[5] += yaw_command;
    cmd[6] += yaw_command;
  }
};
}
