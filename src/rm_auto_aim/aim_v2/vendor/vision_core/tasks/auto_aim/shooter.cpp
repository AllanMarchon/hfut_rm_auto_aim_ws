#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector2d & current_yaw_pitch)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  const auto yaw_command_delta = std::abs(tools::limit_rad(last_command_.yaw - command.yaw));
  const auto pitch_command_delta = std::abs(last_command_.pitch - command.pitch);
  const auto yaw_tracking_error = std::abs(tools::limit_rad(current_yaw_pitch[0] - last_command_.yaw));
  const auto pitch_tracking_error = std::abs(current_yaw_pitch[1] - last_command_.pitch);

  if (
    yaw_command_delta < tolerance * 2 && pitch_command_delta < tolerance * 2 &&
    yaw_tracking_error < tolerance && pitch_tracking_error < tolerance && aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim
