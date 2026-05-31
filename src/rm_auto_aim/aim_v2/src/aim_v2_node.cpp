#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/math_tools.hpp"

namespace aim_v2
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double radToDeg(double rad) { return rad * 180.0 / kPi; }

double degToRad(double deg) { return deg * kPi / 180.0; }

double normalizeDeg(double deg)
{
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

std::string armorName(auto_aim::ArmorName name)
{
  const auto index = static_cast<size_t>(name);
  if (index < auto_aim::ARMOR_NAMES.size()) return auto_aim::ARMOR_NAMES[index];
  return "unknown";
}

Eigen::Quaterniond makeGimbalQuaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
  const Eigen::AngleAxisd roll(degToRad(roll_deg), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(degToRad(-pitch_deg), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(degToRad(yaw_deg), Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(yaw * pitch * roll).normalized();
}

}  // namespace

class AimV2Node : public rclcpp::Node
{
public:
  explicit AimV2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aim_v2", options)
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory("aim_v2");
    const auto default_config = (std::filesystem::path(share_dir) / "config" / "aim_v2.yaml").string();

    config_path_ = declare_parameter<std::string>("config_path", default_config);
    detector_type_ = declare_parameter<std::string>("detector_type", "yolo");
    debug_core_ = declare_parameter<bool>("debug_core", false);
    respect_mode_ = declare_parameter<bool>("respect_mode", true);
    enable_fire_ = declare_parameter<bool>("enable_fire", false);
    default_bullet_speed_ = declare_parameter<double>("default_bullet_speed", 23.0);
    auto_aim_modes_ = declare_parameter<std::vector<int64_t>>("auto_aim_modes", {0, 1});
    serial_.bullet_speed = default_bullet_speed_;
    const auto enemy_color = declare_parameter<std::string>("enemy_color", "");

    runtime_config_path_ = materializeRuntimeConfig(config_path_, share_dir, enemy_color);

    solver_ = std::make_unique<auto_aim::Solver>(runtime_config_path_);
    tracker_ = std::make_unique<auto_aim::Tracker>(runtime_config_path_, *solver_);
    aimer_ = std::make_unique<auto_aim::Aimer>(runtime_config_path_);
    shooter_ = std::make_unique<auto_aim::Shooter>(runtime_config_path_);

    if (detector_type_ == "traditional") {
      traditional_detector_ = std::make_unique<auto_aim::Detector>(runtime_config_path_, debug_core_);
    } else {
      yolo_detector_ = std::make_unique<auto_aim::YOLO>(runtime_config_path_, debug_core_);
      detector_type_ = "yolo";
    }

    cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>("cmd_gimbal", rclcpp::SensorDataQoS());

    serial_sub_ = create_subscription<rm_interfaces::msg::SerialReceiveData>(
      "serial/receive", rclcpp::SensorDataQoS(),
      [this](const rm_interfaces::msg::SerialReceiveData::SharedPtr msg) { updateSerial(*msg); });

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { handleImage(msg); });

    set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
      "~/set_mode",
      [this](
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
        {
          std::lock_guard<std::mutex> lock(serial_mutex_);
          serial_.mode = request->mode;
        }
        response->success = true;
        response->message = "aim_v2 mode updated";
      });

    RCLCPP_INFO(
      get_logger(),
      "aim_v2 ready: detector=%s config=%s fire=%s respect_mode=%s",
      detector_type_.c_str(), runtime_config_path_.c_str(), enable_fire_ ? "on" : "off",
      respect_mode_ ? "on" : "off");
  }

private:
  struct SerialState
  {
    bool received = false;
    uint8_t mode = 0;
    double bullet_speed = 23.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
  };

  std::string materializeRuntimeConfig(
    const std::string & source_config, const std::string & share_dir, const std::string & enemy_color)
  {
    YAML::Node yaml = YAML::LoadFile(source_config);
    const auto share_path = std::filesystem::path(share_dir);

    auto resolve_path = [&](const char * key) {
      if (!yaml[key] || !yaml[key].IsScalar()) return;
      std::filesystem::path path(yaml[key].as<std::string>());
      if (path.is_relative()) {
        path = share_path / path;
      }
      yaml[key] = path.lexically_normal().string();
    };

    resolve_path("classify_model");
    resolve_path("yolo11_model_path");
    resolve_path("yolov8_model_path");
    resolve_path("yolov5_model_path");
    resolve_path("model");

    if (!enemy_color.empty()) {
      yaml["enemy_color"] = enemy_color;
    }

    const auto output_path =
      std::filesystem::temp_directory_path() /
      ("aim_v2_runtime_" + std::to_string(now().nanoseconds()) + ".yaml");
    YAML::Emitter emitter;
    emitter << yaml;
    std::ofstream output(output_path);
    output << emitter.c_str();
    output.close();
    return output_path.string();
  }

  void updateSerial(const rm_interfaces::msg::SerialReceiveData & msg)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.received = true;
    serial_.mode = msg.mode;
    serial_.bullet_speed = msg.bullet_speed > 1.0f ? msg.bullet_speed : default_bullet_speed_;
    serial_.roll_deg = msg.roll;
    serial_.pitch_deg = msg.pitch;
    serial_.yaw_deg = msg.yaw;
  }

  SerialState serialSnapshot() const
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    return serial_;
  }

  bool isAutoAimMode(uint8_t mode) const
  {
    for (const auto allowed_mode : auto_aim_modes_) {
      if (allowed_mode == static_cast<int64_t>(mode)) return true;
    }
    return false;
  }

  void handleImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const auto serial = serialSnapshot();
    if (respect_mode_ && !isAutoAimMode(serial.mode)) {
      publishIdle(msg->header);
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "cv_bridge conversion failed: %s", e.what());
      publishIdle(msg->header);
      return;
    }

    if (cv_ptr->image.empty()) {
      publishIdle(msg->header);
      return;
    }

    try {
      const auto timestamp = std::chrono::steady_clock::now();
      solver_->set_R_gimbal2world(
        makeGimbalQuaternion(serial.roll_deg, serial.pitch_deg, serial.yaw_deg));

      std::list<auto_aim::Armor> armors;
      if (detector_type_ == "traditional") {
        armors = traditional_detector_->detect(cv_ptr->image, frame_count_);
      } else {
        armors = yolo_detector_->detect(cv_ptr->image, frame_count_);
      }

      auto targets = tracker_->track(armors, timestamp);
      auto command = aimer_->aim(targets, timestamp, serial.bullet_speed);

      if (command.control && enable_fire_) {
        const Eigen::Vector3d ypr = tools::eulers(solver_->R_gimbal2world(), 2, 1, 0);
        command.shoot = shooter_->shoot(command, *aimer_, targets, ypr);
      } else {
        command.shoot = false;
      }

      publishCommand(msg->header, command, targets, serial);
      frame_count_++;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "aim_v2 frame failed: %s", e.what());
      publishIdle(msg->header);
    }
  }

  void publishIdle(const std_msgs::msg::Header & header)
  {
    const auto serial = serialSnapshot();
    rm_interfaces::msg::GimbalCmd cmd;
    cmd.header = header;
    cmd.yaw = serial.yaw_deg;
    cmd.pitch = serial.pitch_deg;
    cmd.yaw_diff = 0.0;
    cmd.pitch_diff = 0.0;
    cmd.distance = -1.0;
    cmd.fire_advice = false;
    cmd.mode = rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
    cmd_pub_->publish(cmd);
  }

  void publishCommand(
    const std_msgs::msg::Header & header, const io::Command & command,
    const std::list<auto_aim::Target> & targets, const SerialState & serial)
  {
    rm_interfaces::msg::GimbalCmd cmd;
    cmd.header = header;
    cmd.yaw = command.control ? radToDeg(command.yaw) : serial.yaw_deg;
    cmd.pitch = command.control ? radToDeg(command.pitch) : serial.pitch_deg;
    cmd.yaw_diff = normalizeDeg(cmd.yaw - serial.yaw_deg);
    cmd.pitch_diff = cmd.pitch - serial.pitch_deg;
    cmd.distance = estimateDistance(targets);
    cmd.fire_advice = enable_fire_ && command.shoot;
    cmd.mode = command.control ? rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT
                               : rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
    if (!targets.empty()) {
      cmd.target_id = armorName(targets.front().name);
    }
    cmd_pub_->publish(cmd);
  }

  double estimateDistance(const std::list<auto_aim::Target> & targets) const
  {
    if (targets.empty()) return -1.0;
    const auto x = targets.front().ekf_x();
    if (x.size() < 3) return -1.0;
    return std::sqrt(x[0] * x[0] + x[2] * x[2]);
  }

  std::string config_path_;
  std::string runtime_config_path_;
  std::string detector_type_;
  bool debug_core_ = false;
  bool respect_mode_ = true;
  bool enable_fire_ = false;
  double default_bullet_speed_ = 23.0;
  std::vector<int64_t> auto_aim_modes_;
  int frame_count_ = 0;

  mutable std::mutex serial_mutex_;
  SerialState serial_;

  std::unique_ptr<auto_aim::Solver> solver_;
  std::unique_ptr<auto_aim::Tracker> tracker_;
  std::unique_ptr<auto_aim::Aimer> aimer_;
  std::unique_ptr<auto_aim::Shooter> shooter_;
  std::unique_ptr<auto_aim::YOLO> yolo_detector_;
  std::unique_ptr<auto_aim::Detector> traditional_detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
};

}  // namespace aim_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aim_v2::AimV2Node>());
  rclcpp::shutdown();
  return 0;
}
