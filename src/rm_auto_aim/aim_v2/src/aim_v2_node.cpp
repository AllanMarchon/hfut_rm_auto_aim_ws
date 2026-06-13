#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
#include <rm_utils/heartbeat.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <list>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

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

const char * colorName(auto_aim::Color color)
{
  return color == auto_aim::blue ? "blue" : "red";
}

std::optional<auto_aim::Color> enemyColorFromMode(uint8_t mode)
{
  if (mode == 0) return auto_aim::red;
  if (mode == 1) return auto_aim::blue;
  return std::nullopt;
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
    control_backend_ = declare_parameter<std::string>("control_backend", "planner");
    debug_core_ = declare_parameter<bool>("debug_core", false);
    respect_mode_ = declare_parameter<bool>("respect_mode", true);
    require_serial_ = declare_parameter<bool>("require_serial", true);
    derive_enemy_color_from_mode_ = declare_parameter<bool>("derive_enemy_color_from_mode", true);
    async_inference_ = declare_parameter<bool>("async_inference", true);
    enable_fire_ = declare_parameter<bool>("enable_fire", false);
    debug_visualization_ = declare_parameter<bool>("debug_visualization", true);
    debug_marker_frame_ = declare_parameter<std::string>("debug_marker_frame", "odom");
    default_bullet_speed_ = declare_parameter<double>("default_bullet_speed", 23.0);
    auto_aim_modes_ = declare_parameter<std::vector<int64_t>>("auto_aim_modes", {0, 1});
    serial_.bullet_speed = default_bullet_speed_;
    const auto enemy_color = declare_parameter<std::string>("enemy_color", "");
    const auto runtime_config_overrides = declareRuntimeConfigOverrides(config_path_);

    runtime_config_path_ =
      materializeRuntimeConfig(config_path_, share_dir, enemy_color, runtime_config_overrides);

    solver_ = std::make_unique<auto_aim::Solver>(runtime_config_path_);
    tracker_ = std::make_unique<auto_aim::Tracker>(runtime_config_path_, *solver_);
    aimer_ = std::make_unique<auto_aim::Aimer>(runtime_config_path_);
    shooter_ = std::make_unique<auto_aim::Shooter>(runtime_config_path_);
    if (control_backend_ != "planner" && control_backend_ != "aimer") {
      RCLCPP_WARN(
        get_logger(), "Unknown control_backend '%s', falling back to aimer", control_backend_.c_str());
      control_backend_ = "aimer";
    }
    use_planner_ = control_backend_ == "planner";
    if (use_planner_) {
      planner_ = std::make_unique<auto_aim::Planner>(runtime_config_path_);
    }
    current_enemy_color_ = loadConfiguredEnemyColor(runtime_config_path_);
    const auto yolo_name = loadYoloName(runtime_config_path_);

    if (detector_type_ == "traditional") {
      async_inference_ = false;
      traditional_detector_ = std::make_unique<auto_aim::Detector>(runtime_config_path_, debug_core_);
    } else {
      detector_type_ = "yolo";
      if (async_inference_ && yolo_name == "yolov8") {
        async_inference_ = false;
        RCLCPP_WARN(get_logger(), "SP async detector is disabled for yolov8 because it uses 416 input");
      }
      if (async_inference_) {
        async_detector_ =
          std::make_unique<auto_aim::multithread::MultiThreadDetector>(runtime_config_path_, debug_core_);
      } else {
        yolo_detector_ = std::make_unique<auto_aim::YOLO>(runtime_config_path_, debug_core_);
      }
    }

    cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>("cmd_gimbal", rclcpp::SensorDataQoS());
    if (debug_visualization_) {
      debug_image_pub_ =
        create_publisher<sensor_msgs::msg::Image>("~/debug/image", rclcpp::SensorDataQoS());
      debug_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 10);
      detector_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 10);
      solver_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
    }

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
        updateEnemyColorFromMode(request->mode);
        response->success = true;
        response->message = "aim_v2 mode updated";
      });

    heartbeat_ = fyt::HeartBeatPublisher::create(this);

    if (async_inference_) {
      async_running_.store(true);
      async_worker_ = std::thread(&AimV2Node::asyncResultLoop, this);
    }

    RCLCPP_INFO(
      get_logger(),
      "aim_v2 ready: detector=%s async=%s control=%s config=%s fire=%s respect_mode=%s",
      detector_type_.c_str(), async_inference_ ? "on" : "off",
      control_backend_.c_str(),
      runtime_config_path_.c_str(), enable_fire_ ? "on" : "off", respect_mode_ ? "on" : "off");
  }

  ~AimV2Node() override
  {
    async_running_.store(false);
    if (async_worker_.joinable()) {
      async_worker_.join();
    }
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

  struct AsyncFrameContext
  {
    std_msgs::msg::Header header;
    SerialState serial;
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat image;
  };

  struct ControlOutput
  {
    bool control = false;
    bool shoot = false;
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_vel_rad = 0.0;
    double pitch_vel_rad = 0.0;
    double yaw_acc_rad = 0.0;
    double pitch_acc_rad = 0.0;
  };

  struct RuntimeConfigOverrides
  {
    std::vector<double> camera_matrix;
    std::vector<double> distort_coeffs;
    std::vector<double> R_camera2gimbal;
    std::vector<double> t_camera2gimbal;
    double yaw_offset = 0.0;
    double pitch_offset = 0.0;
  };

  RuntimeConfigOverrides declareRuntimeConfigOverrides(const std::string & source_config)
  {
    const auto yaml = YAML::LoadFile(source_config);
    auto read_vector = [&](const char * key, std::size_t expected_size) {
      auto values = yaml[key].as<std::vector<double>>();
      if (values.size() != expected_size) {
        throw std::runtime_error(
          std::string("aim_v2 config key '") + key + "' expects " +
          std::to_string(expected_size) + " values, got " + std::to_string(values.size()));
      }
      return values;
    };

    RuntimeConfigOverrides overrides;
    overrides.camera_matrix =
      declare_parameter<std::vector<double>>("camera_matrix", read_vector("camera_matrix", 9));
    overrides.distort_coeffs =
      declare_parameter<std::vector<double>>("distort_coeffs", read_vector("distort_coeffs", 5));
    overrides.R_camera2gimbal =
      declare_parameter<std::vector<double>>("R_camera2gimbal", read_vector("R_camera2gimbal", 9));
    overrides.t_camera2gimbal =
      declare_parameter<std::vector<double>>("t_camera2gimbal", read_vector("t_camera2gimbal", 3));
    overrides.yaw_offset = declare_parameter<double>("yaw_offset", yaml["yaw_offset"].as<double>());
    overrides.pitch_offset =
      declare_parameter<double>("pitch_offset", yaml["pitch_offset"].as<double>());
    return overrides;
  }

  static void applyVectorOverride(
    YAML::Node & yaml, const char * key, const std::vector<double> & values,
    std::size_t expected_size)
  {
    if (values.empty()) return;
    if (values.size() != expected_size) {
      throw std::runtime_error(
        std::string("aim_v2 parameter '") + key + "' expects " +
        std::to_string(expected_size) + " values, got " + std::to_string(values.size()));
    }
    yaml[key] = values;
  }

  static void applyScalarOverride(YAML::Node & yaml, const char * key, double value)
  {
    yaml[key] = value;
  }

  std::string materializeRuntimeConfig(
    const std::string & source_config, const std::string & share_dir, const std::string & enemy_color,
    const RuntimeConfigOverrides & overrides)
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

    applyVectorOverride(yaml, "camera_matrix", overrides.camera_matrix, 9);
    applyVectorOverride(yaml, "distort_coeffs", overrides.distort_coeffs, 5);
    applyVectorOverride(yaml, "R_camera2gimbal", overrides.R_camera2gimbal, 9);
    applyVectorOverride(yaml, "t_camera2gimbal", overrides.t_camera2gimbal, 3);
    applyScalarOverride(yaml, "yaw_offset", overrides.yaw_offset);
    applyScalarOverride(yaml, "pitch_offset", overrides.pitch_offset);

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
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_.received = true;
      serial_.mode = msg.mode;
      serial_.bullet_speed = msg.bullet_speed > 1.0f ? msg.bullet_speed : default_bullet_speed_;
      serial_.roll_deg = msg.roll;
      serial_.pitch_deg = msg.pitch;
      serial_.yaw_deg = msg.yaw;
    }
    updateEnemyColorFromMode(msg.mode);
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
    if (require_serial_ && !serial.received) {
      publishIdle(msg->header);
      return;
    }

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
      if (async_inference_) {
        if (async_detector_->push(cv_ptr->image, timestamp)) {
          pushAsyncContext(
            {msg->header, serial, timestamp,
             debug_visualization_ ? cv_ptr->image.clone() : cv::Mat{}});
          frame_count_++;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "aim_v2 async detector queue is full");
        }
        return;
      }

      std::list<auto_aim::Armor> armors;
      if (detector_type_ == "traditional") {
        armors = traditional_detector_->detect(cv_ptr->image, frame_count_);
      } else {
        armors = yolo_detector_->detect(cv_ptr->image, frame_count_);
      }

      processDetectedFrame(msg->header, cv_ptr->image, armors, serial, timestamp);
      frame_count_++;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "aim_v2 frame failed: %s", e.what());
      publishIdle(msg->header);
    }
  }

  void asyncResultLoop()
  {
    while (async_running_.load()) {
      std::list<auto_aim::Armor> armors;
      std::chrono::steady_clock::time_point timestamp;
      if (!async_detector_->try_pop(armors, timestamp)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      auto context = waitForAsyncContext(timestamp);
      if (!context) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "aim_v2 async frame context was not matched");
        continue;
      }

      processDetectedFrame(context->header, context->image, armors, context->serial, timestamp);
    }
  }

  void processDetectedFrame(
    const std_msgs::msg::Header & header, const cv::Mat & image,
    std::list<auto_aim::Armor> & armors, const SerialState & serial,
    std::chrono::steady_clock::time_point timestamp)
  {
    try {
      std::list<auto_aim::Target> targets;
      ControlOutput output;
      auto detected_armors = armors;

      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        solver_->set_R_gimbal2world(
          makeGimbalQuaternion(serial.roll_deg, serial.pitch_deg, serial.yaw_deg));
        solveDetectedArmors(detected_armors);
        auto tracking_armors = detected_armors;
        tracking_armors.remove_if(
          [this](const auto_aim::Armor & armor) { return !finite3(armor.xyz_in_world); });
        targets = tracker_->track(tracking_armors, timestamp);
        output = buildControlOutput(targets, timestamp, serial);
      }

      publishCommand(header, output, targets, serial);
      publishDebug(header, image, detected_armors, targets, output);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "aim_v2 frame failed: %s", e.what());
      publishIdle(header);
    }
  }

  ControlOutput buildControlOutput(
    const std::list<auto_aim::Target> & targets,
    std::chrono::steady_clock::time_point timestamp, const SerialState & serial)
  {
    if (use_planner_) {
      return buildPlannerOutput(targets, serial);
    }
    return buildAimerOutput(targets, timestamp, serial);
  }

  ControlOutput buildPlannerOutput(
    const std::list<auto_aim::Target> & targets, const SerialState & serial)
  {
    std::optional<auto_aim::Target> target;
    if (!targets.empty()) {
      target = targets.front();
    }

    const auto plan = planner_->plan(target, serial.bullet_speed);
    return {
      plan.control,
      enable_fire_ && plan.fire,
      plan.yaw,
      plan.pitch,
      plan.yaw_vel,
      plan.pitch_vel,
      plan.yaw_acc,
      plan.pitch_acc,
    };
  }

  ControlOutput buildAimerOutput(
    const std::list<auto_aim::Target> & targets,
    std::chrono::steady_clock::time_point timestamp, const SerialState & serial)
  {
    auto command = aimer_->aim(targets, timestamp, serial.bullet_speed);

    if (command.control && enable_fire_) {
      const Eigen::Vector2d current_yaw_pitch{degToRad(serial.yaw_deg), degToRad(serial.pitch_deg)};
      command.shoot = shooter_->shoot(command, *aimer_, targets, current_yaw_pitch);
    } else {
      command.shoot = false;
    }

    return {
      command.control,
      command.shoot,
      command.yaw,
      command.pitch,
      0.0,
      0.0,
      0.0,
      0.0,
    };
  }

  void pushAsyncContext(const AsyncFrameContext & context)
  {
    std::lock_guard<std::mutex> lock(async_context_mutex_);
    async_contexts_.push(context);
  }

  std::optional<AsyncFrameContext> waitForAsyncContext(
    std::chrono::steady_clock::time_point timestamp)
  {
    constexpr auto kMaxWait = std::chrono::milliseconds(5);
    const auto deadline = std::chrono::steady_clock::now() + kMaxWait;
    while (async_running_.load() && std::chrono::steady_clock::now() < deadline) {
      auto context = popAsyncContext(timestamp);
      if (context) return context;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return popAsyncContext(timestamp);
  }

  std::optional<AsyncFrameContext> popAsyncContext(std::chrono::steady_clock::time_point timestamp)
  {
    std::lock_guard<std::mutex> lock(async_context_mutex_);
    if (async_contexts_.empty()) return std::nullopt;

    while (!async_contexts_.empty() && async_contexts_.front().timestamp != timestamp) {
      async_contexts_.pop();
    }
    if (async_contexts_.empty()) return std::nullopt;

    auto context = async_contexts_.front();
    async_contexts_.pop();
    return context;
  }

  void invalidateArmorPose(auto_aim::Armor & armor) const
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    armor.xyz_in_gimbal = Eigen::Vector3d(nan, nan, nan);
    armor.xyz_in_world = Eigen::Vector3d(nan, nan, nan);
    armor.ypr_in_gimbal = Eigen::Vector3d(nan, nan, nan);
    armor.ypr_in_world = Eigen::Vector3d(nan, nan, nan);
    armor.ypd_in_world = Eigen::Vector3d(nan, nan, nan);
  }

  void solveDetectedArmors(std::list<auto_aim::Armor> & armors)
  {
    for (auto & armor : armors) {
      invalidateArmorPose(armor);
      if (armor.points.size() < 4) continue;

      try {
        solver_->solve(armor);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "aim_v2 solve detected armor failed: %s", e.what());
        invalidateArmorPose(armor);
      }
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
    cmd.yaw_v = 0.0;
    cmd.pitch_v = 0.0;
    cmd.yaw_a = 0.0;
    cmd.pitch_a = 0.0;
    cmd.distance = -1.0;
    cmd.fire_advice = false;
    cmd.mode = rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
    cmd_pub_->publish(cmd);
  }

  void publishCommand(
    const std_msgs::msg::Header & header, const ControlOutput & output,
    const std::list<auto_aim::Target> & targets, const SerialState & serial)
  {
    const bool has_control = output.control && isFiniteOutput(output);
    rm_interfaces::msg::GimbalCmd cmd;
    cmd.header = header;
    cmd.yaw = has_control ? radToDeg(output.yaw_rad) : serial.yaw_deg;
    cmd.pitch = has_control ? radToDeg(output.pitch_rad) : serial.pitch_deg;
    cmd.yaw_diff = normalizeDeg(cmd.yaw - serial.yaw_deg);
    cmd.pitch_diff = cmd.pitch - serial.pitch_deg;
    cmd.yaw_v = has_control ? radToDeg(output.yaw_vel_rad) : 0.0;
    cmd.pitch_v = has_control ? radToDeg(output.pitch_vel_rad) : 0.0;
    cmd.yaw_a = has_control ? radToDeg(output.yaw_acc_rad) : 0.0;
    cmd.pitch_a = has_control ? radToDeg(output.pitch_acc_rad) : 0.0;
    cmd.distance = estimateDistance(targets);
    cmd.fire_advice = has_control && output.shoot;
    cmd.mode = has_control ? rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT
                           : rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
    if (!targets.empty()) {
      cmd.target_id = armorName(targets.front().name);
    }
    cmd_pub_->publish(cmd);
  }

  void publishDebug(
    const std_msgs::msg::Header & header, const cv::Mat & image,
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    const ControlOutput & output)
  {
    if (!debug_visualization_) return;
    publishDebugImage(header, image, armors, targets, output);
    publishDebugMarkers(header, armors, targets, output);
  }

  cv::Scalar armorDrawColor(auto_aim::Color color) const
  {
    if (color == auto_aim::red) return {0, 0, 255};
    if (color == auto_aim::blue) return {255, 80, 0};
    if (color == auto_aim::purple) return {255, 0, 255};
    return {160, 160, 160};
  }

  std::string armorColorLabel(auto_aim::Color color) const
  {
    const auto index = static_cast<std::size_t>(color);
    if (index < auto_aim::COLORS.size()) return auto_aim::COLORS[index];
    return "unknown";
  }

  bool finitePoint(const cv::Point2f & p) const
  {
    return std::isfinite(p.x) && std::isfinite(p.y);
  }

  bool finite4(const Eigen::Vector4d & p) const
  {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
           std::isfinite(p[3]);
  }

  double armorMarkerWidth(auto_aim::ArmorType type) const
  {
    return type == auto_aim::big ? 0.23 : 0.135;
  }

  std::optional<Eigen::Vector4d> selectedAimXyza(const ControlOutput & output) const
  {
    if (!output.control) return std::nullopt;

    if (use_planner_) {
      if (planner_ && finite4(planner_->debug_xyza)) return planner_->debug_xyza;
      return std::nullopt;
    }

    if (aimer_ && aimer_->debug_aim_point.valid && finite4(aimer_->debug_aim_point.xyza)) {
      return aimer_->debug_aim_point.xyza;
    }
    return std::nullopt;
  }

  void drawProjectedArmor(
    cv::Mat & vis, const std::vector<cv::Point2f> & points, const cv::Scalar & color,
    int thickness) const
  {
    if (points.size() < 4) return;
    for (const auto & point : points) {
      if (!finitePoint(point)) return;
    }

    for (std::size_t i = 0; i < points.size(); ++i) {
      const cv::Point p0(points[i]);
      const cv::Point p1(points[(i + 1) % points.size()]);
      cv::line(vis, p0, p1, color, thickness, cv::LINE_AA);
      cv::circle(vis, p0, 3, color, -1, cv::LINE_AA);
    }
  }

  void drawAimCross(cv::Mat & vis, const cv::Point2f & point, const cv::Scalar & color) const
  {
    if (!finitePoint(point)) return;

    constexpr int kHalfSize = 10;
    const cv::Point center(point);
    cv::line(
      vis, cv::Point(center.x - kHalfSize, center.y), cv::Point(center.x + kHalfSize, center.y),
      color, 2, cv::LINE_AA);
    cv::line(
      vis, cv::Point(center.x, center.y - kHalfSize), cv::Point(center.x, center.y + kHalfSize),
      color, 2, cv::LINE_AA);
    cv::circle(vis, center, 6, color, 2, cv::LINE_AA);
    cv::putText(
      vis, "AIM", center + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
      cv::LINE_AA);
  }

  void publishDebugImage(
    const std_msgs::msg::Header & header, const cv::Mat & image,
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    const ControlOutput & output)
  {
    if (!debug_image_pub_ || image.empty()) return;

    cv::Mat vis;
    image.copyTo(vis);
    const cv::Rect frame_rect(0, 0, vis.cols, vis.rows);

    for (const auto & armor : armors) {
      const auto color = armorDrawColor(armor.color);
      if (armor.box.width > 0 && armor.box.height > 0) {
        const auto box = armor.box & frame_rect;
        if (!box.empty()) cv::rectangle(vis, box, color, 2, cv::LINE_AA);
      }

      if (armor.points.size() >= 4) {
        drawProjectedArmor(vis, armor.points, color, 2);
      }

      std::ostringstream label;
      label << "D " << armorName(armor.name) << " " << armorColorLabel(armor.color) << " "
            << static_cast<int>(std::round(armor.confidence * 100.0)) << "%";
      if (finite3(armor.xyz_in_world)) {
        label << " " << std::fixed << std::setprecision(1) << armor.xyz_in_world.norm() << "m";
      }
      const auto text_origin = armor.box.empty()
        ? cv::Point(armor.center)
        : cv::Point(armor.box.x, std::max(12, armor.box.y - 6));
      cv::putText(
        vis, label.str(), text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    int target_id = 0;
    for (const auto & target : targets) {
      int armor_id = 0;
      for (const auto & xyza : target.armor_xyza_list()) {
        if (!finite4(xyza)) continue;
        const Eigen::Vector3d xyz = xyza.head<3>();
        const auto projected =
          solver_->reproject_armor(xyz, xyza[3], target.armor_type, target.name);
        drawProjectedArmor(vis, projected, cv::Scalar(0, 220, 255), 2);

        if (!projected.empty() && finitePoint(projected.front())) {
          std::ostringstream label;
          label << "T" << target_id << ":" << armor_id;
          cv::putText(
            vis, label.str(), cv::Point(projected.front()) + cv::Point(4, -4),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
        }
        ++armor_id;
      }
      ++target_id;
    }

    const auto aim_xyza = selectedAimXyza(output);
    if (aim_xyza) {
      const std::vector<cv::Point3f> aim_world{
        {static_cast<float>((*aim_xyza)[0]), static_cast<float>((*aim_xyza)[1]),
         static_cast<float>((*aim_xyza)[2])}};
      const auto pixels = solver_->world2pixel(aim_world);
      if (!pixels.empty()) drawAimCross(vis, pixels.front(), cv::Scalar(255, 0, 255));
    }

    std::ostringstream status;
    status << "det=" << armors.size() << " targets=" << targets.size()
           << " ctrl=" << (output.control ? "yes" : "no")
           << " yaw=" << std::fixed << std::setprecision(1) << radToDeg(output.yaw_rad)
           << " pitch=" << radToDeg(output.pitch_rad) << " fire="
           << (output.shoot ? "yes" : "no");
    cv::putText(
      vis, status.str(), cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.65,
      cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    auto msg = cv_bridge::CvImage(header, "bgr8", vis).toImageMsg();
    debug_image_pub_->publish(*msg);
  }

  bool finite3(const Eigen::Vector3d & p) const
  {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
  }

  void setMarkerColor(
    visualization_msgs::msg::Marker & marker, double r, double g, double b, double a) const
  {
    marker.color.r = static_cast<float>(r);
    marker.color.g = static_cast<float>(g);
    marker.color.b = static_cast<float>(b);
    marker.color.a = static_cast<float>(a);
  }

  void setMarkerPosition(visualization_msgs::msg::Marker & marker, const Eigen::Vector3d & p) const
  {
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation.w = 1.0;
  }

  void setMarkerQuaternion(
    visualization_msgs::msg::Marker & marker, const Eigen::Quaterniond & q) const
  {
    const auto normalized = q.normalized();
    marker.pose.orientation.x = normalized.x();
    marker.pose.orientation.y = normalized.y();
    marker.pose.orientation.z = normalized.z();
    marker.pose.orientation.w = normalized.w();
  }

  void setMarkerYawPitch(
    visualization_msgs::msg::Marker & marker, double yaw, double pitch) const
  {
    const Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    setMarkerQuaternion(marker, q);
  }

  void setDetectedArmorOrientation(
    visualization_msgs::msg::Marker & marker, const auto_aim::Armor & armor) const
  {
    const Eigen::Quaterniond q =
      Eigen::AngleAxisd(armor.ypr_in_world[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(armor.ypr_in_world[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(armor.ypr_in_world[2], Eigen::Vector3d::UnitX());
    setMarkerQuaternion(marker, q);
  }

  void setArmorMarkerShape(
    visualization_msgs::msg::Marker & marker, auto_aim::ArmorType type, double height) const
  {
    marker.scale.x = 0.03;
    marker.scale.y = armorMarkerWidth(type);
    marker.scale.z = height;
  }

  geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & p) const
  {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
  }

  void setPredictionColor(visualization_msgs::msg::Marker & marker, int step) const
  {
    switch (step % 6) {
      case 0:
        setMarkerColor(marker, 1.0, 0.0, 0.0, 1.0);
        break;
      case 1:
        setMarkerColor(marker, 1.0, 0.6, 0.0, 1.0);
        break;
      case 2:
        setMarkerColor(marker, 1.0, 1.0, 0.0, 1.0);
        break;
      case 3:
        setMarkerColor(marker, 0.0, 1.0, 0.0, 1.0);
        break;
      case 4:
        setMarkerColor(marker, 0.0, 0.7, 1.0, 1.0);
        break;
      default:
        setMarkerColor(marker, 0.8, 0.0, 1.0, 1.0);
        break;
    }
    marker.color.a = static_cast<float>(std::max(0.15, 1.0 - step * 0.1));
  }

  visualization_msgs::msg::Marker baseMarker(
    const std_msgs::msg::Header & header, const std::string & ns, int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = debug_marker_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 250000000;
    return marker;
  }

  visualization_msgs::msg::Marker clearMarker(const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.header.frame_id = debug_marker_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    return clear;
  }

  visualization_msgs::msg::MarkerArray buildDetectorMarkerArray(
    const std_msgs::msg::Header & header, const std::list<auto_aim::Armor> & armors) const
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(clearMarker(header));

    int armor_id = 0;
    int text_id = 0;
    for (const auto & armor : armors) {
      if (!finite3(armor.xyz_in_world)) continue;

      auto armor_marker =
        baseMarker(header, "armors", armor_id++, visualization_msgs::msg::Marker::CUBE);
      setMarkerPosition(armor_marker, armor.xyz_in_world);
      setDetectedArmorOrientation(armor_marker, armor);
      armor_marker.scale.x = 0.03;
      armor_marker.scale.y = 0.15;
      armor_marker.scale.z = 0.12;
      setMarkerColor(armor_marker, 1.0, 0.0, 0.0, 1.0);
      armor_marker.lifetime.nanosec = 100000000;
      marker_array.markers.emplace_back(armor_marker);

      auto text_marker = baseMarker(
        header, "classification", text_id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      setMarkerPosition(text_marker, armor.xyz_in_world + Eigen::Vector3d(0.0, -0.1, 0.0));
      text_marker.scale.z = 0.1;
      setMarkerColor(text_marker, 1.0, 1.0, 1.0, 1.0);
      text_marker.text = armorName(armor.name);
      text_marker.lifetime.nanosec = 100000000;
      marker_array.markers.emplace_back(text_marker);
    }

    return marker_array;
  }

  visualization_msgs::msg::MarkerArray buildSolverMarkerArray(
    const std_msgs::msg::Header & header, const std::list<auto_aim::Armor> & armors,
    const std::list<auto_aim::Target> & targets, const ControlOutput & output) const
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(clearMarker(header));

    auto armor_points =
      baseMarker(header, "armor_points", 0, visualization_msgs::msg::Marker::POINTS);
    armor_points.scale.x = 0.1;
    armor_points.scale.y = 0.1;
    setMarkerColor(armor_points, 0.0, 1.0, 0.0, 1.0);
    for (const auto & armor : armors) {
      if (finite3(armor.xyz_in_world)) armor_points.points.emplace_back(toPoint(armor.xyz_in_world));
    }
    marker_array.markers.emplace_back(armor_points);

    if (targets.empty()) return marker_array;

    const auto & target = targets.front();
    const auto x = target.ekf_x();
    if (x.size() < 8) return marker_array;

    const Eigen::Vector3d center{x[0], x[2], x[4]};
    const Eigen::Vector3d velocity{x[1], x[3], x[5]};
    if (!finite3(center)) return marker_array;

    auto position_marker =
      baseMarker(header, "position", 0, visualization_msgs::msg::Marker::SPHERE);
    setMarkerPosition(position_marker, center);
    position_marker.scale.x = 0.1;
    position_marker.scale.y = 0.1;
    position_marker.scale.z = 0.1;
    setMarkerColor(position_marker, 0.0, 1.0, 0.0, 1.0);
    marker_array.markers.emplace_back(position_marker);

    auto linear_v_marker =
      baseMarker(header, "linear_v", 0, visualization_msgs::msg::Marker::ARROW);
    linear_v_marker.points.emplace_back(toPoint(center));
    linear_v_marker.points.emplace_back(toPoint(center + velocity));
    linear_v_marker.scale.x = 0.03;
    linear_v_marker.scale.y = 0.05;
    setMarkerColor(linear_v_marker, 1.0, 1.0, 0.0, 1.0);
    marker_array.markers.emplace_back(linear_v_marker);

    auto angular_v_marker =
      baseMarker(header, "angular_v", 0, visualization_msgs::msg::Marker::ARROW);
    angular_v_marker.points.emplace_back(toPoint(center));
    angular_v_marker.points.emplace_back(toPoint(center + Eigen::Vector3d(0.0, 0.0, x[7] / kPi)));
    angular_v_marker.scale.x = 0.03;
    angular_v_marker.scale.y = 0.05;
    setMarkerColor(angular_v_marker, 0.0, 1.0, 1.0, 1.0);
    marker_array.markers.emplace_back(angular_v_marker);

    int filtered_id = 0;
    for (const auto & xyza : target.armor_xyza_list()) {
      if (!finite4(xyza)) continue;

      const Eigen::Vector3d armor_pos{xyza[0], xyza[1], xyza[2]};
      if (!finite3(armor_pos)) continue;

      auto filtered_marker =
        baseMarker(header, "filtered_armors", filtered_id++, visualization_msgs::msg::Marker::CUBE);
      setMarkerPosition(filtered_marker, armor_pos);
      setMarkerYawPitch(
        filtered_marker, xyza[3],
        target.name == auto_aim::outpost ? -15.0 * kPi / 180.0 : 15.0 * kPi / 180.0);
      setArmorMarkerShape(filtered_marker, target.armor_type, 0.125);
      setMarkerColor(filtered_marker, 0.0, 0.0, 1.0, 1.0);
      marker_array.markers.emplace_back(filtered_marker);
    }

    const auto distance = estimateDistance(targets);
    if (output.control && distance > 0.0) {
      const Eigen::Vector3d selection{
        distance * std::cos(output.yaw_rad), distance * std::sin(output.yaw_rad),
        distance * std::sin(output.pitch_rad)};
      auto selection_marker =
        baseMarker(header, "selection", 0, visualization_msgs::msg::Marker::SPHERE);
      setMarkerPosition(selection_marker, selection);
      selection_marker.scale.x = 0.1;
      selection_marker.scale.y = 0.1;
      selection_marker.scale.z = 0.1;
      setMarkerColor(selection_marker, 1.0, 1.0, 0.0, 1.0);
      marker_array.markers.emplace_back(selection_marker);

      auto trajectory_marker =
        baseMarker(header, "trajectory", 0, visualization_msgs::msg::Marker::POINTS);
      trajectory_marker.scale.x = 0.01;
      trajectory_marker.scale.y = 0.01;
      setMarkerColor(
        trajectory_marker, output.shoot ? 0.0 : 1.0, 1.0, output.shoot ? 0.0 : 1.0, 1.0);
      for (int i = 1; i <= 24; ++i) {
        const double ratio = static_cast<double>(i) / 24.0;
        trajectory_marker.points.emplace_back(toPoint(selection * ratio));
      }
      marker_array.markers.emplace_back(trajectory_marker);
    }

    auto future = target;
    for (int step = 0; step < 10; ++step) {
      if (step > 0) future.predict(0.04);

      auto predicted_marker =
        baseMarker(header, "predicted_sequence", step, visualization_msgs::msg::Marker::POINTS);
      predicted_marker.scale.x = 0.05 + step * 0.001;
      predicted_marker.scale.y = 0.05 + step * 0.001;
      setPredictionColor(predicted_marker, step);

      for (const auto & xyza : future.armor_xyza_list()) {
        if (!finite4(xyza)) continue;
        const Eigen::Vector3d p{xyza[0], xyza[1], xyza[2]};
        if (finite3(p)) predicted_marker.points.emplace_back(toPoint(p));
      }
      marker_array.markers.emplace_back(predicted_marker);
    }

    return marker_array;
  }

  void publishDebugMarkers(
    const std_msgs::msg::Header & header, const std::list<auto_aim::Armor> & armors,
    const std::list<auto_aim::Target> & targets, const ControlOutput & output)
  {
    const auto detector_markers = buildDetectorMarkerArray(header, armors);
    const auto solver_markers = buildSolverMarkerArray(header, armors, targets, output);

    if (detector_marker_pub_) detector_marker_pub_->publish(detector_markers);
    if (solver_marker_pub_) solver_marker_pub_->publish(solver_markers);

    if (!debug_marker_pub_) return;

    visualization_msgs::msg::MarkerArray combined;
    combined.markers.push_back(clearMarker(header));
    for (const auto & marker : detector_markers.markers) {
      if (marker.action != visualization_msgs::msg::Marker::DELETEALL) {
        combined.markers.push_back(marker);
      }
    }
    for (const auto & marker : solver_markers.markers) {
      if (marker.action != visualization_msgs::msg::Marker::DELETEALL) {
        combined.markers.push_back(marker);
      }
    }
    debug_marker_pub_->publish(combined);
  }

  bool isFiniteOutput(const ControlOutput & output) const
  {
    return std::isfinite(output.yaw_rad) && std::isfinite(output.pitch_rad) &&
           std::isfinite(output.yaw_vel_rad) && std::isfinite(output.pitch_vel_rad) &&
           std::isfinite(output.yaw_acc_rad) && std::isfinite(output.pitch_acc_rad);
  }

  double estimateDistance(const std::list<auto_aim::Target> & targets) const
  {
    if (targets.empty()) return -1.0;
    const auto x = targets.front().ekf_x();
    if (x.size() < 3) return -1.0;
    return std::sqrt(x[0] * x[0] + x[2] * x[2]);
  }

  auto_aim::Color loadConfiguredEnemyColor(const std::string & config_path) const
  {
    const auto yaml = YAML::LoadFile(config_path);
    return yaml["enemy_color"].as<std::string>() == "blue" ? auto_aim::blue : auto_aim::red;
  }

  std::string loadYoloName(const std::string & config_path) const
  {
    const auto yaml = YAML::LoadFile(config_path);
    return yaml["yolo_name"].as<std::string>();
  }

  void updateEnemyColorFromMode(uint8_t mode)
  {
    if (!derive_enemy_color_from_mode_ || !tracker_) return;

    const auto color = enemyColorFromMode(mode);
    if (!color) return;

    std::lock_guard<std::mutex> lock(core_mutex_);
    if (*color == current_enemy_color_) return;

    current_enemy_color_ = *color;
    tracker_->set_enemy_color(*color);
    RCLCPP_INFO(
      get_logger(), "aim_v2 enemy_color switched to %s by mode %u", colorName(*color), mode);
  }

  std::string config_path_;
  std::string runtime_config_path_;
  std::string detector_type_;
  std::string control_backend_;
  bool debug_core_ = false;
  bool respect_mode_ = true;
  bool require_serial_ = true;
  bool derive_enemy_color_from_mode_ = true;
  bool async_inference_ = true;
  bool use_planner_ = false;
  bool enable_fire_ = false;
  bool debug_visualization_ = true;
  std::string debug_marker_frame_ = "odom";
  double default_bullet_speed_ = 23.0;
  std::vector<int64_t> auto_aim_modes_;
  int frame_count_ = 0;
  auto_aim::Color current_enemy_color_ = auto_aim::red;

  mutable std::mutex serial_mutex_;
  mutable std::mutex core_mutex_;
  std::mutex async_context_mutex_;
  SerialState serial_;
  std::queue<AsyncFrameContext> async_contexts_;

  std::unique_ptr<auto_aim::Solver> solver_;
  std::unique_ptr<auto_aim::Tracker> tracker_;
  std::unique_ptr<auto_aim::Aimer> aimer_;
  std::unique_ptr<auto_aim::Shooter> shooter_;
  std::unique_ptr<auto_aim::Planner> planner_;
  std::unique_ptr<auto_aim::YOLO> yolo_detector_;
  std::unique_ptr<auto_aim::Detector> traditional_detector_;
  std::unique_ptr<auto_aim::multithread::MultiThreadDetector> async_detector_;
  std::atomic_bool async_running_{false};
  std::thread async_worker_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detector_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr solver_marker_pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  fyt::HeartBeatPublisher::SharedPtr heartbeat_;
};

}  // namespace aim_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aim_v2::AimV2Node>());
  rclcpp::shutdown();
  return 0;
}
