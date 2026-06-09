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
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
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

    runtime_config_path_ = materializeRuntimeConfig(config_path_, share_dir, enemy_color);

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

      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        solver_->set_R_gimbal2world(
          makeGimbalQuaternion(serial.roll_deg, serial.pitch_deg, serial.yaw_deg));
        targets = tracker_->track(armors, timestamp);
        output = buildControlOutput(targets, timestamp, serial);
      }

      publishCommand(header, output, targets, serial);
      publishDebug(header, image, armors, targets, output);
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
      const Eigen::Vector3d ypr = tools::eulers(solver_->R_gimbal2world(), 2, 1, 0);
      command.shoot = shooter_->shoot(command, *aimer_, targets, ypr);
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
    publishDebugMarkers(header, armors, targets);
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
        if (!box.empty()) cv::rectangle(vis, box, color, 2);
      }

      if (armor.points.size() >= 4) {
        for (std::size_t i = 0; i < armor.points.size(); ++i) {
          cv::line(
            vis, cv::Point(armor.points[i]), cv::Point(armor.points[(i + 1) % armor.points.size()]),
            color, 2);
          cv::circle(vis, cv::Point(armor.points[i]), 3, color, -1);
        }
      }

      std::ostringstream label;
      label << armorName(armor.name) << " " << armorColorLabel(armor.color) << " "
            << static_cast<int>(std::round(armor.confidence * 100.0)) << "%";
      const auto text_origin = armor.box.empty()
        ? cv::Point(armor.center)
        : cv::Point(armor.box.x, std::max(12, armor.box.y - 6));
      cv::putText(
        vis, label.str(), text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    std::ostringstream status;
    status << "targets=" << targets.size() << " yaw=" << static_cast<int>(radToDeg(output.yaw_rad))
           << " pitch=" << static_cast<int>(radToDeg(output.pitch_rad))
           << " fire=" << (output.shoot ? "yes" : "no");
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

  void publishDebugMarkers(
    const std_msgs::msg::Header & header, const std::list<auto_aim::Armor> & armors,
    const std::list<auto_aim::Target> & targets)
  {
    if (!debug_marker_pub_) return;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.header.frame_id = debug_marker_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    int id = 1;
    for (const auto & armor : armors) {
      if (!finite3(armor.xyz_in_world)) continue;

      auto marker = baseMarker(header, "aim_v2/detected_armor", id++,
        visualization_msgs::msg::Marker::SPHERE);
      setMarkerPosition(marker, armor.xyz_in_world);
      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      if (armor.color == auto_aim::red) {
        setMarkerColor(marker, 1.0, 0.0, 0.0, 0.9);
      } else if (armor.color == auto_aim::blue) {
        setMarkerColor(marker, 0.0, 0.35, 1.0, 0.9);
      } else {
        setMarkerColor(marker, 0.7, 0.7, 0.7, 0.6);
      }
      marker_array.markers.push_back(marker);

      auto text = baseMarker(header, "aim_v2/detected_armor_text", id++,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      setMarkerPosition(text, armor.xyz_in_world + Eigen::Vector3d(0.0, 0.0, 0.08));
      text.scale.z = 0.08;
      setMarkerColor(text, 1.0, 1.0, 1.0, 1.0);
      text.text = armorName(armor.name);
      marker_array.markers.push_back(text);
    }

    for (const auto & target : targets) {
      const auto x = target.ekf_x();
      if (x.size() < 6) continue;
      const Eigen::Vector3d center{x[0], x[2], x[4]};
      const Eigen::Vector3d velocity{x[1], x[3], x[5]};
      if (!finite3(center)) continue;

      auto center_marker = baseMarker(header, "aim_v2/target_center", id++,
        visualization_msgs::msg::Marker::SPHERE);
      setMarkerPosition(center_marker, center);
      center_marker.scale.x = 0.12;
      center_marker.scale.y = 0.12;
      center_marker.scale.z = 0.12;
      setMarkerColor(center_marker, 0.0, 1.0, 0.0, 0.95);
      marker_array.markers.push_back(center_marker);

      auto velocity_marker = baseMarker(header, "aim_v2/target_velocity", id++,
        visualization_msgs::msg::Marker::ARROW);
      geometry_msgs::msg::Point start;
      start.x = center.x();
      start.y = center.y();
      start.z = center.z();
      const auto end_pos = center + velocity * 0.2;
      geometry_msgs::msg::Point end;
      end.x = end_pos.x();
      end.y = end_pos.y();
      end.z = end_pos.z();
      velocity_marker.points.push_back(start);
      velocity_marker.points.push_back(end);
      velocity_marker.scale.x = 0.02;
      velocity_marker.scale.y = 0.04;
      velocity_marker.scale.z = 0.0;
      setMarkerColor(velocity_marker, 0.0, 0.6, 1.0, 0.8);
      marker_array.markers.push_back(velocity_marker);

      for (const auto & xyza : target.armor_xyza_list()) {
        const Eigen::Vector3d armor_pos{xyza[0], xyza[1], xyza[2]};
        if (!finite3(armor_pos)) continue;
        auto armor_marker = baseMarker(header, "aim_v2/target_armors", id++,
          visualization_msgs::msg::Marker::CUBE);
        setMarkerPosition(armor_marker, armor_pos);
        armor_marker.pose.orientation.z = std::sin(xyza[3] * 0.5);
        armor_marker.pose.orientation.w = std::cos(xyza[3] * 0.5);
        armor_marker.scale.x = target.armor_type == auto_aim::big ? 0.23 : 0.13;
        armor_marker.scale.y = 0.04;
        armor_marker.scale.z = 0.06;
        setMarkerColor(armor_marker, 1.0, 0.8, 0.0, 0.45);
        marker_array.markers.push_back(armor_marker);
      }

      auto text = baseMarker(header, "aim_v2/target_text", id++,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      setMarkerPosition(text, center + Eigen::Vector3d(0.0, 0.0, 0.18));
      text.scale.z = 0.1;
      setMarkerColor(text, 1.0, 1.0, 1.0, 1.0);
      std::ostringstream label;
      label << armorName(target.name) << " v_yaw=" << std::fixed << std::setprecision(2)
            << (x.size() > 7 ? x[7] : 0.0);
      text.text = label.str();
      marker_array.markers.push_back(text);
    }

    debug_marker_pub_->publish(marker_array);
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
