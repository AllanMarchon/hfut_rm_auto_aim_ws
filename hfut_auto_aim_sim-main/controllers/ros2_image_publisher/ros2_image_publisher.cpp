#include <webots/Camera.hpp>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

double getEnvDouble(const char *name, double defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;
  return std::stod(value);
}

int getEnvInt(const char *name, int defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;
  return std::stoi(value);
}

std::string getEnvString(const char *name, const std::string &defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;
  return value;
}

bool getEnvBool(const char *name, bool defaultValue) {
  std::string value = getEnvString(name, defaultValue ? "true" : "false");
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

int getControlStepMs(int basicTimeStep, int defaultStep) {
  int controlStep = getEnvInt("WEBOTS_CONTROLLER_STEP_MS", defaultStep);
  if (controlStep < basicTimeStep) {
    throw std::runtime_error(
        "WEBOTS_CONTROLLER_STEP_MS must be >= basicTimeStep");
  }
  if (controlStep % basicTimeStep != 0) {
    throw std::runtime_error(
        "WEBOTS_CONTROLLER_STEP_MS must be a multiple of basicTimeStep");
  }
  return controlStep;
}

std::string getImageEncoding() {
  std::string encoding = getEnvString("WEBOTS_IMAGE_ENCODING", "bgr8");
  std::transform(encoding.begin(), encoding.end(), encoding.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (encoding != "bgr8" && encoding != "rgb8" && encoding != "bgra8") {
    throw std::runtime_error(
        "WEBOTS_IMAGE_ENCODING must be one of bgr8, rgb8, bgra8");
  }
  return encoding;
}

std::vector<double> getEnvDoubleList(const char *name,
                                     const std::vector<double> &defaultValue) {
  std::string value = getEnvString(name, "");
  if (value.empty())
    return defaultValue;

  for (char &ch : value) {
    if (ch == ',')
      ch = ' ';
  }

  std::vector<double> result;
  size_t start = 0;
  while (start < value.size()) {
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])))
      ++start;
    if (start >= value.size())
      break;
    size_t end = start;
    while (end < value.size() && !std::isspace(static_cast<unsigned char>(value[end])))
      ++end;
    result.push_back(std::stod(value.substr(start, end - start)));
    start = end;
  }
  return result;
}

std::vector<std::string> splitWords(const std::string &value) {
  std::istringstream stream(value);
  std::vector<std::string> words;
  std::string word;
  while (stream >> word) {
    words.push_back(word);
  }
  return words;
}

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double stepToward(double current, double target, double maxStep, bool wrap) {
  double error = wrap ? normalizeAngle(target - current) : target - current;
  if (maxStep > 0.0) {
    error = std::clamp(error, -maxStep, maxStep);
  }
  const double next = current + error;
  return wrap ? normalizeAngle(next) : next;
}

sensor_msgs::msg::CameraInfo buildCameraInfo(int width, int height, double fov,
                                             const std::string &frameId) {
  const double defaultFx = width / (2.0 * std::tan(fov / 2.0));
  const double fx = getEnvDouble("WEBOTS_CAMERA_FX", defaultFx);
  const double fy = getEnvDouble("WEBOTS_CAMERA_FY", fx);
  const double cx = getEnvDouble("WEBOTS_CAMERA_CX", (width - 1.0) / 2.0);
  const double cy = getEnvDouble("WEBOTS_CAMERA_CY", (height - 1.0) / 2.0);

  sensor_msgs::msg::CameraInfo msg;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.header.frame_id = frameId;
  msg.distortion_model = getEnvString("WEBOTS_CAMERA_DISTORTION_MODEL", "plumb_bob");
  msg.d = getEnvDoubleList("WEBOTS_CAMERA_D", {0.0, 0.0, 0.0, 0.0, 0.0});
  msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return msg;
}

void fillImageData(const unsigned char *bgra, int width, int height,
                   const std::string &encoding, sensor_msgs::msg::Image &image) {
  const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (encoding == "bgra8") {
    image.step = static_cast<uint32_t>(width * 4);
    image.data.resize(pixelCount * 4);
    std::memcpy(image.data.data(), bgra, image.data.size());
    return;
  }

  image.step = static_cast<uint32_t>(width * 3);
  image.data.resize(pixelCount * 3);
  unsigned char *dst = image.data.data();
  if (encoding == "bgr8") {
    for (size_t i = 0, j = 0; i < pixelCount * 4; i += 4, j += 3) {
      dst[j] = bgra[i];
      dst[j + 1] = bgra[i + 1];
      dst[j + 2] = bgra[i + 2];
    }
  } else {
    for (size_t i = 0, j = 0; i < pixelCount * 4; i += 4, j += 3) {
      dst[j] = bgra[i + 2];
      dst[j + 1] = bgra[i + 1];
      dst[j + 2] = bgra[i];
    }
  }
}

class ProfileTotals {
public:
  void add(const std::string &name, double seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    totals_[name] += seconds;
  }

  void maybeLog(const rclcpp::Logger &logger, int frameCount, int period) {
    if (period <= 0 || frameCount % period != 0)
      return;
    std::lock_guard<std::mutex> lock(mutex_);
    const char *names[] = {"step", "get_image", "copy", "convert", "publish", "spin"};
    std::string line = "profile avg ms:";
    for (const char *name : names) {
      line += " ";
      line += name;
      line += "=";
      line += std::to_string(totals_[name] * 1000.0 / period);
    }
    RCLCPP_INFO(logger, "%s", line.c_str());
    totals_.clear();
  }

private:
  std::mutex mutex_;
  std::unordered_map<std::string, double> totals_;
};

struct PublishFrame {
  rclcpp::Time stamp;
  bool publishImage{false};
  bool publishCameraInfo{false};
  std::vector<unsigned char> bgra;
};

struct SetModeClientState {
  explicit SetModeClientState(
      const std::string &name,
      const rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr &clientPtr)
      : serviceName(name), client(clientPtr) {}

  std::string serviceName;
  rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr client;
  std::atomic<int> mode{-1};
  std::atomic<bool> waiting{false};
};

struct DelayedGimbalAxisCommand {
  std::chrono::steady_clock::time_point executeAt;
  double value{0.0};
  bool relative{false};
};

std::chrono::steady_clock::duration delayMsToDuration(double delayMs) {
  const double boundedDelayMs = std::max(0.0, delayMs);
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(boundedDelayMs));
}

bool consumeDueAxisCommands(
    std::deque<DelayedGimbalAxisCommand> &commands,
    const std::chrono::steady_clock::time_point &now,
    DelayedGimbalAxisCommand &command) {
  bool hasCommand = false;
  while (!commands.empty() && commands.front().executeAt <= now) {
    const auto nextCommand = commands.front();
    commands.pop_front();
    if (nextCommand.relative && hasCommand && command.relative) {
      command.value += nextCommand.value;
      command.executeAt = nextCommand.executeAt;
    } else {
      command = nextCommand;
    }
    hasCommand = true;
  }
  return hasCommand;
}

double elapsedSeconds(const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

template <typename PublisherT>
bool hasSubscribers(const std::shared_ptr<PublisherT> &publisher) {
  return publisher->get_subscription_count() > 0 ||
         publisher->get_intra_process_subscription_count() > 0;
}

geometry_msgs::msg::TransformStamped makeTransform(
    const rclcpp::Time &stamp, const std::string &parentFrame,
    const std::string &childFrame, double x, double y, double z,
    double roll, double pitch, double yaw) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parentFrame;
  transform.child_frame_id = childFrame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  return transform;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    auto robot = std::make_unique<webots::Supervisor>();
    const int basicTimeStep = static_cast<int>(robot->getBasicTimeStep());

    const std::string cameraName = getEnvString("WEBOTS_CAMERA_NAME", "camera");
    const std::string imageTopic = getEnvString("WEBOTS_IMAGE_TOPIC", "/image_raw");
    const std::string cameraInfoTopic = getEnvString("WEBOTS_CAMERA_INFO_TOPIC", "/camera_info");
    const std::string gimbalCmdTopic =
        getEnvString("WEBOTS_GIMBAL_CMD_TOPIC", "/armor_solver/cmd_gimbal");
    const std::string jointStatesTopic =
        getEnvString("WEBOTS_JOINT_STATES_TOPIC", "/joint_states");
    const std::string targetFrameId = getEnvString("WEBOTS_TARGET_FRAME_ID", "odom");
    const std::string baseFrameId = getEnvString("WEBOTS_BASE_FRAME_ID", "base_link");
    const std::string gimbalFrameId = getEnvString("WEBOTS_GIMBAL_FRAME_ID", "gimbal_link");
    const std::string cameraLinkFrameId =
        getEnvString("WEBOTS_CAMERA_LINK_FRAME_ID", "camera_link");
    const std::string cameraOpticalLinkFrameId =
        getEnvString("WEBOTS_CAMERA_OPTICAL_LINK_FRAME_ID", "camera_optical_link");
    const std::string odomRectifyFrameId =
        getEnvString("WEBOTS_ODOM_RECTIFY_FRAME_ID", targetFrameId + "_rectify");
    const std::string frameId = getEnvString("WEBOTS_CAMERA_FRAME_ID", "camera_optical_frame");
    const std::string imageEncoding = getImageEncoding();
    const bool publishImages = !getEnvBool("WEBOTS_DISABLE_IMAGE_PUBLISH", false);
    const bool publishJointStates =
        getEnvBool("WEBOTS_PUBLISH_JOINT_STATES", true);
    const bool publishTf = getEnvBool("WEBOTS_PUBLISH_TF", true);
    const bool publishOdomRectifyTf =
        getEnvBool("WEBOTS_PUBLISH_ODOM_RECTIFY_TF", true);
    const bool publishCameraFrameAliasTf =
        getEnvBool("WEBOTS_PUBLISH_CAMERA_FRAME_ALIAS_TF", true);
    const bool followCmdGimbal =
        getEnvBool("WEBOTS_CAMERA_FOLLOW_CMD_GIMBAL", true);
    const bool useDiffCommands =
        getEnvBool("WEBOTS_GIMBAL_USE_DIFF_COMMANDS", false);
    const bool acceptUnknownMode =
        getEnvBool("WEBOTS_GIMBAL_ACCEPT_UNKNOWN_MODE", true);
    const double maxYawRate = getEnvDouble("WEBOTS_GIMBAL_MAX_YAW_RATE", 20.0);
    const double maxPitchRate = getEnvDouble("WEBOTS_GIMBAL_MAX_PITCH_RATE", 20.0);
    const double yawResponseDelayMs =
        std::max(0.0, getEnvDouble("WEBOTS_GIMBAL_YAW_RESPONSE_DELAY_MS", 0.0));
    const double pitchResponseDelayMs =
        std::max(0.0, getEnvDouble("WEBOTS_GIMBAL_PITCH_RESPONSE_DELAY_MS", 0.0));
    const auto yawResponseDelay = delayMsToDuration(yawResponseDelayMs);
    const auto pitchResponseDelay = delayMsToDuration(pitchResponseDelayMs);
    double webotsYawMotionSign =
        getEnvDouble("WEBOTS_GIMBAL_WEBOTS_YAW_SIGN", 1.0);
    double webotsPitchMotionSign =
        getEnvDouble("WEBOTS_GIMBAL_WEBOTS_PITCH_SIGN", -1.0);
    if (std::abs(webotsYawMotionSign) < 1e-9) {
      webotsYawMotionSign = 1.0;
    }
    if (std::abs(webotsPitchMotionSign) < 1e-9) {
      webotsPitchMotionSign = -1.0;
    }
    const bool enableSetModeClients =
        getEnvBool("WEBOTS_ENABLE_SET_MODE_CLIENTS", true);
    const int serialMode = getEnvInt("WEBOTS_SERIAL_MODE", 0);
    const int setModePeriodMs =
        std::max(50, getEnvInt("WEBOTS_SET_MODE_PERIOD_MS", 500));
    const std::vector<std::string> setModeServices = splitWords(
        getEnvString("WEBOTS_SET_MODE_SERVICES",
                     "armor_detector/set_mode gimbal_pipeline/set_mode"));
    const bool readCamera = !getEnvBool("WEBOTS_DISABLE_CAMERA_READ", false);
    const bool skipUnsubscribedImages =
        getEnvBool("WEBOTS_SKIP_UNSUBSCRIBED_IMAGES", true);
    const int profilePeriod = getEnvInt("WEBOTS_PROFILE_PERIOD", 0);
    const int spinPeriod = std::max(1, getEnvInt("WEBOTS_SPIN_PERIOD", 8));

    auto *camera = robot->getCamera(cameraName);
    if (camera == nullptr)
      throw std::runtime_error("Webots camera device not found: " + cameraName);

    const int cameraPeriodMs = getEnvInt("WEBOTS_CAMERA_PERIOD_MS", basicTimeStep);
    const int timestep = getControlStepMs(basicTimeStep, cameraPeriodMs);
    const int width = camera->getWidth();
    const int height = camera->getHeight();
    auto *cameraRobotNode = robot->getSelf();
    auto *cameraYawField = cameraRobotNode ? cameraRobotNode->getField("rotation") : nullptr;
    auto *cameraPitchNode = robot->getFromDef("SIM_CAMERA_PITCH");
    auto *cameraPitchField = cameraPitchNode ? cameraPitchNode->getField("rotation") : nullptr;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("webots_camera_publisher");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto imagePub = node->create_publisher<sensor_msgs::msg::Image>(imageTopic, qos);
    auto cameraInfoPub =
        node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoTopic, qos);
    auto jointStatePub =
        node->create_publisher<sensor_msgs::msg::JointState>(jointStatesTopic,
                                                             rclcpp::SensorDataQoS());
    auto tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
    auto staticTfBroadcaster =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node);
    std::mutex gimbalCommandMutex;
    std::deque<DelayedGimbalAxisCommand> pendingYawCommands;
    std::deque<DelayedGimbalAxisCommand> pendingPitchCommands;
    auto gimbalCmdSub = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
        gimbalCmdTopic, rclcpp::SensorDataQoS(),
        [&](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
          if (!followCmdGimbal)
            return;
          const bool normalMode =
              msg->mode == rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT;
          const bool unknownMode =
              msg->mode == rm_interfaces::msg::GimbalCmd::MODE_UNKNOWN;
          if (!normalMode && !(acceptUnknownMode && unknownMode))
            return;

          const auto now = std::chrono::steady_clock::now();
          DelayedGimbalAxisCommand yawCommand;
          yawCommand.executeAt = now + yawResponseDelay;
          yawCommand.value = (useDiffCommands ? msg->yaw_diff : msg->yaw) * kDegToRad;
          yawCommand.relative = useDiffCommands;

          DelayedGimbalAxisCommand pitchCommand;
          pitchCommand.executeAt = now + pitchResponseDelay;
          pitchCommand.value = (useDiffCommands ? msg->pitch_diff : msg->pitch) * kDegToRad;
          pitchCommand.relative = useDiffCommands;

          std::lock_guard<std::mutex> lock(gimbalCommandMutex);
          pendingYawCommands.push_back(yawCommand);
          pendingPitchCommands.push_back(pitchCommand);
        });
    std::vector<std::shared_ptr<SetModeClientState>> setModeClients;
    if (enableSetModeClients) {
      setModeClients.reserve(setModeServices.size());
      for (const auto &serviceName : setModeServices) {
        auto client = node->create_client<rm_interfaces::srv::SetMode>(serviceName);
        setModeClients.push_back(
            std::make_shared<SetModeClientState>(serviceName, client));
      }
    }
    auto cameraInfo = buildCameraInfo(width, height, camera->getFov(), frameId);
    sensor_msgs::msg::Image image;
    image.header.frame_id = frameId;
    image.height = static_cast<uint32_t>(height);
    image.width = static_cast<uint32_t>(width);
    image.encoding = imageEncoding;
    image.is_bigendian = 0;
    image.data.resize(static_cast<size_t>(width) * static_cast<size_t>(height) *
                      (imageEncoding == "bgra8" ? 4 : 3));
    const double initialWebotsYaw = getEnvDouble("WEBOTS_CAMERA_YAW", 0.0);
    const double initialWebotsPitch = getEnvDouble("WEBOTS_CAMERA_TILT", 0.0);
    sensor_msgs::msg::JointState jointState;
    jointState.name = {"yaw_joint", "pitch_joint"};
    jointState.position = {initialWebotsYaw / webotsYawMotionSign,
                           initialWebotsPitch / webotsPitchMotionSign};
    jointState.velocity = {0.0, 0.0};
    double desiredYaw = jointState.position[0];
    double desiredPitch = jointState.position[1];
    const double baseTfX = getEnvDouble("WEBOTS_BASE_TF_X", 0.0);
    const double baseTfY = getEnvDouble("WEBOTS_BASE_TF_Y", 0.0);
    const double baseTfZ = getEnvDouble("WEBOTS_BASE_TF_Z", 0.0);
    const double baseTfRoll = getEnvDouble("WEBOTS_BASE_TF_ROLL", 0.0);
    const double baseTfPitch = getEnvDouble("WEBOTS_BASE_TF_PITCH", 0.0);
    const double baseTfYaw = getEnvDouble("WEBOTS_BASE_TF_YAW", 0.0);
    const double gimbalRoll = getEnvDouble("WEBOTS_GIMBAL_ROLL", 0.0);
    const double gimbalTfX = getEnvDouble("WEBOTS_GIMBAL_TF_X", 0.0);
    const double gimbalTfY = getEnvDouble("WEBOTS_GIMBAL_TF_Y", 0.0);
    const double gimbalTfZ = getEnvDouble("WEBOTS_GIMBAL_TF_Z", 0.0);
    const double cameraTfX = getEnvDouble("WEBOTS_CAMERA_TF_X", 0.12514);
    const double cameraTfY = getEnvDouble("WEBOTS_CAMERA_TF_Y", 0.0);
    const double cameraTfZ = getEnvDouble("WEBOTS_CAMERA_TF_Z", 0.0505);
    const double cameraTfRoll = getEnvDouble("WEBOTS_CAMERA_TF_ROLL", 0.0);
    const double cameraTfPitch = getEnvDouble("WEBOTS_CAMERA_TF_PITCH", 0.0);
    const double cameraTfYaw = getEnvDouble("WEBOTS_CAMERA_TF_YAW", 0.0);
    const double cameraOpticalTfRoll =
        getEnvDouble("WEBOTS_CAMERA_OPTICAL_TF_ROLL", -kPi / 2.0);
    const double cameraOpticalTfPitch =
        getEnvDouble("WEBOTS_CAMERA_OPTICAL_TF_PITCH", 0.0);
    const double cameraOpticalTfYaw =
        getEnvDouble("WEBOTS_CAMERA_OPTICAL_TF_YAW", -kPi / 2.0);
    auto publishStaticTransforms = [&]() {
      if (!publishTf)
        return;

      const auto stamp = node->now();
      std::vector<geometry_msgs::msg::TransformStamped> staticTransforms;
      staticTransforms.push_back(makeTransform(
          stamp, targetFrameId, baseFrameId, baseTfX, baseTfY, baseTfZ,
          baseTfRoll, baseTfPitch, baseTfYaw));
      staticTransforms.push_back(makeTransform(
          stamp, gimbalFrameId, cameraLinkFrameId, cameraTfX, cameraTfY,
          cameraTfZ, cameraTfRoll, cameraTfPitch, cameraTfYaw));
      staticTransforms.push_back(makeTransform(
          stamp, cameraLinkFrameId, cameraOpticalLinkFrameId, 0.0, 0.0, 0.0,
          cameraOpticalTfRoll, cameraOpticalTfPitch, cameraOpticalTfYaw));

      const bool needsCameraFrameAlias =
          publishCameraFrameAliasTf && frameId != cameraOpticalLinkFrameId &&
          frameId != cameraLinkFrameId && frameId != gimbalFrameId &&
          frameId != baseFrameId && frameId != targetFrameId &&
          frameId != odomRectifyFrameId;
      if (needsCameraFrameAlias) {
        staticTransforms.push_back(makeTransform(
            stamp, cameraOpticalLinkFrameId, frameId, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0));
      }

      if (publishOdomRectifyTf) {
        staticTransforms.push_back(makeTransform(
            stamp, targetFrameId, odomRectifyFrameId, 0.0, 0.0, 0.0,
            gimbalRoll, 0.0, 0.0));
      }

      staticTfBroadcaster->sendTransform(staticTransforms);
    };
    auto publishTransforms = [&](const rclcpp::Time &stamp) {
      if (!publishTf)
        return;

      tfBroadcaster->sendTransform(makeTransform(
          stamp, baseFrameId, gimbalFrameId, gimbalTfX, gimbalTfY, gimbalTfZ,
          gimbalRoll, -jointState.position[1], jointState.position[0]));
    };
    auto applyWebotsCameraPose = [&]() {
      if (!followCmdGimbal)
        return;
      if (cameraYawField != nullptr) {
        const double yawRotation[] = {
            0.0, 0.0, 1.0, webotsYawMotionSign * jointState.position[0]};
        cameraYawField->setSFRotation(yawRotation);
      }
      if (cameraPitchField != nullptr) {
        const double pitchRotation[] = {
            0.0, 1.0, 0.0, webotsPitchMotionSign * jointState.position[1]};
        cameraPitchField->setSFRotation(pitchRotation);
      }
    };
    auto requestSetMode = [&](const std::shared_ptr<SetModeClientState> &clientState) {
      if (clientState->mode.load(std::memory_order_relaxed) == serialMode ||
          clientState->waiting.exchange(true)) {
        return;
      }

      if (!clientState->client->service_is_ready()) {
        clientState->waiting.store(false, std::memory_order_relaxed);
        return;
      }

      auto request = std::make_shared<rm_interfaces::srv::SetMode::Request>();
      request->mode = static_cast<uint8_t>(serialMode);
      clientState->client->async_send_request(
          request,
          [clientState, serialMode](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
            clientState->waiting.store(false, std::memory_order_relaxed);
            try {
              if (result.get()->success) {
                clientState->mode.store(serialMode, std::memory_order_relaxed);
              }
            } catch (...) {
            }
          });
    };
    auto lastSetModeAttempt =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(setModePeriodMs);
    auto maybeRequestSetMode = [&]() {
      if (!enableSetModeClients || setModeClients.empty())
        return;
      const auto now = std::chrono::steady_clock::now();
      if (now - lastSetModeAttempt < std::chrono::milliseconds(setModePeriodMs))
        return;
      lastSetModeAttempt = now;
      for (const auto &clientState : setModeClients) {
        requestSetMode(clientState);
      }
    };
    auto updateGimbalState = [&](double dt) {
      if (!followCmdGimbal) {
        jointState.velocity = {0.0, 0.0};
        return;
      }

      DelayedGimbalAxisCommand yawCommand;
      DelayedGimbalAxisCommand pitchCommand;
      bool hasYawCommand = false;
      bool hasPitchCommand = false;
      {
        std::lock_guard<std::mutex> lock(gimbalCommandMutex);
        const auto now = std::chrono::steady_clock::now();
        hasYawCommand = consumeDueAxisCommands(pendingYawCommands, now, yawCommand);
        hasPitchCommand = consumeDueAxisCommands(pendingPitchCommands, now, pitchCommand);
      }

      if (hasYawCommand) {
        if (yawCommand.relative) {
          desiredYaw = normalizeAngle(desiredYaw + yawCommand.value);
        } else {
          desiredYaw = normalizeAngle(yawCommand.value);
        }
      }
      if (hasPitchCommand) {
        if (pitchCommand.relative) {
          desiredPitch = desiredPitch + pitchCommand.value;
        } else {
          desiredPitch = pitchCommand.value;
        }
      }

      const double previousYaw = jointState.position[0];
      const double previousPitch = jointState.position[1];
      const double maxYawStep = maxYawRate * dt;
      const double maxPitchStep = maxPitchRate * dt;
      jointState.position[0] = stepToward(previousYaw, desiredYaw, maxYawStep, true);
      jointState.position[1] = stepToward(previousPitch, desiredPitch, maxPitchStep, false);
      if (dt > 0.0) {
        jointState.velocity = {
            normalizeAngle(jointState.position[0] - previousYaw) / dt,
            (jointState.position[1] - previousPitch) / dt};
      } else {
        jointState.velocity = {0.0, 0.0};
      }
    };
    bool cameraEnabled = false;
    auto setCameraEnabled = [&](bool enabled) {
      if (enabled == cameraEnabled)
        return;
      if (enabled) {
        camera->enable(cameraPeriodMs);
      } else {
        camera->disable();
      }
      cameraEnabled = enabled;
    };
    setCameraEnabled(readCamera && !skipUnsubscribedImages);

    std::mutex frameMutex;
    std::condition_variable frameCv;
    PublishFrame pendingFrame;
    bool hasPendingFrame = false;
    std::atomic<bool> stopPublisher{false};
    std::atomic<bool> imageHasSubscribers{false};
    std::atomic<bool> cameraInfoHasSubscribers{false};

    const std::string tfTree = publishTf
        ? targetFrameId + "->" + baseFrameId + "->" + gimbalFrameId + "->" +
              cameraLinkFrameId + "->" + cameraOpticalLinkFrameId
        : "disabled";
    RCLCPP_INFO(node->get_logger(),
                "Publishing Webots camera '%s' to %s as %s with decoupled ROS "
                "publisher thread; read_camera=%s, publish_images=%s, "
                "skip_unsubscribed_images=%s, joint_states=%s, tf_tree=%s, "
                "image_frame=%s, cmd_gimbal=%s(%s), set_mode=%s(mode=%d), "
                "gimbal_delay_ms=(yaw:%.3f,pitch:%.3f), spin_period=%d",
                cameraName.c_str(), imageTopic.c_str(), imageEncoding.c_str(),
                readCamera ? "true" : "false", publishImages ? "true" : "false",
                skipUnsubscribedImages ? "true" : "false",
                publishJointStates ? jointStatesTopic.c_str() : "disabled",
                tfTree.c_str(),
                frameId.c_str(),
                followCmdGimbal ? gimbalCmdTopic.c_str() : "disabled",
                useDiffCommands ? "diff" : "absolute",
                enableSetModeClients ? "enabled" : "disabled",
                serialMode,
                yawResponseDelayMs,
                pitchResponseDelayMs,
                spinPeriod);
    publishStaticTransforms();

    ProfileTotals profile;
    int frameCount = 0;
    auto finishFrame = [&]() {
      ++frameCount;
      profile.maybeLog(node->get_logger(), frameCount, profilePeriod);
    };

    auto publishThread = std::thread([&]() {
      sensor_msgs::msg::Image publishImageMsg = image;
      int publishLoopCount = 0;

      while (rclcpp::ok()) {
        PublishFrame frame;
        bool hasFrame = false;
        {
          std::unique_lock<std::mutex> lock(frameMutex);
          frameCv.wait_for(lock, std::chrono::milliseconds(2), [&]() {
            return stopPublisher.load() || hasPendingFrame;
          });
          if (hasPendingFrame) {
            frame = std::move(pendingFrame);
            hasPendingFrame = false;
            hasFrame = true;
          }
        }

        if (publishLoopCount % spinPeriod == 0) {
          auto spinStart = std::chrono::steady_clock::now();
          rclcpp::spin_some(node);
          profile.add("spin", elapsedSeconds(spinStart));
        }
        ++publishLoopCount;

        imageHasSubscribers.store(hasSubscribers(imagePub), std::memory_order_relaxed);
        cameraInfoHasSubscribers.store(hasSubscribers(cameraInfoPub),
                                       std::memory_order_relaxed);

        if (!hasFrame) {
          if (stopPublisher.load())
            break;
          continue;
        }

        if (!publishImages)
          continue;

        auto publishStart = std::chrono::steady_clock::now();
        if (frame.publishImage) {
          publishImageMsg.header.stamp = frame.stamp;
          auto convertStart = std::chrono::steady_clock::now();
          fillImageData(frame.bgra.data(), width, height, imageEncoding, publishImageMsg);
          profile.add("convert", elapsedSeconds(convertStart));
          imagePub->publish(publishImageMsg);
        }
        if (frame.publishCameraInfo) {
          cameraInfo.header.stamp = frame.stamp;
          cameraInfoPub->publish(cameraInfo);
        }
        profile.add("publish", elapsedSeconds(publishStart));

        if (stopPublisher.load()) {
          std::lock_guard<std::mutex> lock(frameMutex);
          if (!hasPendingFrame)
            break;
        }
      }
    });

    while (rclcpp::ok()) {
      auto stepStart = std::chrono::steady_clock::now();
      if (robot->step(timestep) == -1)
        break;
      profile.add("step", elapsedSeconds(stepStart));

      const rclcpp::Time stamp = node->get_clock()->now();
      updateGimbalState(static_cast<double>(timestep) / 1000.0);
      applyWebotsCameraPose();
      if (publishJointStates) {
        jointState.header.stamp = stamp;
        jointStatePub->publish(jointState);
      }
      publishTransforms(stamp);
      maybeRequestSetMode();

      if (!readCamera) {
        finishFrame();
        continue;
      }

      const bool hasImageSubscribers =
          imageHasSubscribers.load(std::memory_order_relaxed);
      const bool hasCameraInfoSubscribers =
          cameraInfoHasSubscribers.load(std::memory_order_relaxed);
      const bool shouldReadImage = !skipUnsubscribedImages || hasImageSubscribers;
      setCameraEnabled(shouldReadImage);

      if (!shouldReadImage) {
        if (publishImages && hasCameraInfoSubscribers) {
          PublishFrame frame;
          frame.stamp = stamp;
          frame.publishCameraInfo = true;
          {
            std::lock_guard<std::mutex> lock(frameMutex);
            pendingFrame = std::move(frame);
            hasPendingFrame = true;
          }
          frameCv.notify_one();
        }
        finishFrame();
        continue;
      }

      auto getImageStart = std::chrono::steady_clock::now();
      const unsigned char *cameraImage = camera->getImage();
      profile.add("get_image", elapsedSeconds(getImageStart));
      if (cameraImage == nullptr) {
        finishFrame();
        continue;
      }

      if (publishImages) {
        PublishFrame frame;
        frame.stamp = stamp;
        frame.publishImage = !skipUnsubscribedImages || hasImageSubscribers;
        frame.publishCameraInfo =
            !skipUnsubscribedImages || hasImageSubscribers || hasCameraInfoSubscribers;

        if (frame.publishImage) {
          auto copyStart = std::chrono::steady_clock::now();
          const size_t bgraSize =
              static_cast<size_t>(width) * static_cast<size_t>(height) * 4;
          frame.bgra.resize(bgraSize);
          std::memcpy(frame.bgra.data(), cameraImage, bgraSize);
          profile.add("copy", elapsedSeconds(copyStart));
        }

        if (frame.publishImage || frame.publishCameraInfo) {
          {
            std::lock_guard<std::mutex> lock(frameMutex);
            pendingFrame = std::move(frame);
            hasPendingFrame = true;
          }
          frameCv.notify_one();
        }
      }

      finishFrame();
    }

    stopPublisher.store(true);
    frameCv.notify_one();
    if (publishThread.joinable())
      publishThread.join();

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "ros2_image_publisher controller error: " << error.what() << std::endl;
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
