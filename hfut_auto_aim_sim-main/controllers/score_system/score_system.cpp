#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kArmorFacingMaxAngleRad = 75.0 * kDegToRad;
constexpr double kDamagePerHit = 20.0;

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
  if (value == "1" || value == "true" || value == "yes" || value == "on")
    return true;
  if (value == "0" || value == "false" || value == "no" || value == "off")
    return false;
  throw std::runtime_error(std::string(name) + " must be a boolean value");
}

int getControlStepMs(int basicTimeStep) {
  const int controlStep =
      getEnvInt("WEBOTS_SCORE_CONTROLLER_STEP_MS", basicTimeStep);
  if (controlStep < basicTimeStep) {
    throw std::runtime_error(
        "WEBOTS_SCORE_CONTROLLER_STEP_MS must be >= basicTimeStep");
  }
  if (controlStep % basicTimeStep != 0) {
    throw std::runtime_error(
        "WEBOTS_SCORE_CONTROLLER_STEP_MS must be a multiple of basicTimeStep");
  }
  return controlStep;
}

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vec3 operator+(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 &a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}

double dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3 &v) {
  return std::sqrt(dot(v, v));
}

Vec3 normalized(const Vec3 &v) {
  const double n = norm(v);
  if (n <= 1e-12)
    return {};
  return v * (1.0 / n);
}

Vec3 matrixColumn(const double *orientation, int column) {
  return {
      orientation[column],
      orientation[3 + column],
      orientation[6 + column]};
}

Vec3 matrixVectorProduct(const double *orientation, const Vec3 &local) {
  return {
      orientation[0] * local.x + orientation[1] * local.y + orientation[2] * local.z,
      orientation[3] * local.x + orientation[4] * local.y + orientation[5] * local.z,
      orientation[6] * local.x + orientation[7] * local.y + orientation[8] * local.z};
}

Vec3 nodePosition(webots::Node *node) {
  const double *position = node->getPosition();
  if (position == nullptr)
    throw std::runtime_error("cannot read node position");
  return {position[0], position[1], position[2]};
}

Vec3 nodeLocalDirection(webots::Node *node, const Vec3 &localDirection) {
  const double *orientation = node->getOrientation();
  if (orientation == nullptr)
    throw std::runtime_error("cannot read node orientation");
  return normalized(matrixVectorProduct(orientation, localDirection));
}

struct ArmorPlane {
  std::string name;
  webots::Node *node{nullptr};
  Vec3 center;
  Vec3 normal;
  Vec3 uAxis;
  Vec3 vAxis;
  double halfWidth{0.0};
  double halfHeight{0.0};
};

ArmorPlane readArmorPlane(webots::Node *node, const std::string &name,
                          double halfWidth, double halfHeight,
                          double armorPitch) {
  if (node == nullptr)
    throw std::runtime_error("missing Webots armor node: " + name);
  const double *orientation = node->getOrientation();
  if (orientation == nullptr)
    throw std::runtime_error("cannot read armor orientation: " + name);

  ArmorPlane armor;
  armor.name = name;
  armor.node = node;
  armor.center = nodePosition(node);
  armor.uAxis = normalized(matrixColumn(orientation, 0));
  armor.normal = normalized(matrixVectorProduct(
      orientation, {0.0, std::cos(armorPitch), std::sin(armorPitch)}));
  armor.vAxis = normalized(matrixVectorProduct(
      orientation, {0.0, -std::sin(armorPitch), std::cos(armorPitch)}));
  armor.halfWidth = halfWidth;
  armor.halfHeight = halfHeight;
  return armor;
}

struct GimbalSnapshot {
  bool fireAdvice{false};
  bool valid{false};
};

struct PendingShot {
  double fireTime{0.0};
};

struct ActiveProjectile {
  uint64_t shotId{0};
  double launchTime{0.0};
  Vec3 origin;
  Vec3 velocity;
  Vec3 previousPosition;
};

struct ScoreStats {
  uint64_t shots{0};
  uint64_t hits{0};
  uint64_t misses{0};
  double startTime{0.0};

  double elapsed(double now) const {
    return std::max(0.0, now - startTime);
  }

  double hitRate() const {
    const uint64_t resolvedShots = hits + misses;
    return resolvedShots > 0
               ? static_cast<double>(hits) / static_cast<double>(resolvedShots)
               : 0.0;
  }

  double dps(double now) const {
    const double dt = elapsed(now);
    return dt > 0.0 ? static_cast<double>(hits) * kDamagePerHit / dt : 0.0;
  }
};

Vec3 projectilePosition(const Vec3 &origin, const Vec3 &velocity,
                        const Vec3 &gravity, double t) {
  return origin + velocity * t + gravity * (0.5 * t * t);
}

bool armorFacesCamera(const ArmorPlane &armor, const Vec3 &cameraPosition) {
  const Vec3 toCamera = normalized(cameraPosition - armor.center);
  const double cosAngle = dot(armor.normal, toCamera);
  return cosAngle >= std::cos(kArmorFacingMaxAngleRad);
}

bool segmentIntersectsArmor(const Vec3 &prev, const Vec3 &curr,
                            const ArmorPlane &armor, double &fraction) {
  const double d0 = dot(prev - armor.center, armor.normal);
  const double d1 = dot(curr - armor.center, armor.normal);
  if (d0 <= 0.0 || d1 > 0.0)
    return false;

  const double denom = d0 - d1;
  if (std::abs(denom) <= 1e-12)
    return false;

  fraction = d0 / denom;
  if (fraction < 0.0 || fraction > 1.0)
    return false;

  const Vec3 point = prev + (curr - prev) * fraction;
  const Vec3 rel = point - armor.center;
  const double u = dot(rel, armor.uAxis);
  const double v = dot(rel, armor.vAxis);
  return std::abs(u) <= armor.halfWidth && std::abs(v) <= armor.halfHeight;
}

ArmorPlane orientArmorTowardCamera(ArmorPlane armor, const Vec3 &cameraPosition) {
  if (dot(armor.normal, cameraPosition - armor.center) < 0.0) {
    armor.normal = armor.normal * -1.0;
    armor.vAxis = armor.vAxis * -1.0;
  }
  return armor;
}

std::string makeScoreMessage(const ScoreStats &stats, double now, bool fired,
                             bool fireAdvice, double cooldownRemaining) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "shots=" << stats.shots
         << " hits=" << stats.hits
         << " misses=" << stats.misses
         << " hit_rate=" << stats.hitRate()
         << " dps=" << stats.dps(now)
         << " elapsed=" << stats.elapsed(now)
         << " fire_advice=" << (fireAdvice ? 1 : 0)
         << " fired=" << (fired ? 1 : 0)
         << " cooldown_remaining=" << cooldownRemaining;
  return stream.str();
}

}  // namespace

int main(int argc, char **argv) {
  try {
    webots::Supervisor robot;
    const int basicTimeStep = static_cast<int>(robot.getBasicTimeStep());
    const int timestep = getControlStepMs(basicTimeStep);
    const bool scoreEnabled = getEnvBool("WEBOTS_SCORE_ENABLED", true);
    const std::string gimbalCmdTopic =
        getEnvString("WEBOTS_GIMBAL_CMD_TOPIC", "/armor_solver/cmd_gimbal");
    const std::string scoreTopic =
        getEnvString("WEBOTS_SCORE_TOPIC", "/webots/score");
    const double fireDelay =
        std::max(0.0, getEnvDouble("WEBOTS_FIRE_DELAY_MS", 0.0) / 1000.0);
    const double fireRateHz = std::max(0.0, getEnvDouble("WEBOTS_FIRE_RATE_HZ", 0.0));
    const double fireInterval =
        fireRateHz > 0.0 ? 1.0 / fireRateHz : 0.0;
    const double bulletSpeed =
        std::max(0.0, getEnvDouble("WEBOTS_BULLET_SPEED", 25.0));
    const double gravityMagnitude =
        std::max(0.0, getEnvDouble("WEBOTS_SCORE_GRAVITY", 9.80665));
    const double maxFlightTime =
        std::max(0.01, getEnvDouble("WEBOTS_SCORE_MAX_FLIGHT_TIME", 2.0));
    const int publishPeriodMs =
        std::max(1, getEnvInt("WEBOTS_SCORE_PUBLISH_PERIOD_MS", 200));
    const double armorWidth =
        std::max(0.001, getEnvDouble("WEBOTS_SCORE_ARMOR_WIDTH", 0.135));
    const double armorHeight =
        std::max(0.001, getEnvDouble("WEBOTS_SCORE_ARMOR_HEIGHT", 0.135));
    const double armorPitch = getEnvDouble("WEBOTS_ARMOR_PITCH", 0.0);
    const double shooterOffsetX = getEnvDouble("WEBOTS_SCORE_SHOOTER_OFFSET_X", 0.0);
    const double shooterOffsetY = getEnvDouble("WEBOTS_SCORE_SHOOTER_OFFSET_Y", 0.0);
    const double shooterOffsetZ = getEnvDouble("WEBOTS_SCORE_SHOOTER_OFFSET_Z", 0.0);
    const Vec3 shooterOffset{shooterOffsetX, shooterOffsetY, shooterOffsetZ};
    const Vec3 shooterLocalForward{
        getEnvDouble("WEBOTS_SCORE_SHOOTER_FORWARD_X", 1.0),
        getEnvDouble("WEBOTS_SCORE_SHOOTER_FORWARD_Y", 0.0),
        getEnvDouble("WEBOTS_SCORE_SHOOTER_FORWARD_Z", 0.0)};

    webots::Node *cameraPitchNode = robot.getFromDef("SIM_CAMERA_PITCH");
    webots::Node *frontArmor = robot.getFromDef("FRONT_ARMOR");
    webots::Node *rearArmor = robot.getFromDef("REAR_ARMOR");
    webots::Node *leftArmor = robot.getFromDef("LEFT_ARMOR");
    webots::Node *rightArmor = robot.getFromDef("RIGHT_ARMOR");
    if (cameraPitchNode == nullptr)
      throw std::runtime_error("missing SIM_CAMERA_PITCH");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("webots_score_system");
    auto scorePub = node->create_publisher<std_msgs::msg::String>(scoreTopic, 10);

    std::mutex gimbalMutex;
    GimbalSnapshot gimbal;
    auto gimbalSub = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
        gimbalCmdTopic, rclcpp::SensorDataQoS(),
        [&](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
          GimbalSnapshot next;
          next.fireAdvice = msg->fire_advice;
          next.valid = msg->mode == rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT ||
                       msg->mode == rm_interfaces::msg::GimbalCmd::MODE_UNKNOWN;
          std::lock_guard<std::mutex> lock(gimbalMutex);
          gimbal = next;
        });

    std::atomic<bool> stopRos{false};
    std::thread rosThread([&]() {
      while (rclcpp::ok() && !stopRos.load(std::memory_order_relaxed)) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });

    RCLCPP_INFO(
        node->get_logger(),
        "score_system enabled=%s cmd_gimbal=%s score_topic=%s fire_delay=%.3fs "
        "fire_rate=%.3fHz bullet_speed=%.3fm/s armor=%.3fx%.3fm",
        scoreEnabled ? "true" : "false",
        gimbalCmdTopic.c_str(),
        scoreTopic.c_str(),
        fireDelay,
        fireRateHz,
        bulletSpeed,
        armorWidth,
        armorHeight);

    ScoreStats stats;
    stats.startTime = robot.getTime();
    std::deque<PendingShot> pendingShots;
    std::vector<ActiveProjectile> activeProjectiles;
    double nextFireReadyTime = 0.0;
    double previousScoreTime = stats.startTime;
    bool previousFireGate = false;
    double lastPublishTime = -std::numeric_limits<double>::infinity();
    const Vec3 gravity{0.0, 0.0, -gravityMagnitude};

    while (rclcpp::ok() && robot.step(timestep) != -1) {
      if (!scoreEnabled)
        continue;

      const double now = robot.getTime();
      GimbalSnapshot snapshot;
      {
        std::lock_guard<std::mutex> lock(gimbalMutex);
        snapshot = gimbal;
      }
      const bool fireGate = snapshot.valid && snapshot.fireAdvice;
      bool firedThisTick = false;
      if (fireGate && nextFireReadyTime <= now + 1e-9) {
        double decisionTime = now;
        // If fire_advice stayed high across ticks, fire at the exact CD boundary
        // instead of quantizing the interval up to the score controller step.
        if (fireInterval > 0.0 && previousFireGate &&
            nextFireReadyTime > previousScoreTime + 1e-9) {
          decisionTime = nextFireReadyTime;
        }
        pendingShots.push_back({decisionTime + fireDelay});
        nextFireReadyTime =
            fireInterval > 0.0
                ? decisionTime + fireInterval
                : now + static_cast<double>(timestep) / 1000.0;
        firedThisTick = true;
      }
      const double cooldownRemaining =
          std::max(0.0, nextFireReadyTime - now);

      while (!pendingShots.empty() && pendingShots.front().fireTime <= now) {
        const PendingShot shot = pendingShots.front();
        pendingShots.pop_front();

        const double *cameraOrientation = cameraPitchNode->getOrientation();
        if (cameraOrientation == nullptr)
          throw std::runtime_error("cannot read SIM_CAMERA_PITCH orientation");

        const Vec3 cameraPosition = nodePosition(cameraPitchNode);
        const Vec3 origin = cameraPosition + matrixVectorProduct(cameraOrientation, shooterOffset);
        const Vec3 direction = nodeLocalDirection(cameraPitchNode, shooterLocalForward);
        ++stats.shots;
        ActiveProjectile projectile;
        projectile.shotId = stats.shots;
        projectile.launchTime = shot.fireTime;
        projectile.origin = origin;
        projectile.velocity = direction * bulletSpeed;
        projectile.previousPosition = origin;
        activeProjectiles.push_back(projectile);
      }

      const Vec3 cameraPosition = nodePosition(cameraPitchNode);
      const std::vector<ArmorPlane> allArmors = {
          orientArmorTowardCamera(
              readArmorPlane(frontArmor, "front", armorWidth * 0.5,
                             armorHeight * 0.5, armorPitch),
              cameraPosition),
          orientArmorTowardCamera(
              readArmorPlane(rearArmor, "rear", armorWidth * 0.5,
                             armorHeight * 0.5, armorPitch),
              cameraPosition),
          orientArmorTowardCamera(
              readArmorPlane(leftArmor, "left", armorWidth * 0.5,
                             armorHeight * 0.5, armorPitch),
              cameraPosition),
          orientArmorTowardCamera(
              readArmorPlane(rightArmor, "right", armorWidth * 0.5,
                             armorHeight * 0.5, armorPitch),
              cameraPosition)};

      std::vector<ArmorPlane> hittableArmors;
      hittableArmors.reserve(allArmors.size());
      for (const auto &armor : allArmors) {
        if (armorFacesCamera(armor, cameraPosition))
          hittableArmors.push_back(armor);
      }

      auto projectileIt = activeProjectiles.begin();
      while (projectileIt != activeProjectiles.end()) {
        const double flightTime = now - projectileIt->launchTime;
        if (flightTime > maxFlightTime) {
          ++stats.misses;
          const uint64_t shotId = projectileIt->shotId;
          projectileIt = activeProjectiles.erase(projectileIt);
          RCLCPP_INFO(node->get_logger(),
                      "shot=%lu miss %s",
                      static_cast<unsigned long>(shotId),
                      makeScoreMessage(stats, now, firedThisTick, snapshot.fireAdvice,
                                       cooldownRemaining).c_str());
          continue;
        }

        const Vec3 curr = projectilePosition(projectileIt->origin,
                                             projectileIt->velocity,
                                             gravity,
                                             std::max(0.0, flightTime));
        bool hit = false;
        std::string hitArmor;
        double hitFraction = 0.0;
        for (const auto &armor : hittableArmors) {
          double fraction = 0.0;
          if (segmentIntersectsArmor(projectileIt->previousPosition, curr,
                                     armor, fraction)) {
            hit = true;
            hitArmor = armor.name;
            hitFraction = fraction;
            break;
          }
        }

        if (hit) {
          ++stats.hits;
          const uint64_t shotId = projectileIt->shotId;
          const double dt = static_cast<double>(timestep) / 1000.0;
          const double hitFlightTime =
              std::max(0.0, flightTime - dt + dt * hitFraction);
          projectileIt = activeProjectiles.erase(projectileIt);
          RCLCPP_INFO(node->get_logger(),
                      "shot=%lu hit=%s flight=%.3fs %s",
                      static_cast<unsigned long>(shotId),
                      hitArmor.c_str(),
                      hitFlightTime,
                      makeScoreMessage(stats, now, firedThisTick, snapshot.fireAdvice,
                                       cooldownRemaining).c_str());
          continue;
        }

        projectileIt->previousPosition = curr;
        ++projectileIt;
      }

      if (now - lastPublishTime >= publishPeriodMs / 1000.0) {
        std_msgs::msg::String msg;
        msg.data = makeScoreMessage(stats, now, firedThisTick, snapshot.fireAdvice,
                                    cooldownRemaining);
        scorePub->publish(msg);
        lastPublishTime = now;
      }

      previousScoreTime = now;
      previousFireGate = fireGate;
    }

    stopRos.store(true, std::memory_order_relaxed);
    if (rosThread.joinable())
      rosThread.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "score_system controller error: " << error.what() << std::endl;
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
