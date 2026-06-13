#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORLD_TEMPLATE="${PROJECT_DIR}/worlds/flat_camera_world.wbt.template"
WORLD_FILE="${PROJECT_DIR}/worlds/dark_camera_world.wbt"
RENDER_ONLY=0

if [ "${1:-}" = "--render-only" ]; then
  RENDER_ONLY=1
  shift
fi

CONFIG_FILES=(
  "${PROJECT_DIR}/config/environment.env"
  "${PROJECT_DIR}/config/target_robot.env"
  "${PROJECT_DIR}/config/camera_robot.env"
)

if [ -n "${WEBOTS_CAMERA_CONFIG:-}" ]; then
  CONFIG_FILES+=("${WEBOTS_CAMERA_CONFIG}")
fi

for CONFIG_FILE in "${CONFIG_FILES[@]}"; do
  if [ -f "${CONFIG_FILE}" ]; then
    # shellcheck source=/dev/null
    set -a
    source "${CONFIG_FILE}"
    set +a
  fi
done

: "${WEBOTS_BASIC_TIME_STEP:=16}"
: "${WEBOTS_OPTIMAL_THREAD_COUNT:=2}"
: "${WEBOTS_GROUND_SIZE_X:=12}"
: "${WEBOTS_GROUND_SIZE_Y:=8}"
: "${WEBOTS_GROUND_THICKNESS:=0.05}"
: "${WEBOTS_GROUND_Z:=-0.025}"
: "${WEBOTS_GROUND_COLOR:=0.24 0.25 0.24}"
: "${WEBOTS_GROUND_ROUGHNESS:=0.92}"
: "${WEBOTS_SKY_COLOR:=0.012 0.013 0.016}"
: "${WEBOTS_LIGHT_INTENSITY:=1.045}"
: "${WEBOTS_LIGHT_AMBIENT_INTENSITY:=0.55}"
: "${WEBOTS_LIGHT_DIRECTION:=0.25 0.35 -1}"
: "${WEBOTS_ARMOR_TARGET_X:=3}"
: "${WEBOTS_ARMOR_TARGET_Y:=0}"
: "${WEBOTS_ARMOR_TARGET_Z:=0.165}"
: "${WEBOTS_ARMOR_TARGET_YAW:=0}"
: "${WEBOTS_ARMOR_BODY_MESH_URL:=../meshes/armor_red4_body_plate_lights.obj}"
: "${WEBOTS_ARMOR_MESH_SCALE:=0.001 0.001 0.001}"
: "${WEBOTS_ARMOR_MESH_ROTATION:=0 1 0 3.14159265359}"
: "${WEBOTS_ARMOR_MESH_OFFSET:=-0.0018354 -0.13427105 -0.01999975}"
: "${WEBOTS_ARMOR_MESH_CCW:=TRUE}"
: "${WEBOTS_ARMOR_BOUNDING_SIZE:=0.135 0.080 0.135}"
: "${WEBOTS_ARMOR_PITCH:=0.2617993877991494}"
: "${WEBOTS_CAMERA_X:=0}"
: "${WEBOTS_CAMERA_Y:=0}"
: "${WEBOTS_CAMERA_Z:=0.35}"
: "${WEBOTS_CAMERA_LOOK_AT_TARGET:=true}"
: "${WEBOTS_CAMERA_LOOK_AT_Z:=}"
: "${WEBOTS_CAMERA_LOOK_AT_Z_OFFSET:=0}"
: "${WEBOTS_CAMERA_GREEN_AXIS_OFFSET:=1.57079632679}"
: "${WEBOTS_CAMERA_YAW:=0}"
: "${WEBOTS_CAMERA_ROTATION:=0 0 1 ${WEBOTS_CAMERA_YAW}}"
: "${WEBOTS_CAMERA_TILT:=0.06158867632}"
: "${WEBOTS_CAMERA_X_ROTATION:=0}"
: "${WEBOTS_CAMERA_ROLL:=0}"
: "${WEBOTS_CAMERA_NAME:=camera}"
: "${WEBOTS_CAMERA_WIDTH:=1440}"
: "${WEBOTS_CAMERA_HEIGHT:=1080}"
: "${WEBOTS_CAMERA_FOV:=0.7850335620966933}"
: "${WEBOTS_CAMERA_NEAR:=0.02}"
: "${WEBOTS_CAMERA_FAR:=20}"
: "${WEBOTS_CAMERA_ANTI_ALIASING:=TRUE}"
: "${WEBOTS_CAMERA_MOTION_BLUR:=0}"
: "${WEBOTS_CAMERA_BODY_SIZE:=0.10 0.08 0.08}"
: "${WEBOTS_CAMERA_BODY_COLOR:=0.82 0.86 0.90}"
: "${WEBOTS_CAMERA_FX:=1739.130435}"
: "${WEBOTS_CAMERA_FY:=1739.130435}"
: "${WEBOTS_CAMERA_CX:=719.5}"
: "${WEBOTS_CAMERA_CY:=539.5}"
: "${WEBOTS_CAMERA_DISTORTION_MODEL:=plumb_bob}"
: "${WEBOTS_CAMERA_D:=0 0 0 0 0}"
: "${WEBOTS_TARGET_SPIN_RATE:=3.0}"
: "${WEBOTS_CONTROLLER_STEP_MS:=32}"
: "${WEBOTS_CAMERA_PERIOD_MS:=32}"
: "${WEBOTS_TARGET_CONTROLLER_STEP_MS:=16}"
: "${WEBOTS_DISABLE_CAMERA_READ:=false}"
: "${WEBOTS_DISABLE_IMAGE_PUBLISH:=false}"
: "${WEBOTS_SKIP_UNSUBSCRIBED_IMAGES:=true}"
: "${WEBOTS_SPIN_PERIOD:=8}"
: "${WEBOTS_PROFILE_PERIOD:=120}"
: "${WEBOTS_FIRE_DELAY_MS:=0.0}"
: "${WEBOTS_FIRE_RATE_HZ:=20.0}"
: "${WEBOTS_BULLET_SPEED:=22.5}"
: "${WEBOTS_SCORE_ENABLED:=true}"
: "${WEBOTS_SCORE_TOPIC:=/webots/score}"
: "${WEBOTS_SCORE_CONTROLLER_STEP_MS:=32}"
: "${WEBOTS_SCORE_PUBLISH_PERIOD_MS:=200}"
: "${WEBOTS_SCORE_ARMOR_WIDTH:=0.135}"
: "${WEBOTS_SCORE_ARMOR_HEIGHT:=0.135}"
: "${WEBOTS_SCORE_MAX_FLIGHT_TIME:=2.0}"
: "${WEBOTS_SCORE_GRAVITY:=9.80665}"
: "${WEBOTS_SCORE_SHOOTER_OFFSET_X:=0.0}"
: "${WEBOTS_SCORE_SHOOTER_OFFSET_Y:=0.0}"
: "${WEBOTS_SCORE_SHOOTER_OFFSET_Z:=0.0}"
: "${WEBOTS_SCORE_SHOOTER_FORWARD_X:=1.0}"
: "${WEBOTS_SCORE_SHOOTER_FORWARD_Y:=0.0}"
: "${WEBOTS_SCORE_SHOOTER_FORWARD_Z:=0.0}"

render_world() {
  local content
  content="$(<"${WORLD_TEMPLATE}")"
  content="${content//@WEBOTS_BASIC_TIME_STEP@/${WEBOTS_BASIC_TIME_STEP}}"
  content="${content//@WEBOTS_OPTIMAL_THREAD_COUNT@/${WEBOTS_OPTIMAL_THREAD_COUNT}}"
  content="${content//@WEBOTS_GROUND_SIZE_X@/${WEBOTS_GROUND_SIZE_X}}"
  content="${content//@WEBOTS_GROUND_SIZE_Y@/${WEBOTS_GROUND_SIZE_Y}}"
  content="${content//@WEBOTS_GROUND_THICKNESS@/${WEBOTS_GROUND_THICKNESS}}"
  content="${content//@WEBOTS_GROUND_Z@/${WEBOTS_GROUND_Z}}"
  content="${content//@WEBOTS_GROUND_COLOR@/${WEBOTS_GROUND_COLOR}}"
  content="${content//@WEBOTS_GROUND_ROUGHNESS@/${WEBOTS_GROUND_ROUGHNESS}}"
  content="${content//@WEBOTS_SKY_COLOR@/${WEBOTS_SKY_COLOR}}"
  content="${content//@WEBOTS_LIGHT_INTENSITY@/${WEBOTS_LIGHT_INTENSITY}}"
  content="${content//@WEBOTS_LIGHT_AMBIENT_INTENSITY@/${WEBOTS_LIGHT_AMBIENT_INTENSITY}}"
  content="${content//@WEBOTS_LIGHT_DIRECTION@/${WEBOTS_LIGHT_DIRECTION}}"
  content="${content//@WEBOTS_ARMOR_TARGET_X@/${WEBOTS_ARMOR_TARGET_X}}"
  content="${content//@WEBOTS_ARMOR_TARGET_Y@/${WEBOTS_ARMOR_TARGET_Y}}"
  content="${content//@WEBOTS_ARMOR_TARGET_Z@/${WEBOTS_ARMOR_TARGET_Z}}"
  content="${content//@WEBOTS_ARMOR_TARGET_YAW@/${WEBOTS_ARMOR_TARGET_YAW}}"
  content="${content//@WEBOTS_ARMOR_BODY_MESH_URL@/${WEBOTS_ARMOR_BODY_MESH_URL}}"
  content="${content//@WEBOTS_ARMOR_MESH_SCALE@/${WEBOTS_ARMOR_MESH_SCALE}}"
  content="${content//@WEBOTS_ARMOR_MESH_ROTATION@/${WEBOTS_ARMOR_MESH_ROTATION}}"
  content="${content//@WEBOTS_ARMOR_MESH_OFFSET@/${WEBOTS_ARMOR_MESH_OFFSET}}"
  content="${content//@WEBOTS_ARMOR_MESH_CCW@/${WEBOTS_ARMOR_MESH_CCW}}"
  content="${content//@WEBOTS_ARMOR_BOUNDING_SIZE@/${WEBOTS_ARMOR_BOUNDING_SIZE}}"
  content="${content//@WEBOTS_ARMOR_PITCH@/${WEBOTS_ARMOR_PITCH}}"
  content="${content//@WEBOTS_CAMERA_X@/${WEBOTS_CAMERA_X}}"
  content="${content//@WEBOTS_CAMERA_Y@/${WEBOTS_CAMERA_Y}}"
  content="${content//@WEBOTS_CAMERA_Z@/${WEBOTS_CAMERA_Z}}"
  content="${content//@WEBOTS_CAMERA_ROTATION@/${WEBOTS_CAMERA_ROTATION}}"
  content="${content//@WEBOTS_CAMERA_TILT@/${WEBOTS_CAMERA_TILT}}"
  content="${content//@WEBOTS_CAMERA_X_ROTATION@/${WEBOTS_CAMERA_X_ROTATION}}"
  content="${content//@WEBOTS_CAMERA_ROLL@/${WEBOTS_CAMERA_ROLL}}"
  content="${content//@WEBOTS_CAMERA_NAME@/${WEBOTS_CAMERA_NAME}}"
  content="${content//@WEBOTS_CAMERA_WIDTH@/${WEBOTS_CAMERA_WIDTH}}"
  content="${content//@WEBOTS_CAMERA_HEIGHT@/${WEBOTS_CAMERA_HEIGHT}}"
  content="${content//@WEBOTS_CAMERA_FOV@/${WEBOTS_CAMERA_FOV}}"
  content="${content//@WEBOTS_CAMERA_NEAR@/${WEBOTS_CAMERA_NEAR}}"
  content="${content//@WEBOTS_CAMERA_FAR@/${WEBOTS_CAMERA_FAR}}"
  content="${content//@WEBOTS_CAMERA_ANTI_ALIASING@/${WEBOTS_CAMERA_ANTI_ALIASING}}"
  content="${content//@WEBOTS_CAMERA_MOTION_BLUR@/${WEBOTS_CAMERA_MOTION_BLUR}}"
  content="${content//@WEBOTS_CAMERA_BODY_SIZE@/${WEBOTS_CAMERA_BODY_SIZE}}"
  content="${content//@WEBOTS_CAMERA_BODY_COLOR@/${WEBOTS_CAMERA_BODY_COLOR}}"
  printf '%s\n' "${content}" > "${WORLD_FILE}"
}

compute_camera_look_at() {
  case "${WEBOTS_CAMERA_LOOK_AT_TARGET}" in
    1|true|TRUE|yes|YES|on|ON) ;;
    *) return ;;
  esac

  local look_at_z
  look_at_z="${WEBOTS_CAMERA_LOOK_AT_Z}"
  if [ -z "${look_at_z}" ]; then
    look_at_z="$(
      awk \
        -v target_z="${WEBOTS_ARMOR_TARGET_Z}" \
        -v offset_z="${WEBOTS_CAMERA_LOOK_AT_Z_OFFSET}" \
        'BEGIN { printf "%.12g", target_z + offset_z; }'
    )"
  fi

  local computed
  computed="$(
    awk \
      -v cx="${WEBOTS_CAMERA_X}" -v cy="${WEBOTS_CAMERA_Y}" -v cz="${WEBOTS_CAMERA_Z}" \
      -v tx="${WEBOTS_ARMOR_TARGET_X}" -v ty="${WEBOTS_ARMOR_TARGET_Y}" \
      -v offset="${WEBOTS_CAMERA_GREEN_AXIS_OFFSET}" \
      -v tz="${look_at_z}" \
      'BEGIN {
        dx = tx - cx;
        dy = ty - cy;
        dz = tz - cz;
        h = sqrt(dx * dx + dy * dy);
        yaw = atan2(dy, dx);
        tilt = atan2(-h, -dz) + offset;
        printf "%.12g %.12g", yaw, tilt;
      }'
  )"
  WEBOTS_CAMERA_YAW="${computed%% *}"
  WEBOTS_CAMERA_TILT="${computed#* }"
  WEBOTS_CAMERA_ROTATION="0 0 1 ${WEBOTS_CAMERA_YAW}"
  WEBOTS_CAMERA_X_ROTATION=0
  WEBOTS_CAMERA_ROLL=0
}

compute_camera_look_at
render_world

if [ "${RENDER_ONLY}" -eq 1 ]; then
  echo "Rendered ${WORLD_FILE}"
  exit 0
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [ -f /root/hfut_rm_auto_aim_ws/install/setup.bash ]; then
  # shellcheck source=/dev/null
  set +u
  source /root/hfut_rm_auto_aim_ws/install/setup.bash
  set -u
fi

build_cpp_controller() {
  local controller_dir="$1"

  if [ ! -f "${controller_dir}/CMakeLists.txt" ]; then
    return
  fi

  cmake -S "${controller_dir}" -B "${controller_dir}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-/opt/ros/humble}"
  cmake --build "${controller_dir}/build" --parallel "${WEBOTS_CONTROLLER_BUILD_JOBS:-4}"
}

: "${WEBOTS_BUILD_CPP_CONTROLLERS:=true}"
case "${WEBOTS_BUILD_CPP_CONTROLLERS}" in
  1|true|TRUE|yes|YES|on|ON)
    build_cpp_controller "${PROJECT_DIR}/controllers/ros2_image_publisher"
    build_cpp_controller "${PROJECT_DIR}/controllers/target_spinner"
    build_cpp_controller "${PROJECT_DIR}/controllers/score_system"
    ;;
esac

if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  export LD_LIBRARY_PATH="${WEBOTS_HOME:-/usr/local/webots}/lib/controller:${LD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="${WEBOTS_HOME:-/usr/local/webots}/lib/controller"
fi
export WEBOTS_PYTHON_COMMAND="${WEBOTS_PYTHON_COMMAND:-/usr/bin/python3}"
export NO_PROXY="${NO_PROXY:+${NO_PROXY},}localhost,127.0.0.1,::1"
export no_proxy="${no_proxy:+${no_proxy},}localhost,127.0.0.1,::1"

# Qt/Webots may apply proxy environment variables to local controller sockets.
# Keeping these unset avoids "Connection to proxy closed prematurely" on WSL.
unset http_proxy https_proxy ftp_proxy all_proxy
unset HTTP_PROXY HTTPS_PROXY FTP_PROXY ALL_PROXY

if command -v webots >/dev/null 2>&1; then
  WEBOTS_BIN="webots"
elif [ -x /usr/local/webots/webots ]; then
  WEBOTS_BIN="/usr/local/webots/webots"
elif [ -x /opt/webots/webots ]; then
  WEBOTS_BIN="/opt/webots/webots"
else
  echo "webots executable not found. Install Webots or set PATH/WEBOTS_HOME first." >&2
  exit 127
fi

exec "${WEBOTS_BIN}" "${WORLD_FILE}" "$@"
