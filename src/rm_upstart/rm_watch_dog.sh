#!/bin/bash
# ROS 2 watchdog for the v2 SP bringup.
# Default mode is local video + virtual serial.

# ===========================
# Configurable parameters
# ===========================
TIMEOUT="${TIMEOUT:-10}"
NAMESPACE="${NAMESPACE:-}"
IMAGE_SOURCE="${IMAGE_SOURCE:-video}"
VIRTUAL_SERIAL="${VIRTUAL_SERIAL:-true}"

SERIAL_NODE="serial_driver"
if [[ "$VIRTUAL_SERIAL" == "true" ]]; then
    SERIAL_NODE="virtual_serial"
fi

if [[ -n "${WATCHDOG_NODES:-}" ]]; then
    read -r -a NODE_NAMES <<< "$WATCHDOG_NODES"
else
    NODE_NAMES=("$SERIAL_NODE" "gimbal_pipeline")
    if [[ "$IMAGE_SOURCE" == "video" ]]; then
        NODE_NAMES=("video_player" "${NODE_NAMES[@]}")
    fi
fi

USER="$(whoami)"
HOME_DIR=$(eval echo "~$USER")
WORKING_DIR="${WORKING_DIR:-$HOME_DIR/hfut_rm_auto_aim_ws/}"
OUTPUT_FILE="${OUTPUT_FILE:-$WORKING_DIR/screen.output}"

# ===========================
# ROS 2 / RMW environment
# ===========================
rmw="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMW_IMPLEMENTATION="$rmw"
export FASTDDS_SHM_DISABLE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export ROS_HOSTNAME=$(hostname)
export ROS_HOME="${ROS_HOME:-$HOME_DIR/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp}"

source /opt/ros/humble/setup.bash
source "$WORKING_DIR/install/setup.bash"

rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" && -n "$rmw_config" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$rmw_config
elif [[ "$rmw" == "rmw_cyclonedds_cpp" && -n "$rmw_config" ]]; then
    export CYCLONEDDS_URI=$rmw_config
fi

echo "[WATCHDOG] START DOMAIN=$ROS_DOMAIN_ID"
echo "[WATCHDOG] IMAGE_SOURCE=$IMAGE_SOURCE VIRTUAL_SERIAL=$VIRTUAL_SERIAL"
echo "[WATCHDOG] NODES=${NODE_NAMES[*]}"

# ===========================
# SHM cleanup
# ===========================
function cleanup_shm() {
    echo "[WATCHDOG] Cleaning FastDDS SHM..."
    rm -rf /dev/shm/fastrtps_*
    rm -rf /dev/shm/sem.fastrtps_*
    echo "[WATCHDOG] SHM cleaned."
}

# ===========================
# Bringup
# ===========================
function bringup() {
    echo "[WATCHDOG] Bringing up ROS2..."
    echo "[WATCHDOG] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

    source /opt/ros/humble/setup.bash
    source "$WORKING_DIR/install/setup.bash"
    if [[ -f /opt/MVS/bin/set_env_path.sh ]]; then
        source /opt/MVS/bin/set_env_path.sh
    fi

    cd "$WORKING_DIR" || {
        echo "[WATCHDOG] Failed to enter WORKING_DIR=$WORKING_DIR"
        exit 1
    }

    cleanup_shm

    nohup ros2 launch rm_bringup bringup_v2.launch.py \
        "image_source:=$IMAGE_SOURCE" \
        "virtual_serial:=$VIRTUAL_SERIAL" \
        > "$OUTPUT_FILE" 2>&1 &

    echo "[WATCHDOG] ROS2 launched, logging to $OUTPUT_FILE"
}

# ===========================
# Restart
# ===========================
function restart() {
    echo "[WATCHDOG] Restarting ROS2 nodes..."

    pkill -f ros2
    pkill -f component_container
    pkill -f rmw
    sleep 2
    pkill -9 -f ros2

    cleanup_shm

    ros2 daemon stop
    sleep 1
    ros2 daemon start

    bringup
}

# ===========================
# Initial bringup
# ===========================
bringup
sleep "$TIMEOUT"
sleep "$TIMEOUT"

# ===========================
# Heartbeat monitoring loop
# ===========================
while true; do
    for node in "${NODE_NAMES[@]}"; do
        topic="$NAMESPACE/$node/heartbeat"
        echo "[WATCHDOG] Checking $node heartbeat..."

        if ros2 topic list 2>/dev/null | grep -qx "$topic" 2>/dev/null; then
            data_value=$(timeout 10 ros2 topic echo "$topic" --once | grep -o "data: [0-9]*" | awk '{print $2}' 2>/dev/null)
            if [[ -n "$data_value" ]]; then
                echo "    $node is OK. Heartbeat Count: $data_value"
            else
                echo "    Heartbeat lost for $topic, restarting all nodes..."
                restart
                break
            fi
        else
            echo "    Heartbeat topic $topic does not exist, restarting all nodes..."
            restart
            break
        fi
    done
    sleep "$TIMEOUT"
done
