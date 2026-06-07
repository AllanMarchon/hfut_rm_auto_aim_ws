# aim_v2 简单版环境搭建

这个文件是简单版

这里不是完整复刻 SP25 工程，也不是让整个 HFUT 仓库所有包一次性通过编译。当前 `aim_v2` 的目标是：

- 使用 HFUT 现有 ROS2 相机、串口、话题链路。
- 使用 SP25 的自瞄核心算法、OpenVINO 推理、Planner/TinyMPC。
- 先跑通自瞄，再考虑打符、调试工具、全仓库构建。

## 1. 系统环境

推荐环境：

- Ubuntu 22.04
- ROS2 Humble
- OpenVINO 2024.x
- CPU 推理优先，先不要折腾 GPU

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
```

## 2. 安装基础依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  git \
  cmake \
  g++ \
  python3-colcon-common-extensions \
  python3-rosdep \
  libopencv-dev \
  libfmt-dev \
  libeigen3-dev \
  libspdlog-dev \
  libyaml-cpp-dev \
  libusb-1.0-0-dev \
  nlohmann-json3-dev \
  can-utils \
  screen
```

ROS2 相关依赖：

```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-serial-driver \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-rosidl-default-generators
```

如果后面要一起编译串口链路，需要 Ceres：

```bash
sudo apt install -y libceres-dev
```

## 3. 配 OpenVINO

SP25 使用 OpenVINO，`aim_v2` 也需要 OpenVINO。建议安装 OpenVINO 2024.x。

如果安装在 `/opt/intel/openvino_2024.6.0`，编译前执行：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
```

检查 CMake 配置文件是否存在：

```bash
ls $OpenVINO_DIR/OpenVINOConfig.cmake
```

能看到文件就说明 CMake 大概率能找到 OpenVINO。

如果你的 OpenVINO 不是 `2024.6.0`，把路径换成实际版本。

## 4. 进入工作空间

```bash
cd ~/hfut_rm_auto_aim_ws-2.0
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
```

如果你的工作空间不在 `~/hfut_rm_auto_aim_ws-2.0`，换成实际路径。

## 5. 编译自瞄

先编 `aim_v2`：

```bash
colcon build --symlink-install --packages-up-to aim_v2
```

成功后执行：

```bash
source install/setup.bash
```

如果编译报 OpenVINO 找不到，回到第 3 步检查 `OpenVINO_DIR`。

如果编译报 `GimbalCmd` 字段不对，清理旧构建后重来：

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
colcon build --symlink-install --packages-up-to aim_v2
```

## 6. 编译串口链路

自瞄本体通过后，再编串口：

```bash
colcon build --symlink-install --packages-up-to rm_serial_driver
source install/setup.bash
```

如果这里报 Ceres：

```bash
sudo apt install -y libceres-dev
```

然后重新编译。

## 7. 先用安全方式运行

第一次不要开火。

如果相机和串口话题已经由别的 launch 启动，可以只启动 `aim_v2`：

```bash
ros2 launch aim_v2 aim_v2.launch.py \
  image_topic:=/image_raw \
  serial_topic:=/serial/receive \
  cmd_topic:=/armor_solver/cmd_gimbal \
  enable_fire:=false \
  control_backend:=planner
```

如果 Planner 跑起来有问题，先回退：

```bash
ros2 launch aim_v2 aim_v2.launch.py \
  enable_fire:=false \
  control_backend:=aimer \
  async_inference:=false
```

## 8. 使用 bringup_v2

如果 `rm_bringup` 也能编译通过，可以直接跑：

```bash
ros2 launch rm_bringup bringup_v2.launch.py \
  virtual_serial:=true \
  image_source:=video \
  enable_fire:=false \
  control_backend:=planner
```

如果使用真实相机：

```bash
ros2 launch rm_bringup bringup_v2.launch.py \
  virtual_serial:=true \
  image_source:=mindvision \
  enable_fire:=false \
  control_backend:=planner
```

或者海康：

```bash
ros2 launch rm_bringup bringup_v2.launch.py \
  virtual_serial:=true \
  image_source:=hik \
  enable_fire:=false \
  control_backend:=planner
```

当前 v2 分支只保留 SP `aim_v2` 主链路，`rm_bringup` 已不再声明旧 `rm_rune` 或旧 `rm_auto_aim` 聚合包依赖。

## 9. 串口权限

实车串口需要权限：

```bash
sudo usermod -a -G dialout $USER
```

执行后重新登录或重启。

查看串口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

如果使用固定串口名，例如 `/dev/gimbal`，按真实设备 ID 写 udev 规则。

## 10. 跑起来后看什么

确认话题：

```bash
ros2 topic list | grep -E "image_raw|serial/receive|cmd_gimbal"
```

看自瞄输出：

```bash
ros2 topic echo /armor_solver/cmd_gimbal --once
```

确认接口字段：

```bash
ros2 interface show rm_interfaces/msg/GimbalCmd
```

应该能看到：

- `yaw`
- `pitch`
- `yaw_v`
- `pitch_v`
- `yaw_a`
- `pitch_a`
- `fire_advice`

## 11. 实车前必须确认

实车前保持：

```bash
enable_fire:=false
```

确认下面几件事后再考虑开火：

- 图像话题 `/image_raw` 正常。
- 串口反馈 `/serial/receive` 正常。
- 自瞄输出 `/armor_solver/cmd_gimbal` 正常。
- yaw/pitch 方向没有反。
- 下位机确认 `infantry_32` 协议后 8 字节确实接收 `pitch_a/yaw_a`。
- `control_backend:=planner` 稳定；不稳定就先用 `control_backend:=aimer`。

## 12. 最容易遇到的几个错误

### OpenVINO 找不到

重新 source：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
```

### cv_bridge 找不到

```bash
sudo apt install -y ros-humble-cv-bridge
```

### Ceres 找不到

```bash
sudo apt install -y libceres-dev
```

### rm_bringup 编译范围

当前主链路编译：

```bash
colcon build --symlink-install --packages-up-to rm_bringup
```

如果只验证 `aim_v2` 本体，可以直接：

```bash
colcon build --symlink-install --packages-up-to aim_v2
source install/setup.bash
ros2 launch aim_v2 aim_v2.launch.py enable_fire:=false
```
