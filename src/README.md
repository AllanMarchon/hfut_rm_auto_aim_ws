# HFUT RM Auto Aim v2

当前 v2 分支只保留 SP `aim_v2` 自瞄主链路，旧 HFUT 自瞄/打符/滤波实验包已移除。

## 项目结构

```text
src/
├── rm_auto_aim/aim_v2          SP 自瞄核心
├── rm_bringup                  启动文件和调参配置
├── rm_hardware_driver          相机、视频源、串口/虚拟串口
├── rm_interfaces               自定义 msg/srv
├── rm_robot_description        URDF 和坐标系
├── rm_upstart                  自启动脚本
└── rm_utils                    通用工具
```

## 环境配置

SP25 原工程的实车参考环境：

- Ubuntu 22.04
- Intel NUC12WSKI7，i7-1260P，16GB
- 海康 MV-CS016-10UC + 海康 6mm 镜头
- RoboMaster 开发板 C 型，STM32F407，C 板内置 BMI088 IMU
- 通信方式：USB2CAN（旧）或 MicroUSB 虚拟串口（新）
- 辅助工具：NoMachine、PlotJuggler

本仓库已经把 SP25 自瞄核心接入 HFUT ROS2 硬件链路，不再使用 SP25 原工程的纯 CMake 主程序。推荐环境：

- Ubuntu 22.04
- ROS2 Humble
- OpenVINO 2024.x，优先先用 CPU 跑通
- OpenCV 4、Eigen3、Ceres、fmt、spdlog、yaml-cpp
- 海康相机或 MindVision 相机；没有实体相机时可以先用 `video_player`
- MicroUSB 串口或虚拟串口；没有下位机时可以先用 `virtual_serial`

安装基础工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  git \
  g++ \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  openssh-server \
  screen \
  can-utils
```

第一次使用 `rosdep` 时初始化：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经初始化，忽略即可。

安装常用依赖：

```bash
sudo apt install -y \
  libopencv-dev \
  libeigen3-dev \
  libceres-dev \
  libfmt-dev \
  libspdlog-dev \
  libyaml-cpp-dev \
  libusb-1.0-0-dev \
  nlohmann-json3-dev \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-camera-calibration \
  ros-humble-serial-driver \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-xacro \
  ros-humble-rosidl-default-generators
```

`aim_v2` 需要 OpenVINO。以 `/opt/intel/openvino_2024.6.0` 为例：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
ls "$OpenVINO_DIR/OpenVINOConfig.cmake"
```

如果你的 OpenVINO 版本或安装路径不同，把路径改成实际值。

进入工作空间根目录后安装 ROS 依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

如果 `rosdep` 报 `openvino`、`ceres` 等 key 无法解析，确认已手动安装 OpenVINO 和 `libceres-dev` 后跳过这些 key：

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "openvino ceres"
```

实体串口需要把当前用户加入 `dialout` 组，执行后重新登录：

```bash
sudo usermod -a -G dialout $USER
```

海康相机链路需要安装 MVS SDK 到 `/opt/MVS`，运行前确认动态库路径：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64
```

MindVision 相机包使用仓库内的 `mvsdk`，通常不需要额外安装 SDK。

USB2CAN 是 SP25 旧通信方式，本仓库优先使用 MicroUSB 串口或 `virtual_serial`。如果确实要用 CAN，可以参考 SP25 的 udev 规则思路，把 `can0/can1` 自动设置为 1 Mbps：

```text
ACTION=="add", KERNEL=="can0", RUN+="/sbin/ip link set can0 up type can bitrate 1000000"
ACTION=="add", KERNEL=="can1", RUN+="/sbin/ip link set can1 up type can bitrate 1000000"
```

SP25 原工程的编译方式是：

```bash
cmake -B build
make -C build/ -j`nproc`
./build/auto_aim_test
```

本仓库不要直接用这套命令编 `aim_v2`，应使用下面的 ROS2 `colcon` 流程。

更详细的 OpenVINO、相机、串口和常见编译错误说明见：

```text
rm_auto_aim/aim_v2/readme.md
```

## 调参入口

优先看：

```text
rm_bringup/config/launch_params_decoupled.yaml
rm_bringup/config/video_player_params.yaml
rm_bringup/config/virtual_serial_params.yaml
rm_bringup/config/camera_info.yaml
rm_auto_aim/aim_v2/config/aim_v2.yaml
```

视频虚拟仿真默认链路：

```text
video_player -> /image_raw
virtual_serial -> /serial/receive
aim_v2_node -> /armor_solver/cmd_gimbal
rm_serial_driver -> 下位机
```

## 编译与运行

```bash
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
colcon build --symlink-install --packages-up-to rm_bringup
source install/setup.bash
ros2 launch rm_bringup bringup_v2.launch.py \
  image_source:=video \
  virtual_serial:=true \
  enable_fire:=false
```

如果只想先编 SP 自瞄核心：

```bash
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
colcon build --symlink-install --packages-up-to aim_v2
```

## Foxglove 可视化

SP v2 版可以直接通过 `foxglove_bridge` 连接 Foxglove。旧版的分散可视化话题已经随旧包移除；现在主要看 `aim_v2` 自己发布的调试话题：

```text
/image_raw
/camera_info
/tf
/serial/receive
/armor_solver/cmd_gimbal
/gimbal_pipeline/debug/image
/gimbal_pipeline/debug/markers
```

启动桥接：

```bash
sudo apt install -y ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove 连接：

```text
ws://localhost:8765
```
