# aim_v2 环境配置与编译说明

本文档用于把 SP25 自瞄核心移植到 HFUT ROS2 工程后的环境准备和编译排错。`aim_v2` 不是原样编译 SP25 工程，而是：

- 保留 HFUT 的 ROS2 相机、串口、话题链路。
- 使用 SP25 的自瞄核心算法、OpenVINO 推理、Planner、TinyMPC。
- 用 `colcon` 编译 ROS2 包，而不是 SP25 README 中的 `cmake -B build && make`。

## 1. 推荐系统

SP25 README 中的实车环境：

- Ubuntu 22.04
- Intel NUC12WSKI7 / i7-1260P / 16GB
- 海康相机或 MindVision 相机
- OpenVINO
- MicroUSB 虚拟串口或 USB2CAN

`aim_v2` 建议环境：

- Ubuntu 22.04
- ROS2 Humble
- OpenVINO 2024.x，优先 CPU 跑通
- OpenCV 4、Eigen3、fmt、spdlog、yaml-cpp
- 如果需要编译 `rm_serial_driver`，还需要 `rm_utils` 依赖的 Ceres

## 2. 基础 ROS2 环境

先确认 ROS2 Humble 已安装，并且每个新终端都能 source：

```bash
source /opt/ros/humble/setup.bash
```

建议安装常用编译工具：

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
  can-utils \
  openssh-server \
  screen
```

如果是第一次使用 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经初始化，忽略即可。

## 3. aim_v2 必需的系统库

SP25 README 中给出的基础依赖对 `aim_v2` 仍然适用：

```bash
sudo apt install -y \
  libopencv-dev \
  libfmt-dev \
  libeigen3-dev \
  libspdlog-dev \
  libyaml-cpp-dev \
  libusb-1.0-0-dev \
  nlohmann-json3-dev
```

`aim_v2` 自身 CMake 直接查找：

- `Eigen3`
- `OpenCV`
- `OpenVINO`
- `fmt`
- `spdlog`
- `yaml-cpp`
- ROS2: `rclcpp`、`sensor_msgs`、`std_msgs`、`cv_bridge`、`ament_index_cpp`
- 本仓库: `rm_interfaces`

如果你的 ROS2 是最小安装，可能还需要补：

```bash
sudo apt install -y \
  ros-humble-ament-cmake \
  ros-humble-ament-index-cpp \
  ros-humble-rclcpp \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-serial-driver \
  ros-humble-rosidl-default-generators
```

## 4. OpenVINO

SP25 的 CMake 写死过：

```cmake
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
```

`aim_v2` 没有写死路径，而是：

```cmake
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
```

所以只要 CMake 能找到 OpenVINO 即可。推荐使用 OpenVINO 2024.x。

如果使用 archive 安装到 `/opt/intel/openvino_2024.6.0`，每次编译前执行：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
```

验证：

```bash
ls "$OpenVINO_DIR/OpenVINOConfig.cmake"
```

如果该文件存在，`find_package(OpenVINO REQUIRED COMPONENTS Runtime)` 通常就能通过。

当前 `aim_v2/config/aim_v2.yaml` 默认：

```yaml
device: CPU
```

因此先不要折腾 Intel GPU 运行时。CPU 跑通后，再考虑 GPU/iGPU。SP25 README 中 GPU 推理是可选项，不是第一阶段必须项。

## 5. Ceres 与 rm_serial_driver

`aim_v2` 本身不直接依赖 Ceres，但 `rm_serial_driver` 依赖 `rm_utils`，而 `rm_utils` 的 CMake 会查找 Ceres。

如果要一起编译串口链路：

```bash
sudo apt install -y libceres-dev
```

如果只想先验证 `aim_v2` 本体，可以先不编 `rm_serial_driver`。

## 6. 相机 SDK

SP25 README 要求 MindVision SDK 或 HikRobot SDK。`aim_v2` 本体不直接打开相机，它只订阅 `/image_raw`，所以：

- 只编译 `aim_v2`：不需要相机 SDK。
- 要跑 `bringup_v2.launch.py` 并启动实体相机：需要安装对应相机 SDK。
- 用视频或已有 `/image_raw`：可以先不装实体相机 SDK。

海康相机通常还需要配置 MVS 环境；本仓库 `bringup_v2.launch.py` 中给了：

```bash
MVCAM_SDK_PATH=/opt/MVS
MVCAM_COMMON_RUNENV=/opt/MVS/lib
```

## 7. 串口权限

SP25 README 的串口权限步骤仍然适用：

```bash
sudo usermod -a -G dialout $USER
```

执行后需要重新登录，或者重启。

查看串口设备：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

如果要固定设备名，例如 `/dev/gimbal`：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'
sudo touch /etc/udev/rules.d/99-usb-serial.rules
```

在规则文件中写入类似内容，替换为真实 ID：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="1234", ATTRS{serial}=="A1234567", SYMLINK+="gimbal"
```

重新加载：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/gimbal
```

## 8. 推荐编译顺序

进入工作空间根目录：

```bash
cd ~/hfut_rm_auto_aim_ws-2.0
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
```

先安装可解析的依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

如果 rosdep 报 `openvino`、`ceres` 等 key 不能解析，先确认已经手动安装 OpenVINO 和 `libceres-dev`，然后跳过这些 key：

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "openvino ceres"
```

第一步，只编 `aim_v2` 和它的接口依赖：

```bash
colcon build --symlink-install --packages-up-to aim_v2
```

第二步，编串口链路：

```bash
colcon build --symlink-install --packages-up-to rm_serial_driver
```

第三步，再尝试 bringup：

```bash
colcon build --symlink-install --packages-up-to rm_bringup
```

如果机器内存紧张：

```bash
colcon build --symlink-install --packages-up-to aim_v2 --parallel-workers 2
```

## 9. 运行方式

编译成功后：

```bash
source install/setup.bash
```

只启动 `aim_v2` 节点，适合已有相机和串口话题时使用：

```bash
ros2 launch aim_v2 aim_v2.launch.py \
  image_topic:=/image_raw \
  serial_topic:=/serial/receive \
  cmd_topic:=/armor_solver/cmd_gimbal \
  enable_fire:=false \
  control_backend:=planner
```

如果 Planner 编译通过但运行不稳，可以先回退 Aimer：

```bash
ros2 launch aim_v2 aim_v2.launch.py \
  enable_fire:=false \
  control_backend:=aimer \
  async_inference:=false
```

如果 `rm_bringup` 已经编译通过，可以使用整条测试链路：

```bash
ros2 launch rm_bringup bringup_v2.launch.py \
  virtual_serial:=true \
  image_source:=video \
  enable_fire:=false \
  control_backend:=planner
```

实车前保持：

```bash
enable_fire:=false
```

确认 yaw/pitch 方向、模式切换、串口协议后，再打开发射建议。

## 10. 常见编译错误

### 10.1 找不到 OpenVINO

报错特征：

```text
Could not find a package configuration file provided by "OpenVINO"
```

处理：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
ls "$OpenVINO_DIR/OpenVINOConfig.cmake"
```

如果你的 OpenVINO 版本不是 `2024.6.0`，把路径改成实际版本。

### 10.2 找不到 Ceres

报错特征：

```text
Could not find a package configuration file provided by "Ceres"
```

处理：

```bash
sudo apt install -y libceres-dev
```

这通常来自 `rm_utils`，不是 `aim_v2` 本体。

### 10.3 rm_bringup 编译范围

当前 v2 分支只保留 SP `aim_v2` 主链路。`rm_bringup/package.xml` 不再声明旧 `rm_auto_aim` 聚合包或 `rm_rune` 依赖。

常用编译：

```bash
colcon build --symlink-install --packages-up-to rm_bringup
```

### 10.4 找不到 cv_bridge

处理：

```bash
sudo apt install -y ros-humble-cv-bridge
```

### 10.5 GimbalCmd 字段不匹配

`aim_v2` 会发布：

- `yaw`
- `pitch`
- `yaw_v`
- `pitch_v`
- `yaw_a`
- `pitch_a`
- `fire_advice`

如果编译报 `GimbalCmd` 没有某个字段，说明 `rm_interfaces/msg/GimbalCmd.msg` 和 `aim_v2_node.cpp` 不匹配。先检查本仓库的接口文件，不要用旧 install 空间里的接口：

```bash
rm -rf build install log
colcon build --symlink-install --packages-up-to aim_v2
```

### 10.6 TinyMPC/codegen 相关错误

`aim_v2` 只需要 TinyMPC 求解器，不需要生成代码工具。当前 CMake 编译：

- `admm.cpp`
- `rho_benchmark.cpp`
- `tiny_api.cpp`

不应把 `codegen.cpp` 当作必须目标。如果错误来自 codegen，优先检查 CMake 是否被误改。

## 11. 和 SP25 README 的差异

SP25 原工程：

```bash
cmake -B build
make -C build/ -j`nproc`
./build/auto_aim_test
```

HFUT `aim_v2`：

```bash
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
colcon build --symlink-install --packages-up-to aim_v2
source install/setup.bash
ros2 launch aim_v2 aim_v2.launch.py enable_fire:=false
```

不要把 SP25 的 `CBoard`、相机 main loop、USB2CAN 主程序作为 `aim_v2` 必需环境。这里的目标是让 SP25 自瞄算法跑在 HFUT 已有硬件链路上。

## 12. 最小验证清单

编译后先确认这些点：

```bash
ros2 pkg prefix aim_v2
ros2 pkg prefix rm_interfaces
ros2 interface show rm_interfaces/msg/GimbalCmd
```

启动后看话题：

```bash
ros2 topic list | grep -E "image_raw|serial/receive|cmd_gimbal"
ros2 topic echo /armor_solver/cmd_gimbal --once
```

安全原则：

- 第一次运行使用 `enable_fire:=false`。
- 没接实体串口时使用 `virtual_serial:=true`。
- Planner 不稳定时先用 `control_backend:=aimer`。
- OpenVINO 异步路径不稳定时先用 `async_inference:=false`。
