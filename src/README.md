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

### 0. 从干净终端开始

ROS2 Humble 建议使用系统 Python 环境编译，不建议在 Conda 的 `base`、`py310` 等环境中编译。若终端前缀显示 `(py310)`，或者 `conda env list` 报：

```text
bash: /home/allan/miniconda3/bin/conda: 没有那个文件或目录
```

说明 `.bashrc` 里还残留旧 Conda 初始化配置。先备份并清理：

```bash
cd ~
cp ~/.bashrc ~/.bashrc.bak.$(date +%Y%m%d_%H%M%S)
grep -nE "conda|py310|miniconda|anaconda|miniforge|mamba" ~/.bashrc ~/.profile ~/.bash_aliases 2>/dev/null
nano ~/.bashrc
```

删除或注释掉类似下面的整段：

```bash
# >>> conda initialize >>>
...
# <<< conda initialize <<<
```

然后重开当前 shell：

```bash
exec bash
```

确认没有 Conda 残留：

```bash
type conda
echo "$CONDA_DEFAULT_ENV"
echo "$CONDA_PREFIX"
echo "$PATH" | tr ':' '\n' | grep -i conda || echo "no conda path"
```

后续命令的目录约定：

- 清理 `.bashrc`、`sudo apt install`：在哪个目录都可以，建议在 `~`。
- 安装 OpenVINO：最终安装到 `/opt/intel/openvino_2024.6.0`。
- 编译和启动本工程：必须进入 `~/hfut_rm_auto_aim_ws-2.0` 工作空间根目录。

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

`aim_v2` 需要 OpenVINO C++ SDK。先检查机器上是否已经安装过：

```bash
find /opt/intel /usr -name OpenVINOConfig.cmake 2>/dev/null
```

如果能找到，例如：

```text
/opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake
```

则编译前使用对应路径：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
ls "$OpenVINO_DIR/OpenVINOConfig.cmake"
```

如果没有找到 `OpenVINOConfig.cmake`，安装 OpenVINO 2024.6 到推荐路径：

```bash
cd ~/Downloads
curl -L https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz -o openvino_2024.6.0.tgz
tar -xf openvino_2024.6.0.tgz

sudo mkdir -p /opt/intel
sudo rm -rf /opt/intel/openvino_2024.6.0
sudo mv l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64 /opt/intel/openvino_2024.6.0

cd /opt/intel/openvino_2024.6.0
sudo -E ./install_dependencies/install_openvino_dependencies.sh
```

`pip install openvino` 主要面向 Python，不等价于本工程需要的 CMake SDK。`ros-humble-openvino` 也不是这里 `find_package(OpenVINO REQUIRED COMPONENTS Runtime)` 要找的包。

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

每个新终端先进入工作空间并加载 ROS2、OpenVINO：

```bash
cd ~/hfut_rm_auto_aim_ws-2.0
source /opt/ros/humble/setup.bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
```

第一步，先编译自瞄核心：

```bash
colcon build --symlink-install --packages-up-to aim_v2 --parallel-workers 2
```

第二步，如果要跑“本地视频 + 虚拟串口”的完整测试链路，只补编需要的运行包：

```bash
source install/setup.bash
colcon build --symlink-install \
  --packages-select video_player rm_serial_driver rm_bringup \
  --parallel-workers 2
```

编译完成后重新加载 install 空间：

```bash
source install/setup.bash
ros2 pkg list | grep rm_bringup
```

启动视频测试链路前，先确认 `src/rm_bringup/config/video_player_params.yaml` 里的 `video_path` 指向真实存在的视频文件。默认 `decoded/output.mp4` 只是占位路径；不确定时建议写绝对路径。

```bash
find ~/hfut_rm_auto_aim_ws-2.0 -iname "*.mp4" -o -iname "*.avi"
nano src/rm_bringup/config/video_player_params.yaml
```

然后启动：

```bash
ros2 launch rm_bringup bringup_v2.launch.py \
  image_source:=video \
  virtual_serial:=true \
  enable_fire:=false
```

判断图像是否正常发布：

```bash
ros2 topic hz /image_raw
ros2 topic list | grep gimbal_pipeline
```

如果要使用海康或 MindVision 实体相机，再单独安装相机 SDK 并编译对应相机包。只跑视频链路时不需要编译 `ros2_hik_camera` 或 `mindvision_camera`。

## Foxglove 可视化

SP v2 版可以直接通过 `foxglove_bridge` 连接 Foxglove。`aim_v2` 仍然保留旧版 Foxglove/RViz 常用的 marker 话题，方便沿用原来的可视化习惯：

```text
/image_raw
/camera_info
/tf
/serial/receive
/armor_solver/cmd_gimbal
/armor_detector/marker
/armor_solver/marker
/gimbal_pipeline/debug/image
/gimbal_pipeline/debug/markers
```

推荐先看这些：

- `/gimbal_pipeline/debug/image`：二维调试图，包含检测板轮廓、跟踪模型板重投影、最终瞄准点。
- `/armor_detector/marker`：兼容旧版检测可视化，namespace 主要是 `armors` 和 `classification`。
- `/armor_solver/marker`：兼容旧版解算/预测可视化，namespace 主要是 `position`、`linear_v`、`angular_v`、`filtered_armors`、`selection`、`armor_points`、`predicted_sequence`。
- `/gimbal_pipeline/debug/markers`：把上面两类 marker 合并到一个话题里，适合临时查看。

启动桥接：

```bash
sudo apt install -y ros-humble-foxglove-bridge

cd ~/hfut_rm_auto_aim_ws-2.0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix rm_interfaces
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765
```

`foxglove_bridge` 必须能找到本仓库的自定义消息包，例如 `rm_interfaces/msg/GimbalCmd`。如果只 source 了 `/opt/ros/humble`，会报 `package 'rm_interfaces' not found`。

确认 bridge 正在对外监听：

```bash
ss -lntp | grep 8765
```

理想情况下能看到 `0.0.0.0:8765` 或 `*:8765`。如果只看到 `127.0.0.1:8765`，本地电脑无法直接连接。

Foxglove 如果开在虚拟机里，连接：

```text
ws://localhost:8765
```

Foxglove 如果开在本地电脑上，而 ROS2 跑在虚拟机里，先查虚拟机 IP：

```bash
hostname -I
```

然后在本地电脑 Foxglove 中连接：

```text
ws://<虚拟机IP>:8765
```

如果本地电脑连不上，先在本地电脑测试端口：

```powershell
Test-NetConnection <虚拟机IP> -Port 8765
```

如果 `PingSucceeded` 和 `TcpTestSucceeded` 都是 `False`，说明宿主机到虚拟机网络不通，不是 ROS2 或 Foxglove 的问题。优先把虚拟机网络改成桥接模式，让虚拟机拿到和本地电脑同一网段的 IP；或者在 NAT 模式里做端口转发，把宿主机 `8765` 转发到虚拟机 `8765`，然后 Foxglove 连接 `ws://127.0.0.1:8765`。
