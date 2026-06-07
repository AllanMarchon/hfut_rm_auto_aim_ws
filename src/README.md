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
colcon build --symlink-install --packages-up-to rm_bringup
source install/setup.bash
ros2 launch rm_bringup bringup_v2.launch.py
```

如果只想先编 SP 自瞄核心：

```bash
colcon build --symlink-install --packages-up-to aim_v2
```
