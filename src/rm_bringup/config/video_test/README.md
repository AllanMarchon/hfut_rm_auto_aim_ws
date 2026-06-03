# video_test 兼容配置

普通 `bringup_v2` 离线调试现在优先改 `config` 根目录下的这几个文件：

```text
src/rm_bringup/config/launch_params_decoupled.yaml
src/rm_bringup/config/video_player_params.yaml
src/rm_bringup/config/virtual_serial_params.yaml
src/rm_bringup/config/camera_info.yaml
```

其中：

- `launch_params_decoupled.yaml`：启动模式、图像源、串口模式
- `video_player_params.yaml`：本地视频路径、播放帧率、视频内参引用
- `virtual_serial_params.yaml`：虚拟下位机姿态和 `vision_mode`
- `camera_info.yaml`：视频源内参
- `aim_v2` 参数

本目录里的 YAML 保留为默认/兼容版。`bringup_v2` 的读取顺序是：

```text
config/*_params.yaml
config/<robot>/*_params.yaml
config/uav/*_params.yaml
```

所以根目录存在同名调参文件时，会优先使用根目录版本；根目录没有同名文件时，才会回退到本目录或对应车种目录。

- `video_player_params.yaml`：兼容用的视频源参数。
- `virtual_serial_params.yaml`：兼容用的虚拟下位机参数。
- `camera_info.yaml`：兼容用的视频内参。

常用启动命令：

```bash
ros2 launch rm_bringup bringup_v2.launch.py
```

检查话题：

```bash
ros2 topic echo /serial/receive --once
ros2 topic hz /image_raw
ros2 topic echo /armor_solver/cmd_gimbal
```
