# 调参文件说明

当前 v2 分支只保留 SP `aim_v2` 主链路。日常调参入口直接放在 `src/rm_bringup/config/` 下。

优先看这几个文件：

```text
src/rm_bringup/config/launch_params_decoupled.yaml
src/rm_bringup/config/video_player_params.yaml
src/rm_bringup/config/virtual_serial_params.yaml
src/rm_bringup/config/camera_info.yaml
src/rm_auto_aim/aim_v2/config/aim_v2.yaml
```

启动：

```bash
ros2 launch rm_bringup bringup_v2.launch.py
```

`config/<robot>/` 目录只保留相机、串口、视频和内参等硬件输入配置，作为根目录没有同名参数文件时的 fallback。

更完整的说明见同目录的 `调参文档.md`。
