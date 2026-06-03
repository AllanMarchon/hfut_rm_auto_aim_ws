# 调参文件说明

`bringup_v2` 日常调参入口现在直接放在 `src/rm_bringup/config/` 下。

优先看这几个文件：

```text
src/rm_bringup/config/launch_params_decoupled.yaml
src/rm_bringup/config/video_player_params.yaml
src/rm_bringup/config/virtual_serial_params.yaml
src/rm_bringup/config/camera_info.yaml
src/rm_auto_aim/aim_v2/config/aim_v2.yaml
```

`config/video_test/` 目录继续保留默认/兼容版。`bringup_v2` 读取节点参数时会先找 `config/*_params.yaml`，找不到再回退到 `config/<robot>/*_params.yaml`，最后回退到 `config/uav/*_params.yaml`。

更完整的说明见同目录的 `调参文档.md`。
