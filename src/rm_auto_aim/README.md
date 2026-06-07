# rm_auto_aim

当前 v2 分支只保留 `aim_v2`。

`aim_v2` 是 SP 自瞄核心接入 HFUT ROS2 硬件链路后的包：

```text
camera/video -> /image_raw
serial/virtual_serial -> /serial/receive
aim_v2_node -> /armor_solver/cmd_gimbal
```

启动入口在：

```text
src/rm_bringup/launch/bringup_v2.launch.py
```
