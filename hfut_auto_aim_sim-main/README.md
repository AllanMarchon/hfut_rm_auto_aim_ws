# Webots 自瞄仿真环境

本项目提供 Webots 下的自瞄仿真场景：一台相机机器人和一台带四块装甲板的可旋转/平移标靶机器人。
`run_stationary_spin_target_test.sh` 和 `run_moving_spin_target_test.sh` 会根据配置渲染可复用的
`worlds/dark_camera_world.wbt`，
并在启动前构建 Webots C++ 控制器。

当前相机机器人控制器 `ros2_image_publisher` 负责把 Webots 数据桥接到 ROS2：

- `/image_raw`，消息类型为 `sensor_msgs/msg/Image`，默认编码为 `bgr8`
- `/camera_info`，消息类型为 `sensor_msgs/msg/CameraInfo`
- `/joint_states`，消息类型为 `sensor_msgs/msg/JointState`
  关节名为 `yaw_joint`、`pitch_joint`
- `/tf`
  - `base_link -> gimbal_link` 动态云台角度
- `/tf_static`
  - `odom -> base_link`
  - `gimbal_link -> camera_link -> camera_optical_link`
  - `camera_optical_link -> camera_optical_frame` 兼容别名
  - `odom -> odom_rectify`
- `SetMode` 服务桥
  定期调用 `armor_detector/set_mode` 和 `gimbal_pipeline/set_mode`，用于在 Webots 运行时替代
  `virtual_serial_node` 的模式切换职责
- `/webots/score`，消息类型为 `std_msgs/msg/String`
  由独立评分控制器发布发弹量、命中数、命中率和 DPS

相机控制器将 Webots 仿真步进和 ROS 话题发布拆到两个线程中运行。Webots 线程负责
`step()`、`getImage()` 和最新 BGRA 图像缓存；ROS 线程负责 `spin_some()`、像素格式转换
和 DDS 发布。这样即使 ROS 发布阻塞，旧帧也只会被覆盖，不会反向卡住 Webots。

`bringup_sim.launch.py` 中建议保持 `virtual_serial:=false`，由 Webots 控制器发布
`/joint_states`、动态 TF、固定 TF 和 `SetMode` 请求。Webots 联调时应关闭
`robot_state_publisher`，避免和控制器重复发布同一批 TF。

## 运行

```bash
cd /root/hfut_auto_aim_sim
./run_stationary_spin_target_test.sh
```

旋转并左右平移的移动靶测试入口：

```bash
cd /root/hfut_auto_aim_sim
./run_moving_spin_target_test.sh
```

在另一个终端中查看基础话题：

```bash
source /opt/ros/humble/setup.bash
source /root/hfut_rm_auto_aim_ws/install/setup.bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
ros2 topic echo /joint_states --once
ros2 topic echo /webots/score
ros2 run tf2_ros tf2_echo base_link gimbal_link
ros2 service list | grep set_mode
```

接入现有自瞄仿真 bringup：

```bash
source /opt/ros/humble/setup.bash
source /root/hfut_rm_auto_aim_ws/install/setup.bash
ros2 launch rm_bringup bringup_sim.launch.py
```
理论上现在只需要去掉自瞄程序中的相机包和串口包即可接入运作
`camera_robot.env` 内的 `WEBOTS_CAMERA_FOLLOW_CMD_GIMBAL` 参数决定云台是否跟随
`/armor_solver/cmd_gimbal`。开启后，Webots 控制器会订阅 `GimbalCmd`，更新
`yaw_joint`、`pitch_joint`，并让 `base_link -> gimbal_link` TF 和 Webots 相机实际视角
跟随该状态。

控制器依赖 `rm_interfaces`，所以首次构建前需要保证
`/root/hfut_rm_auto_aim_ws/install/setup.bash` 存在且其中包含 `rm_interfaces`。

## 配置文件

启动脚本默认按下面顺序加载配置：

- `config/environment.env`
  Webots world、地面、光照、控制器构建等环境配置。
- `config/target_robot.env`
  标靶机器人的初始位姿、装甲板 mesh、灯光和旋转控制参数。
- `config/camera_robot.env`
  本机相机机器人的位姿、相机内参、ROS 话题、TF、`SetMode` 桥和控制周期。

这些配置文件大多使用 `: "${VAR:=default}"` 形式设置默认值，所以启动前在 shell 中
`export VAR=value` 会覆盖文件中的默认值。也可以通过 `WEBOTS_CAMERA_CONFIG` 追加加载一个
自定义配置文件；如果要覆盖前面已经设置过的变量，自定义文件中应直接写 `VAR=value`。

## ROS 接口配置

常用 ROS 侧参数在 `config/camera_robot.env` 中：

```bash
WEBOTS_IMAGE_TOPIC=/image_raw
WEBOTS_CAMERA_INFO_TOPIC=/camera_info
WEBOTS_JOINT_STATES_TOPIC=/joint_states
WEBOTS_CAMERA_FRAME_ID=camera_optical_frame
WEBOTS_CAMERA_NAME=camera
WEBOTS_IMAGE_ENCODING=bgr8

WEBOTS_PUBLISH_JOINT_STATES=true
WEBOTS_PUBLISH_TF=true
WEBOTS_PUBLISH_ODOM_RECTIFY_TF=true
WEBOTS_PUBLISH_CAMERA_FRAME_ALIAS_TF=true
WEBOTS_TARGET_FRAME_ID=odom
WEBOTS_BASE_FRAME_ID=base_link
WEBOTS_GIMBAL_FRAME_ID=gimbal_link
WEBOTS_CAMERA_LINK_FRAME_ID=camera_link
WEBOTS_CAMERA_OPTICAL_LINK_FRAME_ID=camera_optical_link
WEBOTS_CAMERA_TF_X=0.12514
WEBOTS_CAMERA_TF_Y=0
WEBOTS_CAMERA_TF_Z=0.0505
WEBOTS_GIMBAL_CMD_TOPIC=/armor_solver/cmd_gimbal
WEBOTS_CAMERA_FOLLOW_CMD_GIMBAL=true
WEBOTS_GIMBAL_USE_DIFF_COMMANDS=false
WEBOTS_GIMBAL_MAX_YAW_RATE=20.0
WEBOTS_GIMBAL_MAX_PITCH_RATE=20.0
WEBOTS_GIMBAL_ACCEPT_UNKNOWN_MODE=true
WEBOTS_GIMBAL_WEBOTS_YAW_SIGN=1.0
WEBOTS_GIMBAL_WEBOTS_PITCH_SIGN=-1.0

WEBOTS_ENABLE_SET_MODE_CLIENTS=true
WEBOTS_SET_MODE_SERVICES="armor_detector/set_mode gimbal_pipeline/set_mode"
WEBOTS_SERIAL_MODE=0
WEBOTS_GIMBAL_YAW_RESPONSE_DELAY_MS=0.0
WEBOTS_GIMBAL_PITCH_RESPONSE_DELAY_MS=0.0
WEBOTS_FIRE_DELAY_MS=0.0
WEBOTS_FIRE_RATE_HZ=20.0
WEBOTS_BULLET_SPEED=22.5
WEBOTS_SCORE_ENABLED=true
WEBOTS_SCORE_TOPIC=/webots/score
WEBOTS_SCORE_CONTROLLER_STEP_MS=32
WEBOTS_SCORE_PUBLISH_PERIOD_MS=200
WEBOTS_SCORE_ARMOR_WIDTH=0.135
WEBOTS_SCORE_ARMOR_HEIGHT=0.135
WEBOTS_SCORE_MAX_FLIGHT_TIME=2.0
WEBOTS_SCORE_GRAVITY=9.80665
WEBOTS_SCORE_SHOOTER_FORWARD_X=1.0
WEBOTS_SCORE_SHOOTER_FORWARD_Y=0.0
WEBOTS_SCORE_SHOOTER_FORWARD_Z=0.0
WEBOTS_SET_MODE_PERIOD_MS=500
```

`WEBOTS_SERIAL_MODE=0` 表示自瞄红，`1` 表示自瞄蓝。`armor_detector_nn` 启动后默认处于
DISABLED 状态；如果没有 `SetMode` 请求，它会收到图像但不执行检测。

拟真参数中，`WEBOTS_GIMBAL_YAW_RESPONSE_DELAY_MS` 和
`WEBOTS_GIMBAL_PITCH_RESPONSE_DELAY_MS` 表示 `cmd_gimbal` 到云台 yaw/pitch 电机响应之间的固定延迟；
`WEBOTS_FIRE_DELAY_MS` 表示允许开火到弹丸发射的固定延迟；
`WEBOTS_FIRE_RATE_HZ` 表示弹丸发射频率；`WEBOTS_BULLET_SPEED` 表示弹丸初速度，单位 m/s。
所有延迟都可以设为 `0`，`WEBOTS_FIRE_RATE_HZ=0` 表示不限制发射频率。
云台响应延迟只作用在控制器内部的电机执行队列上，不会延迟或阻塞 `cmd_gimbal` 话题收发。

评分系统由独立的 Webots Supervisor 控制器 `score_system` 运行，不放在相机主控制循环中。
它订阅 `WEBOTS_GIMBAL_CMD_TOPIC`，每个评分周期只检查当前时刻的 `fire_advice` 和发射 CD；
只有当 `fire_advice=1` 且 CD 已结束时，才登记一次已经允许的发射。
登记后等待 `WEBOTS_FIRE_DELAY_MS`，弹丸会必定射出；如果 CD 未结束，则不会缓存本次 `fire_advice`
等待后续补发。弹丸射出时按当前 Webots 相机/云台真实朝向生成，并用
`WEBOTS_BULLET_SPEED` 和重力做斜抛运动判定。命中只统计装甲板朝外面，且装甲板法线和相机方向夹角
不超过 `75 deg` 的目标面。DPS 按每命中一发 `20` 伤害计算，靶车视为无限血量。
连续允许开火时，发射 CD 按 `WEBOTS_FIRE_RATE_HZ` 的精确间隔结算，避免被评分控制器步长向后量化。
`/webots/score` 中的 `misses` 表示超过最大飞行时间仍未命中的弹丸数；
`hit_rate` 按 `hits / (hits + misses)` 计算，不把正在等待发射或正在飞行的弹丸计入分母；
`fired=1` 表示当前评分周期登记了一次发射。
默认 `WEBOTS_SCORE_SHOOTER_FORWARD_*=(1,0,0)`，对齐当前 Webots 相机模型中 `SIM_CAMERA_PITCH` 的本地看向方向。

默认 `WEBOTS_GIMBAL_USE_DIFF_COMMANDS=false`，控制器使用 `GimbalCmd.yaw` 和
`GimbalCmd.pitch` 作为绝对目标角，单位按自瞄消息定义为度，内部转换为弧度发布到
`/joint_states` 和 TF。如果设为 `true`，则使用 `yaw_diff` 和 `pitch_diff` 作为相对当前云台
状态的增量命令。`WEBOTS_GIMBAL_MAX_YAW_RATE` 和 `WEBOTS_GIMBAL_MAX_PITCH_RATE`
用于限制仿真云台每秒最大角速度，单位为 rad/s。

`WEBOTS_GIMBAL_WEBOTS_YAW_SIGN` 和 `WEBOTS_GIMBAL_WEBOTS_PITCH_SIGN` 只影响 Webots 相机实体的
运动方向，不改变 `/joint_states` 和 TF 的 ROS 语义。默认 pitch 为 `-1.0`，对齐
`virtual_serial_node` 中 `q.setRPY(roll, -pitch, yaw)` 的约定。

`WEBOTS_IMAGE_ENCODING` 可选 `bgr8`、`rgb8` 或 `bgra8`。为了和
`hfut_rm_auto_aim_ws` 中的检测节点保持一致，默认使用 `bgr8`。

## 相机与场景配置

启动前优先编辑三类配置文件：

- 场景和运行环境：`config/environment.env`
- 标靶机器人：`config/target_robot.env`
- 本机相机机器人：`config/camera_robot.env`

常用场景参数和 Webots 相机参数如下：

```bash
WEBOTS_ARMOR_TARGET_X=3
WEBOTS_ARMOR_TARGET_Y=0
WEBOTS_ARMOR_TARGET_Z=0.165
WEBOTS_ARMOR_TARGET_YAW=0
WEBOTS_TARGET_LATERAL_ENABLED=false
WEBOTS_TARGET_LATERAL_AXIS=y
WEBOTS_TARGET_LATERAL_AMPLITUDE=1.5
WEBOTS_TARGET_LATERAL_SPEED=1.0
WEBOTS_TARGET_LATERAL_ACCEL=3.0
WEBOTS_TARGET_LATERAL_DECEL=3.0
WEBOTS_TARGET_LATERAL_INITIAL_DIRECTION=1.0
WEBOTS_SKY_COLOR="0.012 0.013 0.016"
WEBOTS_GROUND_COLOR="0.24 0.25 0.24"
WEBOTS_LIGHT_INTENSITY=1.045
WEBOTS_LIGHT_AMBIENT_INTENSITY=0.55
WEBOTS_LIGHT_DIRECTION="0.25 0.35 -1"
WEBOTS_ARMOR_BODY_MESH_URL=../meshes/armor_red4_body_plate_lights.obj
WEBOTS_ARMOR_MESH_SCALE="0.001 0.001 0.001"
WEBOTS_ARMOR_MESH_OFFSET="-0.0018354 -0.13427105 -0.01999975"

WEBOTS_CAMERA_X=0
WEBOTS_CAMERA_Y=0
WEBOTS_CAMERA_Z=0.405
WEBOTS_CAMERA_LOOK_AT_TARGET=true
WEBOTS_CAMERA_LOOK_AT_Z=
WEBOTS_CAMERA_LOOK_AT_Z_OFFSET=0.055
WEBOTS_CAMERA_YAW=0
WEBOTS_CAMERA_TILT=0.06158867632
WEBOTS_CAMERA_WIDTH=1440
WEBOTS_CAMERA_HEIGHT=1080
WEBOTS_CAMERA_FOV=0.7850335620966933
WEBOTS_CAMERA_FX=1739.130435
WEBOTS_CAMERA_FY=1739.130435
WEBOTS_CAMERA_CX=719.5
WEBOTS_CAMERA_CY=539.5
WEBOTS_CAMERA_NEAR=0.05
WEBOTS_CAMERA_FAR=30
```

`WEBOTS_ARMOR_TARGET_Z` 表示目标 `base_link` 的高度。目标控制器会让机器人绕自身中心的
`+Z` 轴旋转，角速度由 `WEBOTS_TARGET_SPIN_RATE` 控制，默认值是 `3.0 rad/s`。
`run_moving_spin_target_test.sh` 会额外开启 `WEBOTS_TARGET_LATERAL_ENABLED=true`，让目标以初始
位置为中心沿 `WEBOTS_TARGET_LATERAL_AXIS` 往返平移；默认沿 `y` 轴移动 `±1.5 m`，最大速度
`1 m/s`，最大加速度 `3 m/s^2`，最大减速度 `3 m/s^2`。目标会在端点前按减速度降到
`0 m/s` 后再反向。

`WEBOTS_CAMERA_LOOK_AT_TARGET=true` 时，脚本会在渲染 world 前根据相机位置和标靶位置自动计算
`WEBOTS_CAMERA_YAW` 和 `WEBOTS_CAMERA_TILT`。如果 `WEBOTS_CAMERA_LOOK_AT_Z` 为空，则使用
`WEBOTS_ARMOR_TARGET_Z + WEBOTS_CAMERA_LOOK_AT_Z_OFFSET` 作为瞄准高度。

性能调优时，`WEBOTS_BASIC_TIME_STEP` 控制物理步长，`WEBOTS_CONTROLLER_STEP_MS` 和
`WEBOTS_TARGET_CONTROLLER_STEP_MS` 控制相机控制器和目标控制器的唤醒频率。默认相机周期是
`32 ms`，这是为了在 `1440x1080` 分辨率下尽量稳定发布图像；只有当你确实需要接近
`60 FPS` 且机器性能足够时，才建议把 `WEBOTS_CAMERA_PERIOD_MS` 和
`WEBOTS_CONTROLLER_STEP_MS` 都设为 `16`。

`WEBOTS_SKIP_UNSUBSCRIBED_IMAGES=true` 会让 Webots 相机在没有图像订阅者时保持关闭状态。
这样在只观察 Webots 自身运行速度时，可以避免 GPU 读回和 DDS 发布的额外开销。
如果希望即使没有订阅者也持续运行相机链路，就把它设为 `false`。

`WEBOTS_PROFILE_PERIOD=120` 会周期性打印 `step`、`get_image`、`copy`、`convert`、
`publish`、`spin` 的平均耗时，便于判断 FPS 卡在哪个阶段。

## 相机内参

`WEBOTS_CAMERA_FOV` 是 Webots 使用的水平视场角，单位是弧度。如果已经有目标焦距 `fx`，
可以按下面的公式换算：

```text
WEBOTS_CAMERA_FOV = 2 * atan(width / (2 * fx))
```

下面是适用于 Hikrobot MV-CS016-10UC + 6 mm C-Mount 的 ROS `CameraInfo` 内参覆盖示例：

```bash
WEBOTS_CAMERA_FX=1739.130435
WEBOTS_CAMERA_FY=1739.130435
WEBOTS_CAMERA_CX=719.5
WEBOTS_CAMERA_CY=539.5
WEBOTS_CAMERA_DISTORTION_MODEL=plumb_bob
WEBOTS_CAMERA_D="0 0 0 0 0"
```

如果不显式设置这些参数，控制器会根据当前 Webots 图像宽高和视场角自动推导 `fx`、`fy`、
`cx`、`cy`。

## 常见检查

图像链路：

```bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

自瞄状态反馈和 TF：

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link gimbal_link
ros2 run tf2_ros tf2_echo odom camera_optical_link
ros2 run tf2_ros tf2_echo odom camera_optical_frame
```

检测节点是否被启用：

```bash
ros2 service list | grep set_mode
ros2 topic echo /armor_detector/armors
```
