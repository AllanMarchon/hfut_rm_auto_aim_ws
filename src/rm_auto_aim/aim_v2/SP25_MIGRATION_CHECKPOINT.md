# SP25 Migration Checkpoint

Last updated: 2026-06-03

## Goal

Use HFUT's existing camera, ROS2 topic/service, and serial hardware chain, but replace/augment the self-aim logic with SP25 algorithms where practical.

Branch: `v2`

## Current Direction

Do not copy SP25 hardware `CBoard` or camera code directly. Keep HFUT hardware links:

- image input: `/image_raw`
- serial feedback: `/serial/receive`
- gimbal output: `/armor_solver/cmd_gimbal`
- serial mode service target name: `gimbal_pipeline/set_mode`

The new node is `aim_v2_node`, launched as node name `gimbal_pipeline` for service compatibility.

## Already Integrated

- Added `src/rm_auto_aim/aim_v2`.
- SP auto-aim core copied into `vendor/vision_core`:
  - traditional detector
  - YOLO detector family
  - classifier
  - solver
  - tracker
  - target model
  - voter
  - aimer
  - shooter
- SP models/assets copied into `aim_v2/assets`.
- `bringup_v2.launch.py` starts camera/serial plus `aim_v2_node`.
- `rm_bringup/package.xml` depends on `aim_v2`.
- `aim_v2_node` subscribes HFUT topics and publishes HFUT `GimbalCmd`.

## Latest In-Progress Changes

These changes are local and not committed yet.

- Added SP async inference path:
  - uses `auto_aim::multithread::MultiThreadDetector`
  - launch arg/parameter: `async_inference`, default `true`
  - automatically disables async for `traditional`
  - automatically disables async for `yolov8` because SP async detector is fixed around 640 input while yolov8 code uses 416
- Added serial safety:
  - parameter `require_serial`, default `true`
  - before any serial packet arrives, node publishes idle instead of aiming from yaw/pitch zero
- Added mode to enemy color switching:
  - parameter `derive_enemy_color_from_mode`, default `true`
  - mode `0` -> enemy red
  - mode `1` -> enemy blue
  - tracker now has `set_enemy_color`
- Added SP `Planner + TinyMPC`:
  - copied to `vendor/vision_core/tasks/auto_aim/planner`
  - `control_backend` launch arg/parameter, default `planner`
  - fallback option: `control_backend:=aimer`
  - `Planner` output is mapped into `GimbalCmd`:
    - yaw/pitch
    - yaw_v/pitch_v
    - yaw_a/pitch_a
    - fire_advice only when `enable_fire:=true`
- Added planner config keys to `aim_v2.yaml`:
  - `fire_thresh`
  - `max_yaw_acc`
  - `Q_yaw`
  - `R_yaw`
  - `max_pitch_acc`
  - `Q_pitch`
  - `R_pitch`
- Changed `infantry_protocol_32.cpp` output bytes:
  - offset 14: `pitch_v`
  - offset 18: `yaw_v`
  - offset 22: `pitch_a`
  - offset 26: `yaw_a`
  - verify with lower machine before real firing.

## Continued On 2026-06-02

- Tightened async inference result matching:
  - async frame context now stores the detector timestamp
  - result thread waits briefly for the matching context instead of blindly popping the first context
  - stale unmatched contexts are dropped
- Reduced async queue risk:
  - `ThreadSafeQueue` now has `full()`
  - `MultiThreadDetector::push()` returns before starting OpenVINO async inference when the queue is already full
- Hardened SP Planner output:
  - `Plan` fields now have safe default values
  - TinyMPC solver pointers are initialized to `nullptr`
  - initial bullet trajectory in `Planner::plan()` now returns no-control when unsolvable instead of using an uninitialized fly time
  - `aim_v2_node` now drops non-finite Planner/Aimer outputs to no-valid before publishing `GimbalCmd`
- Reduced tracker color switching race:
  - `current_enemy_color_` update and `tracker_->set_enemy_color()` are now under `core_mutex_`

## Checks Already Run

- `python -m py_compile src/rm_bringup/launch/bringup_v2.launch.py src/rm_auto_aim/aim_v2/launch/aim_v2.launch.py` passed.
- `git diff --check` passed, only CRLF warnings.
- `aim_v2.yaml` parses with Python YAML.
- Planner copied files had trailing whitespace mechanically cleaned.
- Re-ran Python launch syntax and YAML checks on 2026-06-02; still passed.

## Continued On 2026-06-03

- Added environment/build documentation under `aim_v2`:
  - `readme.md`: detailed environment setup, SP25 vs HFUT ROS2 differences, dependency notes, common build errors.
  - `readme01.md`: simpler "make self-aim run" version; this is the one to give users first.
- SP25 README reference path used for docs:
  - `D:\HFUT苍穹战队视觉组\sp_vision_25-main\readme.md`
- SP25 README says its normal build is plain CMake:
  - `cmake -B build`
  - `make -C build/ -j\`nproc\``
- For HFUT `aim_v2`, do not use SP25 plain CMake flow directly; use ROS2/colcon:
  - `source /opt/ros/humble/setup.bash`
  - `source /opt/intel/openvino_2024.6.0/setupvars.sh`
  - `export OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake`
  - `colcon build --symlink-install --packages-up-to aim_v2`
- Important package-state note:
  - Current v2 keeps only the SP `aim_v2` main chain.
  - `rm_bringup/package.xml` no longer declares the old `rm_auto_aim` aggregate package or `rm_rune`.
- Current untracked docs seen on 2026-06-03:
  - `src/rm_auto_aim/aim_v2/6.02.md`
  - `src/rm_auto_aim/aim_v2/readme.md`
  - `src/rm_auto_aim/aim_v2/readme01.md`
  - Do not blindly `git add .`; choose intentionally.

## Not Verified

- Full ROS2/colcon build was not verified in this Windows PowerShell environment.
- `colcon` is not available here.
- WSL previously failed to start with `HCS_E_SERVICE_NOT_AVAILABLE`.
- Local `g++ -fsyntax-only` could not check C++ because Eigen headers were not available in this shell.
- Default video file paths were missing earlier, so video launch may need a valid video path.

## Important Next Steps

1. Run on Ubuntu/ROS2:
   - first: `colcon build --symlink-install --packages-up-to aim_v2`
   - then, if needed: `colcon build --symlink-install --packages-up-to rm_serial_driver`
   - then build `rm_bringup` for the camera/video + serial + aim_v2 launch chain.
2. If compile fails, first inspect:
   - `planner/tinympc` includes
   - Eigen include path
   - `GimbalCmd` field names
   - OpenVINO linkage
3. Launch with a safe setup:
   - `ros2 launch rm_bringup bringup_v2.launch.py virtual_serial:=true enable_fire:=false`
4. For fallback testing:
   - `control_backend:=aimer`
   - `async_inference:=false`
5. Before real car firing:
   - confirm lower machine expects `infantry_32` acceleration fields at bytes 22/26
   - keep `enable_fire:=false` until aim direction and mode switching are verified.

## Files Most Relevant Next Time

- `src/rm_auto_aim/aim_v2/SP25_MIGRATION_CHECKPOINT.md`
- `src/rm_auto_aim/aim_v2/readme01.md`
- `src/rm_auto_aim/aim_v2/readme.md`
- `src/rm_auto_aim/aim_v2/src/aim_v2_node.cpp`
- `src/rm_auto_aim/aim_v2/CMakeLists.txt`
- `src/rm_auto_aim/aim_v2/config/aim_v2.yaml`
- `src/rm_auto_aim/aim_v2/launch/aim_v2.launch.py`
- `src/rm_bringup/launch/bringup_v2.launch.py`
- `src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol_32.cpp`
- `src/rm_auto_aim/aim_v2/vendor/vision_core/tasks/auto_aim/planner`

## Resume Protocol

When continuing this work in a later session:

1. Read this checkpoint first.
2. Run `git status --short --branch`.
3. Check whether the user wants documentation cleanup, build fixes, or more SP25 migration.
4. If building is requested, prioritize Ubuntu/ROS2 `colcon build` errors over adding new SP25 modules.
5. Keep HFUT hardware links; do not replace them with SP25 `CBoard`/camera main loop unless explicitly requested.
