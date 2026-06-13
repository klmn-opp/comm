# comm

ROS 2 + MAVLink target geolocation bridge.

目标：

- 从 ArduPilot SITL/飞控读取飞机状态
- 订阅视觉节点输出的 `/target_pose_camera`
- 结合相机外参和飞机姿态，估计目标经纬度

## 环境

不要使用 Conda 环境安装本工程依赖。推荐：

```bash
cd /home/klmn/cudac/comm
source /opt/ros/humble/setup.bash
/usr/bin/python3.10 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`--system-site-packages` 用于复用 ROS 2 Humble 的 `rclpy`、`geometry_msgs`、`sensor_msgs`，不会修改系统环境。

## 构建

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch target_geo_comm target_geo.launch.py \
  mavlink_connection:=udp:127.0.0.1:14550 \
  target_pose_topic:=/target_pose_camera
```

## 输出

- `/aircraft_state`: `Float64MultiArray`
  - `[lat_deg, lon_deg, alt_msl_m, rel_alt_m, roll_rad, pitch_rad, yaw_rad, vx_mps, vy_mps, vz_mps, airspeed_mps, groundspeed_mps, heading_deg]`
- `/target_local_ned`: `PointStamped`
  - 目标相对飞机当前位置的 NED 偏移，单位米
  - `x=north, y=east, z=down`
- `/target_geo`: `NavSatFix`
  - 估计出的目标经纬度和高度
- `/target_geo_raw`: `NavSatFix`
  - 未启用仿真拉远前的原始目标经纬度，用于和 `/target_geo` 对比

## 外参参数

第一版先估计外参，后续实测调整：

- `camera_roll_deg`
- `camera_pitch_deg`
- `camera_yaw_deg`
- `camera_x_m`
- `camera_y_m`
- `camera_z_m`
- `target_altitude_mode`: 默认 `pnp`
  - `pnp`: 使用视觉 PnP 的三维位置估计目标高度
  - `ground_msl_offset`: 目标高度固定为 `target_altitude_offset_m`
  - `aircraft_msl_plus_offset`: 目标高度固定为飞机当前 MSL 高度加偏移

## 目标经纬度仿真

默认不启用。模拟阶段如果靶标就在摄像头前方，导致 `/target_geo` 几乎在飞机正下方，可以启用目标拉远：

```bash
ros2 launch target_geo_comm target_geo.launch.py \
  use_sim_target_geo:=true \
  sim_target_distance_m:=200.0 \
  sim_target_bearing_offset_deg:=15.0
```

- `use_sim_target_geo`: 是否发布拉远后的 `/target_geo`，默认 `false`
- `sim_target_distance_m`: 拉远后的水平距离，单位米，默认 `80.0`
- `sim_min_direction_distance_m`: 原始目标水平距离小于该值时，使用飞机航向作为拉远方向，默认 `3.0`
- `sim_target_bearing_offset_deg`: 在选定方向上额外旋转的方位角，正值向右偏，负值向左偏，默认 `0.0`
- `publish_raw_target_geo`: 是否发布 `/target_geo_raw`，默认 `true`

如果只增大 `sim_target_distance_m`，目标仍会沿原方向拉远；原方向左右分量很小时，左右偏移也会很小。需要明显左右偏移时设置 `sim_target_bearing_offset_deg`，例如 200 米距离配 15 度偏角，横向偏移约 52 米。

启用后，`/target_geo_raw` 保留真实视觉换算结果，`/target_geo` 输出拉远后的模拟航点，下游航点发送节点无需修改。

机体系约定：

- `x`: 机头前
- `y`: 机体右
- `z`: 机体下

OpenCV 相机系约定：

- `x`: 图像右
- `y`: 图像下
- `z`: 相机前

如果相机竖直朝下，初始可尝试：

```bash
camera_roll_deg:=0 camera_pitch_deg:=90 camera_yaw_deg:=0
```

如果方向不对，优先调 `camera_pitch_deg` 和 `camera_yaw_deg`。
