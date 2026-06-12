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
