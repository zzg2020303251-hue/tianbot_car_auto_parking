# tianbot_mini_parking

这是一个基于 ROS1 Noetic 的 `tianbot_mini` 自动泊车项目。项目把 notebook 中的自动泊车思路整理成可以在 Gazebo 仿真和真实小车上运行的 ROS 包。

核心功能包括：

- 使用 Gazebo 构建停车场仿真环境。
- 使用 SLAM 建图，并保存停车场地图。
- 使用固定地图和 AMCL 完成真实小车定位。
- 使用 RViz 的 `2D Nav Goal` 指定停车目标点。
- 使用 Hybrid A* 规划泊车路径。
- 使用路径跟踪节点发布 `/tianbot_mini/cmd_vel` 控制小车运动。
- 对规划路径进行 shortcut、重采样、平滑和障碍物距离检查。

## 目录结构

```text
tianbot_mini_parking/
├── launch/
│   ├── parking_demo.launch          # 只启动泊车规划器和跟踪器
│   ├── parking_gazebo_demo.launch   # Gazebo 仿真泊车
│   ├── parking_mapping.launch       # 真实小车 SLAM 建图
│   └── parking_real_map.launch      # 真实小车固定地图导航
├── maps/
│   ├── parking_map.yaml             # 保存后的地图配置
│   └── parking_map.pgm              # 保存后的地图图片
├── rviz/
│   └── parking_demo.rviz            # RViz 显示配置
├── scripts/
│   ├── hybrid_parking_planner_node.py
│   └── parking_path_follower_node.py
└── worlds/
    └── parking_lot.world            # Gazebo 停车场世界
```

## 编译

```bash
cd /home/tianbot/ros1_project_leader_follower
catkin_make
source devel/setup.bash
```

## Gazebo 仿真

启动 Gazebo 停车场仿真：

```bash
roslaunch tianbot_mini_parking parking_gazebo_demo.launch
```

该命令会启动：

- Gazebo 停车场环境
- `tianbot_mini` 小车模型
- SLAM 建图
- Hybrid A* 泊车规划器
- 路径跟踪器
- RViz

## 真实小车建图

启动建图：

```bash
cd /home/tianbot/ros1_project_leader_follower
source devel/setup.bash

roslaunch tianbot_mini_parking parking_mapping.launch lidar_port:=/dev/ttyUSB0
```

如果雷达在 `/dev/ttyUSB1`，改成：

```bash
roslaunch tianbot_mini_parking parking_mapping.launch lidar_port:=/dev/ttyUSB1
```

建图时建议：

- 小车速度尽量慢。
- 转弯不要太急。
- 多扫墙体和障碍物边界。
- 尽量让地图闭合。
- 建图完成后不要随意移动环境中的障碍物。

保存地图：

```bash
rosrun map_server map_saver -f /home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/maps/parking_map map:=/tianbot_mini/map
```

保存后会生成：

```text
src/tianbot_mini_parking/maps/parking_map.yaml
src/tianbot_mini_parking/maps/parking_map.pgm
```

## 真实小车固定地图导航

推荐先使用较保守的低速参数：

```bash
cd /home/tianbot/ros1_project_leader_follower
source devel/setup.bash

roslaunch tianbot_mini_parking parking_real_map.launch \
  map:=/home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/maps/parking_map.yaml \
  lidar_port:=/dev/ttyUSB0 \
  obstacle_inflation:=0.03 \
  max_linear_speed:=0.02 \
  enable_path_smoothing:=true \
  enable_obstacle_clearance:=true \
  min_obstacle_clearance:=0.05 \
  obstacle_clearance_weight:=0.25 \
  max_clearance_push:=0.02 \
  adjust_colliding_goal:=true
```

如果雷达端口是 `/dev/ttyUSB1`，把命令中的：

```text
lidar_port:=/dev/ttyUSB0
```

改成：

```text
lidar_port:=/dev/ttyUSB1
```

## RViz 操作流程

启动后在 RViz 中按顺序操作：

1. 使用 `2D Pose Estimate` 设置小车初始位置和朝向。
2. 观察雷达点云是否和地图墙体、障碍物重合。
3. 如果不重合，重新设置 `2D Pose Estimate`。
4. 雷达点云和地图基本重合后，再使用 `2D Nav Goal` 设置停车目标。
5. 观察是否出现 `/parking_path` 路径。
6. 确认路径合理后，小车会低速跟踪路径。

## 关键话题

```text
/tianbot_mini/scan          # 雷达数据
/tianbot_mini/odom          # 里程计
/tianbot_mini/amcl_pose     # AMCL 定位结果
/move_base_simple/goal      # RViz 2D Nav Goal
/parking_path               # Hybrid A* 规划结果
/tianbot_mini/cmd_vel       # 小车速度控制
```

## 常用检查命令

检查雷达是否稳定：

```bash
rostopic hz /tianbot_mini/scan
```

正常应稳定在约 7 到 8 Hz。若没有数据，先不要运行导航。

检查 AMCL 是否更新：

```bash
rostopic hz /tianbot_mini/amcl_pose
rostopic echo -n 1 /tianbot_mini/amcl_pose
```

检查路径是否发布：

```bash
rostopic echo -n 1 /parking_path/header
```

检查速度命令：

```bash
rostopic echo /tianbot_mini/cmd_vel
```

检查雷达端口：

```bash
ls -l /dev/ttyUSB*
udevadm info -q property -n /dev/ttyUSB0 | grep -E "ID_MODEL|ID_VENDOR|ID_SERIAL|ID_PATH"
udevadm info -q property -n /dev/ttyUSB1 | grep -E "ID_MODEL|ID_VENDOR|ID_SERIAL|ID_PATH"
```

## 参数说明

### 规划安全距离

```text
obstacle_inflation
```

障碍物膨胀半径，单位是米。该参数太大会导致窄路或起点被判定为碰撞。

建议从小到大测试：

```text
0.02 -> 0.03 -> 0.04 -> 0.05
```

在当前真实小车和地图条件下，不建议一开始使用 `0.10`，因为它可能直接让起点或岔口进入碰撞区。

### 路径平滑

```text
enable_path_smoothing
```

是否启用路径平滑。启用后会对 Hybrid A* 的原始路径做 shortcut、重采样和平滑。

### 障碍物距离约束

```text
enable_obstacle_clearance
min_obstacle_clearance
obstacle_clearance_weight
max_clearance_push
```

这些参数用于让平滑后的路径尽量远离墙体和障碍物。若通道较窄，参数过大可能导致规划困难。

推荐起始值：

```text
min_obstacle_clearance:=0.05
obstacle_clearance_weight:=0.25
max_clearance_push:=0.02
```

### 目标点微调

```text
adjust_colliding_goal
```

如果 RViz 中点选的目标落在地图障碍物、未知区或膨胀区内，规划器会在附近搜索一个可行目标点，避免直接失败。

## AMCL 定位

真实小车固定地图导航使用 `parking_real_map.launch` 中的 AMCL 参数。当前配置偏向小场地和低速运行：

```xml
<param name="odom_alpha1" value="0.4"/>
<param name="odom_alpha2" value="0.4"/>
<param name="odom_alpha3" value="0.4"/>
<param name="odom_alpha4" value="0.4"/>
<param name="laser_max_beams" value="120"/>
<param name="min_particles" value="1000"/>
<param name="max_particles" value="3000"/>
<param name="update_min_d" value="0.01"/>
<param name="update_min_a" value="0.02"/>
<param name="transform_tolerance" value="0.5"/>
```

如果雷达点云和地图越走越偏，优先检查：

- `/tianbot_mini/scan` 是否稳定。
- 初始位姿是否设置正确。
- 地图是否和当前环境一致。
- 地面是否打滑或不平。
- 小车速度是否过快。

## 常见问题

### 只看到黄色箭头，没有规划路径

黄色箭头通常只是 RViz 的目标姿态标记，不代表规划成功。应查看终端是否出现：

```text
Published parking path with XX poses
Parking follower received path with XX poses
```

如果出现：

```text
Parking start pose is in collision
Published parking path with 1 poses
```

说明起点被判定为碰撞。可以尝试降低：

```text
obstacle_inflation
min_obstacle_clearance
```

### 雷达有时能打开，有时不能打开

可能原因：

- 雷达 USB 接触不稳定。
- 小车电量不足。
- Linux 的 `ModemManager` 抢占串口。
- 雷达异常退出后没有复位。
- `/dev/ttyUSB0` 和 `/dev/ttyUSB1` 顺序变化。

实验前可以执行：

```bash
sudo systemctl stop ModemManager
```

若要永久禁用，需要在终端执行：

```bash
sudo systemctl disable ModemManager
sudo systemctl mask ModemManager
```

### RViz 崩溃

如果出现：

```text
Duration has to be finite
```

通常和雷达或 TF 时间戳异常有关。可以先关闭自动启动 RViz：

```bash
use_rviz:=false
```

再单独启动 RViz：

```bash
rosrun rviz rviz -d /home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/rviz/parking_demo.rviz
```

## 运行视频

[Watch the parking demo](https://github.com/zzg2020303251-hue/tianbot_car_auto_parking/releases/tag/demo-v1)
## 安全建议

- 第一次测试时建议架空小车轮子。
- 地面测试时保持低速，例如 `max_linear_speed:=0.02`。
- 人要随时准备急停。
- 不要在雷达数据不稳定时运行导航。
- 不要在 AMCL 位姿和地图明显不匹配时运行导航。
