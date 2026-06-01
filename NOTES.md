# Tianbot Mini 自动泊车项目交接记录

本文档用于在更换 GPT 账号或重新开启会话后，快速恢复项目上下文。

## 1. 项目目标

本项目目标是基于真实 `tianbot_mini` 小车实现一个自动泊车实验流程：

1. 使用 SLAM 扫描停车场/障碍物环境。
2. 保存地图。
3. 使用固定地图和 AMCL 定位真实小车。
4. 在 RViz 中使用 `2D Nav Goal` 给定停车目标点。
5. 使用 Hybrid A* 规划泊车路径。
6. 使用路径跟踪控制器控制小车沿路径运动。

项目路径：

```text
/home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking
```

主要运行环境：

```text
ROS1 Noetic
ROS2GO
tianbot_mini
```

## 2. 主要文件

```text
tianbot_mini_parking/
├── launch/
│   ├── parking_demo.launch
│   ├── parking_gazebo_demo.launch
│   ├── parking_mapping.launch
│   └── parking_real_map.launch
├── maps/
│   ├── parking_map.yaml
│   └── parking_map.pgm
├── rviz/
│   └── parking_demo.rviz
├── scripts/
│   ├── hybrid_parking_planner_node.py
│   ├── parking_path_follower_node.py
│   └── parking_lqr_follower_node.py
├── worlds/
│   └── parking_lot.world
├── README.md
└── NOTES.md
```

## 3. 当前代码结构

### 3.1 `hybrid_parking_planner_node.py`

这是路径规划节点。

功能：

- 订阅地图 `/tianbot_mini/map`
- 订阅 AMCL 位姿 `/tianbot_mini/amcl_pose`
- 订阅 RViz 目标 `/move_base_simple/goal`
- 使用 Hybrid A* 规划泊车路径
- 发布路径 `/parking_path`

已经加入的功能：

- 起点碰撞时不立刻失败，而是忽略第一采样点继续尝试。
- 支持原地旋转扩展，适配差速小车。
- 路径后处理：
  - shortcut
  - 重采样
  - 平滑
  - 障碍物距离场
  - 路径远离障碍物的轻量推离
- 如果优化路径碰撞，则自动退回原始路径。
- 收到新目标时会先发布空路径，清除旧的 latch 路径。
- 如果目标点在碰撞区，会在目标附近搜索一个可行目标点。

注意：

- 如果日志出现 `Published parking path with 1 poses`，通常说明规划失败，只返回了一个起点附近的 partial path。
- 如果日志出现 `Parking start pose is in collision`，说明当前 AMCL 位姿落在障碍物或膨胀区内。

### 3.2 `parking_path_follower_node.py`

这是原始路径跟踪节点，使用简化 Pure Pursuit 控制。

功能：

- 订阅 `/parking_path`
- 通过 TF 获取小车位姿
- 发布 `/tianbot_mini/cmd_vel`
- 根据 `/tianbot_mini/scan` 做前后方安全停车
- 收到空路径时会停车并清空旧路径

控制方式：

```text
Pure Pursuit + 角度比例控制 + 前后雷达安全停车
```

### 3.3 `parking_lqr_follower_node.py`

这是新增的 LQR 路径跟踪节点。

功能和原 follower 类似，但控制核心改成 LQR。

LQR 状态：

```text
x = [横向误差, 航向误差]
```

控制量：

```text
u = angular.z
```

线速度仍然使用低速给定，并保留：

- 前进/倒车判断
- 终点姿态修正
- 前后雷达安全停车

当前 `parking_demo.launch` 默认调用 LQR：

```text
follower_type:=lqr
```

如果要切回原 Pure Pursuit：

```text
follower_type:=pure_pursuit
```

## 4. 当前推荐运行命令

### 4.1 编译

```bash
cd /home/tianbot/ros1_project_leader_follower
catkin_make
source devel/setup.bash
```

### 4.2 建图

```bash
cd /home/tianbot/ros1_project_leader_follower
source devel/setup.bash

roslaunch tianbot_mini_parking parking_mapping.launch lidar_port:=/dev/ttyUSB0
```

如果雷达在 `/dev/ttyUSB1`：

```bash
roslaunch tianbot_mini_parking parking_mapping.launch lidar_port:=/dev/ttyUSB1
```

保存地图：

```bash
rosrun map_server map_saver -f /home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/maps/parking_map map:=/tianbot_mini/map
```

### 4.3 固定地图导航，LQR 控制器

当前推荐低速、较宽松的参数：

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

如果雷达在 `/dev/ttyUSB1`，把：

```text
lidar_port:=/dev/ttyUSB0
```

改成：

```text
lidar_port:=/dev/ttyUSB1
```

### 4.4 使用原 Pure Pursuit 控制器

```bash
roslaunch tianbot_mini_parking parking_real_map.launch \
  map:=/home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/maps/parking_map.yaml \
  lidar_port:=/dev/ttyUSB0 \
  follower_type:=pure_pursuit \
  obstacle_inflation:=0.03 \
  max_linear_speed:=0.02
```

## 5. 当前重要参数

### 5.1 碰撞膨胀

```text
obstacle_inflation
```

不要一开始设太大。

实际验证过：

- 在某次起点 `start=(0.33, -1.32, 42.4 deg)` 附近：
  - `0.03` 不碰撞
  - `0.05` 开始碰撞
  - `0.10` 一定碰撞

因此当前建议：

```text
obstacle_inflation:=0.02 到 0.04
```

不要优先用 `0.10`，否则窄口和起点容易直接被判死。

### 5.2 路径安全距离

```text
min_obstacle_clearance
obstacle_clearance_weight
max_clearance_push
```

推荐起始值：

```text
min_obstacle_clearance:=0.05
obstacle_clearance_weight:=0.25
max_clearance_push:=0.02
```

如果太保守，会导致规划失败。

### 5.3 LQR 参数

```text
lqr_q_lateral
lqr_q_heading
lqr_r_angular
```

默认值：

```text
lqr_q_lateral:=8.0
lqr_q_heading:=3.0
lqr_r_angular:=1.2
```

如果转向太猛：

```text
lqr_r_angular:=2.0
```

如果跟踪不积极：

```text
lqr_q_lateral:=12.0
```

## 6. AMCL 当前配置

AMCL 参数在：

```text
launch/parking_real_map.launch
```

当前偏向小场地、低速定位：

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

目的：

- 更频繁用雷达更新位姿。
- 更不相信 odom。
- 使用更多粒子和更多雷达束。

## 7. 当前已知问题

### 7.1 雷达不稳定

曾经多次出现：

```text
No laser scan received
timout count
Failed to start scan mode
Received message with unrecognized topicId
```

判断：

- 这不是路径规划代码问题。
- 很可能和雷达串口、USB 线、供电、ModemManager 抢串口有关。

每次实验前建议：

```bash
sudo systemctl stop ModemManager
```

如需永久禁用：

```bash
sudo systemctl disable ModemManager
sudo systemctl mask ModemManager
```

但这两条需要 sudo 权限。

检查雷达：

```bash
rostopic hz /tianbot_mini/scan
```

正常应稳定在约 7 到 8 Hz。

### 7.2 AMCL 越走越漂

现象：

- 初始能对上。
- 小车走一段后雷达点云和地图偏离。

可能原因：

- 地面不平。
- 轮子打滑。
- 雷达数据不稳定。
- 地图质量不够好。
- 当前环境和建图时不一致。

建议：

- 降低速度到 `max_linear_speed:=0.02`。
- 重新建更清晰的地图。
- 确认 `/tianbot_mini/scan` 稳定。
- 使用 RViz 的 `2D Pose Estimate` 先把雷达点云和地图对齐，再点目标。

### 7.3 规划器发布 1 个点

现象：

```text
Published parking path with 1 poses
Parking goal reached
```

含义：

```text
规划器没有找到有效路径，只返回了 partial path。
```

常见原因：

- 起点被判 collision。
- 目标点在 collision 区域。
- `obstacle_inflation` 太大。
- 地图中窄口被膨胀封死。

解决：

```text
obstacle_inflation:=0.02 或 0.03
enable_obstacle_clearance:=false 做对照测试
先点空旷区域目标，不要直接点车位最里面
```

### 7.4 RViz 崩溃

曾经出现：

```text
Duration has to be finite
```

可能和雷达/TF 时间戳异常有关。

可以先不自动启动 RViz：

```bash
use_rviz:=false
```

然后单独启动：

```bash
rosrun rviz rviz -d /home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/rviz/parking_demo.rviz
```

## 8. 推荐调试顺序

每次实验建议按这个顺序：

1. 确认雷达稳定：

   ```bash
   rostopic hz /tianbot_mini/scan
   ```

2. 确认 AMCL 有输出：

   ```bash
   rostopic echo -n 1 /tianbot_mini/amcl_pose
   ```

3. 在 RViz 中使用 `2D Pose Estimate` 对齐小车。

4. 先点一个空旷区域目标，测试是否能发布正常路径。

5. 再点车位入口附近，不要一开始点车位最里面。

6. 观察终端是否出现：

   ```text
   Published parking path with XX poses
   Parking LQR follower received path with XX poses
   ```

7. 如果只发布 1 个点，降低 `obstacle_inflation`。

8. 如果路径正常但车跟不好，再调 LQR 参数。

## 9. 当前判断

当前系统瓶颈优先级：

```text
雷达稳定性 / AMCL 定位
> 地图质量
> 碰撞膨胀参数
> Hybrid A* 搜索鲁棒性
> 路径跟踪控制算法
```

LQR 可以让路径跟踪更平滑，但不能解决：

- AMCL 漂移
- 雷达不稳定
- 地图不准
- 起点或目标被判 collision
- Hybrid A* 搜索失败

因此后续调试应优先保证：

```text
/tianbot_mini/scan 稳定
雷达点云和地图重合
obstacle_inflation 不要过大
路径能正常发布几十个点
```

## 10. 换 GPT 账号后的使用方式

换新账号后，可以对新的 GPT 说：

```text
请先阅读 src/tianbot_mini_parking/README.md 和 NOTES.md。
这是我之前做的 Tianbot mini 自动泊车项目。
请基于这些上下文继续协助我调试 ROS1 真实小车自动泊车。
```

如果项目已经上传 GitHub，把仓库链接也发给新 GPT。
