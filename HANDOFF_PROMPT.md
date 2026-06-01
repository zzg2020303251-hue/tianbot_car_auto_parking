# 给新 GPT 的接手提示

请先阅读本文件，然后阅读同目录下的 `README.md` 和 `NOTES.md`。这是一个 ROS1 Noetic 下的 `tianbot_mini` 真实小车自动泊车项目，用户希望继续调试并最终上传到 GitHub。

## 项目路径

```text
/home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking
```

工作区路径：

```text
/home/tianbot/ros1_project_leader_follower
```

## 用户偏好

- 使用中文回答。
- 不要在没有明确指令时随意修改代码。
- 如果要修改，先简要说明要改什么。
- 用户希望一步一步指导终端命令。
- 用户正在真实 `tianbot_mini` 小车上做实验，不只是仿真。
- 用户会把终端日志贴出来，需要根据日志判断问题。

## 当前项目目标

实现真实 `tianbot_mini` 小车自动泊车：

1. SLAM 建图。
2. 保存地图。
3. 固定地图 + AMCL 定位。
4. RViz `2D Nav Goal` 给定停车目标。
5. Hybrid A* 规划路径。
6. LQR 或 Pure Pursuit 路径跟踪。
7. 发布 `/tianbot_mini/cmd_vel` 控制真实小车。

## 当前重要文件

```text
launch/parking_mapping.launch       # 真实小车建图
launch/parking_real_map.launch      # 固定地图导航
launch/parking_demo.launch          # 规划器 + follower
scripts/hybrid_parking_planner_node.py
scripts/parking_path_follower_node.py
scripts/parking_lqr_follower_node.py
rviz/parking_demo.rviz
maps/parking_map.yaml
maps/parking_map.pgm
README.md
NOTES.md
```

## 当前控制器状态

已经保留原 Pure Pursuit 控制器：

```text
scripts/parking_path_follower_node.py
```

并新增 LQR 控制器：

```text
scripts/parking_lqr_follower_node.py
```

`parking_demo.launch` 默认调用 LQR：

```text
follower_type:=lqr
```

如果要切回原 Pure Pursuit：

```text
follower_type:=pure_pursuit
```

## 当前规划器状态

`hybrid_parking_planner_node.py` 当前包含：

- Hybrid A* 搜索。
- 起点碰撞时继续尝试，但会警告。
- 原地旋转扩展。
- 路径 shortcut。
- 路径重采样。
- 梯度平滑。
- 障碍物距离场。
- 路径远离障碍物的轻量推离。
- 目标点碰撞时，自动在目标附近搜索可行目标。
- 新目标到来时先发布空路径，清除旧 latch 路径。

重要现象：

```text
Published parking path with 1 poses
```

通常说明规划失败，只返回 partial path，不是有效路径。

## 当前推荐启动命令

优先使用低速、低膨胀参数，先保证能规划出多点路径：

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

如果雷达不是 `/dev/ttyUSB0`，改为：

```text
lidar_port:=/dev/ttyUSB1
```

## 当前调试结论

目前系统最大问题不是 LQR/Pure Pursuit，而是：

```text
雷达稳定性 / AMCL 定位
> 地图质量
> 碰撞膨胀参数
> Hybrid A* 搜索鲁棒性
> 路径跟踪控制器
```

不要一上来继续换 MPC 或复杂控制器。优先确认：

```bash
rostopic hz /tianbot_mini/scan
rostopic echo -n 1 /tianbot_mini/amcl_pose
rostopic echo -n 1 /parking_path/header
```

## 已经验证过的重要事实

用户曾使用：

```text
obstacle_inflation:=0.10
```

导致起点被判 collision。

用当前地图复算过某次起点：

```text
start=(0.33, -1.32, 42.4 deg)
```

结果：

```text
obstacle_inflation 0.00 -> 不碰撞
obstacle_inflation 0.02 -> 不碰撞
obstacle_inflation 0.03 -> 不碰撞
obstacle_inflation 0.05 -> 碰撞
obstacle_inflation 0.08 -> 碰撞
obstacle_inflation 0.10 -> 碰撞
```

因此当前建议：

```text
obstacle_inflation:=0.02 到 0.04
```

不要默认使用 `0.10`。

## 雷达问题

雷达曾出现：

```text
No laser scan received
timout count
Failed to start scan mode
Received message with unrecognized topicId
```

可能原因：

- USB 线/接口不稳。
- 小车电量不足。
- 雷达供电不稳。
- `/dev/ttyUSB0` 和 `/dev/ttyUSB1` 端口变化。
- `ModemManager` 抢串口。

实验前建议：

```bash
sudo systemctl stop ModemManager
```

如要永久禁用：

```bash
sudo systemctl disable ModemManager
sudo systemctl mask ModemManager
```

注意：这些需要用户自己输入 sudo 密码。

## AMCL 问题

用户反馈：

```text
雷达点云和地图越走越漂
```

AMCL 参数已调成更激进版本，位于：

```text
launch/parking_real_map.launch
```

当前参数：

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

如果 AMCL 仍然漂，优先怀疑：

- 雷达不稳定。
- 地图质量差。
- 地面不平导致 odom 漂。
- 真实环境和地图不一致。

## RViz 问题

曾经 RViz 崩溃：

```text
Duration has to be finite
```

可能和雷达/TF 时间戳异常有关。

可临时关闭 launch 自带 RViz：

```bash
use_rviz:=false
```

再单独启动：

```bash
rosrun rviz rviz -d /home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/rviz/parking_demo.rviz
```

## GitHub 上传注意事项

用户希望把整个项目上传到 GitHub 公开仓库。

当前建议只添加：

```bash
git add src/tianbot_mini_parking
```

不要直接 `git add .`，因为仓库里还有一些无关改动：

```text
src/CMakeLists.txt
src/pursuit_sim/CMakeLists.txt
src/pursuit_sim/package.xml
^C
roslaunch
run_tuning_demo.sh
```

提交命令建议：

```bash
git commit -m "Add Tianbot mini parking project"
git push origin main
```

## 新 GPT 接手后第一步建议

1. 先让用户运行：

   ```bash
   cd /home/tianbot/ros1_project_leader_follower
   git status
   ```

2. 如果用户继续做实验，先检查：

   ```bash
   rostopic hz /tianbot_mini/scan
   ```

3. 如果用户要上传 GitHub，先只提交：

   ```bash
   git add src/tianbot_mini_parking
   git commit -m "Add Tianbot mini parking project"
   git push origin main
   ```

4. 如果用户说规划器只发布 1 个点，优先降低：

   ```text
   obstacle_inflation
   min_obstacle_clearance
   ```

5. 如果用户说小车越走越偏，优先排查：

   ```text
   /tianbot_mini/scan
   AMCL 初始位姿
   地图质量
   地面打滑
   ```

请基于以上上下文继续协助用户，不要从零开始重建项目。
