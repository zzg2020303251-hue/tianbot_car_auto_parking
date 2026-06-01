# 地图保存目录

真小车扫图完成后，建议把地图保存到这里：

```bash
rosrun map_server map_saver -f /home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/maps/parking_map
```

保存后会生成：

```text
parking_map.yaml
parking_map.pgm
```

之后启动固定地图泊车：

```bash
roslaunch tianbot_mini_parking parking_real_map.launch \
  map:=/home/tianbot/ros1_project_leader_follower/src/tianbot_mini_parking/maps/parking_map.yaml
```
