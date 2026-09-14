# G1 统一启动入口

`robot.launch.py` 是整机唯一顶层入口，只编排各包已经存在的原子 launch，避免重复
启动 Livox、Lightning、`g1_nav_bridge`、Nav2 和 `collision_monitor`。

## 互斥模式

```bash
# 传感器、rosbridge 和业务后端，不启动 SLAM/Nav2/运动桥
ros2 launch robot_bringup robot.launch.py mode:=base

# 建图
ros2 launch robot_bringup robot.launch.py mode:=mapping with_ui:=true start_rviz:=true

# 仅定位
ros2 launch robot_bringup robot.launch.py mode:=localization \
  map_dir:=/opt/G1/lighting_ws/data/new_map with_ui:=true start_rviz:=true

# 定位 + 静态地图 + Nav2 + 唯一运动桥
ros2 launch robot_bringup robot.launch.py mode:=navigation \
  map_dir:=/opt/G1/lighting_ws/data/new_map \
  nav_map:=/opt/G1/lighting_ws/data/new_map/map.yaml \
  floor_z:=0.0 base_floor_z:=0.0 \
  use_collision_monitor:=false start_rviz:=true
```

`navigation` 模式中，Lightning 定位由 `g1_localization.launch.py` 创建；Nav2、运动桥和
可选的 collision monitor 由 `g1_navigation.launch.py` 创建。顶层不会再次创建这些节点。

`use_collision_monitor` 在完成点云高度、停车区和软件停车验收前默认关闭。前端当前没有
速度遥控话题，统一入口不会虚构或订阅一个前端遥控接口。
