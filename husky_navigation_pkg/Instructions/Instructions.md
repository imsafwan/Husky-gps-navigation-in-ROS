## GPS experiment

### connect and start robot:

1.  connect to husky's primary computer:

```bash
ssh administrator@192.168.131.1
```

2.  start services:

```bash
sudo systemctl restart clearpath-robot.service
```

```bash
export ROS_DISCOVERY_SERVER=192.168.131.1:11811
```

### experiment setup:

1.  run rviz in offboard computer :

```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_1093
```

2.  connect to husky's secondary computer (onboard):

```bash
ssh administrator@192.168.131.2
```

4.  run slam in onboard computer:

```bash
ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true
```

3.  run nav2 in onboard computer:

```bash
ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true
```

4.  set gps/map frame origin in onboard computer :

```bash
ros2 topic pub /map_origin_gps sensor_msgs/msg/NavSatFix "{latitude: 41.869878, longitude: -87.648470, altitude: 10.0}"
```

5.  run gps_navigation script in offboard computer:

navigate first to directory

```bash
python3 map_plot_v2.py
```