## GPS experiment (without slam)

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



1.  run simple_gps_navigation_fixed_speed_v2.py and simple_map_plot_v2.py in offboard computer :

```bash
cd ~/clearpath_ws/src/husky_navigation_pkg/husky_navigation_pkg/
```

```bash
python3 simple_gps_navigation_fixed_speed_v2.py
```

```bash
python3 simple_map_plot_v2.py
```