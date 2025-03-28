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



1.  connect to husky's secondary computer (onboard):

```bash
ssh administrator@192.168.131.2
```


2.  run simple_map_plot.py in onboard computer :

```bash
cd ~/clearpath_ws/src/husky_navigation_pkg/husky_navigation_pkg/
```

```bash
python3 simple_map_plot.py
```