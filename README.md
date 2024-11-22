# ROHAND URDF BASED ON ROS2

rohand urdf file

## STEPS

### PREPARE

1.Create a ROS2 workspace in ubuntu:

```SHELL
mkdir -p ~/ros2_ws/src
```

2.Copy the rohand_urdf folder to ~/ros2_ws/src:

```SHELL
cp -r /(your path to rohand_urdf folder) ~/ros2_ws/src
```

3.Enter the workspace and compile:

```SHELL
cd ~/ros2_ws/src
colcon build
source install/setup.bash
```

### RUN

Launch 'launch.py':
Left hand：

```SHELL
ros2 launch rohand_urdf left_rviz2.launch.py 
```

Right hand：

```SHELL
ros2 launch rohand_urdf right_rviz2.launch.py
```
