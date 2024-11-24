# ROHAND URDF PACKAGE FOR ROS2

rohand urdf package

## 1.Clone

```BASH
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone ssh://git@github.com/oymotion/rohand_urdf_ros2
```

## 2.Compile

```BASH
colcon build
source install/setup.bash
```

## 4.Node rohand_urdf

| Topic             | Description                                                               |
| ----------------- | ------------------------------------------------------------------------- |
| "if_slider_joint" | slider for index finger, control index finger by changing it's position   |
| "mf_slider_joint" | slider for middle finger, control middle finger by changing it's position |
| "rf_slider_joint" | slider for ring finger, control ring finger by changing it's position     |
| "lf_slider_joint" | slider for little finger, control little finger by changing it's position |
| "th_slider_joint" | slider for thumb, control thumb by changing it's position                 |
| "th_root_joint"   | slider for thumb root, control thumb root by changing it's position       |

## 5.RUN

Launch 'launch.py' file:
Left hand：

```BASH
ros2 launch rohand_urdf left_rviz2.launch.py 
```

Right hand：

```BASH
ros2 launch rohand_urdf right_rviz2.launch.py
```
