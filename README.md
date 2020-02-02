# NGCP-Capstone-UGV

CPE Capstone 350-450 
2019 - 2020

## Dependencies

ROS:
- rplidar_ros
- zed-ros-wrapper
- mavros (mavlink)

## ROS Project Setup

1) Install ZED dependencies: https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/

2) Install MAVLink Dependneices: https://github.com/mavlink/mavros/tree/master/mavros#installation

3) Build ros nodes

```
$ cd ros
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin build
```

