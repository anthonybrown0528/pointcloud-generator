# Point Cloud Generator

## To Use
```
ros2 run pointcloud_proc_cpp gen_pointcloud --ros-args --remap image:=[Name of depth image topic] --remap camera_state:=[Name of camera state topic] --remap pointcloud:=[Name of point cloud topic] --remap camera_params:=[Name of camera params topic]
```

## Dependencies
### rclcpp
### sensor_msgs
### cv_bridge
### pointcloud_interfaces

## To Install
* Clone repository into workspace and build it
* Source ROS workspace