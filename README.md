# Point Cloud Generator

## Prerequisites
<p>Clone and build pybullet_ros package</p>

[pybullet_ros](https://github.com/packbionics/pybullet_ros/)

<p>Clone and build pointcloud_interfaces package</p>

[pointcloud_interfaces](https://github.com/anthonybrown0528/pointcloud-interfaces)

## To Use
<p>Source workspace</p>
<p>Run: <p>

```
ros2 launch pybullet_ros pointcloud_vision_example.launch.py
```

### OR
<p>Source workspace</p>
<p>Run PyBullet Simulation</p>
<p>Then Run: </p>

```
ros2 run pointcloud_proc_cpp gen_pointcloud --ros-args --remap image:=camera/depth/image_raw --remap camera_state:=camera/state --remap pointcloud:=camera/point_cloud --remap camera_params:=camera/params
```

<hr />

## Dependencies
* rclcpp
* sensor_msgs
* cv_bridge
* pointcloud_interfaces

<hr />

## To Install
* Clone repository into workspace and build it
* Source ROS workspace