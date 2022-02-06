# PCL_ROS_LIBRARY

Pointlcloud library and ros integration for pointcloud manipulation, including **crop**, **transform**, **filter** and **clustering**.

![](output.png)

Parameters are found in `config/pcl_ros_lib.yaml` with some crucial values being displayed below, `Transform Parameters` parses the values into a `TransformStamped` message, while `Crop Parameters` parses the values into a PCL box crop 

```yaml
# Transform Parameters
translate_x: 0.0
translate_y: 0.0
translate_z: 1.0
rotate_roll: 0.0
rotate_pitch: 0.0
rotate_yaw: 0.0

# DBSCAN Parameters
resolution: 0.3
nearest_min_distance: 5
min_cluster_pts: 50
eps: 0.3

# Crop Parameters
centroid_x: 0.0
centroid_y: 0.0
centroid_z: 5.0

dimension_x: 39.4
dimension_y: 19.4
dimension_z: 9.7

# Run Parameters
spin_once: false
ros_rate: 2.0
```

### Setup
```
sudo apt-get install -y ros-melodic-tf2-ros ros-melodic-tf2-sensor-msgs ros-melodic-tf2-geometry-msgs
# Add to your ws/src
catkin build pcl_ros_lib -j1
```

### References
1. **DBSCAN** : https://github.com/LingyuDu/dbscan/blob/master/dbscan.cpp
