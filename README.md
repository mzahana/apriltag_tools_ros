# Apriltag Tools ROS

ROS 2 package is designed to provide tools and utilities for working with AprilTags in ROS-based robotics applications.


## Installation

```
git clone https://github.com/khaledgabr77/apriltag_tools_ros.git
```



## AprilTag Detection to Pose Conversion

This package now includes functionality to convert AprilTag detections from the `isaac_ros_apriltag` package to pose arrays. This conversion enables users to obtain pose information directly from AprilTag detections, which can be useful for localization, mapping, and navigation tasks in robotics applications.

### How to Use

1. **Launch the Conversion Node**: Use the provided launch file to start the conversion node.

    ```
    ros2 launch apriltag_tools_ros apriltag_detection_to_pose.launch.py
    ```

2. **Subscribe to Pose Array**: Subscribe to the `apriltag_pose_array` topic to receive pose information corresponding to detected AprilTags.

### Conversion Logic

The conversion node (`apriltag_detection_to_pose.py`) subscribes to the AprilTag detections topic published by the `isaac_ros_apriltag` package. Upon receiving detection messages, it converts each detection to a pose message and publishes them as a pose array.




