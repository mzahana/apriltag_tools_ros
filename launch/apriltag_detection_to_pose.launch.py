import launch
from launch_ros.actions import Node


def generate_launch_description():
    apriltag_detections_to_pose = Node(
        package='apriltag_tools_ros',
        executable='apriltag_detections_to_pose',
        name='apriltag_detections_to_pose',
        output='screen',
    )
 
    return launch.LaunchDescription([apriltag_detections_to_pose])