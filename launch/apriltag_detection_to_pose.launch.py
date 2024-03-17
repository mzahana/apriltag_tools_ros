import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    reference_frame_arg = DeclareLaunchArgument(
        name='reference_frame', 
        default_value = 'odom', 
        description='Reference frame ID (string)')
    
    reference_frame = LaunchConfiguration('reference_frame')

    apriltag_detections_to_pose = Node(
        package='apriltag_tools_ros',
        executable='apriltag_detections_to_pose',
        name='apriltag_detections_to_pose',
        output='screen',
        parameters=[{
            'reference_frame_id': reference_frame
        }]
    )
 
    return launch.LaunchDescription([reference_frame_arg,
                                     apriltag_detections_to_pose])