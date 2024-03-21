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

    tag_detections_topic_arg = DeclareLaunchArgument(
        name='tag_detections_topic', 
        default_value = 'in/tag_detections', 
        description='Tag detections topic name')
    
    tag_detections_topic = LaunchConfiguration('tag_detections_topic')

    output_posearray_topic_arg = DeclareLaunchArgument(
        name='output_posearray_topic', 
        default_value = 'apriltags/pose_array', 
        description='Topic name of the tag output pose array')
    
    output_posearray_topic = LaunchConfiguration('output_posearray_topic')

    apriltag_detections_to_pose = Node(
        package='apriltag_tools_ros',
        executable='apriltag_detections_to_pose',
        name='apriltag_detections_to_pose',
        output='screen',
        parameters=[{
            'reference_frame_id': reference_frame
        }],
        remappings=[
            ('in/tag_detections', tag_detections_topic),
            ('apriltags/pose_array', output_posearray_topic)
        ]
    )
 
    return launch.LaunchDescription([reference_frame_arg,
                                     tag_detections_topic_arg,
                                     output_posearray_topic_arg,
                                     apriltag_detections_to_pose])