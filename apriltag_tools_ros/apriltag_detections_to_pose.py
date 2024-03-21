import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from apriltag_ros_interfaces.msg import AprilTagDetectionArray, AprilTagDetection
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import Pose as TF2Pose
from tf2_geometry_msgs import do_transform_pose

class AprilTagConverter(Node):
    def __init__(self):
        super().__init__('apriltag_detections_to_pose')

        self.declare_parameter("reference_frame_id", "odom")
        self.reference_frame_id_ = self.get_parameter('reference_frame_id').get_parameter_value().string_value

        # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_,self)

        self.create_subscription(
            AprilTagDetectionArray,
            'in/tag_detections',  
            self.tag_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            PoseArray,
            'apriltags/pose_array',  
            10)

    def tag_callback(self, msg):
        # Make sure there is TF from reference frame to camera frame
        try:
            transform = self.tf_buffer_.lookup_transform(
                self.reference_frame_id_,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().error(
            f'[AprilTagConverter::tag_callback] Could not transform from {msg.header.frame_id} to {self.reference_frame_id_} : {ex}')
            return
        
        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header
        for detection in msg.detections:
            pose_msg = Pose()
            pose_msg.position = detection.pose.pose.pose.position
            pose_msg.orientation = detection.pose.pose.pose.orientation
            
            transformed_pose_msg = self.transform_pose(pose_msg, transform)
            pose_array_msg.poses.append(transformed_pose_msg)
        
        self.publisher.publish(pose_array_msg)

    def transform_pose(self, pose: Pose, tr: TransformStamped) -> Pose:
        """
        @brief Converts 3D positions in the camera frame to a parent_frame
        @param pose:  3D position in the child frame (sensor e.g. camera)
        @param tr: Transform 4x4 matrix theat encodes rotation and translation
        @return transformed_pose: Pose of transformed position
        """     
        tf2_pose_msg = TF2Pose()
        tf2_pose_msg.position.x = pose.position.x
        tf2_pose_msg.position.y = pose.position.y
        tf2_pose_msg.position.z = pose.position.z
        tf2_pose_msg.orientation.w = pose.orientation.w
        tf2_pose_msg.orientation.x = pose.orientation.x
        tf2_pose_msg.orientation.y = pose.orientation.y
        tf2_pose_msg.orientation.z = pose.orientation.z
   
    
        try:
            transformed_pose = do_transform_pose(tf2_pose_msg, tr)
        except Exception as e:
            self.get_logger().error("[transform_pose] Error in transforming pose {}".format(e))
            return None

        return transformed_pose

def main(args=None):
    rclpy.init(args=args)
    april_tag_converter = AprilTagConverter()
    rclpy.spin(april_tag_converter)
    april_tag_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
