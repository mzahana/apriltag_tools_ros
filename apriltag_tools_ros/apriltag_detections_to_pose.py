import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray, AprilTagDetection 

class AprilTagConverter(Node):
    def __init__(self):
        super().__init__('apriltag_detections_to_pose')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',  
            self.tag_callback,
            10)
        self.publisher = self.create_publisher(
            PoseArray,
            'pose_array_sub_',  
            10)

    def tag_callback(self, msg):
        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header
        for detection in msg.detections:
            pose_msg = Pose()
            pose_msg.position = detection.pose.pose.pose.position
            pose_msg.orientation = detection.pose.pose.pose.orientation
            pose_array_msg.poses.append(pose_msg)
            x = pose_msg.position.x 
            y = pose_msg.position.y
            print(f"PoseX: {x}") 
            print(f"PoseY: {y}") 
        self.publisher.publish(pose_array_msg)

def main(args=None):
    rclpy.init(args=args)
    april_tag_converter = AprilTagConverter()
    rclpy.spin(april_tag_converter)
    april_tag_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
