"""
implementation of " ros2 run tf2_ros tf2_echo frame1 frame2"
"""



import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
from geometry_msgs.msg import PoseStamped, PoseArray
from rclpy.logging import get_logger

class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.initial_pose_publisher = InitialPosePublisher(self.tf_buffer, self.tf_listener)

    def timer_callback(self):
        try:
            time_now = self.get_clock().now()

            seconds, nanoseconds = time_now.seconds_nanoseconds()
            target_time = Time(seconds=seconds, nanoseconds=nanoseconds) - Duration(seconds=1.0)

            map_to_odom = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='odom',
                time=Time(),  
                timeout=Duration(seconds=2.0)
            )
            odom_to_base_link = self.tf_buffer.lookup_transform(
                target_frame='odom',
                source_frame='base_link',
                time=Time(), 
                timeout=Duration(seconds=2.0)
            )

            combined_transform = self.combine_transforms(map_to_odom, odom_to_base_link)
            self.print_transform(combined_transform)
            if combined_transform:
                self.initial_pose_publisher.update_initial_pose(combined_transform)


        except TransformException as ex:
            self.get_logger().warn('%s; retrying...' % ex)

    def combine_transforms(self, transform1, transform2):
        t1_translation = transform1.transform.translation
        t1_rotation = transform1.transform.rotation
        t2_translation = transform2.transform.translation
        t2_rotation = transform2.transform.rotation

        t1_matrix = tf_transformations.translation_matrix(
            [t1_translation.x, t1_translation.y, t1_translation.z])
        t1_rotation_matrix = tf_transformations.quaternion_matrix(
            [t1_rotation.x, t1_rotation.y, t1_rotation.z, t1_rotation.w])
        t1_matrix = tf_transformations.concatenate_matrices(t1_matrix, t1_rotation_matrix)

        t2_matrix = tf_transformations.translation_matrix(
            [t2_translation.x, t2_translation.y, t2_translation.z])
        t2_rotation_matrix = tf_transformations.quaternion_matrix(
            [t2_rotation.x, t2_rotation.y, t2_rotation.z, t2_rotation.w])
        t2_matrix = tf_transformations.concatenate_matrices(t2_matrix, t2_rotation_matrix)

        combined_matrix = tf_transformations.concatenate_matrices(t1_matrix, t2_matrix)

        combined_transform = transform1
        combined_transform.transform.translation.x, combined_transform.transform.translation.y, combined_transform.transform.translation.z = tf_transformations.translation_from_matrix(
            combined_matrix)
        combined_transform.transform.rotation.x, combined_transform.transform.rotation.y, combined_transform.transform.rotation.z, combined_transform.transform.rotation.w = tf_transformations.quaternion_from_matrix(
            combined_matrix)

        return combined_transform

    def print_transform(self, trans):
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        rot_matrix = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

        self.get_logger().info(f'At time {trans.header.stamp.sec}.{trans.header.stamp.nanosec}')
        self.get_logger().info(f'- Translation: [{translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}]')
        self.get_logger().info(f'- Rotation: in Quaternion [{rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}, {rotation.w:.3f}]')
        rpy_rad = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        rpy_deg = [angle * 180.0 / 3.141592653589793 for angle in rpy_rad]
        self.get_logger().info(f'- Rotation: in RPY (radian) [{rpy_rad[0]:.3f}, {rpy_rad[1]:.3f}, {rpy_rad[2]:.3f}]')
        self.get_logger().info(f'- Rotation: in RPY (degree) [{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]')
        self.get_logger().info('- Matrix:')
        for row in rot_matrix:
            self.get_logger().info(f' {row[0]:.3f} {row[1]:.3f} {row[2]:.3f} {row[3]:.3f}')

class InitialPosePublisher(Node):

    def __init__(self, tf_buffer, tf_listener):
        super().__init__('initial_pose_publisher')
        self.tf_buffer = tf_buffer
        self.tf_listener = tf_listener
        self.initial_pose = None

    def update_initial_pose(self, trans):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = trans.transform.translation.x
        pose_msg.pose.position.y = trans.transform.translation.y
        pose_msg.pose.position.z = trans.transform.translation.z
        pose_msg.pose.orientation = trans.transform.rotation
        self.initial_pose = pose_msg

    def get_initial_pose(self):
        return self.initial_pose

def main(args=None):
    logger = get_logger('localization_main')

    rclpy.init()

    localization_node = LocalizationNode()

    while rclpy.ok() and not localization_node.initial_pose_publisher.get_initial_pose():
        rclpy.spin_once(localization_node, timeout_sec=1.0)

    initial_pose = localization_node.initial_pose_publisher.get_initial_pose()
    if initial_pose:
        logger.info('Got initial pose from RTAB-Map')
        logger.info(str(initial_pose))
        print('Got initial pose from RTAB-Map')
        print(initial_pose)
        # Use the initial_pose as needed
    else:
        print('Failed to get initial pose')
        logger.warn('Failed to get initial pose')

    localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
