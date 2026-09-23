import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
from rclpy.duration import Duration
import transforms3d
import subprocess

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the BasicNavigator for navigation tasks
        self.navigator = BasicNavigator()

        # Subscribe to the aruco_poses topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10
        )

    def aruco_pose_callback(self, msg: PoseArray):
        if msg.poses:
            aruco_pose = msg.poses[0]

            try:
                # Wait for the transform from camera_link to map frame to become available
                transform = self.tf_buffer.lookup_transform(
                    'map',  # Target frame
                    'camera_link',  # Source frame (assuming aruco poses are in camera_link)
                    rclpy.time.Time()
                )
                self.get_logger().info(f"transform from camera_link to map: {transform}")

                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = 0.0
                initial_pose.pose.position.y = 0.0 
                initial_pose.pose.orientation.z = 0.0
                initial_pose.pose.orientation.w = 0.0
                self.navigator.setInitialPose(initial_pose)

                # Transform the pose to the map frame
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.navigator.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'map'
                self.get_logger().info(f"Aruco Pose : Position - x: {aruco_pose.position.x}, y: {aruco_pose.position.y}, z: {aruco_pose.position.z}; Orientation - x: {aruco_pose.orientation.x}, y: {aruco_pose.orientation.y}, z: {aruco_pose.orientation.z}, w: {aruco_pose.orientation.w}")

                # Assign the position and orientation from aruco_pose to pose_stamped.pose
                pose_stamped.pose.position.x = aruco_pose.position.x
                pose_stamped.pose.position.y = aruco_pose.position.y
                pose_stamped.pose.position.z = 0
                quat = transforms3d.euler.euler2quat(0,0,aruco_pose.orientation.z)
                pose_stamped.pose.orientation.x = quat[1]
                pose_stamped.pose.orientation.y = quat[2]
                pose_stamped.pose.orientation.z = quat[3]
                pose_stamped.pose.orientation.w = quat[0]
                self.get_logger().info(f"PoseStamped Pose: Position - x: {pose_stamped.pose.position.x}, y: {pose_stamped.pose.position.y}, z: {pose_stamped.pose.position.z}; Orientation - x: {pose_stamped.pose.orientation.x}, y: {pose_stamped.pose.orientation.y}, z: {pose_stamped.pose.orientation.z}, w: {pose_stamped.pose.orientation.w}")
                
                # Convert the pose to the 'map' frame
                try:
                    transformed_pose = do_transform_pose(pose_stamped.pose, transform)
                except Exception as e:
                    self.get_logger().info(f"{str(e)}")
                # Adjust position to stop at a safe distance before the Aruco marker
                stop_distance = 0.2  # Safety distance from the marker
                transformed_pose.position.x -= stop_distance
                output_stamped = PoseStamped()
                output_stamped.pose.position.x = transformed_pose.position.x
                output_stamped.pose.position.y = transformed_pose.position.y
                output_stamped.pose.position.z = transformed_pose.position.z
                output_stamped.pose.orientation.x = transformed_pose.orientation.x
                output_stamped.pose.orientation.y = transformed_pose.orientation.y
                output_stamped.pose.orientation.z = transformed_pose.orientation.z
                output_stamped.pose.orientation.w = transformed_pose.orientation.w
                try:
                    path = self.navigator.getPath(initial_pose, pose_stamped)
                except Exception as e:
                    self.get_logger().info(f"{str(e)}")
                # Send goal using the BasicNavigator
                # self.send_goal(output_stamped)
                subprocess.run(['ros2', 'topic', 'pub', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', '\"{header: {stamp:{ sec: 0}, frame_id:\'base_link\'}, pose: {pose: {position: {x:0, y:0, z:0}, orientation:{ z:0, w:0}}}}\"'])
                subprocess.run(['ros2', 'action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', '\"{pose: {header: {frame_id: \'map\'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}\"'
])
            except Exception as e:
                self.get_logger().error(f"Failed to transform aruco pose to map frame: {str(e)}")

    def send_goal(self, goal_pose: PoseStamped):
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('Navigating to the Aruco marker...')
        self.navigator.goToPose(goal_pose)
        i = 0

        # Monitor the navigation status
        while not self.navigator.isTaskComplete():        
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = 0.0
                self.navigator.goToPose(goal_pose)

        # Check the result of the navigation task
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')

def main():
    rclpy.init()
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
