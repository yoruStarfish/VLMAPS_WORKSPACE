import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UserCommandPublisher(Node):
    def __init__(self):
        super().__init__('user_command_publisher')

        self.publisher_ = self.create_publisher(
            String,
            '/user_command',
            10
        )

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published: {command}')


def main(args=None):
    rclpy.init(args=args)

    node = UserCommandPublisher()

    try:
        while rclpy.ok():
            command = input('Enter command: ')

            if command.lower() == 'quit':
                break

            node.publish_command(command)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()