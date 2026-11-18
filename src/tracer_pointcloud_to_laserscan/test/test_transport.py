"""Exercise the compiled converter on isolated topics, without robot commands."""
import array
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


@pytest.fixture
def converter():
    rclpy.init()
    node = Node('isolated_scan_transport_test')
    executable = Path(get_package_prefix('tracer_pointcloud_to_laserscan')) / 'lib/tracer_pointcloud_to_laserscan/pointcloud_to_laserscan_node'
    process = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__node:=isolated_scan_converter',
        '-r', 'cloud_in:=/isolated_test/cloud', '-r', 'scan:=/isolated_test/scan',
        '-p', 'target_frame:=isolated_scan_base', '-p', 'queue_size:=1',
        '-p', 'min_height:=0.1', '-p', 'max_height:=1.5',
        '-p', 'range_min:=0.45', '-p', 'range_max:=3.0',
        '-p', f'angle_increment:={math.pi/2}',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    messages = []
    node.create_subscription(LaserScan, '/isolated_test/scan',
                             lambda msg: messages.append((time.monotonic(), msg)),
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
    publisher = node.create_publisher(PointCloud2, '/isolated_test/cloud',
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = 'isolated_scan_base'
    tf.child_frame_id = 'isolated_scan_lidar'
    tf.transform.translation.z = 0.71
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    deadline = time.monotonic()+8
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    try:
        assert publisher.get_subscription_count() > 0, 'Converter did not subscribe'
        assert process.poll() is None
        yield node, publisher, messages
    finally:
        process.terminate()
        process.communicate(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


def cloud(points):
    data = np.zeros((len(points), 8), dtype='<f4')  # 32-byte Velodyne-like stride.
    data[:, :3] = points
    message = PointCloud2()
    message.header.frame_id = 'isolated_scan_lidar'
    message.height = 1
    message.width = len(points)
    message.point_step = 32
    message.row_step = message.width*32
    message.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
    message.data = array.array('B', data.tobytes())
    return message


def publish_for(node, publisher, message, messages, duration=1):
    deadline = time.monotonic()+duration
    next_send = time.monotonic()
    while time.monotonic() < deadline:
        if time.monotonic() >= next_send:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            next_send += 0.1
        rclpy.spin_once(node, timeout_sec=0.005)


def test_transport_qos_and_projection_preserve_filters(converter):
    node, publisher, messages = converter
    subscriptions = node.get_subscriptions_info_by_topic('/isolated_test/cloud')
    assert subscriptions[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    message = cloud([[1, 0, -0.51], [0.8, 0, -0.71], [0.2, 0, 0],
                     [4, 0, 0], [0, 1.5, 0], [0, -0.9, 0],
                     [0.7, 0, 2], [-2, 0, 0], [float('nan'), 0, 0]])
    publish_for(node, publisher, message, messages)
    assert messages
    result = messages[-1][1]
    assert result.header.frame_id == 'isolated_scan_base'
    assert len(result.ranges) == 4
    assert math.isinf(result.ranges[0])  # Exact upper angle is safely rejected.
    assert result.ranges[1] == pytest.approx(0.9)
    assert result.ranges[2] == pytest.approx(1.0)
    assert result.ranges[3] == pytest.approx(1.5)
    assert result.header.stamp.sec > 0


def test_large_reliable_clouds_do_not_have_multi_second_scan_gaps(converter):
    node, publisher, messages = converter
    message = cloud(np.tile([1.0, 0.0, 0.0], (30000, 1)))
    publish_for(node, publisher, message, messages, duration=3)
    assert len(messages) >= 20
    gaps = [messages[i][0]-messages[i-1][0] for i in range(1, len(messages))]
    assert max(gaps) < 0.75
