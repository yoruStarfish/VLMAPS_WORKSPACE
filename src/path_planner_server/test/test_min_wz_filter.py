"""Regression checks for navigation commands crossing the turn filter."""

from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Twist

from path_planner_server.min_wz_filter import MinWzFilter


def make_filter():
    node = object.__new__(MinWzFilter)
    node.min_wz = 0.6
    node.trigger_wz = 0.001
    node.low_vx_threshold = 0.025
    node.turn_threshold = 0.2
    node.turn_release = 0.1
    node.command_timeout = 0.5
    node.turning = False
    node.last_command_time = None
    node.pub = SimpleNamespace(publish=Mock())
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    return node


def command(node, vx, wz):
    msg = Twist()
    msg.linear.x = vx
    msg.angular.z = wz
    node.cmd_callback(msg)
    return node.pub.publish.call_args.args[0]


def test_forward_navigation_keeps_small_angular_command():
    node = make_filter()
    result = command(node, 0.0142, -0.0018)
    assert result.linear.x == 0.0142
    assert result.angular.z == -0.0018


def test_forward_command_exits_turning_state():
    node = make_filter()
    assert command(node, 0.0, 0.3).angular.z == 0.6
    result = command(node, 0.02, 0.3)
    assert result.linear.x == 0.02
    assert result.angular.z == 0.3
    assert not node.turning


def test_explicit_turn_still_receives_minimum_angular_speed():
    node = make_filter()
    result = command(node, 0.0, 0.3)
    assert result.linear.x == 0.0
    assert result.angular.z == 0.6
