"""Offline checks: no action goals or velocity commands reach a robot."""
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from navigatethroughposes import pure_navigation as nav


def mission():
    navigator = Mock()
    navigator.declare_parameter.side_effect = (
        lambda key, value: SimpleNamespace(value=value))
    with patch.object(nav, 'TransformListener'):
        return nav.PureNavigation(navigator)


def test_laptop_only_uses_nav2():
    with patch.object(nav, 'mission', 'laptop'):
        task = mission()
    task.wait_for_pose = Mock()
    task.navigate = Mock()
    task.rotate_right = Mock()
    task.drive_forward = Mock()
    task.run()
    task.navigate.assert_called_once_with('laptop')
    task.wait_for_pose.assert_not_called()
    task.rotate_right.assert_not_called()
    task.drive_forward.assert_not_called()
    task.navigator.waitUntilNav2Active.assert_called_once_with(localizer='bt_navigator')


def test_between_sequence_and_failed_navigation():
    with patch.object(nav, 'mission', 'between'):
        task = mission()
    task.wait_for_pose = Mock()
    calls = Mock()
    task.navigate = calls.navigate
    task.rotate_right = calls.rotate
    task.drive_forward = calls.drive
    with patch.object(nav.rclpy, 'ok', return_value=False):
        task.run()
    assert [call[0] for call in calls.mock_calls] == ['navigate', 'drive']
    calls.navigate.assert_called_once_with('Before_between')
    task.wait_for_pose.assert_not_called()
    calls.drive.assert_called_once_with(1.35)
    calls.reset_mock()
    task.navigate.side_effect = RuntimeError('Nav2 failed')
    with pytest.raises(RuntimeError):
        task.run()
    calls.rotate.assert_not_called()
    calls.drive.assert_not_called()


def test_right_turn_and_wraparound():
    task = mission()
    task.wait_for_pose = Mock(return_value=(2.0, 0.0, -math.pi))
    task.control_motion = Mock()
    task.rotate_right()
    command = task.control_motion.call_args.args[0]
    assert command((2.0, 0.0, -math.pi)).angular.z < 0
    assert command((2.0, 0.0, math.pi / 2)) is None


def test_drive_forward_align_and_arrive():
    task = mission()
    task.config.update(between_x=2.0, between_y=-1.0,
                       between_qz=-0.7071, between_qw=0.7071)
    task.control_motion = Mock()
    task.drive_to_between()
    command = task.control_motion.call_args.args[0]
    assert command((2.0, 0.0, -math.pi / 2)).linear.x > 0
    assert command((2.0, 0.0, 0.0)).linear.x == 0
    assert command((2.0, -1.0, 0.0)).linear.x == 0
    assert command((2.0, -1.0, 0.0)).angular.z < 0
    assert command((2.0, -1.0, -math.pi / 2)) is None


def test_stale_tf_stops_and_aborts():
    task = mission()
    task.get_pose = Mock(side_effect=ValueError('stale TF'))
    command = Mock()
    with patch.object(nav.rclpy, 'ok', return_value=True), \
            patch.object(nav.rclpy, 'spin_once'), \
            patch.object(nav.time, 'sleep'), \
            patch.object(nav.time, 'monotonic', side_effect=range(100)):
        with pytest.raises(RuntimeError, match='TF'):
            task.control_motion(command)
    command.assert_not_called()
    assert task.cmd_pub.publish.call_count >= 2
    for call in task.cmd_pub.publish.call_args_list:
        assert call.args[0].linear.x == 0
        assert call.args[0].angular.z == 0


def test_rejected_nav2_goal_aborts():
    task = mission()
    task.navigator.get_clock().now().to_msg.return_value = nav.Time().to_msg()
    task.navigator.goToPose.return_value = False
    with pytest.raises(RuntimeError, match='拒絕'):
        task.navigate('laptop')


@pytest.mark.parametrize('name, degrees', [('laptop', 90.0), ('Before_between', -135.0)])
def test_nav2_uses_waypoint_orientation(name, degrees):
    points = dict(nav.WAYPOINTS)
    half_angle = math.radians(degrees) / 2.0
    points[name] = (*points[name][:2], math.sin(half_angle), math.cos(half_angle))
    with patch.object(nav, 'WAYPOINTS', points):
        task = mission()
    task.navigator.get_clock().now().to_msg.return_value = nav.Time().to_msg()
    task.navigator.getResult.return_value = nav.TaskResult.SUCCEEDED
    with patch.object(nav.rclpy, 'ok', return_value=True):
        task.navigate(name)
    goal = task.navigator.goToPose.call_args.args[0]
    assert nav.get_yaw_from_quaternion(goal.pose.orientation) == pytest.approx(
        math.radians(degrees))


def test_quaternion_normalization_and_zero_rejection():
    task = mission()
    task.config['between_qz'] = -0.7071
    task.config['between_qw'] = 0.7071
    assert math.hypot(*task.quaternion('between')) == pytest.approx(1.0)
    assert task.yaw('between') == pytest.approx(-math.pi / 2)
    task.config['between_qz'] = 0.0
    task.config['between_qw'] = 0.0
    with pytest.raises(ValueError, match='四元數'):
        task.validate()


def test_forward_uses_time_only_and_stops():
    task = mission()
    task.get_pose = Mock(side_effect=AssertionError('must not read TF'))
    task.wait_for_pose = Mock(side_effect=AssertionError('must not wait for TF'))
    task.control_motion = Mock(side_effect=AssertionError('must not use feedback'))
    clock = [0.0]
    with patch.object(nav.rclpy, 'ok', return_value=True), \
            patch.object(nav.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(nav.time, 'sleep', side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
        task.drive_forward(0.3)
    assert clock[0] == pytest.approx(0.3 / task.config['linear_speed'])
    messages = [call.args[0] for call in task.cmd_pub.publish.call_args_list]
    assert all(msg.linear.x == task.config['linear_speed'] for msg in messages[:-1])
    assert all(msg.angular.z == 0 for msg in messages)
    assert messages[-1].linear.x == 0
    assert task.tf_buffer is None
    task.get_pose.assert_not_called()
    task.wait_for_pose.assert_not_called()
    task.control_motion.assert_not_called()


def test_forward_interrupt_stops():
    task = mission()
    with patch.object(nav.rclpy, 'ok', return_value=True), \
            patch.object(nav.time, 'sleep', side_effect=KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):
            task.drive_forward(0.3)
    assert task.cmd_pub.publish.call_args.args[0].linear.x == 0


@pytest.mark.parametrize('distance', [-1.0, float('nan'), float('inf')])
def test_invalid_forward_distance(distance):
    task = mission()
    with pytest.raises(ValueError):
        task.drive_forward(distance)


def test_wait_for_pose_drains_stale_callbacks_until_fresh():
    task = mission()
    reader = Mock(side_effect=[ValueError('stale'), ValueError('stale'), (1, 2, 0)])
    with patch.object(nav.rclpy, 'ok', return_value=True), \
            patch.object(nav.rclpy, 'spin_once') as spin, \
            patch.object(nav.time, 'sleep') as sleep:
        assert task.wait_for_pose(reader) == (1, 2, 0)
    assert spin.call_count == 3
    sleep.assert_not_called()
    assert task.cmd_pub.publish.call_count == 2


def test_before_between_timeout_cancels_then_continues():
    task = mission()
    task.config['nav_timeout'] = 1.0
    task.navigator.get_clock().now().to_msg.return_value = nav.Time().to_msg()
    task.navigator.isTaskComplete.side_effect = [False, False, True]
    with patch.object(nav.rclpy, 'ok', return_value=True), \
            patch.object(nav.rclpy, 'spin_once'), \
            patch.object(nav.time, 'monotonic', side_effect=[0, 2, 2, 3]):
        task.navigate('Before_between')
    task.navigator.cancelTask.assert_called_once()
    task.navigator.get_logger().warn.assert_called_once()


def test_before_between_does_not_continue_if_cancel_unconfirmed():
    task = mission()
    task.config['nav_timeout'] = 1.0
    task.navigator.get_clock().now().to_msg.return_value = nav.Time().to_msg()
    task.navigator.isTaskComplete.return_value = False
    with patch.object(nav.rclpy, 'ok', return_value=True), \
            patch.object(nav.time, 'monotonic', side_effect=[0, 2, 2, 8]):
        with pytest.raises(RuntimeError, match='仍未結束'):
            task.navigate('Before_between')
