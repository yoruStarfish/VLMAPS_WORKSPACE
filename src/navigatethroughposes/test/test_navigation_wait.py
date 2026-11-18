"""Keep localization subscriptions running while waiting for Nav2 actions."""
import unittest
from unittest.mock import Mock, patch

from rclpy.time import Time
from navigatethroughposes import navi_using_api_vlmap as module


class NavigationWaitTests(unittest.TestCase):
    def test_wait_processes_localization_and_navigator(self):
        navigator, localization = Mock(), Mock()
        navigator.isTaskComplete.side_effect = [False, False, True]
        navigator.get_clock().now.return_value = Time(seconds=100)
        with patch.object(module, '_localization_node', localization), \
                patch.object(module.rclpy, 'spin_once') as spin:
            module.wait_for_task(navigator)
        self.assertEqual([call.args[0] for call in spin.call_args_list],
                         [localization, navigator, localization, navigator])
        navigator.cancelTask.assert_not_called()

    def test_timeout_still_cancels_action(self):
        navigator = Mock()
        navigator.isTaskComplete.return_value = False
        navigator.get_clock().now.side_effect = [Time(seconds=0), Time(seconds=181)]
        with patch.object(module, '_localization_node', Mock()), \
                patch.object(module.rclpy, 'spin_once'):
            module.wait_for_task(navigator)
        navigator.cancelTask.assert_called_once()


if __name__ == '__main__':
    unittest.main()
