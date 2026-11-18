"""Offline command validation and LLM request tests; no robot is contacted."""
import ast
import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from navigatethroughposes import llm_brain as brain


class BrainTests(unittest.TestCase):
    def test_near_call_and_optional_distances(self):
        for code in (
            'robot.move_to_object_near("paper cup", "laptop")',
            'robot.move_to_object_near(name="paper cup", reference_name="laptop")',
            'robot.move_to_object_near("paper cup", "laptop", safe_distance_m=0.5)',
            'robot.move_to_object_near("paper cup", "laptop", 0.8, 0.45)',
            'robot.move_to_object_near("bottle", "kettle", near_distance_m=0.6)\n'
            'robot.turn(-90)',
        ):
            with self.subTest(code=code):
                self.assertEqual(brain.validate_robot_code(code), ast.unparse(ast.parse(code)))

    def test_missing_or_invalid_reference_is_rejected(self):
        for code in (
            'robot.move_to_object_near("paper cup")',
            'robot.move_to_object_near(reference_name="laptop")',
            'robot.move_to_object_near("paper cup", "television")',
            'robot.move_to_object_near("paper cup next to the laptop", "laptop")',
            'robot.move_to_object_near("paper cup", "paper cup")',
            'robot.move_to_object_near("paper cup", robot.face("laptop"))',
            'robot.move_to_object_near("paper cup", 1)',
        ):
            with self.subTest(code=code), self.assertRaises(ValueError):
                brain.validate_robot_code(code)

    def test_invalid_near_distances_are_rejected(self):
        for parameter in ('near_distance_m', 'safe_distance_m'):
            for value in ('0', '-0.1', '1e309', 'True', '"0.5"', 'distance'):
                code = f'robot.move_to_object_near("paper cup", "laptop", {parameter}={value})'
                with self.subTest(code=code), self.assertRaises(ValueError):
                    brain.validate_robot_code(code)

    def test_unknown_duplicate_or_excess_parameters_are_rejected(self):
        for code in (
            'robot.move_to_object_near("paper cup", "laptop", reference_name="kettle")',
            'robot.move_to_object_near("paper cup", "laptop", unexpected=1)',
            'robot.move_to_object_near("paper cup", "laptop", 0.8, 0.45, 1)',
            'robot.move_to_object_near("paper cup", **options)',
        ):
            with self.subTest(code=code), self.assertRaises(ValueError):
                brain.validate_robot_code(code)

    def test_existing_api_and_all_examples_still_validate(self):
        for code in ('robot.move_to_object("chair")',
                     'robot.move_in_between("laptop", "cardboard box")',
                     'robot.move_to_left("table")', 'robot.move_forward(1)',
                     'robot.turn(-90)'):
            brain.validate_robot_code(code)
        for message in brain.EXAMPLE_MESSAGES:
            if message['role'] == 'assistant':
                brain.validate_robot_code(message['content'])

    def test_invalid_step_rejects_entire_sequence(self):
        with self.assertRaises(ValueError):
            brain.validate_robot_code(
                'robot.turn(90)\nrobot.move_to_object_near("paper cup", "unknown")')

    def test_streamed_near_command_is_returned_with_relation_prompt(self):
        create = Mock(return_value=[
            SimpleNamespace(choices=[SimpleNamespace(delta=SimpleNamespace(content=part))])
            for part in ('robot.move_to_object_near(', '"paper cup", "laptop")')
        ])
        instruction = 'Move to the paper cup next to the laptop.'
        with patch.object(brain.client.chat.completions, 'create', create), \
                contextlib.redirect_stdout(io.StringIO()):
            code = brain.get_code_from_llm(instruction)
        self.assertEqual(code, "robot.move_to_object_near('paper cup', 'laptop')")
        create.assert_called_once()
        messages = create.call_args.kwargs['messages']
        self.assertIn('move_to_object_near', messages[0]['content'])
        self.assertEqual(messages[-1], {'role': 'user', 'content': instruction})


if __name__ == '__main__':
    unittest.main()
