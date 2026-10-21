"""Offline regression checks for scoped, heading-free between navigation."""

import ast
from copy import deepcopy
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from path_planner_server.position_navigation import (
    POSITION_CHECKER, POSITION_CONTROLLER, POSITION_PLANNER,
    controller_launch_overrides, navigator_launch_overrides,
    planner_launch_overrides,
)

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'src/path_planner_server/config'
ROBOT = ROOT / 'vlmapsSrc/vlmaps/robot/tracer_lang_robot.py'


def test_position_profile_inherits_current_mppi_and_xy_tolerance():
    original = yaml.safe_load((CONFIG / 'controller.yaml').read_text())[
        'controller_server']['ros__parameters']
    before = deepcopy(original)
    overrides = controller_launch_overrides(CONFIG / 'controller.yaml')
    expected = deepcopy(original['FollowPath'])
    expected['GoalAngleCritic']['enabled'] = False
    assert overrides[POSITION_CONTROLLER] == expected
    assert overrides[POSITION_CHECKER]['xy_goal_tolerance'] == (
        original['general_goal_checker']['xy_goal_tolerance'])
    assert overrides[POSITION_CHECKER]['yaw_goal_tolerance'] > math.pi
    assert overrides[POSITION_CHECKER]['stateful'] is False
    assert original == before
    assert overrides['controller_plugins'] == ['FollowPath', POSITION_CONTROLLER]
    assert overrides['goal_checker_plugins'] == [
        'general_goal_checker', POSITION_CHECKER]


def test_position_tree_selects_all_three_matching_profiles():
    tree = ET.parse(CONFIG / 'navigate_to_position.xml')
    planner = planner_launch_overrides(CONFIG / 'planner_server.yaml')
    controller = controller_launch_overrides(CONFIG / 'controller.yaml')
    for node in tree.iter('ComputePathToPose'):
        assert node.attrib['planner_id'] in planner['planner_plugins']
        assert node.attrib['planner_id'] == POSITION_PLANNER
    for node in tree.iter('FollowPath'):
        assert node.attrib['controller_id'] in controller['controller_plugins']
        assert node.attrib['goal_checker_id'] in controller['goal_checker_plugins']
        assert node.attrib['controller_id'] == POSITION_CONTROLLER
        assert node.attrib['goal_checker_id'] == POSITION_CHECKER
    profile = planner[POSITION_PLANNER]
    assert profile['plugin'] == 'nav2_footprint_planner/StrictHybridPlanner'
    assert profile['allow_unknown'] is False
    assert profile['local_costmap_topic'] == '/local_costmap/costmap_raw'
    assert profile['goal_tolerance'] == .025
    # Preserve the user's current experiment switch; launch must not force it.
    configured = yaml.safe_load((CONFIG / 'planner_server.yaml').read_text())[
        'planner_server']['ros__parameters']['PositionOnlyGrid']
    assert profile['reuse_path_enabled'] == configured['reuse_path_enabled']
    assert profile['reuse_max_age'] == 3.0


def test_ordinary_trees_keep_heading_aware_navigation_explicit():
    paths = navigator_launch_overrides(CONFIG)
    for filename in paths.values():
        tree = ET.parse(filename)
        for node in tree.iter('FollowPath'):
            assert node.attrib['controller_id'] == 'FollowPath'
            assert node.attrib['goal_checker_id'] == 'general_goal_checker'
        for tag in ['ComputePathToPose', 'ComputePathThroughPoses']:
            for node in tree.iter(tag):
                assert node.attrib['planner_id'] == 'GridBased'


def _method(name, namespace):
    # Load only the real method; no ROS nodes, models or camera imports.
    cls = next(node for node in ast.parse(ROBOT.read_text()).body
               if isinstance(node, ast.ClassDef) and node.name == 'TracerLangRobot')
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(ROBOT), 'exec'),
         namespace)
    return namespace[name]


def _pose(x=0., y=0., yaw=0.):
    return NS(pose=NS(position=NS(x=x, y=y, z=0.), orientation=NS(
        x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))


@pytest.mark.parametrize('initial_yaw', [0., math.pi/2, -2.4])
def test_between_uses_position_bt_even_when_initial_and_path_yaw_differ(
        monkeypatch, initial_yaw):
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(CONFIG.parent)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    requested, navigated = [], []

    def get_path(start, goal, planner_id, use_start):
        requested.append((deepcopy(goal), planner_id))
        if len(requested) == 1:
            return None
        return NS(poses=[_pose(.01, 5., yaw=-.8)])

    robot = NS(
        between_robot_radius_m=.25, between_clearance_m=.05,
        map=NS(gs=1000, cs=.05), curr_ang_deg_on_map=math.degrees(initial_yaw),
        current_world_pose=_pose(yaw=initial_yaw), _set_nav_curr_pose=lambda: None,
        get_pos=lambda name: [400, 490] if name == 'laptop' else [400, 510],
        _make_goal=lambda pos: _pose((pos[1]-500)*.05, -(pos[0]-500)*.05),
        navigator=NS(getPath=get_path),
        _navigate_and_wait=lambda goal, behavior_tree: (
            navigated.append((deepcopy(goal), behavior_tree)) or 'SUCCEEDED'),
    )
    method = _method('move_in_between', {'np': np, 'os': __import__('os')})
    assert method(robot, 'laptop', 'cardboard box', max_path_checks=2) == 'SUCCEEDED'
    assert len(requested) == 2
    assert all(planner == POSITION_PLANNER for _, planner in requested)
    goal, bt = navigated[0]
    assert (goal.pose.position.x, goal.pose.position.y) == (.01, 5.)
    assert Path(bt) == CONFIG / 'navigate_to_position.xml'


def test_navigation_passes_bt_and_preserves_failure_handling(monkeypatch):
    rclpy = ModuleType('rclpy')
    rclpy.ok = lambda: True
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    results = NS(SUCCEEDED=1, FAILED=2)
    calls, canceled = [], []
    nav = NS(goToPose=lambda goal, **kwargs: calls.append(kwargs),
             cancelTask=lambda: canceled.append(True))
    robot = NS(navigator=nav, wait_fn=lambda _: results.SUCCEEDED)
    method = _method('_navigate_and_wait', {
        'TaskResult': results, 'time': __import__('time')})
    assert method(robot, _pose(), behavior_tree='position.xml') == results.SUCCEEDED
    assert calls[-1] == {'behavior_tree': 'position.xml'}
    assert method(robot, _pose()) == results.SUCCEEDED
    assert calls[-1] == {}
    robot.wait_fn = lambda _: results.FAILED
    with pytest.raises(RuntimeError, match='導航未成功'):
        method(robot, _pose(), behavior_tree='position.xml')
    assert canceled == [True]


def test_strict_search_and_mppi_use_the_same_padded_footprint():
    global_params = yaml.safe_load((CONFIG / 'planner_server.yaml').read_text())[
        'global_costmap']['global_costmap']['ros__parameters']
    local_params = yaml.safe_load((CONFIG / 'controller.yaml').read_text())[
        'local_costmap']['local_costmap']['ros__parameters']
    assert ast.literal_eval(global_params['footprint']) == ast.literal_eval(
        local_params['footprint'])
    assert global_params['footprint_padding'] == local_params['footprint_padding']
