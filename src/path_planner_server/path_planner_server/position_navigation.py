"""Scoped Nav2 profiles for goals that specify a position but no final heading."""

from copy import deepcopy
import math
from pathlib import Path

import yaml


POSITION_PLANNER = 'PositionOnlyGrid'
POSITION_CONTROLLER = 'FollowPosition'
POSITION_CHECKER = 'position_only_goal_checker'
POSITION_TREE = 'navigate_to_position.xml'


def _parameters(filename, node):
    with open(filename, encoding='utf-8') as stream:
        return yaml.safe_load(stream)[node]['ros__parameters']


def controller_launch_overrides(filename):
    """Clone current MPPI settings; keep the usual heading-aware profile intact."""
    params = _parameters(filename, 'controller_server')
    controller = deepcopy(params['FollowPath'])
    controller.setdefault('GoalAngleCritic', {})['enabled'] = False
    checker = deepcopy(params['general_goal_checker'])
    checker['yaw_goal_tolerance'] = math.pi + 1e-6
    checker['stateful'] = False
    return {
        'controller_plugins': list(dict.fromkeys(
            [*params['controller_plugins'], POSITION_CONTROLLER])),
        'goal_checker_plugins': list(dict.fromkeys(
            [*params['goal_checker_plugins'], POSITION_CHECKER])),
        POSITION_CONTROLLER: controller,
        POSITION_CHECKER: checker,
    }


def planner_launch_overrides(filename):
    """Select the dedicated swept-footprint search only for between goals."""
    params = _parameters(filename, 'planner_server')
    profile = deepcopy(params[POSITION_PLANNER])
    if profile.get('plugin') != 'nav2_footprint_planner/StrictHybridPlanner':
        raise ValueError('PositionOnlyGrid must use the strict footprint planner')
    return {
        'planner_plugins': list(dict.fromkeys(
            [*params['planner_plugins'], POSITION_PLANNER])),
        POSITION_PLANNER: profile,
    }


def navigator_launch_overrides(config_dir):
    """Name the normal goal checker explicitly when multiple checkers exist."""
    config_dir = Path(config_dir)
    return {
        'default_nav_to_pose_bt_xml': str(
            config_dir / 'navigate_to_pose_with_goal_checker.xml'),
        'default_nav_through_poses_bt_xml': str(
            config_dir / 'navigate_through_poses_with_goal_checker.xml'),
    }
