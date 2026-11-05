"""The composition of the example workcells, without ROS."""

import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
from workcell_template import composition  # noqa: E402


def test_example_workcells_are_listed():
    assert set(composition.list_workcells(ROOT)) == {"ur5e_2f85", "ur3e_hande"}


def test_border_names_are_the_instances():
    modules = composition.Workcell("ur5e_2f85", ROOT).variant("real").modules()
    assert modules["gripper"]["frames"] == {"mount": "gripper_mount", "tcp": "gripper_tcp"}
    assert modules["arm"]["frames"] == {"mount": "arm_mount", "base": "base_link", "flange": "tool0"}
    assert modules["gripper"]["parent"] == "tool0"


def test_a_second_instance_gets_its_own_borders():
    descriptor = {"frames": {"mount": "_mount", "tcp": "_tcp"}}
    frames = composition.border_frames("left_gripper", descriptor, {})
    assert frames == {"mount": "left_gripper_mount", "tcp": "left_gripper_tcp"}


def test_kinds_and_roles_are_valid():
    assert composition.Workcell("ur3e_hande", ROOT).variant("real").module_problems() == []
