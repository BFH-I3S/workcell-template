"""cell_description.composition on the workspace's own cells (source tree).

Needs ROS and the install space sourced: the cells reference installed files
(ur_moveit_config's joint limits)."""

import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from cell_description import composition  # noqa: E402

SOURCE_CELLS = composition.list_cells(ROOT)


def needs(*cells, skill_base=False):
    """Skip where the cells are not in the source tree or skill_base is missing (the public repository)."""
    missing = [c for c in cells if c not in SOURCE_CELLS]
    if skill_base:
        import importlib.util
        if importlib.util.find_spec("skill_base") is None:
            missing.append("skill_base")
    return pytest.mark.skipif(bool(missing), reason=f"needs {', '.join(missing)}")


def test_merge_rules():
    base = {"a": {"x": 1, "y": [1, 2]}, "b": 2, "c": 3}
    merged = composition.merge(base, {"a": {"y": [3]}, "b": None, "d": 4})
    assert merged == {"a": {"x": 1, "y": [3]}, "c": 3, "d": 4}
    assert base["a"]["y"] == [1, 2]                   # inputs untouched


@pytest.fixture
def gantry():
    if "gantry" not in SOURCE_CELLS:
        pytest.skip("needs gantry")
    return composition.Cell("gantry", ROOT)


def test_group_from_modules_takes_their_joints_in_order(gantry):
    groups = gantry.variant("sim").groups()
    assert list(groups)[0] == "ur_manipulator"        # RViz's default group
    assert groups["ur16e_on_gantry"]["joints"] == [
        "linear_actuator_to_linear_actuator_plate_joint", "shoulder_pan_joint", "shoulder_lift_joint",
        "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    # the cell extends a module group without replacing it
    assert groups["ur_manipulator"]["chain"] == ["base_link", "tool0"]
    assert groups["ur_manipulator"]["kinematics"]["kinematics_solver_timeout"] == 0.05
    assert {"home", "arm_home"} <= set(groups["ur_manipulator"]["states"])


def test_controllers_per_variant(gantry):
    sim = gantry.variant("sim").moveit_controllers()["moveit_simple_controller_manager"]
    assert sim["controller_names"] == ["scaled_joint_trajectory_controller", "linear_actuator_controller",
                                       "gripper_controller"]
    real = gantry.variant("real").moveit_controllers()["moveit_simple_controller_manager"]
    assert real["parker_linear_actuator"]["default"] and not real["linear_actuator_controller"]["default"]
    assert gantry.variant("real").ros2_controllers() is None   # the real manager is started elsewhere


def test_controller_manager_file_points_at_its_parameters(gantry):
    params = gantry.variant("sim").ros2_controllers()
    manager = params["controller_manager"]["ros__parameters"]
    assert manager["robotiq_gripper_controller"]["params_file"] == ["$(this)"]
    assert params["robotiq_gripper_controller"]["ros__parameters"]["joints"] == ["robotiq_85_left_knuckle_joint"]
    assert manager["scaled_joint_trajectory_controller"]["params_file"] == [
        "$(find-pkg-share ur_simulation_gz)/config/ur_controllers.yaml",
        "$(find-pkg-share manipulation_skills)/config/sim_force_controllers.yaml"]
    assert manager["forward_velocity_controller"]["type"] == "forward_command_controller/ForwardCommandController"
    assert "tcp_fts_raw_broadcaster" in manager


@needs("acroba")
def test_cell_override_replaces_params_files():
    acroba = composition.Cell("acroba", ROOT).variant("sim")
    controllers = acroba.controllers()
    assert len(controllers["scaled_joint_trajectory_controller"]["params_files"]) == 2
    assert controllers["scaled_joint_trajectory_controller"]["moveit"] == {"default": True}   # kept from the module
    assert "tcp_fts_raw_broadcaster" in controllers


def test_srdf_has_end_effector_on_the_arm(gantry):
    robot = gantry.variant("sim").srdf_without_collisions("gantry_system")
    ee = robot.find("end_effector")
    assert (ee.get("group"), ee.get("parent_link"), ee.get("parent_group")) == ("gripper", "tool0", "ur_manipulator")
    assert robot.find("virtual_joint").get("child_link") == "floor_link"


def test_resolve_tokens():
    text = composition.resolve_tokens("$(find-pkg-share cell_description)/x.yaml $(this)", this="/tmp/f.yaml")
    assert text.endswith("/share/cell_description/x.yaml /tmp/f.yaml")


def test_generated_files_parse():
    for cell in composition.list_cells(ROOT):
        for variant in composition.Cell(cell, ROOT).variant_names():
            directory = composition.Cell(cell, ROOT).variant(variant).generated_dir()
            for name in os.listdir(directory):
                if name.endswith(".yaml"):
                    with open(os.path.join(directory, name)) as handle:
                        assert yaml.safe_load(handle), f"{cell}/{variant}/{name}"


@needs("acroba", "gantry")
def test_module_capabilities_merge_per_cell_and_variant():
    acroba = composition.Cell("acroba", ROOT).variant("sim").modules()
    gantry_sim = composition.Cell("gantry", ROOT).variant("sim").modules()
    gantry_real = composition.Cell("gantry", ROOT).variant("real").modules()
    # descriptor + model
    assert acroba["arm"]["description"]["capabilities"]["payload"] == 5.0
    assert gantry_sim["arm"]["description"]["capabilities"]["payload"] == 16.0
    # cell override on the instance, and the variant override on top
    assert acroba["arm"]["description"]["capabilities"]["upright"] is True
    assert gantry_real["arm"]["description"]["capabilities"]["upright"] is False
    assert gantry_real["arm"]["description"]["capabilities"]["force_sensing"] is True
    assert gantry_sim["arm"]["description"]["capabilities"]["force_sensing"] is True
    # unverified accumulates: the descriptor's, plus the variant's
    assert gantry_sim["axis"]["description"]["unverified"] == []
    assert gantry_real["axis"]["description"]["unverified"] == ["travel", "velocity"]
    # borders: the instance's name plus the descriptor's suffix, the parent from the
    # cell, a pass-through output from the cell's instance; groups built from modules
    assert gantry_sim["gripper"]["description"]["frames"] == {"mount": "gripper_mount", "tcp": "gripper_tcp"}
    assert gantry_sim["gripper"]["description"]["parent"] == "changer_coupling"
    assert gantry_sim["camera"]["description"]["frames"] == {"mount": "camera_mount", "base": "d405_link",
                                                             "output": "camera_output"}
    assert gantry_sim["arm"]["description"]["frames"]["flange"] == "tool0"
    assert "ur16e_on_gantry" in gantry_sim["axis"]["description"]["groups"]


@needs(skill_base=True)
def test_module_announcements_are_valid():
    for cell in composition.list_cells(ROOT):
        c = composition.Cell(cell, ROOT)
        for variant in c.variant_names():
            assert c.variant(variant).module_problems() == [], f"{cell}/{variant}"


@needs("gantry", skill_base=True)
def test_module_problems_are_reported():
    gantry = composition.Cell("gantry", ROOT)
    gantry.specs["arm"] = {**gantry.specs["arm"], "model": "ur5e"}                 # ur_type is ur16e
    gantry.specs["gripper"] = {**gantry.specs["gripper"], "capabilities": {"stroke": 0.085}}
    problems = gantry.variant("sim").module_problems()
    assert any("ur_type is ur16e" in p for p in problems)
    assert any("unknown capability 'stroke'" in p for p in problems)


@needs("gantry", skill_base=True)
def test_module_status_from_observations():
    from skill_base.module_announcement import STATUS_NOT_WORKING, STATUS_WORKING
    from cell_description.module_announcer import module_status
    modules = composition.Cell("gantry", ROOT).variant("sim").modules()
    axis, gripper = modules["axis"]["liveness"], modules["gripper"]["liveness"]
    joint = "linear_actuator_to_linear_actuator_plate_joint"
    fresh = {joint: 10.0}
    assert module_status(axis, 11.0, 2.0, fresh, {"linear_actuator_controller"}, {}) == (STATUS_WORKING, "")
    status, reason = module_status(axis, 11.0, 2.0, fresh, set(), {})
    assert status == STATUS_NOT_WORKING and "controllers not active: ['linear_actuator_controller']" in reason
    status, reason = module_status(axis, 11.0, 2.0, {}, {"linear_actuator_controller"}, {})
    assert status == STATUS_NOT_WORKING and "joints not on /joint_states" in reason
    status, reason = module_status(axis, 20.0, 2.0, fresh, None, {})
    assert "no joint states for 2 s" in reason and "controller manager not available" in reason
    assert module_status(gripper, 11.0, 2.0, {}, None, {"/gripper/position": 10.5})[0] == STATUS_WORKING
    assert "nothing on /gripper/position" in module_status(gripper, 13.0, 2.0, {}, None,
                                                           {"/gripper/position": 10.5})[1]


@needs("acroba", "robocoop")
def test_gripper_driver_comes_from_the_gripper_module():
    assert composition.gripper_driver("acroba", ROOT) == {
        "model": "2f_85", "joint": "robotiq_85_left_knuckle_joint", "closed_position": 0.7929,
        "stroke": 0.085, "goal_tolerance": 0.02}
    hande = composition.gripper_driver("robocoop", ROOT)
    assert (hande["model"], hande["joint"], hande["closed_position"], hande["stroke"]) == \
        ("hand_e", "hande_left_finger_joint", 0.025, 0.05)


@needs("acroba", "gantry", "robocoop")
def test_gripper_geometry_per_cell():
    two_f = composition.gripper_geometry("acroba", ROOT)
    assert (two_f["gripper.max_stroke"], two_f["gripper.max_gap"], two_f["gripper.gap_per_stroke"],
            two_f["gripper.open_threshold"], two_f["gripper.closed_threshold"],
            two_f["gripper.finger_joint"], two_f["gripper.closed_position"]) == \
        (0.085, 0.085, 0.91, 0.010, 0.080, "robotiq_85_left_knuckle_joint", 0.7929)
    assert composition.gripper_geometry("gantry", ROOT) == two_f
    hande = composition.gripper_geometry("robocoop", ROOT)
    assert (hande["gripper.max_stroke"], hande["gripper.max_gap"], hande["gripper.gap_per_stroke"],
            hande["gripper.finger_joint"], hande["gripper.closed_position"], hande["gripper.finger_reach"]) == \
        (0.05, 0.05, 1.0, "hande_left_finger_joint", 0.025, 0.012)
    assert hande["gripper.open_threshold"] < 0.010 < hande["gripper.closed_threshold"] < 0.05


@needs("gantry", "gantry_gen3", "gantry_abb", "gantry_irb1200")
def test_an_arms_force_control_is_declared_per_variant():
    from cell_description.composition import arm_force_control, force_control_of
    assert arm_force_control("gantry", "real")["node"] == "ur_force_control"
    assert arm_force_control("gantry", "sim")["controller"] == "forward_velocity_controller"
    assert arm_force_control("gantry_gen3", "sim")["controller"] == "forward_position_controller"
    # Kinova's real arm serves none yet: a variant not listed has no force control.
    assert arm_force_control("gantry_gen3", "real") is None
    assert arm_force_control("gantry_abb", "sim")["controller"] == "forward_velocity_controller"
    assert arm_force_control("gantry_irb1200", "sim")["controller"] == "forward_velocity_controller"
    assert force_control_of({}, "sim") is None


@needs("gantry_gen3")
def test_the_capability_follows_the_declaration():
    from cell_description.composition import Cell
    arm = Cell("gantry_gen3").variant("sim").modules()["arm"]
    assert arm["description"]["capabilities"]["force_control"] is True


@needs("gantry_gen3", "gantry_abb", "gantry_irb1200")
def test_an_arm_names_the_wrapper_a_description_mounts():
    from cell_description.composition import Cell
    for cell, macro in (("gantry_gen3", "kinova_gen3_module"), ("gantry_abb", "abb_irb1100_module"),
                        ("gantry_irb1200", "abb_irb1200_module")):
        arm = Cell(cell).descriptors["arm"]
        assert arm["wrapper"]["macro"] == macro
        assert (arm["wrapper"].get("wrist_sensor_in_carrier", False)) == (cell == "gantry_gen3")

