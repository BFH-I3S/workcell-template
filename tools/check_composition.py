#!/usr/bin/env python3
"""
Check how the work cells are composed, before anything is launched.

A cell's robot is assembled from third-party descriptions, this workspace's
wrappers, an SRDF, joint limits, controller configurations and, in
simulation, worlds and a gripper driver. Each piece can be right on its own
and the cell still broken at a seam between them. This script checks those
seams for every variant of every cell composition
(src/<package>/cells/*.yaml), as MoveIt plans with it and, where the
variant is simulated, as Gazebo runs it:

  generated    the generated SRDF and configuration files are up to date with
               the composition (tools/generate_cell.py --check)
  modules      each module's announcement (capabilities, frames, model) follows
               skill_base/capabilities.py; the arm's model matches ur_type
  names        URDF and SRDF describe the same robot; SRDF groups exist
  limits       every controlled joint has velocity and acceleration limits
  controllers  MoveIt's list is in the right layout, covers every planning
               joint with a default controller, and agrees with the joints the
               controllers really drive; controller types exist as plugins
  frames       workcell exists, is fixed in the cell, z up; the TCP sits where
               its module config says, or within MAX_TCP_CORRECTION of it (a
               cell's measured calibration)
  borders      the cell template (cell_description/README.md): each module declares the
               output borders its module class must offer; its mount exists, hangs on the parent the cell names, and on another
               module's output border keeps its z and lies on it (offset
               along z only); its output borders lie below it, and a TCP or an
               arm's base not behind a revolute joint keeps its z, other outputs
               may point any way; borders carry no geometry
  reference    code outside a module names no frame inside it, only borders
               (the cell template's reference rule); a file that may, a module's
               own driver or simulator, is listed in REFERENCE_EXCEPTIONS with why
  collisions   no pair of robot links that touch is missing from the SRDF's
               disabled collisions (MoveIt's collisions_updater; slow)
  grasp        graspable models in the worlds, grasp joints in the description
               and the gripper driver's list agree
  carriers     a joint that carries the rest of the robot can push its weight
  vendored     third-party packages are vanilla (tools/check_vendored.py)

What it cannot see are run-time faults (start-up order, a joint that stops
following); those remain for the simulation tests.

Usage (from the workspace root, after colcon build and sourcing install/):
    python3 tools/check_composition.py                 all cells, all checks
    python3 tools/check_composition.py --cell gantry
    python3 tools/check_composition.py --fast          without collisions
Exit status 1 if any check reports an error.
"""

import argparse
import ast
import importlib.util
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np
import yaml
from ament_index_python import get_resource, get_resources

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CELL_DIR = os.path.join(ROOT, "src", "cell_description")
sys.path.insert(0, CELL_DIR)
from cell_description import composition  # noqa: E402  (the source tree's)
GRAVITY = 9.81


# ----------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------

class Report:
    def __init__(self):
        self.counts = defaultdict(int)

    def __call__(self, level, where, message):
        self.counts[level] += 1
        mark = {"OK": "  ok ", "WARN": " WARN", "ERROR": "ERROR"}[level]
        print(f"{mark}  {where:28s} {message}")


# ----------------------------------------------------------------------
# Files
# ----------------------------------------------------------------------

share_path = composition.ref_path


def load_yaml(ref):
    with open(share_path(ref)) as handle:
        return yaml.safe_load(handle) or {}


def expand_xacro(ref, out_dir, name):
    args = [f"{k}:={v}" for k, v in (ref.get("args") or {}).items()]
    result = subprocess.run(["xacro", share_path(ref), *args], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro {share_path(ref)} failed: {result.stderr.strip()[:300]}")
    path = os.path.join(out_dir, name)
    with open(path, "w") as handle:
        handle.write(result.stdout)
    return path


# ----------------------------------------------------------------------
# URDF / SRDF models
# ----------------------------------------------------------------------

def _origin(element):
    origin = element.find("origin") if element is not None else None
    xyz = [float(v) for v in ((origin.get("xyz") if origin is not None else None) or "0 0 0").split()]
    rpy = [float(v) for v in ((origin.get("rpy") if origin is not None else None) or "0 0 0").split()]
    return np.array(xyz), np.array(rpy)


def _rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


class Urdf:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.name = self.root.get("name")
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.parent_joint = {j.find("child").get("link"): j for j in self.joints.values()}
        self.children = defaultdict(list)
        for j in self.joints.values():
            self.children[j.find("parent").get("link")].append(j)

    def is_active(self, joint_name):
        j = self.joints.get(joint_name)
        return j is not None and j.get("type") not in ("fixed", "floating") and j.find("mimic") is None

    def chain(self, base, tip):
        """Joint names from base to tip, or None if tip is not below base."""
        names, link = [], tip
        while link != base:
            j = self.parent_joint.get(link)
            if j is None:
                return None
            names.append(j.get("name"))
            link = j.find("parent").get("link")
        return list(reversed(names))

    def fixed_transform(self, base, link):
        """(R, p) of link in base if every joint between them is fixed, else None."""
        chain = self.chain(base, link)
        if chain is None:
            return None
        rotation, position = np.eye(3), np.zeros(3)
        for name in chain:
            j = self.joints[name]
            if j.get("type") != "fixed":
                return None
            xyz, rpy = _origin(j)
            position = position + rotation @ xyz
            rotation = rotation @ _rotation(rpy)
        return rotation, position

    def zero_pose(self, link):
        """(R, p) of link in the root, every joint at 0."""
        rotation, position = np.eye(3), np.zeros(3)
        names, current = [], link
        while current in self.parent_joint:
            names.append(self.parent_joint[current].get("name"))
            current = self.parent_joint[current].find("parent").get("link")
        for name in reversed(names):
            xyz, rpy = _origin(self.joints[name])
            position = position + rotation @ xyz
            rotation = rotation @ _rotation(rpy)
        return rotation, position

    def subtree_mass(self, link):
        total, stack = 0.0, [link]
        while stack:
            current = stack.pop()
            inertial = self.links[current].find("inertial") if current in self.links else None
            if inertial is not None and inertial.find("mass") is not None:
                total += float(inertial.find("mass").get("value"))
            stack.extend(j.find("child").get("link") for j in self.children[current])
        return total


class Srdf:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.name = self.root.get("name")
        self.groups = {g.get("name"): g for g in self.root.findall("group")}
        self.end_effector_groups = {e.get("group") for e in self.root.findall("end_effector")}

    def group_joints(self, name, urdf, seen=None):
        """Joint names of a group (joints, chains, subgroups); None for unknown parts."""
        seen = seen or set()
        group = self.groups.get(name)
        if group is None or name in seen:
            return None
        seen.add(name)
        joints = []
        for j in group.findall("joint"):
            joints.append(j.get("name"))
        for c in group.findall("chain"):
            chain = urdf.chain(c.get("base_link"), c.get("tip_link"))
            if chain is None:
                return None
            joints.extend(chain)
        for sub in group.findall("group"):
            sub_joints = self.group_joints(sub.get("name"), urdf, seen)
            if sub_joints is None:
                return None
            joints.extend(sub_joints)
        return joints


# ----------------------------------------------------------------------
# Checks
# ----------------------------------------------------------------------

def check_names(report, where, urdf, srdf):
    if urdf.name == srdf.name:
        report("OK", where, f"URDF and SRDF both describe '{urdf.name}'")
    else:
        report("ERROR", where, f"URDF robot '{urdf.name}' but SRDF robot '{srdf.name}' (MoveIt rejects the semantics)")
    for name, group in srdf.groups.items():
        missing = [j.get("name") for j in group.findall("joint") if j.get("name") not in urdf.joints]
        missing += [l.get("name") for l in group.findall("link") if l.get("name") not in urdf.links]
        missing += [f"chain {c.get('base_link')}->{c.get('tip_link')}" for c in group.findall("chain")
                    if urdf.chain(c.get("base_link"), c.get("tip_link")) is None]
        if missing:
            report("ERROR", where, f"SRDF group '{name}' names what the URDF does not have: {missing}")


def planning_joints(srdf, urdf, groups):
    joints = []
    for group in groups:
        names = srdf.group_joints(group, urdf)
        if names is None:
            return None, group
        joints.extend(j for j in names if urdf.is_active(j) and j not in joints)
    return joints, None


def check_limits(report, where, variant, urdf, srdf):
    joints, bad = planning_joints(srdf, urdf, variant.get("planning_groups", []))
    if joints is None:
        report("ERROR", where, f"planning group '{bad}' is missing or broken")
        return
    limits = {}
    for ref in variant.get("joint_limits", []):
        for name, entry in (load_yaml(ref).get("joint_limits") or {}).items():
            limits.setdefault(name, {}).update(entry or {})
    problems = []
    for name in joints:
        entry, urdf_limit = limits.get(name, {}), urdf.joints[name].find("limit")
        velocity = (entry.get("max_velocity") if entry.get("has_velocity_limits", True) else None) \
            or (float(urdf_limit.get("velocity")) if urdf_limit is not None and urdf_limit.get("velocity") else None)
        acceleration = entry.get("max_acceleration") if entry.get("has_acceleration_limits") else None
        if not velocity:
            problems.append(f"{name}: no velocity limit")
        if not acceleration:
            problems.append(f"{name}: no acceleration limit")
    if problems:
        report("ERROR", where, "time parameterisation will refuse trajectories: " + "; ".join(problems))
    else:
        report("OK", where, f"velocity and acceleration limits for all {len(joints)} planning joints")


def position_bounds(urdf, joint, overrides=None):
    """
    (lower, upper) of a joint as MoveIt bounds it: the URDF limit, narrowed by
    the safety controller's soft limits where there are any (MoveIt and
    ros2_control both take those), then joint_limits.yaml's position limits
    if it has any. None for a joint without position limits.
    """
    j = urdf.joints[joint]
    if j.get("type") == "continuous":
        bounds = None
    else:
        limit = j.find("limit")
        bounds = None if limit is None else [float(limit.get("lower", 0)), float(limit.get("upper", 0))]
        safety = j.find("safety_controller")
        if bounds is not None and safety is not None:
            bounds = [max(bounds[0], float(safety.get("soft_lower_limit", bounds[0]))),
                      min(bounds[1], float(safety.get("soft_upper_limit", bounds[1])))]
    entry = (overrides or {}).get(joint, {})
    if entry.get("has_position_limits"):
        bounds = [float(entry.get("min_position", -math.inf)), float(entry.get("max_position", math.inf))]
    return None if bounds is None else tuple(bounds)


def check_position_limits(report, where, moveit_bounds, urdf):
    """MoveIt must not plan a joint where the controllers (urdf: the description they run) will not go."""
    problems = []
    for joint, planned in sorted(moveit_bounds.items()):
        if joint not in urdf.joints:
            continue
        enforced = position_bounds(urdf, joint)
        if enforced is None:
            continue
        if planned is None or planned[0] < enforced[0] - 1e-6 or planned[1] > enforced[1] + 1e-6:
            shown = "unbounded" if planned is None else f"[{planned[0]:.4f}, {planned[1]:.4f}]"
            problems.append(f"{joint}: MoveIt plans {shown}, the controllers stop at "
                            f"[{enforced[0]:.4f}, {enforced[1]:.4f}]")
    if problems:
        report("ERROR", where, "a planned position the controllers clamp is never reached (a trajectory "
                               "without goal tolerances then never ends): " + "; ".join(problems))
    else:
        report("OK", where, f"position limits: MoveIt plans within what the controllers enforce "
                            f"({len(moveit_bounds)} joints)")


def installed_controller_types():
    types = set()
    for package in get_resources("controller_interface__pluginlib__plugin"):
        content, prefix = get_resource("controller_interface__pluginlib__plugin", package)
        for relative in content.split():
            path = os.path.join(prefix, relative)
            if not os.path.isfile(path):
                continue
            for cls in ET.parse(path).getroot().iter("class"):
                types.add(cls.get("name") or cls.get("type"))
    return types


def controller_params(refs):
    """{controller: joints} and {controller: type} from controller parameter files."""
    joints, types = {}, {}
    for ref in refs:
        data = load_yaml(ref)
        for name, entry in data.items():
            params = (entry or {}).get("ros__parameters", {})
            if name == "controller_manager":
                for controller, spec in params.items():
                    if isinstance(spec, dict) and spec.get("type"):
                        types[controller] = spec["type"]
            elif params.get("joints"):
                joints[name] = list(params["joints"])
    return joints, types


def check_controllers(report, where, variant, urdf, srdf, plugin_types):
    if "moveit_controllers" in variant:
        data = load_yaml(variant["moveit_controllers"])
        manager = data.get("moveit_simple_controller_manager")
        if not manager or not manager.get("controller_names"):
            report("ERROR", where, "MoveIt controller list has no moveit_simple_controller_manager.controller_names "
                                   "(move_group: 'No controller_names specified')")
        else:
            listed = {name: manager.get(name) or {} for name in manager["controller_names"]}
            real_joints, _ = controller_params(variant.get("controller_joints", []))
            problems = []
            for name, entry in listed.items():
                if not entry.get("joints"):
                    problems.append(f"{name} lists no joints")
                    continue
                unknown = [j for j in entry["joints"] if j not in urdf.joints]
                if unknown:
                    problems.append(f"{name} names joints not in the URDF: {unknown}")
                if name in real_joints and set(real_joints[name]) != set(entry["joints"]):
                    problems.append(f"{name}: MoveIt says {entry['joints']}, the controller drives {real_joints[name]}")
                elif real_joints and name not in real_joints and name not in variant.get("action_controllers", ()):
                    problems.append(f"{name} is not configured in any controller parameter file")
            joints, _ = planning_joints(srdf, urdf, variant.get("planning_groups", []))
            for joint in joints or []:
                defaults = [n for n, e in listed.items() if e.get("default") and joint in (e.get("joints") or [])]
                if len(defaults) != 1:
                    problems.append(f"{joint} has {len(defaults)} default controllers (needs exactly one)")
            if problems:
                report("ERROR", where, "controllers: " + "; ".join(problems))
            else:
                report("OK", where, f"MoveIt controllers {list(listed)} cover the planning joints")
    if variant.get("controller_types"):
        _, types = controller_params(variant["controller_types"])
        missing = {name: t for name, t in types.items() if t not in plugin_types}
        if missing:
            report("ERROR", where, f"controller types not installed: {missing}")
        else:
            report("OK", where, f"all {len(types)} configured controller types are installed plugins")


# How far a cell's measured calibration may move the TCP from its module config (m).
MAX_TCP_CORRECTION = 0.01


def check_frames(report, where, root, tcp, urdf):
    if root not in urdf.links:
        report("ERROR", where, f"root link '{root}' not in the description")
        return
    if "workcell" not in urdf.links:
        report("ERROR", where, "no workcell frame (cell_description's workcell_frame macro)")
    else:
        fixed = urdf.fixed_transform(root, "workcell")
        if fixed is None:
            report("ERROR", where, f"workcell is not fixed to {root} (a moving joint lies between them)")
        elif fixed[0][2, 2] < 0.999:
            report("ERROR", where, f"workcell's z does not point up in {root} (z.z = {fixed[0][2, 2]:.3f})")
        else:
            report("OK", where, f"workcell fixed to {root}, z up")
    if tcp:
        expected = tcp["expected"]
        # Fixed joints between the mount and the TCP: the gripper module moves the
        # whole gripper by a cell's calibration through a link of its own.
        fixed = urdf.fixed_transform(tcp["parent"], tcp["frame"]) if tcp["frame"] in urdf.parent_joint else None
        if tcp["frame"] not in urdf.parent_joint:
            report("ERROR", where, f"no TCP frame '{tcp['frame']}'")
        elif fixed is None:
            report("ERROR", where, f"{tcp['frame']} is not fixed below {tcp['parent']}")
        else:
            xyz = fixed[1]
            expected_rotation = _rotation(np.asarray(expected["rpy"], dtype=float))
            rotation_ok = np.allclose(fixed[0], expected_rotation, atol=1e-6)
            # A cell may correct the TCP by what it measured (the gantry's real variant,
            # gantry_description config/tool_calibration.yaml): millimetres, never more
            # than max_tcp_correction.
            correction = np.asarray(xyz, dtype=float) - np.asarray(expected["xyz"], dtype=float)
            if np.allclose(xyz, expected["xyz"], atol=1e-6) and rotation_ok:
                report("OK", where, f"{tcp['frame']} at {[round(float(v), 4) for v in xyz]} from {tcp['parent']}, as the module config says")
            elif np.linalg.norm(correction) <= MAX_TCP_CORRECTION and rotation_ok:
                report("OK", where, f"{tcp['frame']} at {[round(float(v), 5) for v in xyz]} from {tcp['parent']}, the module "
                                    f"config's corrected by {[round(1000 * float(v), 2) for v in correction]} mm (calibration)")
            else:
                report("ERROR", where, f"{tcp['frame']} at {list(xyz)}, rotation {'as' if rotation_ok else 'not as'} configured; module config says "
                                       f"{expected['xyz']}/{expected['rpy']}")


# What each module class must offer others, by role (cell_description/README.md's table). `gripper` is the template's
# end effector so far.
REQUIRED_BORDERS = {
    "arm": ("base", "flange"),
    "carrier": ("output",),
    "changer": ("coupling",),
    "gripper": ("tcp",),
    "sensor": ("base",),
}


def missing_borders(module_class, frames):
    """The roles a module of this class must declare and does not. An unknown class has none to check."""
    return [role for role in REQUIRED_BORDERS.get(module_class, ()) if role not in frames]


def check_borders(report, where, modules, urdf):
    """The cell template's borders: every module's mount on its parent, its outputs below it."""
    outputs = {name: instance for instance, module in modules.items()
               for role, name in module["frames"].items() if role not in ("mount", "base")}
    for instance, module in modules.items():
        frames, parent = module["frames"], module.get("parent")
        for role in missing_borders(module.get("module_class"), frames):
            report("ERROR", where, f"{instance}: module class '{module.get('module_class')}' must declare the output border '{role}'")
        mount = frames.get("mount")
        if mount is None:
            report("WARN", where, f"{instance}: its descriptor declares no mount border")
            continue
        joint = urdf.parent_joint.get(mount)
        if joint is None:
            report("ERROR", where, f"{instance}: no mount border '{mount}'")
            continue
        problems = []
        on = joint.find("parent").get("link")
        if joint.get("type") != "fixed":
            problems.append(f"{mount} is not on a fixed joint")
        if parent and on != parent:
            problems.append(f"{mount} hangs on {on}, the cell says {parent}")
        if on in outputs:
            xyz, rpy = _origin(joint)
            if _rotation(rpy)[2, 2] < 0.9999:
                problems.append(f"{mount} is tilted against {on} (z.z {_rotation(rpy)[2, 2]:.4f})")
            if np.linalg.norm(xyz[:2]) > 1e-6:
                problems.append(f"{mount} is {1000 * np.linalg.norm(xyz[:2]):.1f} mm off {on}'s z axis")
        z_mount = urdf.zero_pose(mount)[0][:, 2]
        for role, name in frames.items():
            if role == "mount":
                continue
            chain = urdf.chain(mount, name)
            if chain is None:
                problems.append(f"{role} '{name}' is not below {mount}")
                continue
            # Other outputs may point any way, a side port for one. Skills approach along a TCP's z.
            if not (role == "tcp" or (role == "base" and module.get("module_class") == "arm")):
                continue
            if any(urdf.joints[j].get("type") in ("revolute", "continuous") for j in chain):
                continue
            if urdf.zero_pose(name)[0][:, 2] @ z_mount < 0.9999:
                problems.append(f"{role} '{name}' does not keep {mount}'s z axis")
        for role, name in frames.items():
            if role != "base" and name in urdf.links and any(
                    urdf.links[name].find(tag) is not None for tag in ("visual", "collision", "inertial")):
                problems.append(f"border '{name}' has geometry or mass")
        if problems:
            for problem in problems:
                report("ERROR", where, f"{instance}: {problem}")
        else:
            onto = f"{outputs[on]}'s {on}" if on in outputs else f"the cell's {on}"
            report("OK", where, f"{instance}: {mount} on {onto}, "
                                f"{', '.join(n for r, n in frames.items() if r != 'mount')} below it")


# The packages that use modules without being part of one.
REFERENCE_PACKAGES = ("skill_base", "condition_skill", "manipulation_skills", "perception_skills", "planning_agent")
# REP 199's frames, whichever arm: borders of the template or the REP's own.
REP199_FRAMES = {"base", "base_link", "flange", "tool0"}
# Spelled like a frame inside a module (Robotiq's own), but the planner's parameter and key, holding a border.
REFERENCE_NOT_FRAMES = {"tcp_link"}
# A module's own software may name its frames. Everything else goes through borders.
REFERENCE_EXCEPTIONS = {
    "manipulation_skills/manipulation_skills/force_mode_sim.py": "simulates the UR's force mode and wrist sensor",
    "perception_skills/perception_skills/ft_frame_calibration.py": "measures the UR's wrist sensor in its own frame",
    "perception_skills/perception_skills/hand_eye_calibration.py": "sets the joint behind the camera's mount",
}


INNER_FRAMES = set()   # of every description checked, for check_reference


def module_frames(modules, urdf):
    """The links inside the modules: on or below a mount, not a border of any module, not REP 199's."""
    borders = {name for module in modules.values() for name in module["frames"].values()}
    inside = set()
    for module in modules.values():
        todo = [module["frames"].get("mount")]
        while todo:
            link = todo.pop()
            if link in urdf.links:
                inside.add(link)
                todo.extend(j.find("child").get("link") for j in urdf.children[link])
    return inside - borders - REP199_FRAMES - REFERENCE_NOT_FRAMES


def named_frames(path, frames):
    """(line, frame) for each string literal of a Python file that is exactly one of these frames.

    A parameter's own name is not a frame (the planner's `tcp_link`)."""
    tree = ast.parse(open(path).read())
    parameter_names = set()
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and call.args and isinstance(call.args[0], ast.Constant) and (
                getattr(call.func, "attr", getattr(call.func, "id", "")) in
                ("declare_parameter", "get_parameter", "has_parameter", "p", "d", "DeclareLaunchArgument")):
            parameter_names.add(id(call.args[0]))
    return [(node.lineno, node.value) for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and node.value in frames and id(node) not in parameter_names]


def check_reference(report, frames, src=None):
    """Code outside a module refers to it by its borders only, never by a frame inside it."""
    src = src or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
    found = 0
    for package in REFERENCE_PACKAGES:
        for folder, dirs, files in os.walk(os.path.join(src, package)):
            dirs[:] = [d for d in dirs if d not in ("test", "__pycache__")]
            for name in files:
                path = os.path.join(folder, name)
                where = os.path.relpath(path, src)
                if not name.endswith(".py") or where in REFERENCE_EXCEPTIONS:
                    continue
                for line, frame in named_frames(path, frames):
                    report("ERROR", where, f"line {line} names '{frame}', a frame inside a module; "
                           "go through its borders (skill_base.robot_links)")
                    found += 1
    if not found:
        report("OK", "reference", f"no code outside a module names one of its {len(frames)} inner frames "
               f"({len(REFERENCE_EXCEPTIONS)} declared exceptions)")


def check_calibrations(report, where, variant, urdf):
    """Each module's measured values: the file exists, a pose in it is what the description has."""
    for calibration in variant.calibrations():
        instance, path, data = calibration["instance"], calibration["path"], calibration["data"]
        name = os.path.basename(path)
        if data is None:
            report("ERROR", where, f"{instance}: calibration file {path} is missing")
            continue
        for key, entry in data.items():
            if not isinstance(entry, dict):
                continue
            if entry.get("status") == "estimate":
                report("WARN", where, f"{instance}: {name} is an estimate, not a measurement")
            if not {"parent", "child", "xyz", "rpy"} <= set(entry):
                continue
            joint = urdf.parent_joint.get(entry["child"])
            if joint is None or joint.find("parent").get("link") != entry["parent"]:
                report("ERROR", where, f"{instance}: {name} places {entry['child']} on {entry['parent']}, "
                                       "the description has no such joint")
                continue
            xyz, rpy = _origin(joint)
            if np.abs(xyz - np.array(entry["xyz"])).max() > 1e-6 or np.abs(rpy - np.array(entry["rpy"])).max() > 1e-6:
                report("ERROR", where, f"{instance}: the description does not have the pose of {name}")
            else:
                report("OK", where, f"{instance}: {entry['child']} on {entry['parent']} as in {name}")
        if not any(isinstance(e, dict) and "child" in e for e in data.values()):
            report("OK", where, f"{instance}: calibration {name} present")


def driver_default_grasp_models():
    spec = importlib.util.find_spec("manipulation_skills.sim_gripper_driver")
    source = open(spec.origin).read()
    match = re.search(r'declare_parameter\(\s*"grasp_models",\s*\[(.*?)\]', source, re.S)
    return re.findall(r'"([^"]+)"', match.group(1)) if match else []


def check_grasp(report, where, variant, urdf):
    grasp = variant["grasp"]
    joints = {p.findtext("child_model") for g in urdf.root.findall("gazebo")
              for p in g.findall("plugin") if "DetachableJoint" in (p.get("name") or "")}
    params = load_yaml(grasp["driver_params"]).get("sim_gripper_driver", {}).get("ros__parameters", {})
    # [""] is a world without parts (sim_gripper_driver drops empty names).
    models = params.get("grasp_models")
    driver = {m for m in models if m} if models else set(driver_default_grasp_models())
    pattern = re.compile(grasp["graspable"] + r"$")
    in_worlds = set()
    for ref in grasp["worlds"]:
        world = ET.parse(share_path(ref)).getroot()
        names = [e.findtext("name") for e in world.iter("include")] + [m.get("name") for m in world.iter("model")]
        in_worlds |= {n for n in names if n and pattern.match(n)}
    problems = []
    if joints != driver:
        problems.append(f"grasp joints and driver list differ: only joints {sorted(joints - driver)}, "
                        f"only driver {sorted(driver - joints)}")
    if in_worlds - joints:
        problems.append(f"graspable in a world but no grasp joint: {sorted(in_worlds - joints)}")
    if problems:
        report("ERROR", where, "grasp: " + "; ".join(problems))
    else:
        report("OK", where, f"{len(in_worlds)} graspable models in the worlds, all with grasp joints "
                            f"({len(joints)} joints = driver list)")


def check_force_control(report, where, variant):
    """An arm's force_control: its node is installed, and in a simulation its controller is configured."""
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    controllers = (variant.ros2_controllers() or {}).get("controller_manager", {}).get("ros__parameters", {})
    for instance, descriptor in variant.cell.descriptors.items():
        if descriptor.get("module_class") != "arm":
            continue
        served = composition.force_control_of(descriptor, variant.name)
        if served is None:
            report("OK", where, f"{instance}: offers no force control here (declared, force skills refuse)")
            continue
        try:
            binary = os.path.join(get_package_prefix("manipulation_skills"), "lib", "manipulation_skills", served["node"])
        except PackageNotFoundError:
            report("WARN", where, "force control not checked: manipulation_skills is not built")
            continue
        if not os.path.exists(binary):
            report("ERROR", where, f"{instance}: force_control's node {served['node']} is not installed")
        elif variant.ros2_controllers() is not None and served.get("controller") not in controllers:
            report("ERROR", where, f"{instance}: force_control's controller {served.get('controller')} is not configured")
        else:
            report("OK", where, f"{instance}: force control by {served['node']} ({served.get('controller')})")


def check_carriers(report, where, variant, urdf):
    for name in variant.get("carriers", []):
        joint = urdf.joints.get(name)
        if joint is None:
            report("ERROR", where, f"carrier joint '{name}' not in the description")
            continue
        mass = urdf.subtree_mass(joint.find("child").get("link"))
        limit = joint.find("limit")
        effort = float(limit.get("effort")) if limit is not None and limit.get("effort") else 0.0
        needed = mass * GRAVITY
        level = "OK" if effort >= needed else "ERROR"
        report(level, where, f"{name} drives {effort:.0f} N for {mass:.1f} kg it carries "
                             f"(needs at least its weight, {needed:.0f} N)")


def check_collisions(report, where, urdf_path, srdf_path, out_dir, checked=()):
    """checked: pairs the cell keeps checked although they touch in the zero state."""
    out = os.path.join(out_dir, os.path.basename(srdf_path) + ".updated")
    result = subprocess.run(
        ["ros2", "run", "moveit_setup_assistant", "collisions_updater", "--urdf", urdf_path,
         "--srdf", srdf_path, "--output", out, "--default", "--always", "--keep", "--trials", "0"],
        capture_output=True, text=True, timeout=600)
    if result.returncode != 0 or not os.path.isfile(out):
        report("WARN", where, f"collisions_updater failed: {result.stderr.strip()[-200:]}")
        return

    def pairs(path):
        found = re.findall(r'link1="([^"]*)" link2="([^"]*)" reason="([^"]*)"', open(path).read())
        return {frozenset((a, b)): reason for a, b, reason in found}

    have, computed = pairs(srdf_path), pairs(out)
    missing = {tuple(sorted(p)): r for p, r in computed.items() if p not in have and r != "Never"}
    touching = {p: r for p, r in missing.items()
                if r == "Always" or (r == "Default" and p not in checked)}
    kept = sorted(p for p, r in missing.items() if r == "Default" and p in checked)
    if kept:
        report("OK", where, f"touching in the zero state, kept checked by the cell: {kept}")
    adjacent = {p: r for p, r in missing.items() if r == "Adjacent"}
    if touching:
        report("ERROR", where, f"links that touch but are not disabled in the SRDF: {sorted(touching.items())}")
    if adjacent:
        report("WARN", where, f"adjacent links not disabled in the SRDF (checked for nothing): {sorted(adjacent)}")
    if not touching and not adjacent:
        report("OK", where, "no touching or adjacent link pair missing from the SRDF")


def check_kept(report, where, variant, srdf_path):
    """No pair of keep_collisions disabled as Never (a hand edit or an SRDF older than the list)."""
    if not variant.keep_collisions():
        return
    found = re.findall(r'link1="([^"]*)" link2="([^"]*)" reason="Never"', open(srdf_path).read())
    disabled = sorted((a, b) for a, b in found if variant.kept(a, b))
    if disabled:
        report("ERROR", where, f"keep_collisions pairs disabled in the SRDF (regenerate): {disabled}")
    else:
        report("OK", where, f"no keep_collisions pair disabled ({len(variant.keep_collisions())} patterns)")


def check_modules(report, where, variant):
    """The modules' announcements: vocabulary, module classes, forms, models."""
    if importlib.util.find_spec("skill_base") is None:
        report("WARN", where, "module capabilities not checked: skill_base (their vocabulary) is not installed")
        return
    try:
        problems = variant.module_problems()
    except Exception as exc:
        problems = [str(exc)]
    if problems:
        report("ERROR", where, "module capabilities: " + "; ".join(problems))
    else:
        modules = variant.modules()
        report("OK", where, f"module capabilities valid for {', '.join(modules)}")


def check_generated(report, cell):
    result = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "generate_cell.py"), "--check", cell],
                            capture_output=True, text=True)
    if result.returncode == 0:
        report("OK", f"{cell}", "generated files match the composition")
    else:
        stale = [line.split()[1] for line in result.stdout.splitlines() if line.startswith("STALE")]
        report("ERROR", f"{cell}", f"generated files are stale, run tools/generate_cell.py {cell}: "
                                   f"{stale or result.stderr.strip()[-300:]}")


def resolved_controller_files(variant, out_dir):
    """Controller parameter files of a variant: the generated manager file with
    its tokens resolved, plus every params_file it names; or, for a manager
    started elsewhere, the variant's controller_files."""
    generated = variant.generated("ros2_controllers.yaml")
    if not os.path.isfile(generated):
        return list(variant.controller_files())
    path = os.path.join(out_dir, f"{variant.cell.name}_{variant.name}_controllers.yaml")
    with open(generated) as handle:
        text = composition.resolve_tokens(handle.read(), this=path)
    with open(path, "w") as handle:
        handle.write(text)
    refs = [{"path": path}]
    manager = (yaml.safe_load(text).get("controller_manager") or {}).get("ros__parameters", {})
    for spec in manager.values():
        for file in (spec.get("params_file") or []) if isinstance(spec, dict) else []:
            if {"path": file} not in refs:
                refs.append({"path": file})
    return refs


def check_vendored(report):
    result = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "check_vendored.py")],
                            capture_output=True, text=True)
    for line in result.stdout.splitlines():
        level = "ERROR" if "NOT vanilla" in line else ("OK" if "vanilla (" in line else "WARN")
        report(level, "vendored", line.strip())


# ----------------------------------------------------------------------

def check_variant(report, variant, plugin_types, fast, tmp):
    """The variant as MoveIt plans with it; returns MoveIt's position bounds of the planning joints."""
    where = variant.where
    try:
        urdf_path = expand_xacro(variant.urdf, tmp, f"{variant.cell.name}_{variant.name}.urdf")
        urdf = Urdf(urdf_path)
        srdf_path = variant.generated("robot.srdf")
        srdf = Srdf(srdf_path)
    except Exception as exc:
        report("ERROR", where, str(exc))
        return {}
    controller_files = resolved_controller_files(variant, tmp)
    spec = {
        "planning_groups": variant.planning_groups,
        "joint_limits": [{"path": variant.generated("joint_limits.yaml")}],
        "moveit_controllers": {"path": variant.generated("moveit_controllers.yaml")},
        "controller_joints": controller_files,
        # MoveIt-only action servers (a module's moveit_controllers): no
        # controller parameter file has them.
        "action_controllers": list(variant.action_controllers()),
        # Types only from the manager's own file: the controller_manager
        # section of a controller's params_file never reaches the manager.
        "controller_types": controller_files[:1] if os.path.isfile(variant.generated("ros2_controllers.yaml"))
                            else controller_files,
    }
    check_names(report, where, urdf, srdf)
    check_modules(report, where, variant)
    if spec["planning_groups"]:
        check_limits(report, where, spec, urdf, srdf)
    try:
        check_controllers(report, where, spec, urdf, srdf, plugin_types)
    except Exception as exc:          # a controller file of a package not installed here
        report("WARN", where, f"controllers not checked: {exc}")
    check_frames(report, where, variant.root, variant.tcp, urdf)
    modules = {i: {"module_class": m["module_class"], **m["description"]} for i, m in variant.modules().items()}
    check_borders(report, where, modules, urdf)
    INNER_FRAMES.update(module_frames(modules, urdf))
    check_calibrations(report, where, variant, urdf)
    check_force_control(report, where, variant)
    check_kept(report, where, variant, srdf_path)
    if not fast:
        check_collisions(report, where, urdf_path, srdf_path, tmp, variant.checked_pairs())
    joints, _ = planning_joints(srdf, urdf, spec["planning_groups"])
    overrides = load_yaml({"path": variant.generated("joint_limits.yaml")}).get("joint_limits") or {} \
        if os.path.isfile(variant.generated("joint_limits.yaml")) else {}
    return {j: position_bounds(urdf, j, overrides) for j in joints or []}


def check_converts(report, where, path):
    """Gazebo's own converter takes the description (gz sdf, ign sdf on Humble).

    A description it refuses is not spawned, and nothing else here sees that: on
    ACROBA a chain of two links without inertials (the camera on its mount) left
    a dangling frame, and the robot did not appear in Gazebo for days.
    """
    tool = shutil.which("gz") or shutil.which("ign")
    if tool is None:
        report("WARN", where, "neither gz nor ign found, conversion to SDF not checked")
        return
    result = subprocess.run([tool, "sdf", "-k", path], capture_output=True, text=True)
    errors = [line for line in (result.stdout + result.stderr).splitlines() if "Error" in line]
    if result.returncode or errors:
        first = errors[0] if errors else f"exit status {result.returncode}"
        report("ERROR", where, f"Gazebo's converter refuses the description ({len(errors)} errors): {first}")
    else:
        report("OK", where, "Gazebo's converter takes the description")


def check_registered(report):
    """Every cell in the source tree is installed and found by name (the ament index), as launch files look it up."""
    source = composition.list_cells(CELL_DIR)
    try:
        installed = composition.list_cells()
    except Exception as exc:
        report("WARN", "registered", f"installed cells not looked up: {exc}")
        return
    for name in source:
        if name not in installed:
            report("ERROR", "registered", f"{name}: not installed or not in the ament index (cell_compositions); "
                                         "build its package, register the cell there")
            continue
        here = os.path.realpath(composition.find_cell(name, CELL_DIR))
        there = os.path.realpath(composition.find_cell(name))
        if os.path.basename(os.path.dirname(os.path.dirname(here))) != os.path.basename(os.path.dirname(os.path.dirname(there))):
            report("ERROR", "registered", f"{name}: installed from another package ({there})")
    report("OK", "registered", f"{len(source)} cells found by name: {', '.join(source)}")


def check_gazebo(report, variant, tmp, moveit_bounds=None):
    """The variant as Gazebo simulates it."""
    gazebo = variant.gazebo
    where = f"{variant.where}/gazebo"
    try:
        path = expand_xacro(gazebo["urdf"], tmp, f"{variant.cell.name}_{variant.name}_gazebo.urdf")
        urdf = Urdf(path)
    except Exception as exc:
        report("ERROR", where, str(exc))
        return
    check_converts(report, where, path)
    check_frames(report, where, gazebo.get("root") or variant.root, variant.tcp, urdf)
    check_borders(report, where, {i: {"module_class": m["module_class"], **m["description"]} for i, m in variant.modules().items()}, urdf)
    if moveit_bounds:
        check_position_limits(report, where, moveit_bounds, urdf)
    if "grasp" in gazebo:
        check_grasp(report, where, gazebo, urdf)
    if gazebo.get("carriers"):
        check_carriers(report, where, gazebo, urdf)


def check_variant_and_gazebo(report, variant, plugin_types, fast, tmp):
    """A variant's checks; a package it names that is not installed is an error, not a crash."""
    try:
        bounds = check_variant(report, variant, plugin_types, fast, tmp)
        if variant.gazebo is not None:
            check_gazebo(report, variant, tmp, bounds)
    except Exception as exc:     # PackageNotFoundError and the like: report and go on
        report("ERROR", variant.where, f"not checked further: {str(exc).split(', searching')[0]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--cell", help="only this cell")
    parser.add_argument("--variant", help="only this variant (real needs no simulation packages)")
    parser.add_argument("--fast", action="store_true", help="skip the collision matrix (collisions_updater)")
    parser.add_argument("--no-vendored", action="store_true", help="skip the vendored-package check")
    args = parser.parse_args()

    report = Report()
    plugin_types = installed_controller_types()

    with tempfile.TemporaryDirectory(prefix="composition_") as tmp:
        for name in composition.list_cells(CELL_DIR):
            if args.cell and name != args.cell:
                continue
            cell = composition.Cell(name, CELL_DIR)
            check_generated(report, name)
            for variant_name in cell.variant_names():
                if args.variant and variant_name != args.variant:
                    continue
                variant = cell.variant(variant_name)
                check_variant_and_gazebo(report, variant, plugin_types, args.fast, tmp)
            # A module the cell can swap: every alternative is checked too, so
            # that a swap does not find a descriptor gone stale. The SRDF is the
            # type's in the cell, a real swap regenerates it.
            for instance, spec in cell.specs.items():
                for alternative in spec.get("alternatives") or []:
                    swapped = composition.Cell(name, CELL_DIR, swap={instance: alternative})
                    for variant_name in swapped.variant_names():
                        if args.variant and variant_name != args.variant:
                            continue
                        variant = swapped.variant(variant_name)
                        check_variant_and_gazebo(report, variant, plugin_types, args.fast, tmp)
        check_registered(report)
        check_reference(report, INNER_FRAMES)
        if not args.no_vendored:
            check_vendored(report)

    print(f"\n{report.counts['ERROR']} error(s), {report.counts['WARN']} warning(s), "
          f"{report.counts['OK']} ok")
    return 1 if report.counts["ERROR"] else 0


if __name__ == "__main__":
    sys.exit(main())
