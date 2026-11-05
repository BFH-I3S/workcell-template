#!/usr/bin/env python3
"""
Check how a workcell is composed, before anything is launched.

A workcell's robot is assembled from vendor descriptions, wrappers, an SRDF,
joint limits and controller configurations. Each piece can be right on its own
and the workcell still broken at a seam between them. This script checks those
seams for every variant of every workcell file (workcell_template/workworkcells/*.yaml)
as MoveIt plans with it:

  generated    the generated files are up to date with the workcell file
  modules      kinds and border roles are known
  names        URDF and SRDF describe the same robot; SRDF groups exist
  limits       every controlled joint has velocity and acceleration limits
  controllers  MoveIt's list has the right layout, covers every planning joint
               with one default controller, and agrees with the joints the
               controllers drive; controller types exist as plugins
  frames       workcell exists, is fixed in the workcell, z up; the TCP sits
               where its descriptor says
  borders      CONVENTION.md: each module declares the output borders its kind
               must offer; its mount exists, hangs on the parent the workcell
               file names, and on another module's output border keeps its z and
               lies on it; a TCP or an arm's base not behind a revolute joint
               keeps its mount's z; borders carry no geometry
  collisions   no pair of links that touch is missing from the SRDF's disabled
               collisions (MoveIt's collisions_updater; slow)

It compares files with each other. It cannot see the robot, or run-time faults.

Usage (from the repository root, after colcon build and sourcing install/):
    python3 tools/check_workcell.py                    all workcells, all checks
    python3 tools/check_workcell.py --workcell ur5e_2f85
    python3 tools/check_workcell.py --fast             without collisions
Exit status 1 if any check reports an error.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np
import yaml
from ament_index_python import get_resource, get_resources

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_DIR = os.path.join(ROOT, "workcell_template")
sys.path.insert(0, PACKAGE_DIR)
from workcell_template import composition  # noqa: E402  (the source tree's)


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
                # A driver's file leaves the prefix to launch; a workcell's arm has none.
                joints[name] = [str(j).replace("$(var tf_prefix)", "") for j in params["joints"]]
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


# What each kind of module must offer others, by role (CONVENTION.md's table). `gripper` is the template's
# end effector so far.
REQUIRED_BORDERS = {
    "arm": ("base", "flange"),
    "carrier": ("output",),
    "changer": ("coupling",),
    "gripper": ("tcp",),
    "sensor": ("base",),
}


def missing_borders(kind, frames):
    """The roles a module of this kind must declare and does not. An unknown kind has none to check."""
    return [role for role in REQUIRED_BORDERS.get(kind, ()) if role not in frames]


def check_borders(report, where, modules, urdf):
    """The workcell template's borders: every module's mount on its parent, its outputs below it."""
    outputs = {name: instance for instance, module in modules.items()
               for role, name in module["frames"].items() if role not in ("mount", "base")}
    for instance, module in modules.items():
        frames, parent = module["frames"], module.get("parent")
        for role in missing_borders(module.get("kind"), frames):
            report("ERROR", where, f"{instance}: kind '{module.get('kind')}' must declare the output border '{role}'")
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
            problems.append(f"{mount} hangs on {on}, the workcell says {parent}")
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
            if not (role == "tcp" or (role == "base" and module.get("kind") == "arm")):
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
            onto = f"{outputs[on]}'s {on}" if on in outputs else f"the workcell's {on}"
            report("OK", where, f"{instance}: {mount} on {onto}, "
                                f"{', '.join(n for r, n in frames.items() if r != 'mount')} below it")





def check_collisions(report, where, urdf_path, srdf_path, out_dir, checked=()):
    """checked: pairs the workcell keeps checked although they touch in the zero state."""
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
        report("OK", where, f"touching in the zero state, kept checked by the workcell: {kept}")
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


def check_generated(report, workcell):
    result = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "generate_workcell.py"), "--check", workcell],
                            capture_output=True, text=True)
    if result.returncode == 0:
        report("OK", f"{workcell}", "generated files match the composition")
    else:
        stale = [line.split()[1] for line in result.stdout.splitlines() if line.startswith("STALE")]
        report("ERROR", f"{workcell}", f"generated files are stale, run tools/generate_workcell.py {workcell}: "
                                   f"{stale or result.stderr.strip()[-300:]}")


# ----------------------------------------------------------------------

def check_modules(report, where, variant):
    """Kinds and border roles are the template's."""
    problems = variant.module_problems()
    if problems:
        report("ERROR", where, "modules: " + "; ".join(problems))
    else:
        report("OK", where, f"kinds and border roles valid for {', '.join(variant.modules())}")


def check_frames(report, where, root, tcp, urdf):
    """workcell fixed with z up; the TCP where its descriptor says."""
    if root not in urdf.links:
        report("ERROR", where, f"root link '{root}' not in the description")
        return
    if "workcell" not in urdf.links:
        report("ERROR", where, "no workcell frame (workcell_template's workcell_frame macro)")
    else:
        fixed = urdf.fixed_transform(root, "workcell")
        if fixed is None:
            report("ERROR", where, f"workcell is not fixed to {root} (a moving joint lies between them)")
        elif fixed[0][2, 2] < 0.999:
            report("ERROR", where, f"workcell's z does not point up in {root} (z.z = {fixed[0][2, 2]:.3f})")
        else:
            report("OK", where, f"workcell fixed to {root}, z up")
    if not tcp:
        return
    if tcp["frame"] not in urdf.parent_joint:
        report("ERROR", where, f"no TCP frame '{tcp['frame']}'")
        return
    fixed = urdf.fixed_transform(tcp["parent"], tcp["frame"])
    if fixed is None:
        report("ERROR", where, f"{tcp['frame']} is not fixed below {tcp['parent']}")
        return
    expected = tcp["expected"]
    if np.allclose(fixed[1], expected["xyz"], atol=1e-6) and \
            np.allclose(fixed[0], _rotation(np.asarray(expected["rpy"], dtype=float)), atol=1e-6):
        report("OK", where, f"{tcp['frame']} at {[round(float(v), 4) for v in fixed[1]]} from {tcp['parent']}")
    else:
        report("ERROR", where, f"{tcp['frame']} at {list(fixed[1])}, its descriptor says {expected}")


def check_variant(report, variant, plugin_types, fast, tmp):
    """The variant as MoveIt plans with it."""
    where = variant.where
    try:
        urdf_path = expand_xacro(variant.urdf, tmp, f"{variant.workcell.name}_{variant.name}.urdf")
        urdf = Urdf(urdf_path)
        srdf_path = variant.generated("robot.srdf")
        srdf = Srdf(srdf_path)
    except Exception as exc:
        report("ERROR", where, str(exc))
        return
    controller_files = controller_param_files(variant)
    spec = {
        "planning_groups": variant.planning_groups,
        "joint_limits": [{"path": variant.generated("joint_limits.yaml")}],
        "moveit_controllers": {"path": variant.generated("moveit_controllers.yaml")},
        "controller_joints": controller_files,
        "action_controllers": list(variant.action_controllers()),
        "controller_types": controller_files,
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
    check_borders(report, where, variant.modules(), urdf)
    check_kept(report, where, variant, srdf_path)
    if not fast:
        check_collisions(report, where, urdf_path, srdf_path, tmp, variant.checked_pairs())


def controller_param_files(variant):
    """The parameter files of the variant's controllers, as {package, file} references."""
    files = []
    for spec in variant.controllers().values():
        for ref in spec.get("params_files") or []:
            if ref not in files:
                files.append(ref)
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workcell", help="only this workcell")
    parser.add_argument("--fast", action="store_true", help="skip the collision matrix (collisions_updater)")
    args = parser.parse_args()

    report = Report()
    plugin_types = installed_controller_types()
    with tempfile.TemporaryDirectory(prefix="workcell_") as tmp:
        for name in composition.list_workcells(PACKAGE_DIR):
            if args.workcell and name != args.workcell:
                continue
            workcell = composition.Workcell(name, PACKAGE_DIR)
            check_generated(report, name)
            for variant_name in workcell.variant_names():
                check_variant(report, workcell.variant(variant_name), plugin_types, args.fast, tmp)
    print(f"\n{report.counts['ERROR']} error(s), {report.counts['WARN']} warning(s), "
          f"{report.counts['OK']} ok")
    return 1 if report.counts["ERROR"] else 0


if __name__ == "__main__":
    sys.exit(main())
