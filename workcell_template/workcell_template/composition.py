"""
Workcell compositions: load a workcell (workcells/<workcell>.yaml) with its module
descriptors (modules/<type>.yaml), and render what follows from it.

Used by
  tools/generate_workcell.py       writes workcells/<workcell>/<variant>/*
  tools/check_workcell.py   checks the composed workcell
  launch files                 controller_manager_params(), moveit_builder()

A workcell lists its modules; each module type contributes groups (with named
states and kinematics), joint limits and controllers per variant, and the
workcell adds what only it knows: groups spanning modules, its root and virtual
joint, overrides. Merging is one rule everywhere: dicts merge key by key,
anything else (lists included) is replaced, and null removes an entry.
See CONVENTION.md for the format.
"""

import copy
import fnmatch
import os
import re
import xml.etree.ElementTree as ET

import yaml

PACKAGE = "workcell_template"
GENERATED_FILES = ("robot.srdf", "kinematics.yaml", "joint_limits.yaml",
                   "moveit_controllers.yaml")

# The template's kinds and border roles (CONVENTION.md). A new kind starts here.
KINDS = ("arm", "carrier", "changer", "gripper", "sensor")
FRAME_ROLES = ("base", "coupling", "flange", "mount", "output", "tcp")

TRAJECTORY_EXECUTION = {
    "allowed_execution_duration_scaling": 1.2,
    "allowed_goal_duration_margin": 0.5,
    "allowed_start_tolerance": 0.01,
    # A scaled controller stretches trajectories on the real arm, which would trip the monitor.
    "execution_duration_monitoring": False,
}


def share_dir(package=PACKAGE):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def merge(base, override):
    """base with override merged in: dicts key by key, None removes, else replace."""
    if not isinstance(base, dict) or not isinstance(override, dict):
        return copy.deepcopy(override)
    result = copy.deepcopy(base)
    for key, value in override.items():
        if value is None:
            result.pop(key, None)
        elif key in result:
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def is_ref(item):
    return isinstance(item, dict) and ("package" in item or "path" in item)


def ref_path(ref):
    """A {package, file} reference in an installed share directory, or {path}."""
    if "path" in ref:
        return os.path.expanduser(ref["path"])
    return os.path.join(share_dir(ref["package"]), ref["file"])


def ref_token(ref):
    """A reference as a launch-style substitution string, resolved by resolve_tokens()."""
    if "path" in ref:
        return os.path.expanduser(ref["path"])
    return f"$(find-pkg-share {ref['package']})/{ref['file']}"


def resolve_tokens(text, this=None):
    """Replace $(find-pkg-share pkg) and $(this) in text."""
    text = re.sub(r"\$\(find-pkg-share ([\w-]+)\)", lambda m: share_dir(m.group(1)), text)
    if this is not None:
        text = text.replace("$(this)", this)
    return text


def load_yaml(path):
    with open(path) as handle:
        return yaml.safe_load(handle) or {}


# ----------------------------------------------------------------------
# Cells
# ----------------------------------------------------------------------

def border_frames(instance, descriptor, spec):
    """{role: frame} of a module instance: the descriptor's frames, then the
    workcell's for the instance (a pass-through output the vendor part does not
    have). A name starting with `_` is the instance's name plus it, so two
    grippers get left_gripper_tcp and right_gripper_tcp (the workcell template,
    CONVENTION.md)."""
    frames = merge(dict(descriptor.get("frames") or {}), spec.get("frames") or {})
    return {role: f"{instance}{name}" if name.startswith("_") else name for role, name in frames.items()}


class Workcell:
    def __init__(self, name, root_dir=None):
        """root_dir: the workcell_template directory (source tree or share); default: installed share."""
        self.name = name
        self.root_dir = root_dir or share_dir()
        self.path = os.path.join(self.root_dir, "workcells", f"{name}.yaml")
        self.data = load_yaml(self.path)
        self.modules = {}
        self.descriptors, self.specs = {}, {}       # unmerged, for Variant.modules()
        for instance, spec in (self.data.get("modules") or {}).items():
            descriptor = load_yaml(os.path.join(self.root_dir, "modules", f"{spec['type']}.yaml"))
            self.descriptors[instance], self.specs[instance] = descriptor, spec
            self.modules[instance] = {"instance": instance, **descriptor, **spec,
                                      "frames": border_frames(instance, descriptor, spec)}

    def variant_names(self):
        return list((self.data.get("variants") or {}).keys())

    def variant(self, name):
        variants = self.data.get("variants") or {}
        if name not in variants:
            raise KeyError(f"workcell '{self.name}' has no variant '{name}' (has {list(variants)})")
        return Variant(self, name, variants[name] or {})


def list_workcells(root_dir=None):
    cells_dir = os.path.join(root_dir or share_dir(), "workcells")
    return sorted(f[:-5] for f in os.listdir(cells_dir) if f.endswith(".yaml"))


class Variant:
    def __init__(self, workcell, name, data):
        self.workcell, self.name, self.data = workcell, name, data
        self.where = f"{workcell.name}/{name}"

    # -- descriptions ----------------------------------------------------

    @property
    def urdf(self):
        """The description MoveIt plans with: {package, file, args}."""
        ref = merge(self.data.get("urdf") or self.workcell.data["urdf"], {})
        ref["args"] = merge(ref.get("args") or {}, self.data.get("urdf_args") or {})
        return ref

    @property
    def root(self):
        return self.data.get("root") or self.workcell.data.get("root", "world")

    @property
    def tcp(self):
        """{frame, parent, module config} of the TCP, or None."""
        tcp = self.workcell.data.get("tcp")
        if not tcp:
            return None
        module = self.workcell.modules[tcp["module"]]
        return {"frame": tcp.get("frame") or module["frames"]["tcp"], "parent": module["frames"]["mount"],
                "expected": module["tcp"]}

    @property
    def planning_groups(self):
        return list(self.data.get("planning_groups") or self.workcell.data.get("planning_groups") or [])

    def generated_dir(self, root_dir=None):
        return os.path.join(root_dir or self.workcell.root_dir, "workcells", self.workcell.name, self.name)

    def generated(self, filename, root_dir=None):
        return os.path.join(self.generated_dir(root_dir), filename)

    # -- semantics ---------------------------------------------------------

    def groups(self):
        """{group: spec} in SRDF order: module groups in module order, then the workcell's own."""
        groups = {}
        for module in self.workcell.modules.values():
            for name, spec in (module.get("groups") or {}).items():
                groups[name] = merge(spec, {"module": module["instance"]})
        workcell_groups = merge(self.workcell.data.get("groups") or {}, self.data.get("groups") or {})
        for name, spec in workcell_groups.items():
            groups[name] = merge(groups.get(name, {}), spec)
            if "modules" in spec:
                joints = []
                for instance in spec["modules"]:
                    joints += [j for j in self.workcell.modules[instance].get("joints", []) if j not in joints]
                groups[name]["joints"] = joints + [j for j in groups[name].get("joints", []) if j not in joints]
        return groups

    def arm_group(self):
        for module in self.workcell.modules.values():
            if module.get("kind") == "arm":
                return next(iter(module.get("groups") or {}), None)
        return None

    def keep_collisions(self):
        """
        [(patterns, patterns)]: link pairs that stay checked although
        collisions_updater's samples never saw them touch, from the workcell's and
        the variant's keep_collisions. Each side is an fnmatch pattern or a
        list of them; a pair matches in either order. For parts the arm
        reaches only in a small region, which random samples rarely hit (a
        tool rack).
        """
        pairs = (self.workcell.data.get("keep_collisions") or []) + (self.data.get("keep_collisions") or [])
        side = lambda s: [s] if isinstance(s, str) else list(s)
        return [(side(x), side(y)) for x, y in pairs]

    def kept(self, a, b):
        """Whether the link pair (a, b) matches keep_collisions."""
        match = lambda link, patterns: any(fnmatch.fnmatchcase(link, p) for p in patterns)
        return any((match(a, x) and match(b, y)) or (match(a, y) and match(b, x))
                   for x, y in self.keep_collisions())

    def end_effectors(self):
        result = []
        groups = self.groups()
        for module in self.workcell.modules.values():
            group = module.get("end_effector")
            if group:
                parent_group = self.workcell.data.get("end_effector_parent_group") or self.arm_group()
                # MoveIt wants the parent link inside the parent group; a module
                # mounted on fixed links past a chain's tip hangs from that tip.
                chain = (groups.get(parent_group) or {}).get("chain")
                result.append({"name": group, "group": group,
                               "parent_link": chain[1] if chain else module.get("parent"),
                               "parent_group": parent_group})
        return result

    def srdf_without_collisions(self, robot_name, urdf_links=()):
        """The SRDF: groups, states, end effectors, virtual joint, and the links
        whose collision geometry a module replaces (`collision_replacements`,
        only where the replacing link is in the URDF, urdf_links); the disabled
        collision pairs are added by tools/generate_workcell.py (collisions_updater)."""
        robot = ET.Element("robot", name=robot_name)
        groups = self.groups()
        for name, spec in groups.items():
            group = ET.SubElement(robot, "group", name=name)
            if spec.get("chain"):
                base, tip = spec["chain"]
                ET.SubElement(group, "chain", base_link=base, tip_link=tip)
            for joint in spec.get("joints") or []:
                ET.SubElement(group, "joint", name=joint)
            for link in spec.get("links") or []:
                ET.SubElement(group, "link", name=link)
        for name, spec in groups.items():
            for state, values in (spec.get("states") or {}).items():
                element = ET.SubElement(robot, "group_state", name=state, group=name)
                for joint, value in values.items():
                    ET.SubElement(element, "joint", name=joint, value=_number(value))
        for ee in self.end_effectors():
            ET.SubElement(robot, "end_effector", name=ee["name"], parent_link=ee["parent_link"],
                          group=ee["group"], parent_group=ee["parent_group"])
        vj = self.data.get("virtual_joint", self.workcell.data.get("virtual_joint"))
        if vj:
            ET.SubElement(robot, "virtual_joint", name=vj["name"], type=vj.get("type", "fixed"),
                          parent_frame=vj["parent_frame"], child_link=vj["child_link"])
        for link in self.replaced_collision_links(urdf_links):
            ET.SubElement(robot, "disable_default_collisions", link=link)
        return robot

    def replaced_collision_links(self, urdf_links):
        """Links whose collision geometry a module replaces with another link
        present in the URDF: MoveIt checks them against nothing."""
        links = set(urdf_links)
        result = []
        for module in self.workcell.modules.values():
            for link, replacement in (module.get("collision_replacements") or {}).items():
                if replacement in links and link in links:
                    result.append(link)
        return sorted(result)

    def checked_pairs(self):
        """Link pairs that touch in the zero state only by chance and stay
        checked (`collisions: {checked: [[a, b], ...]}`, workcell and variant):
        an arm that reaches into the workcell's frame when stretched out."""
        pairs = (self.workcell.data.get("collisions") or {}).get("checked") or []
        pairs += (self.data.get("collisions") or {}).get("checked") or []
        return {tuple(sorted(pair)) for pair in pairs}

    def kinematics(self):
        return {name: spec["kinematics"] for name, spec in self.groups().items() if spec.get("kinematics")}

    def joint_limits(self):
        """Merged MoveIt joint limits: module sources in module order, then the workcell's."""
        result = {}
        sources = []
        for module in self.workcell.modules.values():
            sources += module.get("joint_limits") or []
        sources += self.workcell.data.get("joint_limits") or []
        sources += self.data.get("joint_limits") or []
        for source in sources:
            result = merge(result, load_yaml(ref_path(source)) if is_ref(source) else source)
        return result

    # -- modules -----------------------------------------------------------

    def modules(self):
        """{instance: {type, kind, frames, parent}} of this workcell, for the checks."""
        return {instance: {"type": module["type"], "kind": module.get("kind", ""), "frames": module["frames"],
                           "parent": module.get("parent", "")}
                for instance, module in self.workcell.modules.items()}

    def module_problems(self):
        """What is wrong with the modules' kinds and border roles, as messages; empty if nothing."""
        problems = []
        for instance, module in self.modules().items():
            if module["kind"] not in KINDS:
                problems.append(f"{instance}: unknown kind '{module['kind']}' (known: {list(KINDS)})")
            problems += [f"{instance}: unknown border role '{role}' (known: {list(FRAME_ROLES)})"
                         for role in module["frames"] if role not in FRAME_ROLES]
        return problems


    # -- controllers -------------------------------------------------------

    def controllers(self):
        """{name: spec} for this variant: the modules' (in module order) merged with the workcell's."""
        result = {}
        for module in self.workcell.modules.values():
            for name, spec in ((module.get("controllers") or {}).get(self.name) or {}).items():
                result[name] = merge(spec, {"module": module["instance"]})
        overrides = merge(self.workcell.data.get("controllers") or {}, self.data.get("controllers") or {})
        for name, spec in overrides.items():
            if spec is None:
                result.pop(name, None)
            else:
                result[name] = merge(result.get(name, {}), spec)
        return result

    def controller_joints(self, name, spec):
        """The joints a controller drives: its params, else its module's joints."""
        params = spec.get("params") or {}
        if params.get("joints"):
            return list(params["joints"])
        if spec.get("joints"):
            return list(spec["joints"])
        module = self.workcell.modules.get(spec.get("module"))
        return list(module.get("joints") or []) if module else []

    def action_controllers(self):
        """{name: spec} of the modules' MoveIt-only controllers (their
        `moveit_controllers`: an action server MoveIt executes on, no
        ros2_control controller behind that name), for every variant."""
        result = {}
        for module in self.workcell.modules.values():
            for name, spec in (module.get("moveit_controllers") or {}).items():
                result[name] = merge(spec, {"module": module["instance"]})
        return result

    def moveit_controllers(self):
        names, entries = [], {}
        for name, spec in self.controllers().items():
            moveit = spec.get("moveit")
            if not moveit:
                continue
            names.append(name)
            entries[name] = {
                "action_ns": moveit.get("action_ns", "follow_joint_trajectory"),
                "type": moveit.get("type", "FollowJointTrajectory"),
                "default": bool(moveit.get("default", False)),
                "joints": self.controller_joints(name, spec),
            }
        for name, spec in self.action_controllers().items():
            names.append(name)
            entries[name] = {
                "action_ns": spec["action_ns"],
                "type": spec["type"],
                "default": bool(spec.get("default", False)),
                "joints": self.controller_joints(name, spec),
            }
        execution = merge(TRAJECTORY_EXECUTION, merge(self.workcell.data.get("trajectory_execution") or {},
                                                      self.data.get("trajectory_execution") or {}))
        return {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "trajectory_execution": execution,
            "moveit_simple_controller_manager": {"controller_names": names, **entries},
        }

    def controller_files(self):
        """Parameter files of a controller manager started elsewhere (real workcells)."""
        return list(self.data.get("controller_files") or [])


def _number(value):
    return repr(float(value)).rstrip("0").rstrip(".") if float(value) != int(float(value)) else str(int(float(value)))


# ----------------------------------------------------------------------
# For launch files
# ----------------------------------------------------------------------

def moveit_builder(workcell, variant, urdf_args=None, package_name=PACKAGE):
    """A MoveItConfigsBuilder with the workcell's description and generated semantics,
    kinematics, joint limits and controller list. package_name is where the
    builder looks for anything else (planning pipelines, sensors). urdf_args
    may hold launch substitutions (a LaunchConfiguration)."""
    from moveit_configs_utils import MoveItConfigsBuilder
    v = Workcell(workcell).variant(variant)
    urdf = v.urdf
    mappings = {k: str(val) for k, val in (urdf.get("args") or {}).items()}
    mappings.update(urdf_args or {})      # not str(): substitutions stay substitutions
    return (
        MoveItConfigsBuilder(f"{workcell}_workcell", package_name=package_name)
        .robot_description(file_path=ref_path(urdf), mappings=mappings)
        .robot_description_semantic(file_path=v.generated("robot.srdf"))
        .robot_description_kinematics(file_path=v.generated("kinematics.yaml"))
        .joint_limits(file_path=v.generated("joint_limits.yaml"))
        .trajectory_execution(file_path=v.generated("moveit_controllers.yaml"))
    )
