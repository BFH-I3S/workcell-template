"""
Cell compositions: load a cell (cells/<cell>.yaml in the workcell's own
package) with its module descriptors (modules/<type>.yaml, in this package),
and render what follows from it.

A workcell package holds cells/<cell>.yaml and the generated
cells/<cell>/<variant>/*, installs them to its share directory and registers
each cell in the ament index (resource type cell_compositions, content its
package name), so a cell is found by name. Working on the source tree (the
generator, the checks), root_dir is this package's source directory and the
cells are looked for in its sibling packages.

Used by
  tools/generate_cell.py       writes <package>/cells/<cell>/<variant>/*
  tools/check_composition.py   checks the composed cell
  launch files                 controller_manager_params(), moveit_builder()

A cell lists its modules; each module type contributes groups (with named
states and kinematics), joint limits and controllers per variant, and the
cell adds what only it knows: groups spanning modules, its root and virtual
joint, overrides. Merging is one rule everywhere: dicts merge key by key,
anything else (lists included) is replaced, and null removes an entry.
See README.md (this package) for the format.
"""

import copy
import fnmatch
import os
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml

PACKAGE = "cell_description"
CELL_RESOURCE = "cell_compositions"     # ament index: cell name -> its package
GENERATED_FILES = ("robot.srdf", "kinematics.yaml", "joint_limits.yaml",
                   "moveit_controllers.yaml", "ros2_controllers.yaml")

TRAJECTORY_EXECUTION = {
    "allowed_execution_duration_scaling": 1.2,
    "allowed_goal_duration_margin": 0.5,
    "allowed_start_tolerance": 0.01,
    # Gazebo runs below real time under load, and the scaled controller
    # stretches trajectories on the real arm; either would trip the monitor.
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
    cell's for the instance (a pass-through output the vendor part does not
    have). A name starting with `_` is the instance's name plus it, so two
    grippers get left_gripper_tcp and right_gripper_tcp (the cell template,
    README.md)."""
    frames = merge(dict(descriptor.get("frames") or {}), spec.get("frames") or {})
    return {role: f"{instance}{name}" if name.startswith("_") else name for role, name in frames.items()}


def force_control_of(descriptor, variant):
    """{node, controller} an arm's descriptor names for serving force control in this variant, or None."""
    return (descriptor.get("force_control") or {}).get(variant)


class Cell:
    def __init__(self, name, root_dir=None, swap=None):
        """root_dir: the cell_description directory (source tree or share); default: installed share.
        A source directory also means the cell is looked for in the source tree (find_cell).
        swap: {instance: type}, the cell with one of an instance's `alternatives` in its place."""
        self.name = name
        self.swap = swap or {}
        self.root_dir = root_dir or share_dir()
        self.path = find_cell(name, root_dir)
        self.data = load_yaml(self.path)
        self.modules = {}
        self.descriptors, self.specs = {}, {}       # unmerged, for Variant.modules()
        for instance, spec in (self.data.get("modules") or {}).items():
            if instance in self.swap:
                spec = {**spec, "type": self.swap[instance]}
            descriptor = load_yaml(os.path.join(self.root_dir, "modules", f"{spec['type']}.yaml"))
            self.descriptors[instance], self.specs[instance] = descriptor, spec
            self.modules[instance] = {"instance": instance, **descriptor, **spec,
                                      "frames": border_frames(instance, descriptor, spec)}

    def variant_names(self):
        return list((self.data.get("variants") or {}).keys())

    def variant(self, name):
        variants = self.data.get("variants") or {}
        if name not in variants:
            raise KeyError(f"cell '{self.name}' has no variant '{name}' (has {list(variants)})")
        return Variant(self, name, variants[name] or {})


def _source_cells(root_dir):
    """{cell: path} of the cells in the source tree next to root_dir (src/<package>/cells/<cell>.yaml)."""
    import glob
    found = {}
    for path in sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(root_dir)), "*", "cells", "*.yaml"))):
        name = os.path.basename(path)[:-5]
        if name in found:
            raise ValueError(f"cell '{name}' is in two packages: {found[name]} and {path}")
        found[name] = path
    return found


def find_cell(name, root_dir=None):
    """The file of a cell: in the source tree when root_dir is given, else through the ament index."""
    if root_dir:
        cells = _source_cells(root_dir)
        if name not in cells:
            raise FileNotFoundError(f"no cell '{name}' in the source tree (has {sorted(cells)})")
        return cells[name]
    from ament_index_python import get_resource
    try:
        package, prefix = get_resource(CELL_RESOURCE, name)
    except LookupError:
        raise FileNotFoundError(f"no cell '{name}' installed (has {list_cells()}); "
                                "a workcell package registers its cells in the ament index") from None
    return os.path.join(prefix, "share", package.strip(), "cells", f"{name}.yaml")


def list_cells(root_dir=None):
    if root_dir:
        return sorted(_source_cells(root_dir))
    from ament_index_python import get_resources
    return sorted(get_resources(CELL_RESOURCE))


class Variant:
    def __init__(self, cell, name, data):
        self.cell, self.name, self.data = cell, name, data
        self.where = f"{cell.name}/{name}" + "".join(f" ({i}: {t})" for i, t in cell.swap.items())

    # -- descriptions ----------------------------------------------------

    @property
    def urdf(self):
        """The description MoveIt plans with: {package, file, args}."""
        ref = merge(self.data.get("urdf") or self.cell.data["urdf"], {})
        ref["args"] = merge(ref.get("args") or {}, self.data.get("urdf_args") or {})
        ref["args"].update({f"{instance}_type": module_type for instance, module_type in self.cell.swap.items()})
        return ref

    @property
    def gazebo(self):
        """The Gazebo part of a variant (urdf, grasp, carriers), or None."""
        gazebo = self.data.get("gazebo")
        if gazebo is None:
            return None
        gazebo = copy.deepcopy(gazebo)
        ref = merge(gazebo.get("urdf") or self.urdf, {})
        ref["args"] = merge(ref.get("args") or {}, gazebo.get("urdf_args") or {})
        gazebo["urdf"] = ref
        return gazebo

    @property
    def root(self):
        return self.data.get("root") or self.cell.data.get("root", "world")

    @property
    def tcp(self):
        """{frame, parent, module config} of the TCP, or None."""
        tcp = self.cell.data.get("tcp")
        if not tcp:
            return None
        module = self.cell.modules[tcp["module"]]
        return {"frame": tcp.get("frame") or module["frames"]["tcp"], "parent": module["frames"]["mount"],
                "expected": module["tcp"]}

    @property
    def planning_groups(self):
        return list(self.data.get("planning_groups") or self.cell.data.get("planning_groups") or [])

    def generated_dir(self):
        """cells/<cell>/<variant>/ next to the cell's file, in the workcell's package."""
        return os.path.join(os.path.dirname(self.cell.path), self.cell.name, self.name)

    def generated(self, filename):
        return os.path.join(self.generated_dir(), filename)

    # -- semantics ---------------------------------------------------------

    def groups(self):
        """{group: spec} in SRDF order: module groups in module order, then the cell's own."""
        groups = {}
        for module in self.cell.modules.values():
            for name, spec in (module.get("groups") or {}).items():
                groups[name] = merge(spec, {"module": module["instance"]})
        cell_groups = merge(self.cell.data.get("groups") or {}, self.data.get("groups") or {})
        for name, spec in cell_groups.items():
            groups[name] = merge(groups.get(name, {}), spec)
            if "modules" in spec:
                joints = []
                for instance in spec["modules"]:
                    joints += [j for j in self.cell.modules[instance].get("joints", []) if j not in joints]
                groups[name]["joints"] = joints + [j for j in groups[name].get("joints", []) if j not in joints]
        return groups

    def arm_group(self):
        for module in self.cell.modules.values():
            if module.get("module_class") == "arm":
                return next(iter(module.get("groups") or {}), None)
        return None

    def keep_collisions(self):
        """
        [(patterns, patterns)]: link pairs that stay checked although
        collisions_updater's samples never saw them touch, from the cell's and
        the variant's keep_collisions. Each side is an fnmatch pattern or a
        list of them; a pair matches in either order. For parts the arm
        reaches only in a small region, which random samples rarely hit (the
        gantry's tool rack).
        """
        pairs = (self.cell.data.get("keep_collisions") or []) + (self.data.get("keep_collisions") or [])
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
        for module in self.cell.modules.values():
            group = module.get("end_effector")
            if group:
                parent_group = self.cell.data.get("end_effector_parent_group") or self.arm_group()
                # MoveIt wants the parent link inside the parent group; a module
                # mounted on fixed links past a chain's tip (the gantry's
                # changer_coupling beyond tool0) hangs from that tip.
                chain = (groups.get(parent_group) or {}).get("chain")
                result.append({"name": group, "group": group,
                               "parent_link": chain[1] if chain else module.get("parent"),
                               "parent_group": parent_group})
        return result

    def srdf_without_collisions(self, robot_name, urdf_links=()):
        """The SRDF: groups, states, end effectors, virtual joint, and the links
        whose collision geometry a module replaces (`collision_replacements`,
        only where the replacing link is in the URDF, urdf_links); the disabled
        collision pairs are added by tools/generate_cell.py (collisions_updater)."""
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
        vj = self.data.get("virtual_joint", self.cell.data.get("virtual_joint"))
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
        for module in self.cell.modules.values():
            for link, replacement in (module.get("collision_replacements") or {}).items():
                if replacement in links and link in links:
                    result.append(link)
        return sorted(result)

    def checked_pairs(self):
        """Link pairs that touch in the zero state only by chance and stay
        checked (`collisions: {checked: [[a, b], ...]}`, cell and variant):
        an arm that reaches into the cell's frame when stretched out."""
        pairs = (self.cell.data.get("collisions") or {}).get("checked") or []
        pairs += (self.data.get("collisions") or {}).get("checked") or []
        return {tuple(sorted(pair)) for pair in pairs}

    def kinematics(self):
        return {name: spec["kinematics"] for name, spec in self.groups().items() if spec.get("kinematics")}

    def joint_limits(self):
        """Merged MoveIt joint limits: module sources in module order, then the cell's."""
        result = {}
        sources = []
        for module in self.cell.modules.values():
            sources += module.get("joint_limits") or []
        sources += self.cell.data.get("joint_limits") or []
        sources += self.data.get("joint_limits") or []
        for source in sources:
            result = merge(result, load_yaml(ref_path(source)) if is_ref(source) else source)
        return result

    # -- module announcements ------------------------------------------------

    def modules(self):
        """{instance: {type, module_class, model, description}} for this variant, where
        description is what the module announces on /modules/registry
        (frames, joints, groups, capabilities, unverified; format in
        skill_base/capabilities.py).

        Capabilities merge in this order: the descriptor's, its `models`
        entry for the instance's model, the descriptor's `variants` entry,
        the instance in the cell, the variant's `modules` entry in the cell.
        `unverified` accumulates over the same layers."""
        groups = self.groups()
        overrides = self.data.get("modules") or {}
        result = {}
        for instance, descriptor in self.cell.descriptors.items():
            spec = self.cell.specs[instance]
            model = spec.get("model")
            layers = [descriptor]
            if model:
                layers.append((descriptor.get("models") or {}).get(model) or {})
            layers.append((descriptor.get("variants") or {}).get(self.name) or {})
            layers += [spec, overrides.get(instance) or {}]
            capabilities, unverified = {}, []
            for layer in layers:
                capabilities = merge(capabilities, layer.get("capabilities") or {})
                unverified += [n for n in layer.get("unverified") or [] if n not in unverified]
            if descriptor.get("module_class") == "arm":
                capabilities["force_control"] = force_control_of(descriptor, self.name) is not None
            frames = border_frames(instance, descriptor, spec)
            result[instance] = {
                "type": spec["type"],
                "module_class": descriptor.get("module_class", ""),
                "model": model or "",
                "liveness": self._liveness(instance, descriptor.get("liveness") or {}),
                "description": {
                    "frames": frames,
                    "parent": spec.get("parent", ""),
                    "joints": list(descriptor.get("joints") or []),
                    "groups": [name for name, group in groups.items()
                               if group.get("module") == instance or instance in (group.get("modules") or [])],
                    "capabilities": capabilities,
                    "unverified": [n for n in unverified if n in capabilities],
                },
            }
        return result

    def calibrations(self):
        """[{instance, path, data}]: the measured values of this variant, one
        file per module that names one (`calibration` in the cell). `{type}`
        in the file name is the module's type; `variants` limits the entry."""
        result = []
        for instance, spec in self.cell.specs.items():
            ref = spec.get("calibration")
            if not ref or self.name not in ref.get("variants", [self.name]):
                continue
            path = ref_path({"package": ref["package"], "file": ref["file"].format(type=spec["type"])})
            result.append({"instance": instance, "path": path,
                           "data": load_yaml(path) if os.path.isfile(path) else None})
        return result

    def _liveness(self, instance, spec):
        """What the announcer watches for one module: {joints, controllers, topic}."""
        joints = list(self.cell.descriptors[instance].get("joints") or []) if spec.get("joints") else []
        controllers = []
        if spec.get("controllers"):
            controllers = [name for name, c in self.controllers().items()
                           if c.get("module") == instance and (c.get("moveit") or {}).get("default")]
        return {"joints": joints, "controllers": controllers, "topic": spec.get("topic")}

    def module_problems(self):
        """What is wrong with the modules' announcements, as messages; empty if nothing."""
        try:
            from skill_base.capabilities import check_description   # skill_base pulls in rclpy
        except ImportError:
            check_description = None    # without skill_base (the public repository): capabilities unchecked
        problems = []
        ur_type = (self.urdf.get("args") or {}).get("ur_type")
        for instance, module in self.modules().items():
            descriptor = self.cell.descriptors[instance]
            models = descriptor.get("models") or {}
            if models and module["model"] not in models:
                problems.append(f"{instance}: model must be one of {sorted(models)}, got '{module['model']}'")
            if module["module_class"] == "arm" and module["model"] and ur_type and module["model"] != ur_type:
                problems.append(f"{instance}: model {module['model']} but the description's ur_type is {ur_type}")
            if check_description is not None:
                problems += [f"{instance}: {p}" for p in check_description(module["module_class"], module["description"])]
            unknown = set(descriptor.get("liveness") or {}) - {"joints", "controllers", "topic"}
            if unknown:
                problems.append(f"{instance}: unknown liveness keys {sorted(unknown)}")
            topic = (descriptor.get("liveness") or {}).get("topic")
            if topic is not None and set(topic) != {"name", "type"}:
                problems.append(f"{instance}: liveness topic must be {{name, type}}")
        return problems

    # -- controllers -------------------------------------------------------

    def controllers(self):
        """{name: spec} for this variant: the modules' (in module order) merged with the cell's."""
        result = {}
        for module in self.cell.modules.values():
            for name, spec in ((module.get("controllers") or {}).get(self.name) or {}).items():
                result[name] = merge(spec, {"module": module["instance"]})
        overrides = merge(self.cell.data.get("controllers") or {}, self.data.get("controllers") or {})
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
        module = self.cell.modules.get(spec.get("module"))
        return list(module.get("joints") or []) if module else []

    def action_controllers(self):
        """{name: spec} of the modules' MoveIt-only controllers (their
        `moveit_controllers`: an action server MoveIt executes on, no
        ros2_control controller behind that name), for every variant."""
        result = {}
        for module in self.cell.modules.values():
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
        execution = merge(TRAJECTORY_EXECUTION, merge(self.cell.data.get("trajectory_execution") or {},
                                                      self.data.get("trajectory_execution") or {}))
        return {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "trajectory_execution": execution,
            "moveit_simple_controller_manager": {"controller_names": names, **entries},
        }

    def ros2_controllers(self):
        """Controller-manager parameters, or None when the variant's manager is started elsewhere.

        Paths are $(find-pkg-share ...) tokens and $(this) stands for the
        file itself (its inline controller parameters); controller_manager_params()
        resolves both, because params_file needs absolute paths."""
        manager = self.data.get("controller_manager")
        if manager is None:
            return None
        params = {k: v for k, v in manager.items()}
        inline = {}
        for name, spec in self.controllers().items():
            entry = {"type": spec["type"]}
            files = [ref_token(ref) for ref in spec.get("params_files") or []]
            if spec.get("params"):
                files.append("$(this)")
                inline[name] = {"ros__parameters": spec["params"]}
            if files:
                entry["params_file"] = files
            params[name] = entry
        return {"controller_manager": {"ros__parameters": params}, **inline}

    def controller_files(self):
        """Parameter files of a controller manager started elsewhere (real cells)."""
        return list(self.data.get("controller_files") or [])


def _number(value):
    return repr(float(value)).rstrip("0").rstrip(".") if float(value) != int(float(value)) else str(int(float(value)))


# ----------------------------------------------------------------------
# For launch files
# ----------------------------------------------------------------------

def controller_manager_params(cell, variant="sim"):
    """Path of a resolved copy of the generated ros2_controllers.yaml, for
    gz_ros2_control (simulation_controllers) or a controller manager."""
    source = Cell(cell).variant(variant).generated("ros2_controllers.yaml")
    with open(source) as handle:
        text = handle.read()
    fd, path = tempfile.mkstemp(prefix=f"{cell}_{variant}_controllers_", suffix=".yaml")
    with os.fdopen(fd, "w") as handle:
        handle.write(resolve_tokens(text, this=path))
    return path


def _gripper_module(cell, root_dir=None):
    for module in Cell(cell, root_dir).modules.values():
        if module.get("module_class") == "gripper":
            return module
    return None


def _finger(module):
    """The driven finger joint and its value fully closed (the gripper group's `closed` state)."""
    joint = module["joints"][0]
    closed = next(states["closed"][joint] for states in
                  (g.get("states") or {} for g in (module.get("groups") or {}).values())
                  if joint in (states.get("closed") or {}))
    return joint, float(closed)


def gripper_geometry(cell, root_dir=None):
    """The cell's gripper as the skills reason about it, as the node
    parameters gripper.* that condition_skill's GripperGeometry.from_node
    reads: the descriptor's `geometry`, max_gap from grasp_width, the finger
    joint and its closed value from the gripper group. Empty if the cell has
    no gripper or it has no `geometry`."""
    module = _gripper_module(cell, root_dir)
    if module is None or not module.get("geometry"):
        return {}
    joint, closed = _finger(module)
    values = {key: float(value) for key, value in module["geometry"].items()}
    values.update(max_gap=float(module["capabilities"]["grasp_width"]["max"]),
                  finger_joint=joint, closed_position=closed)
    return {f"gripper.{key}": value for key, value in values.items()}


def depth_camera(cell, variant="sim"):
    """The cell's depth camera as the scan needs what its driver does not
    publish: {depth_min, depth_max} in metres from the first sensor module
    with depth (its descriptor's depth_range, with the variant's entry);
    empty if the cell has none."""
    for module in Cell(cell).variant(variant).modules().values():
        capabilities = module["description"]["capabilities"]
        if module["module_class"] == "sensor" and capabilities.get("depth") and capabilities.get("depth_range"):
            return {"depth_min": float(capabilities["depth_range"]["min"]),
                    "depth_max": float(capabilities["depth_range"]["max"])}
    return {}


def arm_force_control(cell, variant="sim", root_dir=None):
    """{node, controller} that serves the cell's arm's force control, or None if nothing does."""
    composed = Cell(cell, root_dir)
    for instance, descriptor in composed.descriptors.items():
        if descriptor.get("module_class") == "arm":
            return force_control_of(descriptor, variant)
    return None


def gripper_driver(cell, root_dir=None):
    """The cell's gripper as Robotiq's ros2_control driver needs it, from the
    gripper module's descriptor: {model, joint, closed_position, stroke,
    goal_tolerance}. model is robotiq_description's name (2f_85, hand_e),
    joint the driven joint, closed_position its value when fully closed (the
    gripper group's `closed` state), stroke the skills' full stroke in metres
    (the geometry's max_stroke). None if the cell's gripper has no `driver` entry."""
    module = _gripper_module(cell, root_dir)
    if module is None or not module.get("driver"):
        return None
    joint, closed = _finger(module)
    return {
        "model": module["driver"]["robotiq_model"],
        "joint": joint,
        "closed_position": closed,
        "stroke": float((module.get("geometry") or {}).get("max_stroke",
                                                           module["capabilities"]["grasp_width"]["max"])),
        "goal_tolerance": float(module["driver"]["goal_tolerance"]),
    }


def moveit_builder(cell, variant, urdf_args=None, package_name=PACKAGE):
    """A MoveItConfigsBuilder with the cell's description and generated semantics,
    kinematics, joint limits and controller list. package_name is where the
    builder looks for anything else (planning pipelines, sensors). urdf_args
    may hold launch substitutions (a LaunchConfiguration)."""
    from moveit_configs_utils import MoveItConfigsBuilder
    v = Cell(cell).variant(variant)
    urdf = v.urdf
    mappings = {k: str(val) for k, val in (urdf.get("args") or {}).items()}
    mappings.update(urdf_args or {})      # not str(): substitutions stay substitutions
    return (
        MoveItConfigsBuilder(f"{cell}_cell", package_name=package_name)
        .robot_description(file_path=ref_path(urdf), mappings=mappings)
        .robot_description_semantic(file_path=v.generated("robot.srdf"))
        .robot_description_kinematics(file_path=v.generated("kinematics.yaml"))
        .joint_limits(file_path=v.generated("joint_limits.yaml"))
        .trajectory_execution(file_path=v.generated("moveit_controllers.yaml"))
    )
