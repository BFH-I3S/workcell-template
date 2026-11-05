# cell_description: modules, the template, cell compositions

Part of [the workcell template](../../README.md). A work cell is composed of modules (an arm, a carrier such as a gantry axis, a
gripper) whose descriptions come from third-party packages that stay vanilla.

**What lives where** (5 Nov 2026): this package holds what every workcell
shares, the module files (descriptors in [modules/](modules/), wrappers and the
template's macros in [urdf/](urdf/)) and the software (`composition`; the
full workspace also has the modules' meshes and `module_announcer`, which are not
in this repository). Each workcell's own files are in its own
package, `cells/` there: [acroba](../acroba_description/cells/acroba.yaml),
and in the full workspace the gantry cells `gantry`, `gantry_gen3`,
`gantry_abb` and `gantry_irb1200` (not in this repository),
[robocoop](../robocoop_description/cells/robocoop.yaml). A workcell package
installs `cells/` and registers each cell in the ament index (resource type
`cell_compositions`, content the package's name; `ament_index_register_resource`
in CMake, a marker in `resource/cell_compositions/` for an ament_python
package), and `composition` finds a cell by its name.
[tools/generate_cell.py](../../tools/generate_cell.py) turns a workcell file,
together with the module descriptors, into the cell's MoveIt and ros2_control
configuration:

```
<package>/cells/<cell>.yaml  the composition (hand-written)
modules/<type>.yaml          what a module type contributes (hand-written)
<package>/cells/<cell>/<variant>/  generated, one directory per variant:
    robot.srdf               groups, named states, end effector, virtual joint,
                             disabled collisions (MoveIt's collisions_updater)
    kinematics.yaml          IK solver per group
    joint_limits.yaml        modules' limits merged with the cell's
    moveit_controllers.yaml  what move_group executes on
    ros2_controllers.yaml    controller-manager parameters (only variants that
                             start their own manager, i.e. simulations)
```

The generated sets are next to each workcell file, `<package>/cells/<cell>/<variant>/`. Never edit the generated files.
Change a composition or a descriptor, then:

```bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash
python3 tools/generate_cell.py              # all cells (gantry / --variant sim to narrow)
colcon build --packages-select cell_description
python3 tools/check_composition.py          # includes generate_cell.py --check
```

[tools/check_composition.py](../../tools/check_composition.py) also
checks names, limits, controllers, frames, borders, calibrations, the
collision matrix, the simulated grasp and the gantry's drive force, for a
module's `alternatives` too.

The descriptions a cell names must be built first: the URDF is expanded from
the install space.

## The cell template: modules and borders

A cell is a tree: its rig at the root (frame, table, fixtures, racks: all
hardware that is not a module), and on it modules: carriers (a linear axis, a gantry, a mobile base),
arms, tool changers, end effectors (grippers, nut runners) and sensors. Any module class may be
missing or there more than once, and new classes can be added. Each module
instance has one **input border**, `<instance>_mount`, and **output borders**
that other modules are mounted on. Borders are frames without geometry, as
REP 199's `flange` and `tool0`. A module's `parent` in the cell names the
output border its mount hangs on, or a link of the cell's rig; nothing
else of a vendor's frames is referred to.

| Class | Input | Outputs | Gantry | Robocoop, ACROBA |
|---|---|---|---|---|
| rig | (root) | `workcell` (z up), its links | `workcell` on `floor_link`, `gantry_link` | `workcell`, `world` |
| carrier | `<i>_mount` | `<i>_output` (moves) | `axis_mount` on `gantry_link`, `axis_output` | - |
| arm | `<i>_mount` (base_link's parent) | REP 199's `base_link`, `tool0` | `arm_mount` on `axis_output` | `arm_mount` on `world` |
| sensor | `<i>_mount` | its own frames, a pass-through `<i>_output` | `camera_mount` on `tool0`, `d405_link`, `camera_output` | ACROBA: `camera_mount` on `world`, `camera_link` |
| changer | `<i>_mount` | `<i>_coupling` | `changer_mount` on `camera_output`, `changer_coupling` | - |
| end effector | `<i>_mount` | `<i>_tcp`, named TCPs | `gripper_mount` on `changer_coupling`, `gripper_tcp` | `gripper_mount` on `tool0`, `gripper_tcp` |

- Each module class must declare its row's outputs: `arm` `base` and `flange`,
  `carrier` `output`, `changer` `coupling`, `gripper` `tcp`, `sensor` `base`
  (`REQUIRED_BORDERS`, checked by `borders`). The code's `module_class: gripper` is the
  template's end effector so far; a working nut runner would need its own class
  on that list.
- Names: a descriptor declares its borders by role in `frames`; a name
  starting with `_` is the instance's name plus it, so two grippers in one
  cell get `left_gripper_tcp` and `right_gripper_tcp`. A cell can add
  outputs for an instance (`frames` in its module entry), as the gantry's
  camera mount, a plate between tool0 and the changer, adds `camera_output`.
- Orientation: `workcell` has z up. A mount keeps the z axis of the output
  border it hangs on and lies on it (an offset along z and a turn about it
  only); every border's z points along the mounting direction, away from
  its parent, as tool0's does. A hanging axis therefore has `axis_mount` z
  down (the axis's own links are turned back on it).
- The URDF macros make the borders with
  [border.urdf.xacro](urdf/border.urdf.xacro): the gripper modules their
  mount and TCP (`instance`, default `gripper`), the gantry description the
  rest.
- `check_composition.py`'s `borders` check: every mount exists on a fixed
  joint on the parent the cell names, keeps that border's z and lies on its
  axis; every output lies below the mount, a TCP or an arm's base not behind a
  revolute joint keeps its z, and other outputs may point any way (a side port); borders carry no geometry.
- Reference: code outside a module refers to it by its borders only.
  `skill_base/robot_links.py` gets a module's links from its borders through
  the description (`links_below`, a held part's touch links; `moving_links`,
  the arm's links between `base_link` and `tool0`). `check_composition.py`'s
  `reference` check fails on a string literal in `skill_base`,
  `condition_skill`, `manipulation_skills`, `perception_skills` or
  `planning_agent` that is exactly a frame inside a module. A module's own
  software is exempt, each file listed with why in `REFERENCE_EXCEPTIONS`. It
  covers frames only, not joints, and Python, not YAML.
- Not modelled: stowed end effectors. A rack's slots would be outputs of the cell
  (`<rack>_slot_<n>`), and equipping an end effector would move its mount from a slot
  to the changer's coupling; the gantry's nut runner is stowed statically.

## Merging

A module's capabilities merge in this order: the descriptor's, its `models`
entry, its `variants` entry, the instance in the cell, the variant's `modules`
entry. `unverified` accumulates over the same layers.
`composition.Variant.modules()` returns the result, the description each module
announces; `tools/check_composition.py` and `tools/generate_cell.py` reject
invalid ones.

One rule everywhere: dicts merge key by key, anything else (lists included) is
replaced, and `null` removes an entry. Module contributions come first, in the
order the cell lists its modules; the cell's entries are merged on top, then
the variant's.

## Module descriptor (`modules/<type>.yaml`)

Existing descriptors: [ur.yaml](modules/ur.yaml),
[robotiq_2f_85.yaml](modules/robotiq_2f_85.yaml),
[robotiq_hande.yaml](modules/robotiq_hande.yaml),
`parker_linear_axis.yaml` (not in this repository),
`realsense_d405.yaml` (not in this repository),
[zivid_one_plus_m.yaml](modules/zivid_one_plus_m.yaml),
[photoneo_motioncam_m_plus.yaml](modules/photoneo_motioncam_m_plus.yaml),
`wingman_tool_changer.yaml` (not in this repository).

| Key | Meaning |
|---|---|
| `module_class` | `arm`, `carrier`, `changer`, `gripper` or `sensor` (the key was `kind` until 5 Nov 2026). The first `arm`'s first group is the end effector's parent group. |
| `joints` | The module's actuated joints, in order. Groups built from modules, and controllers without their own joint list, use them. |
| `groups` | `{name: {chain: [base, tip]} or {joints: [...]} or {links: [...]}, states: {state: {joint: value}}, kinematics: {...}}` |
| `joint_limits` | List of MoveIt joint-limit sources, `{package, file}` or inline dicts. |
| `end_effector` | Name of the module's group that is the SRDF end effector; its parent link is the cell's `parent` for the module. |
| `collision_replacements` | `{link: replacing link}`: where the expanded URDF has the replacing link, the SRDF gets `disable_default_collisions` for `link`, so MoveIt checks the replacement instead (the gripper's fingertip hulls, replaced by the pads). |
| `controllers` | Per variant name (`sim`, `real`): `{name: {type, params_files: [{package, file}], params: {...}, moveit: {default: bool}}}`. `params` are written inline into `ros2_controllers.yaml`; `moveit` puts the controller into MoveIt's list. |
| `moveit_controllers` | For every variant: `{name: {type, action_ns, default, joints?}}`, an action server MoveIt executes on that is no ros2_control controller (the gripper's `gripper_controller/joint_cmd`, served by `manipulation_skills`' `moveit_gripper_bridge`). Goes into MoveIt's list only; `check_composition.py` does not look for it in a controller parameter file. |
| `tcp` | Gripper only: `{xyz, rpy}` from the module's mount; the frame is its `tcp` border. The URDF module macro reads the same file. |
| `frames` | The module's borders by role (see the cell template): `mount`, and outputs `flange`, `coupling`, `tcp`; `base` for its own frame (an arm's base_link). `_suffix` is the instance's name plus it. Announced on `/modules/registry`, with the cell's `parent`. |
| `capabilities` | What the module can do, in the fixed vocabulary of `skill_base/capabilities.py` (not in this repository) (names, forms, SI units). Announced. |
| `unverified` | Capabilities taken from a datasheet or guessed, not measured. They count, but are flagged. |
| `models` | `{model: {capabilities, unverified}}` for a module type with several models (`ur5e`, `ur16e`); the cell picks one with `model`. |
| `variants` | `{variant: {capabilities, unverified}}` (variant `sim` or `real`) where a variant differs. |
| `geometry` | Gripper only: `{max_stroke, gap_per_stroke, open_threshold, closed_threshold, finger_reach, closed_tip_below_tcp}`, the gripper as the skills reason about it (condition_skill `GripperGeometry`); with grasp_width's max and the gripper group's finger joint and `closed` state it becomes the skills' `gripper.*` parameters (`composition.gripper_geometry`). |
| `body` | A camera for [depth_camera_module](urdf/depth_camera_module.urdf.xacro): `{size, xyz, mass}`, its housing as a collision box in its `base` frame; the mass for Gazebo. |
| `driver` | Gripper only: `{robotiq_model, goal_tolerance}` for Robotiq's ros2_control driver on the real cell (`composition.gripper_driver`, `manipulation_skills` `robotiq_gripper.launch.py`). |

## Cell composition (`cells/<cell>.yaml`)

| Key | Meaning |
|---|---|
| `modules` | `{instance: {type, parent?, frames?, model?, capabilities?, unverified?, calibration?, alternatives?}}` (`parent`: the output border, or a link of the cell's rig, the instance's mount hangs on; `frames`: outputs the instance adds), in SRDF group order (RViz offers the first group). `model` must match the description's `ur_type` for an arm. `capabilities` override the descriptor's for this cell. `calibration`: `{package, file, variants?}`, the measured values of this module in this cell (`{type}` in `file` is its type). `alternatives`: other types this instance can be, which the check tests too. |
| `urdf` | `{package, file, args}`: the description MoveIt plans with. The robot name comes from it. |
| `root` | The description's root link (frames are checked from it). |
| `virtual_joint` | `{name, parent_frame, child_link, type?}`, if the SRDF needs one. |
| `tcp` | `{module, frame?}`: the module whose `tcp` defines the TCP; the frame defaults to its `tcp` border. |
| `groups` | Extends module groups (states, kinematics) or adds groups. `modules: [a, b]` builds a group from the joints of those modules, in that order. |
| `planning_groups` | Groups the skills plan for: their joints must be limited and controlled. |
| `joint_limits` | Further joint-limit sources, merged after the modules'. |
| `controllers` | Overrides of the modules' controllers, for every variant. A controller the modules do not have names its module (`module: arm`) to drive that module's joints, else it lists `joints`. |
| `collisions` | `{checked: [[link, link], ...]}`: pairs that touch in the zero state only by chance (an arm stretched out into the cell's frame) and must stay checked. The generator leaves them out of the disabled pairs, the check accepts them. Also per variant. |
| `keep_collisions` | `[[side, side], ...]`, each side a link-name pattern (fnmatch) or a list of them: pairs the generator never disables as "Never", although `collisions_updater`'s random samples did not see them touch. For parts the robot reaches only in a small region (the gantry's tool rack). Pairs that touch in the zero state (unless under `collisions: checked`) or are adjacent stay disabled (the generator says so). Also allowed per variant, appended. `check_composition.py` reports a listed pair disabled as Never. |
| `variants` | `{sim: {...}, real: {...}}`, see below. |

Per variant: `urdf_args` (merged into `urdf.args`), `root`, `groups`,
`controllers`, `keep_collisions`, `joint_limits`, `trajectory_execution`,
`modules: {instance: {capabilities, unverified}}` (capability overrides for
this variant), and

- `controller_manager: {update_rate: ...}` when the variant starts its own
  controller manager (a simulation). Then `ros2_controllers.yaml` is generated.
  Its paths are `$(find-pkg-share pkg)/...` tokens and `$(this)` (the file
  itself, for inline parameters). `cell_description.composition.controller_manager_params(cell, variant)`
  resolves them into a temporary file at launch, because a controller's
  `params_file` needs absolute paths.
- `controller_files: [{package, file}]` when the manager is started elsewhere
  (a real driver). Only the composition check reads them.
- `gazebo: {urdf | urdf_args, root, grasp, carriers}`: the description Gazebo
  simulates and what the check verifies on it (graspable models, grasp joints
  and driver list agree; a carrier joint can push its load).

## In launch files

[cell_description/composition.py](cell_description/composition.py)
provides two helpers for launch files:

```python
from cell_description import composition

# move_group (or RViz) with the cell's generated semantics, kinematics,
# joint limits and controller list; returns a MoveItConfigsBuilder.
config = composition.moveit_builder("gantry", "sim", urdf_args={"ur_type": "ur16e"},
                                    package_name="gantry_description")
config = config.planning_pipelines(pipelines=["ompl"]).to_moveit_configs()

# gz_ros2_control's simulation_controllers
controllers_file = composition.controller_manager_params("gantry", "sim")
```

## Calibrations and swapping a module

A measured value corrects one border and belongs to the module in this cell,
not to its type, so it is a file of the cell's description package, named by
`calibration` in the cell. A file holding `{parent, child, xyz, rpy}` is compared
with that joint in the description; `status: estimate` in it gives a warning, so
an unmeasured pose does not pass for a measured one.

To change a swappable module (ACROBA's camera), change `type` in the cell and
swap the name into `alternatives`, regenerate, rebuild. The description reads
the type from the cell (`acroba.urdf.xacro`), so nothing else names it, and the
new type needs a calibration file. Keep pairs that the old pose might have
missed in `keep_collisions`, or a swap can inherit a lost check.

## Adding a cell

1. Compose its URDF from the module macros ([../urdf/](urdf/), the
   vendors' macros) and the `workcell_frame` macro.
2. Write `cells/<cell>.yaml` in the workcell's own package, install `cells/` and
   register the cell in the ament index there (see What lives where). Add a
   module descriptor for any new module type.
3. Generate, build, check (above). Review the "touching in the zero state"
   pairs the generator prints. They are disabled because they touch in the
   zero state, which is right for parts bolted together and wrong for parts
   that only happen to touch there.
4. Use `composition.moveit_builder` / `controller_manager_params` in the
   cell's launch files.
