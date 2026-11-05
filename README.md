# Workcell template

A draft convention for where the modules of a robot workcell meet, and a small tool set
for working with it in ROS 2 and MoveIt: one short workcell file per workcell, unmodified
vendor packages, a generator of MoveIt's configuration, and a checker for the seams
between the parts.

The convention is in [CONVENTION.md](CONVENTION.md). It needs none of this code.

**Status:** an early version. It holds two real workcells of the Bern University of
Applied Sciences (BFH): the ACROBA demonstrator (a UR5e with a Robotiq 2F-85 and a depth
camera, a Zivid or a Photoneo) and the robocoop cell (a UR3e with a Hand-E). It generates
and checks their MoveIt configuration and starts `move_group` without a robot. Their
simulation variants are generated here too, but running them needs simulation and skill
packages that are not in this repository; neither real workcell has run with these
descriptions on anything but the driver's mock hardware. Tested on ROS 2 Lyrical on Ubuntu.

## Try it

No vendor code is copied into this repository. `ur_description` and the other UR packages
come from apt, Robotiq's description from its own repository.

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p ws/src && cd ws/src
git clone <this repository> workcell-template
git clone https://github.com/robotiq/robotiq-ros.git && git -C robotiq-ros checkout be8aa38
cd ..
# Robotiq's description names its driver packages as dependencies; planning needs none of them.
colcon build --symlink-install --packages-up-to acroba_description robocoop_description \
  --packages-ignore robotiq_driver robotiq_msgs robotiq_controllers robotiq_hardware_tests robotiq_tsf
source install/setup.bash

cd src/workcell-template
python3 tools/generate_cell.py --check                 # generated files up to date
python3 tools/check_composition.py --variant real      # every check, both workcells
ros2 launch acroba_description acroba_moveit.launch.py standalone:=true launch_rviz:=true
```

`check_composition.py` without `--variant` also checks the simulation variants, and then
reports their simulation packages missing.

## How it is organised

Module files are shared by every workcell, in `cell_description`; a workcell's own files are
in its own package (`<pkg>`, here `acroba_description` and `robocoop_description`).

| Where | What |
|---|---|
| `src/cell_description/modules/<type>.yaml` | A module descriptor: what a vendor package does not say and a workcell needs. One per module type |
| `src/cell_description/urdf/<type>_module.urdf.xacro` | A wrapper around a vendor's macro, which adds the module's borders and changes nothing of the vendor's. Only where a module needs one |
| `src/cell_description/urdf/border.urdf.xacro`, `workcell_frame.urdf.xacro` | The template's macros: a border, and the frame `workcell` |
| `src/cell_description/cell_description/composition.py` | Finds a workcell by its name and loads its file with the descriptors |
| `src/<pkg>/cells/<workcell>.yaml` | A workcell file: which modules, and which border each hangs on |
| `src/<pkg>/cells/<workcell>/<variant>/` | Generated: SRDF, kinematics, joint limits, MoveIt's controller list, for simulations the controller manager's parameters. Never edit these |
| `src/<pkg>/urdf/<workcell>.urdf.xacro` | The workcell's description: mounts the modules on their borders |
| `src/<pkg>/urdf/<workcell>_rig.urdf.xacro`, `meshes/` | The rig: the workcell's hardware that is not a module |
| `src/<pkg>/config/<type>_calibration.yaml` | A module's measured values in this workcell, or an estimate marked as one |
| `src/<pkg>/launch/<workcell>_moveit.launch.py` | Starts `move_group` for the workcell, found by name; names no module |
| `tools/generate_cell.py` | Writes the generated files |
| `tools/check_composition.py` | Tests the seams. Compares files with each other, and cannot see the robot |
| `tools/check_vendored.py`, `third_party/vendored.yaml` | Checks vendor packages kept in the repository byte for byte; here there are none |

A workcell package installs `cells/` and registers each workcell in the ament index
(resource type `cell_compositions`), so tools and launch files find it by name. The format
of descriptors and workcell files is in [src/cell_description/README.md](src/cell_description/README.md).

## Adding a module or a workcell

1. **A new module type:** a descriptor in `cell_description/modules/`, and, where the
   vendor's macro needs one, a wrapper in `cell_description/urdf/` that adds the borders and
   changes nothing of the vendor's.
2. **A new workcell:** a package of its own with `cells/<workcell>.yaml`, a description that
   mounts the modules on the borders it names and calls `workcell_frame` once, its rig, and a
   calibration file per module whose pose it measures. Install `cells/` and register the
   workcell in the ament index, as `acroba_description`'s CMakeLists.txt does.
3. Run `python3 tools/generate_cell.py <workcell>`, and read what it says about pairs
   touching in the zero state: they get disabled. Read the SRDF's diff too: a newly disabled
   pair of parts that can meet is a lost collision check.
4. Run `python3 tools/check_composition.py`.

The generator cannot deduce controller descriptions. A descriptor refers to the vendor's
parameter files or holds the parameters itself.

## License

Copyright 2026 Berner Fachhochschule (BFH). Apache-2.0, see [LICENSE](LICENSE) and
[NOTICE](NOTICE). Vendor packages keep their own licences.
