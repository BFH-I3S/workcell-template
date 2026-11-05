# Workcell template

A draft convention for where the modules of a robot workcell meet, and a small tool set
for working with it in ROS 2 and MoveIt: one short workcell file per workcell, unmodified
vendor packages, a generator of MoveIt's configuration, and a checker for the seams
between the parts.

The convention is in [CONVENTION.md](CONVENTION.md). It needs none of this code.

**Status:** an early version, for planning only. It generates and checks MoveIt's
configuration and starts `move_group` without a robot. It contains no simulation, no driver
and no skills, and nothing here has moved a robot. Tested on ROS 2 Lyrical on Ubuntu.

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
colcon build --symlink-install --packages-up-to workcell_template \
  --packages-ignore robotiq_driver robotiq_msgs robotiq_controllers robotiq_hardware_tests robotiq_tsf
source install/setup.bash

cd src/workcell-template
python3 tools/check_workcell.py                 # all workcells, all checks
ros2 launch workcell_template moveit.launch.py workcell:=ur5e_2f85
```

## How it is organised

| Where | What |
|---|---|
| `workcell_template/modules/<type>.yaml` | A module descriptor: what a vendor package does not say and a workcell needs. One per module type |
| `workcell_template/workcells/<name>.yaml` | A workcell file: which modules, and which border each hangs on |
| `workcell_template/workcells/<name>/<variant>/` | Generated: SRDF, kinematics, joint limits, MoveIt's controller list. Never edit these |
| `workcell_template/urdf/` | The border and workcell-frame macros, wrappers around vendor macros, the example descriptions |
| `tools/generate_workcell.py` | Writes the generated files |
| `tools/check_workcell.py` | Tests the seams. Compares files with each other, and cannot see the robot |

There are two example workcells, a UR5e with a Robotiq 2F-85 and a UR3e with a Hand-E.
Their tables and tool poses are made up for the example.

## Adding a module or a workcell

1. **A new module type:** write a macro around the vendor's, which adds the borders and
   changes nothing of the vendor's, and a descriptor in `modules/`.
2. **A new workcell:** write its file in `workcells/`, and a description that mounts the
   modules on the borders it names and calls `workcell_frame` once.
3. Run `python3 tools/generate_workcell.py <name>`, and read what it says about pairs
   touching in the zero state: they get disabled. Read the SRDF's diff too: a newly disabled
   pair of parts that can meet is a lost collision check.
4. Run `python3 tools/check_workcell.py`.

The generator cannot deduce controller descriptions. A descriptor refers to the vendor's
parameter files or holds the parameters itself. The grippers here have no controller, so
MoveIt plans for the arm and does not move the fingers.

## License

Copyright 2026 Berner Fachhochschule (BFH). Apache-2.0, see [LICENSE](LICENSE) and
[NOTICE](NOTICE). Vendor packages keep their own licences.
