# acroba_description

Part of [skill_ws](../../README.md#packages). The ACROBA demonstrator (BFH):
a UR5e with a Robotiq 2F-85 on a table, with a frame across the table behind
the arm.

Composed like the other cells ([cells README](../cell_description/README.md)):

| Part | From |
|---|---|
| Arm | vanilla `ur_description`, `ur_type` ur5e, **no tf_prefix** |
| Arm kinematics | [config/ur5e_calibration.yaml](config/ur5e_calibration.yaml), the real robot's calibration (UR5e 20205500232) |
| Gripper | `cell_description`'s `robotiq_2f_85_module`: Robotiq's UR coupling (11 mm) and 2F-85, TCP `gripper_tcp` 148 mm from `tool0` ([descriptor](../cell_description/modules/robotiq_2f_85.yaml)), the same module as the block and gantry cells |
| Rig | [urdf/acroba_rig.urdf.xacro](urdf/acroba_rig.urdf.xacro), the hardware in no vendor package: the cell, [meshes/ACROBA_Demonstrator_ROS.stl](meshes/ACROBA_Demonstrator_ROS.stl) (table, base plate, frame; visual and collision), and the cable cover, `cable_protection_link` on `tool0`, the demonstrator description's box (34 x 22 x 56 mm) |
| `workcell` | on `world`, identity; the arm's base at the origin on its base plate (10 mm, r 85 mm), the table top 10 mm lower |
| Composition | [cells/acroba.yaml](../acroba_description/cells/acroba.yaml), variants `sim` and `real` |
| Simulation | `sim_gazebo:=true` ([acroba_sim](../acroba_sim/README.md), dartsim): ros2_control blocks, wrist sensor, the 2F-85's pads, wrist spring, rubber springs and raised finger velocity limits as on the gantry, grasp joints (none yet); the reasoning for the Gazebo parts is in the xacro |

## Run

```bash
# Look at the cell and plan, without the robot (own ROS_DOMAIN_ID next to a sim)
ros2 launch acroba_description acroba_moveit.launch.py standalone:=true launch_rviz:=true

# On the real cell: the UR driver with this description and the robot's
# calibration (its ros2_control block, cell_description's ur_driver macro;
# use_mock_hardware:=true without the robot), then MoveIt, then the skills
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<ip> \
  description_file:=$(ros2 pkg prefix acroba_description)/share/acroba_description/urdf/acroba.urdf.xacro \
  kinematics_params_file:=$(ros2 pkg prefix acroba_description)/share/acroba_description/config/ur5e_calibration.yaml
ros2 launch acroba_description acroba_moveit.launch.py
ros2 launch manipulation_skills manipulation.launch.py cell:=acroba
```

## Where it came from

The BFH repository `demonstrator_ros_setup` (GitLab
`work/acroba-demonstrator/demonstrator_ros_setup`, commit 7ff73d5, June
2023), a ROS 1 (Noetic-era) setup:
`acroba_demonstrator_description` (URDF and meshes; of it, only the cell mesh
came here, see below),
`acroba_demonstrator_config` (MoveIt Setup Assistant package),
`acroba_demonstrator_control` (bringup, the calibration, a Robotiq RTU
gripper node), a 500 MB rosbag of the box with parts. What changed in coming
here:

- **ROS 2:** its xacros use the ROS 1 `ur5e.xacro` and
  `robotiq_2f_85_gripper_visualization`; here the arm is Lyrical's
  `ur_description` and the gripper `cell_description`'s 2F-85 module.
- **TCP:** the demonstrator's `virtual_tcp_link` is 174 mm from `tool0`
  (a TODO there says 174.3); here the TCP is `gripper_tcp`, 148 mm, the pad
  centre at a 65 mm grasp, as on the other 2F-85 cells. The fingertips end
  at 167 mm.
- **Coupling:** theirs is a 16.4 mm cylinder with the gripper 11.5 mm from
  `tool0` (Robotiq GRP-ES-CPL-062, its mesh unused); here the 2F-85
  module's vanilla `ur_to_robotiq` adapter, 11 mm.
- **Gripper orientation:** theirs turns the gripper a quarter turn about
  `tool0`'s z on the coupling, for the ROS 1 gripper model's frame; here
  the module's mounting, as on the gantry. Which way the fingers close
  relative to `tool0` has not been compared with the real gripper.
- **Cell placement:** theirs puts the mesh 10 mm above `world`, so the
  robot's base plate reaches 17 mm into the arm's base (its SRDF disables
  the pair). Here the arm stands on the plate, the mesh 7 mm below `world`.
- **Joint limits:** theirs leaves the UR's ±2π; here the UR soft limits
  (`safety_limits`, 0.15 rad inside), as ros2_control enforces them.
  Velocity and acceleration come from `ur_moveit_config`.
- `config/ur5e_calibration.yaml` is `acroba_demonstrator_control`'s
  `config/UR5e_20205500232_calibration.yaml`, unchanged.
- `acroba_home` is their MoveIt config's `home_arm`.

The cell mesh, `meshes/ACROBA_Demonstrator_ROS.stl`, is
`acroba_demonstrator_description/meshes/` at 7ff73d5, byte for byte (SHA-256
`11b9692faa26a7d3f8e3f3c480b844479c41cb30e01bf6549094bd31f2662c55`). It is part
of this cell's rig, so it is ours now and not vendored. From 24 Oct to 5 Nov 2026
it lived in a package `acroba_cell_description`, upstream's package ported to
ROS 2 and listed in `third_party/vendored.yaml`; the port's xacros, launch and
limits were unused and are in git before that. The coupling and cable protector
meshes (369k and 23k triangles, in CAD coordinates) were never used.

Not taken over: the MoveIt config (SRDF, limits and controllers are
generated here), the ROS 1 control and bringup launch files, the Robotiq
RTU node (the real gripper runs on Robotiq's ros2_control driver here), the
Zivid capture scripts, the rosbag. The Zivid's pose from
`zivid_camera.launch` is in `config/zivid_one_plus_m_calibration.yaml`.

## Open

- The real cell is untested with this description. With the driver's mock
  hardware (24 Oct 2026): hardware `ur5e` initialised from this description,
  `workcell` and `gripper_tcp` in TF from it, MoveIt executing on
  `joint_trajectory_controller` (the driver's; Lyrical's has no scaled one)
  to within 3.5 mrad.
- Finger closing direction against the real gripper (above).
- No parts in the simulation, and in Gazebo only the camera's housing, no images.
  The camera (a Zivid or a Photoneo, `cells/acroba.yaml`; see the cells README for
  changing it) has no driver started. The Zivid's pose is
  upstream's, June 2023; the Photoneo's is an estimate; both housings' sizes
  are unchecked.
- The frame's crossbeam box and panel are as the mesh has them, not
  measured on the cell.
