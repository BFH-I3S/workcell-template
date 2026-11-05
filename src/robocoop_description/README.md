# robocoop_description

Part of [skill_ws](../../README.md#packages). The robocoop cell (BFH): a UR3e
with a Robotiq Hand-E on an open table with a portal frame across it behind
the arm, a turning station and two play areas, green and red: robocoop's
second cell, `robocoop_cell_2` (its first, `robocoop_cell`, is enclosed by
glazing, a rear wall and a roof).

Composed like the other cells ([cells README](../cell_description/README.md)):

| Part | From |
|---|---|
| Arm | vanilla `ur_description`, `ur_type` ur3e, **no tf_prefix** |
| Arm kinematics | [config/ur3e_calibration.yaml](config/ur3e_calibration.yaml), the real robot's calibration |
| Gripper | `cell_description`'s `robotiq_hande_module`: an 11 mm UR coupling and Robotiq's own Hand-E description (`robotiq_description`, robotiq-ros), TCP `gripper_tcp` 145 mm from `tool0` ([descriptor](../cell_description/modules/robotiq_hande.yaml)) |
| Rig | [urdf/robocoop_rig.urdf.xacro](urdf/robocoop_rig.urdf.xacro), the hardware in no vendor package: cell 2's [meshes/](meshes/) `Aufbau_Zelle_SIPBB.stl`, the play areas and the turning station, placed as `robocoop_cell_2_macro.xacro` places them |
| Cell collision | boxes measured on the cell 2 mesh: table (top z -0.013), the arm's base plate (128 x 128 mm, top z 0), the portal's posts and beam, the turning station |
| `workcell` | on `world`, identity; the arm's base at the origin on its base plate, the table top 13 mm lower |
| Composition | [cells/robocoop.yaml](../robocoop_description/cells/robocoop.yaml), variants `sim` and `real` |
| Simulation | `sim_gazebo:=true` (robocoop_sim): ros2_control blocks, wrist sensor, grasp joints, the Hand-E's Gazebo pads |

## Run

```bash
# Look at the cell and plan, without the robot (own ROS_DOMAIN_ID next to a sim)
ros2 launch robocoop_description robocoop_moveit.launch.py standalone:=true launch_rviz:=true

# On the real cell: the UR driver with this description and the robot's
# calibration (its ros2_control block, cell_description's ur_driver macro;
# use_mock_hardware:=true without the robot), then MoveIt, then the skills
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ip> \
  description_file:=$(ros2 pkg prefix robocoop_description)/share/robocoop_description/urdf/robocoop.urdf.xacro \
  kinematics_params_file:=$(ros2 pkg prefix robocoop_description)/share/robocoop_description/config/ur3e_calibration.yaml
ros2 launch robocoop_description robocoop_moveit.launch.py
ros2 launch manipulation_skills manipulation.launch.py cell:=robocoop
```

## Where it came from

The BFH repository `cell-robocoop` (GitLab `coop_roadshow/robocoop/cell-configs`,
commit c079632, Dec 2025; updated to 5f1550a, Oct 2026, which added cell
2). What changed in coming here:

- **The rig's meshes:** `meshes/Aufbau_Zelle_SIPBB.stl`, `play_area_green.stl`,
  `play_area_red.stl` and `Wendestation_m.stl` are
  `cell_setup/robocoop_cell_description/meshes/` at 5f1550a, byte for byte
  (SHA-256 `3355d518...`, `1612c100...`, `2d35b9e3...`, `119dfbad...`). They are
  the cell's rig, so ours now and not vendored. Until 5 Nov 2026 the whole
  package (78 MB, both cells' CAD, the Hand-E mesh, its macros) was vendored
  unmodified as `robocoop_cell_description`; the rest was never used here (a
  copy of it is in `~/robocoop_cell_description_5f1550a`, and in git before).

- **Prefix:** robocoop runs the arm with `tf_prefix` `ur3e_`
  (`ur3e_tool0`, ...). Here the arm is unprefixed, as the skills and the
  module descriptors expect, so `ur_robot_driver` must run with an empty
  `tf_prefix` on the real cell.
- **TCP:** robocoop's `tcp_link` is at 157 mm, the fingertips. Here the TCP is
  `gripper_tcp` at the pad centre, 145 mm, which is how the 2F-85's TCP is
  defined too. The measurement is in the module descriptor.
- **Gripper:** robocoop had the Hand-E as one static mesh with closed
  fingers and one 20 x 25 mm collision box for both. Here it is Robotiq's
  description with its two prismatic fingers, which the planning scene
  holds open (0) while `/joint_states` does not carry them. robocoop's mesh,
  which includes the UR coupling, gave the coupling's 11 mm.
- **Turning station:** it had a visual mesh only; here it also has a
  collision box.
- **Collision:** robocoop's `robocoop_cell_2` keeps cell 1's
  `collision_cell.stl` (the enclosure with rear wall and roof), which cell 2
  does not have. Here cell 2 has boxes around its own geometry instead.
- **Joint limits:** robocoop's MoveIt position limits are kept, in the
  composition. Velocity and acceleration come from `ur_moveit_config`, as
  for the other UR cells.
- `config/ur3e_calibration.yaml` is robocoop's
  `robocoop_cell_control/config/robocoop_calibration.yaml`, unchanged.
  Re-extract it with `ur_calibration` if the arm is swapped or recalibrated.

Not taken over: the robocoop repository's MoveIt package (the SRDF, limits and
controllers are generated here), its control launch files, vision (stereo
uEye cameras, ArUco brick locator), `move_bricks` and `move_robot`. The
robotiq and ueye submodules are on BFH's GitLab (login needed). The robotiq
one (bf1076f) is robocoop's own gripper driver, `robotiq_ctrl`: the URCap
socket on port 63352, raw 0-255 positions with auto-calibration, a
`MoveGripper` action; no description.

## Open

- The gripper runs on Robotiq's driver (`manipulation_skills`
  `robotiq_gripper.launch.py cell:=robocoop`, tried on the ros2_control mock
  only). The skills compute openings from the Hand-E's geometry in its
  module descriptor (50 mm stroke); in simulation `pick` and `place` work
  with it, on the real gripper it is untested.
- The real cell is untested with this description. With the driver's mock
  hardware (24 Oct 2026): hardware `ur3e` initialised from this description,
  `workcell` and `gripper_tcp` (0.145) in TF from it, MoveIt executing on
  `joint_trajectory_controller` to within 5 mrad.
