# A convention for robot workcells

A draft, for discussion between teams. It has run on three workcells of two labs,
with arms of one vendor. No other team has used it yet.

## The problem

ROS describes an entire workcell as a single robot model. URDF knows only links and
joints, and each vendor names its frames its own way, so nothing says where one part
ends and the next begins. REP 199 settles this for the arm. Above the wrist, each team
decides for itself, and whoever takes over a part, a skill or a task from another team
has to find out and adapt.

## The proposal

Teams agree on one thing only: where modules meet. How a module is built, what its
vendor's description contains and which tools produce a workcell's configuration stay
each team's own affair. You can follow this with hand-written URDF and none of the
code in this repository.

### What each module declares

**Borders.** Hardware changes, and what refers to it should not have to. Every module
therefore has a few links without geometry, as names that stay when the module behind
them changes: one input border, `<instance>_mount`, where it is fastened, and named
output borders, where others are fastened. `<instance>` is the module's name in this
workcell, so two grippers get distinct borders.

**Module class.** Each module belongs to one of a few classes, and each class must offer
certain output borders, so a team knows what to publish for its module.

| Class | Must offer | What it is for |
|---|---|---|
| carrier | `<i>_output` | the next module rides on it |
| arm | `base_link`, `tool0` | REP 199's frames; the next module hangs on `tool0` |
| sensor | its base frame | where its data is measured from; add `<i>_output` if others hang on it |
| changer | `<i>_coupling` | the end effector hangs on it |
| end effector | `<i>_tcp` | what tasks name |

**Rig.** Whatever in the workcell is not a module is its rig: tables, frames, racks and
brackets, whoever made or sold them. The rig is mounted on nothing; its fixed part is the
root of the tree, provides the frame `workcell` and the links modules hang on, and names
those as it needs. A part of it may move with a module without being one (a cable cover
on `tool0`). A team that needs a new class can add it. In this repository's code a module's
class is `module_class` in its descriptor, and the end effector's is called `gripper`, so far.

### What everyone respects

**Direction.** A task should not need new signs because an arm hangs.

- `workcell` is fixed in the workcell with z up, and tasks are written in it.
- A TCP's z points along the tool, because skills approach along it.
- A mount on another module's output border keeps that border's z, offset only along it
  and turned only about it, so that modules can be exchanged.
- A mount on the rig may sit as the rig requires, and other outputs may point
  any way (a connector on a module's side).

The arm keeps REP 199's frames, with one departure: REP 199 attaches end effectors to
`flange`, whose x points away from the arm, while mounts here hang on `tool0`, whose z
points along the mounting direction.

**Reference.** Others refer to a module only by its borders, never by its vendor's
frames. A TCP changes with its end effector, and its name does not.

## What is still open

- **Which way x points at a border.** ISO 9409-1 fixes it at a flange plate, by its pin
  hole, but a robot description does not record that hole, so no check of the files can
  show that a frame follows it.
- **Where on a module its origin sits.** Two conforming teams can split the same stack of
  plates differently.
- **Which way fingers close, or a camera looks.** Left to each vendor.
- **Stowed end effectors**, and a **moving base**, which has no link fixed in the workcell.
- **Joints.** The rule covers frames. Skills may still name an arm's joints.
- **Two active modules of one class** have not run.

## What it does not do

It says how modules meet, not what each can do. It does not make modules physically
interchangeable, and a check of the files cannot see the robot: a gripper mounted a
quarter turn off its description passes every check.
