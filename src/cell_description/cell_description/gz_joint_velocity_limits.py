"""
gz_joint_velocity_limits.py
---------------------------
A Gazebo system (Python, loaded by gz-sim's PythonSystemLoader) that sets
the velocity limits of named joints of its model in the physics engine,
leaving the description's limits as they are.

Why it exists: under dartsim a joint driven by a velocity command, which is
how gz_ros2_control drives position-commanded and mimic joints, loses that
command once it reaches its velocity limit, and then coasts at the limit
until something stops it. The Robotiq 2F-85's finger joints have 0.5 rad/s
in the vendored URDF, which the simulated linkage reaches in normal use, so
the fingertips ran away onto their position limits (see the gripper module,
3.). Raising the limit in Gazebo keeps every joint servoed; the gripper's
speed comes from sim_gripper_driver's ramp, not from the limit.

The same happens at a position limit: a joint sitting on it does not take a
velocity command at all. The Robotiq Hand-E's fingers are commanded to
exactly their URDF limits (0 open, 0.025 m closed), so under dartsim a finger
opened or closed fully stuck there for good, and the right finger, which
starts at 0, never followed the left one as a mimic joint (Oct 2026).
<position_limits>lower upper</position_limits> widens them in the engine, so
that the commanded range lies inside.

In a description, as a model plugin

    <gazebo>
      <plugin filename="gz-sim-python-system-loader-system"
              name="gz::sim::systems::PythonSystemLoader">
        <module_name>cell_description.gz_joint_velocity_limits</module_name>
        <joints>joint_a joint_b</joints>
        <velocity>10.0</velocity>
        <position_limits>-0.002 0.027</position_limits>  (optional)
      </plugin>
    </gazebo>

The velocity limit is symmetric, rad/s (m/s for prismatic joints); the
position limits, if given, apply to every named joint. The module is found
through PYTHONPATH, so Gazebo must run with the workspace sourced.
"""

from gz.math import Vector2d
from gz.sim import Joint, Model

# Physics creates the joints after the first update; the limits are sent this
# many times, so that one reaches the engine once the joint exists.
REPEATS = 100


def _sdf_value(sdf, getter, key, default):
    if not sdf.has_element(key):
        return default
    value = getattr(sdf, getter)(key)
    # Some sdformat bindings return (value, found).
    return value[0] if isinstance(value, tuple) else value


class JointVelocityLimits:

    def configure(self, entity, sdf, ecm, event_mgr):
        self.model = Model(entity)
        self.names = _sdf_value(sdf, "get_string", "joints", "").split()
        self.limit = float(_sdf_value(sdf, "get_double", "velocity", 10.0))
        bounds = _sdf_value(sdf, "get_string", "position_limits", "").split()
        self.position_limits = [Vector2d(float(bounds[0]), float(bounds[1]))] if len(bounds) == 2 else None
        self.remaining = REPEATS

    def pre_update(self, info, ecm):
        if self.remaining <= 0:
            return
        self.remaining -= 1
        for name in self.names:
            joint = Joint(self.model.joint_by_name(ecm, name))
            if joint.valid(ecm):
                joint.set_velocity_limits(ecm, [Vector2d(-self.limit, self.limit)])
                if self.position_limits:
                    joint.set_position_limits(ecm, self.position_limits)
        if self.remaining == 0:
            missing = [n for n in self.names if not Joint(self.model.joint_by_name(ecm, n)).valid(ecm)]
            if missing:
                print(f"gz_joint_velocity_limits: joints not found in the model: {missing}")


def get_system():
    return JointVelocityLimits()
