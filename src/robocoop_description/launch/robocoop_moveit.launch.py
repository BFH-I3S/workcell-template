"""
MoveIt for the robocoop cell: move_group with robocoop.urdf.xacro and the
cell's generated semantics, kinematics, joint limits and controller list
(robocoop_description/cells/robocoop.yaml, tools/generate_cell.py; variant real).

  ur_manipulator   the UR3e, base_link to tool0; the TCP is gripper_tcp
                   (Hand-E pad centre, 145 mm from tool0)

The cell's table, walls, roof and turning station are collision geometry of
the model, so plans avoid them without planning-scene objects.

In the simulation robocoop_sim.launch.py includes it (sim:=true,
use_sim_time:=true). On the real cell, next to ur_robot_driver started with
this description and an empty tf_prefix (the robocoop repository's own launch uses ur3e_, which the
skills and this configuration do not):
    ros2 launch robocoop_description robocoop_moveit.launch.py

Without the robot, to look at the cell and plan (nothing executes): a
robot_state_publisher and a joint_state_publisher at robocoop_home. Run it
under its own ROS_DOMAIN_ID next to a simulation (CLAUDE.md, Known issues):
    ros2 launch robocoop_description robocoop_moveit.launch.py standalone:=true launch_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from cell_description import composition

# robocoop_home (cells/robocoop.yaml): the zero state lies in the cell's frame.
STANDALONE_POSITIONS = {
    "shoulder_pan_joint": 1.5707, "shoulder_lift_joint": -1.5707, "elbow_joint": 1.5707,
    "wrist_1_joint": -1.5707, "wrist_2_joint": -1.5707, "wrist_3_joint": 0.0,
}


def launch_setup(context):
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    moveit_config = (
        composition.moveit_builder("robocoop", "sim" if sim else "real", package_name="ur_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    standalone = IfCondition(LaunchConfiguration("standalone"))

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # As in the skills' move_group: a start state up to 0.1 rad
            # outside the joint limits is clamped instead of rejected.
            {"ompl.start_state_max_bounds_error": 0.1, "use_sim_time": use_sim_time},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
        condition=standalone,
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description,
                    {"zeros": STANDALONE_POSITIONS}],
        condition=standalone,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("ur_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return [move_group_node, robot_state_publisher, joint_state_publisher, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "standalone", default_value="false",
            description="Publish the description and fixed joint states (robocoop_home) "
                        "without a robot, to look at the cell and plan."),
        DeclareLaunchArgument("sim", default_value="false",
                              description="Plan with the cell's sim variant (Gazebo's controllers)."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="false",
                              description="RViz with the MotionPlanning panel."),
        OpaqueFunction(function=launch_setup),
    ])
