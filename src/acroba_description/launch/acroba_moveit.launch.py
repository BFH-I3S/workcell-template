"""
MoveIt for the ACROBA demonstrator: move_group with acroba.urdf.xacro and the
cell's generated semantics, kinematics, joint limits and controller list
(acroba_description/cells/acroba.yaml, tools/generate_cell.py; variant real).

  ur_manipulator   the UR5e, base_link to tool0; the TCP is gripper_tcp
                   (2F-85 pad centre, 148 mm from tool0)

The cell's table, base plate and frame are collision geometry of the model,
so plans avoid them without planning-scene objects.

In the simulation acroba_sim.launch.py includes it (sim:=true,
use_sim_time:=true). On the real cell, next to ur_robot_driver started with
this description and an empty tf_prefix:
    ros2 launch acroba_description acroba_moveit.launch.py

Without the robot, to look at the cell and plan (nothing executes): a
robot_state_publisher and a joint_state_publisher at acroba_home. Run it
under its own ROS_DOMAIN_ID next to a simulation (CLAUDE.md, Known issues):
    ros2 launch acroba_description acroba_moveit.launch.py standalone:=true launch_rviz:=true
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

# acroba_home (cells/acroba.yaml), the demonstrator MoveIt configuration's home_arm.
STANDALONE_POSITIONS = {
    "shoulder_pan_joint": 1.57, "shoulder_lift_joint": -1.57, "elbow_joint": 1.57,
    "wrist_1_joint": -1.57, "wrist_2_joint": -1.57, "wrist_3_joint": 0.0,
}


def launch_setup(context):
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    moveit_config = (
        composition.moveit_builder("acroba", "sim" if sim else "real", package_name="ur_moveit_config")
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
            description="Publish the description and fixed joint states (acroba_home) "
                        "without a robot, to look at the cell and plan."),
        DeclareLaunchArgument("sim", default_value="false",
                              description="Plan with the cell's sim variant (Gazebo's controllers)."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="false",
                              description="RViz with the MotionPlanning panel."),
        OpaqueFunction(function=launch_setup),
    ])
