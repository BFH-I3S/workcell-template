"""
move_group for a workcell, without a robot: the description, fixed joint states
and MoveIt with the workcell's generated configuration, to look at it and plan.
Nothing executes.

    ros2 launch workcell_template moveit.launch.py workcell:=ur5e_2f85
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from workcell_template import composition

# Clear of the table: straight joint zeros would lay the arm into it.
START = {"shoulder_lift_joint": -1.5707, "wrist_1_joint": -1.5707}


def launch_setup(context):
    workcell = LaunchConfiguration("workcell").perform(context)
    config = (
        composition.moveit_builder(workcell, "real", package_name="ur_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[config.robot_description]),
        Node(package="joint_state_publisher", executable="joint_state_publisher", output="log",
             parameters=[config.robot_description, {"zeros": START}]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config.to_dict()]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("workcell", description="A workcell file's name, for example ur5e_2f85."),
        OpaqueFunction(function=launch_setup),
    ])
