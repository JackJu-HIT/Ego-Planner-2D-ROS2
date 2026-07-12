from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("rviz_car_sim"), "config", "rviz.rviz"]
                ),
                description="Path to the RViz config file.",
            ),
            Node(
                package="ego_planner",
                executable="motion_plan",
                name="motion_plan",
                output="screen",
            ),
            Node(
                package="rviz_car_sim",
                executable="fake_sim_node",
                name="fake_sim_node",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
