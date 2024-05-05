import launch
import launch_ros

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
# from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart

# from launch.substitutions import (
#     Command,
#     FindExecutable,
#     LaunchConfiguration,
#     PathJoinSubstitution,
# )
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = launch.LaunchDescription()

    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             [FindPackageShare("fast_lio"), "/launch", "/mapping.launch.py"]
    #         )
    #     )
    # )

    ld.add_action(
        launch_ros.actions.Node(
            package="radar_serial",
            executable="radar_serial",
            respawn=True,
            respawn_delay=2.0,
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            respawn=True,
            respawn_delay=2.0,
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package="car_detect",
            executable="car_detect",
            respawn=True,
            respawn_delay=2.0,
        )
    )

    return ld
