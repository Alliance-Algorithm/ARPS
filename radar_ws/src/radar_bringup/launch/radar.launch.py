import launch
import launch_ros # type: ignore


def generate_launch_description():
    ld = launch.LaunchDescription()

    ld.add_action(
        launch_ros.actions.Node(
            package="radar_updater",
            executable="radar_updater",
            respawn=True,
            respawn_delay=2.0,
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package="radar_detector",
            executable="radar_detector",
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
    return ld
