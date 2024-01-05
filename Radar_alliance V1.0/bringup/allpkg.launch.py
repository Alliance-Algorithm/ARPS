import launch
import launch_ros

def generate_launch_description():
    exec_cpp = launch_ros.actions.Node(
        package = 'radar_remap_cpp',
        executable = 'points_subscriber',
        name = "cpp",
        output = 'screen'
    )
    exec_py = launch_ros.actions.Node(
        package = 'radar_yolov5_py',
        executable = 'points_publisher',
        name = "py",
        output = 'screen'
    )

    return launch.LaunchDescription(
        [
            exec_cpp,
            exec_py
        ]
    )