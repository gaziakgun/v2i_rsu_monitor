from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package='v2i_rsu_monitor',
            executable='rsu_monitor',
            name='rsu_monitor',
            output='screen',
            emulate_tty=True,
        )
    )

    return ld
