from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cloud_integrated')
    param_file = os.path.join(pkg_share, 'config', 'cloud_integrated.yaml')

    return LaunchDescription([
        Node(
            package='cloud_integrated',
            executable='cloud_integrator_node',
            name='cloud_integrator_node',
            output='screen',
            parameters=[param_file]
        ),
        Node(
            package='cloud_integrated',
            executable='armor_judge_node',
            name='armor_judge_node',
            output='screen',
            parameters=[param_file]
        )
    ])
