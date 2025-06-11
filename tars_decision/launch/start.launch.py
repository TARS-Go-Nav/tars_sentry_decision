import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('tars_decision'), 'config', 'dog.yaml')
    decision_node = Node(
        package='tars_decision',
        executable='tars_dog',
        name='tars_dog',
        output='screen',
        parameters=[config_file_path],
        # prefix='gdb -ex run --args'
    )
    ld = LaunchDescription()
    ld.add_action(decision_node)
    return ld