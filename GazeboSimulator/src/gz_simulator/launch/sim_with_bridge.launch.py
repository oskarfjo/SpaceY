import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    gz_simulator_dir    = get_package_share_directory('gz_simulator')
   

    world_path = os.path.join(gz_simulator_dir, 'worlds', 'RocketWorld2.sdf')

    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '--gui'],
        output='screen'
    )

    bridge_node = Node(
        package='gz_bridge',
        executable='gz_bridge',
        name='gz_bridge',
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
        bridge_node
    ])