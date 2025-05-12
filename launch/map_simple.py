import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Map server parameters (ensure the correct path to the config folder)
    map_server_config_path = os.path.join(
        get_package_share_directory('articubot_one'),  # Ensure this is correct
        'config',  # Path to the 'config' directory
        'map_server_params.yaml'  # The YAML parameter file
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[map_server_config_path],
        name='map_server',  # Explicitly define the name of the node
        emulate_tty=True  # This can help in some cases where the terminal output is mangled
    )

    # Add the map server node to the launch description
    ld.add_action(map_server_cmd)

    return ld
