import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share_dir = get_package_share_directory('rrt_planner')
    parameter_file = LaunchConfiguration('params_file')
    map_path = os.path.join(share_dir, 'map', 'map.yaml')
    rviz_config_file = os.path.join(share_dir, 'config', 'config.rviz')

    params_declare = DeclareLaunchArgument('params_file',
                    default_value=os.path.join(share_dir, 'config', 'rrt_planner.yaml'),
                    description='FPath to the ROS2 parameters file to use.')
    
    return LaunchDescription([
        params_declare,
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{'yaml_filename': map_path}],
            output='screen'
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
        
        # RRT Planner
        Node(
            package='rrt_planner',
            executable='rrt_planner',
            parameters=[parameter_file],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])