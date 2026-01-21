import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share_dir = get_package_share_directory('semantic_reasoning')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                    default_value=os.path.join(share_dir, 'config', 'semantic_reasoning.yaml'),
                    description='FPath to the ROS2 parameters file to use.')
    
    return LaunchDescription([
        params_declare,
        # Semantic mapper node
        Node(
            package='semantic_reasoning',
            executable='semantic_mapper_node',
            name='semantic_mapper',
            output='screen',
            parameters=[parameter_file]
        ),
        
        # Query handler node
        Node(
            package='semantic_reasoning',
            executable='query_handler_node',
            name='query_handler',
            output='screen',
            parameters=[parameter_file]
        )
    ])