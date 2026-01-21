import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_turtlebot3_gazebo = get_package_share_directory('burger_exploration')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    twist_mux_params = os.path.join(pkg_turtlebot3_gazebo,'config','twist_mux.yaml')
    rviz_config_file = os.path.join(pkg_turtlebot3_gazebo, 'config', 'mapping.rviz')
    
    # World file
    world = os.path.join(pkg_turtlebot3_gazebo,'worlds', 'office_cpr.world')
    
    # URDF file for robot_state_publisher
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_path = os.path.join(pkg_turtlebot3_gazebo,'urdf', urdf_file_name)
    
    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # SDF model file for spawning
    sdf_path = os.path.join(pkg_turtlebot3_gazebo,'models','turtlebot3_burger','model.sdf')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true',
                                                     description='Use simulation (Gazebo) clock if true')
    
    declare_x_position_cmd = DeclareLaunchArgument('x_pose', default_value='-2.0',
                                                   description='X position of the robot')
    
    declare_y_position_cmd = DeclareLaunchArgument('y_pose', default_value='-0.5',
                                                   description='Y position of the robot')
    
    # Start Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )
    
    # Start Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Spawn Turtlebot3 in Gazebo
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen'
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out','/burger/cmd_vel')]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    
    # Add nodes/actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(twist_mux)
    ld.add_action(rviz2)
    
    return ld