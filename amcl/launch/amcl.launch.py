import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    # Simulate argument
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='False',
    )
    simulate = LaunchConfiguration('simulate')

    # Static transform from map to odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', 'odom'], # supposed to use ros2 style args but i dont think it matters tbh
        output='screen',
        condition=IfCondition(PythonExpression(['not ', simulate]))
    )
    
    # Rviz
    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    # AMCL
    amcl_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'params', 
        'nav2_params.yaml'
    )
    amcl_unique_params = {
        'max_particles': 10000,
        'min_particles': 1000,
        'recovery_alpha_fast': 0.1,
        'transform_tolerance': 0.1,
        'update_min_a': 0.05,
        'update_min_d': 0.02,
        'z_hit': 0.9,
        'z_rand': 0.1,
        'sigma_hit': 0.1,
        'pf_err': 0.02,
        'use_sim_time': simulate
    }
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config_path,
            amcl_unique_params
        ]
    )
    
    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='localizer_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': simulate,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # Map server
    map_file_path = os.path.join(
        get_package_share_directory('exploration'),
        'mapdata', 
        'small_field.yaml'
    )
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename':map_file_path,
             'use_sim_time': simulate
             }
        ]
    )
    
    # Control nodes
    controller = Node(
        package='control',
        executable='controller.py',
        output='screen',
        name='controller',
        remappings=[
            ('/move_base_simple/goal', '/none'),
            ('/nav_path', '/nav_path'),
        ],
        parameters=[{'use_sim_time': simulate}]
    )   
    path_planner= Node(
        package='planning',
        executable='path_planner.py',
        output='screen',
        name='path_planner',
        parameters=[{'use_sim_time': simulate}]
    )
    localizer=Node(
        package='amcl',
        executable='localizer.py',
        output='screen',
        name='localizer',
        parameters=[{'use_sim_time': simulate}]
    )
    
    # Simulation vs physical
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('simulation'), 'launch', 'world_test.launch.py')),
        launch_arguments = [
            ('use_sim_time', 'True'),
            ('publish_tf', 'False')
        ],
        condition = IfCondition(simulate)
    )
    physical_robot = ExecuteProcess(
        cmd=['gnome-terminal',
             '--',
             'bash',
             '-c',
             'sshpass -p turtlebot ssh ubuntu@toad.dyn.wpi.edu', #REMEMBER TO CHANGE IN config.xml
             ],
        output = 'screen',
        condition = IfCondition(PythonExpression(['not ', simulate])),
    )

    # Reinitialize global localization
    reinitialize_global_localization = ExecuteProcess(
        cmd=['ros2',
             'service',
             'call',
             '/reinitialize_global_localization',
             'std_srvs/Empty',
             ],
    )
    
    return LaunchDescription([ # return all of the launch functions
        
        # Simulate argument
        simulate_arg,
        # Static transform from map to odom
        static_tf,
        # Rviz
        rviz2,
        # AMCL
        amcl,
        # Lifecycle manager
        lifecycle_manager,
        # Map server
        map_server,
        # Control nodes
        controller,
        path_planner,
        localizer,
        # Simulation vs physical
        gz_sim,
        physical_robot,
        # Reinitialize global localization
        reinitialize_global_localization,
    ])
