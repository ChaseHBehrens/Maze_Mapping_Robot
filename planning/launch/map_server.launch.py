import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():

    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='False',
    )
    
    simulate = LaunchConfiguration('simulate')

    # map file path for simple_map
    map_file_path = os.path.join(
        get_package_share_directory('planning'), 
        'maps',
        'simple_map.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz')

    # map_server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename':map_file_path}]
    )
    
    # start the node lifecyle manager
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    ) 

    # map to odom static transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', 'odom'], # supposed to use ros2 style args but i dont think it matters tbh
        output='screen'
    )

    # launch rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    path_planner = Node(
        package = 'planning',
        executable = 'path_planner.py',
        output = 'screen',
        parameters = [{'use_sim_time': True}]
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch', 'empty_world.launch.py')),
        launch_arguments = [
            ('use_sim_time', 'True')
        ],
        condition = IfCondition(simulate)
    )
    
    control = Node(
        package = 'control',
        executable = 'controller.py',
        output = 'screen',
        #parameters = [{'use_sim_time': True}]
    )
    
    controller_remap = Node(
            package='control',
            executable='controller.py',
            output='screen',
            name='controller',
            remappings=[
                ('/move_base_simple/goal', '/none'),
                ('/nav_path', '/nav_path'),
            ],
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

    return LaunchDescription([ # return all of the launch functions
        simulate_arg,
        map_server,
        lifecycle_manager, 
        static_tf,
        rviz2,
        gz_sim,
        path_planner,
        # control,
        controller_remap,
    ])