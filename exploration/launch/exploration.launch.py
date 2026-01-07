import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='False',
    )
    
    simulate = LaunchConfiguration('simulate')

    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz'
    )
    
    # slam toolbox config
    slam_config_file = os.path.join(
        get_package_share_directory('exploration'), 
        'launch', 
        'exploration_params.yaml'
    )
    
    # slam toolbox node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # Load parameters from the YAML file
        parameters=[
            slam_config_file,
            {
            'use_sim_time': False,
            }
        ],
    )
    
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("laser_filters"),
                "examples", "bin_filter_example.yaml",
            ])],        
    )

    # Automatically configure the lifecycle node after startup
    configure_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )
    
    # Automatically activate the lifecycle node
    activate_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )
    
    
    # launch rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both':'log'},#'screen',
        arguments=['-d', rviz_config_path]
    )
    
    path_planner = Node(
        package = 'planning',
        executable = 'path_planner.py',
        output={'both':'log'},#'screen',
        parameters = [{'use_sim_time': False}]
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('simulation'), 'launch', 'world_test.launch.py')),
        launch_arguments = [
            ('use_sim_time', 'False')
        ],
        condition = IfCondition(simulate)
    )
    
    controller_remap = Node(
        package='control',
        executable='controller.py',
        output={'both':'log'},#'screen',
        name='controller',
        remappings=[
            ('/move_base_simple/goal', '/none'),
            ('/nav_path', '/nav_path'),
        ],
        parameters = [{'use_sim_time': False}]
    )   

    mapper = Node(
        package = 'exploration',
        executable = 'mapper.py',
        output = 'screen',
        name = 'mapper',
        parameters = [{'use_sim_time': False}]
    )

    return LaunchDescription([ # return all of the launch functions
        simulate_arg,
        rviz2,
        gz_sim,
        mapper,
        path_planner,
        controller_remap,
        slam_toolbox, 
        laser_filter,
        TimerAction(period=2.0, actions=[configure_slam]),
        TimerAction(period=3.0, actions=[activate_slam]),    
    ])
