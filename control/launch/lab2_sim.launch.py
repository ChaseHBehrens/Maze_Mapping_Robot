import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from ..src.controller import Controller

def generate_launch_description():

    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='False',
    )
    
    simulate = LaunchConfiguration('simulate')

    rviz_config_dir = os.path.join(
        get_package_share_directory('control'),
        'rviz',
        'control_pkg.rviz'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch', 'empty_world.launch.py')),
        launch_arguments = [
            ('use_sim_time', 'True')
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

    rviz2 =  Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        arguments = ['-d', rviz_config_dir],
        output = 'screen'
    )

    static_transform = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments=["0.5", "0.2", "0", "0.78", "0", "0", "odom", "map"] 
    )

    controller = Node(
        package = 'control',
        executable = 'controller.py',
        output = 'screen',
        #parameters = [{'use_sim_time': True}]
    )

    path_generator = Node(
        package = 'control',
        executable = 'path_generator.py',
        output = 'screen',
    )

    return LaunchDescription([
        simulate_arg,
        gz_sim,
        rviz2,
        static_transform,
        controller,
        path_generator,
        physical_robot,
    ])
