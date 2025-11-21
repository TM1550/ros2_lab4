import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'switch_threshold',
            default_value='1.0',
            description='Threshold for automatic target switching'
        ),
        
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # Spawn turtle3
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
                 '{"x": 8.0, "y": 8.0, "theta": 0.0, "name": "turtle3"}'],
            output='screen'
        ),
        
        # TF2 broadcasters for all turtles
        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            parameters=[{'turtlename': 'turtle1'}]
        ),
        
        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='turtle2_tf2_broadcaster', 
            parameters=[{'turtlename': 'turtle2'}]
        ),
        
        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='turtle3_tf2_broadcaster',
            parameters=[{'turtlename': 'turtle3'}]
        ),
        
        # Target switcher node
        Node(
            package='turtle_multi_target',
            executable='target_switcher',
            name='target_switcher'
        ),
        
        # Turtle controller node
        Node(
            package='turtle_multi_target',
            executable='turtle_controller',
            name='turtle_controller',
            parameters=[{'switch_threshold': LaunchConfiguration('switch_threshold')}]
        ),
        
        # Keyboard switch node
        Node(
            package='turtle_multi_target',
            executable='keyboard_switch',
            name='keyboard_switch',
            prefix='xterm -e',
            output='screen'
        ),
        
        # Teleop node for turtle1
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('turtle_multi_target'), 'resource', 'multi_target.rviz')]
        )
    ])