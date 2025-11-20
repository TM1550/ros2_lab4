import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'radius',
            default_value='2.0',
            description='Radius of rotation for carrot around turtle1'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation',
            default_value='1',
            description='Direction of rotation: 1 for clockwise, -1 for counterclockwise'
        ),
        
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # Turtle1 TF2 broadcaster
        Node(
            package='ex02',
            executable='turtle_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            parameters=[{'turtlename': 'turtle1'}]
        ),
        
        # Turtle2 TF2 broadcaster  
        Node(
            package='ex02',
            executable='turtle_tf2_broadcaster',
            name='turtle2_tf2_broadcaster',
            parameters=[{'turtlename': 'turtle2'}]
        ),
        
        # Carrot frame broadcaster
        Node(
            package='ex02',
            executable='carrot_broadcaster',
            name='carrot_broadcaster',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation')
            }]
        ),
        
        # Turtle2 listener (follows carrot)
        Node(
            package='ex02',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[{'target_frame': 'carrot1'}]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('ex02'), 'resource', 'carrot.rviz')]
        )
    ])