from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='5.0',
            description='Time delay in seconds for turtle2 to follow turtle1'
        ),
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        Node(
            package='ex03',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        
        Node(
            package='ex03',
            executable='turtle_tf2_broadcaster', 
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        
        Node(
            package='ex03',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'delay': LaunchConfiguration('delay')}
            ]
        ),
    ])