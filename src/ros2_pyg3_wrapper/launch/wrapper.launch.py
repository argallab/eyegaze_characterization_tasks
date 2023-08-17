import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node



def generate_launch_description():
    g3_addr_launch_arg = DeclareLaunchArgument(
        'glasses_address', default_value=TextSubstitution(text='192.168.8.109')
    )

    return LaunchDescription([
        g3_addr_launch_arg,
        Node(
            package='ros2_pyg3_wrapper',
            executable='g3_wrapper.py',
            name='g3_wrapper',
            output='screen',
            parameters=[{
                'glasses_address': LaunchConfiguration('glasses_address')
            }]
        ),
    ])

