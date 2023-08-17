import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ## ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
    unity_bridge_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        # name='ros2_unity_bridge',
        output='screen',
        parameters=[{
            'ROS_IP': '0.0.0.0'
        }]
    )

    screen_detect_node = Node(
        package='ros2_pyg3_wrapper',
        executable='scene_camera_reader_node',
        name='scene_camera_reader',
        # output='screen'
    )

    wrapper_node = Node(
        package='ros2_pyg3_wrapper',
        executable='g3_wrapper.py',
        name='wrapper',
        # output='screen'
    )

    ld.add_action(unity_bridge_node)
    ld.add_action(screen_detect_node)
    ld.add_action(wrapper_node)

    return ld

