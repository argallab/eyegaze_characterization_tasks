import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node

import time


def generate_launch_description():
    ld = LaunchDescription()

    sid = LaunchConfiguration('sid')
    train = LaunchConfiguration('train')
    task = LaunchConfiguration('task')
    rosbag_name_abs = LaunchConfiguration('rosbag_name_abs')
    timestr = time.strftime("%Y%m%d-%H%M%S")

    sid_launch_arg = DeclareLaunchArgument(
        'sid',
        description='subject id, used to tag rosbag',
        default_value='00'
    )
    train_launch_arg = DeclareLaunchArgument(
        'train',
        description='flag for training',
        default_value='false'
    )

    task_launch_arg = DeclareLaunchArgument(
        'task',
        description='string of task',
        default_value='test'
    )

    rosbag_name_abs_arg = DeclareLaunchArgument(
        'rosbag_name_abs', 
        description='dummy argument, do not use at launch',
        default_value=[
            TextSubstitution(text='/home/eyegaze_ws/src/casmi_data_analysis/data/bag_files/s'), 
            sid, 
            TextSubstitution(text='/s'),
            sid,
            TextSubstitution(text='_'),
            task,
            TextSubstitution(text='_task_train_'),
            train, 
            TextSubstitution(text=('_' + timestr))]
    )   
    print("rosbag name: {}".format(rosbag_name_abs_arg))

    rosbag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '/gaze_screen',
             '/gaze_stream',
             '/glasses_utility', 
             '/imu_stream',
             '/screen_corners',
             '/task_event',
             '/countdown_timer',
             '/trace_target',
             '/alpha', 
             '/focus_target', 
             '-o', rosbag_name_abs],
        output='screen'
    )

    ld.add_action(sid_launch_arg)
    ld.add_action(rosbag_name_abs_arg)
    ld.add_action(train_launch_arg)
    ld.add_action(task_launch_arg)
    ld.add_action(rosbag_recorder)

    return ld

