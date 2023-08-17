import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ############################################
    ##### Declare launch arguments #############
    ############################################
    sid = LaunchConfiguration('sid')
    train = LaunchConfiguration('train')
    task = LaunchConfiguration('task')    

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
        description='name of task to run',
        default_value='paint'
    )

    ############################################
    ##### Launch gaze controller node ##########
    ############################################
    controller_node = Node(
        package='eyegaze_virtual_tasks',
        executable='unity_gaze_controller.py',
        name='controller',
        output='screen'
    )

    ############################################
    ##### Launch task-specific node ############
    ############################################
    paint_node = Node(
        package='eyegaze_virtual_tasks',
        executable='unity_paint_controller.py',
        name='paint_controller',
        output='screen',
        emulate_tty=True,
        arguments=[
            train
        ],
        condition=IfCondition(PythonExpression(["'", task, "' == 'paint'"]))
    )
    focus_node = Node(
        package='eyegaze_virtual_tasks',
        executable='unity_focus_controller.py',
        name='focus_controller',
        output='screen',
        emulate_tty=True,
        arguments=[
            train
        ],
        condition=IfCondition(PythonExpression(["'", task, "' == 'focus'"]))
    )
    correction_node = Node(
        package='eyegaze_virtual_tasks',
        executable='unity_correction_controller.py',
        output='screen',
        emulate_tty=True,
        arguments=[
            train
        ],
        condition=IfCondition(PythonExpression(["'", task, "' == 'correction'"]))
    )
    trace_node = Node(
        package='eyegaze_virtual_tasks',
        executable='unity_trace_controller.py',
        name='trace_controller',
        output='screen',
        emulate_tty=True,
        arguments=[
            train
        ],
        condition=IfCondition(PythonExpression(["'", task, "' == 'trace'"]))
    )

    ############################################
    ##### Launch bag recording #################
    ############################################
    launch_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ThisLaunchFileDir(), '/record.launch.py'
        ]),
        launch_arguments={
            'sid': sid, 
            'train': train,
            'task': task
        }.items()
    )

    ################################################
    ##### Launch actions to launch description #####
    ################################################
    ld.add_action(sid_launch_arg)
    ld.add_action(train_launch_arg)
    ld.add_action(task_launch_arg)

    ld.add_action(controller_node)
    ld.add_action(paint_node)
    ld.add_action(focus_node)
    ld.add_action(correction_node)
    ld.add_action(trace_node)
    ld.add_action(launch_record)

    return ld

