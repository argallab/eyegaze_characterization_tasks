#!/usr/bin/env python3
"""
This node provides convenience key presses for task services 
"""
import numpy as np
from scipy.interpolate import interp1d
from enum import Enum
import sys

## ros2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Float32
from std_srvs.srv import Empty

## import custom ROS2 types
from ros2_pyg3_common.srv import UnityTarget, WebsocketRequest
from ros2_pyg3_common.msg import (
    UtilityStamped, 
    ImuStamped, 
    ScreenCornersStamped,
    GazeScreenStamped, GazeStamped, 
    CalibrationMarkerStamped,
    FocusTarget, Alpha,
    TimeInt32
)

from eyegaze_virtual_tasks.transform2d import ScreenResolution


src_path = "/home/eyegaze_ws/src/eyegaze_virtual_tasks/study_parameters/"


class KeyInputTaskSupport(Node):
    def __init__(self, task):
        super().__init__('keyinput_task_support')

        self.task_type = task
        self.__initialize_services()


    def __initialize_services(self):
        self.start_cli = self.create_client(Empty, 'start_game')
        while not self.start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('START service not available, waiting again...')
        
        if 'trace' in self.task_type.lower():
            self.next_cli = self.create_client(Empty, 'next_trace')
            while not self.next_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('NEXT service not available, waiting again...')

            self.get_logger().info("[SANITY CHECK] the current task is TRACE task")
        
        elif 'focus' in self.task_type.lower() or 'correction' in self.task_type.lower():
            self.pause_cli = self.create_client(Empty, 'pause_game')
            while not self.pause_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('PAUSE service not available, waiting again...')

            self.resume_cli = self.create_client(Empty, 'resume_game')
            while not self.resume_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('RESUME service not available, waiting again...')
            
            self.get_logger().info("[SANITY CHECK] the current task is FOCUS / CORRECTION task")
        
        else:
            self.get_logger().info("[SANITY CHECK] the current task is PAINTING task")


    def keyinput(self):
        self.get_logger().info("in keyinput")
        keypress = input("Enter a task key: ")
        req = Empty.Request()
        if 's' in keypress.lower():
            # self.get_logger().info("pressed s")
            future = self.start_cli.call_async(req)
        elif 'n' in keypress.lower():
            future = self.next_cli.call_async(req)
        elif 'p' in keypress.lower():
            future = self.pause_cli.call_async(req)
        elif 'r' in keypress.lower():
            future = self.resume_cli.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("here")


def main():
    rclpy.init()
    support_node = KeyInputTaskSupport(sys.argv[1])

    while rclpy.ok():
        support_node.get_logger().info("in while")
        support_node.keyinput()


    support_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()