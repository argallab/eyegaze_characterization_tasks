#!/usr/bin/env python3
"""
Code developed by Larisa YC Loke in February 2023. Copyright (c) 2023. Larisa YC Loke, Argallab.

This node controls the PAINTING TASK and computes the countdown timer.
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

src_path = "/home/eyegaze_ws/src/eyegaze_virtual_tasks/study_parameters/"


class UnityPaintController(Node):
    def __init__(self, training_flag):
        super().__init__('unity_paint_controller')

        self.training_flag = training_flag

        if "true" in self.training_flag.lower():
            self.total_time = 45.0          # training is 45 seconds
        elif "false" in self.training_flag.lower():
            self.total_time = 5.0 * 60.0    # actual task is 5 min
        else:
            self.get_logger().info("[set time] unexpected training flag, typo?")
            raise KeyboardInterrupt

        self.__initialize_services()
        self.__initialize_publishers()

        self.get_logger().info("task time: {}".format(self.total_time))

    def __initialize_services(self):
        self.start_srv = self.create_service(Empty, 'start_game', self.start_callback)

    def __initialize_publishers(self):
        self.event_pub = self.create_publisher(UtilityStamped, 'task_event', 1)
        self.countdown_pub = self.create_publisher(TimeInt32, '/countdown_timer', 1)

    def __initialize_timers(self, duration):
        self.countdown_timer = self.create_timer(duration, self.countdown_timer_callback)

    def start_callback(self, request, response):
        self.timer_step = 2.0
        self.remaining_time = self.total_time    # 5 minutes for painting task
        self.__initialize_timers(self.timer_step)

        msgpub = UtilityStamped()
        msgpub.header.stamp = self.get_clock().now().to_msg()
        msgpub.body.data = "start"
        self.event_pub.publish(msgpub)

        self.get_logger().info("\n\n\n======================= START PAINT GAME =======================\n\n\n")
        return response

    def countdown_timer_callback(self):
        self.remaining_time = self.remaining_time - self.timer_step
        # self.get_logger().info("time left is {}".format(self.remaining_time))
        if self.remaining_time >= 0.0:
            time_pub = TimeInt32()
            time_pub.time = int(self.remaining_time)
            # self.get_logger().info("publishing time now")
            self.countdown_pub.publish(time_pub)
        else: 
            msgpub = UtilityStamped()
            msgpub.header.stamp = self.get_clock().now().to_msg()
            msgpub.body.data = "end"
            self.event_pub.publish(msgpub)

            self.get_logger().info("\n\n\n======================= FINISH PAINT GAME =======================\n\n\n")
            raise KeyboardInterrupt


def main():
    rclpy.init()
    print("argument in main: {}; of type: {}".format(sys.argv[1], type(sys.argv[1])))
    controller = UnityPaintController(sys.argv[1])

    executor = MultiThreadedExecutor()

    executor.add_node(controller)

    try:
        controller.get_logger().info('Beginning painting task, shut down with CTRL-C')
        executor.spin()

    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt, shutting down \n')

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()