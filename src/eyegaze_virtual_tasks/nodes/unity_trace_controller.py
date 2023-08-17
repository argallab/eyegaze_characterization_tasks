#!/usr/bin/env python3
"""
Code developed by Larisa YC Loke in February 2023. Copyright (c) 2023. Larisa YC Loke, Argallab.

This node controls the TRACKING TASK and position of the tracking circle target in unity and computes 
    the alpha value to be published to the alpha topic based on gaze distance from the target.
"""
import numpy as np
from scipy.interpolate import interp1d
from enum import Enum
import json
from collections import OrderedDict
import sys

## ros2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Float32
from std_srvs.srv import Empty

## import custom ROS2 types
from ros2_pyg3_common.srv import UnityTrace
from ros2_pyg3_common.msg import (
    UtilityStamped, 
    ImuStamped, 
    ScreenCornersStamped,
    GazeScreenStamped, GazeStamped, 
    CalibrationMarkerStamped,
    TraceTarget, 
    Alpha
)

# from eyegaze_virtual_tasks.transform2d import ScreenResolution


src_path = "/home/eyegaze_ws/src/eyegaze_virtual_tasks/study_parameters/"


class UnityTraceController(Node):
    def __init__(self, training_flag):
        super().__init__('unity_trace_controller')

        self.training_flag = training_flag
        self.alpha_mapper = interp1d([100, 500], [1.0, 0.1], bounds_error=False, fill_value=(1.0, 0.1))

        self.__load_traces()
        self.__initialize_callback_groups()
        self.__initialize_services()
        self.__initialize_subscribers()
        self.__initialize_publishers()

    def startup(self):
        self.timeout = 20.0
        self.gaze_screen = np.zeros(2)
        self.get_logger().info("time is {}".format(self.get_clock().now().nanoseconds))

        self.current_trace_idx = 0
        self.alpha = 0.2
        self.current_trace = self.trace_dict[str(self.current_trace_idx)]
        self.current_point = np.array([self.current_trace[0, 0], self.current_trace[0, 1]])
        self.publish_trace_flag = False

    def __load_traces(self):
        if "true" in self.training_flag.lower():
            self.trace_fname = "trace_paths_train.json"
        elif "false" in self.training_flag.lower():
            self.trace_fname = "trace_paths.json"
        else: 
            self.get_logger().info("[load traces] unexpected training flag, typo?")
            raise KeyboardInterrupt

        trace_dict_json = {}
        with open(src_path + self.trace_fname, "r") as infile:
            trace_dict_json = json.load(infile)

        self.trace_dict = OrderedDict()

        _idx = 0
        for key in trace_dict_json.keys():
            x = np.array(trace_dict_json[key]['x']).reshape((len(trace_dict_json[key]['x']), 1))
            y = np.array(trace_dict_json[key]['y']).reshape((len(trace_dict_json[key]['y']), 1))
            self.trace_dict[str(_idx)] = np.concatenate((x, y), axis=1)
            self.get_logger().info("trace {} has shape {}".format(_idx, self.trace_dict[str(_idx)].shape))
            _idx += 1

        self.num_traces = len(trace_dict_json.keys())
        self.get_logger().info("training: {}; number of traces: {}".format(self.training_flag, self.num_traces))

        return 

    # def __initialize_parameters(self):
    #     self.declare_parameter('training_flag', 'false')
    #     self.training_flag = bool(self.get_parameter('training_flag'))
    #     if self.training_flag:
    #         self.trace_fname = "trace_paths_train.json"
    #     else:
    #         self.trace_fname = "trace_paths.json"   

    def __initialize_callback_groups(self):
        self.__srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.__start_cb_group = MutuallyExclusiveCallbackGroup()
        self.__next_cb_group = MutuallyExclusiveCallbackGroup()
        self.__pause_cb_group = MutuallyExclusiveCallbackGroup()
        self.__trace_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.__gaze_sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.__main_timer_cb_group = MutuallyExclusiveCallbackGroup()

    def __initialize_services(self):
        self.start_srv = self.create_service(Empty, 'start_game', self.start_callback, callback_group=self.__start_cb_group)
        self.next_srv = self.create_service(Empty, 'next_trace', self.next_callback, callback_group=self.__next_cb_group)
        self.pause_srv = self.create_service(Empty, 'pause_game', self.pause_callback, callback_group=self.__pause_cb_group)

    def __initialize_subscribers(self): 
        self.gaze_screen_sub = self.create_subscription(GazeScreenStamped, 'gaze_screen', self.gaze_screen_sub_callback, 10, callback_group=self.__gaze_sub_cb_group)

    def __initialize_publishers(self):
        self.alpha_pub = self.create_publisher(Alpha, 'alpha', 1)
        self.trace_pub = self.create_publisher(TraceTarget, 'trace_target', 1)
        self.event_pub = self.create_publisher(UtilityStamped, 'task_event', 1)

    def __initialize_timers(self):
        self.trace_pub_timer = self.create_timer(0.02, self.trace_pub_timer_callback, callback_group=self.__trace_timer_cb_group)

    def gaze_screen_sub_callback(self, msg):
        self.gaze_screen[0] = msg.gaze2d.x
        self.gaze_screen[1] = msg.gaze2d.y

    def start_callback(self, request, response):
        self.__initialize_timers()

        msgpub = UtilityStamped()
        msgpub.header.stamp = self.get_clock().now().to_msg()
        msgpub.body.data = "start"
        self.event_pub.publish(msgpub)

        self.target_start_time = self.get_clock().now().nanoseconds / (1e9)

        self.publish_trace_flag = True
        self.get_logger().info("\n\n\n======================= START TRACE GAME =======================\n\n\n")
        return response

    def next_callback(self, request, response):
        self.current_trace_idx += 1
        if self.current_trace_idx < self.num_traces:
            self.current_trace = self.trace_dict[str(self.current_trace_idx)]
            self.publish_trace_flag = True

            msgpub = UtilityStamped()
            msgpub.header.stamp = self.get_clock().now().to_msg()
            msgpub.body.data = "next"
            self.event_pub.publish(msgpub)
        else:
            msgpub = UtilityStamped()
            msgpub.header.stamp = self.get_clock().now().to_msg()
            msgpub.body.data = "end"
            self.event_pub.publish(msgpub)
            self.get_logger().info("\n\n\n======================= END TRACE GAME =======================\n\n\n")

        return response

    def pause_callback(self, request, response):
        return response

    def trace_pub_timer_callback(self):
        if len(self.current_trace) > 0:
            trace_to_pub = self.__create_tracetarget_msg()
            alpha_to_pub = self.__check_gaze_trace_distance()
            if self.publish_trace_flag:
                self.trace_pub.publish(trace_to_pub)
                self.alpha_pub.publish(alpha_to_pub)
        else: 
            self.get_logger().info("trace finished, waiting for next trace...")
            self.publish_trace_flag = False
        return

    def __check_gaze_trace_distance(self):
        gaze_dist_from_trace = np.linalg.norm(self.gaze_screen - self.current_point)

        self.alpha = self.alpha_mapper(gaze_dist_from_trace)

        alpha_msg = Alpha()

        alpha_msg.alpha = float(self.alpha)
        return alpha_msg

    def __create_tracetarget_msg(self):
        output = TraceTarget()
        output.point.x = self.current_trace[0, 0]
        output.point.y = self.current_trace[0, 1]
        self.current_trace = np.delete(self.current_trace, 0, 0)
        self.current_point = np.array([self.current_trace[0, 0], self.current_trace[0, 1]])
        return output


def main():
    rclpy.init()
    controller = UnityTraceController(sys.argv[1])

    executor = MultiThreadedExecutor()

    executor.add_node(controller)
    controller.startup()

    while rclpy.ok(): 
        executor.spin_once()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()