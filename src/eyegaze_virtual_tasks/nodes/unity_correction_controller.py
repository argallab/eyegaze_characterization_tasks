#!/usr/bin/env python3
"""
Code developed by Larisa YC Loke in February 2023. Copyright (c) 2023. Larisa YC Loke, Argallab.

This node controls the CORRECTION TASK i.e. FOCUS TASK (WITH visual gaze position feedback) 
    and the position of the focus circle target in unity through service calls. 
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
    FocusTarget, Alpha
)

# from eyegaze_virtual_tasks.transform2d import ScreenResolution

src_path = "/home/eyegaze_ws/src/eyegaze_virtual_tasks/study_parameters/"

class UnityFocusController(Node):
    def __init__(self, training_flag):
        super().__init__('unity_correction_controller')

        self.training_flag = training_flag

        self.__load_targets()
        self.__initialize_callback_groups()
        self.__initialize_services()
        self.__initialize_subscribers()
        self.__initialize_publishers()
        

    def startup(self):
        self.game_paused = True
        self.overlap_flag = False
        self.timeout = 20.0
        self.fixation_time = 2.0
        self.dwell_time = 0.0
        self.gaze_screen = np.zeros(2)
        self.get_logger().info("time is {}".format(self.get_clock().now().nanoseconds))
        self.focus_start_time = self.get_clock().now().nanoseconds / (1e9)
        self.target_start_time = self.get_clock().now().nanoseconds / (1e9)

        self.current_target_idx = 0
        self.new_target_flag = True

        self.alpha_mapper = interp1d([0, self.fixation_time], [1.0, 0.0], bounds_error=False, fill_value=(1.0, 0.0))
        
    def __load_targets(self):
        ## load different target sets depending on whether training or testing
        if "true" in self.training_flag.lower():
            self.targets_fname = "focus_targets_random_0.csv"
        elif "false" in self.training_flag.lower():
            self.targets_fname = "focus_targets_random_2.csv" 
        else:
            self.get_logger().info("[load targets] unexpected training flag, typo?")
            raise KeyboardInterrupt
            
        self.target_params = np.loadtxt(src_path + self.targets_fname, \
                                        delimiter=",")

        self.get_logger().info("training: {}; number of targets: {}".format(self.training_flag, len(self.target_params)))

        self.num_targets = len(self.target_params)

    def __initialize_callback_groups(self):
        self.__srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.__start_cb_group = MutuallyExclusiveCallbackGroup()
        self.__pause_cb_group = MutuallyExclusiveCallbackGroup()
        self.__alpha_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.__gaze_sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.__main_timer_cb_group = MutuallyExclusiveCallbackGroup()

    def __initialize_services(self):
        self.start_srv = self.create_service(Empty, 'start_game', self.start_callback, callback_group=self.__start_cb_group)
        self.pause_srv = self.create_service(Empty, 'pause_game', self.pause_callback, callback_group=self.__pause_cb_group)
        self.resume_srv = self.create_service(Empty, 'resume_game', self.resume_callback, callback_group=self.__pause_cb_group)

        self.unity_target_cli = self.create_client(UnityTarget, 'unity_target', callback_group=self.__srv_cb_group)

        while not self.unity_target_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('UNITY TARGET CLIENT service not available, waiting again...')

    def __initialize_subscribers(self): 
        self.gaze_screen_sub = self.create_subscription(GazeScreenStamped, 'gaze_screen', self.gaze_screen_sub_callback, 10, callback_group=self.__gaze_sub_cb_group)

    def __initialize_publishers(self):
        self.alpha_pub = self.create_publisher(Alpha, 'alpha', 1)
        self.target_dummy_pub = self.create_publisher(FocusTarget, 'focus_target', 1)
        self.event_pub = self.create_publisher(UtilityStamped, 'task_event', 1)

    def __initialize_timers(self):
        self.main_timer = self.create_timer(0.02, self.main_timer_callback, callback_group=self.__main_timer_cb_group)
        self.alpha_timer = self.create_timer(0.1, self.alpha_timer_callback, callback_group=self.__alpha_timer_cb_group)

    def gaze_screen_sub_callback(self, msg):
        self.gaze_screen[0] = msg.gaze2d.x
        self.gaze_screen[1] = msg.gaze2d.y

    def start_callback(self, request, response):
        self.__initialize_timers()
        self.game_paused = False
        self.event_msg_publisher("start")

        self.focus_start_time = self.get_clock().now().nanoseconds / (1e9)
        self.target_start_time = self.get_clock().now().nanoseconds / (1e9)
        self.get_logger().info("\n\n\n======================= START CORRECTION GAME =======================\n\n\n")
        return response

    def pause_callback(self, request, response):
        self.game_paused = True
        self.event_msg_publisher("pause")
        return response

    def resume_callback(self, request, response):
        self.game_paused = False
        self.event_msg_publisher("resume")
        return response 

    def main_timer_callback(self):
        """
        This is the main timer loop that controls the entire game

        1. Publish the current target (if flag new_target)
        2. Check gaze-target overlap and time elapsed
        3. Modify alpha accordingly and publish
        4. Change target if timeout OR dwell achieved
        """
        if not self.game_paused:
            if self.new_target_flag:
                alpha_msg = Alpha()
                alpha_msg.alpha = 1.0
                req = UnityTarget.Request()
                req.target = self.__create_focustarget_message(self.current_target_idx)
                self.event_msg_publisher("next")

                self.alpha_pub.publish(alpha_msg)
                targetsrv_future = self.unity_target_cli.call_async(req)
                while not targetsrv_future.done():
                    self.get_logger().info("...")
                    # self.get_logger().info("waiting for unity target service call to complete...")
                self.target_dummy_pub.publish(req.target)   ## publish once on the dummy topic 
                self.get_logger().info("target {}: {}".format(self.current_target_idx, req.target))
                
                self.target_start_time = self.get_clock().now().nanoseconds / (1e9)
                self.current_target = self.target_params[self.current_target_idx, :]
                self.target_radius = targetsrv_future.result().radius
                self.new_target_flag = False        

            self.__check_gaze_target_overlap()
        
        else:
            return

    def __check_gaze_target_overlap(self):
        ## compute the distance
        gaze_dist_from_target = np.linalg.norm(self.gaze_screen - self.current_target[:-1])

        ## if overlap
        if gaze_dist_from_target <= self.target_radius:
            if not self.overlap_flag:   ## newly overlapped
                self.overlap_flag = True
                # set the start time 
                self.focus_start_time = self.get_clock().now().nanoseconds / (1e9)  ## save this in seconds
            self.dwell_time = self.get_clock().now().nanoseconds / (1e9) - self.focus_start_time
        else:   ## if no overlap
            if self.overlap_flag:
                self.overlap_flag = False   ## unset the overlap flag
                self.dwell_time = 0.0
        
        self.alpha = self.alpha_mapper(self.dwell_time)

        ## bump to next target if this condition has been achieved
        if (self.dwell_time >= self.fixation_time) or \
        ((self.get_clock().now().nanoseconds / (1e9) - self.target_start_time) > self.timeout):
            # self.get_logger().info("\n\ntimed out!!! OR target reached!!!\n\n")
            self.alpha = 1.0
            self.current_target_idx += 1
            self.new_target_flag = True  
            self.overlap_flag = False
            self.dwell_time = 0.0
            self.focus_start_time = self.get_clock().now().nanoseconds / (1e9)
            if self.current_target_idx >= self.num_targets:
                self.event_msg_publisher("end")
                self.get_logger().info("\n\n\n======================= END CORRECTION GAME =======================\n\n\n")
                self.game_paused = True

    def event_msg_publisher(self, msg):
        '''
        msg is a python string of the message
        '''
        msgpub = UtilityStamped()
        msgpub.header.stamp = self.get_clock().now().to_msg()
        msgpub.body.data = str(msg)
        self.event_pub.publish(msgpub)

    def alpha_timer_callback(self):
        alpha_msg = Alpha()
        alpha_msg.alpha = float(self.alpha)
        self.alpha_pub.publish(alpha_msg)

    def __create_focustarget_message(self, idx):
        msg = FocusTarget()
        target_param = self.target_params[idx, :]
        msg.position.x = target_param[0]
        msg.position.y = target_param[1]
        msg.size = target_param[2]  
        return msg


def main():
    rclpy.init()
    controller = UnityFocusController(sys.argv[1])

    executor = MultiThreadedExecutor()

    executor.add_node(controller)
    controller.startup()

    while rclpy.ok(): 
        executor.spin_once()


    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()