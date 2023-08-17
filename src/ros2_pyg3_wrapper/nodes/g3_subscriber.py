#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

## ros2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

## import custom ROS2 types
from ros2_pyg3_common.srv import WebsocketRequest
from ros2_pyg3_common.msg import UtilityStamped, ImuStamped, GazeStamped, CalibrationMarkerStamped

## importing PyGlasses3 python modules
# sys.path.insert(0, '/home/eyetrack_ws/src/ros2_PyGlasses3/')
from ros2_pyg3_wrapper.utilities import MsgID


class GlassesController(Node):
    def __init__(self):
        super().__init__('glasses_controller')

        self.__srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.__keepalive_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.__cmarker_timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.__initialize_services()
        self.__initialize_timers()
        self.__initialize_subscribers()


    def startup(self):
        startup_msgs = [MsgID.GET_RECORDING_UNIT_INFO.value,
                        MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value, 
                        MsgID.SIGNAL_RUDIMENTARY_GAZE.value]
                        # MsgID.SIGNAL_RUDIMENTARY_IMU.value]

        # startup_msgs = [MsgID.ACTION_EMIT_MARKERS.value, 
        #                 MsgID.SIGNAL_CALIBRATION_MARKERS.value]
        
        for i in range(len(startup_msgs)):
            req = WebsocketRequest.Request()
            req.msgid = startup_msgs[i]
            future = self.ws_req_cli.call_async(req)
            # while not future.done():
            #     self.get_logger().info("waiting for startup service call to complete...")
            # self.get_logger().info("[NOTE]: Service request {} completed".format(startup_msgs[i]))


    def __initialize_services(self):
        self.ws_req_cli = self.create_client(WebsocketRequest, 'websocket_request', callback_group=self.__srv_cb_group)
        while not self.ws_req_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WebsocketRequest.Request()

    def __initialize_subscribers(self):
        self.gaze_sub = self.create_subscription(GazeStamped, 'gaze_stream', self.gaze_sub_callback, 10)
        self.imu_sub = self.create_subscription(ImuStamped, 'imu_stream', self.imu_sub_callback, 10)
        self.calibmarker_sub = self.create_subscription(CalibrationMarkerStamped, 'calibration_marker_stream', self.calibmarker_sub_callback, 10)
        self.glasses_utility_sub = self.create_subscription(UtilityStamped, 'glasses_utility', self.utility_sub_callback, 10)

    def __initialize_timers(self):
        self.keepalive_timer = self.create_timer(5, self.keepalive_timer_callback, callback_group=self.__keepalive_timer_cb_group)
        # self.cmarker_timer = self.create_timer(5, self.cmarker_timer_callback, callback_group=self.__cmarker_timer_cb_group)
        return

    def gaze_sub_callback(self, gaze):
        # self.get_logger().info("GAZE 2D RECEIVED: [{}, {}]".format(gaze.gaze2d.x, gaze.gaze2d.y))
        ## pickle this information? rosbag? save to a csv file? 
        return 

    def imu_sub_callback(self, imu):
        return
        # self.get_logger().info("IMU RECEIVED, ACCEL: [{}, {}, {}]".format(imu.imu.accel.linear.x, imu.imu.accel.linear.y, imu.imu.accel.linear.z))

    def calibmarker_sub_callback(self, calibmarker):
        # self.get_logger().info("calibration marker position: 3d: [{}, {}, {}]; 2d: [{}, {}]".format( \
        #     calibmarker.marker_3d_pos.x, calibmarker.marker_3d_pos.y, calibmarker.marker_3d_pos.z, calibmarker.marker_2d_pos.x, calibmarker.marker_2d_pos.y))
        return

    def utility_sub_callback(self, msg):
        self.get_logger().info("UTILITY MSG RECEIVED, TYPE: {}, MSG: {}".format(msg.type.data, msg.body.data))
        return

    def keepalive_timer_callback(self):
        ka_req = WebsocketRequest.Request()
        ka_req.msgid = MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value
        future = self.ws_req_cli.call_async(ka_req)
        while not future.done():
            self.get_logger().info("waiting for keep alive call to complete...")
        self.get_logger().info("[NOTE]: Service request completed")

    def cmarker_timer_callback(self):
        em_req = WebsocketRequest.Request()
        em_req.msgid = MsgID.ACTION_EMIT_MARKERS.value
        future = self.ws_req_cli.call_async(em_req)
        self.get_logger().info("EMIT marker requested")

        # cm_req = WebsocketRequest.Request()
        # cm_req.msgid = MsgID.SIGNAL_CALIBRATION_MARKERS.value
        # future = self.ws_req_cli.call_async(cm_req)
        # self.get_logger().info("marker SIGNAL requested")
        # while not future.done():
        #     self.get_logger().info("waiting for EM service call to complete...")
        # self.get_logger().info("[NOTE]: EM Service request completed")


def main():
    rclpy.init()
    g3controller = GlassesController()

    executor = MultiThreadedExecutor()

    executor.add_node(g3controller)
    g3controller.startup()
    # executor.spin()

    while rclpy.ok():
        executor.spin_once()


    g3controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()