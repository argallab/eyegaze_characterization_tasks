#!/usr/bin/env python3
"""
ROS2-Python wrapper for Tobii Pro Glasses 3 API
Contains PyGlasses3 class which wraps the API into convenient 
function calls 

Some useful websockets references:
https://github.com/ricardolsmendes/websockets-asyncio/blob/main/document_inspector.py
https://stackoverflow.com/questions/67734115/how-to-use-multithreading-with-websockets

Some useful ROS2 references:
https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255
https://github.com/mavlink/MAVSDK-Python/issues/419#issuecomment-1008905339

"""
import websocket
import asyncio
import json
import cv2
import logging
import threading
import sys
import base64

## ros2 libs
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
from ros2_pyg3_wrapper.sender import Sender
from ros2_pyg3_wrapper.receiver import Receiver


class PyGlasses3Node(Node):
    def __init__(self):
        super().__init__('pyglasses3_node')

        self.__glasses_addr = "192.168.8.110"

        self.__ws_url = "ws://{}/websocket".format(self.__glasses_addr)
        self.__rtsp_url = "rtsp://{}:8554/live/all".format(self.__glasses_addr)
        self.get_logger().info("glasses url is: {}".format(self.__ws_url))

        self.__ws_srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.__gaze_pub_cb_group = MutuallyExclusiveCallbackGroup()
        self.__imu_pub_cb_group = MutuallyExclusiveCallbackGroup()
        self.__calibmarker_pub_cb_group = MutuallyExclusiveCallbackGroup()
        self.__utility_pub_cb_group = MutuallyExclusiveCallbackGroup()
        self.__ws_cb_group = MutuallyExclusiveCallbackGroup()
        self.__stream_srv_cb_group = MutuallyExclusiveCallbackGroup()

        self.sender = Sender()
        self.receiver = Receiver(self.__rtsp_url, self.get_logger())

        self.__parsed_msg = None

        self.__initialize_publishers()
        self.__initialize_timers()
        self.websocket_request_srv = self.create_service(WebsocketRequest, 'websocket_request', \
            self.websocket_request_callback, callback_group=self.__ws_srv_cb_group)

        self.get_logger().info("websocket connection created!")
        self.ws_conn = websocket.create_connection(self.__ws_url, subprotocols=["g3api"])

    def __initialize_publishers(self):
        self.gaze_pub = self.create_publisher(GazeStamped, 'gaze_stream', 1)
        self.imu_pub = self.create_publisher(ImuStamped, 'imu_stream', 1)
        self.calibmarker_pub = self.create_publisher(CalibrationMarkerStamped, 'calibration_marker_stream', 1)
        self.glasses_utility_pub = self.create_publisher(UtilityStamped, 'glasses_utility', 1)

    def __initialize_timers(self):
        self.gaze_pub_timer = self.create_timer(0.005, self.gaze_pub_timer_callback, callback_group=self.__gaze_pub_cb_group)
        self.imu_pub_timer = self.create_timer(0.01, self.imu_pub_timer_callback, callback_group=self.__imu_pub_cb_group)
        # self.calibmarker_pub_timer = self.create_timer(0.01, self.calibmarker_pub_timer_callback, callback_group=self.__calibmarker_pub_cb_group)
        self.utility_pub_timer = self.create_timer(0.01, self.utility_pub_timer_callback, callback_group=self.__utility_pub_cb_group)
        self.websocket_timer = self.create_timer(0.001, self.ws_timer_callback, callback_group=self.__ws_cb_group) # 100Hz websocket 


    def websocket_request_callback(self, request, response):
        msgid = request.msgid
        self.sender.write_send_message(msgid)
        self.ws_conn.send(json.dumps(self.sender.msgs[0]))
        self.sender.msgs.pop(0)
        response.success = True
        return response

    def gaze_pub_timer_callback(self):
        # self.get_logger().info("{}, {}".format(str(self.get_clock().now().to_msg().sec), str(self.get_clock().now().to_msg().nanosec)))
        if len(self.receiver.gaze_stream):
            gaze_to_pub = self.receiver.gaze_stream[0]
            gaze_to_pub.header.stamp = self.get_clock().now().to_msg()
            self.gaze_pub.publish(gaze_to_pub)
            # self.get_logger().info("2d gaze: [{}, {}]".format(gaze_to_pub.gaze2d.x, gaze_to_pub.gaze2d.y))
            self.receiver.gaze_stream.pop(0)

    def imu_pub_timer_callback(self):
        if len(self.receiver.imu_stream):
            imu_to_pub = self.receiver.imu_stream[0]
            imu_to_pub.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub.publish(imu_to_pub)
            self.receiver.imu_stream.pop(0)

    def calibmarker_pub_timer_callback(self):
        if len(self.receiver.calibration_marker_stream):
            cm_to_pub = self.receiver.calibration_marker_stream[0]
            cm_to_pub.header.stamp = self.get_clock().now().to_msg()
            self.calibmarker_pub.publish(cm_to_pub)
            self.receiver.calibration_marker_stream.pop(0)

    def utility_pub_timer_callback(self):
        if len(self.receiver.misc_stream):
            utility_to_pub = self.receiver.misc_stream[0]
            utility_to_pub.header.stamp = self.get_clock().now().to_msg()
            self.glasses_utility_pub.publish(utility_to_pub)
            # self.get_logger().info("UTILITY type:{}; data:{}".format(utility_to_pub.type.data, utility_to_pub.body.data))
            self.receiver.misc_stream.pop(0)

    def ws_timer_callback(self):
        # self.get_logger().info("reading from websocket on timer")
        recv_msg = self.ws_conn.recv()
        # self.get_logger().info("received: {}".format(json.loads(recv_msg)))
        self.receiver.msgs.append(json.loads(recv_msg))
        try:
            self.__parsed_msg = self.receiver.parse_messages()
        except:
            pass




def main():
    rclpy.init()

    wrapper = PyGlasses3Node()
    
    executor = MultiThreadedExecutor()
    executor.add_node(wrapper)
    
    try:
        wrapper.get_logger().info('Beginning wrapper, shut down with CTRL-C')
        executor.spin()

    except KeyboardInterrupt:
        wrapper.get_logger().info('Keyboard interrupt, shutting down \n')

    wrapper.destroy_node()
    rclpy.shutdown()
    wrapper.ws_conn.close()

if __name__ == "__main__":
    main()