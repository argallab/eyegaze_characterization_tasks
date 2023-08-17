#!/usr/bin/env python3
"""
Code developed by Larisa YC Loke in February 2023. Copyright (c) 2023. Larisa YC Loke, Argallab.

This node sets up the ROS2 service client to the websocket service in the PyGlasses3 wrapper.
This node also subscribes to the gaze_stream and screen_corners topics from the PyGlasses3 wrapper, 
    and computes the transformed gaze position on the screen to publish the gaze_screen topic.
"""
import numpy as np
from scipy.interpolate import interp1d
from enum import Enum
import matplotlib.path as mpltPath

## ros2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

## import custom ROS2 types
from ros2_pyg3_common.srv import WebsocketRequest
from ros2_pyg3_common.msg import (
    UtilityStamped, 
    ImuStamped, 
    ScreenCornersStamped,
    GazeScreenStamped, GazeStamped, 
    CalibrationMarkerStamped
)

## importing PyGlasses3 python modules
from ros2_pyg3_wrapper.utilities import MsgID
from eyegaze_virtual_tasks.transform2d import SceneToScreen2D
from eyegaze_virtual_tasks.streamprocessing import GazeSmoothing
from eyegaze_virtual_tasks.transform2d import ScreenResolution


class UnityGazeController(Node):
    def __init__(self):
        super().__init__('unity_gaze_controller')
        
        self.__glasses_addr = "192.168.8.109"
        self.__ws_url = "ws://{}/websocket".format(self.__glasses_addr)
        
        self.vcap = None
        self.__screen_size = ScreenResolution.STUDY.value        
        self.__screen_updated = False
        self.screen_corners = np.zeros((4, 2))
        self.screen_poly = None

        self.gaze_2d = np.zeros(2)

        self.se2transform = None
        self.screen_pos_2d = np.zeros(2)       ## have this as normalized coordinate in video frame

        self.gaze_window = GazeSmoothing(20, self.get_logger()) ## smooth over 20 samples

        self.__initialize_callback_groups()
        self.__initialize_services()
        self.__initialize_timers()
        self.__initialize_subscribers()
        self.__initialize_publishers()

    def startup(self):
        startup_msgs = [MsgID.GET_RECORDING_UNIT_INFO.value,
                        MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value, 
                        MsgID.SIGNAL_RUDIMENTARY_GAZE.value,
                        MsgID.SIGNAL_RUDIMENTARY_IMU.value]

        width = 1920.0
        height = 1080.0
        self.vcap_vertical_mapper = interp1d([0, height], [height, 0], bounds_error=False, fill_value="extrapolate")
        self.x_gaze_to_scene_mapper = interp1d([0, 1], [0, width], bounds_error=False, fill_value="extrapolate")
        self.y_gaze_to_scene_mapper = interp1d([0, 1], [height, 0], bounds_error=False, fill_value="extrapolate")
        self.vcap_resolution = (width, height)
        self.get_logger().info("video resolution: {}".format(self.vcap_resolution))
        
        for i in range(len(startup_msgs)):
            req = WebsocketRequest.Request()
            req.msgid = startup_msgs[i]
            future = self.ws_req_cli.call_async(req)

    def __initialize_callback_groups(self):
        self.__srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.__keepalive_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.__gaze2dpub_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.__screendetect_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.__gazesub_cb_group = MutuallyExclusiveCallbackGroup()

    def __initialize_services(self):
        self.ws_req_cli = self.create_client(WebsocketRequest, 'websocket_request', callback_group=self.__srv_cb_group)
        while not self.ws_req_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WebsocketRequest.Request()

    def __initialize_publishers(self):
        self.gaze2d_pub = self.create_publisher(GazeScreenStamped, 'gaze_screen', 1)

    def __initialize_subscribers(self):
        self.gaze_sub = self.create_subscription(GazeStamped, 'gaze_stream', self.gaze_sub_callback, 1, callback_group=self.__gazesub_cb_group)
        # self.imu_sub = self.create_subscription(ImuStamped, 'imu_stream', self.imu_sub_callback, 10)
        # self.calibmarker_sub = self.create_subscription(CalibrationMarkerStamped, 'calibration_marker_stream', self.calibmarker_sub_callback, 10)
        self.glasses_utility_sub = self.create_subscription(UtilityStamped, 'glasses_utility', self.utility_sub_callback, 1)
        self.screencorners_sub = self.create_subscription(ScreenCornersStamped, 'screen_corners', self.screencorners_sub_callback, 1)

    def __initialize_timers(self):
        self.keepalive_timer = self.create_timer(5, self.keepalive_timer_callback, callback_group=self.__keepalive_timer_cb_group)

        ## gaze2pub timer is 100Hz
        # self.gaze2dpub_timer = self.create_timer(0.01, self.gaze2dpub_timer_callback, callback_group=self.__gaze2dpub_timer_cb_group)
        self.screendetect_timer = self.create_timer(2, self.screendetect_timer_callback, callback_group=self.__screendetect_timer_cb_group)
        return

    def gaze_sub_callback(self, gaze):
        """
        Subscriber callback for gaze subscription from wrapper

        Smoothing is done on the gaze signal in this callback. If the gaze signal is invalid (None), 
        then ignore those signals and only use the valid signals in the time window
        
        Gaze is published as a coordinate position on the screen
        Screen limits are dependent on the resolution of the screen being used
        
        """
        if gaze.gaze2d.x < 2:
            self.gaze_2d = self.gaze_window.compute_smoothed_signal(True, self.x_gaze_to_scene_mapper(gaze.gaze2d.x), self.y_gaze_to_scene_mapper(gaze.gaze2d.y))
            # self.get_logger().error("GAZE IN SCENE: {}".format(self.gaze_2d))
        else:
            self.gaze_2d = self.gaze_window.compute_smoothed_signal(False)

        if self.screen_poly is not None and self.__screen_updated:
            gaze2pub = GazeScreenStamped()

            ################# extrapolate gaze block ################
            gaze2pub.gaze2d.x, gaze2pub.gaze2d.y = self.se2transform.compute(self.gaze_2d, self.__check_gaze_on_screen())
            
            self.gaze2d_pub.publish(gaze2pub)
            #########################################################
        else:
            return

    def imu_sub_callback(self, imu):
        raise NotImplementedError("not used in this study")
        return
        # self.get_logger().info("IMU RECEIVED, ACCEL: [{}, {}, {}]".format(imu.imu.accel.linear.x, imu.imu.accel.linear.y, imu.imu.accel.linear.z))

    def calibmarker_sub_callback(self, calibmarker):
        raise NotImplementedError("not used in this study")
        # self.get_logger().info("calibration marker position: 3d: [{}, {}, {}]; 2d: [{}, {}]".format( \
            # calibmarker.marker_3d_pos.x, calibmarker.marker_3d_pos.y, calibmarker.marker_3d_pos.z, calibmarker.marker_2d_pos.x, calibmarker.marker_2d_pos.y))
        return

    def utility_sub_callback(self, msg):
        raise NotImplementedError("not used in this study")
        # self.get_logger().info("UTILITY MSG RECEIVED, TYPE: {}, MSG: {}".format(msg.type.data, msg.body.data))
        return

    def __check_gaze_on_screen(self):
        """
        Using matplotlib.Path to check if point (2d gaze) is contained in a polygon (screen)
        """
        if self.gaze_2d[0] is not None:
            return self.screen_poly.contains_point(self.gaze_2d)
        else:
            return False

    def keepalive_timer_callback(self):
        ka_req = WebsocketRequest.Request()
        ka_req.msgid = MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value
        future = self.ws_req_cli.call_async(ka_req)
    def screencorners_sub_callback(self, corners):
        self.__screen_updated = bool(corners.updatedonce)

        ## the numbers that come in are in (1920, 1080) coordinates 
        if self.__screen_updated:

            ### CW FROM BOTTOM LEFT CORNER (0,0)
            self.screen_corners[0, 0] = int(corners.bottomleft.x)
            self.screen_corners[0, 1] = int(self.vcap_vertical_mapper(corners.bottomleft.y))
            self.screen_corners[1, 0] = int(corners.topleft.x)
            self.screen_corners[1, 1] = int(self.vcap_vertical_mapper(corners.topleft.y))
            self.screen_corners[2, 0] = int(corners.topright.x)
            self.screen_corners[2, 1] = int(self.vcap_vertical_mapper(corners.topright.y))
            self.screen_corners[3, 0] = int(corners.bottomright.x)
            self.screen_corners[3, 1] = int(self.vcap_vertical_mapper(corners.bottomright.y))

        return
        
    def screendetect_timer_callback(self):
        if self.__screen_updated:
            poly_ = np.concatenate((self.screen_corners, self.screen_corners[0, :].reshape((1, 2))), axis=0)
            self.screen_poly = mpltPath.Path(poly_)
            self.se2transform = SceneToScreen2D(self.__screen_size, self.screen_corners, self.get_logger())

def main():
    rclpy.init()
    gaze_controller = UnityGazeController()

    executor = MultiThreadedExecutor()

    executor.add_node(gaze_controller)
    gaze_controller.startup()

    while rclpy.ok(): 
        executor.spin_once()

    gaze_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()