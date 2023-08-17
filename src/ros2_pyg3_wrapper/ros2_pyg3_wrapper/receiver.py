"""
Receiver module for glasses 3 python wrapper.
Handles parsing of messages received from the websocket and 
publishes data to predefined topics on the ROS server
"""
from geometry_msgs.msg import Point, Vector3

## import custom ros message types 
from ros2_pyg3_common.msg import UtilityStamped, ImuStamped, GazeStamped, CalibrationMarkerStamped, Point2D

# sys.path.insert(0, '/home/eyetrack_ws/src/ros2_PyGlasses3/')
from ros2_pyg3_wrapper.utilities import MsgID
# from utilities import MsgID


class Receiver():
    def __init__(self, rtsp_url=None, logger=None):
        self.__rtsp_url = rtsp_url
        self.logger = logger

        self.msgs = []

        self.misc_stream = []
        self.calibration_marker_stream = []
        self.gaze_stream = []
        self.imu_stream = []

        self.prev_calibmarker_id = None

        self.id_parser_map = {
            str(MsgID.GET_RECORDING_UNIT_INFO.value) : self.__recv_recording_unit_info,
        #     str(MsgID.GET_CURRENT_GAZE_FREQUENCY.value) : func,
        #     str(MsgID.GET_NUM_GAZE_SAMPLES.value) : func,
        #     str(MsgID.GET_NUM_VALID_GAZE_SAMPLES.value) : func,
        #     str(MsgID.GET_GAZE_SAMPLE.value) : func,
        #     str(MsgID.GET_IMU_SAMPLE.value) : func,
            str(MsgID.ACTION_EMIT_MARKERS.value) : self.__recv_emit_marker_response,
            str(MsgID.ACTION_RUN_CALIBRATE.value) : self.__recv_run_calibrate_response,
        #     str(MsgID.ACTION_CANCEL_RECORD.value) : func,
        #     str(MsgID.ACTION_SNAPSHOT_CURRENT.value) : func,
        #     str(MsgID.ACTION_START_RECORD.value) : func,
        #     str(MsgID.ACTION_STOPSAVE_RECORD.value) : func,
            # str(MsgID.ACTION_RUDIMENTARY_CALIBRATE.value) : func,
            str(MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value) : self.__recv_keepalive_response,
        #     str(MsgID.ACTION_RUDIMENTARY_SENDEVENT.value) : func,
            str(MsgID.SIGNAL_CALIBRATION_MARKERS.value) : self.__recv_calibration_markers_signal_id,
            str(MsgID.SIGNAL_RUDIMENTARY_GAZE.value) : self.__recv_gaze_stream_id,
            str(MsgID.SIGNAL_RUDIMENTARY_IMU.value) : self.__recv_imu_stream_id,
        }

    #####################################################################

    def parse_messages(self):
        """ Main function that goes through all received messages and parses
        REQUIREMENTS:
            [SORT]identify what each message is (id/signal value to meaning) 
            [BROADCAST] to ROS?        
        """
        ## get first message to read
        msg_to_parse = self.msgs[0]
        parsed_id = None

        try: 
            msg_id_str = str(msg_to_parse["id"])
        except:
            msg_id_str = str(msg_to_parse["signal"])

        msg_body = msg_to_parse["body"]

        parsed_id = self.id_parser_map[msg_id_str](msg_body)
        # self.logger.error("[RECEIVER] parsed {}".format(parsed_id))
        ## remove the processed message from the list 
        self.msgs.pop(0)

        return msg_id_str #parsed_id


    #####################################################################
    ######################### UTILITY FUNCTIONS #########################
    #####################################################################

    # def __identify_signal_id(self, msg):

    #     try:
    #         msg_id_str = str(msg["id"])
    #     except:
    #         msg_id_str = str(msg["signal"])

    #     self.id_parser_map[msg_id_str](msg)


    # def __get_pos_nums(self, num):
    #     """
    #     https://stackoverflow.com/questions/32752750/how-to-find-the-numbers-in-the-thousands-hundreds-tens-and-ones-place-in-pyth
    #     """
    #     pos_nums = []
    #     while num != 0:
    #         pos_nums.append(num % 10)
    #         num = num // 10
    #     return pos_nums

    ## function to update id-parser map (call after sending signal start)


    #####################################################################
    ################ API PROPERTIES GETTERS AND SETTERS #################
    #####################################################################

    def __recv_recording_unit_info(self, msg):
        utility_msg = UtilityStamped()
        utility_msg.type.data = str(MsgID.GET_RECORDING_UNIT_INFO.value)
        utility_msg.body.data = str(msg)
        self.misc_stream.append(utility_msg)
        return MsgID.GET_RECORDING_UNIT_INFO.value

    #####################################################################
    ############################ API ACTIONS ############################
    #####################################################################

    def __recv_emit_marker_response(self, msg):
        utility_msg = UtilityStamped()
        utility_msg.type.data = str(MsgID.ACTION_EMIT_MARKERS.value)
        utility_msg.body.data = str(msg)
        self.misc_stream.append(utility_msg)
        return MsgID.ACTION_EMIT_MARKERS.value

    def __recv_run_calibrate_response(self, msg):
        """
        Do not use this, just do calibration with the web client gui
        """
        return MsgID.ACTION_RUN_CALIBRATE.value

    def __recv_recorder_snapshot(self, msg):
        return MsgID.ACTION_SNAPSHOT_CURRENT.value

    def __recv_keepalive_response(self, msg):
        utility_msg = UtilityStamped()
        utility_msg.type.data = str(MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value)
        utility_msg.body.data = str(msg)
        self.misc_stream.append(utility_msg)
        return MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value

    #####################################################################
    ############################ API SIGNALS ############################
    #####################################################################

    ## get the signal id and pass to append the relevant function to the parser map
    def __recv_calibration_markers_signal_id(self, signal_id):
        self.id_parser_map[str(signal_id)] = self.__recv_calibration_marker_stream
        self.prev_calibmarker_id = signal_id
        return MsgID.SIGNAL_CALIBRATION_MARKERS.value
        
    def __recv_gaze_stream_id(self, signal_id):
        self.id_parser_map[str(signal_id)] = self.__recv_gaze_stream
        return MsgID.SIGNAL_RUDIMENTARY_GAZE.value

    def __recv_imu_stream_id(self, signal_id):
        self.id_parser_map[str(signal_id)] = self.__recv_imu_stream
        return MsgID.SIGNAL_RUDIMENTARY_IMU.value

    def __recv_calibration_marker_stream(self, msg):
        calib_marker = CalibrationMarkerStamped()
        calib_marker.g3_time = msg[0]

        try:
            calib_marker.marker_3d_pos.x = msg[1][0]
            calib_marker.marker_3d_pos.y = msg[1][1]
            calib_marker.marker_3d_pos.z = msg[1][2]
            calib_marker.marker_2d_pos.x = msg[2][0]
            calib_marker.marker_2d_pos.y = msg[2][1]
            self.calibration_marker_stream.append(calib_marker)
            return "calib marker stream"
        except:
            return "invalid calib marker stream"

    def __recv_gaze_stream(self, msg):
        gaze_msg = GazeStamped()
        try:
            gaze_msg.g3_time = msg[0]
            gaze_data = msg[1]
            
            if 'gaze3d' in gaze_data:
                gaze_msg.gaze3d.x = gaze_data['gaze3d'][0]
                gaze_msg.gaze3d.y = gaze_data['gaze3d'][1]
                gaze_msg.gaze3d.z = gaze_data['gaze3d'][2]
            if 'eyeleft' in gaze_data:
                if 'gazeorigin' in gaze_data['eyeleft']:
                    gaze_msg.eyeleft_gazeorigin.x = gaze_data['eyeleft']['gazeorigin'][0]
                    gaze_msg.eyeleft_gazeorigin.y = gaze_data['eyeleft']['gazeorigin'][1]
                    gaze_msg.eyeleft_gazeorigin.z = gaze_data['eyeleft']['gazeorigin'][2]
                if 'gazedirection' in gaze_data['eyeleft']:
                    gaze_msg.eyeleft_gazedirection.x = gaze_data['eyeleft']['gazedirection'][0]
                    gaze_msg.eyeleft_gazedirection.y = gaze_data['eyeleft']['gazedirection'][1]
                    gaze_msg.eyeleft_gazedirection.z = gaze_data['eyeleft']['gazedirection'][2]
                if 'pupildiameter' in gaze_data['eyeleft']:
                    gaze_msg.eyeleft_pupildiameter = gaze_data['eyeleft']['pupildiameter']
            if 'eyeright' in gaze_data:
                if 'gazeorigin' in gaze_data['eyeright']:
                    gaze_msg.eyeright_gazeorigin.x = gaze_data['eyeright']['gazeorigin'][0]
                    gaze_msg.eyeright_gazeorigin.y = gaze_data['eyeright']['gazeorigin'][1]
                    gaze_msg.eyeright_gazeorigin.z = gaze_data['eyeright']['gazeorigin'][2]
                if 'gazedirection' in gaze_data['eyeright']:
                    gaze_msg.eyeright_gazedirection.x = gaze_data['eyeright']['gazedirection'][0]
                    gaze_msg.eyeright_gazedirection.y = gaze_data['eyeright']['gazedirection'][1]
                    gaze_msg.eyeright_gazedirection.z = gaze_data['eyeright']['gazedirection'][2]
                if 'pupildiameter' in gaze_data['eyeright']:
                    gaze_msg.eyeright_pupildiameter = gaze_data['eyeright']['pupildiameter']

            ## check that gaze_data has stuff
            if 'gaze2d' in gaze_data:
                gaze_msg.gaze2d.x = gaze_data['gaze2d'][0]
                gaze_msg.gaze2d.y = gaze_data['gaze2d'][1]
                self.gaze_stream.append(gaze_msg)
            else: 
                return "invalid gaze stream"
                # gaze_msg.gaze2d.x = float(10)
                # gaze_msg.gaze2d.y = float(10)
            
            return "gaze stream"
        except:
            # gaze_msg.gaze2d.x = float(10)
            # gaze_msg.gaze2d.y = float(10)
            # self.gaze_stream.append(gaze_msg)
            return "gaze stream"
        

    def __recv_imu_stream(self, msg):
        imu_msg = ImuStamped()
        imu_msg.g3_time = msg[0]
        imu_data = msg[1]

        if 'accelerometer' in imu_data:
            imu_msg.imu.accel.linear.x = imu_data['accelerometer'][0]
            imu_msg.imu.accel.linear.y = imu_data['accelerometer'][1]
            imu_msg.imu.accel.linear.z = imu_data['accelerometer'][2]
        if 'gyroscope' in imu_data:
            imu_msg.imu.accel.angular.x = imu_data['gyroscope'][0]
            imu_msg.imu.accel.angular.y = imu_data['gyroscope'][1]
            imu_msg.imu.accel.angular.z = imu_data['gyroscope'][2]

        self.imu_stream.append(imu_msg)
        return "imu stream"