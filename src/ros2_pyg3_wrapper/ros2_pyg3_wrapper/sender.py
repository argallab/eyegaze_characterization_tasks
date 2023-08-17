"""
Sender module for glasses 3 python wrapper.
Handles creation of messages for sending to the websocket 
"""
import base64

# sys.path.insert(0, '/home/eyetrack_ws/src/ros2_PyGlasses3/')
from ros2_pyg3_wrapper.utilities import MsgID
# from utilities import MsgID


class Sender():
    def __init__(self):
        self.msgs = []

        self.msgid_writefunc_map = {
            str(MsgID.GET_RECORDING_UNIT_INFO.value) : self.__get_recording_unit_info,
            str(MsgID.GET_CURRENT_GAZE_FREQUENCY.value) : self.__get_current_gaze_frequency,
            str(MsgID.GET_NUM_GAZE_SAMPLES.value) : self.__get_num_gaze_samples,
            str(MsgID.GET_NUM_VALID_GAZE_SAMPLES.value) : self.__get_num_valid_gaze_samples,
            str(MsgID.GET_GAZE_SAMPLE.value) : self.__get_gaze_sample,
            str(MsgID.GET_IMU_SAMPLE.value) : self.__get_imu_sample,
            str(MsgID.ACTION_EMIT_MARKERS.value) : self.__calibration_emit_markers,
            str(MsgID.ACTION_RUN_CALIBRATE.value) : self.__calibration_run,
            str(MsgID.ACTION_CANCEL_RECORD.value) : self.__cancel_delete_recording,
            # str(MsgID.ACTION_SEND_EVENT.value) : self.__recorder_send_event,  ## 2 *args
            str(MsgID.ACTION_SNAPSHOT_CURRENT.value) : self.__recorder_snapshot,
            str(MsgID.ACTION_START_RECORD.value) : self.__recorder_start,
            str(MsgID.ACTION_STOPSAVE_RECORD.value) : self.__recorder_stop,
            # str(MsgID.ACTION_DELETE_RECORDING.value) : self.__delete_stored_recording,    ## 1 arg
            str(MsgID.ACTION_RUDIMENTARY_CALIBRATE.value) : self.__rudimentary_calibrate,
            str(MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value) : self.__rudimentary_keepalive,
            # str(MsgID.ACTION_RUDIMENTARY_SENDEVENT.value) : self.__rudimentary_send_event,    ## 2 args
            str(MsgID.SIGNAL_CALIBRATION_MARKERS.value) : self.__signal_calibration_markers,
            str(MsgID.SIGNAL_RUDIMENTARY_GAZE.value) : self.__signal_rudimentary_gaze,
            str(MsgID.SIGNAL_RUDIMENTARY_IMU.value) : self.__signal_rudimentary_imu,
        }

    def write_send_message(self, msgid, *args):
        self.msgid_writefunc_map[str(msgid)](*args)
        return

    #####################################################################
    ################ API PROPERTIES GETTERS AND SETTERS #################
    #####################################################################

    ## GETTERS
    def __get_recording_unit_info(self, *args):
        """ Get recording unit serial number
        """
        msg = { 
            "path":"system.recording-unit-serial", 
            "id":MsgID.GET_RECORDING_UNIT_INFO.value, 
            "method":"GET"
        }
        self.msgs.append(msg)
        return
        # print("wrote msg to get recording unit info")
    
    def __get_recording_unit_time(self, *args):
        """ Get recording unit system time
        """
        msg = {
            "path":"system.time",
            "id":MsgID.GET_RECORDING_UNIT_TIME.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_battery_charging_state(self, *args):
        msg = {
            "path":"system/battery.charging",
            "id":MsgID.GET_BATTERY_CHARGING_STATE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_battery_level(self, *args):
        msg = {
            "path":"system/battery.level",
            "id":MsgID.GET_BATTERY_LEVEL.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_battery_remaining_time(self, *args):
        msg = {
            "path":"system/battery.remaining-time",
            "id":MsgID.GET_BATTERY_REMAINING_TIME.value,
            "method":"GET"
        }
        self.msgs.append(msg)
    
    def __get_battery_state(self, *args):
        msg = {
            "path":"system/battery.state",
            "id":MsgID.GET_BATTERY_STATE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_wifi_status(self, *args):
        """ Get recording unit wifi status
        """
        msg = {
            "path":"network.wifi-enable",
            "id":MsgID.GET_WIFI_STATUS.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_recording_creation_time(self, *args):
        """ Get creation time of the current recording (ongoing)
        """
        msg = {
            "path":"recorder.created",
            "id":MsgID.GET_RECORDING_CREATION_TIME.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_current_gaze_frequency(self, *args):
        """ Get current gaze frequency
        """
        msg = {
            "path":"recorder.current-gaze-frequency",
            "id":MsgID.GET_CURRENT_GAZE_FREQUENCY.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_recording_duration(self, *args):
        """ Get current duration of ongoing recording
        """
        msg = {
            "path":"recorder.duration",
            "id":MsgID.GET_RECORDING_DURATION.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_recording_folder(self, *args):
        """ Get the name of the folder holding the recording
        """
        msg = {
            "path":"recorder.folder",
            "id":MsgID.GET_RECORDING_FOLDER.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_if_gaze_overlay(self, *args):
        """ Get whether the recording was started with gaze overlay
        """
        msg = {
            "path":"recorder.gaze-overlay",
            "id":MsgID.GET_IF_GAZE_OVERLAY.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_num_gaze_samples(self, *args):
        msg = {
            "path":"recorder.gaze-samples",
            "id":MsgID.GET_NUM_GAZE_SAMPLES.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_remaining_time(self, *args):
        msg = {
            "path":"recorder.remaining-time",
            "id":MsgID.GET_REMAINING_TIME.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_recording_uuid(self, *args):
        msg = {
            "path":"recorder.uuid",
            "id":MsgID.GET_RECORDING_UUID.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_num_valid_gaze_samples(self, *args):
        msg = {
            "path":"recorder.valid-gaze-samples",
            "id":MsgID.GET_NUM_VALID_GAZE_SAMPLES.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_recording_visible_name(self, *args):
        msg = {
            "path":"recorder.visible-name",
            "id":MsgID.GET_RECORDING_VISIBLE_NAME.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_event_sample(self, *args):
        msg = {
            "path":"rudimentary.event-sample",
            "id":MsgID.GET_EVENT_SAMPLE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_gaze_sample(self, *args):
        msg = {
            "path":"rudimentary.gaze-sample",
            "id":MsgID.GET_GAZE_SAMPLE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_imu_sample(self, *args):
        msg = {
            "path":"rudimentary.imu-sample",
            "id":MsgID.GET_IMU_SAMPLE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_scene_quality(self, *args):
        msg = {
            "path":"rudimentary.scene-quality",
            "id":MsgID.GET_SCENE_QUALITY.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_scene_scale(self, *args):
        msg = {
            "path":"rudimentary.scene-scale",
            "id":MsgID.GET_SCENE_SCALE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_sync_port_sample(self, *args):
        msg = {
            "path":"rudimentary.sync-port-sample",
            "id":MsgID.GET_SYNC_PORT_SAMPLE.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_gaze_frequency_selection(self, *args):
        """
        Default val = 100
        """
        msg = {
            "path":"settings.gaze-frequency",
            "id":MsgID.GET_GAZE_FREQUENCY_SELECTION.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    def __get_gaze_overlay_selection(self, *args):
        """
        Default val = False
        """
        msg = {
            "path":"settings.gaze-overlay",
            "id":MsgID.GET_GAZE_OVERLAY_SELECTION.value,
            "method":"GET"
        }
        self.msgs.append(msg)

    ## SETTERS
    def __set_wifi_status(self, *args):
        """ Creates a message to set wifi status

        Args:
            status - a bool 
        """
        status = args[0]
        msg = {
            "path":"network.wifi-enable",
            "id":MsgID.SET_WIFI_STATUS.value,
            "method":"POST",
            "body":bool(status)
        }
        self.msgs.append(msg)

    def __set_recording_folder(self, *args):
        direc = args[0]
        msg = {
            "path":"recorder.folder",
            "id":MsgID.SET_RECORDING_FOLDER.value,
            "method":"POST",
            "body":str(direc)
        }
        self.msgs.append(msg)

    def __set_recording_visible_name(self, *args):
        name = args[0]
        msg = {
            "path":"recorder.visible-name",
            "id":MsgID.SET_RECORDING_VISIBLE_NAME.value,
            "method":"POST",
            "body":str(name)
        }
        self.msgs.append(msg)

    def __set_scene_quality(self, *args):
        """
        Default val = 75
        """
        valint = args[0]
        msg = {
            "path":"rudimentary.scene-quality",
            "id":MsgID.SET_SCENE_QUALITY.value,
            "method":"POST",
            "body":int(valint)
        }
        self.msgs.append(msg)

    def __set_scene_scale(self, *args):
        """
        Default val = 2
        """
        valint = args[0]
        msg = {
            "path":"rudimentary.scene-scale",
            "id":MsgID.SET_SCENE_SCALE.value,
            "method":"POST",
            "body":int(valint)
        }
        self.msgs.append(msg)

    def __set_gaze_frequency_selection(self, *args):
        """
        Default val = 100
        """
        valint = args[0]
        msg = {
            "path":"settings.gaze-frequency",
            "id":MsgID.SET_GAZE_FREQUENCY_SELECTION.value,
            "method":"POST",
            "body":int(valint)
        }
        self.msgs.append(msg)

    def __set_gaze_overlay_selection(self, *args):
        """
        Default val = False
        """
        status = args[0]
        msg = {
            "path":"settings.gaze-overlay",
            "id":MsgID.SET_GAZE_OVERLAY_SELECTION.value,
            "method":"POST",
            "body":bool(status)
        }
        self.msgs.append(msg)

    
    #####################################################################
    ############################ API ACTIONS ############################
    #####################################################################

    def __calibration_emit_markers(self, *args):
        msg = {
            "path":"calibrate!emit-markers",
            "id":MsgID.ACTION_EMIT_MARKERS.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __calibration_run(self, *args):
        msg = {
            "path":"calibrate!run",
            "id":MsgID.ACTION_RUN_CALIBRATE.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __reset_network(self, *args):
        msg = {
            "path":"network!reset",
            "id":MsgID.ACTION_RESET_NETWORK.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __cancel_delete_recording(self, *args):
        msg = {
            "path":"recorder!cancel",
            "id":MsgID.ACTION_CANCEL_RECORD.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __recording_meta_insert(self, *args):
        """ Insert a key-value pair into the meta data

        Args:
            key: Name of meta object (string)
            val: Value base64 meta data, supply null to remove key        
        """
        key = args[0]
        val = args[1]
        valbytes = base64.b64encode(val.encode("ascii"))    # encode to bytes, then b64 encode
        msg = {
            "path":"recorder!meta-insert",
            "id":MsgID.ACTION_META_INSERT.value,
            "method":"POST",
            "body":[key, valbytes]
        }
        self.msgs.append(msg)

    def __list_meta_keys(self, *args):
        msg = {
            "path":"recorder!meta-keys",
            "id":MsgID.ACTION_META_KEYS.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __key_meta_lookup(self, *args):
        """ Get the meta data associated with a specific key 
        
        Args:
            key: Name of meta object (string)

        method should return base64 meta data
        """
        key = args[0]
        msg = {
            "path":"recorder!meta-lookup",
            "id":MsgID.ACTION_META_LOOKUP.value,
            "method":"POST",
            "body":[key]
        }
        self.msgs.append(msg)
        
    def __recorder_send_event(self, *args):
        """ Send event to recording
        
        Args:
            tag: tag name of the event (string)
            data: object data for the event (object)
        """
        tag = args[0]
        data = args[1]
        msg = {
            "path":"recorder!send-event",
            "id":MsgID.ACTION_SEND_EVENT.value,
            "method":"POST",
            "body":[tag, data]
        }
        self.msgs.append(msg)

    def __recorder_snapshot(self, *args):
        """ Store a snapshot jpeg image of the current scene 
        """
        msg = {
            "path":"recorder!snapshot",
            "id":MsgID.ACTION_SNAPSHOT_CURRENT.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __recorder_start(self, *args):
        """ Create and start a new recording
        """
        msg = {
            "path":"recorder!start",
            "id":MsgID.ACTION_START_RECORD.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __recorder_stop(self, *args):
        """ Stop and save an ongoing recording 
        """
        msg = {
            "path":"recorder!stop",
            "id":MsgID.ACTION_STOPSAVE_RECORD.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __delete_stored_recording(self, *args):
        """ Delete recording by UUID.
            This only marks the recording for deletion and removes it 
            from the list of recordings, the recording is deleted when 
            the last viewer has closed the connection to the recording (see deleted signal).

        Args:
            uuid: uuid of the recording to be deleted (string)
        """
        uuid = args[0]
        msg = {
            "path":"recordings!delete",
            "id":MsgID.ACTION_DELETE_RECORDING.value,
            "method":"POST",
            "body":[uuid]
        }
        self.msgs.append(msg)

    def __rudimentary_calibrate(self, *args):
        msg = {
            "path":"rudimentary!calibrate",
            "id":MsgID.ACTION_RUDIMENTARY_CALIBRATE.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __rudimentary_keepalive(self, *args):
        """ Start and keep rudimentary streams alive 
            Ensure signal emission for 6 seconds
        """
        msg = {
            "path":"rudimentary!keepalive",
            "id":MsgID.ACTION_RUDIMENTARY_KEEPALIVE.value,
            "method":"POST",
            "body":[]
        }
        self.msgs.append(msg)

    def __rudimentary_send_event(self, *args):
        """ Send event
        
        Args:
            tag: tag name of the event (string)
            data: object data for the event (object)
        """
        tag = args[0]
        data = args[1]
        msg = {
            "path":"rudimentary!send-event",
            "id":MsgID.ACTION_RUDIMENTARY_SENDEVENT.value,
            "method":"POST",
            "body":[tag, data]
        }
        self.msgs.append(msg)

    
    #####################################################################
    ############################ API SIGNALS ############################
    #####################################################################

    def __signal_calibration_markers(self, *args):
        """ Send out calibration markers when found 
        
        While the marker detection is enabled, the signal calibrate:marker 
        will emit values for each video frame that has been processed.
        """
        msg = {
            "path":"calibrate:marker",
            "id":MsgID.SIGNAL_CALIBRATION_MARKERS.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)

    def __signal_recording_started(self, *args):
        """ Signal emitted when a recording is started 
        """
        msg = {
            "path":"recorder:started",
            "id":MsgID.SIGNAL_RECORDER_STARTED.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)

    def __signal_recording_stopped(self, *args):
        """ Signal emitted when a recording is completed
        """
        msg = {
            "path":"recorder:stopped",
            "id":MsgID.SIGNAL_RECORDER_STOPPED.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)

    def __signal_rudimentary_event(self, *args):
        """ Emits event objects 
        """
        msg = {
            "path":"rudimentary:event",
            "id":MsgID.SIGNAL_RUDIMENTARY_EVENT.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)

    def __signal_rudimentary_gaze(self, *args):
        """ Emits gaze objects 
        """
        msg = {
            "path":"rudimentary:gaze",
            "id":MsgID.SIGNAL_RUDIMENTARY_GAZE.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)

    def __signal_rudimentary_imu(self, *args):
        """ Emits imu sample objects 
        """
        msg = {
            "path":"rudimentary:imu",
            "id":MsgID.SIGNAL_RUDIMENTARY_IMU.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)
    
    def __signal_rudimentary_scene(self, *args):
        """ Emits down-scaled jpeg images from the scene camera

        NOTE: this is a heavy task by the RU and will greatly reduce 
            battery time and processing time for gaze data. 
            Use only if absolutely needed.
        """
        msg = {
            "path":"rudimentary:scene",
            "id":MsgID.SIGNAL_RUDIMENTARY_SCENE.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)
    
    def __signal_rudimentary_syncport(self, *args):
        """ Emits syncport objects 
        """
        msg = {
            "path":"rudimentary:sync-port",
            "id":MsgID.SIGNAL_RUDIMENTARY_SYNCPORT.value,
            "method":"POST",
            "body":None     ## null in json
        }
        self.msgs.append(msg)

