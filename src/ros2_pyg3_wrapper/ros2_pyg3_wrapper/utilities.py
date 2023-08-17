from enum import Enum


class MsgID(Enum):
    """
    id format: A B C D
    A --> 1st level child of root
    B --> child of A
    C --> type of action (0:get, 1:set, 2:action, 3:signal)
    D --> val for id
    """

    #####################################################################
    ############## GETTERS AND SETTERS OF PROPERTIES IDS ################
    #####################################################################
     
    ## SYSTEM properties
    GET_RECORDING_UNIT_INFO = 1000
    GET_RECORDING_UNIT_TIME = 1001
    GET_BATTERY_CHARGING_STATE = 1100
    GET_BATTERY_LEVEL = 1101
    GET_BATTERY_REMAINING_TIME = 1102
    GET_BATTERY_STATE = 1103
    ### TODO: implement the /system/storage properties 

    ## NETWORK properties
    GET_WIFI_STATUS = 3000
    SET_WIFI_STATUS = 3010

    ## RECORDER properties
    GET_RECORDING_CREATION_TIME = 4000
    GET_CURRENT_GAZE_FREQUENCY = 4001
    GET_RECORDING_DURATION = 4002
    GET_RECORDING_FOLDER = 4003
    GET_IF_GAZE_OVERLAY = 4004
    GET_NUM_GAZE_SAMPLES = 4005
    GET_REMAINING_TIME = 4006
    GET_RECORDING_UUID = 4007
    GET_NUM_VALID_GAZE_SAMPLES = 4008
    GET_RECORDING_VISIBLE_NAME = 4009
    SET_RECORDING_FOLDER = 4010
    SET_RECORDING_VISIBLE_NAME = 4011

    ## RUDIMENTARY properties 
    ## Realistically, will only use these properties
    GET_EVENT_SAMPLE = 6000
    GET_GAZE_SAMPLE = 6001
    GET_IMU_SAMPLE = 6002
    GET_SCENE_QUALITY = 6003
    GET_SCENE_SCALE = 6004
    GET_SYNC_PORT_SAMPLE = 6005
    SET_SCENE_QUALITY = 6010
    SET_SCENE_SCALE = 6011

    ## SETTINGS properties
    GET_GAZE_FREQUENCY_SELECTION = 7000
    GET_GAZE_OVERLAY_SELECTION = 7001
    SET_GAZE_FREQUENCY_SELECTION = 7010
    SET_GAZE_OVERLAY_SELECTION = 7011

    #####################################################################
    ############################ ACTIONS IDS ############################
    #####################################################################
    
    ## SYSTEM actions (NOT YET IMPLEMENTED)
    ACTION_USE_NTP = 1020

    ## CALIBRATE actions
    ACTION_EMIT_MARKERS = 2020
    ACTION_RUN_CALIBRATE = 2021

    ## NETWORK actions
    ACTION_RESET_NETWORK = 3020

    ## RECORDER actions
    ACTION_CANCEL_RECORD = 4020
    ACTION_META_INSERT = 4021
    ACTION_META_KEYS = 4022
    ACTION_META_LOOKUP = 4023
    ACTION_SEND_EVENT = 4024
    ACTION_SNAPSHOT_CURRENT = 4025
    ACTION_START_RECORD = 4026
    ACTION_STOPSAVE_RECORD = 4027

    ## RECORDINGS actions
    ACTION_DELETE_RECORDING = 5020

    ## RUDIMENTARY actions
    ACTION_RUDIMENTARY_CALIBRATE = 6020
    ACTION_RUDIMENTARY_KEEPALIVE = 6021 ## ensures signal emission for 6s
    ACTION_RUDIMENTARY_SENDEVENT = 6022


    #####################################################################
    ############################ SIGNALS IDS ############################
    #####################################################################

    ## CALIBRATE signals 
    SIGNAL_CALIBRATION_MARKERS = 2030

    ## RECORDER signals 
    SIGNAL_RECORDER_STARTED = 3030
    SIGNAL_RECORDER_STOPPED = 3031

    ## RUDIMENTARY signals 
    SIGNAL_RUDIMENTARY_EVENT = 6030
    SIGNAL_RUDIMENTARY_GAZE = 6031
    SIGNAL_RUDIMENTARY_IMU = 6032
    SIGNAL_RUDIMENTARY_SCENE = 6033
    SIGNAL_RUDIMENTARY_SYNCPORT = 6034

    ## SETTINGS signals (NOT YET IMPLEMENTED)
    SIGNAL_SETTING_CHANGED = 7030

    ## UPGRADE signals (NOT YET IMPLEMENTED)
    SIGNAL_UPGRADE_COMPLETED = 8030
    SIGNAL_UPGRADE_PROGRESS = 8031

