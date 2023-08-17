# Custom ROS2 interfaces for ros2_pyg3_wrapper package
## ROS2 package containing custom interfaces types for ROS2-Glasses3 communication

## Messages
* Alpha - float32 wrapper for alpha value of targets to unity
* CalibrationMarkerStamped - for calibration marker stream publisher
* FocusTarget - position and size of target
* GazeScreenStamped - position of gaze in screen coordinates
* GazeStamped - for gaze stream publisher
* ImuStamped - for imu stream publisher, wraps geometry_msgs/AccelStamped
* Point2D - convenience type for a x,y point
* ScreenCornersStamped - positions of screen corners
* TimeInt32 - time convenience function
* TraceTarget - position of trace target
* UtilityStamped - wrapper for String type, for miscellaneous messages


## Services
* UnityTarget - sending a target to unity and getting unity pixel scale radius back
* UnityTrace - sending waypoints to unity and getting unity pixel waypoints back
* WebsocketRequest - service to request specific msgid be sent to websocket via wrapper node
