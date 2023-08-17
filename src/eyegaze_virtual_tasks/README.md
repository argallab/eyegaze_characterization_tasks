# ROS2 Python package argallab eyegaze studies

## Setting up Unity environment 
1. `Window` --> `Package Manager` --> `+` --> `Add package from git url` --> paste `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
    This gives the interface between ROS and unity. Make sure to change unity ROS settings to ROS2: `Robotics` --> `ROS settings` --> drop down to ROS2
2. Compile ros messages: `Robotics` --> `Generate ROS Messages` 
    Set ROS message path to the path to the `ros2_pyg3_common` package
    Build all msgs and srvs

## Calibration
Do calibration through Glasses web API

## Launch glasses wrapper
`ros2 launch ros2_pyg3_wrapper glasses.launch.py`

## Launch main study launch file 
`ros2 launch eyegaze_virtual_tasks study_unity.launch.py sid:=<SUBJECT_ID> train:=<BOOL> task:=<TASK_STRING>`

## Run keyinput node
`ros2 run eyegaze_virtual_tasks keyinput_task_support.py <TASK_STRING>`
* `s` to start
* `n` for next trace (only for trace task)
* `p` for pause `r` for resume (only for focus and correction task)