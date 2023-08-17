cmake_minimum_required(VERSION 3.8)
project(ros2_pyg3_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/UtilityStamped.msg"
  "msg/ImuStamped.msg"
  "msg/GazeScreenStamped.msg"
  "msg/GazeStamped.msg"
  "msg/CalibrationMarkerStamped.msg"
  "msg/ScreenCornersStamped.msg"
  "msg/FocusTarget.msg"
  "msg/TraceTarget.msg"
  "msg/Alpha.msg"
  "msg/TimeInt32.msg"
  "srv/RecordJoyTrace.srv"
  "srv/UnityTarget.srv"
  "srv/UnityTrace.srv"
  "srv/WebsocketRequest.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
