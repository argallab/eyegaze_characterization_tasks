cmake_minimum_required(VERSION 3.8)
project(eyegaze_virtual_tasks)

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
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_pyg3_common REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME}) 

install(PROGRAMS
  nodes/snapshot_recorder.py
  nodes/unity_controller_2d.py
  nodes/unity_gaze_controller.py
  nodes/unity_paint_controller.py
  nodes/unity_focus_controller.py
  nodes/unity_correction_controller.py
  nodes/unity_trace_controller.py
  nodes/gaze_animator.py
  nodes/keyinput_task_support.py
  study_parameters/joystick_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
