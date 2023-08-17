cmake_minimum_required(VERSION 3.8)
project(ros2_pyg3_wrapper)

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

add_executable(scene_camera_reader_node src/scene_camera_reader.cpp)
target_link_libraries(scene_camera_reader_node ${OpenCV_LIBS})
ament_target_dependencies(scene_camera_reader_node 
                          rclcpp 
                          OpenCV
                          ros2_pyg3_common)

install(TARGETS
  scene_camera_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/g3_wrapper.py
  nodes/g3_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
