cmake_minimum_required(VERSION 3.8)
project(assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(assignment_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(fsm_node assignment/FSM.cpp)
ament_target_dependencies(fsm_node
  rclcpp
  assignment_interfaces
  geometry_msgs
  nav_msgs
)

install(TARGETS
  fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  assignment/aruco_image_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_detector
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
