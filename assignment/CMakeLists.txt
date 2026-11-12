cmake_minimum_required(VERSION 3.8)
project(assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(assignment_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Locate FSM.cpp (supports standard src/ or package directory)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/FSM.cpp")
  set(FSM_SRC "src/FSM.cpp")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assignment/FSM.cpp")
  set(FSM_SRC "assignment/FSM.cpp")
else()
  set(FSM_SRC "FSM.cpp")
endif()

# Build FSM node executable
add_executable(fsm_node ${FSM_SRC})
ament_target_dependencies(fsm_node
  rclcpp
  assignment_interfaces
  geometry_msgs
  nav_msgs
)

# Also provide 'fsm' as an executable alias
add_executable(fsm ${FSM_SRC})
ament_target_dependencies(fsm
  rclcpp
  assignment_interfaces
  geometry_msgs
  nav_msgs
)

# Install C++ nodes
install(TARGETS
  fsm_node
  fsm
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes/scripts
install(PROGRAMS
  assignment/aruco_image_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_detector
)

install(PROGRAMS
  assignment/aruco_image_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
