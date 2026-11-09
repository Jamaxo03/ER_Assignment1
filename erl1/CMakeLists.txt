cmake_minimum_required(VERSION 3.8)
project(erl1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(aruco_image_detector src/aruco_image_detector.cpp)
target_include_directories(aruco_image_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(aruco_image_detector
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)
target_link_libraries(aruco_image_detector ${OpenCV_LIBS})

install(TARGETS
  aruco_image_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  erl1/aruco_image_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_image_detector_py
)

install(DIRECTORY
  launch
  worlds
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
