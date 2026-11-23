cmake_minimum_required(VERSION 3.8)
project(assignment_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco core highgui imgproc)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TakePhoto.srv"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  DEPENDENCIES std_msgs
)

add_executable(take_photo_service src/take_photo_service.cpp)
target_include_directories(take_photo_service PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(take_photo_service
  cv_bridge
  rclcpp
  sensor_msgs
  image_transport
  std_msgs
)
target_link_libraries(take_photo_service ${OpenCV_LIBS})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(take_photo_service "${cpp_typesupport_target}")

install(TARGETS take_photo_service DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs)

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
