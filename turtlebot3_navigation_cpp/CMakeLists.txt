cmake_minimum_required(VERSION 3.8)
project(turtlebot3_navigation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(robot_driver src/robot_driver.cpp)
ament_target_dependencies(robot_driver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  navigation_interfaces
  rosidl_typesupport_cpp
)

add_executable(wall_finder_service src/wall_finder_service.cpp)
ament_target_dependencies(wall_finder_service
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  navigation_interfaces
  rosidl_typesupport_cpp
)

add_executable(lap_time_action_server src/lap_time_action_server.cpp)
ament_target_dependencies(lap_time_action_server
  rclcpp
  action_msgs
  rclcpp_action
  navigation_interfaces
  rosidl_typesupport_cpp
)

add_executable(lap_time_action_client src/lap_time_action_client.cpp)
ament_target_dependencies(lap_time_action_client
  rclcpp
  action_msgs
  rclcpp_action
  navigation_interfaces
  rosidl_typesupport_cpp
)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${navigation_interfaces_INCLUDE_DIRS}
)


install(TARGETS
  robot_driver
  wall_finder_service
  lap_time_action_server
  lap_time_action_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
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
