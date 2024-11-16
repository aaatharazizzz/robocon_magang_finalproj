cmake_minimum_required(VERSION 3.8)
project(sim2025_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(sim2025_control_joy src/sim2025_control_joy.cpp)
ament_target_dependencies(sim2025_control_joy rclcpp geometry_msgs sensor_msgs)

add_executable(sim2025_control_key src/sim2025_control_key.cpp)
ament_target_dependencies(sim2025_control_key rclcpp geometry_msgs)

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

install(TARGETS
  sim2025_control_joy
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
sim2025_control_key
  DESTINATION lib/${PROJECT_NAME})

ament_package()
