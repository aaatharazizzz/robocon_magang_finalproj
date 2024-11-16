cmake_minimum_required(VERSION 3.8)
project(sim2025_follow_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sim2025_interfaces REQUIRED)

add_library(action_server SHARED
  src/sim2025_follow_target.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_server
  "sim2025_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs")
rclcpp_components_register_node(action_server PLUGIN "Sim2025FollowTarget" EXECUTABLE follow_target)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# add_executable(followtarget src/sim2025_followtarget.cpp)
# ament_target_dependencies(followtarget rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs sim2025_interfaces)

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


# install(TARGETS
#  followtarget
#  DESTINATION lib/${PROJECT_NAME})

ament_package()
