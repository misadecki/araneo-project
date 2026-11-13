cmake_minimum_required(VERSION 3.8)
project(quadruped_gait_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadruped_gait REQUIRED)
find_package(quadruped_config REQUIRED)

add_executable(gait_node
  src/gait_node.cpp
  src/gait_node_main.cpp
)
target_include_directories(gait_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gait_node 
  quadruped_gait::quadruped_gait
  quadruped_config::quadruped_config
)

install(TARGETS gait_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(gait_node rclcpp sensor_msgs)
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
