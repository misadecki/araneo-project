cmake_minimum_required(VERSION 3.8)
project(quadruped_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(quadruped_kinematics REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(quadruped_gait
  src/trajectory.cpp
  src/gait_generator.cpp
)
target_include_directories(quadruped_gait PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(quadruped_gait PUBLIC
  quadruped_kinematics::quadruped_kinematics
)

add_executable(test_traj 
  tools/src/trajectory_test.cpp
  tools/src/gait_config.cpp
)
target_include_directories(test_traj PRIVATE tools/include)
target_link_libraries(test_traj 
  quadruped_gait 
  yaml-cpp
)

add_executable(test_gait 
  tools/src/gait_generator_test.cpp 
  tools/src/gait_config.cpp
)
target_include_directories(test_gait PRIVATE tools/include)
target_link_libraries(test_gait quadruped_gait yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS quadruped_gait 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS test_traj test_gait
  DESTINATION lib/${PROJECT_NAME}
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
