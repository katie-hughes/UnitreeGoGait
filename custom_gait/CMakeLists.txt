cmake_minimum_required(VERSION 3.8)
project(custom_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(gaitlib REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(custom_gait src/custom_gait.cpp)
ament_target_dependencies(custom_gait
  rclcpp
  ros2_unitree_legged_msgs
  std_srvs)
target_link_libraries(custom_gait gaitlib::gaitlib "${cpp_typesupport_target}")

# target_link_libraries(custom_gait "${cpp_typesupport_target}")
install(TARGETS
  custom_gait
  DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
