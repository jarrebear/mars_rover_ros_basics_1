cmake_minimum_required(VERSION 3.8)
project(basics_ros2_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

# Create executables
add_executable(function src/function.cpp)
add_executable(callback_function src/callback_function.cpp)
# add_executable(callback_spinonce_function src/callback_spinonce_function)
# add_executable(plant_detector src/plant_detector.cpp)

# Add dependencies
ament_target_dependencies(function rclcpp)
ament_target_dependencies(callback_function rclcpp)
# ament_target_dependencies(callback_spinonce_function rclcpp)
# ament_target_dependencies(plant_detector rclcpp)

# Install executables
install(TARGETS
  function
  callback_function
  #callback_spinonce_function
  #plant_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
