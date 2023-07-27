cmake_minimum_required(VERSION 3.8)
project(beliv_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Add the new source files here
set(SOURCES
  src/beliv_veh_interface.cpp
  src/beliv_veh_interface_node.cpp
)

# Add the executable for your node
add_executable(beliv_veh_interface_node ${SOURCES})

# Example: If your node depends on ROS 2 messages, uncomment and fill this line.
# ament_target_dependencies(my_node
#   <ros2_message_package>
# )

# install the executable to lib/beliv_vehicle_interface
install(TARGETS
  beliv_veh_interface_node
  DESTINATION lib/${beliv_vehicle_interface}
)
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}  # Add this line
)

endif()

ament_package()
