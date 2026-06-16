cmake_minimum_required(VERSION 3.8)
project(mycobot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(mycobot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# ── Shared library ────────────────────────────────────────────────────────────
# Other packages (future VLA node, task planner) can link against this.
add_library(mycobot_motion_lib SHARED
  src/motion_manager.cpp
)
target_include_directories(mycobot_motion_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mycobot_motion_lib
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
)

# ── Action server node ────────────────────────────────────────────────────────
add_executable(motion_server_node
  src/motion_server_node.cpp
)
target_link_libraries(motion_server_node
  mycobot_motion_lib
)
ament_target_dependencies(motion_server_node
  mycobot_interfaces
  rclcpp
  rclcpp_action
)

# ── Demo / smoke-test node ────────────────────────────────────────────────────
add_executable(motion_demo_node
  src/motion_demo_node.cpp
)
ament_target_dependencies(motion_demo_node
  mycobot_interfaces
  rclcpp
  rclcpp_action
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS mycobot_motion_lib
  EXPORT export_mycobot_motion_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS motion_server_node motion_demo_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Export for downstream packages ───────────────────────────────────────────
ament_export_targets(export_mycobot_motion_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
