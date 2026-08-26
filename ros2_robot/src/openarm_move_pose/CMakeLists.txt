cmake_minimum_required(VERSION 3.8)
project(openarm_move_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MovePose.action"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# rosidl already owns the openarm_move_pose Python package; merge our modules into it.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  FILES
    openarm_move_pose/move_group_goal.py
    openarm_move_pose/move_pose_node.py
    openarm_move_pose/ee_pose_publisher.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_pose_node
  scripts/ee_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
