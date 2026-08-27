cmake_minimum_required(VERSION 3.22)
project(openarm_gravity_pd_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# External C++ dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(OpenArmCAN REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)

# Compatibility shim for distributions that ship yaml-cpp without an imported target
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
  if(TARGET yaml-cpp)
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE yaml-cpp)
  else()
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE ${yaml-cpp_LIBRARIES})
    target_include_directories(yaml-cpp::yaml-cpp INTERFACE ${yaml-cpp_INCLUDE_DIRS})
  endif()
endif()

# Compatibility shim for distributions that ship kdl_parser without imported target
if(NOT TARGET kdl_parser::kdl_parser)
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser
                        INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser
                             INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Library: arm_dynamics  (KDL gravity solver — no ROS dependency)
# ─────────────────────────────────────────────────────────────────────────────
add_library(arm_dynamics STATIC
  src/arm_dynamics.cpp
)
target_include_directories(arm_dynamics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_dynamics
  PUBLIC
    ${orocos_kdl_LIBRARIES}
    kdl_parser::kdl_parser
    urdfdom::urdfdom_model
    Eigen3::Eigen
)

# ─────────────────────────────────────────────────────────────────────────────
# Library: arm_controller  (CAN init + gravity + PD control)
# ─────────────────────────────────────────────────────────────────────────────
add_library(arm_controller STATIC
  src/arm_controller.cpp
)
target_include_directories(arm_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_controller
  PUBLIC
    arm_dynamics
    OpenArmCAN::openarm_can
    yaml-cpp::yaml-cpp
)
# arm_controller uses rclcpp logger and sensor_msgs types in its public header
ament_target_dependencies(arm_controller PUBLIC rclcpp sensor_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# Executable: openarm_gravity_pd_node
# ─────────────────────────────────────────────────────────────────────────────
add_executable(openarm_gravity_pd_node
  src/openarm_gravity_pd_node.cpp
)
target_include_directories(openarm_gravity_pd_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(openarm_gravity_pd_node
  arm_controller
  Threads::Threads
)
ament_target_dependencies(openarm_gravity_pd_node
  rclcpp
  sensor_msgs
  std_srvs
)

# ─────────────────────────────────────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS openarm_gravity_pd_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
