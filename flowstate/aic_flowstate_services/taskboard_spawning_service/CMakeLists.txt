cmake_minimum_required(VERSION 3.8)
project(taskboard_spawning_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aic_flowstate_interface REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)

# Include directories
include_directories(include)

# 1. Build Logic Library (Clean separation of concerns)
add_library(taskboard_spawner_logic
  src/taskboard_spawner_logic.cc
)
target_link_libraries(taskboard_spawner_logic
  yaml-cpp
)

# 2. Build ROS 2 Node Executable
add_executable(taskboard_spawner_node
  src/taskboard_spawner_node.cpp
)
target_link_libraries(taskboard_spawner_node
  taskboard_spawner_logic
  rclcpp::rclcpp
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  ${aic_flowstate_interface_TARGETS}
)

# Install rules
install(
  TARGETS taskboard_spawner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#===============================================================================
# Generate Solution Manifest
intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME taskboard_spawner_node
  MANIFEST taskboard_spawning_service.manifest.textproto
  PARAMETER_DESCRIPTOR taskboard_spawning_service.proto
  DEFAULT_CONFIGURATION default_config.pbtxt
  PROTOS_TARGET taskboard_spawning_service_protos
)

# Force manifest generation
add_dependencies(taskboard_spawner_node
  taskboard_spawner_node_manifest
  taskboard_spawner_node_default_config
)

# Move manifest files to expected subfolder for Dockerfile.service
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
  ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
  ${CMAKE_CURRENT_BINARY_DIR}/taskboard_spawner_node_protos.desc
  DESTINATION share/${PROJECT_NAME}/taskboard_spawner_node
)

ament_package()
