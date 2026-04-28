cmake_minimum_required(VERSION 3.8)
project(taskboard_spawning_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aic_flowstate_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(taskboard_state_server_main src/taskboard_state_server.cpp)
target_include_directories(taskboard_state_server_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(taskboard_state_server_main
  rclcpp::rclcpp
  ${aic_flowstate_interface_TARGETS}
  yaml-cpp
)

install(TARGETS
  taskboard_state_server_main
  DESTINATION lib/${PROJECT_NAME}
)

set(SERVICE_NAME "taskboard_state_server")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb"
  "${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb"
  "${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc"
  DESTINATION share/${PROJECT_NAME}/${SERVICE_NAME}
)

ament_package()
