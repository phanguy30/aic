# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME get_taskboard_state_skill
  SOURCES src/get_taskboard_state_skill.proto
  TARGET get_taskboard_state_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/get_taskboard_state_skill.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET get_taskboard_state_skill_config
  SKILL_NAME get_taskboard_state_skill
  MANIFEST src/get_taskboard_state_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/get_taskboard_state_skill.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/get_taskboard_state_skill_config.pbbin"
)

install(
  FILES
    src/get_taskboard_state_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/get_taskboard_state_skill_config.pbbin"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/get_taskboard_state_skill.manifest.textproto
  HEADER_FILES src/get_taskboard_state_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/get_taskboard_state_skill_main.cc"
)

add_executable(get_taskboard_state_skill_main
  src/get_taskboard_state_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/get_taskboard_state_skill_main.cc
)

target_include_directories(get_taskboard_state_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
  /usr/local/google/home/kbalasundar/aic_ws/install/aic_flowstate_interface/include/aic_flowstate_interface/
)

target_link_libraries(get_taskboard_state_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  get_taskboard_state_skill_protos
  rclcpp::rclcpp
  ${aic_flowstate_interface_TARGETS}
  /usr/local/google/home/kbalasundar/aic_ws/install/aic_flowstate_interface/lib/libaic_flowstate_interface__rosidl_typesupport_cpp.so
)

add_dependencies(get_taskboard_state_skill_main get_taskboard_state_skill_config)
add_dependencies(get_taskboard_state_skill_main get_taskboard_state_skill_protos)

install(
  TARGETS get_taskboard_state_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
