cmake_minimum_required(VERSION 3.8)
project(local_map_generator)

find_package(ament_cmake REQUIRED)

add_custom_target(rust_build ALL
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust/swath_processing_node
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust/map_generation_node
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/rust/swath_processing_node/target/release/swath_processing_node
  ${CMAKE_CURRENT_SOURCE_DIR}/rust/map_generation_node/target/release/map_generation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()