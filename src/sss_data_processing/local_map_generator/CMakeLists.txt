cmake_minimum_required(VERSION 3.8)
project(local_map_generator)

find_package(ament_cmake REQUIRED)

set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/rust_target)

add_custom_target(rust_build ALL
  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR}
          cargo build --release --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/swath_processing_node/Cargo.toml

  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR}
          cargo build --release --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/map_generation_node/Cargo.toml

  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR}
          cargo build --release --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/feature_extraction_node/Cargo.toml
)

install(PROGRAMS
  ${RUST_TARGET_DIR}/release/swath_processing_node
  ${RUST_TARGET_DIR}/release/map_generation_node
  ${RUST_TARGET_DIR}/release/feature_extraction_node
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