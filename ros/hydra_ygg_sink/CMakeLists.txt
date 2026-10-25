cmake_minimum_required(VERSION 3.22)
project(hydra_ygg_sink LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hydra REQUIRED)
find_package(config_utilities REQUIRED)

include(FetchContent)
FetchContent_Declare(Corrosion GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git GIT_TAG v0.5.1)
FetchContent_MakeAvailable(Corrosion)

# The Rust workspace sits two levels up (hydra-ygg/Cargo.toml).
corrosion_import_crate(MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../Cargo.toml CRATES ygg-bridge CRATE_TYPES staticlib)
corrosion_add_cxxbridge(ygg_bridge_cxx CRATE ygg_bridge FILES lib.rs)
set_target_properties(ygg_bridge_cxx PROPERTIES POSITION_INDEPENDENT_CODE ON)
corrosion_set_env_vars(ygg_bridge RUSTFLAGS=-Crelocation-model=pic)

add_library(${PROJECT_NAME} SHARED src/yggdrasil_sink.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC hydra::hydra PRIVATE ygg_bridge_cxx)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
