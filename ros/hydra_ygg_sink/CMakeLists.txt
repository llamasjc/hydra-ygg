cmake_minimum_required(VERSION 3.22)
project(hydra_ygg_sink LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hydra REQUIRED)
find_package(config_utilities REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hydra_ros REQUIRED)

include(FetchContent)
FetchContent_Declare(Corrosion GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git GIT_TAG v0.5.1)
FetchContent_MakeAvailable(Corrosion)

# The Rust workspace sits two levels up (hydra-ygg/Cargo.toml).
corrosion_import_crate(MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../Cargo.toml CRATES ygg-bridge CRATE_TYPES staticlib)
corrosion_add_cxxbridge(ygg_bridge_cxx CRATE ygg_bridge FILES lib.rs)
set_target_properties(ygg_bridge_cxx PROPERTIES POSITION_INDEPENDENT_CODE ON)
corrosion_set_env_vars(ygg_bridge RUSTFLAGS=-Crelocation-model=pic)

# Graph store for the patched Hydra backend (patches/hydra), loaded with LD_PRELOAD.
add_library(${PROJECT_NAME} SHARED src/yggdrasil_store.cpp src/yggdrasil_association.cpp
  src/yggdrasil_nn_backend.cpp src/yggdrasil_graph_backend.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC hydra::hydra PRIVATE ygg_bridge_cxx)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Standalone listener: mirrors /hydra/backend/dsg from outside Hydra (no patch needed).
add_executable(ygg_mirror_node src/ygg_mirror_node.cpp)
target_link_libraries(ygg_mirror_node ygg_bridge_cxx)
ament_target_dependencies(ygg_mirror_node rclcpp hydra_ros)
target_compile_features(ygg_mirror_node PRIVATE cxx_std_17)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# Checks the spark_dsg observer patch against Yggdrasil on a saved dsg.json.
add_executable(observer_check test/observer_check.cpp)
target_link_libraries(observer_check hydra::hydra ygg_bridge_cxx)
target_compile_features(observer_check PRIVATE cxx_std_17)

# Checks Yggdrasil as spark_dsg's structure store on a saved dsg.json (SPARK_DSG_BACKEND=both).
add_executable(backend_check test/backend_check.cpp)
target_link_libraries(backend_check hydra::hydra)
target_compile_features(backend_check PRIVATE cxx_std_17)

# Typed Yggdrasil store on a saved graph: exact round trip, and timing against the JSON path.
add_executable(typed_check test/typed_check.cpp)
target_link_libraries(typed_check hydra::hydra ygg_bridge_cxx)
target_compile_features(typed_check PRIVATE cxx_std_17)

# Station graph (station replay --out) to a Spark-DSG file for the static visualizer.
add_executable(station_to_dsg test/station_to_dsg.cpp)
target_link_libraries(station_to_dsg hydra::hydra)
target_compile_features(station_to_dsg PRIVATE cxx_std_17)

install(TARGETS ygg_mirror_node observer_check backend_check typed_check station_to_dsg RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_package()
