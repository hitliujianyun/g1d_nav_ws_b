add_library(g1d_common STATIC
  src/config.cpp
  src/time.cpp
)
target_include_directories(g1d_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party  # <nlohmann/json.hpp>
)
target_link_libraries(g1d_common PUBLIC g1d_eigen g1d_ros_shim yaml-cpp::yaml-cpp Threads::Threads)
