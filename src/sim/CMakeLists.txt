# Simulation helpers for tests and the local demo (box world, Mid-360 / IMU / 2D scan, diff drive).
add_library(g1d_sim STATIC
  src/world.cpp
  src/sensors.cpp
  src/robot.cpp
)
target_include_directories(g1d_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(g1d_sim PUBLIC g1d_common g1d_lio)
