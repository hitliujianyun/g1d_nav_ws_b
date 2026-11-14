add_library(g1d_navcore STATIC
  src/body_kinematics.cpp
  src/state_estimator.cpp
  src/costmap/layers.cpp
  src/costmap/layered_costmap.cpp
  src/planner.cpp
  src/controller.cpp
  src/nav_core.cpp
)
target_include_directories(g1d_navcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# The vendored Nav2 objects self-register MPPI critics / motion models / validators at static
# initialisation; link the whole archive so the linker keeps those registrars.
option(G1D_LEGACY_WHOLE_ARCHIVE "Use -Wl,--whole-archive even on CMake >= 3.24 (tests the fallback)" OFF)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24 AND NOT G1D_LEGACY_WHOLE_ARCHIVE)
  target_link_libraries(g1d_navcore PUBLIC "$<LINK_LIBRARY:WHOLE_ARCHIVE,g1d_nav2>" g1d_common)
else()
  # CMake < 3.24 (Ubuntu 20.04 ships 3.16), GNU ld.
  target_link_libraries(g1d_navcore PUBLIC -Wl,--whole-archive g1d_nav2 -Wl,--no-whole-archive g1d_common)
endif()
