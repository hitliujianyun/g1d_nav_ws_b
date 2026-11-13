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
target_link_libraries(g1d_navcore PUBLIC
  "$<LINK_LIBRARY:WHOLE_ARCHIVE,g1d_nav2>"
  g1d_common)
