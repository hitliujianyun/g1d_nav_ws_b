set(TP ${PROJECT_SOURCE_DIR}/third_party)

# ---------------------------------------------------------------------------- ros_shim
add_library(g1d_ros_shim INTERFACE)
target_include_directories(g1d_ros_shim INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/ros_shim/include)

# ---------------------------------------------------------------------------- vendored Nav2
set(NAV2 ${TP}/nav2)
set(MPPI ${NAV2}/nav2_mppi_controller/src)
add_library(g1d_nav2 STATIC
  ${NAV2}/nav2_costmap_2d/src/costmap_2d.cpp
  ${NAV2}/nav2_costmap_2d/src/footprint.cpp
  ${NAV2}/nav2_costmap_2d/src/footprint_collision_checker.cpp
  ${NAV2}/nav2_costmap_2d/src/costmap_math.cpp
  ${NAV2}/nav2_util/src/array_parser.cpp
  ${NAV2}/nav2_voxel_grid/src/voxel_grid.cpp
  ${NAV2}/nav2_navfn_planner/src/navfn.cpp
  ${MPPI}/optimizer.cpp
  ${MPPI}/critic_manager.cpp
  ${MPPI}/noise_generator.cpp
  ${MPPI}/motion_models.cpp
  ${MPPI}/parameters_handler.cpp
  ${MPPI}/trajectory_validators/optimal_trajectory_validator.cpp
  ${MPPI}/critics/constraint_critic.cpp
  ${MPPI}/critics/cost_critic.cpp
  ${MPPI}/critics/goal_angle_critic.cpp
  ${MPPI}/critics/goal_critic.cpp
  ${MPPI}/critics/obstacles_critic.cpp
  ${MPPI}/critics/path_align_critic.cpp
  ${MPPI}/critics/path_angle_critic.cpp
  ${MPPI}/critics/path_follow_critic.cpp
  ${MPPI}/critics/prefer_forward_critic.cpp
  ${MPPI}/critics/twirling_critic.cpp
  ${MPPI}/critics/velocity_deadband_critic.cpp
)
# Shim headers must win over nothing else; vendored package include dirs hold the real code.
target_include_directories(g1d_nav2 PUBLIC
  ${NAV2}/nav2_costmap_2d/include
  ${NAV2}/nav2_util/include
  ${NAV2}/nav2_core/include
  ${NAV2}/nav2_voxel_grid/include
  ${NAV2}/nav2_navfn_planner/include
  ${NAV2}/nav2_mppi_controller/include
)
target_link_libraries(g1d_nav2 PUBLIC g1d_ros_shim g1d_eigen g1d_openmp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(g1d_nav2 PRIVATE -Wno-reorder -Wno-unused-variable -Wno-unknown-pragmas)
endif()

add_subdirectory(common)
add_subdirectory(lio)
add_subdirectory(nav)
add_subdirectory(sim)
