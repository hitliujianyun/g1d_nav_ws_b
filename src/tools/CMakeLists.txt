# Host-side tools (no DDS): offline closed-loop demo used for local verification.
add_executable(g1d_sim_demo g1d_sim_demo.cpp)
target_link_libraries(g1d_sim_demo PRIVATE g1d_sim)
