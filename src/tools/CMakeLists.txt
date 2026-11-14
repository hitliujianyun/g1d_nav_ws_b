# Host-side tools (no DDS): offline closed-loop demo and the body-chain printer.
add_executable(g1d_sim_demo g1d_sim_demo.cpp)
target_link_libraries(g1d_sim_demo PRIVATE g1d_sim)

add_executable(g1d_fk g1d_fk.cpp)
target_link_libraries(g1d_fk PRIVATE g1d_navcore)

install(TARGETS g1d_fk g1d_sim_demo RUNTIME DESTINATION bin)
