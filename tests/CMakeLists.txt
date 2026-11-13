add_library(g1d_doctest_main STATIC test_main.cpp)
target_include_directories(g1d_doctest_main PUBLIC ${PROJECT_SOURCE_DIR}/third_party/doctest)

function(g1d_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE g1d_doctest_main)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

g1d_add_test(test_common test_common.cpp)
target_link_libraries(test_common PRIVATE g1d_common)

g1d_add_test(test_lio_units test_lio_units.cpp)
target_link_libraries(test_lio_units PRIVATE g1d_lio)

g1d_add_test(test_lio_sim test_lio_sim.cpp)
target_link_libraries(test_lio_sim PRIVATE g1d_lio g1d_sim)
set_tests_properties(test_lio_sim PROPERTIES TIMEOUT 900)
