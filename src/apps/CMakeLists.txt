# Robot executables (Linux + unitree_sdk2 only).
if(NOT G1D_WITH_UNITREE)
  message(STATUS "g1d: G1D_WITH_UNITREE=OFF -> g1d_lio / g1d_nav executables are not built")
  return()
endif()
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "G1D_WITH_UNITREE requires Linux (unitree_sdk2 ships Linux libraries only)")
endif()

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/unitree_sdk2 ${CMAKE_BINARY_DIR}/unitree_sdk2 EXCLUDE_FROM_ALL)

add_executable(g1d_nav g1d_nav_main.cpp)
target_include_directories(g1d_nav PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g1d_nav PRIVATE g1d_navcore unitree_sdk2)

add_executable(g1d_lio g1d_lio_main.cpp)
target_include_directories(g1d_lio PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g1d_lio PRIVATE g1d_lio unitree_sdk2)

if(G1D_WITH_LIVOX)
  add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/Livox-SDK2 ${CMAKE_BINARY_DIR}/livox_sdk2 EXCLUDE_FROM_ALL)
  target_sources(g1d_lio PRIVATE livox_sdk_source.cpp)
  target_include_directories(g1d_lio PRIVATE ${PROJECT_SOURCE_DIR}/third_party/Livox-SDK2/include)
  target_link_libraries(g1d_lio PRIVATE livox_lidar_sdk_static)
  target_compile_definitions(g1d_lio PRIVATE G1D_HAVE_LIVOX)
endif()

# Runtime CycloneDDS libs: binaries load libddsc.so.0 / libddscxx.so.0 (the SONAMEs). In the SDK
# those names are git symlinks, which break when the tree is copied from a Windows checkout, so
# the real files are installed under their SONAMEs into a private dir (the SDK's own install rules
# also write lib/, possibly with the broken links).
set(G1D_DDS_LIB_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR})
set_target_properties(g1d_nav g1d_lio PROPERTIES INSTALL_RPATH "$ORIGIN/../lib/g1d")
install(FILES ${G1D_DDS_LIB_DIR}/libddsc.so DESTINATION lib/g1d RENAME libddsc.so.0)
install(FILES ${G1D_DDS_LIB_DIR}/libddscxx.so DESTINATION lib/g1d RENAME libddscxx.so.0)
install(TARGETS g1d_nav g1d_lio RUNTIME DESTINATION bin)
