cmake_minimum_required(VERSION 3.24)
project(g1d_ws VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(G1D_BUILD_TESTS "Build unit / simulation tests" ON)
option(G1D_WITH_UNITREE "Build DDS executables against unitree_sdk2 (Linux only)" OFF)
option(G1D_WITH_LIVOX "Build Livox-SDK2 and the live Mid-360S source (Linux only)" OFF)
set(G1D_ARCH_FLAGS "" CACHE STRING "Extra arch flags, e.g. -mcpu=cortex-a78ae on Orin")

find_package(Threads REQUIRED)
find_package(OpenMP)

add_compile_definitions(_USE_MATH_DEFINES)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wno-unused-parameter -Wno-sign-compare ${G1D_ARCH_FLAGS})
endif()
if(MINGW)
  # Large translation units (FAST-LIO, MPPI) exceed the default COFF section limit.
  add_compile_options(-Wa,-mbig-obj)
endif()

# ---------------------------------------------------------------------------- third party
add_library(g1d_eigen INTERFACE)
target_include_directories(g1d_eigen SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
target_compile_definitions(g1d_eigen INTERFACE EIGEN_MPL2_ONLY)

# yaml-cpp 0.8.0 declares cmake_minimum_required(3.4), which CMake >= 4 rejects.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/yaml-cpp EXCLUDE_FROM_ALL)

add_library(g1d_openmp INTERFACE)
if(OpenMP_CXX_FOUND)
  target_link_libraries(g1d_openmp INTERFACE OpenMP::OpenMP_CXX)
endif()

add_subdirectory(src)

if(G1D_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
