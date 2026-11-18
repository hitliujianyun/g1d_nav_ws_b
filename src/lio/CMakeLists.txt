set(FASTLIO ${PROJECT_SOURCE_DIR}/third_party/fast_lio)

# FAST-LIO2 estimator (GPL-2.0, kept in its own library / directory).
add_library(g1d_fastlio STATIC
  ${FASTLIO}/src/laserMapping.cpp
  ${FASTLIO}/include/ikd-Tree/ikd_Tree.cpp
)
target_include_directories(g1d_fastlio PUBLIC
  ${FASTLIO}/include
  ${PROJECT_SOURCE_DIR}/src/pcl_lite/include
)
# IKFoM's MTK_BUILD_MANIFOLD needs Boost.Preprocessor only (header-only submodule).
target_include_directories(g1d_fastlio SYSTEM PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/boost_preprocessor/include)
target_include_directories(g1d_fastlio PRIVATE ${FASTLIO}/src)
include(ProcessorCount)
ProcessorCount(G1D_NPROC)
if(G1D_NPROC GREATER 4)
  set(G1D_MP_PROC 4)
elseif(G1D_NPROC GREATER 1)
  set(G1D_MP_PROC ${G1D_NPROC})
else()
  set(G1D_MP_PROC 1)
endif()
target_compile_definitions(g1d_fastlio PRIVATE ROOT_DIR="./" MP_PROC_NUM=${G1D_MP_PROC})
if(OpenMP_CXX_FOUND)
  target_compile_definitions(g1d_fastlio PRIVATE MP_EN)
endif()
target_link_libraries(g1d_fastlio PUBLIC g1d_eigen g1d_openmp Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(g1d_fastlio PRIVATE
    -Wno-unused-variable -Wno-unused-but-set-variable -Wno-reorder -Wno-maybe-uninitialized
    -Wno-deprecated-declarations -Wno-class-memaccess -Wno-unused-function -Wno-misleading-indentation
    -Wno-comment -Wno-return-type -Wno-unused-local-typedefs -Wno-parentheses -Wno-narrowing
    -Wno-overflow -Wno-ignored-qualifiers -Wno-pessimizing-move -Wno-catch-value)
endif()

# M-detector moving-point detection (GPL-2.0, de-ROSed port in its own library / directory).
set(MDET ${PROJECT_SOURCE_DIR}/third_party/m_detector)
add_library(g1d_mdetector STATIC ${MDET}/src/DynObjFilter.cpp)
target_include_directories(g1d_mdetector PUBLIC
  ${MDET}/include
  ${PROJECT_SOURCE_DIR}/src/pcl_lite/include
)
target_link_libraries(g1d_mdetector PUBLIC g1d_eigen g1d_openmp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(g1d_mdetector PRIVATE
    -Wno-unused-variable -Wno-unused-but-set-variable -Wno-reorder -Wno-maybe-uninitialized
    -Wno-sign-compare -Wno-unused-function -Wno-misleading-indentation -Wno-switch-unreachable
    -Wno-parentheses -Wno-narrowing -Wno-unused-parameter -Wno-comment)
endif()

add_library(g1d_lio STATIC
  src/livox_frame.cpp
  src/preprocess.cpp
  src/lio_health.cpp
  src/dynamic_detector.cpp
  src/lio_estimator.cpp
)
target_include_directories(g1d_lio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(g1d_lio PUBLIC g1d_fastlio g1d_common PRIVATE g1d_mdetector)
