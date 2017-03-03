cmake_minimum_required (VERSION 3.1)
project (xvc CXX C)

# Project options
option(HIGH_BITDEPTH "Store pixel samples as 16bit values." OFF)
option(BUILD_SHARED_LIBS "Build shared instead of static libraries." OFF)
option(BUILD_TESTS "Build all test code" ON)
option(CODE_ANALYZE "Compile with code analyzer (MSVC)" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

if(HIGH_BITDEPTH)
    add_definitions(-DXVC_HIGH_BITDEPTH=1)
else()
    add_definitions(-DXVC_HIGH_BITDEPTH=0)
endif()

add_subdirectory (src)
add_subdirectory (app)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory (test)
endif()

