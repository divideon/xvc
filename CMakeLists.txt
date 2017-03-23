cmake_minimum_required (VERSION 3.1)
if(POLICY CMP0056) # added in CMake 3.2
  cmake_policy(SET CMP0056 NEW)
endif()

project (xvc CXX C)

# Project options
option(HIGH_BITDEPTH "Store pixel samples as 16bit values." ON)
option(BUILD_SHARED_LIBS "Build shared instead of static libraries." OFF)
option(BUILD_TESTS "Build all test code" ON)
option(ENABLE_ASSERTIONS "Compile with assertions" ON)
option(CODE_ANALYZE "Compile with code analyzer (MSVC)" OFF)
if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  set(SANITIZE_BUILD "" CACHE STRING "Compile with sanitizer enabled (GCC/clang)")
endif()

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

