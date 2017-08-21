cmake_minimum_required (VERSION 3.1)
if(POLICY CMP0056) # added in CMake 3.2
  cmake_policy(SET CMP0056 NEW)
endif()

project (xvc CXX C)

find_package(Threads REQUIRED)

# Project options
option(HIGH_BITDEPTH "Store pixel samples as 16bit values." ON)
option(BUILD_SHARED_LIBS "Build shared instead of static libraries." OFF)
option(BUILD_TESTS "Build all test code" ON)
option(BUILD_TESTS_LIBS "Build all test code as library" OFF)
option(ENABLE_ASSEMBLY "Compile with assembly coded functions" ON)
option(ENABLE_ASSERTIONS "Compile with assertions" ON)
option(CODE_ANALYZE "Compile with code analyzer (MSVC)" OFF)
if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  set(SANITIZE_BUILD "" CACHE STRING "Compile with sanitizer enabled (GCC/clang)")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Target arch detection
set(XVC_HAVE_NEON FALSE)
if(NOT XVC_TARGET_ARCH)
  # Determine arch based on host cpu and generator used
  if("${CMAKE_GENERATOR}" MATCHES "^Visual Studio .+ ARM$")
    set(XVC_TARGET_ARCH "ARM")
    set(XVC_HAVE_NEON TRUE)
  elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "AMD64" OR
    "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" OR
    "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i386" OR
    "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86")
    set(XVC_TARGET_ARCH "x86")
  elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "armv7")
    set(XVC_TARGET_ARCH "ARM")
    set(XVC_HAVE_NEON TRUE)
  else()
    message(FATAL_ERROR "Unexpected host cpu: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
else()
  # Extract architecture family from user specified arch
  if("${XVC_TARGET_ARCH}" STREQUAL "x86_64")
    set(XVC_TARGET_ARCH "x86")
  elseif("${XVC_TARGET_ARCH}" MATCHES "^arm")
    if ("${XVC_TARGET_ARCH}" MATCHES "^arm64")
      set(XVC_HAVE_NEON TRUE)
    elseif ("${XVC_TARGET_ARCH}" MATCHES "^armv7|^arm.*-v7")
      set(XVC_HAVE_NEON TRUE)
    endif()
    set(XVC_TARGET_ARCH "ARM")
  elseif("${XVC_TARGET_ARCH}" MATCHES "^mips")
    set(XVC_TARGET_ARCH "MIPS")
  endif()
endif()
if(NOT "${XVC_TARGET_ARCH}" MATCHES "^(x86|ARM|MIPS)$")
  message(FATAL_ERROR "Unexpected target architecture: ${XVC_TARGET_ARCH}")
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

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/xvc.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/xvc.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/xvc.pc"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")

