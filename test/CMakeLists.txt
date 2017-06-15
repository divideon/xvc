set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib")
add_subdirectory(googletest)

if(MSVC)
  set(cxx_flags /GS /W3 /WX /Za /EHsc)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(cxx_flags -Wall -Wshadow -Werror -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wno-sign-compare -fexceptions)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
    set(cxx_flags ${cxx_flags} -Wno-error=missing-braces)
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(cxx_flags -Wshadow -Werror -fexceptions)
endif()

file(GLOB SRC_TEST xvc_test/*.cc xvc_test/*.h)

if(BUILD_TESTS_LIBS)
  # xvc_test_lib
  add_library(xvc_test_lib STATIC ${SRC_TEST})
  target_compile_options(xvc_test_lib PRIVATE ${cxx_flags})
  target_include_directories(xvc_test_lib PUBLIC . ../src)
  target_link_libraries(xvc_test_lib PRIVATE xvc_enc_lib xvc_dec_lib PUBLIC gtest)
endif()

# xvc_test
add_executable(xvc_test ${SRC_TEST})
target_compile_options(xvc_test PRIVATE ${cxx_flags})
target_include_directories(xvc_test PUBLIC . ../src)
target_link_libraries(xvc_test LINK_PUBLIC xvc_enc_lib xvc_dec_lib gtest_main)
add_test(xvc_test xvc_test)

