set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib")
add_subdirectory(googletest EXCLUDE_FROM_ALL)

set(XVC_TEST_SOURCES
    "xvc_test/all_intra_test.cc"
    "xvc_test/checksum_enc_dec_test.cc"
    "xvc_test/decoder_api_test.cc"
    "xvc_test/decoder_helper.h"
    "xvc_test/decoder_resample_test.cc"
    "xvc_test/decoder_scalability_test.cc"
    "xvc_test/encode_decode_test.cc"
    "xvc_test/encoder_api_test.cc"
    "xvc_test/encoder_helper.h"
    "xvc_test/hls_test.cc"
    "xvc_test/residual_coding_test.cc"
    "xvc_test/resolution_test.cc"
    "xvc_test/restrictions_test.cc"
    "xvc_test/simd_test.cc"
    "xvc_test/transform_test.cc"
    "xvc_test/yuv_helper.cc"
    "xvc_test/yuv_helper.h")

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

if(BUILD_TESTS_LIBS)
  # xvc_test_lib
  add_library(xvc_test_lib STATIC ${XVC_TEST_SOURCES})
  target_compile_options(xvc_test_lib PRIVATE ${cxx_flags})
  target_include_directories(xvc_test_lib PUBLIC . ../src)
  target_link_libraries(xvc_test_lib PRIVATE xvc_enc_lib xvc_dec_lib PUBLIC gtest)
endif()

# xvc_test
add_executable(xvc_test ${XVC_TEST_SOURCES})
target_compile_options(xvc_test PRIVATE ${cxx_flags})
target_include_directories(xvc_test PUBLIC . ../src)
target_link_libraries(xvc_test LINK_PUBLIC xvc_enc_lib xvc_dec_lib gtest_main)
add_test(xvc_test xvc_test)

