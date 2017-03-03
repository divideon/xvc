set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib")
add_subdirectory(googletest)

# xvc_test
file (GLOB SRC_TEST xvc_test/*.cc)
add_executable (xvc_test ${SRC_TEST} ${GTEST_SRCS})
target_include_directories (xvc_test PUBLIC . ../src)
target_link_libraries (xvc_test LINK_PUBLIC xvc_enc_lib xvc_dec_lib gtest_main)
add_test(xvc_test xvc_test)

