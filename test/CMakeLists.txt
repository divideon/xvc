set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib")
add_subdirectory(googletest)

# xvc_test_lib
file(GLOB SRC_TEST xvc_test/*.cc)
add_library(xvc_test_lib STATIC ${SRC_TEST})
target_include_directories(xvc_test_lib PUBLIC . ../src)
target_link_libraries(xvc_test_lib PRIVATE xvc_enc_lib xvc_dec_lib PUBLIC gtest)

# xvc_test
add_executable(xvc_test "${gtest_SOURCE_DIR}/src/gtest_main.cc")
target_link_libraries(xvc_test PRIVATE xvc_test_lib)
add_test(xvc_test xvc_test)

