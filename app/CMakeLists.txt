set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# xvc_enc_app
file(GLOB SRC_ENC_APP xvc_enc_app/*.cc xvc_enc_app/*.h)
add_executable(xvc_enc_app ${SRC_ENC_APP})
set_target_properties(xvc_enc_app PROPERTIES OUTPUT_NAME "xvcenc")
target_compile_options(xvc_enc_app PRIVATE ${cxx_flags})
target_include_directories(xvc_enc_app PUBLIC . ../src)
target_link_libraries(xvc_enc_app LINK_PUBLIC xvc_enc_lib)

# xvc_dec_app
file(GLOB SRC_DEC_APP xvc_dec_app/*.cc xvc_dec_app/*.h)
add_executable(xvc_dec_app ${SRC_DEC_APP})
set_target_properties(xvc_dec_app PROPERTIES OUTPUT_NAME "xvcdec")
target_compile_options(xvc_dec_app PRIVATE ${cxx_flags})
target_include_directories(xvc_dec_app PUBLIC . ../src)
target_link_libraries(xvc_dec_app LINK_PUBLIC xvc_dec_lib)

install(TARGETS xvc_enc_app xvc_dec_app DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")

