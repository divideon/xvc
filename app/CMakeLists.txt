set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

SET(XVC_DEC_APP_SOURCES
    "xvc_dec_app/decoder_app.cc"
    "xvc_dec_app/decoder_app.h"
    "xvc_dec_app/main_dec.cc"
    "xvc_dec_app/y4m_writer.cc"
    "xvc_dec_app/y4m_writer.h")

set(XVC_ENC_APP_SOURCES
    "xvc_enc_app/encoder_app.cc"
    "xvc_enc_app/encoder_app.h"
    "xvc_enc_app/main_enc.cc"
    "xvc_enc_app/y4m_reader.cc"
    "xvc_enc_app/y4m_reader.h")

if(ENABLE_ASSERTIONS)
  add_definitions(-UNDEBUG)
  string(REGEX REPLACE "(^| )[/-]D *NDEBUG($| )" " " CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
endif()

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
add_executable(xvc_enc_app ${XVC_ENC_APP_SOURCES})
set_target_properties(xvc_enc_app PROPERTIES OUTPUT_NAME "xvcenc")
target_compile_options(xvc_enc_app PRIVATE ${cxx_flags})
target_include_directories(xvc_enc_app PUBLIC . ../src)
target_link_libraries(xvc_enc_app LINK_PUBLIC xvc_enc_lib)

# xvc_dec_app
add_executable(xvc_dec_app ${XVC_DEC_APP_SOURCES})
set_target_properties(xvc_dec_app PROPERTIES OUTPUT_NAME "xvcdec")
target_compile_options(xvc_dec_app PRIVATE ${cxx_flags})
target_include_directories(xvc_dec_app PUBLIC . ../src)
target_link_libraries(xvc_dec_app LINK_PUBLIC xvc_dec_lib)

install(TARGETS xvc_enc_app xvc_dec_app DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")

