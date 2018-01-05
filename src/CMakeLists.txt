set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(XVC_COMMON_LIB_SOURCES
    "xvc_common_lib/cabac.cc"
    "xvc_common_lib/cabac.h"
    "xvc_common_lib/checksum.cc"
    "xvc_common_lib/checksum.h"
    "xvc_common_lib/coding_unit.cc"
    "xvc_common_lib/coding_unit.h"
    "xvc_common_lib/common.cc"
    "xvc_common_lib/common.h"
    "xvc_common_lib/context_model.cc"
    "xvc_common_lib/context_model.h"
    "xvc_common_lib/cu_types.h"
    "xvc_common_lib/deblocking_filter.cc"
    "xvc_common_lib/deblocking_filter.h"
    "xvc_common_lib/inter_prediction.cc"
    "xvc_common_lib/inter_prediction.h"
    "xvc_common_lib/intra_prediction.cc"
    "xvc_common_lib/intra_prediction.h"
    "xvc_common_lib/picture_data.cc"
    "xvc_common_lib/picture_data.h"
    "xvc_common_lib/picture_types.h"
    "xvc_common_lib/quantize.cc"
    "xvc_common_lib/quantize.h"
    "xvc_common_lib/reference_list_sorter.h"
    "xvc_common_lib/reference_picture_lists.cc"
    "xvc_common_lib/reference_picture_lists.h"
    "xvc_common_lib/resample.cc"
    "xvc_common_lib/resample.h"
    "xvc_common_lib/restrictions.cc"
    "xvc_common_lib/restrictions.h"
    "xvc_common_lib/sample_buffer.h"
    "xvc_common_lib/segment_header.cc"
    "xvc_common_lib/segment_header.h"
    "xvc_common_lib/simd_cpu.cc"
    "xvc_common_lib/simd_cpu.h"
    "xvc_common_lib/simd_functions.cc"
    "xvc_common_lib/simd_functions.h"
    "xvc_common_lib/transform.cc"
    "xvc_common_lib/transform.h"
    "xvc_common_lib/utils.cc"
    "xvc_common_lib/utils.h"
    "xvc_common_lib/utils_md5.cc"
    "xvc_common_lib/utils_md5.h"
    "xvc_common_lib/yuv_pic.cc"
    "xvc_common_lib/yuv_pic.h")

set(XVC_COMMON_LIB_SIMD_SOURCES
    "xvc_common_lib/simd/inter_prediction_simd.cc"
    "xvc_common_lib/simd/inter_prediction_simd.h")

set(XVC_DEC_LIB_SOURCES
    "xvc_dec_lib/bit_reader.cc"
    "xvc_dec_lib/bit_reader.h"
    "xvc_dec_lib/cu_decoder.cc"
    "xvc_dec_lib/cu_decoder.h"
    "xvc_dec_lib/cu_reader.cc"
    "xvc_dec_lib/cu_reader.h"
    "xvc_dec_lib/decoder.cc"
    "xvc_dec_lib/decoder.h"
    "xvc_dec_lib/entropy_decoder.cc"
    "xvc_dec_lib/entropy_decoder.h"
    "xvc_dec_lib/picture_decoder.cc"
    "xvc_dec_lib/picture_decoder.h"
    "xvc_dec_lib/segment_header_reader.cc"
    "xvc_dec_lib/segment_header_reader.h"
    "xvc_dec_lib/syntax_reader.cc"
    "xvc_dec_lib/syntax_reader.h"
    "xvc_dec_lib/thread_decoder.cc"
    "xvc_dec_lib/thread_decoder.h"
    "xvc_dec_lib/xvcdec.cc"
    "xvc_dec_lib/xvcdec.h")

set(XVC_ENC_LIB_SOURCES
    "xvc_enc_lib/bit_writer.cc"
    "xvc_enc_lib/bit_writer.h"
    "xvc_enc_lib/cu_cache.cc"
    "xvc_enc_lib/cu_cache.h"
    "xvc_enc_lib/cu_encoder.cc"
    "xvc_enc_lib/cu_encoder.h"
    "xvc_enc_lib/cu_writer.cc"
    "xvc_enc_lib/cu_writer.h"
    "xvc_enc_lib/encoder.cc"
    "xvc_enc_lib/encoder.h"
    "xvc_enc_lib/encoder_settings.h"
    "xvc_enc_lib/entropy_encoder.cc"
    "xvc_enc_lib/entropy_encoder.h"
    "xvc_enc_lib/inter_search.cc"
    "xvc_enc_lib/inter_search.h"
    "xvc_enc_lib/inter_tz_search.cc"
    "xvc_enc_lib/inter_tz_search.h"
    "xvc_enc_lib/intra_search.cc"
    "xvc_enc_lib/intra_search.h"
    "xvc_enc_lib/picture_encoder.cc"
    "xvc_enc_lib/picture_encoder.h"
    "xvc_enc_lib/rdo_quant.cc"
    "xvc_enc_lib/rdo_quant.h"
    "xvc_enc_lib/sample_metric.cc"
    "xvc_enc_lib/sample_metric.h"
    "xvc_enc_lib/segment_header_writer.cc"
    "xvc_enc_lib/segment_header_writer.h"
    "xvc_enc_lib/syntax_writer.cc"
    "xvc_enc_lib/syntax_writer.h"
    "xvc_enc_lib/transform_encoder.cc"
    "xvc_enc_lib/transform_encoder.h"
    "xvc_enc_lib/xvcenc.cc"
    "xvc_enc_lib/xvcenc.h")

# Restrictions control (internal)
set(RESTRICTION_DEFINES "" CACHE INTERNAL "Restriction flag control (internal use only)")

if(ENABLE_ASSERTIONS)
  add_definitions(-UNDEBUG)
endif()

if(ENABLE_ASSEMBLY)
  # Arch define is only used by SIMD code otherwise add new define for assembly
  message(STATUS "xvc target architecture: ${XVC_TARGET_ARCH}")
  string(TOUPPER "XVC_ARCH_${XVC_TARGET_ARCH}" XVC_ARCH_DEFINE)
  add_definitions("-D${XVC_ARCH_DEFINE}=1")
  if(XVC_HAVE_NEON)
    message(STATUS "xvc with NEON support")
    add_definitions("-DXVC_HAVE_NEON=1")
  endif()
else()
  message("-- xvc compile without assembly support")
endif()

set(cxx_simd_flags "")
if(MSVC)
  set(cxx_base_flags /GS /W4 /WX /Za)
  set(cxx_exception_flags "-EHsc")
  set(cxx_strict_flags /we4388 /we4389 /wd4100 /wd4127)
  if(CODE_ANALYZE)
    set(cxx_base_flags ${cxx_base_flags} /analyze /wd6240 /wd6326)
  endif()
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(cxx_base_flags -Wall -Wshadow)
  set(cxx_exception_flags "-fexceptions")
  set(cxx_strict_flags -Werror -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
  if(XVC_HAVE_NEON)
    set(cxx_simd_flags -mfpu=neon)
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
    message(FATAL_ERROR "thread_local requires gcc >= 4.8")
    set(cxx_strict_flags ${cxx_strict_flags} -Wno-error=missing-braces)
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(cxx_base_flags -Wshadow)
  set(cxx_exception_flags "-fexceptions")
  set(cxx_strict_flags -Werror)
  if(XVC_HAVE_NEON)
    set(cxx_simd_flags -mfpu=neon -Wno-error=unused-command-line-argument)
  endif()
endif()
set(cxx_default ${cxx_base_flags} ${cxx_exception_flags})
set(cxx_strict ${cxx_strict_flags})
set(linker_flags "")

if(SANITIZE_BUILD)
  set(cxx_default ${cxx_default} -fsanitize=${SANITIZE_BUILD} -fno-omit-frame-pointer)
  set(linker_flags ${linker_flags} "-fsanitize=${SANITIZE_BUILD}")
endif()

# xvc_common_lib
add_library (xvc_common_lib OBJECT ${XVC_COMMON_LIB_SOURCES})
target_compile_options(xvc_common_lib PRIVATE ${cxx_default} ${cxx_strict})
target_include_directories(xvc_common_lib PUBLIC .)
set(xvc_common_lib_extra "")

if(ENABLE_ASSEMBLY)
  # xvc_common_lib_simd
  add_library (xvc_common_lib_simd OBJECT ${XVC_COMMON_LIB_SIMD_SOURCES})
  target_compile_options(xvc_common_lib_simd PRIVATE ${cxx_default} ${cxx_strict} ${cxx_simd_flags})
  target_include_directories (xvc_common_lib_simd PUBLIC .)
  set(xvc_common_lib_extra ${xvc_common_lib_extra} $<TARGET_OBJECTS:xvc_common_lib_simd>)
endif()

# xvc_enc_lib
add_library(xvc_enc_lib ${XVC_ENC_LIB_SOURCES} $<TARGET_OBJECTS:xvc_common_lib> ${xvc_common_lib_extra})
set_target_properties(xvc_enc_lib PROPERTIES OUTPUT_NAME "xvcenc")
target_compile_options(xvc_enc_lib PRIVATE ${cxx_default} ${cxx_strict})
target_include_directories (xvc_enc_lib PUBLIC .)
target_link_libraries(xvc_enc_lib INTERFACE ${linker_flags})

# xvc_dec_lib
add_library(xvc_dec_lib ${XVC_DEC_LIB_SOURCES} $<TARGET_OBJECTS:xvc_common_lib> ${xvc_common_lib_extra})
set_target_properties(xvc_dec_lib PROPERTIES OUTPUT_NAME "xvcdec")
target_compile_options(xvc_dec_lib PRIVATE ${cxx_default} ${cxx_strict})
target_include_directories (xvc_dec_lib PUBLIC .)
target_link_libraries(xvc_dec_lib INTERFACE ${linker_flags} PUBLIC Threads::Threads)

if(RESTRICTION_DEFINES)
  set_source_files_properties(xvc_common_lib/restrictions.cc PROPERTIES COMPILE_FLAGS ${RESTRICTION_DEFINES})
endif()

install(FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/xvc_enc_lib/xvcenc.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/xvc_dec_lib/xvcdec.h"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
install(TARGETS xvc_enc_lib xvc_dec_lib DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")

