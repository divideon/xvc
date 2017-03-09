set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Experimental mode (internal)
set(EXPERIMENTAL "" CACHE INTERNAL "Enable experimental codec mode (internal use only)")
# Restrictions control (internal)
set(RESTRICTION_DEFINES "" CACHE INTERNAL "Restriction flag control (internal use only)")

if(EXPERIMENTAL)
  add_definitions(-DHM_STRICT=0)
endif()

if(MSVC)
  set(cxx_base_flags /GS /W4 /WX /Za)
  set(cxx_exception_flags "-EHsc")
  set(cxx_strict_flags /we4388 /we4389 /wd4100 /wd4127)
  if (CODE_ANALYZE)
    set(cxx_base_flags ${cxx_base_flags} /analyze /wd6240)
  endif()
elseif (CMAKE_COMPILER_IS_GNUCXX)
  set(cxx_base_flags -Wall -Wshadow)
  set(cxx_exception_flags "-fexceptions")
  set(cxx_strict_flags -Werror -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()
set(cxx_default ${cxx_base_flags} ${cxx_exception_flags})
set(cxx_strict ${cxx_default} ${cxx_strict_flags})

# xvc_common_lib
file (GLOB SRC_COMMON_LIB xvc_common_lib/*.cc)
add_library (xvc_common_lib OBJECT ${SRC_COMMON_LIB})
target_compile_options(xvc_common_lib PRIVATE ${cxx_strict})
target_include_directories (xvc_common_lib PUBLIC .)

# xvc_enc_lib
file (GLOB SRC_ENC_LIB xvc_enc_lib/*.cc)
add_library (xvc_enc_lib ${SRC_ENC_LIB} $<TARGET_OBJECTS:xvc_common_lib>)
target_compile_options(xvc_enc_lib PRIVATE ${cxx_strict})
target_include_directories (xvc_enc_lib PUBLIC .)

# xvc_dec_lib
file (GLOB SRC_DEC_LIB xvc_dec_lib/*.cc)
add_library (xvc_dec_lib ${SRC_DEC_LIB} $<TARGET_OBJECTS:xvc_common_lib>)
target_compile_options(xvc_dec_lib PRIVATE ${cxx_strict})
target_include_directories (xvc_dec_lib PUBLIC .)

if(RESTRICTION_DEFINES)
  set_source_files_properties(xvc_common_lib/restrictions.cc PROPERTIES COMPILE_FLAGS ${RESTRICTION_DEFINES})
endif()

