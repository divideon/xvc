set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# xvc_enc_app
file (GLOB SRC_ENC_APP xvc_enc_app/*.cc)
add_executable (xvcenc ${SRC_ENC_APP})
target_include_directories (xvcenc PUBLIC . ../src)
target_link_libraries (xvcenc LINK_PUBLIC xvc_enc_lib)

# xvc_dec_app
file (GLOB SRC_DEC_APP xvc_dec_app/*.cc)
add_executable (xvcdec ${SRC_DEC_APP})
target_include_directories (xvcdec PUBLIC . ../src)
target_link_libraries (xvcdec LINK_PUBLIC xvc_dec_lib)

