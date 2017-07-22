/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_APP_ENCODER_APP_H_
#define XVC_ENC_APP_ENCODER_APP_H_

#include <stddef.h>

#include <chrono>
#include <fstream>
#include <limits>
#include <string>

#include "xvc_enc_lib/xvcenc.h"

namespace xvc_app {

class EncoderApp {
public:
  EncoderApp() : cli_(), xvc_api_(nullptr), params_(nullptr),
    encoder_(nullptr) {
  }
  ~EncoderApp();
  void ReadArguments(int argc, const char *argv[]);
  bool CheckParameters();
  void CreateAndConfigureApi();
  void PrintEncoderSettings();
  void MainEncoderLoop();
  void CloseStream();
  void PrintStatistics();

private:
  void PrintUsage();
  void PrintNalInfo(xvc_enc_nal_unit nal_unit);

  std::ifstream input_stream_;
  std::ofstream file_output_stream_;
  std::ofstream rec_stream_;
  std::streamoff start_skip_;
  std::streamoff picture_skip_;

  int picture_index_ = 0;
  size_t total_bytes_ = 0;

  // command line arguments
  struct {
    std::string input_filename;
    std::string output_filename;
    std::string rec_file;
    int width = 0;
    int height = 0;
    xvc_enc_chroma_format chroma_format = XVC_ENC_CHROMA_FORMAT_UNDEFINED;
    xvc_enc_color_matrix color_matrix = XVC_ENC_COLOR_MATRIX_UNDEFINED;
    int input_bitdepth = 0;
    int internal_bitdepth = 0;
    double framerate = 0;
    int skip_pictures = 0;
    int temporal_subsample = -1;
    int max_num_pictures = -1;
    int sub_gop_length = -1;
    int max_keypic_distance = -1;
    int closed_gop = -1;
    int num_ref_pics = -1;
    int restricted_mode = -1;
    int checksum_mode = -1;
    int chroma_qp_offset_table = -1;
    int chroma_qp_offset_u = std::numeric_limits<int>::min();
    int chroma_qp_offset_v = std::numeric_limits<int>::min();
    int deblock = -1;
    int beta_offset = std::numeric_limits<int>::min();
    int tc_offset = std::numeric_limits<int>::min();
    int qp = -1;
    int flat_lambda = -1;
    int speed_mode = -1;
    int tune_mode = -1;
    int simd_mask = -1;
    std::string explicit_encoder_settings;
    int verbose = 0;
  } cli_;

  const xvc_encoder_api *xvc_api_;
  xvc_encoder_parameters *params_;
  xvc_encoder *encoder_;
  xvc_enc_pic_buffer rec_pic_buffer_ = { 0, 0 };

  std::chrono::time_point<std::chrono::steady_clock> start_;
  std::chrono::time_point<std::chrono::steady_clock> end_;
};

}  // namespace xvc_app

#endif  // XVC_ENC_APP_ENCODER_APP_H_
