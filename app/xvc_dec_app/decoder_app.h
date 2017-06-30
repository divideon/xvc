/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_APP_DECODER_APP_H_
#define XVC_DEC_APP_DECODER_APP_H_

#include <stddef.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <string>

#include "xvc_dec_lib/xvcdec.h"

namespace xvc_app {

class DecoderApp {
public:
  DecoderApp() : xvc_api_(nullptr), params_(nullptr), decoder_(nullptr) {}
  ~DecoderApp();
  void ReadArguments(int argc, const char *argv[]);
  bool CheckParameters();
  void CreateAndConfigureApi();
  void PrintDecoderSettings();
  void MainDecoderLoop();
  void CloseStream();
  void PrintStatistics();
  int CheckConformance();

private:
  void PrintUsage();
  size_t ReadNextNalSize(std::istream *input);
  void PrintPictureInfo(xvc_dec_pic_stats pic_stats);
  std::ostream& GetLog() {
    return !log_to_stderr_ ? std::cout : std::cerr;
  }

  std::ifstream input_stream_;
  std::ofstream file_output_stream_;
  bool output_to_stdout_ = false;
  bool log_to_stderr_ = false;
  bool output_y4m_format_ = false;

  int num_pictures_decoded_ = 0;
  int segment_info_printed_ = 0;

  // command line arguments
  struct {
    std::string input_filename;
    std::string output_filename;
    int output_width = -1;
    int output_height = -1;
    xvc_dec_chroma_format output_chroma_format =
      XVC_DEC_CHROMA_FORMAT_UNDEFINED;
    xvc_dec_color_matrix output_color_matrix = XVC_DEC_COLOR_MATRIX_UNDEFINED;
    int output_bitdepth = -1;
    int max_framerate = -1;
    int simd_mask = -1;
    int threads = -1;
    int loop = -1;
    int verbose = 0;
  } cli_;

  const xvc_decoder_api *xvc_api_;
  xvc_decoder_parameters *params_;
  xvc_decoder *decoder_;

  std::chrono::time_point<std::chrono::steady_clock> start_;
  std::chrono::time_point<std::chrono::steady_clock> end_;
};

}  // namespace xvc_app

#endif  // XVC_DEC_APP_DECODER_APP_H_
