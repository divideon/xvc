/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_app/decoder_app.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "xvc_dec_app/y4m_writer.h"

namespace xvc_app {

DecoderApp::~DecoderApp() {
  if (decoder_) {
    xvc_api_->decoder_destroy(decoder_);
    decoder_ = nullptr;
  }
  if (params_) {
    xvc_api_->parameters_destroy(params_);
    params_ = nullptr;
  }
}

void DecoderApp::ReadArguments(int argc, const char *argv[]) {
  if (argc <= 1) {
    PrintUsage();
    std::exit(0);
  }

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-h") {
      PrintUsage();
      std::exit(0);
    } else if (i == argc - 1) {
      std::cerr << "Error: Invalid argument / Missing value: " << arg <<
        std::endl;
      PrintUsage();
      std::exit(1);
    } else if (arg == "-bitstream-file") {
      cli_.input_filename = argv[++i];
    } else if (arg == "-output-file") {
      cli_.output_filename = argv[++i];
    } else if (arg == "-output-width") {
      std::stringstream(argv[++i]) >> cli_.output_width;
    } else if (arg == "-output-height") {
      std::stringstream(argv[++i]) >> cli_.output_height;
    } else if (arg == "-output-chroma-format") {
      int tmp;
      std::stringstream(argv[++i]) >> tmp;
      cli_.output_chroma_format = static_cast<xvc_dec_chroma_format>(tmp);
    } else if (arg == "-output-color-matrix") {
      int tmp;
      std::stringstream(argv[++i]) >> tmp;
      cli_.output_color_matrix = static_cast<xvc_dec_color_matrix>(tmp);
    } else if (arg == "-output-bitdepth") {
      std::stringstream(argv[++i]) >> cli_.output_bitdepth;
    } else if (arg == "-max-framerate") {
      std::stringstream(argv[++i]) >> cli_.max_framerate;
    } else if (arg == "-simd-mask") {
      std::stringstream(argv[++i]) >> cli_.simd_mask;
    } else if (arg == "-threads") {
      std::stringstream(argv[++i]) >> cli_.threads;
    } else if (arg == "-loop") {
      std::stringstream(argv[++i]) >> cli_.loop;
    } else if (arg == "-verbose") {
      std::stringstream(argv[++i]) >> cli_.verbose;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      PrintUsage();
      std::exit(1);
    }
  }
}

bool DecoderApp::CheckParameters() {
  if (cli_.input_filename.empty()) {
    std::cerr << "Error: Missing bitstream file argument" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  input_stream_.open(cli_.input_filename, std::ios_base::binary);
  if (!input_stream_) {
    std::cerr << "Failed to open bitstream file: "
      << cli_.input_filename << std::endl;
    std::exit(1);
  }

  if (!cli_.output_filename.empty()) {
    if (cli_.output_filename == "-") {
      log_to_stderr_ = 1;
      output_to_stdout_ = true;
      output_y4m_format_ = true;
    } else {
      std::string filename_suffix =
        cli_.output_filename.substr(
          std::max(0, static_cast<int>(cli_.output_filename.size()) - 4));
      if (filename_suffix == ".y4m") {
        output_y4m_format_ = true;
      }
      file_output_stream_.open(cli_.output_filename, std::ios_base::binary);
      if (!file_output_stream_) {
        std::cerr << "Failed to open output file for writing: "
          << cli_.output_filename << std::endl;
        std::exit(1);
      }
    }
  }

  return true;
}

void DecoderApp::CreateAndConfigureApi() {
  xvc_api_ = xvc_decoder_api_get();
  params_ = xvc_api_->parameters_create();
  xvc_api_->parameters_set_default(params_);
  if (cli_.output_width != -1) {
    params_->output_width = cli_.output_width;
  }
  if (cli_.output_height != -1) {
    params_->output_height = cli_.output_height;
  }
  if (cli_.output_chroma_format != XVC_DEC_CHROMA_FORMAT_UNDEFINED) {
    params_->output_chroma_format = cli_.output_chroma_format;
  }
  if (cli_.output_color_matrix != XVC_DEC_COLOR_MATRIX_UNDEFINED) {
    params_->output_color_matrix = cli_.output_color_matrix;
  }
  if (cli_.output_bitdepth != -1) {
    params_->output_bitdepth = cli_.output_bitdepth;
  }
  if (cli_.max_framerate != -1) {
    params_->max_framerate = cli_.max_framerate;
  }
  if (cli_.simd_mask != -1) {
    params_->simd_mask = cli_.simd_mask;
  }
  if (cli_.threads != -1) {
    params_->threads = cli_.threads;
  }
  if (xvc_api_->parameters_check(params_) != XVC_DEC_OK) {
    std::cerr << "Error. Invalid parameters. Please check the values of the"
      " command line parameters." << std::endl;
    PrintUsage();
    std::exit(1);
  }
  decoder_ = xvc_api_->decoder_create(params_);
}

void DecoderApp::PrintDecoderSettings() {
  if (!params_) {
    return;
  }
  GetLog() << "Bitstream-file:   " << cli_.input_filename << std::endl;
  GetLog() << "Output:           " << cli_.output_filename << std::endl;
}

void DecoderApp::MainDecoderLoop() {
  std::vector<uint8_t> nal_bytes_;
  xvc_decoded_picture decoded_pic;
  xvc_dec_return_code ret;
  num_pictures_decoded_ = 0;
  start_ = std::chrono::steady_clock::now();
  std::ostream &output_stream = output_to_stdout_ ?
    std::cout : file_output_stream_;
  if (output_to_stdout_) {
    std::cout.setf(std::ios::unitbuf);
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif
  }
  int loop_iterations = std::max(1, cli_.loop);
  if (cli_.loop == 0) {
    loop_iterations = std::numeric_limits<int>::max();
  }
  Y4mWriter y4m_writer;

  while (true) {
    // Get size of next Nal Unit.
    // size = 0 means no more Nal Units (end of file).
    size_t nal_size = ReadNextNalSize(&input_stream_);
    if (!nal_size) {
      if (--loop_iterations > 0) {
        input_stream_.clear();
        input_stream_.seekg(0, input_stream_.beg);
        continue;
      }
      break;
    }

    // Read next Nal Unit from file.
    nal_bytes_.resize(nal_size);
    input_stream_.read(reinterpret_cast<char *>(&nal_bytes_[0]), nal_size);
    if (static_cast<size_t>(input_stream_.gcount()) < nal_size) {
      std::cerr << "Unable to read nal." << std::endl;
      std::exit(1);
    }

    // Decode next Nal Unit.
    ret = xvc_api_->decoder_decode_nal(decoder_, &nal_bytes_[0], nal_size, 0);
    if (ret == XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER) {
      std::cerr << xvc_api_->xvc_dec_get_error_text(ret) << std::endl;
      std::exit(XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER);
    } else if (ret == XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH) {
      std::cerr << xvc_api_->xvc_dec_get_error_text(ret) << std::endl;
      std::exit(XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH);
    }

    // Check if there is a decoded picture ready to be output.
    if (xvc_api_->decoder_get_picture(decoder_, &decoded_pic) == XVC_DEC_OK) {
      if (output_stream.good()) {
        if (output_y4m_format_) {
          y4m_writer.WriteHeader(decoded_pic.stats, &output_stream);
        }
        output_stream.write(decoded_pic.bytes, decoded_pic.size);
      }
      if (cli_.verbose) {
        PrintPictureInfo(decoded_pic.stats);
      }
      num_pictures_decoded_++;
    }
  }

  // Flush out all remaining decoded pictures.
  while (xvc_api_->decoder_flush(decoder_, &decoded_pic) == XVC_DEC_OK) {
    if (output_stream.good()) {
      if (output_y4m_format_) {
        y4m_writer.WriteHeader(decoded_pic.stats, &output_stream);
      }
      output_stream.write(decoded_pic.bytes, decoded_pic.size);
    }
    PrintPictureInfo(decoded_pic.stats);
    num_pictures_decoded_++;
  }

  end_ = std::chrono::steady_clock::now();
}

void DecoderApp::CloseStream() {
  if (file_output_stream_.is_open()) {
    file_output_stream_.close();
  }
  input_stream_.close();
}

void DecoderApp::PrintStatistics() {
  GetLog() << std::endl;
  GetLog() << "Decoded:    " << num_pictures_decoded_
    << " pictures" << std::endl;
  GetLog() << "Total time: " <<
    std::chrono::duration<float>(end_ - start_).count() << " s" << std::endl;
}

int DecoderApp::CheckConformance() {
  if (num_pictures_decoded_ == 0) {
    GetLog() << std::endl;
    GetLog() << "No pictures were decoded." << std::endl;
    GetLog() << std::endl;
    return XVC_DEC_NO_DECODED_PIC;
  }
  int num_corrupted_pics;
  xvc_dec_return_code ret;
  ret = xvc_api_->decoder_check_conformance(decoder_, &num_corrupted_pics);
  if (ret == XVC_DEC_NOT_CONFORMING) {
    GetLog() << std::endl;
    GetLog() << "Error: A decoding mismatch occured in " <<
      num_corrupted_pics << " pictures." << std::endl;
    GetLog() << "The bitstream is NOT a conforming bitstream." << std::endl;
    GetLog() << std::endl;
    return ret;
  } else if (ret == XVC_DEC_OK) {
    assert(num_corrupted_pics == 0);
    GetLog() << std::endl;
    GetLog() << "Conformance verified." << std::endl;
    GetLog() << "The bitstream is a conforming bitstream." << std::endl;
    GetLog() << std::endl;
    return ret;
  }
  GetLog() << std::endl;
  GetLog() << "Error: Conformance check unsuccessful." << std::endl;
  GetLog() << std::endl;
  return ret;
}

void DecoderApp::PrintUsage() {
  GetLog() << std::endl << "Usage: -bitstream-file <string>"
    " -output-file <string>  [Optional parameters]" << std::endl;
  GetLog() << std::endl << "Optional parameters:" << std::endl;
  GetLog() << "  -output-width <int>" << std::endl;
  GetLog() << "  -output-height <int>" << std::endl;
  GetLog() << "  -output-chroma-format <int>" << std::endl;
  GetLog() << "      0: Monochrome" << std::endl;
  GetLog() << "      1: 4:2:0" << std::endl;
  GetLog() << "      2: 4:2:2" << std::endl;
  GetLog() << "      3: 4:4:4" << std::endl;
  GetLog() << "  -output-bitdepth <int>" << std::endl;
  GetLog() << "  -max-framerate <int>" << std::endl;
  GetLog() << "  -loop <int>" << std::endl;
  GetLog() << "  -verbose <0/1>" << std::endl;
}

size_t DecoderApp::ReadNextNalSize(std::istream *input) {
  size_t length;
  uint8_t nal_size[4];
  input->read(reinterpret_cast<char *>(nal_size), 4);
  if (input->gcount() < 4) {
    return 0;
  }
  length = nal_size[0] | (nal_size[1] << 8) | (nal_size[2] << 16) |
    (nal_size[3] << 24);
  return length;
}

void DecoderApp::PrintPictureInfo(xvc_dec_pic_stats pic_stats) {
  if (!segment_info_printed_) {
    segment_info_printed_ = 1;
    GetLog() << "Width:" << std::setw(21) << pic_stats.width << std::endl;
    GetLog() << "Height:" << std::setw(20) << pic_stats.height << std::endl;
    GetLog() << "Output bitdepth:" << std::setw(11) << pic_stats.bitdepth
      << std::endl;
    GetLog() << "Bitstream bitdepth:" << std::setw(8)
      << pic_stats.bitstream_bitdepth << std::endl;
    if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_MONOCHROME) {
      GetLog() << "Chroma format:  Monochorome" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_420) {
      GetLog() << "Chroma format:        4:2:0" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_422) {
      GetLog() << "Chroma format:        4:2:2" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_444) {
      GetLog() << "Chroma format:        4:4:4" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_ARGB) {
      GetLog() << "Chroma format:         ARGB" << std::endl;
    } else {
      GetLog() << "Chroma format:    " << pic_stats.chroma_format << std::endl;
    }
    if (pic_stats.color_matrix == XVC_DEC_COLOR_MATRIX_601) {
      GetLog() << "Color matrix:        BT.601" << std::endl;
    } else if (pic_stats.color_matrix == XVC_DEC_COLOR_MATRIX_709) {
      GetLog() << "Color matrix:        BT.709" << std::endl;
    } else if (pic_stats.color_matrix == XVC_DEC_COLOR_MATRIX_2020) {
      GetLog() << "Color matrix:       BT.2020" << std::endl;
    }
    GetLog() << "Output framerate:" << std::setw(10) << pic_stats.framerate
      << std::endl;
    GetLog() << "Bitstream framerate:" << std::setw(7)
      << pic_stats.bitstream_framerate << std::endl << std::endl;
  }
  if (cli_.verbose) {
    GetLog() << "NUT:" << std::setw(6) << pic_stats.nal_unit_type;
    GetLog() << "  POC:" << std::setw(6) << pic_stats.poc;
    GetLog() << "  DOC:" << std::setw(6) << pic_stats.doc;
    GetLog() << "  SOC:" << std::setw(6) << pic_stats.soc;
    GetLog() << "  TID:" << std::setw(6) << pic_stats.tid;
    GetLog() << "   QP:" << std::setw(6) << pic_stats.qp;
    if (pic_stats.l0[0] >= 0 || pic_stats.l1[0] >= 0) {
      GetLog() << "  RefPics: L0: { ";
      int length_l0 = sizeof(pic_stats.l0) / sizeof(pic_stats.l0[0]);
      for (int i = 0; i < length_l0; i++) {
        if (pic_stats.l0[i] > -1) {
          if (i > 0) {
            GetLog() << ", ";
          }
          GetLog() << std::setw(3) << pic_stats.l0[i];
        }
      }
      GetLog() << " } L1: { ";
      int length_l1 = sizeof(pic_stats.l1) / sizeof(pic_stats.l1[0]);
      for (int i = 0; i < length_l1; i++) {
        if (pic_stats.l1[i] > -1) {
          if (i > 0) {
            GetLog() << ", ";
          }
          GetLog() << std::setw(3) << pic_stats.l1[i];
        }
      }
      GetLog() << " }";
    }
    if (pic_stats.conforming == 0) {
      GetLog() << " <- Checksum mismatch!";
    }
    GetLog() << std::endl;
  }
}

}  // namespace xvc_app
