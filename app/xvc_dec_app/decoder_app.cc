/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_app/decoder_app.h"

#include <cassert>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

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
      continue;
    } else if (arg == "-bitstream-file") {
      cli_.input_file = argv[++i];
    } else if (arg == "-output-file") {
      cli_.output_file = argv[++i];
    } else if (arg == "-output-width") {
      std::stringstream(argv[++i]) >> cli_.output_width;
    } else if (arg == "-output-height") {
      std::stringstream(argv[++i]) >> cli_.output_height;
    } else if (arg == "-output-chroma-format") {
      int tmp;
      std::stringstream(argv[++i]) >> tmp;
      cli_.output_chroma_format = static_cast<xvc_dec_chroma_format>(tmp);
    } else if (arg == "-output-bitdepth") {
      std::stringstream(argv[++i]) >> cli_.output_bitdepth;
    } else if (arg == "-max-framerate") {
      std::stringstream(argv[++i]) >> cli_.max_framerate;
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
  if (cli_.input_file.empty()) {
    std::cerr << "Error: Missing bitstream file argument" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  input_stream_.open(cli_.input_file, std::ios_base::binary);
  if (!input_stream_) {
    std::cerr << "Failed to open bitstream file: "
      << cli_.input_file << std::endl;
    std::exit(1);
  }

  if (!cli_.output_file.empty()) {
    output_stream_.open(cli_.output_file, std::ios_base::binary);
    if (!output_stream_) {
      std::cerr << "Failed to open output file for writing: "
        << cli_.output_file << std::endl;
      std::exit(1);
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
  if (cli_.output_bitdepth != -1) {
    params_->output_bitdepth = cli_.output_bitdepth;
  }
  if (cli_.max_framerate != -1) {
    params_->max_framerate = cli_.max_framerate;
  }
  if (xvc_api_->parameters_check(params_) != XVC_DEC_OK) {
    std::cout << "Error. Invalid parameters. Please check the values of the"
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
  std::cout << "Bitstream-file:   " << cli_.input_file << std::endl;
  std::cout << "Output:           " << cli_.output_file << std::endl;
}

void DecoderApp::MainDecoderLoop() {
  std::vector<uint8_t> nal_bytes_;
  xvc_decoded_picture decoded_pic;
  xvc_dec_return_code ret;
  num_pictures_decoded_ = 0;
  start_ = std::chrono::steady_clock::now();

  while (true) {
    // Get size of next Nal Unit.
    // size = 0 means no more Nal Units (end of file).
    size_t nal_size = ReadNextNalSize();

    if (nal_size) {
      // Read next Nal Unit from file.
      nal_bytes_.resize(nal_size);
      input_stream_.read(reinterpret_cast<char *>(&nal_bytes_[0]), nal_size);
      if (static_cast<size_t>(input_stream_.gcount()) < nal_size) {
        std::cerr << "Unable to read nal." << std::endl;
        std::exit(1);
      }

      // Decode next Nal Unit.
      ret = xvc_api_->decoder_decode_nal(decoder_, &nal_bytes_[0], nal_size);
      if (ret == XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER) {
        std::cerr << "The xvc version indicated in the segment header is " <<
          "higher than the xvc version of the decoder." << std::endl <<
          "Please update the xvc decoder to the latest version." << std::endl;
        std::exit(XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER);
      } else if (ret == XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH) {
        std::cerr << "The bitstream is of higher bitdepth than what the " <<
          "decoder has been compiled for." << std::endl << "Please "
          "recompile the decoder to support higher bitdepth " <<
          "(by setting XVC_HIGH_BITDEPTH equal to 1)." << std::endl;
        std::exit(XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH);
      }

      // Check if there is a decoded picture ready to be output.
      if (xvc_api_->decoder_get_picture(decoder_, &decoded_pic) == XVC_DEC_OK) {
        if (output_stream_.is_open()) {
          output_stream_.write(decoded_pic.bytes, decoded_pic.size);
        }
        if (cli_.verbose) {
          PrintPictureInfo(decoded_pic.stats);
        }
        num_pictures_decoded_++;
      }
    } else {
      // Flush out all remaining decoded pictures.
      while (xvc_api_->decoder_flush(decoder_, &decoded_pic) == XVC_DEC_OK) {
        if (output_stream_.is_open()) {
          output_stream_.write(decoded_pic.bytes, decoded_pic.size);
        }
        PrintPictureInfo(decoded_pic.stats);
        num_pictures_decoded_++;
      }
      break;
    }
  }
  end_ = std::chrono::steady_clock::now();
}

void DecoderApp::CloseStream() {
  if (output_stream_.is_open()) {
    output_stream_.close();
  }
  input_stream_.close();
}

void DecoderApp::PrintStatistics() {
  std::cout << std::endl << "Decoded:    " << num_pictures_decoded_
    << " pictures" << std::endl;
  std::cout << "Total time: " <<
    std::chrono::duration<float>(end_ - start_).count() << " s" << std::endl;
}

int DecoderApp::CheckConformance() {
  if (num_pictures_decoded_ == 0) {
    std::cout << std::endl;
    std::cout << "No pictures were decoded." << std::endl;
    std::cout << std::endl;
    return XVC_DEC_NO_DECODED_PIC;
  }
  int num_corrupted_pics;
  xvc_dec_return_code ret;
  ret = xvc_api_->decoder_check_conformance(decoder_, &num_corrupted_pics);
  if (ret == XVC_DEC_NOT_CONFORMING) {
    std::cout << std::endl;
    std::cout << "Error: A decoding mismatch occured in " <<
      num_corrupted_pics << " pictures." << std::endl;
    std::cout << "The bitstream is NOT a conforming bitstream." << std::endl;
    std::cout << std::endl;
    return ret;
  } else if (ret == XVC_DEC_OK) {
    assert(num_corrupted_pics == 0);
    std::cout << std::endl;
    std::cout << "Conformance verified." << std::endl;
    std::cout << "The bitstream is a conforming bitstream." << std::endl;
    std::cout << std::endl;
    return ret;
  }
  std::cout << std::endl;
  std::cout << "Error: Conformance check unsuccessful." << std::endl;
  std::cout << std::endl;
  return ret;
}

void DecoderApp::PrintUsage() {
  std::cout << std::endl << "Usage: -bitstream-file <string>"
    " -output-file <string>  [Optional parameters]" << std::endl;
  std::cout << std::endl << "Optional parameters:" << std::endl;
  std::cout << "  -output-width <int>" << std::endl;
  std::cout << "  -output-height <int>" << std::endl;
  std::cout << "  -output-chroma-format <int>" << std::endl;
  std::cout << "      0: Monochrome" << std::endl;
  std::cout << "      1: 4:2:0" << std::endl;
  std::cout << "      2: 4:2:2" << std::endl;
  std::cout << "      3: 4:4:4" << std::endl;
  std::cout << "  -output-bitdepth <int>" << std::endl;
  std::cout << "  -max-framerate <int>" << std::endl;
  std::cout << "  -verbose <0/1>" << std::endl;
}

size_t DecoderApp::ReadNextNalSize() {
  size_t length;
  uint8_t nal_size[4];
  input_stream_.read(reinterpret_cast<char *>(nal_size), 4);
  if (input_stream_.gcount() < 4) {
    return 0;
  }
  length = nal_size[0] | (nal_size[1] << 8) | (nal_size[2] << 16) |
    (nal_size[3] << 24);
  return length;
}

void DecoderApp::PrintPictureInfo(xvc_dec_pic_stats pic_stats) {
  if (!segment_info_printed_) {
    segment_info_printed_ = 1;
    std::cout << "Width:" << std::setw(21) << pic_stats.width << std::endl;
    std::cout << "Height:" << std::setw(20) << pic_stats.height << std::endl;
    std::cout << "Output bitdepth:" << std::setw(11) << pic_stats.bitdepth
      << std::endl;
    std::cout << "Bitstream bitdepth:" << std::setw(8)
      << pic_stats.bitstream_bitdepth << std::endl;
    if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_MONOCHROME) {
      std::cout << "Chroma format:  Monochorome" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_420) {
      std::cout << "Chroma format:        4:2:0" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_422) {
      std::cout << "Chroma format:        4:2:2" << std::endl;
    } else if (pic_stats.chroma_format == XVC_DEC_CHROMA_FORMAT_444) {
      std::cout << "Chroma format:        4:4:4" << std::endl;
    }
    std::cout << "Output framerate:" << std::setw(10) << pic_stats.framerate
      << std::endl;
    std::cout << "Bitstream framerate:" << std::setw(7)
      << pic_stats.bitstream_framerate << std::endl << std::endl;
  }
  if (cli_.verbose) {
    std::cout << "NUT:" << std::setw(6) << pic_stats.nal_unit_type;
    std::cout << "  POC:" << std::setw(6) << pic_stats.poc;
    std::cout << "  DOC:" << std::setw(6) << pic_stats.doc;
    std::cout << "  SOC:" << std::setw(6) << pic_stats.soc;
    std::cout << "  TID:" << std::setw(6) << pic_stats.tid;
    std::cout << "   QP:" << std::setw(6) << pic_stats.qp;
    if (pic_stats.l0[0] >= 0 || pic_stats.l1[0] >= 0) {
      std::cout << "  RefPics: L0: { ";
      int length_l0 = sizeof(pic_stats.l0) / sizeof(pic_stats.l0[0]);
      for (int i = 0; i < length_l0; i++) {
        if (pic_stats.l0[i] > -1) {
          if (i > 0) {
            std::cout << ", ";
          }
          std::cout << std::setw(3) << pic_stats.l0[i];
        }
      }
      std::cout << " } L1: { ";
      int length_l1 = sizeof(pic_stats.l1) / sizeof(pic_stats.l1[0]);
      for (int i = 0; i < length_l1; i++) {
        if (pic_stats.l1[i] > -1) {
          if (i > 0) {
            std::cout << ", ";
          }
          std::cout << std::setw(3) << pic_stats.l1[i];
        }
      }
      std::cout << " }";
    }
    std::cout << std::endl;
  }
}

}  // namespace xvc_app
