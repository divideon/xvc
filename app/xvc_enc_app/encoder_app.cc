/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_app/encoder_app.h"

#include <cassert>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "xvc_enc_app/y4m.h"

namespace xvc_app {

EncoderApp::~EncoderApp() {
  if (encoder_) {
    xvc_api_->encoder_destroy(encoder_);
    encoder_ = nullptr;
  }
  if (params_) {
    xvc_api_->parameters_destroy(params_);
    params_ = nullptr;
  }
}

void EncoderApp::ReadArguments(int argc, const char *argv[]) {
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
    } else if (arg == "-input-file") {
      cli_.input_file = argv[++i];
    } else if (arg == "-output-file") {
      cli_.output_file = argv[++i];
    } else if (arg == "-rec-file") {
      cli_.rec_file = argv[++i];
    } else if (arg == "-input-width") {
      std::stringstream(argv[++i]) >> cli_.width;
    } else if (arg == "-input-height") {
      std::stringstream(argv[++i]) >> cli_.height;
    } else if (arg == "-input-chroma-format") {
      int tmp;
      std::stringstream(argv[++i]) >> tmp;
      cli_.chroma_format = static_cast<xvc_enc_chroma_format>(tmp);
    } else if (arg == "-input-bitdepth") {
      std::stringstream(argv[++i]) >> cli_.input_bitdepth;
    } else if (arg == "-internal-bitdepth") {
      std::stringstream(argv[++i]) >> cli_.internal_bitdepth;
    } else if (arg == "-framerate") {
      std::stringstream(argv[++i]) >> cli_.framerate;
    } else if (arg == "-skip-pictures") {
      std::stringstream(argv[++i]) >> cli_.skip_pictures;
    } else if (arg == "-temporal-subsample") {
      std::stringstream(argv[++i]) >> cli_.temporal_subsample;
    } else if (arg == "-max-pictures") {
      std::stringstream(argv[++i]) >> cli_.max_num_pictures;
    } else if (arg == "-sub-gop-length") {
      std::stringstream(argv[++i]) >> cli_.sub_gop_length;
    } else if (arg == "-max-keypic-distance") {
      std::stringstream(argv[++i]) >> cli_.max_keypic_distance;
    } else if (arg == "-closed-gop") {
      std::stringstream(argv[++i]) >> cli_.closed_gop;
    } else if (arg == "-num-ref-pics") {
      std::stringstream(argv[++i]) >> cli_.num_ref_pics;
    } else if (arg == "-restricted-mode") {
      std::stringstream(argv[++i]) >> cli_.restricted_mode;
    } else if (arg == "-checksum-mode") {
      std::stringstream(argv[++i]) >> cli_.checksum_mode;
    } else if (arg == "-deblock") {
      std::stringstream(argv[++i]) >> cli_.deblock;
    } else if (arg == "-beta-offset") {
      std::stringstream(argv[++i]) >> cli_.beta_offset;
    } else if (arg == "-tc-offset") {
      std::stringstream(argv[++i]) >> cli_.tc_offset;
    } else if (arg == "-qp") {
      std::stringstream(argv[++i]) >> cli_.qp;
    } else if (arg == "-flat-lambda") {
      std::stringstream(argv[++i]) >> cli_.flat_lambda;
    } else if (arg == "-speed-mode") {
      std::stringstream(argv[++i]) >> cli_.speed_mode;
    } else if (arg == "-explicit-speed-settings") {
      cli_.explicit_speed_settings = argv[++i];
    } else if (arg == "-verbose") {
      std::stringstream(argv[++i]) >> cli_.verbose;
    } else {
      std::cerr << "Error: Unknown argument: " << arg << std::endl;
      PrintUsage();
      std::exit(1);
    }
  }
}

bool EncoderApp::CheckParameters() {
  if (cli_.input_file.empty()) {
    std::cerr << "Error: Missing input file argument" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.output_file.empty()) {
    std::cerr << "Error: Missing output file argument" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  input_stream_.open(cli_.input_file, std::ios_base::binary);
  if (!input_stream_) {
    std::cerr << "Failed to open input file: " << cli_.input_file << std::endl;
    std::exit(1);
  }

  Y4M Y4MParser(input_stream_);
  if (!Y4MParser.ParseY4M(cli_.width, cli_.height, cli_.framerate,
                          cli_.input_bitdepth, start_skip_, &picture_skip_)) {
    start_skip_ = 0;
    picture_skip_ = 0;
  }

  if (cli_.width <= 0 || cli_.height <= 0) {
    std::cerr << "Error: Invalid width or height" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.input_bitdepth &&
    (cli_.input_bitdepth < 8 || cli_.input_bitdepth > 14)) {
    std::cerr << "Error: Input bitdepth must be between 8 and 14, inclusive."
      << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.internal_bitdepth &&
    (cli_.internal_bitdepth < 8 || cli_.internal_bitdepth > 14)) {
    std::cerr << "Error: Internal bitdepth must be between 8 and 14,"
      " inclusive." << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.internal_bitdepth && cli_.input_bitdepth &&
    (cli_.input_bitdepth > cli_.internal_bitdepth)) {
    std::cerr << "Error: Input bitdepth cannot be greater than internal"
      " bitdepth." << std::endl;
    PrintUsage();
    std::exit(1);
  }

  output_stream_.open(cli_.output_file, std::ios_base::binary);
  if (!output_stream_) {
    std::cerr << "Failed to open output file: " << cli_.output_file
      << std::endl;
    std::exit(1);
  }

  if (!cli_.rec_file.empty()) {
    rec_stream_.open(cli_.rec_file, std::ios_base::binary);
    if (!rec_stream_) {
      std::cerr << "Failed to open reconstruct file for writing: "
        << cli_.rec_file << std::endl;
      std::exit(1);
    }
  }

  if ((cli_.sub_gop_length > 0 && cli_.max_keypic_distance > 0)
      && (cli_.sub_gop_length > cli_.max_keypic_distance)) {
    std::cerr << "Error: Sub Gop length cannot be greater than Max"
      " keypic distance." << std::endl;
    PrintUsage();
    std::exit(1);
  }

  return true;
}

void EncoderApp::CreateAndConfigureApi() {
  xvc_api_ = xvc_encoder_api_get();
  params_ = xvc_api_->parameters_create();
  xvc_api_->parameters_set_default(params_);
  if (cli_.width) {
    params_->width = cli_.width;
  }
  if (cli_.height) {
    params_->height = cli_.height;
  }
  if (cli_.chroma_format != XVC_ENC_CHROMA_FORMAT_UNDEFINED) {
    params_->chroma_format = cli_.chroma_format;
  }
  if (cli_.input_bitdepth) {
    params_->input_bitdepth = cli_.input_bitdepth;
  }
  if (cli_.internal_bitdepth) {
    params_->internal_bitdepth = cli_.internal_bitdepth;
  }
  if (cli_.framerate) {
    params_->framerate = cli_.framerate;
  }
  if (cli_.sub_gop_length >= 0) {
    params_->sub_gop_length = cli_.sub_gop_length;
  }
  if (cli_.max_keypic_distance >= 0) {
    params_->max_keypic_distance = cli_.max_keypic_distance;
  }
  if (cli_.closed_gop != -1) {
    params_->closed_gop = cli_.closed_gop;
  }
  if (cli_.num_ref_pics != -1) {
    params_->num_ref_pics = cli_.num_ref_pics;
  }
  if (cli_.restricted_mode != -1) {
    params_->restricted_mode = cli_.restricted_mode;
  }
  if (cli_.deblock != -1) {
    params_->deblock = cli_.deblock;
  }
  if (cli_.checksum_mode != -1) {
    params_->checksum_mode = cli_.checksum_mode;
  }
  if (cli_.beta_offset != std::numeric_limits<int>::min()) {
    params_->beta_offset = cli_.beta_offset;
  }
  if (cli_.tc_offset != std::numeric_limits<int>::min()) {
    params_->tc_offset = cli_.tc_offset;
  }
  if (cli_.qp != -1) {
    params_->qp = cli_.qp;
  }
  if (cli_.flat_lambda >= 0) {
    params_->flat_lambda = cli_.flat_lambda;
  }
  if (cli_.speed_mode != -1) {
    params_->speed_mode = cli_.speed_mode;
  }
  if (!cli_.explicit_speed_settings.empty()) {
    params_->explicit_speed_settings = &cli_.explicit_speed_settings[0];
  }
  xvc_enc_return_code ret = xvc_api_->parameters_check(params_);
  if (ret != XVC_ENC_OK) {
    std::cout << xvc_api_->xvc_enc_get_error_text(ret) << std::endl;
    PrintUsage();
    std::exit(1);
  }
  encoder_ = xvc_api_->encoder_create(params_);
}

void EncoderApp::PrintEncoderSettings() {
  if (!params_) {
    return;
  }
  std::cout << "Input:        " << cli_.input_file << std::endl;
  std::cout << "Output:       " << cli_.output_file << std::endl;
  std::cout << "Size:         " << params_->width << "x" << params_->height
    << std::endl;
  std::cout << "Bitdepth:     " << params_->input_bitdepth << std::endl;
  std::cout << "Framerate:    " << params_->framerate << std::endl;
  std::cout << "QP:           " << params_->qp << std::endl;
}

void EncoderApp::MainEncoderLoop() {
  // Calculate how much data to read for each picture.
  std::vector<uint8_t> picture_bytes;
  int picture_samples = 0;
  if (params_->chroma_format == XVC_ENC_CHROMA_FORMAT_MONOCHROME) {
    picture_samples = params_->width * params_->height;
  } else if (params_->chroma_format == XVC_ENC_CHROMA_FORMAT_420) {
    picture_samples = (3 * (params_->width * params_->height)) >> 1;
  } else if (params_->chroma_format == XVC_ENC_CHROMA_FORMAT_422) {
    picture_samples = 2 * params_->width * params_->height;
  } else if (params_->chroma_format == XVC_ENC_CHROMA_FORMAT_444) {
    picture_samples = 3 * params_->width * params_->height;
  }
  assert(picture_samples > 0);
  picture_bytes.resize(params_->input_bitdepth == 8 ?
                       picture_samples : (picture_samples << 1));

  // Setting the buffer pointer for reconstructed picture to nullptr
  // indicates that no reconstructed picture is requested.
  xvc_enc_pic_buffer *rec_pic_ptr = nullptr;
  if (!cli_.rec_file.empty() && rec_stream_.is_open()) {
    rec_pic_buffer_ = { 0 };
    rec_pic_ptr = &rec_pic_buffer_;
  }

  // Temporal subsampling can be used to encode every n'th frame
  // of the input file.
  int max_num_pics = cli_.max_num_pictures;

  xvc_enc_nal_unit *nal_units;
  int num_nal_units;
  input_stream_.seekg(start_skip_ + (picture_skip_ + picture_bytes.size()) *
                      cli_.skip_pictures, std::ifstream::beg);
  char nal_size[4];
  start_ = std::chrono::steady_clock::now();
  bool loop_check = true;
  // loop_check will become false when the entire file has been encoded
  // or when max_num_pics have been encoded.
  while (loop_check) {
    input_stream_.seekg(picture_skip_, std::ifstream::cur);
    input_stream_.read(reinterpret_cast<char *>(&picture_bytes.front()),
                       picture_bytes.size());
    if (input_stream_.gcount() < static_cast<int>(picture_bytes.size()) ||
      (max_num_pics >= 0 && picture_index_ >= max_num_pics)) {
      // Flush the encoder for remaining nal_units and reconstructed pictures.
      xvc_api_->encoder_flush(encoder_, &nal_units, &num_nal_units,
                              rec_pic_ptr);
      // loop_check will remain true as long as there are buffered pictures
      // that should be reconstructed.
      loop_check = (rec_pic_ptr && (rec_pic_ptr->size > 0));
    } else {
      // Encode one picture and get 0 or 1 reconstructed picture back.
      // Also get back 0 or more nal_units depending on if pictures are being
      // buffered in order to encode a full Sub Gop.
      xvc_api_->encoder_encode(encoder_, &picture_bytes[0], &nal_units,
                               &num_nal_units, rec_pic_ptr);
      picture_index_++;
    }

    // Loop through all Nal Units that were received and write to file
    // the Nal Unit length followed by the actual Nal Unit.
    for (int i = 0; i < num_nal_units; i++) {
      nal_size[0] = nal_units[i].size & 0xFF;
      nal_size[1] = (nal_units[i].size >> 8) & 0xFF;
      nal_size[2] = (nal_units[i].size >> 16) & 0xFF;
      nal_size[3] = (nal_units[i].size >> 24) & 0xFF;
      output_stream_.write(nal_size, 4);
      output_stream_.write(reinterpret_cast<char *>(nal_units[i].bytes),
                           nal_units[i].size);
      total_bytes_ += nal_units[i].size;

      // Conditionally print information for each Nal Unit that is written
      // to the file.
      if (cli_.verbose > 0) {
        PrintNalInfo(nal_units[i]);
      }
    }

    // Print reconstruction only if a reconstruction file has been indicated
    // in the cli parameters.
    if (rec_stream_.is_open() && rec_pic_ptr &&
        rec_pic_ptr->size > 0) {
      rec_stream_.write(reinterpret_cast<char *>(rec_pic_ptr->pic),
                        rec_pic_ptr->size);
    }

    // Jump forward in the input file if temporal subsampling is applied.
    if (cli_.temporal_subsample > 1) {
      std::streamoff skip_picture = picture_bytes.size() + picture_skip_;
      input_stream_.seekg((cli_.temporal_subsample - 1) * skip_picture,
                          std::ifstream::cur);
    }
  }

  end_ = std::chrono::steady_clock::now();
}

void EncoderApp::CloseStream() {
  if (rec_stream_.is_open()) {
    rec_stream_.close();
  }
  output_stream_.close();
  input_stream_.close();
}

void EncoderApp::PrintStatistics() {
  if (!params_) {
    std::cerr << "No parameter data available" << std::endl;
    return;
  }
  float seq_seconds = static_cast<float>(picture_index_ / params_->framerate);
  std::cout << std::endl;
  std::cout << "Encoded:       " << picture_index_ << " pictures" << std::endl;
  std::cout << "Total time:    " <<
    std::chrono::duration<float>(end_ - start_).count() << " s" << std::endl;
  std::cout << "Total written: " << total_bytes_ << " bytes" << std::endl;
  std::cout << "Total bitrate: " <<
    (total_bytes_ * 8 / 1000 / seq_seconds) << " kbit/s" << std::endl;
}

void EncoderApp::PrintUsage() {
  std::cout << std::endl << "Usage:" << std::endl;
  std::cout << "  -input-file <string> -output-file"
    " <string> [Optional parameters]" << std::endl;
  std::cout << std::endl << "Optional parameters:" << std::endl;
  std::cout << "  -rec-file <string>" << std::endl;
  std::cout << "  -input-width <int>" << std::endl;
  std::cout << "  -input-height <int>" << std::endl;
  std::cout << "  -input-chroma-format <int>" << std::endl;
  std::cout << "      0: Monochrome" << std::endl;
  std::cout << "      1: 4:2:0 (default)" << std::endl;
  std::cout << "      2: 4:2:2" << std::endl;
  std::cout << "      3: 4:4:4" << std::endl;
  std::cout << "  -input-bitdepth <int>" << std::endl;
  std::cout << "  -internal-bitdepth <int>" << std::endl;
  std::cout << "  -framerate <float>" << std::endl;
  std::cout << "  -skip-pictures <int>" << std::endl;
  std::cout << "  -temporal-subsample <int>" << std::endl;
  std::cout << "  -max-pictures <int>" << std::endl;
  std::cout << "  -sub-gop-length <int>" << std::endl;
  std::cout << "  -max-keypic-distance <int>" << std::endl;
  std::cout << "  -closed-gop <int>" << std::endl;
  std::cout << "  -num-ref-pics <int>" << std::endl;
  std::cout << "  -checksum-mode <int>" << std::endl;
  std::cout << "      0: Only temporal layer 0, all components combined"
    << std::endl;
  std::cout << "      1: All pictures, all components separately (default)"
    << std::endl;
  std::cout << "  -deblock <0/1>" << std::endl;
  std::cout << "  -beta-offset <int>" << std::endl;
  std::cout << "  -tc-offset <int>" << std::endl;
  std::cout << "  -qp <int>" << std::endl;
  std::cout << "  -speed-mode <int>" << std::endl;
  std::cout << "  -verbose <0/1>" << std::endl;
}


void EncoderApp::PrintNalInfo(xvc_enc_nal_unit nal_unit) {
  std::cout << "NUT:" << std::setw(6) << nal_unit.stats.nal_unit_type;
  if (nal_unit.stats.nal_unit_type < 16) {
    std::cout << "  POC:" << std::setw(6) << nal_unit.stats.poc;
    std::cout << "  DOC:" << std::setw(6) << nal_unit.stats.doc;
    std::cout << "  SOC:" << std::setw(6) << nal_unit.stats.soc;
    std::cout << "  TID:" << std::setw(6) << nal_unit.stats.tid;
    std::cout << "   QP:" << std::setw(6) << nal_unit.stats.qp;
  } else {
    std::cout << "     - not a picture -              ";
  }
  std::cout << "  Bytes: " << std::setw(10) << nal_unit.size;
  if (nal_unit.stats.nal_unit_type < 16) {
    if (nal_unit.stats.l0[0] >= 0 || nal_unit.stats.l1[0] >= 0) {
      std::cout << "  RefPics: L0: { ";
      int length_l0 = sizeof(nal_unit.stats.l0) / sizeof(nal_unit.stats.l0[0]);
      for (int i = 0; i < length_l0; i++) {
        if (nal_unit.stats.l0[i] > -1) {
          if (i > 0) {
            std::cout << ", ";
          }
          std::cout << std::setw(3) << nal_unit.stats.l0[i];
        }
      }
      std::cout << " } L1: { ";
      int length_l1 = sizeof(nal_unit.stats.l1) / sizeof(nal_unit.stats.l1[0]);
      for (int i = 0; i < length_l1; i++) {
        if (nal_unit.stats.l1[i] > -1) {
          if (i > 0) {
            std::cout << ", ";
          }
          std::cout << std::setw(3) << nal_unit.stats.l1[i];
        }
      }
      std::cout << " }";
    }
  }
  std::cout << std::endl;
}

}  // namespace xvc_app
