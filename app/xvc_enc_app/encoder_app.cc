/******************************************************************************
* Copyright (C) 2017, Divideon.
*
* Redistribution and use in source and binary form, with or without
* modifications is permitted only under the terms and conditions set forward
* in the xvc Licence Agreement. For commercial redistribution and use, you are
* required to send a signed copy of the xvc License Agreement to Divideon.
*
* Redistribution and use in source and binary form is permitted free of charge
* for non-commercial purposes. See definition of non-commercial in the xvc
* License Agreement.
*
* All redistribution of source code must retain this copyright notice
* unmodified.
*
* The xvc Licence Agreement is available at https://xvc.io/license/.
******************************************************************************/

#include "xvc_enc_app/encoder_app.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "xvc_enc_app/y4m_reader.h"

namespace xvc_app {

static const int kDefaultSubGopSize = 16;

EncoderApp::EncoderApp() : cli_() {
  xvc_api_ = xvc_encoder_api_get();
  params_ = xvc_api_->parameters_create();
}

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
      std::cerr << "Error: Invalid argument / Missing value: " << arg <<
        std::endl;
      PrintUsage();
      std::exit(1);
    } else if (arg == "-input-file") {
      cli_.input_filename = argv[++i];
    } else if (arg == "-output-file") {
      cli_.output_filename = argv[++i];
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
    } else if (arg == "-input-color-matrix") {
      int tmp;
      std::stringstream(argv[++i]) >> tmp;
      cli_.color_matrix = static_cast<xvc_enc_color_matrix>(tmp);
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
    } else if (arg == "-low-delay") {
      std::stringstream(argv[++i]) >> cli_.low_delay;
    } else if (arg == "-num-ref-pics") {
      std::stringstream(argv[++i]) >> cli_.num_ref_pics;
    } else if (arg == "-restricted-mode") {
      std::stringstream(argv[++i]) >> cli_.restricted_mode;
    } else if (arg == "-checksum-mode") {
      std::stringstream(argv[++i]) >> cli_.checksum_mode;
    } else if (arg == "-chroma-qp-offset-table") {
      std::stringstream(argv[++i]) >> cli_.chroma_qp_offset_table;
    } else if (arg == "-chroma-qp-offset-u") {
      std::stringstream(argv[++i]) >> cli_.chroma_qp_offset_u;
    } else if (arg == "-chroma-qp-offset-v") {
      std::stringstream(argv[++i]) >> cli_.chroma_qp_offset_v;
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
    } else if (arg == "-multi-passes") {
      std::stringstream(argv[++i]) >> cli_.multipass_rd;
    } else if (arg == "-speed-mode") {
      std::stringstream(argv[++i]) >> cli_.speed_mode;
    } else if (arg == "-tune") {
      std::stringstream(argv[++i]) >> cli_.tune_mode;
    } else if (arg == "-simd-mask") {
      std::stringstream(argv[++i]) >> cli_.simd_mask;
    } else if (arg == "-explicit-encoder-settings") {
      cli_.explicit_encoder_settings = argv[++i];
    } else if (arg == "-verbose") {
      std::stringstream(argv[++i]) >> cli_.verbose;
    } else {
      std::cerr << "Error: Unknown argument: " << arg << std::endl;
      PrintUsage();
      std::exit(1);
    }
  }
}

void EncoderApp::CheckParameters() {
  if (cli_.input_filename.empty()) {
    std::cerr << "Error: Missing input file argument" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.output_filename.empty()) {
    std::cerr << "Error: Missing output file argument" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.input_filename == "-") {
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    input_seekable_ = false;
    input_stream_ = &std::cin;
    input_file_size_ = 0;
  } else {
    file_input_stream_.open(cli_.input_filename, std::ios_base::binary);
    if (!file_input_stream_) {
      std::cerr << "Failed to open input file: " << cli_.input_filename
        << std::endl;
      std::exit(1);
    }
    input_seekable_ = true;
    input_stream_ = &file_input_stream_;
    input_stream_->seekg(0, std::ifstream::end);
    input_file_size_ = input_stream_->tellg();
    input_stream_->seekg(0, std::ifstream::beg);
  }

  Y4mReader y4m_reader(input_stream_);
  PictureFormat picture_format;
  if (y4m_reader.Read(&picture_format, &start_skip_, &picture_skip_)) {
    cli_.width = picture_format.width;
    cli_.height = picture_format.height;
    cli_.framerate = picture_format.framerate;
    cli_.input_bitdepth = picture_format.input_bitdepth;
    cli_.chroma_format = picture_format.chroma_format;
  } else {
    start_skip_ = 0;
    picture_skip_ = 0;
    if (input_stream_ == &std::cin) {
      std::cerr << "Error: Failed to parse y4m from stdin" << std::endl;
      std::exit(1);
    }
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

  file_output_stream_.open(cli_.output_filename, std::ios_base::binary);
  if (!file_output_stream_) {
    std::cerr << "Failed to open output file: " << cli_.output_filename
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

  if (cli_.multipass_rd < 0 || cli_.multipass_rd > 2) {
    std::cerr << "Error: Invalid multi-pass configuration" << std::endl;
    PrintUsage();
    std::exit(1);
  }

  if (cli_.multipass_rd > 0 && !input_seekable_) {
    std::cerr << "Warning: Disabling multi-pass and lookahead on "
      "non-seekable input-streams" << std::endl;
    cli_.multipass_rd = 0;
  }

  xvc_enc_return_code ret = ConfigureApiParams(params_);
  if (ret != XVC_ENC_OK) {
    std::cerr << xvc_api_->xvc_enc_get_error_text(ret) << std::endl;
    PrintUsage();
    std::exit(1);
  }
}

xvc_enc_return_code
EncoderApp::ConfigureApiParams(xvc_encoder_parameters *params) {
  xvc_api_->parameters_set_default(params);
  if (cli_.width) {
    params->width = cli_.width;
  }
  if (cli_.height) {
    params->height = cli_.height;
  }
  if (cli_.chroma_format != XVC_ENC_CHROMA_FORMAT_UNDEFINED) {
    params->chroma_format = cli_.chroma_format;
  }
  if (cli_.color_matrix != XVC_ENC_COLOR_MATRIX_UNDEFINED) {
    params->color_matrix = cli_.color_matrix;
  }
  if (cli_.input_bitdepth) {
    params->input_bitdepth = cli_.input_bitdepth;
  }
  if (cli_.internal_bitdepth) {
    params->internal_bitdepth = cli_.internal_bitdepth;
  }
  if (cli_.framerate) {
    params->framerate = cli_.framerate;
  }
  if (cli_.sub_gop_length >= 0) {
    params->sub_gop_length = cli_.sub_gop_length;
  }
  if (cli_.max_keypic_distance >= 0) {
    params->max_keypic_distance = cli_.max_keypic_distance;
  }
  if (cli_.closed_gop != -1) {
    params->closed_gop = cli_.closed_gop;
  }
  if (cli_.low_delay != -1) {
    params->low_delay = cli_.low_delay;
  }
  if (cli_.num_ref_pics != -1) {
    params->num_ref_pics = cli_.num_ref_pics;
  }
  if (cli_.restricted_mode != -1) {
    params->restricted_mode = cli_.restricted_mode;
  }
  if (cli_.chroma_qp_offset_table != -1) {
    params->chroma_qp_offset_table = cli_.chroma_qp_offset_table;
  }
  if (cli_.chroma_qp_offset_u != std::numeric_limits<int>::min()) {
    params->chroma_qp_offset_u = cli_.chroma_qp_offset_u;
  }
  if (cli_.chroma_qp_offset_v != std::numeric_limits<int>::min()) {
    params->chroma_qp_offset_v = cli_.chroma_qp_offset_v;
  }
  if (cli_.deblock != -1) {
    params->deblock = cli_.deblock;
  }
  if (cli_.checksum_mode != -1) {
    params->checksum_mode = cli_.checksum_mode;
  }
  if (cli_.beta_offset != std::numeric_limits<int>::min()) {
    params->beta_offset = cli_.beta_offset;
  }
  if (cli_.tc_offset != std::numeric_limits<int>::min()) {
    params->tc_offset = cli_.tc_offset;
  }
  if (cli_.qp != -1) {
    params->qp = cli_.qp;
  }
  if (cli_.flat_lambda >= 0) {
    params->flat_lambda = cli_.flat_lambda;
  }
  if (cli_.speed_mode != -1) {
    params->speed_mode = cli_.speed_mode;
  }
  if (cli_.tune_mode != -1) {
    params->tune_mode = cli_.tune_mode;
  }
  if (cli_.simd_mask != -1) {
    params->simd_mask = cli_.simd_mask;
  }
  if (!cli_.explicit_encoder_settings.empty()) {
    params->explicit_encoder_settings = &cli_.explicit_encoder_settings[0];
  }
  return xvc_api_->parameters_check(params);
}

void EncoderApp::PrintEncoderSettings() {
  if (!params_) {
    return;
  }
  std::cout << "Input:            " << cli_.input_filename << std::endl;
  std::cout << "Output:           " << cli_.output_filename << std::endl;
  std::cout << "Size:             " << params_->width << "x" << params_->height
    << std::endl;
  std::cout << "Bitdepth:         " << params_->input_bitdepth << std::endl;
  std::cout << "Framerate:        " << params_->framerate << std::endl;
  std::cout << "QP:               " << params_->qp << std::endl;
}

void EncoderApp::MainEncoderLoop() {
  // Calculate how much data to read for each picture.
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
  picture_bytes_.resize(params_->input_bitdepth == 8 ?
                        picture_samples : (picture_samples << 1));

  if (cli_.multipass_rd == 1) {
    StartPictureDetermination(params_);
  } else if (cli_.multipass_rd > 1) {
    MultiPass(params_);
  }
  EncodeOnePass(params_, true);

  // Cleanup
  if (rec_stream_.is_open()) {
    rec_stream_.close();
  }
  file_output_stream_.close();
  file_input_stream_.close();
}

std::pair<uint64_t, int>
EncoderApp::EncodeOnePass(xvc_encoder_parameters *params, bool last) {
  if (encoder_) {
    xvc_enc_return_code ret = xvc_api_->encoder_destroy(encoder_);
    assert(ret == XVC_ENC_OK);
    ((void)(ret));
  }
  encoder_ = xvc_api_->encoder_create(params);
  if (!encoder_) {
    std::cerr << "Error: Failed to allocate encoder" << std::endl;
    std::exit(1);
  }

  // Setting the buffer pointer for reconstructed picture to nullptr
  // indicates that no reconstructed picture is requested.
  xvc_enc_pic_buffer *rec_pic_ptr = nullptr;
  if (!cli_.rec_file.empty() && rec_stream_.is_open()) {
    rec_pic_buffer_ = { 0 };
    rec_pic_ptr = &rec_pic_buffer_;
  }

  if (input_seekable_) {
    // Skip input pictures (if supported by stream)
    std::streamoff start_pos = start_skip_ +
      (picture_skip_ + picture_bytes_.size()) * cli_.skip_pictures;
    if (start_pos >= input_file_size_) {
      std::cerr << "Error: The value of skip-pictures is larger than the "
        << "number of pictures in the input file.";
      std::exit(1);
    }
    input_stream_->seekg(start_pos, std::ifstream::beg);
  }

  xvc_enc_return_code ret;
  xvc_enc_nal_unit *nal_units;
  int num_nal_units;
  char nal_size[4];
  size_t current_segment_bytes = 0;
  int current_segment_pics = 0;
  int highest_qp = std::numeric_limits<int>::min();
  uint64_t total_sse = 0;
  picture_index_ = 0;
  total_bytes_ = 0;
  max_segment_bytes_ = 0;
  max_segment_pics_ = 0;
  start_ = std::chrono::steady_clock::now();

  // loop_check will become false when the entire file has been encoded
  // or when max_num_pics have been encoded.
  bool loop_check = true;
  while (loop_check) {
    bool read_success = ReadNextPicture(&picture_bytes_);
    if (!read_success ||
      (cli_.max_num_pictures >= 0 && picture_index_ >= cli_.max_num_pictures)) {
      // Flush the encoder for remaining nal_units and reconstructed pictures.
      ret = xvc_api_->encoder_flush(encoder_, &nal_units, &num_nal_units,
                                    rec_pic_ptr);
      assert(ret == XVC_ENC_OK || ret == XVC_ENC_NO_MORE_OUTPUT);
      // loop_check will remain true as long as there are buffered pictures
      // that should be reconstructed.
      loop_check = ret == XVC_ENC_OK;
    } else {
      // Encode one picture and get 0 or 1 reconstructed picture back.
      // Also get back 0 or more nal_units depending on if pictures are being
      // buffered in order to encode a full Sub Gop.
      ret = xvc_api_->encoder_encode(encoder_, &picture_bytes_[0], &nal_units,
                                     &num_nal_units, rec_pic_ptr);
      assert(ret == XVC_ENC_OK);
      picture_index_++;
    }

    // Loop through all Nal Units that were received and write to file
    // the Nal Unit length followed by the actual Nal Unit.
    for (int i = 0; i < num_nal_units; i++) {
      nal_size[0] = nal_units[i].size & 0xFF;
      nal_size[1] = (nal_units[i].size >> 8) & 0xFF;
      nal_size[2] = (nal_units[i].size >> 16) & 0xFF;
      nal_size[3] = (nal_units[i].size >> 24) & 0xFF;
      if (nal_units[i].stats.nal_unit_type == 16) {
        if (current_segment_bytes > max_segment_bytes_) {
          max_segment_bytes_ = current_segment_bytes;
          max_segment_pics_ = current_segment_pics;
        }
        current_segment_bytes = 0;
        current_segment_pics = -1;
      } else {
        total_sse += nal_units[i].stats.sse;
        highest_qp = std::max(highest_qp, nal_units[i].stats.qp);
      }
      current_segment_bytes += nal_units[i].size;
      current_segment_pics++;
      file_output_stream_.write(nal_size, 4);
      file_output_stream_.write(reinterpret_cast<char *>(nal_units[i].bytes),
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
    if (input_seekable_ && cli_.temporal_subsample > 1) {
      std::streamoff skip_picture = picture_bytes_.size() + picture_skip_;
      input_stream_->seekg((cli_.temporal_subsample - 1) * skip_picture,
                           std::ifstream::cur);
    }
  }

  end_ = std::chrono::steady_clock::now();
  if (current_segment_bytes > max_segment_bytes_) {
    max_segment_bytes_ = current_segment_bytes;
    max_segment_pics_ = current_segment_pics;
  }

  if (!last) {
    std::cout << "Pass bits:     " << total_bytes_ * 8 << "\n";
    std::cout << "Pass SSE:      " << total_sse << "\n";
    std::cout << std::endl;
    ResetStreams();
  }
  assert(total_bytes_ > 0);
  assert(total_bytes_ < static_cast<size_t>(std::numeric_limits<int>::max()));
  return std::make_pair(total_sse, static_cast<int>(total_bytes_ * 8));
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
  std::cout << "Peak bitrate:  " <<
    (max_segment_bytes_ * 8 / 1000 / (max_segment_pics_ / params_->framerate))
    << " kbit/s" << std::endl;
}

void EncoderApp::StartPictureDetermination(xvc_encoder_parameters *out_params) {
  static const double kPocRatio = 0.6875;
  const auto param_delete = [this](xvc_encoder_parameters *p) {
    xvc_api_->parameters_destroy(p);
  };
  const auto encoder_delete = [this](xvc_encoder *p) {
    xvc_api_->encoder_destroy(p);
  };
  const std::streampos required_size = start_skip_ +
    (picture_skip_ + picture_bytes_.size()) * kDefaultSubGopSize;
  const int sub_gop_length = params_->sub_gop_length < 1 ?
    kDefaultSubGopSize : params_->sub_gop_length;
  const int middle_poc = static_cast<int>(kPocRatio * sub_gop_length + 0.5);
  const std::array<std::array<int, 2>, 2> test_positions = { {
    { 0, middle_poc },
    { (sub_gop_length - 1), middle_poc }
  } };
  if (!input_seekable_ || sub_gop_length < 4 ||
      input_file_size_ < required_size) {
    std::cout << "Warning: Singlepass lookahead not attempted" << std::endl;
    return;
  }

  std::unique_ptr<xvc_encoder_parameters, decltype(param_delete)>
    lookahead_params(xvc_api_->parameters_create(), param_delete);
  std::unique_ptr<xvc_encoder, decltype(encoder_delete)>
    lookahead_encoder(nullptr, encoder_delete);
  xvc_enc_return_code ret =
    ConfigureApiParams(lookahead_params.get());
  if (ret != XVC_ENC_OK) {
    return;
  }
  lookahead_params->speed_mode = 2;
  lookahead_params->sub_gop_length = 2;
  std::array<size_t, 2> result = { 0, 0 };
  const auto time_start = std::chrono::steady_clock::now();

  for (int i = 0; i < static_cast<int>(test_positions.size()); i++) {
    lookahead_encoder.reset(xvc_api_->encoder_create(lookahead_params.get()));
    assert(lookahead_encoder);
    xvc_enc_nal_unit *nal_units;
    int num_nal_units;
    for (int poc : test_positions[i]) {
      std::streamoff seek_pos = start_skip_ +
        (picture_skip_ + picture_bytes_.size()) * poc;
      input_stream_->seekg(seek_pos, std::ifstream::beg);
      if (!ReadNextPicture(&picture_bytes_)) {
        input_stream_->seekg(0, std::ifstream::beg);
        return;
      }
      ret =
        xvc_api_->encoder_encode(lookahead_encoder.get(), &picture_bytes_[0],
                                 &nal_units, &num_nal_units, nullptr);
      assert(ret == XVC_ENC_OK);
    }
    ret = xvc_api_->encoder_flush(lookahead_encoder.get(), &nal_units,
                                  &num_nal_units, nullptr);
    assert(ret == XVC_ENC_OK || ret == XVC_ENC_NO_MORE_OUTPUT);
    result[i] = nal_units[0].size;
  }
  const auto time_end = std::chrono::steady_clock::now();

  out_params->leading_pictures = result[1] <= result[0];
  std::cout << "Leading Picture:  " << out_params->leading_pictures << "\n";
  std::cout << "Lookahead time:   "
    << std::chrono::duration<float>(time_end - time_start).count() << " s"
    << std::endl;
  input_stream_->seekg(0, std::ifstream::beg);
}

void EncoderApp::MultiPass(xvc_encoder_parameters *out_params) {
  const auto param_delete = [this](xvc_encoder_parameters *p) {
    xvc_api_->parameters_destroy(p);
  };
  std::unique_ptr<xvc_encoder_parameters, decltype(param_delete)>
    multipass_params(xvc_api_->parameters_create(), param_delete);
  const auto time_start = std::chrono::steady_clock::now();
  std::cout << "\n";

  int best_preset = 0;
  ConfigureApiParams(multipass_params.get());
  multipass_params->speed_mode = 2;
  xvc_enc_return_code ret =
    xvc_api_->parameters_apply_rd_preset(best_preset, multipass_params.get());
  assert(ret == XVC_ENC_OK);
  int best_qp = multipass_params->qp;

  // Determine shape of lambda curve based on 2 rate points
  multipass_params->qp = best_qp - 2;
  const auto dist_bits1 = EncodeOnePass(multipass_params.get());
  multipass_params->qp = best_qp;
  const auto dist_bits0 = EncodeOnePass(multipass_params.get());
  LambdaCurve lambda_curve(dist_bits0, best_qp, dist_bits1, best_qp - 2);
  const int64_t base_distortion = dist_bits0.first;

  for (int preset = 0; true; preset++) {
    if (preset == best_preset) {
      continue;
    }
    ConfigureApiParams(multipass_params.get());
    multipass_params->speed_mode = 2;
    multipass_params->qp = best_qp;
    ret = xvc_api_->parameters_apply_rd_preset(preset, multipass_params.get());
    if (ret == XVC_ENC_NO_SUCH_PRESET) {
      break;
    }
    if (ret != XVC_ENC_OK) {
      std::cout << xvc_api_->xvc_enc_get_error_text(ret) << std::endl;
      std::exit(1);
    }
    std::cout << "Eval multi-pass preset: " << preset
      << " QP: " << multipass_params->qp << std::endl;
    const auto dist_bits = EncodeOnePass(multipass_params.get());
    const bool is_better = lambda_curve.IsPointBetter(dist_bits);

    if (is_better) {
      // Determine 2nd rate point
      LambdaCurve copy_scale(lambda_curve, dist_bits, multipass_params->qp);
      double qp_steps_frac =
        copy_scale.GetQpAtDistortion(base_distortion) - multipass_params->qp;
      int qp_steps = ::lround(qp_steps_frac);
      bool change_best_qp = qp_steps != 0;
      if (qp_steps == 0) {
        qp_steps = qp_steps_frac > 0 ? 1 : -1;
      }
      multipass_params->qp += qp_steps;

      std::cout << "Refine multi-pass preset: " << preset
        << " QP: " << multipass_params->qp << std::endl;
      auto dist_bits2 = EncodeOnePass(multipass_params.get());
      if (!lambda_curve.IsPointBetter(dist_bits2)) {
        continue;
      }

      best_preset = preset;
      lambda_curve =
        LambdaCurve(dist_bits, best_qp, dist_bits2, multipass_params->qp);
      if (change_best_qp) {
        best_qp += qp_steps;
      }
    }
  }
  const auto time_end = std::chrono::steady_clock::now();

  std::cout << "Best preset:      " << best_preset << "\n";
  std::cout << "Multipass time:   " <<
    std::chrono::duration<float>(time_end - time_start).count() << " s\n";
  std::cout << std::endl;

  assert(best_preset >= 0);
  ret = xvc_api_->parameters_apply_rd_preset(best_preset, out_params);
  assert(ret == XVC_ENC_OK);
  out_params->qp = best_qp;
}

void EncoderApp::ResetStreams() {
  input_stream_->clear();
  input_stream_->seekg(0, std::ifstream::beg);
  file_output_stream_.close();
  file_output_stream_.open(cli_.output_filename, std::ios_base::binary);
  if (rec_stream_.is_open() && !cli_.rec_file.empty()) {
    rec_stream_.close();
    rec_stream_.open(cli_.rec_file, std::ios_base::binary);
  }
}

bool EncoderApp::ReadNextPicture(std::vector<uint8_t> *picture_bytes) {
  assert(picture_bytes->size() > 0);
  if (picture_skip_ > 0) {
    if (input_seekable_) {
      input_stream_->seekg(picture_skip_, std::ifstream::cur);
    } else {
      assert(picture_skip_ <
             static_cast<std::streamoff>(picture_bytes->size()));
      // Read dummy frame header
      input_stream_->read(reinterpret_cast<char *>(&(*picture_bytes)[0]),
                          picture_skip_);
    }
  }
  input_stream_->read(reinterpret_cast<char *>(&(*picture_bytes)[0]),
                      picture_bytes->size());
  return input_stream_->gcount() == static_cast<int>(picture_bytes->size());
}

void EncoderApp::PrintUsage() {
  std::cout << std::endl << "Usage:" << std::endl;
  std::cout << "  -input-file <string> -output-file"
    " <string> [Optional parameters]" << std::endl;
  std::cout << std::endl << "Optional parameters:" << std::endl;
  std::cout << "  -rec-file <string>" << std::endl;
  std::cout << "  -input-width <16..65535>" << std::endl;
  std::cout << "  -input-height <16..65535>" << std::endl;
  std::cout << "  -input-chroma-format <0..3>" << std::endl;
  std::cout << "      0: Monochrome" << std::endl;
  std::cout << "      1: 4:2:0 (default)" << std::endl;
  std::cout << "      2: 4:2:2" << std::endl;
  std::cout << "      3: 4:4:4" << std::endl;
#if XVC_HIGH_BITDEPTH
  std::cout << "  -input-bitdepth <8..14>" << std::endl;
  std::cout << "  -internal-bitdepth <8..14>" << std::endl;
#else
  std::cout << "  -input-bitdepth <8>" << std::endl;
  std::cout << "  -internal-bitdepth <8>" << std::endl;
#endif
  std::cout << "  -framerate <0.0054..90000>" << std::endl;
  std::cout << "  -skip-pictures <int>" << std::endl;
  std::cout << "  -temporal-subsample <int>" << std::endl;
  std::cout << "  -max-pictures <int>" << std::endl;
  std::cout << "  -sub-gop-length <1..64> (default: " << kDefaultSubGopSize
    << ")" << std::endl;
  std::cout << "  -max-keypic-distance <int> (default: 640)" << std::endl;
  std::cout << "  -closed-gop <int> (default: 0)" << std::endl;
  std::cout << "  -num-ref-pics <0..5> (default: 2)" << std::endl;
  std::cout << "  -checksum-mode <0..1>" << std::endl;
  std::cout << "      0: Reduced checksum verification (default)" << std::endl;
  std::cout << "      1: Maximum checksum robustness" << std::endl;
  std::cout << "  -deblock <0..1> (default: 1)" << std::endl;
  std::cout << "  -beta-offset <-32..31>" << std::endl;
  std::cout << "  -tc-offset <-32..31>" << std::endl;
  std::cout << "  -qp <-64..63> (default: 32)" << std::endl;
  std::cout << "  -multi-passes <0..2>" << std::endl;
  std::cout << "      0: Single-pass (default)" << std::endl;
  std::cout << "      1: Single pass with start picture determination"
    << std::endl;
  std::cout << "      2: Multi-pass" << std::endl;
  std::cout << "  -speed-mode <0..2>" << std::endl;
  std::cout << "      0: Placebo" << std::endl;
  std::cout << "      1: Slow (default)" << std::endl;
  std::cout << "      2: Fast" << std::endl;
  std::cout << "  -tune <0..1>" << std::endl;
  std::cout << "      0: Visual quality (default)" << std::endl;
  std::cout << "      1: PSNR" << std::endl;
  std::cout << "  -verbose <0..1>" << std::endl;
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
    std::cout << "     - not a picture -                          " <<
      "            ";
  }
  std::cout << "  Bytes: " << std::setw(10) << nal_unit.size;
  if (nal_unit.stats.nal_unit_type < 16) {
    double bpp =
      (8 * static_cast<double>(nal_unit.size) / (cli_.width * cli_.height));
    std::stringstream bpp_str;
    bpp_str << std::fixed << std::setprecision(5) << bpp;
    std::cout << "  Bpp: " << std::setw(10) << bpp_str.str();
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

LambdaCurve::LambdaCurve(const std::pair<uint64_t, int> &p0, int qp0,
                         const std::pair<uint64_t, int> &p1, int qp1) {
  const double sse0 = log(p0.first);
  const double sse1 = log(p1.first);
  const double bits0 = log(p0.second);
  const double bits1 = log(p1.second);
  const double lambda0 = sse0 - bits0;
  const double lambda1 = sse1 - bits1;
  dist_scale_ = (lambda1 - lambda0) / (sse1 - sse0);
  dist_offset_ = lambda0 - dist_scale_ * sse0;
  qp_scale_ = (lambda1 - lambda0) / (qp1 - qp0);
  qp_offset_ = lambda0 - qp_scale_ * qp0;
}

LambdaCurve::LambdaCurve(const LambdaCurve &curve,
                         const std::pair<uint64_t, int> &p, int qp)
  : dist_scale_(curve.dist_scale_),
  qp_scale_(curve.qp_scale_) {
  const double sse = log(p.first);
  const double bits = log(p.second);
  const double lambda = sse - bits;
  dist_offset_ = lambda - dist_scale_ * sse;
  qp_offset_ = lambda - qp_scale_ * qp;
}

bool LambdaCurve::IsPointBetter(const std::pair<uint64_t, int> &p) {
  const double sse = log(p.first);
  const double bits = log(p.second);
  const double lambda = sse - bits;
  const double lambda_ref = dist_scale_ * sse + dist_offset_;
  return lambda > lambda_ref;
}

double LambdaCurve::GetQpAtDistortion(uint64_t distortion) {
  const double sse = log(distortion);
  const double lambda = dist_scale_ * sse + dist_offset_;
  return (lambda - qp_offset_) / qp_scale_;
}

}  // namespace xvc_app
