/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_LIB_DECODER_H_
#define XVC_DEC_LIB_DECODER_H_

#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_common_lib/simd_functions.h"
#include "xvc_dec_lib/bit_reader.h"
#include "xvc_dec_lib/picture_decoder.h"
#include "xvc_dec_lib/xvcdec.h"

struct xvc_decoder {};

namespace xvc {

class Decoder : public xvc_decoder {
public:
  enum class State {
    kNoSegmentHeader,
    kSegmentHeaderDecoded,
    kPicDecoded,
    kDecoderVersionTooLow,
    kBitstreamBitdepthTooHigh,
  };

  Decoder();

  bool DecodeNal(const uint8_t *nal_unit, size_t nal_unit_size);
  void DecodeOneBufferedNal(const std::vector<uint8_t> &nal);
  bool GetDecodedPicture(xvc_decoded_picture *dec_pic);
  void FlushBufferedTailPics();
  PicNum GetNumDecodedPics() { return num_pics_in_buffer_; }
  PicNum HasPictureReadyForOutput() {
    return num_pics_in_buffer_ >= sliding_window_length_;
  }
  PicNum GetNumCorruptedPics() { return num_corrupted_pics_; }
  void SetCpuCapabilities(std::set<CpuCapability> capabilities) {
    simd_ = SimdFunctions(capabilities);
  }
  void SetOutputWidth(int width) { output_width_ = width; }
  void SetOutputHeight(int height) { output_height_ = height; }
  void SetOutputChromaFormat(xvc_dec_chroma_format chroma_format) {
    output_chroma_format_ = ChromaFormat(chroma_format);
  }
  void SetOutputColorMatrix(xvc_dec_color_matrix color_matrix) {
    output_color_matrix_ = ColorMatrix(color_matrix);
  }
  void SetOutputBitdepth(int bitdepth) { output_bitdepth_ = bitdepth; }
  void SetDecoderTicks(int ticks) { decoder_ticks_ = ticks; }
  State GetState() { return state_; }
  xvc_dec_chroma_format getChromaFormatApiStyle() {
    return xvc_dec_chroma_format(curr_segment_header_.chroma_format);
  }

private:
  void DecodeAllBufferedNals();
  std::shared_ptr<PictureDecoder> GetNewPictureDecoder(
    ChromaFormat chroma_format, int width, int height, int bitdepth);

  void SetOutputStats(std::shared_ptr<PictureDecoder> pic_dec,
                      xvc_decoded_picture *output_pic);

  PicNum sub_gop_end_poc_ = 0;
  PicNum sub_gop_start_poc_ = 0;
  PicNum doc_ = 0;
  SegmentNum soc_ = static_cast<SegmentNum>(-1);
  SegmentHeader curr_segment_header_;
  SegmentHeader prev_segment_header_;
  PicNum num_pics_in_buffer_ = 0;
  PicNum num_corrupted_pics_ = 0;
  PicNum sliding_window_length_ = 0;
  PicNum pic_buffering_num_ = 0;
  PicNum sub_gop_length_ = 0;
  int num_tail_pics_ = 0;
  int output_width_ = 0;
  int output_height_ = 0;
  ChromaFormat output_chroma_format_ = ChromaFormat::kUndefinedChromaFormat;
  ColorMatrix output_color_matrix_ = ColorMatrix::kUndefinedColorMatrix;
  int output_bitdepth_ = 0;
  int decoder_ticks_ = 0;
  int max_tid_ = 0;
  bool enforce_sliding_window_ = true;
  State state_ = State::kNoSegmentHeader;
  SimdFunctions simd_;
  std::vector<uint8_t> output_pic_bytes_;
  std::vector<std::shared_ptr<PictureDecoder>> pic_decoders_;
  std::deque<std::vector<uint8_t>> nal_buffer_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_DECODER_H_
