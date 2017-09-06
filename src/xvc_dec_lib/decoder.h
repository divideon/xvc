/******************************************************************************
* Copyright (C) 2017, Divideon.
*
* Redistribution and use in source and binary form, with or without
* modifications is permitted only under the terms and conditions set forward
* in the xvc License Agreement. For commercial redistribution and use, you are
* required to send a signed copy of the xvc License Agreement to Divideon.
*
* Redistribution and use in source and binary form is permitted free of charge
* for non-commercial purposes. See definition of non-commercial in the xvc
* License Agreement.
*
* All redistribution of source code must retain this copyright notice
* unmodified.
*
* The xvc License Agreement is available at https://xvc.io/license/.
******************************************************************************/

#ifndef XVC_DEC_LIB_DECODER_H_
#define XVC_DEC_LIB_DECODER_H_

#include <deque>
#include <list>
#include <memory>
#include <set>
#include <utility>
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

// To avoid including all thread related system headers
class ThreadDecoder;

class Decoder : public xvc_decoder {
public:
  enum class State {
    kNoSegmentHeader,
    kSegmentHeaderDecoded,
    kPicDecoded,
    kDecoderVersionTooLow,
    kBitstreamBitdepthTooHigh,
    kChecksumMismatch,
  };

  explicit Decoder(int num_threads);
  ~Decoder();
  bool DecodeNal(const uint8_t *nal_unit, size_t nal_unit_size,
                 int64_t user_data = 0);
  bool GetDecodedPicture(xvc_decoded_picture *dec_pic);
  void FlushBufferedNalUnits();
  PicNum GetNumDecodedPics() { return num_pics_in_buffer_; }
  PicNum HasPictureReadyForOutput() {
    return !enforce_sliding_window_ ||
      num_pics_in_buffer_ >= sliding_window_length_;
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
    return xvc_dec_chroma_format(curr_segment_header_->chroma_format);
  }

private:
  using NalUnitPtr = std::unique_ptr<std::vector<uint8_t>>;
  using PicDecList = std::vector<std::shared_ptr<const PictureDecoder>>;
  void DecodeAllBufferedNals();
  bool DecodeSegmentHeaderNal(BitReader *bit_reader);
  void DecodeOneBufferedNal(NalUnitPtr &&nal, int64_t user_data);
  std::shared_ptr<PictureDecoder>
    GetFreePictureDecoder(const SegmentHeader &segment_header);
  void OnPictureDecoded(std::shared_ptr<PictureDecoder> pic_dec, bool success,
                        const PicDecList &inter_deps);
  void SetOutputStats(std::shared_ptr<PictureDecoder> pic_dec,
                      xvc_decoded_picture *output_pic);

  PicNum sub_gop_end_poc_ = 0;
  PicNum sub_gop_start_poc_ = 0;
  PicNum doc_ = 0;
  SegmentNum soc_ = static_cast<SegmentNum>(-1);
  std::shared_ptr<SegmentHeader> curr_segment_header_;
  std::shared_ptr<SegmentHeader> prev_segment_header_;
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
  std::list<std::shared_ptr<PictureDecoder>> zero_tid_pic_dec_;
  std::deque<std::pair<NalUnitPtr, int64_t>> nal_buffer_;
  std::unique_ptr<ThreadDecoder> thread_decoder_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_DECODER_H_
