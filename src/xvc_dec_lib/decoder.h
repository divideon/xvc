/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
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
  static constexpr size_t kInvalidNal = 0;
  enum class State {
    kNoSegmentHeader,
    kSegmentHeaderDecoded,
    kPicDecoded,
    kDecoderVersionTooLow,
    kBitstreamBitdepthTooHigh,
    kChecksumMismatch,
    kBitstreamVersionTooLow,
  };

  explicit Decoder(int num_threads);
  ~Decoder();
  size_t DecodeNal(const uint8_t *nal_unit, size_t nal_unit_size,
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
  void SetAdditionalDecoderBuffers(int nr_of_buffers) {
    additional_decoder_buffers_ = static_cast<PicNum>(nr_of_buffers);
  }
  void SetOutputWidth(int width) { output_pic_format_.width = width; }
  void SetOutputHeight(int height) { output_pic_format_.height = height; }
  void SetOutputChromaFormat(xvc_dec_chroma_format chroma_format) {
    output_pic_format_.chroma_format = ChromaFormat(chroma_format);
  }
  void SetOutputColorMatrix(xvc_dec_color_matrix color_matrix) {
    output_pic_format_.color_matrix = ColorMatrix(color_matrix);
  }
  void SetOutputBitdepth(int bitdepth) {
    output_pic_format_.bitdepth = bitdepth;
  }
  void SetDecoderTicks(int ticks) { decoder_ticks_ = ticks; }
  State GetState() { return state_; }
  xvc_dec_chroma_format getChromaFormatApiStyle() {
    return xvc_dec_chroma_format(curr_segment_header_->chroma_format);
  }
  void SetDithering(bool dither) { output_pic_format_.dither = dither; }
  static bool ParseNalUnitHeader(BitReader *reader, NalUnitType *nal_unit_type,
                                 bool accept_xvc_bit_zero);

private:
  using NalUnitPtr = std::unique_ptr<std::vector<uint8_t>>;
  using PicDecList = std::vector<std::shared_ptr<const PictureDecoder>>;
  void DecodeAllBufferedNals();
  size_t DecodeSegmentHeaderNal(BitReader *bit_reader);
  size_t DecodePictureNal(const uint8_t *nal_unit, size_t nal_unit_size,
                          int64_t user_data, BitReader *bit_reader);
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
  PicNum additional_decoder_buffers_ = 0;
  PicNum sub_gop_length_ = 0;
  int num_tail_pics_ = 0;
  int decoder_ticks_ = 0;
  int max_tid_ = 0;
  bool enforce_sliding_window_ = true;
  State state_ = State::kNoSegmentHeader;
  SimdFunctions simd_;
  PictureFormat output_pic_format_;
  std::vector<uint8_t> output_pic_bytes_;
  std::vector<std::shared_ptr<PictureDecoder>> pic_decoders_;
  std::list<std::shared_ptr<PictureDecoder>> zero_tid_pic_dec_;
  std::deque<std::pair<NalUnitPtr, int64_t>> nal_buffer_;
  std::unique_ptr<ThreadDecoder> thread_decoder_;
  bool accept_xvc_bit_zero_ = true;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_DECODER_H_
